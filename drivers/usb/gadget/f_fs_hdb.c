
/*
 * f_fs_hdb.c -- user mode file system API for USB composite function controllers
 *
 * Copyright (C) 2010 Samsung Electronics
 * Author: Michal Nazarewicz <mina86@mina86.com>
 *
 * Based on inode.c (GadgetFS) which was:
 * Copyright (C) 2003-2004 David Brownell
 * Copyright (C) 2003 Agilent Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/blkdev.h>
#include <linux/pagemap.h>
#include <linux/export.h>
#include <linux/hid.h>
#include <asm/unaligned.h>

#include <linux/usb/composite.h>
#include <linux/usb/functionfs_hdb.h>
#ifdef CONFIG_HUAWEI_USB_DSM
#include <linux/usb/dsm_usb.h>
#endif


#define FUNCTIONFS_MAGIC_HDB	0xa647362 /* Chosen by a honest dice roll ;) */


/* Debugging ****************************************************************/

#ifdef VERBOSE_DEBUG
#ifndef pr_vdebug
#  define pr_vdebug pr_debug
#endif /* pr_vdebug */
#  define hdb_ffs_dump_mem(prefix, ptr, len) \
	print_hex_dump_bytes(pr_fmt(prefix ": "), DUMP_PREFIX_NONE, ptr, len)
#else
#ifndef pr_vdebug
#  define pr_vdebug(...)                 do { } while (0)
#endif /* pr_vdebug */
#  define hdb_ffs_dump_mem(prefix, ptr, len) do { } while (0)
#endif /* VERBOSE_DEBUG */

#define ENTER()    pr_vdebug("%s()\n", __func__)


/* The data structure and setup file ****************************************/

enum hdb_ffs_state {
	/*
	 * Waiting for descriptors and strings.
	 *
	 * In this state no open(2), read(2) or write(2) on epfiles
	 * may succeed (which should not be the problem as there
	 * should be no such files opened in the first place).
	 */
	HDB_FFS_READ_DESCRIPTORS,
	HDB_FFS_READ_STRINGS,

	/*
	 * We've got descriptors and strings.  We are or have called
	 * functionfs_ready_callback().  functionfs_bind() may have
	 * been called but we don't know.
	 *
	 * This is the only state in which operations on epfiles may
	 * succeed.
	 */
	HDB_FFS_ACTIVE,

	/*
	 * All endpoints have been closed.  This state is also set if
	 * we encounter an unrecoverable error.  The only
	 * unrecoverable error is situation when after reading strings
	 * from user space we fail to initialise epfiles or
	 * functionfs_ready_callback() returns with error (<0).
	 *
	 * In this state no open(2), read(2) or write(2) (both on ep0
	 * as well as epfile) may succeed (at this point epfiles are
	 * unlinked and all closed so this is not a problem; ep0 is
	 * also closed but ep0 file exists and so open(2) on ep0 must
	 * fail).
	 */
	HDB_FFS_CLOSING
};


enum hdb_ffs_setup_state {
	/* There is no setup request pending. */
	HDB_FFS_NO_SETUP,
	/*
	 * User has read events and there was a setup request event
	 * there.  The next read/write on ep0 will handle the
	 * request.
	 */
	HDB_FFS_SETUP_PENDING,
	/*
	 * There was event pending but before user space handled it
	 * some other event was introduced which canceled existing
	 * setup.  If this state is set read/write on ep0 return
	 * -EIDRM.  This state is only set when adding event.
	 */
	HDB_FFS_SETUP_CANCELED
};



struct hdb_ffs_epfile;
struct hdb_ffs_function;

struct hdb_ffs_data {
	struct usb_gadget		*gadget;

	/*
	 * Protect access read/write operations, only one read/write
	 * at a time.  As a consequence protects ep0req and company.
	 * While setup request is being processed (queued) this is
	 * held.
	 */
	struct mutex			mutex;

	/*
	 * Protect access to endpoint related structures (basically
	 * usb_ep_queue(), usb_ep_dequeue(), etc. calls) except for
	 * endpoint zero.
	 */
	spinlock_t			eps_lock;

	/*
	 * XXX REVISIT do we need our own request? Since we are not
	 * handling setup requests immediately user space may be so
	 * slow that another setup will be sent to the gadget but this
	 * time not to us but another function and then there could be
	 * a race.  Is that the case? Or maybe we can use cdev->req
	 * after all, maybe we just need some spinlock for that?
	 */
	struct usb_request		*ep0req;		/* P: mutex */
	struct completion		ep0req_completion;	/* P: mutex */
	int				ep0req_status;		/* P: mutex */
	struct completion		epin_completion;
	struct completion		epout_completion;

	/* reference counter */
	atomic_t			ref;
	/* how many files are opened (EP0 and others) */
	atomic_t			opened;

	/* EP0 state */
	enum hdb_ffs_state			state;

	/*
	 * Possible transitions:
	 * + HDB_FFS_NO_SETUP       -> HDB_FFS_SETUP_PENDING  -- P: ev.waitq.lock
	 *               happens only in ep0 read which is P: mutex
	 * + HDB_FFS_SETUP_PENDING  -> HDB_FFS_NO_SETUP       -- P: ev.waitq.lock
	 *               happens only in ep0 i/o  which is P: mutex
	 * + HDB_FFS_SETUP_PENDING  -> HDB_FFS_SETUP_CANCELED -- P: ev.waitq.lock
	 * + HDB_FFS_SETUP_CANCELED -> HDB_FFS_NO_SETUP       -- cmpxchg
	 */
	enum hdb_ffs_setup_state		setup_state;
/*check 1*/
#define HDB_FFS_SETUP_STATE(hdb_ffs)					\
	((enum hdb_ffs_setup_state)cmpxchg(&(hdb_ffs)->setup_state,	\
				       HDB_FFS_SETUP_CANCELED, HDB_FFS_NO_SETUP))

	/* Events & such. */
	struct {
		u8				types[4];
		unsigned short			count;
		/* XXX REVISIT need to update it in some places, or do we? */
		unsigned short			can_stall;
		struct usb_ctrlrequest		setup;

		wait_queue_head_t		waitq;
	} ev; /* the whole structure, P: ev.waitq.lock */

	/* Flags */
	unsigned long			flags;
#define HDB_FFS_FL_CALL_CLOSED_CALLBACK 0
#define HDB_FFS_FL_BOUND                1

	/* Active function */
	struct hdb_ffs_function		*func;

	/*
	 * Device name, write once when file system is mounted.
	 * Intended for user to read if she wants.
	 */
	const char			*dev_name;
	/* Private data for our user (ie. gadget).  Managed by user. */
	void				*private_data;

	/* filled by __hdb_ffs_data_got_descs() */
	/*
	 * Real descriptors are 16 bytes after raw_descs (so you need
	 * to skip 16 bytes (ie. hdb_ffs->raw_descs + 16) to get to the
	 * first full speed descriptor).  raw_descs_length and
	 * raw_fs_hs_descs_length do not have those 16 bytes added.
	 * ss_desc are 8 bytes (ss_magic + count) pass the hs_descs
	 */
	const void			*raw_descs;
	unsigned			raw_descs_length;
	unsigned			raw_fs_hs_descs_length;
	unsigned			raw_ss_descs_offset;
	unsigned			raw_ss_descs_length;
	unsigned			fs_descs_count;
	unsigned			hs_descs_count;
	unsigned			ss_descs_count;

	unsigned short			strings_count;
	unsigned short			interfaces_count;
	unsigned short			eps_count;
	unsigned short			_pad1;

	int				first_id;
	int				old_strings_count;

	/* filled by __hdb_ffs_data_got_strings() */
	/* ids in stringtabs are set in functionfs_bind() */
	const void			*raw_strings;
	struct usb_gadget_strings	**stringtabs;

	/*
	 * File system's super block, write once when file system is
	 * mounted.
	 */
	struct super_block		*sb;

	/* File permissions, written once when fs is mounted */
	struct hdb_ffs_file_perms {
		umode_t				mode;
		kuid_t				uid;
		kgid_t				gid;
	}				file_perms;

	/*
	 * The endpoint files, filled by hdb_ffs_epfiles_create(),
	 * destroyed by hdb_ffs_epfiles_destroy().
	 */
	struct hdb_ffs_epfile		*epfiles;
};

/* Reference counter handling */
static void hdb_ffs_data_get(struct hdb_ffs_data *hdb_ffs);
static void hdb_ffs_data_put(struct hdb_ffs_data *hdb_ffs);
/* Creates new hdb_ffs_data object. */
static struct hdb_ffs_data *__must_check hdb_ffs_data_new(void) __attribute__((malloc));

/* Opened counter handling. */
static void hdb_ffs_data_opened(struct hdb_ffs_data *hdb_ffs);
static void hdb_ffs_data_closed(struct hdb_ffs_data *hdb_ffs);

/* Called with hdb_ffs->mutex held; take over ownership of data. */
static int __must_check
__hdb_ffs_data_got_descs(struct hdb_ffs_data *hdb_ffs, char *data, size_t len);
static int __must_check
__hdb_ffs_data_got_strings(struct hdb_ffs_data *hdb_ffs, char *data, size_t len);


/* The function structure ***************************************************/

struct hdb_ffs_ep;

struct hdb_ffs_function {
	struct usb_configuration	*conf;
	struct usb_gadget		*gadget;
	struct hdb_ffs_data			*hdb_ffs;

	struct hdb_ffs_ep			*eps;
	u8				eps_revmap[16];
	short				*interfaces_nums;

	struct usb_function		function;
};


static struct hdb_ffs_function *hdb_ffs_func_from_usb(struct usb_function *f)
{
	return container_of(f, struct hdb_ffs_function, function);
}

static void hdb_ffs_func_free(struct hdb_ffs_function *func);

static void hdb_ffs_func_eps_disable(struct hdb_ffs_function *func);
static int __must_check hdb_ffs_func_eps_enable(struct hdb_ffs_function *func);

static int hdb_ffs_func_bind(struct usb_configuration *,
			 struct usb_function *);
static void hdb_ffs_func_unbind(struct usb_configuration *,
			    struct usb_function *);
static int hdb_ffs_func_set_alt(struct usb_function *, unsigned, unsigned);
static void hdb_ffs_func_disable(struct usb_function *);
static int hdb_ffs_func_setup(struct usb_function *,
			  const struct usb_ctrlrequest *);
static void hdb_ffs_func_suspend(struct usb_function *);
static void hdb_ffs_func_resume(struct usb_function *);


static int hdb_ffs_func_revmap_ep(struct hdb_ffs_function *func, u8 num);
static int hdb_ffs_func_revmap_intf(struct hdb_ffs_function *func, u8 intf);


/* The endpoints structures *************************************************/

struct hdb_ffs_ep {
	struct usb_ep			*ep;	/* P: hdb_ffs->eps_lock */
	struct usb_request		*req;	/* P: epfile->mutex */

	/* [0]: full speed, [1]: high speed, [2]: super speed */
	struct usb_endpoint_descriptor	*descs[3];

	u8				num;

	int				status;	/* P: epfile->mutex */
};

struct hdb_ffs_epfile {
	/* Protects ep->ep and ep->req. */
	struct mutex			mutex;
	wait_queue_head_t		wait;
	atomic_t			error;

	struct hdb_ffs_data			*hdb_ffs;
	struct hdb_ffs_ep			*ep;	/* P: hdb_ffs->eps_lock */

	struct dentry			*dentry;

	char				name[5];

	unsigned char			in;	/* P: hdb_ffs->eps_lock */
	unsigned char			isoc;	/* P: hdb_ffs->eps_lock */

	unsigned char			_pad;
};

static int  __must_check hdb_ffs_epfiles_create(struct hdb_ffs_data *hdb_ffs);
static void hdb_ffs_epfiles_destroy(struct hdb_ffs_epfile *epfiles, unsigned count);

static struct inode *__must_check
hdb_ffs_sb_create_file(struct super_block *sb, const char *name, void *data,
		   const struct file_operations *fops,
		   struct dentry **dentry_p);


/* Misc helper functions ****************************************************/

static int hdb_ffs_mutex_lock(struct mutex *mutex, unsigned nonblock)
	__attribute__((warn_unused_result, nonnull));
static char *hdb_ffs_prepare_buffer(const char __user *buf, size_t len)
	__attribute__((warn_unused_result, nonnull));


/* Control file aka ep0 *****************************************************/

static void hdb_ffs_ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct hdb_ffs_data *hdb_ffs = req->context;

	complete_all(&hdb_ffs->ep0req_completion);
}

static int __hdb_ffs_ep0_queue_wait(struct hdb_ffs_data *hdb_ffs, char *data, size_t len)
{
	struct usb_request *req = hdb_ffs->ep0req;
	int ret;

	req->zero     = len < le16_to_cpu(hdb_ffs->ev.setup.wLength);

	spin_unlock_irq(&hdb_ffs->ev.waitq.lock);

	req->buf      = data;
	req->length   = len;

	/*
	 * UDC layer requires to provide a buffer even for ZLP, but should
	 * not use it at all. Let's provide some poisoned pointer to catch
	 * possible bug in the driver.
	 */
	if (req->buf == NULL)
		req->buf = (void *)0xDEADBABE;

	INIT_COMPLETION(hdb_ffs->ep0req_completion);

	ret = usb_ep_queue(hdb_ffs->gadget->ep0, req, GFP_ATOMIC);
	if (unlikely(ret < 0))
		return ret;

	ret = wait_for_completion_interruptible(&hdb_ffs->ep0req_completion);
	if (unlikely(ret)) {
		usb_ep_dequeue(hdb_ffs->gadget->ep0, req);
		return -EINTR;
	}

	hdb_ffs->setup_state = HDB_FFS_NO_SETUP;
	return hdb_ffs->ep0req_status;
}

static int __hdb_ffs_ep0_stall(struct hdb_ffs_data *hdb_ffs)
{
	if (hdb_ffs->ev.can_stall) {
		pr_vdebug("ep0 stall\n");
		usb_ep_set_halt(hdb_ffs->gadget->ep0);
		hdb_ffs->setup_state = HDB_FFS_NO_SETUP;
		return -EL2HLT;
	} else {
		pr_debug("bogus ep0 stall!\n");
		return -ESRCH;
	}
}

static ssize_t hdb_ffs_ep0_write(struct file *file, const char __user *buf,
			     size_t len, loff_t *ptr)
{
	struct hdb_ffs_data *hdb_ffs = file->private_data;
	ssize_t ret;
	char *data;

	ENTER();

	/* Fast check if setup was canceled */
	if (HDB_FFS_SETUP_STATE(hdb_ffs) == HDB_FFS_SETUP_CANCELED)
		return -EIDRM;

	/* Acquire mutex */
	ret = hdb_ffs_mutex_lock(&hdb_ffs->mutex, file->f_flags & O_NONBLOCK);
	if (unlikely(ret < 0))
		return ret;

	/* Check state */
	switch (hdb_ffs->state) {
	case HDB_FFS_READ_DESCRIPTORS:
	case HDB_FFS_READ_STRINGS:
		/* Copy data */
		if (unlikely(len < 16)) {
			ret = -EINVAL;
			break;
		}

		data = hdb_ffs_prepare_buffer(buf, len);
		if (IS_ERR(data)) {
			ret = PTR_ERR(data);
			break;
		}

		/* Handle data */
		if (hdb_ffs->state == HDB_FFS_READ_DESCRIPTORS) {
			ret = __hdb_ffs_data_got_descs(hdb_ffs, data, len);
			if (unlikely(ret < 0))
				break;

			hdb_ffs->state = HDB_FFS_READ_STRINGS;
			ret = len;
		} else {
			pr_info("read strings\n");
			ret = __hdb_ffs_data_got_strings(hdb_ffs, data, len);
			if (unlikely(ret < 0))
				break;

			ret = hdb_ffs_epfiles_create(hdb_ffs);
			if (unlikely(ret)) {
				hdb_ffs->state = HDB_FFS_CLOSING;
				break;
			}

			hdb_ffs->state = HDB_FFS_ACTIVE;
			mutex_unlock(&hdb_ffs->mutex);

			ret = functionfs_hdb_ready_callback(hdb_ffs);
			if (unlikely(ret < 0)) {
				hdb_ffs->state = HDB_FFS_CLOSING;
				return ret;
			}

			set_bit(HDB_FFS_FL_CALL_CLOSED_CALLBACK, &hdb_ffs->flags);
			return len;
		}
		break;

	case HDB_FFS_ACTIVE:
		data = NULL;
		/*
		 * We're called from user space, we can use _irq
		 * rather then _irqsave
		 */
		spin_lock_irq(&hdb_ffs->ev.waitq.lock);
		switch (HDB_FFS_SETUP_STATE(hdb_ffs)) {
		case HDB_FFS_SETUP_CANCELED:
			ret = -EIDRM;
			goto done_spin;

		case HDB_FFS_NO_SETUP:
			ret = -ESRCH;
			goto done_spin;

		case HDB_FFS_SETUP_PENDING:
			break;
		}

		/* HDB_FFS_SETUP_PENDING */
		if (!(hdb_ffs->ev.setup.bRequestType & USB_DIR_IN)) {
			spin_unlock_irq(&hdb_ffs->ev.waitq.lock);
			ret = __hdb_ffs_ep0_stall(hdb_ffs);
			break;
		}

		/* HDB_FFS_SETUP_PENDING and not stall */
		len = min(len, (size_t)le16_to_cpu(hdb_ffs->ev.setup.wLength));

		spin_unlock_irq(&hdb_ffs->ev.waitq.lock);

		data = hdb_ffs_prepare_buffer(buf, len);
		if (IS_ERR(data)) {
			ret = PTR_ERR(data);
			break;
		}

		spin_lock_irq(&hdb_ffs->ev.waitq.lock);

		/*
		 * We are guaranteed to be still in HDB_FFS_ACTIVE state
		 * but the state of setup could have changed from
		 * HDB_FFS_SETUP_PENDING to HDB_FFS_SETUP_CANCELED so we need
		 * to check for that.  If that happened we copied data
		 * from user space in vain but it's unlikely.
		 *
		 * For sure we are not in HDB_FFS_NO_SETUP since this is
		 * the only place HDB_FFS_SETUP_PENDING -> HDB_FFS_NO_SETUP
		 * transition can be performed and it's protected by
		 * mutex.
		 */
		if (HDB_FFS_SETUP_STATE(hdb_ffs) == HDB_FFS_SETUP_CANCELED) {
			ret = -EIDRM;
done_spin:
			spin_unlock_irq(&hdb_ffs->ev.waitq.lock);
		} else {
			/* unlocks spinlock */
			ret = __hdb_ffs_ep0_queue_wait(hdb_ffs, data, len);
		}
		kfree(data);
		break;

	default:
		ret = -EBADFD;
		break;
	}

	mutex_unlock(&hdb_ffs->mutex);
	return ret;
}

static ssize_t __hdb_ffs_ep0_read_events(struct hdb_ffs_data *hdb_ffs, char __user *buf,
				     size_t n)
{
	/*
	 * We are holding hdb_ffs->ev.waitq.lock and hdb_ffs->mutex and we need
	 * to release them.
	 */
	struct usb_functionfs_event events[n];
	unsigned i = 0;

	memset(events, 0, sizeof events);

	do {
		events[i].type = hdb_ffs->ev.types[i];
		if (events[i].type == FUNCTIONFS_SETUP) {
			events[i].u.setup = hdb_ffs->ev.setup;
			hdb_ffs->setup_state = HDB_FFS_SETUP_PENDING;
		}
	} while (++i < n);

	if (n < hdb_ffs->ev.count) {
		hdb_ffs->ev.count -= n;
		memmove(hdb_ffs->ev.types, hdb_ffs->ev.types + n,
			hdb_ffs->ev.count * sizeof *hdb_ffs->ev.types);
	} else {
		hdb_ffs->ev.count = 0;
	}

	spin_unlock_irq(&hdb_ffs->ev.waitq.lock);
	mutex_unlock(&hdb_ffs->mutex);

	return unlikely(__copy_to_user(buf, events, sizeof events))
		? -EFAULT : sizeof events;
}

static ssize_t hdb_ffs_ep0_read(struct file *file, char __user *buf,
			    size_t len, loff_t *ptr)
{
	struct hdb_ffs_data *hdb_ffs = file->private_data;
	char *data = NULL;
	size_t n;
	int ret;

	ENTER();

	/* Fast check if setup was canceled */
	if (HDB_FFS_SETUP_STATE(hdb_ffs) == HDB_FFS_SETUP_CANCELED)
		return -EIDRM;

	/* Acquire mutex */
	ret = hdb_ffs_mutex_lock(&hdb_ffs->mutex, file->f_flags & O_NONBLOCK);
	if (unlikely(ret < 0))
		return ret;

	/* Check state */
	if (hdb_ffs->state != HDB_FFS_ACTIVE) {
		ret = -EBADFD;
		goto done_mutex;
	}

	/*
	 * We're called from user space, we can use _irq rather then
	 * _irqsave
	 */
	spin_lock_irq(&hdb_ffs->ev.waitq.lock);

	switch (HDB_FFS_SETUP_STATE(hdb_ffs)) {
	case HDB_FFS_SETUP_CANCELED:
		ret = -EIDRM;
		break;

	case HDB_FFS_NO_SETUP:
		n = len / sizeof(struct usb_functionfs_event);
		if (unlikely(!n)) {
			ret = -EINVAL;
			break;
		}

		if ((file->f_flags & O_NONBLOCK) && !hdb_ffs->ev.count) {
			ret = -EAGAIN;
			break;
		}

		if (wait_event_interruptible_exclusive_locked_irq(hdb_ffs->ev.waitq,
							hdb_ffs->ev.count)) {
			ret = -EINTR;
			break;
		}

		return __hdb_ffs_ep0_read_events(hdb_ffs, buf,
					     min(n, (size_t)hdb_ffs->ev.count));

	case HDB_FFS_SETUP_PENDING:
		if (hdb_ffs->ev.setup.bRequestType & USB_DIR_IN) {
			spin_unlock_irq(&hdb_ffs->ev.waitq.lock);
			ret = __hdb_ffs_ep0_stall(hdb_ffs);
			goto done_mutex;
		}

		len = min(len, (size_t)le16_to_cpu(hdb_ffs->ev.setup.wLength));

		spin_unlock_irq(&hdb_ffs->ev.waitq.lock);

		if (likely(len)) {
			data = kmalloc(len, GFP_KERNEL);
			if (unlikely(!data)) {
				ret = -ENOMEM;
				goto done_mutex;
			}
		}

		spin_lock_irq(&hdb_ffs->ev.waitq.lock);

		/* See hdb_ffs_ep0_write() */
		if (HDB_FFS_SETUP_STATE(hdb_ffs) == HDB_FFS_SETUP_CANCELED) {
			ret = -EIDRM;
			break;
		}

		/* unlocks spinlock */
		ret = __hdb_ffs_ep0_queue_wait(hdb_ffs, data, len);
		if (likely(ret > 0) && unlikely(__copy_to_user(buf, data, len)))
			ret = -EFAULT;
		goto done_mutex;

	default:
		ret = -EBADFD;
		break;
	}

	spin_unlock_irq(&hdb_ffs->ev.waitq.lock);
done_mutex:
	mutex_unlock(&hdb_ffs->mutex);
	kfree(data);
	return ret;
}

static int hdb_ffs_ep0_open(struct inode *inode, struct file *file)
{
	struct hdb_ffs_data *hdb_ffs = inode->i_private;

	ENTER();

	if (unlikely(hdb_ffs->state == HDB_FFS_CLOSING))
		return -EBUSY;

	if (atomic_read(&hdb_ffs->opened))
		return -EBUSY;

	file->private_data = hdb_ffs;
	hdb_ffs_data_opened(hdb_ffs);

	return 0;
}

static int hdb_ffs_ep0_release(struct inode *inode, struct file *file)
{
	struct hdb_ffs_data *hdb_ffs = file->private_data;

	ENTER();

	hdb_ffs_data_closed(hdb_ffs);

	return 0;
}

static long hdb_ffs_ep0_ioctl(struct file *file, unsigned code, unsigned long value)
{
	struct hdb_ffs_data *hdb_ffs = file->private_data;
	struct usb_gadget *gadget = hdb_ffs->gadget;
	long ret;

	ENTER();

	if (code == FUNCTIONFS_INTERFACE_REVMAP) {
		struct hdb_ffs_function *func = hdb_ffs->func;
		ret = func ? hdb_ffs_func_revmap_intf(func, value) : -ENODEV;
	} else if (gadget && gadget->ops->ioctl) {
		ret = gadget->ops->ioctl(gadget, code, value);
	} else {
		ret = -ENOTTY;
	}

	return ret;
}

static const struct file_operations hdb_ffs_ep0_operations = {
	.llseek =	no_llseek,

	.open =		hdb_ffs_ep0_open,
	.write =	hdb_ffs_ep0_write,
	.read =		hdb_ffs_ep0_read,
	.release =	hdb_ffs_ep0_release,
	.unlocked_ioctl =	hdb_ffs_ep0_ioctl,
};


/* "Normal" endpoints operations ********************************************/

static void hdb_ffs_epfile_io_complete(struct usb_ep *_ep, struct usb_request *req)
{
	struct hdb_ffs_ep *ep = _ep->driver_data;
	ENTER();

	/* req may be freed during unbind */
	if (ep && ep->req && likely(req->context)) {
		struct hdb_ffs_ep *ep = _ep->driver_data;
		ep->status = req->status ? req->status : req->actual;
		complete(req->context);
	}
}

#define MAX_BUF_LEN_HDB	65536
#define MAX_UDC_LEN_WR	16384

static ssize_t hdb_ffs_epfile_io(struct file *file,
			     char __user *buf, size_t len, int read)
{
	struct hdb_ffs_epfile *epfile = file->private_data;
	struct hdb_ffs_ep *ep;
	struct hdb_ffs_data *hdb_ffs = epfile->hdb_ffs;
	char *data = NULL;
	ssize_t ret;
	int halt;
	int buffer_len = 0;
	pr_debug("%s: len %zu, read %d\n", __func__, len, read);

	if(!read && len > MAX_UDC_LEN_WR)
	{
		len = MAX_UDC_LEN_WR;
	}

	if (atomic_read(&epfile->error))
		return -ENODEV;

	goto first_try;
	do {
		spin_unlock_irq(&epfile->hdb_ffs->eps_lock);
		mutex_unlock(&epfile->mutex);

first_try:
		/* Are we still active? */
		if (WARN_ON(epfile->hdb_ffs->state != HDB_FFS_ACTIVE)) {
			ret = -ENODEV;
			goto error;
		}

		/* Wait for endpoint to be enabled */
		ep = epfile->ep;
		if (!ep) {
			if (file->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				goto error;
			}

			/* Don't wait on write if device is offline */
			if (!read) {
				ret = -ENODEV;
				goto error;
			}

			/*
			 * if ep is disabled, this fails all current IOs
			 * and wait for next epfile open to happen
			 */
			if (!atomic_read(&epfile->error)) {
				ret = wait_event_interruptible(epfile->wait,
					(ep = epfile->ep));
				if (ret < 0)
					goto error;
			}
			if (!ep) {
				ret = -ENODEV;
				goto error;
			}
		}

		buffer_len = !read ? len : round_up(len,
						ep->ep->desc->wMaxPacketSize);

		/* Do we halt? */
		halt = !read == !epfile->in;
		if (halt && epfile->isoc) {
			ret = -EINVAL;
			goto error;
		}

		/* Allocate & copy */
		if (!halt && !data) {
			data = kzalloc(buffer_len, GFP_KERNEL);
			if (unlikely(!data))
				return -ENOMEM;

			if (!read &&
			    unlikely(__copy_from_user(data, buf, len))) {
				ret = -EFAULT;
				goto error;
			}
		}

		/* We will be using request */
		ret = hdb_ffs_mutex_lock(&epfile->mutex,
				     file->f_flags & O_NONBLOCK);
		if (unlikely(ret))
			goto error;

		/*
		 * We're called from user space, we can use _irq rather then
		 * _irqsave
		 */
		spin_lock_irq(&epfile->hdb_ffs->eps_lock);

		/*
		 * While we were acquiring mutex endpoint got disabled
		 * or changed?
		 */
	} while (unlikely(epfile->ep != ep));

	/* Halt */
	if (unlikely(halt)) {
		if (likely(epfile->ep == ep) && !WARN_ON(!ep->ep))
			usb_ep_set_halt(ep->ep);
		spin_unlock_irq(&epfile->hdb_ffs->eps_lock);
		ret = -EBADMSG;
	} else {
		/* Fire the request */
		struct completion *done;

		struct usb_request *req = ep->req;
		req->complete = hdb_ffs_epfile_io_complete;
		req->buf      = data;
		req->length   = buffer_len;

		if (read) {
			INIT_COMPLETION(hdb_ffs->epout_completion);
			req->context  = done = &hdb_ffs->epout_completion;
		} else {
			INIT_COMPLETION(hdb_ffs->epin_completion);
			req->context  = done = &hdb_ffs->epin_completion;
		}
		ret = usb_ep_queue(ep->ep, req, GFP_ATOMIC);

		spin_unlock_irq(&epfile->hdb_ffs->eps_lock);

		if (unlikely(ret < 0)) {
			ret = -EIO;
		} else if (unlikely(wait_for_completion_interruptible(done))) {
			spin_lock_irq(&epfile->hdb_ffs->eps_lock);
			/*
			 * While we were acquiring lock endpoint got disabled
			 * (disconnect) or changed (composition switch) ?
			 */
			if (epfile->ep == ep)
				usb_ep_dequeue(ep->ep, req);
			spin_unlock_irq(&epfile->hdb_ffs->eps_lock);
			ret = -EINTR;
		} else {
			spin_lock_irq(&epfile->hdb_ffs->eps_lock);
			/*
			 * While we were acquiring lock endpoint got disabled
			 * (disconnect) or changed (composition switch) ?
			 */
			if (epfile->ep == ep)
				ret = ep->status;
			else
				ret = -ENODEV;
			spin_unlock_irq(&epfile->hdb_ffs->eps_lock);
			if (read && ret > 0) {
				if (len != MAX_BUF_LEN_HDB && ret < len)
					pr_err("less data(%zd) recieved than intended length(%zu)\n",
								ret, len);
				if (ret > len) {
					ret = -EOVERFLOW;
					pr_err("More data(%zd) recieved than intended length(%zu)\n",
								ret, len);
				} else if (unlikely(copy_to_user(
							buf, data, ret))) {
					pr_err("Fail to copy to user len:%zd\n",
									ret);
					ret = -EFAULT;
				}
#ifdef CONFIG_HUAWEI_USB_DSM
				if(ret < 0)
				{
					DSM_USB_LOG(DSM_USB_DEVICE, NULL, DSM_USB_DEVICE_ADB_OFFLINE_ERR,
						"%s: adb offline : error number %zd\n",
						__FUNCTION__, ret);
				}
#endif
			}
		}
	}

	mutex_unlock(&epfile->mutex);
error:
	kfree(data);
	if (ret < 0)
		pr_err("Error: returning %zd value\n", ret);
	return ret;
}

static ssize_t
hdb_ffs_epfile_write(struct file *file, const char __user *buf, size_t len,
		 loff_t *ptr)
{
	ENTER();

	return hdb_ffs_epfile_io(file, (char __user *)buf, len, 0);
}

static ssize_t
hdb_ffs_epfile_read(struct file *file, char __user *buf, size_t len, loff_t *ptr)
{
	ENTER();

	return hdb_ffs_epfile_io(file, buf, len, 1);
}

static int
hdb_ffs_epfile_open(struct inode *inode, struct file *file)
{
	struct hdb_ffs_epfile *epfile = inode->i_private;

	ENTER();

	if (WARN_ON(epfile->hdb_ffs->state != HDB_FFS_ACTIVE))
		return -ENODEV;

	file->private_data = epfile;
	hdb_ffs_data_opened(epfile->hdb_ffs);
	atomic_set(&epfile->error, 0);

	return 0;
}

static int
hdb_ffs_epfile_release(struct inode *inode, struct file *file)
{
	struct hdb_ffs_epfile *epfile = inode->i_private;

	ENTER();

	atomic_set(&epfile->error, 1);
	hdb_ffs_data_closed(epfile->hdb_ffs);
	file->private_data = NULL;

	return 0;
}

static long hdb_ffs_epfile_ioctl(struct file *file, unsigned code,
			     unsigned long value)
{
	struct hdb_ffs_epfile *epfile = file->private_data;
	int ret;

	ENTER();

	if (WARN_ON(epfile->hdb_ffs->state != HDB_FFS_ACTIVE))
		return -ENODEV;

	spin_lock_irq(&epfile->hdb_ffs->eps_lock);
	if (likely(epfile->ep)) {
		switch (code) {
		case FUNCTIONFS_FIFO_STATUS:
			ret = usb_ep_fifo_status(epfile->ep->ep);
			break;
		case FUNCTIONFS_FIFO_FLUSH:
			usb_ep_fifo_flush(epfile->ep->ep);
			ret = 0;
			break;
		case FUNCTIONFS_CLEAR_HALT:
			ret = usb_ep_clear_halt(epfile->ep->ep);
			break;
		case FUNCTIONFS_ENDPOINT_REVMAP:
			ret = epfile->ep->num;
			break;
		default:
			ret = -ENOTTY;
		}
	} else {
		ret = -ENODEV;
	}
	spin_unlock_irq(&epfile->hdb_ffs->eps_lock);

	return ret;
}

static const struct file_operations hdb_ffs_epfile_operations = {
	.llseek =	no_llseek,

	.open =		hdb_ffs_epfile_open,
	.write =	hdb_ffs_epfile_write,
	.read =		hdb_ffs_epfile_read,
	.release =	hdb_ffs_epfile_release,
	.unlocked_ioctl =	hdb_ffs_epfile_ioctl,
};


/* File system and super block operations ***********************************/

/*
 * Mounting the file system creates a controller file, used first for
 * function configuration then later for event monitoring.
 */

static struct inode *__must_check
hdb_ffs_sb_make_inode(struct super_block *sb, void *data,
		  const struct file_operations *fops,
		  const struct inode_operations *iops,
		  struct hdb_ffs_file_perms *perms)
{
	struct inode *inode;

	ENTER();

	inode = new_inode(sb);

	if (likely(inode)) {
		struct timespec current_time = CURRENT_TIME;

		inode->i_ino	 = get_next_ino();
		inode->i_mode    = perms->mode;
		inode->i_uid     = perms->uid;
		inode->i_gid     = perms->gid;
		inode->i_atime   = current_time;
		inode->i_mtime   = current_time;
		inode->i_ctime   = current_time;
		inode->i_private = data;
		if (fops)
			inode->i_fop = fops;
		if (iops)
			inode->i_op  = iops;
	}

	return inode;
}

/* Create "regular" file */
static struct inode *hdb_ffs_sb_create_file(struct super_block *sb,
					const char *name, void *data,
					const struct file_operations *fops,
					struct dentry **dentry_p)
{
	struct hdb_ffs_data	*hdb_ffs = sb->s_fs_info;
	struct dentry	*dentry;
	struct inode	*inode;

	ENTER();

	dentry = d_alloc_name(sb->s_root, name);
	if (unlikely(!dentry))
		return NULL;

	inode = hdb_ffs_sb_make_inode(sb, data, fops, NULL, &hdb_ffs->file_perms);
	if (unlikely(!inode)) {
		dput(dentry);
		return NULL;
	}

	d_add(dentry, inode);
	if (dentry_p)
		*dentry_p = dentry;

	return inode;
}

/* Super block */
static const struct super_operations hdb_ffs_sb_operations = {
	.statfs =	simple_statfs,
	.drop_inode =	generic_delete_inode,
};

struct hdb_ffs_sb_fill_data {
	struct hdb_ffs_file_perms perms;
	umode_t root_mode;
	const char *dev_name;
	struct hdb_ffs_data *hdb_ffs_data;
};

static int hdb_ffs_sb_fill(struct super_block *sb, void *_data, int silent)
{
	struct hdb_ffs_sb_fill_data *data = _data;
	struct inode	*inode;
	struct hdb_ffs_data	*hdb_ffs = data->hdb_ffs_data;

	ENTER();

	hdb_ffs->sb              = sb;
	data->hdb_ffs_data       = NULL;
	sb->s_fs_info        = hdb_ffs;
	sb->s_blocksize      = PAGE_CACHE_SIZE;
	sb->s_blocksize_bits = PAGE_CACHE_SHIFT;
	sb->s_magic          = FUNCTIONFS_MAGIC_HDB;
	sb->s_op             = &hdb_ffs_sb_operations;
	sb->s_time_gran      = 1;

	/* Root inode */
	data->perms.mode = data->root_mode;
	inode = hdb_ffs_sb_make_inode(sb, NULL,
				  &simple_dir_operations,
				  &simple_dir_inode_operations,
				  &data->perms);
	sb->s_root = d_make_root(inode);
	if (unlikely(!sb->s_root))
		return -ENOMEM;

	/* EP0 file */
	if (unlikely(!hdb_ffs_sb_create_file(sb, "ep0", hdb_ffs,
					 &hdb_ffs_ep0_operations, NULL)))
		return -ENOMEM;

	return 0;
}

static int hdb_ffs_fs_parse_opts(struct hdb_ffs_sb_fill_data *data, char *opts)
{
	ENTER();

	if (!opts || !*opts)
		return 0;

	for (;;) {
		unsigned long value;
		char *eq, *comma;

		/* Option limit */
		comma = strchr(opts, ',');
		if (comma)
			*comma = 0;

		/* Value limit */
		eq = strchr(opts, '=');
		if (unlikely(!eq)) {
			pr_err("'=' missing in %s\n", opts);
			return -EINVAL;
		}
		*eq = 0;

		/* Parse value */
		if (kstrtoul(eq + 1, 0, &value)) {
			pr_err("%s: invalid value: %s\n", opts, eq + 1);
			return -EINVAL;
		}

		/* Interpret option */
		switch (eq - opts) {
		case 5:
			if (!memcmp(opts, "rmode", 5))
				data->root_mode  = (value & 0555) | S_IFDIR;
			else if (!memcmp(opts, "fmode", 5))
				data->perms.mode = (value & 0666) | S_IFREG;
			else
				goto invalid;
			break;

		case 4:
			if (!memcmp(opts, "mode", 4)) {
				data->root_mode  = (value & 0555) | S_IFDIR;
				data->perms.mode = (value & 0666) | S_IFREG;
			} else {
				goto invalid;
			}
			break;

		case 3:
			if (!memcmp(opts, "uid", 3)) {
				data->perms.uid = make_kuid(current_user_ns(), value);
				if (!uid_valid(data->perms.uid)) {
					pr_err("%s: unmapped value: %lu\n", opts, value);
					return -EINVAL;
				}
			} else if (!memcmp(opts, "gid", 3)) {
				data->perms.gid = make_kgid(current_user_ns(), value);
				if (!gid_valid(data->perms.gid)) {
					pr_err("%s: unmapped value: %lu\n", opts, value);
					return -EINVAL;
				}
			} else {
				goto invalid;
			}
			break;

		default:
invalid:
			pr_err("%s: invalid option\n", opts);
			return -EINVAL;
		}

		/* Next iteration */
		if (!comma)
			break;
		opts = comma + 1;
	}

	return 0;
}

/* "mount -t functionfs dev_name /dev/function" ends up here */

static struct dentry *
hdb_ffs_fs_mount(struct file_system_type *t, int flags,
	      const char *dev_name, void *opts)
{
	struct hdb_ffs_sb_fill_data data = {
		.perms = {
			.mode = S_IFREG | 0600,
			.uid = GLOBAL_ROOT_UID,
			.gid = GLOBAL_ROOT_GID,
		},
		.root_mode = S_IFDIR | 0500,
	};
	struct dentry *rv;
	int ret;
	void *hdb_ffs_dev;
	struct hdb_ffs_data	*hdb_ffs;

	ENTER();

	ret = hdb_ffs_fs_parse_opts(&data, opts);
	if (unlikely(ret < 0))
		return ERR_PTR(ret);

	hdb_ffs = hdb_ffs_data_new();
	if (unlikely(!hdb_ffs))
		return ERR_PTR(-ENOMEM);
	hdb_ffs->file_perms = data.perms;

	hdb_ffs->dev_name = kstrdup(dev_name, GFP_KERNEL);
	if (unlikely(!hdb_ffs->dev_name)) {
		hdb_ffs_data_put(hdb_ffs);
		return ERR_PTR(-ENOMEM);
	}

	hdb_ffs_dev = functionfs_hdb_acquire_dev_callback(dev_name);
	if (IS_ERR(hdb_ffs_dev)) {
		hdb_ffs_data_put(hdb_ffs);
		return ERR_CAST(hdb_ffs_dev);
	}
	hdb_ffs->private_data = hdb_ffs_dev;
	data.hdb_ffs_data = hdb_ffs;

	rv = mount_nodev(t, flags, &data, hdb_ffs_sb_fill);
	if (IS_ERR(rv) && data.hdb_ffs_data) {
		functionfs_hdb_release_dev_callback(data.hdb_ffs_data);
		hdb_ffs_data_put(data.hdb_ffs_data);
	}
	return rv;
}

static void
hdb_ffs_fs_kill_sb(struct super_block *sb)
{
	ENTER();

	kill_litter_super(sb);
	if (sb->s_fs_info) {
		functionfs_release_dev_callback(sb->s_fs_info);
		hdb_ffs_data_put(sb->s_fs_info);
	}
}

static struct file_system_type hdb_ffs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= "functionfs_hdb",
	.mount		= hdb_ffs_fs_mount,
	.kill_sb	= hdb_ffs_fs_kill_sb,
};
MODULE_ALIAS_FS("functionfs_hdb");


/* Driver's main init/cleanup functions *************************************/

static int functionfs_hdb_init(void)
{
	int ret;

	ENTER();

	ret = register_filesystem(&hdb_ffs_fs_type);
	if (likely(!ret))
		pr_info("file system registered\n");
	else
		pr_err("failed registering file system (%d)\n", ret);

	return ret;
}

static void functionfs_hdb_cleanup(void)
{
	ENTER();

	pr_info("unloading\n");
	unregister_filesystem(&hdb_ffs_fs_type);
}


/* hdb_ffs_data and hdb_ffs_function construction and destruction code **************/

static void hdb_ffs_data_clear(struct hdb_ffs_data *hdb_ffs);
static void hdb_ffs_data_reset(struct hdb_ffs_data *hdb_ffs);

static void hdb_ffs_data_get(struct hdb_ffs_data *hdb_ffs)
{
	ENTER();

	atomic_inc(&hdb_ffs->ref);
}

static void hdb_ffs_data_opened(struct hdb_ffs_data *hdb_ffs)
{
	ENTER();

	atomic_inc(&hdb_ffs->ref);
	atomic_inc(&hdb_ffs->opened);
}

static void hdb_ffs_data_put(struct hdb_ffs_data *hdb_ffs)
{
	ENTER();

	if (unlikely(atomic_dec_and_test(&hdb_ffs->ref))) {
		pr_info("%s(): freeing\n", __func__);
		hdb_ffs_data_clear(hdb_ffs);
		BUG_ON(waitqueue_active(&hdb_ffs->ev.waitq) ||
		       waitqueue_active(&hdb_ffs->ep0req_completion.wait));
		kfree(hdb_ffs->dev_name);
		kfree(hdb_ffs);
	}
}

static void hdb_ffs_data_closed(struct hdb_ffs_data *hdb_ffs)
{
	ENTER();

	if (atomic_dec_and_test(&hdb_ffs->opened)) {
		hdb_ffs->state = HDB_FFS_CLOSING;
		hdb_ffs_data_reset(hdb_ffs);
	}

	hdb_ffs_data_put(hdb_ffs);
}

static struct hdb_ffs_data *hdb_ffs_data_new(void)
{
	struct hdb_ffs_data *hdb_ffs = kzalloc(sizeof *hdb_ffs, GFP_KERNEL);
	if (unlikely(!hdb_ffs))
		return 0;

	ENTER();

	atomic_set(&hdb_ffs->ref, 1);
	atomic_set(&hdb_ffs->opened, 0);
	hdb_ffs->state = HDB_FFS_READ_DESCRIPTORS;
	mutex_init(&hdb_ffs->mutex);
	spin_lock_init(&hdb_ffs->eps_lock);
	init_waitqueue_head(&hdb_ffs->ev.waitq);
	init_completion(&hdb_ffs->ep0req_completion);
	init_completion(&hdb_ffs->epout_completion);
	init_completion(&hdb_ffs->epin_completion);

	/* XXX REVISIT need to update it in some places, or do we? */
	hdb_ffs->ev.can_stall = 1;

	return hdb_ffs;
}

static void hdb_ffs_data_clear(struct hdb_ffs_data *hdb_ffs)
{
	ENTER();

	pr_debug("%s: hdb_ffs->gadget= %p, hdb_ffs->flags= %lu\n", __func__,
						hdb_ffs->gadget, hdb_ffs->flags);
	if (test_and_clear_bit(HDB_FFS_FL_CALL_CLOSED_CALLBACK, &hdb_ffs->flags))
		functionfs_hdb_closed_callback(hdb_ffs);

	/* Dump hdb_ffs->gadget and hdb_ffs->flags */
	if (hdb_ffs->gadget)
		pr_err("%s: hdb_ffs->gadget= %p, hdb_ffs->flags= %lu\n", __func__,
						hdb_ffs->gadget, hdb_ffs->flags);
	BUG_ON(hdb_ffs->gadget);

	if (hdb_ffs->epfiles)
		hdb_ffs_epfiles_destroy(hdb_ffs->epfiles, hdb_ffs->eps_count);

	kfree(hdb_ffs->raw_descs);
	kfree(hdb_ffs->raw_strings);
	kfree(hdb_ffs->stringtabs);
}

static void hdb_ffs_data_reset(struct hdb_ffs_data *hdb_ffs)
{
	ENTER();

	hdb_ffs_data_clear(hdb_ffs);

	hdb_ffs->epfiles = NULL;
	hdb_ffs->raw_descs = NULL;
	hdb_ffs->raw_strings = NULL;
	hdb_ffs->stringtabs = NULL;

	hdb_ffs->raw_descs_length = 0;
	hdb_ffs->raw_fs_hs_descs_length = 0;
	hdb_ffs->raw_ss_descs_offset = 0;
	hdb_ffs->raw_ss_descs_length = 0;
	hdb_ffs->fs_descs_count = 0;
	hdb_ffs->hs_descs_count = 0;
	hdb_ffs->ss_descs_count = 0;

	hdb_ffs->strings_count = 0;
	hdb_ffs->interfaces_count = 0;
	hdb_ffs->eps_count = 0;

	hdb_ffs->ev.count = 0;

	hdb_ffs->state = HDB_FFS_READ_DESCRIPTORS;
	hdb_ffs->setup_state = HDB_FFS_NO_SETUP;
	hdb_ffs->flags = 0;
}


static int functionfs_hdb_bind(struct hdb_ffs_data *hdb_ffs, struct usb_composite_dev *cdev)
{
	struct usb_gadget_strings **lang;

	ENTER();

	if (WARN_ON(hdb_ffs->state != HDB_FFS_ACTIVE
		 || test_and_set_bit(HDB_FFS_FL_BOUND, &hdb_ffs->flags)))
		return -EBADFD;

	if (!hdb_ffs->first_id || hdb_ffs->old_strings_count < hdb_ffs->strings_count) {
		int first_id = usb_string_ids_n(cdev, hdb_ffs->strings_count);
		if (unlikely(first_id < 0))
			return first_id;
		hdb_ffs->first_id = first_id;
		hdb_ffs->old_strings_count = hdb_ffs->strings_count;
	}

	hdb_ffs->ep0req = usb_ep_alloc_request(cdev->gadget->ep0, GFP_KERNEL);
	if (unlikely(!hdb_ffs->ep0req))
		return -ENOMEM;
	hdb_ffs->ep0req->complete = hdb_ffs_ep0_complete;
	hdb_ffs->ep0req->context = hdb_ffs;

	lang = hdb_ffs->stringtabs;
	if (lang) {
		for (; *lang; ++lang) {
			struct usb_string *str = (*lang)->strings;
			int id = hdb_ffs->first_id;
			for (; str->s; ++id, ++str)
				str->id = id;
		}
	}

	hdb_ffs->gadget = cdev->gadget;
	hdb_ffs_data_get(hdb_ffs);
	return 0;
}

static void functionfs_hdb_unbind(struct hdb_ffs_data *hdb_ffs)
{
	ENTER();

	if (!WARN_ON(!hdb_ffs->gadget)) {
		usb_ep_free_request(hdb_ffs->gadget->ep0, hdb_ffs->ep0req);
		hdb_ffs->ep0req = NULL;
		hdb_ffs->gadget = NULL;
		hdb_ffs_data_put(hdb_ffs);
		clear_bit(HDB_FFS_FL_BOUND, &hdb_ffs->flags);
	}
}

static int hdb_ffs_epfiles_create(struct hdb_ffs_data *hdb_ffs)
{
	struct hdb_ffs_epfile *epfile, *epfiles;
	unsigned i, count;

	ENTER();

	count = hdb_ffs->eps_count;
	epfiles = kcalloc(count, sizeof(*epfiles), GFP_KERNEL);
	if (!epfiles)
		return -ENOMEM;

	epfile = epfiles;
	for (i = 1; i <= count; ++i, ++epfile) {
		epfile->hdb_ffs = hdb_ffs;
		mutex_init(&epfile->mutex);
		init_waitqueue_head(&epfile->wait);
		sprintf(epfiles->name, "ep%u",  i);
		if (!unlikely(hdb_ffs_sb_create_file(hdb_ffs->sb, epfiles->name, epfile,
						 &hdb_ffs_epfile_operations,
						 &epfile->dentry))) {
			hdb_ffs_epfiles_destroy(epfiles, i - 1);
			return -ENOMEM;
		}
	}

	hdb_ffs->epfiles = epfiles;
	return 0;
}

static void hdb_ffs_epfiles_destroy(struct hdb_ffs_epfile *epfiles, unsigned count)
{
	struct hdb_ffs_epfile *epfile = epfiles;

	ENTER();

	for (; count; --count, ++epfile) {
		BUG_ON(mutex_is_locked(&epfile->mutex) ||
		       waitqueue_active(&epfile->wait));
		if (epfile->dentry) {
			d_delete(epfile->dentry);
			dput(epfile->dentry);
			epfile->dentry = NULL;
		}
	}

	kfree(epfiles);
}

static int functionfs_hdb_bind_config(struct usb_composite_dev *cdev,
				  struct usb_configuration *c,
				  struct hdb_ffs_data *hdb_ffs)
{
	struct hdb_ffs_function *func;
	int ret;

	ENTER();

	func = kzalloc(sizeof *func, GFP_KERNEL);
	if (unlikely(!func))
		return -ENOMEM;

	func->function.name    = "Function FS Gadget";
	func->function.strings = hdb_ffs->stringtabs;

	func->function.bind    = hdb_ffs_func_bind;
	func->function.unbind  = hdb_ffs_func_unbind;
	func->function.set_alt = hdb_ffs_func_set_alt;
	func->function.disable = hdb_ffs_func_disable;
	func->function.setup   = hdb_ffs_func_setup;
	func->function.suspend = hdb_ffs_func_suspend;
	func->function.resume  = hdb_ffs_func_resume;

	func->conf   = c;
	func->gadget = cdev->gadget;
	func->hdb_ffs = hdb_ffs;
	hdb_ffs_data_get(hdb_ffs);

	ret = usb_add_function(c, &func->function);
	if (unlikely(ret))
		hdb_ffs_func_free(func);

	return ret;
}

static void hdb_ffs_func_free(struct hdb_ffs_function *func)
{
	struct hdb_ffs_ep *ep         = func->eps;
	unsigned count            = func->hdb_ffs->eps_count;
	unsigned long flags;

	ENTER();

	/* cleanup after autoconfig */
	spin_lock_irqsave(&func->hdb_ffs->eps_lock, flags);
	do {
		if (ep->ep && ep->req)
			usb_ep_free_request(ep->ep, ep->req);
		ep->req = NULL;
		ep->ep = NULL;
		++ep;
	} while (--count);
	spin_unlock_irqrestore(&func->hdb_ffs->eps_lock, flags);

	hdb_ffs_data_put(func->hdb_ffs);

	kfree(func->eps);
	/*
	 * eps and interfaces_nums are allocated in the same chunk so
	 * only one free is required.  Descriptors are also allocated
	 * in the same chunk.
	 */

	kfree(func);
}

static void hdb_ffs_func_eps_disable(struct hdb_ffs_function *func)
{
	struct hdb_ffs_ep *ep         = func->eps;
	struct hdb_ffs_epfile *epfile = func->hdb_ffs->epfiles;
	unsigned count            = func->hdb_ffs->eps_count;
	unsigned long flags;

	spin_lock_irqsave(&func->hdb_ffs->eps_lock, flags);
	do {
		atomic_set(&epfile->error, 1);
		/* pending requests get nuked */
		if (likely(ep->ep)) {
			usb_ep_disable(ep->ep);
			ep->ep->driver_data = NULL;
		}
		epfile->ep = NULL;

		++ep;
		++epfile;
	} while (--count);
	spin_unlock_irqrestore(&func->hdb_ffs->eps_lock, flags);
}

static int hdb_ffs_func_eps_enable(struct hdb_ffs_function *func)
{
	struct hdb_ffs_data *hdb_ffs      = func->hdb_ffs;
	struct hdb_ffs_ep *ep         = func->eps;
	struct hdb_ffs_epfile *epfile = hdb_ffs->epfiles;
	unsigned count            = hdb_ffs->eps_count;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&func->hdb_ffs->eps_lock, flags);
	do {
		struct usb_endpoint_descriptor *ds;
		int desc_idx;

		if (hdb_ffs->gadget->speed == USB_SPEED_SUPER)
			desc_idx = 2;
		else if (hdb_ffs->gadget->speed == USB_SPEED_HIGH)
			desc_idx = 1;
		else
			desc_idx = 0;

		ds = ep->descs[desc_idx];
		if (!ds) {
			ret = -EINVAL;
			break;
		}

		ep->ep->driver_data = ep;
		ep->ep->desc = ds;
		ret = usb_ep_enable(ep->ep);
		if (likely(!ret)) {
			epfile->ep = ep;
			epfile->in = usb_endpoint_dir_in(ds);
			epfile->isoc = usb_endpoint_xfer_isoc(ds);
		} else {
			break;
		}

		wake_up(&epfile->wait);

		++ep;
		++epfile;
	} while (--count);
	spin_unlock_irqrestore(&func->hdb_ffs->eps_lock, flags);

	return ret;
}


/* Parsing and building descriptors and strings *****************************/

/*
 * This validates if data pointed by data is a valid USB descriptor as
 * well as record how many interfaces, endpoints and strings are
 * required by given configuration.  Returns address after the
 * descriptor or NULL if data is invalid.
 */

enum hdb_ffs_entity_type {
	HDB_FFS_DESCRIPTOR, HDB_FFS_INTERFACE, HDB_FFS_STRING, HDB_FFS_ENDPOINT
};

typedef int (*hdb_ffs_entity_callback)(enum hdb_ffs_entity_type entity,
				   u8 *valuep,
				   struct usb_descriptor_header *desc,
				   void *priv);

static int __must_check hdb_ffs_do_desc(char *data, unsigned len,
				    hdb_ffs_entity_callback entity, void *priv)
{
	struct usb_descriptor_header *_ds = (void *)data;
	u8 length;
	int ret;

	ENTER();

	/* At least two bytes are required: length and type */
	if (len < 2) {
		pr_vdebug("descriptor too short\n");
		return -EINVAL;
	}

	/* If we have at least as many bytes as the descriptor takes? */
	length = _ds->bLength;
	if (len < length) {
		pr_vdebug("descriptor longer then available data\n");
		return -EINVAL;
	}

#define __entity_check_INTERFACE(val)  1
#define __entity_check_STRING(val)     (val)
#define __entity_check_ENDPOINT(val)   ((val) & USB_ENDPOINT_NUMBER_MASK)
#define __entity(type, val) do {					\
		pr_vdebug("entity " #type "(%02x)\n", (val));		\
		if (unlikely(!__entity_check_ ##type(val))) {		\
			pr_vdebug("invalid entity's value\n");		\
			return -EINVAL;					\
		}							\
		ret = entity(HDB_FFS_ ##type, &val, _ds, priv);		\
		if (unlikely(ret < 0)) {				\
			pr_debug("entity " #type "(%02x); ret = %d\n",	\
				 (val), ret);				\
			return ret;					\
		}							\
	} while (0)

	/* Parse descriptor depending on type. */
	switch (_ds->bDescriptorType) {
	case USB_DT_DEVICE:
	case USB_DT_CONFIG:
	case USB_DT_STRING:
	case USB_DT_DEVICE_QUALIFIER:
		/* function can't have any of those */
		pr_vdebug("descriptor reserved for gadget: %d\n",
		      _ds->bDescriptorType);
		return -EINVAL;

	case USB_DT_INTERFACE: {
		struct usb_interface_descriptor *ds = (void *)_ds;
		pr_vdebug("interface descriptor\n");
		if (length != sizeof *ds)
			goto inv_length;

		__entity(INTERFACE, ds->bInterfaceNumber);
		if (ds->iInterface)
			__entity(STRING, ds->iInterface);
	}
		break;

	case USB_DT_ENDPOINT: {
		struct usb_endpoint_descriptor *ds = (void *)_ds;
		pr_vdebug("endpoint descriptor\n");
		if (length != USB_DT_ENDPOINT_SIZE &&
		    length != USB_DT_ENDPOINT_AUDIO_SIZE)
			goto inv_length;
		__entity(ENDPOINT, ds->bEndpointAddress);
	}
		break;

	case HID_DT_HID:
		pr_vdebug("hid descriptor\n");
		if (length != sizeof(struct hid_descriptor))
			goto inv_length;
		break;

	case USB_DT_OTG:
		if (length != sizeof(struct usb_otg_descriptor))
			goto inv_length;
		break;

	case USB_DT_INTERFACE_ASSOCIATION: {
		struct usb_interface_assoc_descriptor *ds = (void *)_ds;
		pr_vdebug("interface association descriptor\n");
		if (length != sizeof *ds)
			goto inv_length;
		if (ds->iFunction)
			__entity(STRING, ds->iFunction);
	}
		break;

	case USB_DT_SS_ENDPOINT_COMP:
		pr_vdebug("EP SS companion descriptor\n");
		if (length != sizeof(struct usb_ss_ep_comp_descriptor))
			goto inv_length;
		break;

	case USB_DT_OTHER_SPEED_CONFIG:
	case USB_DT_INTERFACE_POWER:
	case USB_DT_DEBUG:
	case USB_DT_SECURITY:
	case USB_DT_CS_RADIO_CONTROL:
		/* TODO */
		pr_vdebug("unimplemented descriptor: %d\n", _ds->bDescriptorType);
		return -EINVAL;

	default:
		/* We should never be here */
		pr_vdebug("unknown descriptor: %d\n", _ds->bDescriptorType);
		return -EINVAL;

inv_length:
		pr_vdebug("invalid length: %d (descriptor %d)\n",
			  _ds->bLength, _ds->bDescriptorType);
		return -EINVAL;
	}

#undef __entity
#undef __entity_check_DESCRIPTOR
#undef __entity_check_INTERFACE
#undef __entity_check_STRING
#undef __entity_check_ENDPOINT

	return length;
}

static int __must_check hdb_ffs_do_descs(unsigned count, char *data, unsigned len,
				     hdb_ffs_entity_callback entity, void *priv)
{
	const unsigned _len = len;
	unsigned long num = 0;

	ENTER();

	for (;;) {
		int ret;

		if (num == count)
			data = NULL;

		/* Record "descriptor" entity */
		ret = entity(HDB_FFS_DESCRIPTOR, (u8 *)num, (void *)data, priv);
		if (unlikely(ret < 0)) {
			pr_debug("entity DESCRIPTOR(%02lx); ret = %d\n",
				 num, ret);
			return ret;
		}

		if (!data)
			return _len - len;

		ret = hdb_ffs_do_desc(data, len, entity, priv);
		if (unlikely(ret < 0)) {
			pr_debug("%s returns %d\n", __func__, ret);
			return ret;
		}

		len -= ret;
		data += ret;
		++num;
	}
}

static int __hdb_ffs_data_do_entity(enum hdb_ffs_entity_type type,
				u8 *valuep, struct usb_descriptor_header *desc,
				void *priv)
{
	struct hdb_ffs_data *hdb_ffs = priv;

	ENTER();

	switch (type) {
	case HDB_FFS_DESCRIPTOR:
		break;

	case HDB_FFS_INTERFACE:
		/*
		 * Interfaces are indexed from zero so if we
		 * encountered interface "n" then there are at least
		 * "n+1" interfaces.
		 */
		if (*valuep >= hdb_ffs->interfaces_count)
			hdb_ffs->interfaces_count = *valuep + 1;
		break;

	case HDB_FFS_STRING:
		/*
		 * Strings are indexed from 1 (0 is magic ;) reserved
		 * for languages list or some such)
		 */
		if (*valuep > hdb_ffs->strings_count)
			hdb_ffs->strings_count = *valuep;
		break;

	case HDB_FFS_ENDPOINT:
		/* Endpoints are indexed from 1 as well. */
		if ((*valuep & USB_ENDPOINT_NUMBER_MASK) > hdb_ffs->eps_count)
			hdb_ffs->eps_count = (*valuep & USB_ENDPOINT_NUMBER_MASK);
		break;
	}

	return 0;
}

static int __hdb_ffs_data_got_descs(struct hdb_ffs_data *hdb_ffs,
				char *const _data, size_t len)
{
	unsigned fs_count, hs_count, ss_count = 0;
	int fs_len, hs_len, ss_len, ss_magic, ret = -EINVAL;
	char *data = _data;

	ENTER();

	if (unlikely(get_unaligned_le32(data) != FUNCTIONFS_DESCRIPTORS_MAGIC ||
		     get_unaligned_le32(data + 4) != len))
		goto error;
	fs_count = get_unaligned_le32(data +  8);
	hs_count = get_unaligned_le32(data + 12);

	data += 16;
	len  -= 16;

	if (likely(fs_count)) {
		fs_len = hdb_ffs_do_descs(fs_count, data, len,
				      __hdb_ffs_data_do_entity, hdb_ffs);
		if (unlikely(fs_len < 0)) {
			ret = fs_len;
			goto error;
		}

		data += fs_len;
		len  -= fs_len;
	} else {
		fs_len = 0;
	}

	if (likely(hs_count)) {
		hs_len = hdb_ffs_do_descs(hs_count, data, len,
				   __hdb_ffs_data_do_entity, hdb_ffs);
		if (unlikely(hs_len < 0)) {
			ret = hs_len;
			goto error;
		}
	} else {
		hs_len = 0;
	}

	if ((len >= hs_len + 8)) {
		/* Check SS_MAGIC for presence of ss_descs and get SS_COUNT */
		ss_magic = get_unaligned_le32(data + hs_len);
		if (ss_magic != FUNCTIONFS_SS_DESC_MAGIC)
			goto einval;

		ss_count = get_unaligned_le32(data + hs_len + 4);
		data += hs_len + 8;
		len  -= hs_len + 8;
	} else {
		data += hs_len;
		len  -= hs_len;
	}

	if (!fs_count && !hs_count && !ss_count)
		goto einval;

	if (ss_count) {
		ss_len = hdb_ffs_do_descs(ss_count, data, len,
				   __hdb_ffs_data_do_entity, hdb_ffs);
		if (unlikely(ss_len < 0)) {
			ret = ss_len;
			goto error;
		}
		ret = ss_len;
	} else {
		ss_len = 0;
		ret = 0;
	}

	if (unlikely(len != ret))
		goto einval;

	hdb_ffs->raw_fs_hs_descs_length	 = fs_len + hs_len;
	hdb_ffs->raw_ss_descs_length	 = ss_len;
	hdb_ffs->raw_descs_length		 = hdb_ffs->raw_fs_hs_descs_length + ss_len;
	hdb_ffs->raw_descs			 = _data;
	hdb_ffs->fs_descs_count		 = fs_count;
	hdb_ffs->hs_descs_count		 = hs_count;
	hdb_ffs->ss_descs_count		 = ss_count;
	if (hdb_ffs->ss_descs_count)
		hdb_ffs->raw_ss_descs_offset = 16 + hdb_ffs->raw_fs_hs_descs_length + 8;

	return 0;

einval:
	ret = -EINVAL;
error:
	kfree(_data);
	return ret;
}

static int __hdb_ffs_data_got_strings(struct hdb_ffs_data *hdb_ffs,
				  char *const _data, size_t len)
{
	u32 str_count, needed_count, lang_count;
	struct usb_gadget_strings **stringtabs, *t;
	struct usb_string *strings, *s;
	const char *data = _data;

	ENTER();

	if (unlikely(get_unaligned_le32(data) != FUNCTIONFS_STRINGS_MAGIC ||
		     get_unaligned_le32(data + 4) != len))
		goto error;
	str_count  = get_unaligned_le32(data + 8);
	lang_count = get_unaligned_le32(data + 12);

	/* if one is zero the other must be zero */
	if (unlikely(!str_count != !lang_count))
		goto error;

	/* Do we have at least as many strings as descriptors need? */
	needed_count = hdb_ffs->strings_count;
	if (unlikely(str_count < needed_count))
		goto error;

	/*
	 * If we don't need any strings just return and free all
	 * memory.
	 */
	if (!needed_count) {
		kfree(_data);
		return 0;
	}

	/* Allocate everything in one chunk so there's less maintenance. */
	{
		struct {
			struct usb_gadget_strings *stringtabs[lang_count + 1];
			struct usb_gadget_strings stringtab[lang_count];
			struct usb_string strings[lang_count*(needed_count+1)];
		} *d;
		unsigned i = 0;

		d = kmalloc(sizeof *d, GFP_KERNEL);
		if (unlikely(!d)) {
			kfree(_data);
			return -ENOMEM;
		}

		stringtabs = d->stringtabs;
		t = d->stringtab;
		i = lang_count;
		do {
			*stringtabs++ = t++;
		} while (--i);
		*stringtabs = NULL;

		stringtabs = d->stringtabs;
		t = d->stringtab;
		s = d->strings;
		strings = s;
	}

	/* For each language */
	data += 16;
	len -= 16;

	do { /* lang_count > 0 so we can use do-while */
		unsigned needed = needed_count;

		if (unlikely(len < 3))
			goto error_free;
		t->language = get_unaligned_le16(data);
		t->strings  = s;
		++t;

		data += 2;
		len -= 2;

		/* For each string */
		do { /* str_count > 0 so we can use do-while */
			size_t length = strnlen(data, len);

			if (unlikely(length == len))
				goto error_free;

			/*
			 * User may provide more strings then we need,
			 * if that's the case we simply ignore the
			 * rest
			 */
			if (likely(needed)) {
				/*
				 * s->id will be set while adding
				 * function to configuration so for
				 * now just leave garbage here.
				 */
				s->s = data;
				--needed;
				++s;
			}

			data += length + 1;
			len -= length + 1;
		} while (--str_count);

		s->id = 0;   /* terminator */
		s->s = NULL;
		++s;

	} while (--lang_count);

	/* Some garbage left? */
	if (unlikely(len))
		goto error_free;

	/* Done! */
	hdb_ffs->stringtabs = stringtabs;
	hdb_ffs->raw_strings = _data;

	return 0;

error_free:
	kfree(stringtabs);
error:
	kfree(_data);
	return -EINVAL;
}


/* Events handling and management *******************************************/

static void __hdb_ffs_event_add(struct hdb_ffs_data *hdb_ffs,
			    enum usb_functionfs_event_type type)
{
	enum usb_functionfs_event_type rem_type1, rem_type2 = type;
	int neg = 0;

	/*
	 * Abort any unhandled setup
	 *
	 * We do not need to worry about some cmpxchg() changing value
	 * of hdb_ffs->setup_state without holding the lock because when
	 * state is HDB_FFS_SETUP_PENDING cmpxchg() in several places in
	 * the source does nothing.
	 */
	if (hdb_ffs->setup_state == HDB_FFS_SETUP_PENDING)
		hdb_ffs->setup_state = HDB_FFS_SETUP_CANCELED;

	switch (type) {
	case FUNCTIONFS_RESUME:
		rem_type2 = FUNCTIONFS_SUSPEND;
		/* FALL THROUGH */
	case FUNCTIONFS_SUSPEND:
	case FUNCTIONFS_SETUP:
		rem_type1 = type;
		/* Discard all similar events */
		break;

	case FUNCTIONFS_BIND:
	case FUNCTIONFS_UNBIND:
	case FUNCTIONFS_DISABLE:
	case FUNCTIONFS_ENABLE:
		/* Discard everything other then power management. */
		rem_type1 = FUNCTIONFS_SUSPEND;
		rem_type2 = FUNCTIONFS_RESUME;
		neg = 1;
		break;

	default:
		BUG();
	}

	{
		u8 *ev  = hdb_ffs->ev.types, *out = ev;
		unsigned n = hdb_ffs->ev.count;
		for (; n; --n, ++ev)
			if ((*ev == rem_type1 || *ev == rem_type2) == neg)
				*out++ = *ev;
			else
				pr_vdebug("purging event %d\n", *ev);
		hdb_ffs->ev.count = out - hdb_ffs->ev.types;
	}

	pr_vdebug("adding event %d\n", type);
	hdb_ffs->ev.types[hdb_ffs->ev.count++] = type;
	wake_up_locked(&hdb_ffs->ev.waitq);
}

static void hdb_ffs_event_add(struct hdb_ffs_data *hdb_ffs,
			  enum usb_functionfs_event_type type)
{
	unsigned long flags;
	spin_lock_irqsave(&hdb_ffs->ev.waitq.lock, flags);
	__hdb_ffs_event_add(hdb_ffs, type);
	spin_unlock_irqrestore(&hdb_ffs->ev.waitq.lock, flags);
}


/* Bind/unbind USB function hooks *******************************************/

static int __hdb_ffs_func_bind_do_descs(enum hdb_ffs_entity_type type, u8 *valuep,
				    struct usb_descriptor_header *desc,
				    void *priv)
{
	struct usb_endpoint_descriptor *ds = (void *)desc;
	struct hdb_ffs_function *func = priv;
	struct hdb_ffs_ep *hdb_ffs_ep;

	/*
	 * If hs_descriptors is not NULL then we are reading hs
	 * descriptors now
	 */
	const int is_hs = func->function.hs_descriptors != NULL;
	const int is_ss = func->function.ss_descriptors != NULL;
	unsigned ep_desc_id, idx;

	if (type != HDB_FFS_DESCRIPTOR)
		return 0;

	if (is_ss) {
		func->function.ss_descriptors[(long)valuep] = desc;
		ep_desc_id = 2;
	} else if (is_hs) {
		func->function.hs_descriptors[(long)valuep] = desc;
		ep_desc_id = 1;
	} else {
		func->function.fs_descriptors[(long)valuep]    = desc;
		ep_desc_id = 0;
	}

	if (!desc || desc->bDescriptorType != USB_DT_ENDPOINT)
		return 0;

	idx = (ds->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK) - 1;
	hdb_ffs_ep = func->eps + idx;

	if (unlikely(hdb_ffs_ep->descs[ep_desc_id])) {
		pr_vdebug("two %sspeed descriptors for EP %d\n",
			  is_ss ? "super" : "high/full",
			  ds->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
		return -EINVAL;
	}
	hdb_ffs_ep->descs[ep_desc_id] = ds;

	hdb_ffs_dump_mem(": Original  ep desc", ds, ds->bLength);
	if (hdb_ffs_ep->ep) {
		ds->bEndpointAddress = hdb_ffs_ep->descs[0]->bEndpointAddress;
		if (!ds->wMaxPacketSize)
			ds->wMaxPacketSize = hdb_ffs_ep->descs[0]->wMaxPacketSize;
	} else {
		struct usb_request *req;
		struct usb_ep *ep;

		pr_vdebug("autoconfig\n");
		ep = usb_ep_autoconfig(func->gadget, ds);
		if (unlikely(!ep))
			return -ENOTSUPP;
		ep->driver_data = func->eps + idx;

		req = usb_ep_alloc_request(ep, GFP_KERNEL);
		if (unlikely(!req))
			return -ENOMEM;

		hdb_ffs_ep->ep  = ep;
		hdb_ffs_ep->req = req;
		func->eps_revmap[ds->bEndpointAddress &
				 USB_ENDPOINT_NUMBER_MASK] = idx + 1;
	}
	hdb_ffs_dump_mem(": Rewritten ep desc", ds, ds->bLength);

	return 0;
}

static int __hdb_ffs_func_bind_do_nums(enum hdb_ffs_entity_type type, u8 *valuep,
				   struct usb_descriptor_header *desc,
				   void *priv)
{
	struct hdb_ffs_function *func = priv;
	unsigned idx;
	u8 newValue;

	switch (type) {
	default:
	case HDB_FFS_DESCRIPTOR:
		/* Handled in previous pass by __hdb_ffs_func_bind_do_descs() */
		return 0;

	case HDB_FFS_INTERFACE:
		idx = *valuep;
		if (func->interfaces_nums[idx] < 0) {
			int id = usb_interface_id(func->conf, &func->function);
			if (unlikely(id < 0))
				return id;
			func->interfaces_nums[idx] = id;
		}
		newValue = func->interfaces_nums[idx];
		break;

	case HDB_FFS_STRING:
		/* String' IDs are allocated when fsf_data is bound to cdev */
		newValue = func->hdb_ffs->stringtabs[0]->strings[*valuep - 1].id;
		break;

	case HDB_FFS_ENDPOINT:
		/*
		 * USB_DT_ENDPOINT are handled in
		 * __hdb_ffs_func_bind_do_descs().
		 */
		if (desc->bDescriptorType == USB_DT_ENDPOINT)
			return 0;

		idx = (*valuep & USB_ENDPOINT_NUMBER_MASK) - 1;
		if (unlikely(!func->eps[idx].ep))
			return -EINVAL;

		{
			struct usb_endpoint_descriptor **descs;
			descs = func->eps[idx].descs;
			newValue = descs[descs[0] ? 0 : 1]->bEndpointAddress;
		}
		break;
	}

	pr_vdebug("%02x -> %02x\n", *valuep, newValue);
	*valuep = newValue;
	return 0;
}

static int hdb_ffs_func_bind(struct usb_configuration *c,
			 struct usb_function *f)
{
	struct hdb_ffs_function *func = hdb_ffs_func_from_usb(f);
	struct hdb_ffs_data *hdb_ffs = func->hdb_ffs;

	const int full = !!func->hdb_ffs->fs_descs_count;
	const int high = gadget_is_dualspeed(func->gadget) &&
		func->hdb_ffs->hs_descs_count;
	const int super = gadget_is_superspeed(func->gadget) &&
		func->hdb_ffs->ss_descs_count;

	int fs_len, hs_len, ret;

	/* Make it a single chunk, less management later on */
	struct {
		struct hdb_ffs_ep eps[hdb_ffs->eps_count];
		struct usb_descriptor_header
			*fs_descs[full ? hdb_ffs->fs_descs_count + 1 : 0];
		struct usb_descriptor_header
			*hs_descs[high ? hdb_ffs->hs_descs_count + 1 : 0];
		struct usb_descriptor_header
			*ss_descs[super ? hdb_ffs->ss_descs_count + 1 : 0];
		short inums[hdb_ffs->interfaces_count];
		char raw_descs[hdb_ffs->raw_descs_length];
	} *data;

	ENTER();

	/* Only high/super speed but not supported by gadget? */
	if (unlikely(!(full | high | super)))
		return -ENOTSUPP;

	/* Allocate */
	data = kmalloc(sizeof *data, GFP_KERNEL);
	if (unlikely(!data))
		return -ENOMEM;

	/* Zero */
	memset(data->eps, 0, sizeof data->eps);
	/* Copy only raw (hs,fs) descriptors (until ss_magic and ss_count) */
	memcpy(data->raw_descs, hdb_ffs->raw_descs + 16,
				hdb_ffs->raw_fs_hs_descs_length);
	/* Copy SS descriptors */
	if (func->hdb_ffs->ss_descs_count)
		memcpy(data->raw_descs + hdb_ffs->raw_fs_hs_descs_length,
			hdb_ffs->raw_descs + hdb_ffs->raw_ss_descs_offset,
			hdb_ffs->raw_ss_descs_length);

	memset(data->inums, 0xff, sizeof data->inums);
	for (ret = hdb_ffs->eps_count; ret; --ret)
		data->eps[ret].num = -1;

	/* Save pointers */
	func->eps             = data->eps;
	func->interfaces_nums = data->inums;

	/*
	 * Go through all the endpoint descriptors and allocate
	 * endpoints first, so that later we can rewrite the endpoint
	 * numbers without worrying that it may be described later on.
	 */
	if (likely(full)) {
		func->function.fs_descriptors = data->fs_descs;
		fs_len = hdb_ffs_do_descs(hdb_ffs->fs_descs_count,
				   data->raw_descs,
				   sizeof(data->raw_descs),
				   __hdb_ffs_func_bind_do_descs, func);
		if (unlikely(fs_len < 0)) {
			ret = fs_len;
			goto error;
		}
	} else {
		fs_len = 0;
	}

	if (likely(high)) {
		func->function.hs_descriptors = data->hs_descs;
		hs_len = hdb_ffs_do_descs(hdb_ffs->hs_descs_count,
				   data->raw_descs + fs_len,
				   (sizeof(data->raw_descs)) - fs_len,
				   __hdb_ffs_func_bind_do_descs, func);
		if (unlikely(hs_len < 0)) {
			ret = hs_len;
			goto error;
		}
	} else {
		hs_len = 0;
	}

	if (likely(super)) {
		func->function.ss_descriptors = data->ss_descs;
		ret = hdb_ffs_do_descs(hdb_ffs->ss_descs_count,
				   data->raw_descs + fs_len + hs_len,
				   (sizeof(data->raw_descs)) - fs_len - hs_len,
				   __hdb_ffs_func_bind_do_descs, func);
		if (unlikely(ret < 0))
			goto error;
	}


	/*
	 * Now handle interface numbers allocation and interface and
	 * endpoint numbers rewriting.  We can do that in one go
	 * now.
	 */
	ret = hdb_ffs_do_descs(hdb_ffs->fs_descs_count +
			   (high ? hdb_ffs->hs_descs_count : 0) +
			   (super ? hdb_ffs->ss_descs_count : 0),
			   data->raw_descs, sizeof(data->raw_descs),
			   __hdb_ffs_func_bind_do_nums, func);
	if (unlikely(ret < 0))
		goto error;

	/* And we're done */
	hdb_ffs_event_add(hdb_ffs, FUNCTIONFS_BIND);
	return 0;

error:
	/* XXX Do we need to release all claimed endpoints here? */
	return ret;
}


/* Other USB function hooks *************************************************/

static void hdb_ffs_func_unbind(struct usb_configuration *c,
			    struct usb_function *f)
{
	struct hdb_ffs_function *func = hdb_ffs_func_from_usb(f);
	struct hdb_ffs_data *hdb_ffs = func->hdb_ffs;

	ENTER();

	if (hdb_ffs->func == func) {
		hdb_ffs_func_eps_disable(func);
		hdb_ffs->func = NULL;
	}

	hdb_ffs_event_add(hdb_ffs, FUNCTIONFS_UNBIND);

	hdb_ffs_func_free(func);
}

static int hdb_ffs_func_set_alt(struct usb_function *f,
			    unsigned interface, unsigned alt)
{
	struct hdb_ffs_function *func = hdb_ffs_func_from_usb(f);
	struct hdb_ffs_data *hdb_ffs = func->hdb_ffs;
	int ret = 0, intf;

	if (alt != (unsigned)-1) {
		intf = hdb_ffs_func_revmap_intf(func, interface);
		if (unlikely(intf < 0))
			return intf;
	}

	if (hdb_ffs->func) {
		hdb_ffs_func_eps_disable(hdb_ffs->func);
		hdb_ffs->func = NULL;
	}

	if (hdb_ffs->state != HDB_FFS_ACTIVE)
		return -ENODEV;

	if (alt == (unsigned)-1) {
		hdb_ffs->func = NULL;
		hdb_ffs_event_add(hdb_ffs, FUNCTIONFS_DISABLE);
		return 0;
	}

	hdb_ffs->func = func;
	ret = hdb_ffs_func_eps_enable(func);
	if (likely(ret >= 0))
		hdb_ffs_event_add(hdb_ffs, FUNCTIONFS_ENABLE);
	return ret;
}

static void hdb_ffs_func_disable(struct usb_function *f)
{
	hdb_ffs_func_set_alt(f, 0, (unsigned)-1);
}

static int hdb_ffs_func_setup(struct usb_function *f,
			  const struct usb_ctrlrequest *creq)
{
	struct hdb_ffs_function *func = hdb_ffs_func_from_usb(f);
	struct hdb_ffs_data *hdb_ffs = func->hdb_ffs;
	unsigned long flags;
	int ret;

	ENTER();

	pr_vdebug("creq->bRequestType = %02x\n", creq->bRequestType);
	pr_vdebug("creq->bRequest     = %02x\n", creq->bRequest);
	pr_vdebug("creq->wValue       = %04x\n", le16_to_cpu(creq->wValue));
	pr_vdebug("creq->wIndex       = %04x\n", le16_to_cpu(creq->wIndex));
	pr_vdebug("creq->wLength      = %04x\n", le16_to_cpu(creq->wLength));

	/*
	 * Most requests directed to interface go through here
	 * (notable exceptions are set/get interface) so we need to
	 * handle them.  All other either handled by composite or
	 * passed to usb_configuration->setup() (if one is set).  No
	 * matter, we will handle requests directed to endpoint here
	 * as well (as it's straightforward) but what to do with any
	 * other request?
	 */
	if (hdb_ffs->state != HDB_FFS_ACTIVE)
		return -ENODEV;

	switch (creq->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_INTERFACE:
		ret = hdb_ffs_func_revmap_intf(func, le16_to_cpu(creq->wIndex));
		if (unlikely(ret < 0))
			return ret;
		break;

	case USB_RECIP_ENDPOINT:
		ret = hdb_ffs_func_revmap_ep(func, le16_to_cpu(creq->wIndex));
		if (unlikely(ret < 0))
			return ret;
		break;

	default:
		return -EOPNOTSUPP;
	}

	spin_lock_irqsave(&hdb_ffs->ev.waitq.lock, flags);
	hdb_ffs->ev.setup = *creq;
	hdb_ffs->ev.setup.wIndex = cpu_to_le16(ret);
	__hdb_ffs_event_add(hdb_ffs, FUNCTIONFS_SETUP);
	spin_unlock_irqrestore(&hdb_ffs->ev.waitq.lock, flags);

	return 0;
}

static void hdb_ffs_func_suspend(struct usb_function *f)
{
	ENTER();
	hdb_ffs_event_add(hdb_ffs_func_from_usb(f)->hdb_ffs, FUNCTIONFS_SUSPEND);
}

static void hdb_ffs_func_resume(struct usb_function *f)
{
	ENTER();
	hdb_ffs_event_add(hdb_ffs_func_from_usb(f)->hdb_ffs, FUNCTIONFS_RESUME);
}


/* Endpoint and interface numbers reverse mapping ***************************/

static int hdb_ffs_func_revmap_ep(struct hdb_ffs_function *func, u8 num)
{
	num = func->eps_revmap[num & USB_ENDPOINT_NUMBER_MASK];
	return num ? num : -EDOM;
}

static int hdb_ffs_func_revmap_intf(struct hdb_ffs_function *func, u8 intf)
{
	short *nums = func->interfaces_nums;
	unsigned count = func->hdb_ffs->interfaces_count;

	for (; count; --count, ++nums) {
		if (*nums >= 0 && *nums == intf)
			return nums - func->interfaces_nums;
	}

	return -EDOM;
}


/* Misc helper functions ****************************************************/

static int hdb_ffs_mutex_lock(struct mutex *mutex, unsigned nonblock)
{
	return nonblock
		? likely(mutex_trylock(mutex)) ? 0 : -EAGAIN
		: mutex_lock_interruptible(mutex);
}

static char *hdb_ffs_prepare_buffer(const char __user *buf, size_t len)
{
	char *data;

	if (unlikely(!len))
		return NULL;

	data = kmalloc(len, GFP_KERNEL);
	if (unlikely(!data))
		return ERR_PTR(-ENOMEM);

	if (unlikely(__copy_from_user(data, buf, len))) {
		kfree(data);
		return ERR_PTR(-EFAULT);
	}

	pr_vdebug("Buffer from user space:\n");
	hdb_ffs_dump_mem("", data, len);

	return data;
}

