#ifndef __LINUX_FUNCTIONFS_HDB_H__
#define __LINUX_FUNCTIONFS_HDB_H__ 1

#include <uapi/linux/usb/functionfs.h>


struct hdb_ffs_data;
struct usb_composite_dev;
struct usb_configuration;

static int  functionfs_hdb_init(void) __attribute__((warn_unused_result));
static void functionfs_hdb_cleanup(void);

static int functionfs_hdb_bind(struct hdb_ffs_data *ffs, struct usb_composite_dev *cdev)
	__attribute__((warn_unused_result, nonnull));
static void functionfs_hdb_unbind(struct hdb_ffs_data *ffs)
	__attribute__((nonnull));

static int functionfs_hdb_bind_config(struct usb_composite_dev *cdev,
				  struct usb_configuration *c,
				  struct hdb_ffs_data *ffs)
	__attribute__((warn_unused_result, nonnull));

static int functionfs_hdb_ready_callback(struct hdb_ffs_data *ffs)
	__attribute__((warn_unused_result, nonnull));
static void functionfs_hdb_closed_callback(struct hdb_ffs_data *ffs)
	__attribute__((nonnull));
static void *functionfs_hdb_acquire_dev_callback(const char *dev_name)
	__attribute__((warn_unused_result, nonnull));
static void functionfs_hdb_release_dev_callback(struct hdb_ffs_data *hdb_ffs_data)
	__attribute__((nonnull));

#endif

