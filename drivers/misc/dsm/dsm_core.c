/**********************************************************
 * Filename:	dsm_core.c
 *
 * Discription: Huawei device state monitor driver
 *
 * Copyright: (C) 2014 huawei.
 *
 * Author: sjm
 *
**********************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include "dsm_core.h"
#include "dsm_lib.h"
#include <linux/dsm_pub.h>
/*set default value 0, if when debug, we set it 1*/
int debug_output = 0;
module_param_named(dsm_debug, debug_output, int, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(dsm_debug, "set dsm dubug on/off (off: debug_output == 0)");

static struct dsm_server g_dsm_server;
static struct work_struct dsm_work;


static struct semaphore dsm_wbs;
static struct dsm_client *ext_dsm_client[EXTERN_DSM_CLIENT_MAX];
static struct dsm_dev ext_dev[EXTERN_DSM_CLIENT_MAX];

/* registe client on server */
struct dsm_client *dsm_register_client (struct dsm_dev *dev)
{
	int i;
	int size;
	int conflict = -1;
	struct dsm_client *ptr = NULL;

	if(g_dsm_server.server_state != DSM_SERVER_INITED){
		DSM_LOG_ERR("dsm server uninited\n");
		goto out;
	}

	if(dev == NULL){
		DSM_LOG_ERR("dsm_dev is NULL\n");
		goto out;
	}

	/* memory barrier */
	smp_rmb();
	/* whether client list is full */
	if(g_dsm_server.client_count < CLIENT_SIZE){
		/* malloc memory for this client */
		ptr = (struct dsm_client *)kzalloc((sizeof(struct dsm_client)+dev->buff_size), GFP_KERNEL);
		if(!ptr){
			DSM_LOG_ERR("clients malloc failed\n");
			goto out;
		}

		mutex_lock(&g_dsm_server.mtx_lock);
		/* try to find a free location on server */
		for(i=0; i<CLIENT_SIZE; i++){
			/* whether the client is free */
			if(!test_bit(DSM_CLIENT_VAILD_BIT, &g_dsm_server.client_flag[i]))
				break;
			/* if client is not free,whether a same client is exist */
			conflict = strncmp(g_dsm_server.client_list[i]->client_name, dev->name, CLIENT_NAME_LEN);
			if(!conflict){
				DSM_LOG_ERR("new client %s conflict with No.%d client %s\n",
					dev->name, i, g_dsm_server.client_list[i]->client_name);
				break;
			}
		}

		/* init a client */
		if(i < CLIENT_SIZE && conflict){
			size = strlen(dev->name);
			size = (size < CLIENT_NAME_LEN) ? size : (CLIENT_NAME_LEN - 1);
			memcpy(ptr->client_name, dev->name, size);	// need add a end symbol? size+1?
			ptr->client_id = i;
			ptr->cops = dev->fops;
			ptr->buff_size = dev->buff_size;
			init_waitqueue_head(&ptr->waitq);
			g_dsm_server.client_list[i] = ptr;
			set_bit(DSM_CLIENT_VAILD_BIT, &g_dsm_server.client_flag[i]);
			g_dsm_server.client_count++;
			smp_wmb();
		}else{
			/* if a same client is exist, donot registe */
			DSM_LOG_ERR("clients register failed, index %d, conflict %d\n", i, conflict);
			kfree(ptr);
			ptr = NULL;
		}
		mutex_unlock(&g_dsm_server.mtx_lock);
	}
	else
		DSM_LOG_INFO("clients has full\n");

out:
	return ptr;
}


/**
 * func - unregister dsm_client form server
 * @dsm_client: the client has registered in server
 * @dev: the device which used for creating dsm_client.
 *
 *		find out the same name of dev->name in the server's client_list, and clear the bit, free the dsm_client.
 * NOTE:
 */
void dsm_unregister_client (struct dsm_client *dsm_client,struct dsm_dev *dev)
{
	int i;
	int conflict;

	if(!dsm_client){
		pr_info("dsm_client is NULL, no need to unregister");
		return;
	}

	for(i=0; i<CLIENT_SIZE; i++){
		/* find the client and free it */
		conflict = strncmp(g_dsm_server.client_list[i]->client_name, dev->name, CLIENT_NAME_LEN);
		if(!conflict){
			__clear_bit(DSM_CLIENT_VAILD_BIT, &g_dsm_server.client_flag[i]);
			g_dsm_server.client_list[i] = NULL;
			g_dsm_server.client_count--;
			kfree(dsm_client);
			dsm_client = NULL;
			break;
		}
	}
}

/* set buff as busy, and judge it's status */
inline int dsm_client_ocuppy(struct dsm_client *client)
{
	int ret = -1;
	if(client){
		smp_rmb();
		/* Set CBUFF_OCCUPY_BIT bit as 1, and return its old value */
		ret = test_and_set_bit(CBUFF_OCCUPY_BIT, &client->buff_flag);
	}
	return ret;
}

/* client report its error */
void dsm_client_notify(struct dsm_client *client, int error_no)
{
	if(client){
		client->error_no = error_no;
		/* set client CBUFF_READY_BIT bit as 1 */
		set_bit(CBUFF_READY_BIT, &client->buff_flag);
		/* set service client flag DSM_CLIENT_NOTIFY_BIT bit as 1 */
		set_bit(DSM_CLIENT_NOTIFY_BIT, &g_dsm_server.client_flag[client->client_id]);
		smp_wmb();
		/* add the work to queue, the work will wake up client wait queue */
		queue_work(g_dsm_server.dsm_wq, &dsm_work);
	}
	return;
}

/* write some msg to the buffer */
int dsm_client_record(struct dsm_client *client, const char *fmt, ...)
{
	va_list ap;
	int size = 0;
	char *str;
	struct snprintf_ctxt ctxt;

	if(!client){
		DSM_LOG_ERR("%s no client to record\n", __func__);
		goto out;
	}

	if(client->buff_size <= client->used_size){
		DSM_LOG_ERR("%s no buffer to record\n", __func__);
		goto out;
	}

	ctxt.avail = client->buff_size - client->used_size -1;
	str = (char *)&client->dump_buff[client->used_size];
	ctxt.next = str;

	va_start(ap, fmt);
	__xprintf(fmt, ap, printf_putc, &ctxt);
	va_end(ap);

	*ctxt.next = 0;
	size = ctxt.next - str;
	client->used_size += size;

out:
	return size;
}

/* write data to client buffer, record the uesd sieze */
int dsm_client_copy(struct dsm_client *client, void *src, int sz)
{
	int size = 0;

	if(!client){
		DSM_LOG_ERR("%s no client to record\n", __func__);
		goto out;
	}

	if((client->used_size + sz) > client->buff_size){
		DSM_LOG_ERR("%s no enough buffer to record\n", __func__);
		goto out;
	}

	size = sz;
	memcpy(&client->dump_buff[client->used_size], src, size);
	client->used_size += size;

out:
	return size;
}

/* whether the client buffer is ready */
static inline int dsm_client_readable(struct dsm_client *client)
{
	int ret = 0;
	if(client){
		smp_rmb();
		ret = test_bit(CBUFF_READY_BIT, &client->buff_flag);
	}
	return ret;
}

static inline int dsm_atoi(const char* p){
	int val = 0;

	if(!p)
		return -1;

	while(isdigit(*p))
		val = val*10 + (*p++ - '0');

	return val;
}

/* copy a int value to user */
static inline int copy_int_to_user(void __user *argp, int val)
{
	int ret;
	int size;
	char buff[UINT_BUF_MAX]={0};

	size = snprintf(buff, UINT_BUF_MAX, "%d\n", val);
	ret = copy_to_user(argp, buff, size);
	DSM_LOG_DEBUG("%s result %d\n",__func__, ret);
	return ret;
}

/* find a client on server by client name  */
static struct dsm_client *dsm_find_client(const char *cname)
{
	int i;
	struct dsm_client * client = NULL;

	mutex_lock(&g_dsm_server.mtx_lock);
	smp_rmb();
	for(i=0; i<CLIENT_SIZE; i++){
		if((test_bit(DSM_CLIENT_VAILD_BIT, &g_dsm_server.client_flag[i]))
			&& (!strncasecmp(g_dsm_server.client_list[i]->client_name, cname, CLIENT_NAME_LEN))){
			client = g_dsm_server.client_list[i];
			break;
		}
	}
	mutex_unlock(&g_dsm_server.mtx_lock);
	DSM_LOG_DEBUG("cname: %s find %s\n", cname, client?"success":"failed");

	return client;
}

/*****************************************************************
Parameters    :  ext_client : the arg that from userspace used to register the dsm_client
Return        :
Description   : register the exteranl dsm_client(it maybe not a device, it can be a virtual device,
			such as service, process ....) from userspace with ioctl
*****************************************************************/
static int dsm_register_extern_client(struct dsm_extern_client* ext_client)
{
	int ret = 0;
	static int ext_client_cnt = 0;

	ext_dev[ext_client_cnt].buff_size = ext_client->buf_size;
	ext_dev[ext_client_cnt].name = ext_client->client_name;

	if (0 >= ext_dev[ext_client_cnt].buff_size
		|| NULL == ext_dev[ext_client_cnt].name
		|| ext_client_cnt >= EXTERN_DSM_CLIENT_MAX) {
		pr_err("[dsm_register_extern_client]client name or buf_size is fault."
			   "dont register!\n");
		return -ENOENT;
	} else if (NULL != dsm_find_client(ext_dev[ext_client_cnt].name)) {
		pr_err("[dsm_register_extern_client]register %s has exist, dont register again!\n",
			   ext_dev[ext_client_cnt].name);
		return -EEXIST;
	}

	ext_dsm_client[ext_client_cnt] = dsm_register_client(&ext_dev[ext_client_cnt]);
	if (!ext_dsm_client[ext_client_cnt]) {
		pr_err("[dsm_register_extern_client]register %s failed!\n", ext_dev[ext_client_cnt].name);
		ret = -ENOMEM;
		return ret;
	}

	ext_client_cnt++;
	return 0;
}

/* reset a client */
static inline void dsm_client_set_idle(struct dsm_client *client)
{
	client->used_size = 0;
	client->read_size = 0;
	client->error_no = 0;
	memset(client->dump_buff, 0, client->buff_size);
	clear_bit(CBUFF_READY_BIT, &client->buff_flag);
	clear_bit(CBUFF_OCCUPY_BIT, &client->buff_flag);
	clear_bit(DSM_CLIENT_NOTIFY_BIT, &g_dsm_server.client_flag[client->client_id]);
	smp_wmb();
	return;
}

static inline void dsm_bind_client(struct dsm_client *client)
{
	return;
}


static inline void dsm_unbind_client(struct dsm_client *client)
{
	return;
}

/* notify work */
static void dsm_work_func(struct work_struct *work)
{
	int i;
	struct dsm_client *client;

	DSM_LOG_DEBUG("%s enter\n", __func__);
	mutex_lock(&g_dsm_server.mtx_lock);
	smp_rmb();
	for(i=0; i<CLIENT_SIZE; i++){
		/* whether it is a valid client */
		if(test_bit(DSM_CLIENT_VAILD_BIT, &g_dsm_server.client_flag[i])){
			DSM_LOG_DEBUG("No.%d client name %s flag 0x%lx\n", i,
				g_dsm_server.client_list[i]->client_name, g_dsm_server.client_flag[i]);
			/* whether the client report error msg, clear a bit and return its old value */
			if(!test_and_clear_bit(DSM_CLIENT_NOTIFY_BIT, &g_dsm_server.client_flag[i]))
				continue;

			client = g_dsm_server.client_list[i];
			if(client == NULL){
				DSM_LOG_INFO("%d client is null client.\n",i);
				continue;
			}
			/* wake up wait queue */
			wake_up_interruptible_all(&client->waitq);
			DSM_LOG_INFO("%s finish notify\n", client->client_name);
		}
	}
	mutex_unlock(&g_dsm_server.mtx_lock);
	DSM_LOG_DEBUG("%s exit\n", __func__);

	return;
}

/* sysfs read function */
static ssize_t dsm_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct dsm_client *client = file->private_data;
	size_t copy_size = 0;

	DSM_LOG_DEBUG("%s enter\n",__func__);

	if(!client){
		DSM_LOG_ERR("client not bind\n");
		goto out;
	}

	if(dsm_client_readable(client)){
		copy_size = min(count, (client->used_size -client->read_size));
		if(copy_to_user(buf, &client->dump_buff[client->read_size], copy_size))
			DSM_LOG_ERR("copy to user failed\n");
		client->read_size += copy_size;
		if(client->read_size >= client->used_size)
			dsm_client_set_idle(client);
		DSM_LOG_DEBUG("%d bytes read to user\n", copy_size);
	}

out:
	DSM_LOG_DEBUG("%s exit\n",__func__);
	return copy_size;
}

/* sysfs write function */
static ssize_t dsm_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char client_name[CLIENT_NAME_LEN]={0};
	int size;
	struct dsm_client *client = NULL;
	char *buff = NULL;
	char *ptr;
	char  err_string[20] = {0};
	int   err;
	DSM_LOG_INFO("%s enter\n",__func__);


	/* try to get control of the write buffer */
	if (down_trylock(&dsm_wbs)) {
		/* somebody else has it now;
		 * if we're non-blocking, then exit...
		 */
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		/* ...or if we want to block, then do so here */
		if (down_interruptible(&dsm_wbs)) {
			/* something went wrong with wait */
			return -ERESTARTSYS;
		}
	}


	buff = (char *)kzalloc(count, GFP_KERNEL);
	if(!buff){
		DSM_LOG_ERR("dsm write malloc failed\n");
		goto out;
	}

	if(copy_from_user(buff, buf, count)){
		DSM_LOG_ERR("dsm write copy failed\n");
		goto out;
	}

	buff[count-1] = '\0';
	ptr = buff;
	while(*ptr){
		if(*ptr == '\n')
			break;
		ptr++;
	}

	/* get the client name */
	if(*ptr == '\n')
	{
		size = ptr - buff;
		size = (size < CLIENT_NAME_LEN) ? size : (CLIENT_NAME_LEN - 1);
		memcpy(client_name, buff, size);
		DSM_LOG_INFO( "%s client name is: %s \n", __func__ ,client_name );
		client = dsm_find_client(client_name);
		if( client )
		{
			/* found client name */
			DSM_LOG_DEBUG("dsm write find client - %s\n", client_name);

			ptr++;
			while(*ptr)
			{
				if(*ptr == '\n')
					break;
				ptr++;
			}

			/* get error number */
			if(*ptr == '\n')
			{
				memcpy(err_string,ptr-DSM_ERR_LEN,DSM_ERR_LEN);
				err_string[DSM_ERR_LEN] = '\0';
				sscanf(err_string,"%d",&err);
				DSM_LOG_INFO( "%s error number is: %d \n", __func__ ,err );
				/* judge if the err number is legal */
				if( (err >= DMS_ERR_NUM_MIN ) && (err < DMS_ERR_NUM_MAX) )
				{
					/* report the error */
					dsm_client_copy(client, ptr+1, (count - (ptr+1-buff)));
					dsm_client_notify(client, err);
				}
				else
				{
					DSM_LOG_ERR("dsm write err number is not legal! err:%d\n", err);
				}
			}
		}
		else
		{
			DSM_LOG_INFO("dsm write can't find client - %s\n", client_name);
		}
	}
	else
	{
		DSM_LOG_ERR("dsm write can't find client name\n");
	}

out:
	if(buff)
		kfree(buff);
	DSM_LOG_DEBUG("%s exit\n", __func__);
	/* release the write buffer and wake anyone who's waiting for it */
	up(&dsm_wbs);
	return count;
}

/* poll function, noblock */
static unsigned int dsm_poll(struct file *file, poll_table *wait)
{
	struct dsm_client *client = file->private_data;
	unsigned int mask = 0;

	DSM_LOG_DEBUG("%s enter\n",__func__);
	if(!client){
		DSM_LOG_ERR("dsm can't poll without client\n");
		goto out;
	}
	DSM_LOG_DEBUG("client name :%s\n", client->client_name);
	poll_wait(file, &client->waitq, wait);
	/* if buffer is ready, return POLLIN | POLLRDNORM, means date can be read */
	if(test_bit(CBUFF_READY_BIT, &client->buff_flag))
		mask = POLLIN | POLLRDNORM;

out:
	DSM_LOG_DEBUG("%s exit, mask:%d\n",__func__, mask);
	return mask;
}

/* sysfs open file */
static int dsm_open(struct inode *inode, struct file *file)
{
	DSM_LOG_DEBUG("%s enter\n",__func__);
	file->private_data = NULL;
	DSM_LOG_DEBUG("%s exit\n",__func__);
	return 0;
}

/* sysfs close file */
static int dsm_close(struct inode *inode, struct file *file)
{
	struct dsm_client *client = file->private_data;

	DSM_LOG_DEBUG("%s enter\n",__func__);
	if(client)
		dsm_unbind_client(client);
	DSM_LOG_DEBUG("%s exit\n",__func__);
	return 0;
}

/* io control function */
static long dsm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct dsm_client *client = (struct dsm_client *)file->private_data;
	long ret = 0;
	int error = 0;
	char buff[CLIENT_NAME_LEN]={0};
	struct dsm_extern_client tmp_ext_client;

	DSM_LOG_DEBUG("%s enter,\n",__func__);

	switch (cmd) {
		/* get client count */
		case DSM_IOCTL_GET_CLIENT_COUNT:
			mutex_lock(&g_dsm_server.mtx_lock);
			error = g_dsm_server.client_count;
			mutex_unlock(&g_dsm_server.mtx_lock);
			DSM_LOG_INFO("client count :%d\n", error);
			ret = copy_int_to_user(argp, error);
			break;
		/* bind client, set file private data as the client data  */
		case DSM_IOCTL_BIND:
			if (copy_from_user(buff, argp, CLIENT_NAME_LEN)){
				DSM_LOG_ERR("copy from user failed\n");
				ret = -EFAULT;
			}else{
				DSM_LOG_DEBUG("try bind client %s\n", buff);
				client = dsm_find_client(buff);
				if(client){
					dsm_bind_client(client);	// no use
					file->private_data = (void *)client;
				}
				else{
					DSM_LOG_ERR("dsm find client %s failed\n", buff);
					ret = -ENXIO;
				}
			}
			break;
		/* get client's error status */
		case DSM_IOCTL_POLL_CLIENT_STATE:
			if(client && client->cops && client->cops->poll_state){
				error = client->cops->poll_state();
				DSM_LOG_INFO("poll %s state result :%d\n", client->client_name, error);
				ret = copy_int_to_user(argp, error);
			}
			else{
				DSM_LOG_ERR("dsm client not bound or poll not support\n");
				ret = -ENXIO;
			}
			break;
		/* force dump client's error */
		case DSM_IOCTL_FORCE_DUMP:
			if (copy_from_user(buff, argp, UINT_BUF_MAX)){
				DSM_LOG_ERR("copy from user failed\n");
				ret = -EFAULT;
			}else{
				if(client && client->cops && client->cops->dump_func){
					if(!dsm_client_ocuppy(client)){
						client->error_no = dsm_atoi(buff);
						client->used_size = client->cops->dump_func(client->error_no,(void *)client->dump_buff, (int)client->buff_size);
						set_bit(CBUFF_READY_BIT, &client->buff_flag);
					}
					else{
						DSM_LOG_INFO("client %s's buff ocuppy failed\n", client->client_name);
						ret = -EBUSY;
					}
				}
				else{
					DSM_LOG_ERR("dsm client not bound or dump not support\n");
					ret = -ENXIO;
				}
			}
			break;
		/* get client's error number */
		case DSM_IOCTL_GET_CLIENT_ERROR:
			if(client)
				ret = copy_int_to_user(argp, client->error_no);
			else{
				DSM_LOG_ERR("dsm find client failed\n");
				ret = -ENXIO;
			}
			break;
		case DSM_IOCTL_REGISTER_EXTERN_CLIENT:
			if (copy_from_user(&tmp_ext_client, (struct dsm_extern_client*)arg, sizeof(struct dsm_extern_client))){
				return -EFAULT;
			}
			dsm_register_extern_client(&tmp_ext_client);

			break;
		default:
			DSM_LOG_ERR("unknown ioctl command :%d\n", cmd);
			break;
	}

	DSM_LOG_DEBUG("%s exit\n",__func__);
	return ret;
}


static const struct file_operations dsm_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= dsm_ioctl,
	.open		= dsm_open,
	.release		= dsm_close,
	.read			= dsm_read,
	.write		= dsm_write,
	.poll			= dsm_poll,
};

static struct dsm_dev dsm_pub_clients[] = {
	[0] = {
		.name = "smartpa",
		.fops = NULL,
		.buff_size = 1024,
	},
	[1] = {
		.name = "dsm_bluetooth",
		.fops = NULL,
		.buff_size = 1024,
	},
	[2] = {
		.name = "sdcard_vold",
		.fops = NULL,
		.buff_size = 1024,
	  },
};

/* registe public client */
static void __init dsm_register_public_client(void)
{
	int index;
	struct dsm_client *client = NULL;

	for(index = 0; index < ARRAY_SIZE(dsm_pub_clients); index++){
		client = dsm_register_client(&dsm_pub_clients[index]);
		if(client)
			DSM_LOG_DEBUG("register %d public client - %s success\n", index,
				dsm_pub_clients[index].name);
		else
			DSM_LOG_ERR("register %d public client - %s failed\n", index,
				dsm_pub_clients[index].name);
	}
}

static struct miscdevice dsm_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "dsm",
	.fops		= &dsm_fops,
};

/* dsm module init */
static int __init dsm_init(void)
{
	int ret = -EIO;

	memset(&g_dsm_server, 0, sizeof(struct dsm_server));
	g_dsm_server.server_state = DSM_SERVER_UNINIT;
	mutex_init(&g_dsm_server.mtx_lock);

	g_dsm_server.dsm_wq = create_singlethread_workqueue("dsm_wq");
	if (IS_ERR(g_dsm_server.dsm_wq)){
		DSM_LOG_ERR("alloc workqueue failed\n");
		goto out;
	}

	INIT_WORK(&dsm_work, dsm_work_func);

	ret = misc_register(&dsm_miscdev);
	if(ret){
		DSM_LOG_ERR("misc register failed\n");
		goto out;
	}

	/* set server status as ready, client can registe */
	g_dsm_server.server_state = DSM_SERVER_INITED;

	/* registe public client */
	dsm_register_public_client();

	/* init write semaphore for write func*/
	sema_init(&dsm_wbs, 1);

out:
	DSM_LOG_INFO("%s called, ret %d\n", __func__, ret);
	return ret;
}
subsys_initcall(dsm_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Device state monitor");

