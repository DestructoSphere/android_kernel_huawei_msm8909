#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/export.h>
#include <misc/app_info.h>
#include <linux/slab.h>

#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/kthread.h>
#endif
#ifdef CONFIG_HUAWEI_KERNEL
#include <soc/qcom/smsm.h>

#define SAMSUNG_ID    1
#define ELPIDA_ID     3
#define HYNIX_ID      6

#define DDR_SIZE_ORDER_MAX 35   //32Gbit , 4GB
#define DDR_SIZE_ORDER_MIN 30   //1Gbit,125MB
#define DDR_SIZE_MAX (0x1<<(DDR_SIZE_ORDER_MAX - 30))
#define DDR_SIZE_MIN (0x1<<(DDR_SIZE_ORDER_MIN - 30))

/** DDR types. */
typedef enum
{
  DDR_TYPE_LPDDR1,           /**< Low power DDR1. */
  DDR_TYPE_LPDDR2 = 2,       /**< Low power DDR2  set to 2 for compatibility*/
  DDR_TYPE_PCDDR2,           /**< Personal computer DDR2. */
  DDR_TYPE_PCDDR3,           /**< Personal computer DDR3. */

  DDR_TYPE_LPDDR3,           /**< Low power DDR3. */

  DDR_TYPE_RESERVED,         /**< Reserved for future use. */
  DDR_TYPE_UNUSED = 0x7FFFFFFF  /**< For compatibility with deviceprogrammer(features not using DDR). */
} DDR_TYPE;
#endif

struct info_node
{
    char name[APP_INFO_NAME_LENTH];
    char value[APP_INFO_VALUE_LENTH];
    struct list_head entry;
};

static LIST_HEAD(app_info_list);
static DEFINE_SPINLOCK(app_info_list_lock);

int app_info_set(const char * name, const char * value)
{
    struct info_node *new_node = NULL;
    int name_lenth = 0;
    int value_lenth = 0;

    if(WARN_ON(!name || !value))
        return -1;

    name_lenth = strlen(name);
    value_lenth = strlen(value);

    new_node = kzalloc(sizeof(*new_node), GFP_KERNEL);
    if(new_node == NULL)
    {
        return -1;
    }

    memcpy(new_node->name,name,((name_lenth > (APP_INFO_NAME_LENTH-1))?(APP_INFO_NAME_LENTH-1):name_lenth));
    memcpy(new_node->value,value,((value_lenth > (APP_INFO_VALUE_LENTH-1))?(APP_INFO_VALUE_LENTH-1):value_lenth));

    spin_lock(&app_info_list_lock);
    list_add_tail(&new_node->entry,&app_info_list);
    spin_unlock(&app_info_list_lock);

    return 0;
}

EXPORT_SYMBOL(app_info_set);


static int app_info_proc_show(struct seq_file *m, void *v)
{
    struct info_node *node;

    list_for_each_entry(node,&app_info_list,entry)
    {
        seq_printf(m,"%-32s:%32s\n",node->name,node->value);
    }
    return 0;
}

static int app_info_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, app_info_proc_show, NULL);
}

static const struct file_operations app_info_proc_fops =
{
    .open		= app_info_proc_open,
    .read		= seq_read,
    .llseek		= seq_lseek,
    .release	= single_release,
};

#ifdef CONFIG_HUAWEI_KERNEL
/*
    Function to read the SMEM to get the lpDDR name
*/
void export_ddr_info(unsigned int ddr_vendor_id,unsigned int ddr_size, unsigned int ddr_type )
{
    char ddr_info_all[APP_INFO_VALUE_LENTH];
    char * ddr_info = NULL;
    char *SAMSUNG_DDR = "SAMSUNG";
    char *ELPIDA_DDR = "MICRON";
    char *HYNIX_DDR  = "HYNIX";
	char ddr_size_info[8];
    char *ddr_type_info;

	switch (ddr_vendor_id)
	{
		case SAMSUNG_ID:
		{
			ddr_info = SAMSUNG_DDR;
			break;
		}
		case ELPIDA_ID:
		{
			ddr_info = ELPIDA_DDR;
			break;
		}
		case HYNIX_ID:
		{
			ddr_info = HYNIX_DDR;
			break;
		}
		default:
		{
			ddr_info = "UNKNOWN";
			break;
		}
	}

	if( ddr_size >= DDR_SIZE_MIN && ddr_size <=DDR_SIZE_MAX ) //should be less than 4G
	{
	    snprintf( ddr_size_info, 8, "%dGbit", ddr_size );
	}
	else
	{
	    snprintf( ddr_size_info , 8 , "UNKNOWN" );
	}

	switch(ddr_type)
	{
		case DDR_TYPE_LPDDR1:
		    ddr_type_info = "LPDDR1";
		    break;
		case DDR_TYPE_LPDDR2:
		    ddr_type_info = "LPDDR2";
		    break;
		case DDR_TYPE_LPDDR3:
		    ddr_type_info = "LPDDR3";
		    break;
		default:
		    ddr_type_info = "UNKNOWN";
		    break;
	}

	snprintf(ddr_info_all,APP_INFO_VALUE_LENTH-1, "%s %s %s", ddr_info,ddr_size_info,ddr_type_info );
    /* Set the vendor name in app_info */
    if (app_info_set("ddr_vendor", ddr_info_all))
        pr_err("Error setting DDR vendor info\n");

    /* Print the DDR Name in the kmsg log */
    pr_err("DDR VENDOR is : %s", ddr_info_all);

    return;
}


void app_info_print_smem(void)
{
    unsigned int ddr_vendor_id = 0;
    /* read share memory and get DDR ID */
    smem_exten_huawei_paramater *smem = NULL;
	unsigned int ddr_size0,ddr_size1;
    unsigned int ddr_size=0;
    unsigned int ddr_type;

    smem = smem_alloc(SMEM_ID_VENDOR1, sizeof(smem_exten_huawei_paramater),
						  0,
						  SMEM_ANY_HOST_FLAG);
    if(NULL == smem)
    {
        /* Set the vendor name in app_info */
        if (app_info_set("ddr_vendor", "UNKNOWN"))
            pr_err("Error setting DDR vendor name to UNKNOWN\n");
        pr_err("%s: SMEM Error, READING DDR VENDOR NAME", __func__);
        return;
    }

    ddr_vendor_id = smem->lpddrID;
    ddr_vendor_id &= 0xff;
	ddr_size1 = ( smem->lpddrID >> 8 ) & 0xFF;
    ddr_size0 = ( smem->lpddrID >> 16 ) & 0xFF;

    if(ddr_size0<= DDR_SIZE_ORDER_MAX && ddr_size0>= DDR_SIZE_ORDER_MIN )
        ddr_size = 0x1<<(ddr_size0-30);

    if(ddr_size1<= DDR_SIZE_ORDER_MAX && ddr_size0>= DDR_SIZE_ORDER_MIN )
        ddr_size += 0x1<<(ddr_size1-30);

    ddr_type = ( smem->lpddrID >> 24 ) & 0xFF;

    printk(KERN_ERR "ddr_info %x,%d,%d,%d", smem->lpddrID,ddr_size1,ddr_size0, ddr_size );

    export_ddr_info(ddr_vendor_id, ddr_size, ddr_type );

    return;
}
#endif
#ifdef CONFIG_HUAWEI_KERNEL
extern int usb_update_thread(void *__unused);
static struct task_struct *update_task;
#endif
static int __init proc_app_info_init(void)
{
    proc_create("app_info", 0, NULL, &app_info_proc_fops);
#ifdef CONFIG_HUAWEI_KERNEL
    app_info_print_smem();
#endif
#ifdef CONFIG_HUAWEI_KERNEL
    update_task = kthread_run(usb_update_thread, NULL, "usb_update");
    if (IS_ERR(update_task))
    {
        printk("usb_update thread run error \n");
    }
#endif
    return 0;
}

module_init(proc_app_info_init);
