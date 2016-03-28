#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/stat.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/kprobes.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <asm/barrier.h>
#include <linux/platform_device.h>
#include <linux/of_fdt.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/huawei_reset_detect.h>
#include <linux/of_address.h>

#define DEBUG 1

#define hwlog_debug(fmt, ...) \
         do { if (DEBUG) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__); }while(0)

typedef enum{
	HWLOG_IMEM,
	HWLOG_DDR_MEM
}MEM_TYPE;

struct hwlog_mem_region{
	const char name[20];
	u64 phy_addr;
	u64 mem_size;
	void __iomem * addr;
	MEM_TYPE mem_type;
};

/*firstly, the following memory should be reserved at msm8939-common.dtsi*/
struct hwlog_mem_region hwlog_ddr_region[] = {
	{"hwlog_tz_mem", 0x000000008c700000, 0x100000, NULL, HWLOG_DDR_MEM},/*tz log memory region*/
    {"hw_nff_log_mem", 0x000000008c800000, 0x100000, NULL, HWLOG_DDR_MEM},/*nff log memory region*/
    {"unused", 0x000000008c900000, 0x200000, NULL, HWLOG_DDR_MEM},/*unused*/
};

static int hwlog_open(struct inode *inode, struct file *file)
{
	hwlog_debug("%s enter\n", __func__);
	return 0;
}

static int hwlog_release(struct inode *inode, struct file *file)
{
	hwlog_debug("%s enter\n", __func__);
	return 0;
}

static ssize_t hwlog_read(struct file *file, char __user *buf, size_t count,
			     loff_t *pos)
{
	hwlog_debug("%s enter\n", __func__);
	return 0;
}

void hwlog_vma_open(struct vm_area_struct *vma)
{
	hwlog_debug("hwlog map open\n");
} 

void hwlog_vma_close(struct vm_area_struct *vma)
{
	hwlog_debug("hwlog map close\n");
}

static const struct vm_operations_struct hwlog_mmap_ops = {
	.open = hwlog_vma_open,
	.close = hwlog_vma_close,
};

#if 0
void hwlog_mem_debug(void){

	hwlog_debug("++++++hwlog debug tz mem:\n");
	hwlog_ddr_region[0].addr = ioremap(hwlog_ddr_region[0].phy_addr,
			hwlog_ddr_region[0].mem_size);

	hwlog_debug("%s addr after map: %p\n", hwlog_ddr_region[0].name,
			hwlog_ddr_region[0].addr);

	if(hwlog_ddr_region[0].addr){
		memcpy((char*)hwlog_ddr_region[0].addr, "tz_log_begin", 12);
		memcpy(((char*)hwlog_ddr_region[0].addr + 0x400000 -16), "11111 tz_log_end", 16);
		iounmap(hwlog_ddr_region[0].addr);
		hwlog_debug("unmap in init\n");
	}
	hwlog_debug("-------hwlog debug tz mem:\n");
}
#endif

static int hwlog_memory_map_perm_check(struct vm_area_struct *vma)
{
	int i = 0;
	unsigned long end = 0;
	unsigned long start= 0;
	unsigned long map_start = (vma->vm_pgoff<<PAGE_SHIFT);
	unsigned long map_end = map_start + vma->vm_end - vma->vm_start;

	for(i = 0; i < ARRAY_SIZE(hwlog_ddr_region); i++)
	{
		end = hwlog_ddr_region[i].phy_addr + hwlog_ddr_region[i].mem_size;
		start = hwlog_ddr_region[i].phy_addr;

		/*vma must be in the range of hwlog_ddr_region[i]*/
		if((map_start >= start)&&(map_end <= end))
		{
			pr_info("hwlog %lx--%lx map check permission sucess\n", map_start,
					map_end);
			return 0;
		}
	}

	pr_err("hwlog start=0x%lx, end=%lx is out of range\n", 
			map_start, map_end);
	return -1;
}

static int hwlog_mmap(struct file *filp,
				  struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;

	//hwlog_mem_debug();

	if(hwlog_memory_map_perm_check(vma)){
		pr_err("hwlog map out of memory range\n");
		return -EPERM;
	}

	hwlog_debug("hwlog start=0x%lx, size=%zx, vm_pgoff=0x%lx\n", 
			vma->vm_start, size, vma->vm_pgoff);

	vma->vm_ops = &hwlog_mmap_ops;

	/* Remap-pfn-range will mark the range VM_IO */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    size,
			    vma->vm_page_prot)) {
		pr_err("hwlog remap pfn range failed\n");
		return -EAGAIN;
	}

	hwlog_vma_open(vma);
	hwlog_debug("hwlog mmap sucess\n");
	return 0;
}

static const struct file_operations hwlog_fops= {
	.owner = THIS_MODULE,
	.open = hwlog_open,
	.release = hwlog_release,
	.read = hwlog_read,
	.mmap = hwlog_mmap,
};

static struct miscdevice hwlog_mem_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hwlog_mem",
	.fops = &hwlog_fops
};

static int create_imem_node(void)
{
	hwlog_debug("create imem node\n");
	return 0;
}

static int create_ddr_mem_miscdev(void)
{

	return 0;
}

static int __init hw_log_mem_init(void)
{
    int ret = 0;


	hwlog_debug("create hwlog devices sucess\n");

	if(misc_register(&hwlog_mem_miscdev)){
		pr_err("hwlog_mem: register misc device failed\n");
		return 0;
	}

	create_imem_node();
	create_ddr_mem_miscdev();

    return ret;
}

static void __exit hw_log_mem_exit(void)
{
	misc_deregister(&hwlog_mem_miscdev);
	pr_info("hwlog mem module exit\n");
}

module_init(hw_log_mem_init);
module_exit(hw_log_mem_exit);
MODULE_LICENSE(GPL);
