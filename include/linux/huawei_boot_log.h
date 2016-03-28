#ifndef __HUAWEI_BOOT_LOG_H__
#define __HUAWEI_BOOT_LOG_H__


#define SHARED_IMEM_OEM_BASE (0x08600A94)
#define HUAWEI_BOOT_LOG_OEM_OFFSET 24

#define HUAWEI_BOOT_LOG_ADDR (SHARED_IMEM_OEM_BASE + HUAWEI_BOOT_LOG_OEM_OFFSET)

#define MAGIC_NUMBER_SIZE (4)
#define CHECKSUM_SIZE (4)
#define BOOT_LOG_INFORMATION_START_ADDR (HUAWEI_BOOT_LOG_ADDR + MAGIC_NUMBER_SIZE + CHECKSUM_SIZE)
#define HUAWEI_BOOT_LOG_SIZE (0x100000)

#define SBL_MASK             (1<<0)   /* bit 0 for sbl log */
#define ABOOT_MASK           (1<<1)   /* bit 1 for aboot log */
#define KERNEL_MASK          (1<<2)   /* bit 2 for kernel log */

#define HUAWEI_BOOT_MAGIC_NUMBER   (0xACBEFCDE)

struct boot_log_struct {
	uint32_t sbl_addr;
	uint32_t aboot_addr;
	uint32_t kernel_addr;
	uint32_t sbl_log_size;
	uint32_t aboot_log_size;
	uint32_t kernel_log_size;
	uint32_t boot_process_mask;
};

#endif
