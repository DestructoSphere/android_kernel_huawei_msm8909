/*
 * drivers/staging/android/huawei_logger.c
 *
 * Huawei Logging Subsystem
 *
 * Albin Joy <albin.joy@huawei.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "logger: " fmt

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/aio.h>
#include "logger.h"
#include <asm/ioctls.h>
#include <linux/io.h>

#define SHARED_IMEM_OEM_BASE 0x08600A94
#define SHARED_IMEM_OEM_LOGGER_INFO_BASE (SHARED_IMEM_OEM_BASE + 80)
#define LOGGER_INFO_MAX_COUNT            8
#define FAILED                           1
#define SUCCESS                          0
#define LOGGER_NAME_MAX_SIZE             16
#define LOGGER_INFO_LIST_HEADER_SIZE     24
#define LOGGER_INFO_LIST_HEADER          "HUAWEI_LOGGER_#$&#END"
#define LOGGER_INFO_LIST_TAIL_SIZE       8
#define LOGGER_INFO_LIST_TAIL            "END"
#define LOGGER_IMEM_INFO_HEADER_SIZE     8
#define LOGGER_IMEM_INFO_HEADER          "HD_IMEM"
#define LOGGER_KMSG_LOG_NAME               "log_kmsg"

/**
 * Log parser is a tool which used to parse the logcat logs like main, radio, events,
 * system and exceptions from ramdump.
 */

/**
 * struct log_name - logger informations which will be used in the log parser
 * @log_name:	Name of the Logger
 * @buffer_phy_addr:	Physical start address of the log buffer
 * @buffer_size:	Size of the log buffer
 * @head_offset:	Offset in the log buffer from where the reader will start reading
 */
struct logger_info {
	char log_name[LOGGER_NAME_MAX_SIZE];
	unsigned int buffer_phy_addr;
	unsigned int buffer_size;
	unsigned int head_offset;
};

/**
 * struct logger_info_list - Maintain the list of logger info which is created.
 * @header:	Identifier for the start of the logger info list
 * @count:	Number of logger added into the list
 * @logger:	List of logger info
 * @tail:	Identifier for the end of the logger into.
 */
struct logger_info_list {
	char header[LOGGER_INFO_LIST_HEADER_SIZE];
	unsigned int count;
	struct logger_info logger[LOGGER_INFO_MAX_COUNT];
	char tail[LOGGER_INFO_LIST_TAIL_SIZE];
};

/**
 * struct logger_imem_info - Maintain the list of logger info which is created.
 * @header:	Identifier for the start of the imem logger into
 * @log_info_list_phy_addr:	Physical address of the logger info list
 */
struct logger_imem_info {
	char header[LOGGER_IMEM_INFO_HEADER_SIZE];
	unsigned int log_info_list_phy_addr;
};

/**
 * log_info_list:	Informations required for log-parser tool, from each logger created.
 * The structure contain a header string of 24 byte, an integer count holds the number of logger 
 added into the list, list of logger_info and a tail string of 8 bytes.
 *
 * log_info_list memory is allocated using "kzalloc" interface. Its allocate contiguous 
 * memory address in the DDR.
 * memory layout diagram:
 *		=============<-DDR memory space start point
 *		|				|
 *		|				|
 *		|				|
 *		|				|
 *		-----------------<-log_info_list is stored in a random physical start address
 *		|				|
 *		|				|
 *		-----------------
 *		|				|
 *		|				|
 *		=============
 * Header of the "log_info_list" is hardcoded with string "HUAWEI_LOGGER_#$&#END"
 * and tail is with string "END"
 */
struct logger_info_list *log_info_list;

extern void* get_kernel_log_buf(void);
extern unsigned int get_kernel_log_buf_len(void);

void save_head_off_in_logger_info(struct logger_log *log);
int populate_logger_info_list(char* log_name,
	unsigned int buffer_phy_addr, int size);

/*
 * save_head_off_in_logger_info - Save the head value of the logger in logger_info when
 * ever its changed.
 *
 * head is the log buffer offset from where readers have to start reading.
 * logger_info is used by log parser.
 */
void save_head_off_in_logger_info (struct logger_log *log) {
	size_t index = 0;
	size_t len = 0;

	if (log_info_list == NULL) {
		pr_err("HUAWEI_LOGGER, "
			"Failed to Save head offset, logger info list is NULL\n");
		return;
	}

	for(index = 0; index < log_info_list->count; index++) {
		struct logger_info *log_info = NULL;

		log_info = &log_info_list->logger[index];
		if (log_info == NULL) {
			pr_err("HUAWEI_LOGGER, Logger info is NULL at index: %d\n", index);
			return;
		}

		len = strlen(log->misc.name);
		if (!strncmp(log->misc.name, log_info->log_name, len)) {
			log_info->head_offset = log->head;
			break;
		}
	}
}

/*
 * populate_logger_info_list - Add the logger informations into logger_info list.
 *
 * This function will be called when the logger is created.
 * logger_info->head_offset will be modified later.
 * logger_info is used by log parser.
 */
int populate_logger_info_list( char *log_name,
    unsigned int buffer_phy_addr, int size) {
	struct logger_info *log_info = NULL;
	int ret                      = SUCCESS;

	if(log_info_list == NULL) {
		ret = FAILED;
		pr_err("HUAWEI_LOGGER, Failed to populate logger "
			"info list for logger: %s, logger info list is NULL\n", log_name);
		return ret;
	}

	if (log_info_list->count >= LOGGER_INFO_MAX_COUNT) {
		pr_err("HUAWEI_LOGGER, Logger info count is exceeding the MAX limit\n");
		ret = FAILED;
		return ret;
	}

	log_info = &(log_info_list->logger[log_info_list->count]);
	if (log_info != NULL) {
		strncpy(log_info->log_name, log_name, sizeof(log_info->log_name));
		log_info->buffer_phy_addr = buffer_phy_addr;
		log_info->buffer_size = size;
		log_info->head_offset = 0;
	} else {
		pr_err("HUAWEI_LOGGER, Failed to get the logger info element\n");
	}

	log_info_list->count++;

	return ret;
}

/**
 * populate_kmsg_info_into_logger_info_list - Add the kmsg buffer inforations into
 * logger info list.
 *
 * It creats a logger info with kmsg buffer informations and add the logger info into the list.
 * kmsg buffer and its size are read from printk.
 */
void populate_kmsg_info_into_logger_info_list(void) {
	void* kmsg_buf;
	unsigned int kmsg_buf_len;

	unsigned int kmsg_buf_phy_addr;

	/*get kmsg buffer from printk*/
	kmsg_buf = get_kernel_log_buf();
	/*get kmsg buffer length from printk*/
	kmsg_buf_len = get_kernel_log_buf_len();

	kmsg_buf_phy_addr = virt_to_phys(kmsg_buf);

	pr_info("HUAWEI_LOGGER, name:%s size=%d phy:0x%x\n",
		LOGGER_KMSG_LOG_NAME, kmsg_buf_len,
		kmsg_buf_phy_addr);
	populate_logger_info_list(LOGGER_KMSG_LOG_NAME,
		kmsg_buf_phy_addr, kmsg_buf_len);
}

/*
 *save_logger_info_in_IMEM - Save the logger info list address in the IMEM
 *
 * This function will be called before start creating the logger.
 * log_info_list have list of all the logger information.
 *
 * To store the "logger_imem_info", SHARED_IMEM_OEM_BASE memory area is used.
 * Memory diagram as follows:
 *				============= <- SHARED_IMEM_BASE 
 *				|				|
 *				|				|
 *				|				|
 *				-----------------<- SHARED_IMEM_OEM_BASE 
 *				|				|
 *				|				|
 *				.............................<- SHARED_IMEM_OEM_LOGGER_INFO
 *				|				|
 *				-----------------
 *				|				|
 *				|				|
 *				|				|
 *				=============
 * In 8909, SHARED_IMEM_OEM_BASE memory size is 100bytes.
 * This memory space is used for storing some other oem info also.
 * So the from the 80th byte of the SHARED_IMEM_OEM_BASE space, "logger_imem_info" is stored.
 * Size of the "logger_imem_info" is 12 bytes. So the 12 bytes has mapped using ioremap and 
 * stored the data in the shared imem.
 */
void save_logger_info_in_IMEM(void) {
	uint32_t *imem_info_start_addr     = NULL;
	uint32_t log_info_list_phy_addr    = 0;
	struct logger_imem_info* imem_info = NULL;

	imem_info_start_addr = (uint32_t *)ioremap_nocache(SHARED_IMEM_OEM_LOGGER_INFO_BASE,
		sizeof(struct logger_imem_info));

	if (imem_info_start_addr != NULL) {
		memset(imem_info_start_addr, 0, sizeof(struct logger_imem_info));
		imem_info = (struct logger_imem_info*)imem_info_start_addr;

		log_info_list = kzalloc(sizeof(struct logger_info_list), GFP_KERNEL);
		if (log_info_list != NULL) {
			log_info_list_phy_addr = virt_to_phys(log_info_list);
			strncpy(imem_info->header, LOGGER_IMEM_INFO_HEADER,
				sizeof(imem_info->header));
			imem_info->log_info_list_phy_addr = log_info_list_phy_addr;
			pr_info("HUAWEI_LOGGER, logger info "
				"list start physical address: %x\n", log_info_list_phy_addr);

			strncpy(log_info_list->header, LOGGER_INFO_LIST_HEADER,
				sizeof(log_info_list->header));
			strncpy(log_info_list->tail, LOGGER_INFO_LIST_TAIL,
				sizeof(log_info_list->tail));
		} else {
			pr_err("HUAWEI_LOGGER, "
				"Failed to allocate memory for log_info_list\n");
		}

		iounmap(imem_info_start_addr);
	} else {
		pr_err("HUAWEI_LOGGER, ioremap failed for "
			"address: %x while allocating logcat buffers\n",
			SHARED_IMEM_OEM_LOGGER_INFO_BASE);
	}
}

