
#include <linux/workqueue.h>

#include "synaptics_dsx_i2c.h"

#define SYNAPTICS_STATUS_REG 0x0013
#define SYNAPTICS_ESD_RETRY_TIMES 3

#define SYNAPTICS_HOLSTER_INFO_LENGTH (64) 

struct synaptics_esd {
    atomic_t esd_check_status;
    atomic_t irq_status;
    struct workqueue_struct * esd_work_queue;
    struct delayed_work esd_work;
    unsigned int esd_delay_time;
    unsigned int esd_tirg_count;
};

typedef enum esd_status {
    ESD_CHECK_NOT_READY,
    ESD_CHECK_STOPED,
    ESD_CHECK_START,
} ESD_STATUS;

typedef int (* synaptics_read)(struct synaptics_rmi4_data *, unsigned short, unsigned char *, unsigned short);
typedef int (* synaptics_write)(struct synaptics_rmi4_data *, unsigned short, unsigned char *, unsigned short);

int synaptics_dsx_esd_init(struct synaptics_rmi4_data *rmi4_data, synaptics_read read, synaptics_write write);
int synaptics_dsx_esd_start(void);

int synaptics_dsx_esd_stop(void);
void synaptics_dsx_esd_suspend(void);
void synaptics_dsx_esd_resume(void);

ssize_t synaptics_dsm_record_esd_err_info( int err_num );


