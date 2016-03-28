/* add first time by yanghaizhou 20140527 */
#ifndef _HUAWEI_TP_POWER_H
#define _HUAWEI_TP_POWER_H
#include <linux/device.h>

extern int hw_tp_power_init(struct device *);
extern void hw_tp_power_release(void);

extern void hw_tp_power_on(void);
extern void hw_tp_power_off(void);

extern bool hw_tp_power_is_ready(void);
extern void hw_tp_power_get_info(char *, ssize_t);

#endif

