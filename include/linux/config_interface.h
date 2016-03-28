#ifndef CONFIG_INTERFACE_H
#define CONFIG_INTERFACE_H
#include <linux/of.h>
#include <linux/string.h>

#define DT_PROP_ERROR   1
#define DT_PROP_SUUCESS 0

int get_product_name(char* product_name, int name_len);
int get_hardware_ver(char* hardware_ver, int name_len);
int get_software_ver(char* hardware_ver, int name_len);

int set_sbl1_ver_to_appinfo(void);
int set_hardware_ver_to_appinfo(void);
int set_appboot_ver_to_appinfo(void);
int set_product_name_to_appinfo(void);

int set_software_ver_to_appinfo(void);

#endif
