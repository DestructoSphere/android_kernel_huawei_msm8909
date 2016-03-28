#ifndef __FT6X06_SYSDEBUG_H__
#define __FT6X06_SYSDEBUG_H__
int ft6x06_create_sysfs(struct i2c_client *client);

void ft6x06_release_sysfs(struct i2c_client *client);

#endif
