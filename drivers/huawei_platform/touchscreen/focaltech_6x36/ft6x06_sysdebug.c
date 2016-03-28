/*sysfs debug */
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include "ft6x06_ex_fun.h"
#include "ft6x06_ts.h"
#include "focaltech_log.h"

static struct mutex g_device_mutex;
static ssize_t ft6x06_tpfwver_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	mutex_lock(&g_device_mutex);

	if (ft6x06_read_reg(client, 0xa6, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE,
					"get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

	mutex_unlock(&g_device_mutex);

	return num_read_chars;
}

static ssize_t ft6x06_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t ft6x06_tprwreg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t ft6x06_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 regaddr = 0xff, regvalue = 0xff;
	u8 valbuf[5] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			tp_log_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		tp_log_err("%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		/*read register*/
		regaddr = wmreg;
		if (ft6x06_read_reg(client, regaddr, &regvalue) < 0)
			tp_log_err("Could not read the register(0x%02x)\n",	regaddr);
		else
			tp_log_info("the register(0x%02x) is 0x%02x\n",	regaddr, regvalue);
	} else {
		regaddr = wmreg >> 8;
		regvalue = wmreg;
		if (ft6x06_write_reg(client, regaddr, regvalue) < 0)
			tp_log_err("Could not write the register(0x%02x)\n", regaddr);
		else
			tp_log_info("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}

error_return:
	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t ft6x06_fwupgradeapp_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}


/*upgrade from app.bin*/
static ssize_t ft6x06_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	memset(fwname, 0, sizeof(fwname));

        snprintf(fwname, sizeof(fwname), "%s", buf);
        
	fwname[count - 1] = '\0';

	mutex_lock(&g_device_mutex);
	disable_irq(client->irq);

	fts_ctpm_fw_upgrade_with_app_file(client, fwname);

	enable_irq(client->irq);
	mutex_unlock(&g_device_mutex);

	return count;
}

static int ft6x06_read_project_code(struct i2c_client * client, char * pProjectCode)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32  j=0;
	u32  temp;
	u8 	packet_buf[4];
	u8  	auc_i2c_write_buf[10];
	int      i_ret;
	u8 is_5336_new_bootloader = 0;
	int rc =0;
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
    	/*********Step 1:Reset  CTPM *****/
    	/*write 0xaa to register 0xbc*/
	   	ft6x06_write_reg(client, 0xbc, FT_UPGRADE_AA);
		msleep(FT6X06_UPGRADE_AA_DELAY);
		
		 /*write 0x55 to register 0xbc*/
		ft6x06_write_reg(client, 0xbc, FT_UPGRADE_55);   
		msleep(FT6X06_UPGRADE_55_DELAY+20);   

		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		do{
			j++;
	  	i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);
	  	msleep(5);
	  }while(i_ret <= 0 && j<5);
	    /*********Step 3:check READ-ID***********************/   
		msleep(FT6X06_UPGRADE_READID_DELAY);
	   	auc_i2c_write_buf[0] = 0x90; 
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		rc = ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if(rc < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		if (reg_val[0] == FT6X06_UPGRADE_ID_1 
			&& reg_val[1] == FT6X06_UPGRADE_ID_2)
		{
    		tp_log_debug("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    		break;
		}
		else
		{
			tp_log_err("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	    	continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
    {
		return -EIO;
	}
    
	auc_i2c_write_buf[0] = 0xcd;
	rc = ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if(rc < 0)
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
	if (reg_val[0] > 4)
		is_5336_new_bootloader = 1;

	tp_log_debug("bootloader version:%d\n", reg_val[0]);

	/*read project code*/

	packet_buf[0] = 0x03;
	packet_buf[1] = 0x00;
	for (j=0;j<33;j++)
	{
		if (is_5336_new_bootloader)
			temp = 0x07d0 + j;
		else
			temp = 0x7820 + j;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;

		rc = ft6x06_i2c_Read(client, packet_buf, sizeof(packet_buf), pProjectCode+j, 1);
		if(rc < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		if (*(pProjectCode+j) == '\0')
			break;
		tp_log_debug("j = %d\n",j);
	}
	//if(strcmp(pProjectCode,"FTS0000P000") == 0)
	tp_log_debug("project code = %s \n", pProjectCode);
#if 1
	/*********reset the new FW***********************/
	tp_log_debug("%s %d\n",__func__,__LINE__);
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);  /*make sure CTP startup normally*/
	tp_log_debug("%s %d\n",__func__,__LINE__);
#endif
	return 0;
}
static ssize_t ft6x06_ftsgetprojectcode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	char projectcode[50]; 
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	tp_log_info("%s %d\n",__func__,__LINE__);
	memset(projectcode, 0, sizeof(projectcode));
	mutex_lock(&g_device_mutex);
	if(ft6x06_read_project_code(client, projectcode) < 0)
	{
		tp_log_debug("%s %d num_read_chars = %d\n",__func__,__LINE__,num_read_chars);
		num_read_chars = snprintf(buf, PAGE_SIZE, "get projcet code fail!\n");
		tp_log_debug("%s %d num_read_chars = %d\n",__func__,__LINE__,num_read_chars);
	}
	else
	{
		tp_log_debug("%s %d num_read_chars = %d\n",__func__,__LINE__,num_read_chars);
		num_read_chars = snprintf(buf, PAGE_SIZE, "projcet code = %s\n", projectcode);
		tp_log_debug("%s %d num_read_chars = %d\n",__func__,__LINE__,num_read_chars);
	}
	mutex_unlock(&g_device_mutex);
	return num_read_chars;
	
}
//upgrade from app.bin
static ssize_t ft6x06_ftsgetprojectcode_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
    return -EPERM;
}
/*sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO | S_IWUSR, ft6x06_tpfwver_show,
			ft6x06_tpfwver_store);


/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO | S_IWUSR, ft6x06_tprwreg_show,
			ft6x06_tprwreg_store);


/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO | S_IWUSR, ft6x06_fwupgradeapp_show,
			ft6x06_fwupgradeapp_store);

/*show project code
*example:cat ftsgetprojectcode
*/
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, ft6x06_ftsgetprojectcode_show, ft6x06_ftsgetprojectcode_store);

/*add your attr in here*/
static struct attribute *ft6x06_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,
	NULL
};

static struct attribute_group ft6x06_attribute_group = {
	.attrs = ft6x06_attributes
};

/*create sysfs for debug*/
int ft6x06_create_sysfs(struct i2c_client *client)
{
	int err;
	tp_log_info(" in ft6x06_create_sysfs\n");
	err = sysfs_create_group(&client->dev.kobj, &ft6x06_attribute_group);
	if (0 != err) {
		dev_err(&client->dev,
					 "%s() - ERROR: sysfs_create_group() failed.\n",
					 __func__);
		sysfs_remove_group(&client->dev.kobj, &ft6x06_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_device_mutex);
		pr_info("ft6x06:%s() - sysfs_create_group() succeeded.\n",
				__func__);
	}
	tp_log_info(" out ft6x06_create_sysfs\n");
	return err;
}

void ft6x06_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ft6x06_attribute_group);
	mutex_destroy(&g_device_mutex);
}
