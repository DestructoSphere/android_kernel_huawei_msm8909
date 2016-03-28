/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"

#define FLASH_NAME "ti,lm3642"

#ifdef CONFIG_HUAWEI_DSM
#include "msm_camera_dsm.h"
#endif

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define LM3642_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define LM3642_DBG(fmt, args...)
#endif

//lm3642 flash flag register
#define FLASH_FLAG_REGISTER 0x0B


#define ENABLE_REGISTER 0x0A
#define MODE_BIT_MASK 0x03
#define MODE_BIT_STANDBY 0x00
#define MODE_BIT_INDICATOR 0x01
#define MODE_BIT_TORCH 0x02
#define MODE_BIT_FLASH 0x03
#define ENABLE_BIT_FLASH 0x20
#define ENABLE_BIT_TORCH 0x10

#define CURRENT_REGISTER 0x09
#define CURRENT_TORCH_MASK 0x70
#define CURRENT_TORCH_SHIFT 4
#define CURRENT_FLASH_MASK 0x0F
#define CURRENT_FLASH_SHIFT 0

#define FLASH_FEATURE_REGISTER 0x08
#define FLASH_TIMEOUT_MASK 0x07
#define FLASH_TIMEOUT_SHIFT 0

#define FLASH_CHIP_ID_MASK 0x07
#define FLASH_CHIP_ID 0x0

#define IVFM_MODE_REGISTER 0x01
#define EBABLE_BIT_IVFM    0x80

// Default flash high current, 8 corresponds to 843.75mA.
#define FLASH_HIGH_CURRENT 8 

#ifdef CONFIG_HUAWEI_DSM
#define LED_SHORT                    (1<<1)
#define LED_THERMAL_SHUTDOWN         (1<<2)
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3642_i2c_driver;

/*Enable IFVM and set it threshold to 3.2v ,also set flash ramp time to 512us */
static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	{ENABLE_REGISTER, MODE_BIT_STANDBY | EBABLE_BIT_IVFM},
	/* disable UVLO */
	{IVFM_MODE_REGISTER, 0x8C},
};

/*Enable IFVM and set it threshold to 3.2v ,also set flash ramp time to 512us */
//delete "set current action"
static struct msm_camera_i2c_reg_array lm3642_off_array[] = {
	{ENABLE_REGISTER, MODE_BIT_STANDBY | EBABLE_BIT_IVFM},
};

static struct msm_camera_i2c_reg_array lm3642_release_array[] = {
	{ENABLE_REGISTER, MODE_BIT_STANDBY | EBABLE_BIT_IVFM},
};

static struct msm_camera_i2c_reg_array lm3642_low_array[] = {
	{CURRENT_REGISTER, 0x40},//234mA
	{ENABLE_REGISTER, MODE_BIT_TORCH | EBABLE_BIT_IVFM},
};

static struct msm_camera_i2c_reg_array lm3642_high_array[] = {
	{CURRENT_REGISTER, 0x08},//843.75mA
	// The duration of high flash, 0x05 corresponds to 600ms.
	//{FLASH_FEATURE_REGISTER, 0x05},
	{FLASH_FEATURE_REGISTER, 0x0D},//frash ramp time 512us and flash timeout 600ms
	{ENABLE_REGISTER, MODE_BIT_FLASH | EBABLE_BIT_IVFM},
};

static struct msm_camera_i2c_reg_array lm3642_torch_array[] = {
	{CURRENT_REGISTER, 0x20},//140mA
	{ENABLE_REGISTER, MODE_BIT_TORCH | EBABLE_BIT_IVFM},
};


static const struct of_device_id lm3642_i2c_trigger_dt_match[] = {
	{.compatible = "ti,lm3642"},
	{}
};

MODULE_DEVICE_TABLE(of, lm3642_i2c_trigger_dt_match);
static const struct i2c_device_id lm3642_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value > LED_OFF) {
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_lm3642_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};

#ifdef CONFIG_HUAWEI_DSM
void camera_report_flash_dsm_err(int flash_reg, int rc_value)
{
	ssize_t len = 0;

	memset(camera_dsm_log_buff, 0, MSM_CAMERA_DSM_BUFFER_SIZE);

	if (rc_value < 0)
	{
		pr_err("%s. i2c read error, no dsm error\n",__func__);
	    return;
	}

	/* camera record error info according to err type */
	if (flash_reg & LED_THERMAL_SHUTDOWN)
	{
	    len += snprintf(camera_dsm_log_buff+len, MSM_CAMERA_DSM_BUFFER_SIZE-len, "[msm_camera]Flash temperature reached thermal shutdown value.\n");
	    if ((len < 0) || (len >= MSM_CAMERA_DSM_BUFFER_SIZE -1))
	    {
	      pr_err("%s %d. write camera_dsm_log_buff error\n",__func__, __LINE__);
	      return ;
	    }
	    camera_report_dsm_err(DSM_CAMERA_LED_FLASH_OVER_TEMPERATURE, flash_reg, camera_dsm_log_buff);
	}
	else if (flash_reg & LED_SHORT)
	{
	    len += snprintf(camera_dsm_log_buff+len, MSM_CAMERA_DSM_BUFFER_SIZE-len, "[msm_camera]Flash led short detected.\n");
	    if ((len < 0) || (len >= MSM_CAMERA_DSM_BUFFER_SIZE -1))
	    {
	      pr_err("%s %d. write camera_dsm_log_buff error\n",__func__, __LINE__);
	      return ;
	    }
	    camera_report_dsm_err(DSM_CAMERA_LED_FLASH_CIRCUIT_ERR, flash_reg, camera_dsm_log_buff);
	}
	else
	{
	    len += snprintf(camera_dsm_log_buff+len, MSM_CAMERA_DSM_BUFFER_SIZE-len, "[msm_camera]Flash led voltage error.\n");
	    if ((len < 0) || (len >= MSM_CAMERA_DSM_BUFFER_SIZE -1))
	    {
	      pr_err("%s %d. write camera_dsm_log_buff error\n",__func__, __LINE__);
	      return ;
	    }
	    camera_report_dsm_err(DSM_CAMERA_LED_FLASH_VOUT_ERROR, flash_reg, camera_dsm_log_buff);
	}

	return;
}
#endif

/****************************************************************************
* FunctionName: msm_flash_clear_err_and_unlock;
* Description : clear the error and unlock the IC ;
* NOTE: this funtion must be called before register is read and write
***************************************************************************/
int msm_flash_clear_err_and_unlock(struct msm_led_flash_ctrl_t *fctrl){

        int rc = 0;
        uint16_t reg_value=0;

        LM3642_DBG("%s entry\n", __func__);

        if (!fctrl) {
                pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
                return -EINVAL;
        }


        if (fctrl->flash_i2c_client) {
                rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
                        fctrl->flash_i2c_client,
                        FLASH_FLAG_REGISTER,&reg_value, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0){
                        pr_err("clear err and unlock %s:%d failed\n", __func__, __LINE__);
                }

                LM3642_DBG("clear err and unlock success:%02x\n",reg_value);
#ifdef CONFIG_HUAWEI_DSM
                if (reg_value != 0)
                {
                  camera_report_flash_dsm_err(reg_value, rc);
                }
#endif
        }else{
                pr_err("%s:%d flash_i2c_client NULL\n", __func__, __LINE__);
                return -EINVAL;
        }


       return 0;

}
int msm_flash_lm3642_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	return rc;
}

int msm_flash_lm3642_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	LM3642_DBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);
	
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	return 0;
}

int msm_flash_lm3642_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	return rc;
}

int msm_flash_lm3642_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;


	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_lm3642_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

/****************************************************************************
* FunctionName: msm_torch_lm3642_led_on;
* Description : set torch func ;
***************************************************************************/
int msm_torch_lm3642_led_on(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3642_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->torch_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	return rc;
}

static int msm_flash_lm3642_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3642_DBG("%s entry\n", __func__);
	if (!id) {
		pr_err("msm_flash_lm3642_i2c_probe: id is NULL");
		id = lm3642_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

     /* used for debug  */
	// msm_flash_lm3642_led_low(&fctrl);  

	if (!rc)
		msm_lm3642_torch_create_classdev(&(client->dev),NULL);
	return rc;
}

static int msm_flash_lm3642_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3642_DBG("%s entry\n", __func__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	return rc;
}

static void lm3624_shutdown(struct i2c_client * client)
{
    pr_err("[%s],[%d]\n", __func__, __LINE__);
    msm_flash_led_off(&fctrl);
    msm_flash_led_release(&fctrl);
}

static struct i2c_driver lm3642_i2c_driver = {
	.id_table = lm3642_i2c_id,
	.probe  = msm_flash_lm3642_i2c_probe,
	.remove = msm_flash_lm3642_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_i2c_trigger_dt_match,
	},
	.shutdown = lm3624_shutdown,
};

static int __init msm_flash_lm3642_init(void)
{
	LM3642_DBG("%s entry\n", __func__);
	return i2c_add_driver(&lm3642_i2c_driver);
}

static void __exit msm_flash_lm3642_exit(void)
{
	LM3642_DBG("%s entry\n", __func__);
	i2c_del_driver(&lm3642_i2c_driver);
	return;
}


static struct msm_camera_i2c_client lm3642_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_off_setting = {
	.reg_setting = lm3642_off_array,
	.size = ARRAY_SIZE(lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_release_setting = {
	.reg_setting = lm3642_release_array,
	.size = ARRAY_SIZE(lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting = {
	.reg_setting = lm3642_low_array,
	.size = ARRAY_SIZE(lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting = {
	.reg_setting = lm3642_high_array,
	.size = ARRAY_SIZE(lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_torch_setting = {
	.reg_setting = lm3642_torch_array,
	.size = ARRAY_SIZE(lm3642_torch_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3642_regs = {
	.init_setting = &lm3642_init_setting,
	.off_setting = &lm3642_off_setting,
	.low_setting = &lm3642_low_setting,
	.high_setting = &lm3642_high_setting,
	.release_setting = &lm3642_release_setting,
	.torch_setting = &lm3642_torch_setting,
};

static struct msm_flash_fn_t lm3642_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3642_led_init,
	.flash_led_release = msm_flash_lm3642_led_release,
	.flash_led_off = msm_flash_lm3642_led_off,
	.flash_led_low = msm_flash_lm3642_led_low,
	.flash_led_high = msm_flash_lm3642_led_high,
	.torch_led_on = msm_torch_lm3642_led_on,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3642_i2c_client,
	.reg_setting = &lm3642_regs,
	.func_tbl = &lm3642_func_tbl,
	// Set default flash high current.
	.flash_high_current = FLASH_HIGH_CURRENT,
};

module_init(msm_flash_lm3642_init);
module_exit(msm_flash_lm3642_exit);
MODULE_DESCRIPTION("lm3642 FLASH");
MODULE_LICENSE("GPL v2");
