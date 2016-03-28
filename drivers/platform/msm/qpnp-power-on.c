/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/log2.h>
#include <linux/qpnp/power-on.h>
#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#include <linux/hw_lcd_common.h>
#endif
#ifdef CONFIG_SRECORDER
#include <srecorder_log.h>
#include <linux/srecorder.h>
#endif
#ifdef  CONFIG_HUAWEI_KERNEL
#include <soc/qcom/smsm.h>
#endif
#ifdef CONFIG_HUAWEI_PMU_DSM
#include <linux/power/huawei_dsm_charger.h>
#endif
#ifdef CONFIG_HUAWEI_RESET_DETECT
#include <linux/huawei_reset_detect.h>
#endif
#ifdef CONFIG_HUAWEI_KERNEL
#define PON_INT_RT_STS_KPDPWR_ON  0x0
#define PON_INT_RT_STS_KPDPWR_BARK  0x3
#endif
#define CREATE_MASK(NUM_BITS, POS) \
	((unsigned char) (((1 << (NUM_BITS)) - 1) << (POS)))
#define PON_MASK(MSB_BIT, LSB_BIT) \
	CREATE_MASK(MSB_BIT - LSB_BIT + 1, LSB_BIT)

#define PMIC_VER_8941           0x01
#define PMIC_VERSION_REG        0x0105
#define PMIC_VERSION_REV4_REG   0x0103

#define PMIC8941_V1_REV4	0x01
#define PMIC8941_V2_REV4	0x02
#define PON_REV2_VALUE		0x00

/* Common PNP defines */
#define QPNP_PON_REVISION2(base)		(base + 0x01)

/* PON common register addresses */
#define QPNP_PON_RT_STS(base)			(base + 0x10)
#define QPNP_PON_PULL_CTL(base)			(base + 0x70)
#define QPNP_PON_DBC_CTL(base)			(base + 0x71)

/* PON/RESET sources register addresses */
#define QPNP_PON_REASON1(base)			(base + 0x8)
#define QPNP_PON_WARM_RESET_REASON1(base)	(base + 0xA)
#define QPNP_PON_WARM_RESET_REASON2(base)	(base + 0xB)
#define QPNP_POFF_REASON1(base)			(base + 0xC)
#define QPNP_PON_KPDPWR_S1_TIMER(base)		(base + 0x40)
#define QPNP_PON_KPDPWR_S2_TIMER(base)		(base + 0x41)
#define QPNP_PON_KPDPWR_S2_CNTL(base)		(base + 0x42)
#define QPNP_PON_KPDPWR_S2_CNTL2(base)		(base + 0x43)
#define QPNP_PON_RESIN_S1_TIMER(base)		(base + 0x44)
#define QPNP_PON_RESIN_S2_TIMER(base)		(base + 0x45)
#define QPNP_PON_RESIN_S2_CNTL(base)		(base + 0x46)
#define QPNP_PON_RESIN_S2_CNTL2(base)		(base + 0x47)
#define QPNP_PON_KPDPWR_RESIN_S1_TIMER(base)	(base + 0x48)
#define QPNP_PON_KPDPWR_RESIN_S2_TIMER(base)	(base + 0x49)
#define QPNP_PON_KPDPWR_RESIN_S2_CNTL(base)	(base + 0x4A)
#define QPNP_PON_KPDPWR_RESIN_S2_CNTL2(base)	(base + 0x4B)
#define QPNP_PON_PS_HOLD_RST_CTL(base)		(base + 0x5A)
#define QPNP_PON_PS_HOLD_RST_CTL2(base)		(base + 0x5B)
#define QPNP_PON_WD_RST_S2_CTL(base)		(base + 0x56)
#define QPNP_PON_WD_RST_S2_CTL2(base)		(base + 0x57)
#define QPNP_PON_S3_SRC(base)			(base + 0x74)
#define QPNP_PON_S3_DBC_CTL(base)		(base + 0x75)
#define QPNP_PON_TRIGGER_EN(base)		(base + 0x80)
#define QPNP_PON_XVDD_RB_SPARE(base)		(base + 0x8E)
#define QPNP_PON_SOFT_RB_SPARE(base)		(base + 0x8F)
#define QPNP_PON_SEC_ACCESS(base)		(base + 0xD0)

#define QPNP_PON_SEC_UNLOCK			0xA5

#define QPNP_PON_WARM_RESET_TFT			BIT(4)

#define QPNP_PON_RESIN_PULL_UP			BIT(0)
#define QPNP_PON_KPDPWR_PULL_UP			BIT(1)
#define QPNP_PON_CBLPWR_PULL_UP			BIT(2)
#define QPNP_PON_S2_CNTL_EN			BIT(7)
#define QPNP_PON_S2_RESET_ENABLE		BIT(7)
#define QPNP_PON_DELAY_BIT_SHIFT		6

#define QPNP_PON_S1_TIMER_MASK			(0xF)
#define QPNP_PON_S2_TIMER_MASK			(0x7)
#define QPNP_PON_S2_CNTL_TYPE_MASK		(0xF)

#define QPNP_PON_DBC_DELAY_MASK			(0x7)
#define QPNP_PON_KPDPWR_N_SET			BIT(0)
#define QPNP_PON_RESIN_N_SET			BIT(1)
#define QPNP_PON_CBLPWR_N_SET			BIT(2)
#define QPNP_PON_RESIN_BARK_N_SET		BIT(4)
#define QPNP_PON_KPDPWR_RESIN_BARK_N_SET	BIT(5)

#define QPNP_PON_WD_EN			BIT(7)
#define QPNP_PON_RESET_EN			BIT(7)
#define QPNP_PON_POWER_OFF_MASK			0xF

#define QPNP_PON_S3_SRC_KPDPWR			0
#define QPNP_PON_S3_SRC_RESIN			1
#define QPNP_PON_S3_SRC_KPDPWR_AND_RESIN	2
#define QPNP_PON_S3_SRC_KPDPWR_OR_RESIN		3
#define QPNP_PON_S3_SRC_MASK			0x3
#define QPNP_PON_HARD_RESET_MASK		PON_MASK(7, 5)

#define QPNP_PON_UVLO_DLOAD_EN		BIT(7)

/* Ranges */
#define QPNP_PON_S1_TIMER_MAX			10256
#define QPNP_PON_S2_TIMER_MAX			2000
#define QPNP_PON_S3_TIMER_SECS_MAX		128
#define QPNP_PON_S3_DBC_DELAY_MASK		0x07
#define QPNP_PON_RESET_TYPE_MAX			0xF
#define PON_S1_COUNT_MAX			0xF
#define QPNP_PON_MIN_DBC_US			(USEC_PER_SEC / 64)
#define QPNP_PON_MAX_DBC_US			(USEC_PER_SEC * 2)

#define QPNP_KEY_STATUS_DELAY			msecs_to_jiffies(250)

#define QPNP_PON_BUFFER_SIZE			9

#define QPNP_POFF_REASON_UVLO			13
#ifdef  CONFIG_HUAWEI_KERNEL
#define PON_DIS_PWRKPD_RESET 1
#endif
#ifdef CONFIG_HUAWEI_PMU_DSM
#define CHECK_PMU_STATUS_DELAY			msecs_to_jiffies(30000)
#define PA_CPU_TEMP_THRESH			70
#define TSENS_TEMP_THRESH			90
#define TSENS_ID2		2
#define TSENS_ID5		5
#define PON_REA_SMPL	1
#define PON_HARD_RESET	0
#define POFF_REA_TFT			12
#define POFF_REA_UVLO			13
#define POFF_REA_PMIC_OVETEMP		14
#define POFF_REA_STAGE3		15
#define LDO1_ADDR_BASE			0x14000
#define LDO_OFFSET				0x100
#define STATUS1_OFFSET			0x08
#define STATUS2_OFFSET			0x09
#define LDO_ON_MASK			0x20
#define LDO_VOL_MASK			0x80
#define LDO_NUM					18

/*pmu dsm client definition */
struct dsm_dev dsm_pmu = {
	.name = "dsm_pmu", // dsm client name
	.fops = NULL,
	.buff_size = 1024, // buffer size
};
struct dsm_client *pmu_dclient = NULL;

extern int dsm_get_pa_temp(void);
extern int dsm_get_cpu_temp(void);
extern int get_tsens_temp(uint32_t tsensor_id, long *temp);
#endif
enum pon_type {
	PON_KPDPWR,
	PON_RESIN,
	PON_CBLPWR,
	PON_KPDPWR_RESIN,
};

struct qpnp_pon_config {
	u32 pon_type;
	u32 support_reset;
	u32 key_code;
	u32 s1_timer;
	u32 s2_timer;
	u32 s2_type;
	u32 pull_up;
	u32 state_irq;
	u32 bark_irq;
	u16 s2_cntl_addr;
	u16 s2_cntl2_addr;
	bool old_state;
	bool use_bark;
};

struct qpnp_pon {
	struct spmi_device *spmi;
	struct input_dev *pon_input;
	struct qpnp_pon_config *pon_cfg;
	int num_pon_config;
	u16 base;
	struct delayed_work bark_work;
#ifdef CONFIG_HUAWEI_PMU_DSM
	struct delayed_work dsm_pmu_work;
#endif
	u32 dbc;
	int pon_trigger_reason;
	int pon_power_off_reason;
	u32 uvlo;
	struct dentry *debugfs;
	u8 warm_reset_reason1;
	u8 warm_reset_reason2;
	bool store_hard_reset_reason;
};

static struct qpnp_pon *sys_reset_dev;

bool power_key_ps = false;
static u32 s1_delay[PON_S1_COUNT_MAX + 1] = {
	0 , 32, 56, 80, 138, 184, 272, 408, 608, 904, 1352, 2048,
	3072, 4480, 6720, 10256
};
#ifdef CONFIG_HUAWEI_KERNEL
u32 huawei_pon_regs[MAX_REG_TYPE] = {0};
#endif

static const char * const qpnp_pon_reason[] = {
	[0] = "Triggered from Hard Reset",
	[1] = "Triggered from SMPL (sudden momentary power loss)",
	[2] = "Triggered from RTC (RTC alarm expiry)",
	[3] = "Triggered from DC (DC charger insertion)",
	[4] = "Triggered from USB (USB charger insertion)",
	[5] = "Triggered from PON1 (secondary PMIC)",
	[6] = "Triggered from CBL (external power supply)",
	[7] = "Triggered from KPD (power key press)",
};

static const char * const qpnp_poff_reason[] = {
	[0] = "Triggered from SOFT (Software)",
	[1] = "Triggered from PS_HOLD (PS_HOLD/MSM controlled shutdown)",
	[2] = "Triggered from PMIC_WD (PMIC watchdog)",
	[3] = "Triggered from GP1 (Keypad_Reset1)",
	[4] = "Triggered from GP2 (Keypad_Reset2)",
	[5] = "Triggered from KPDPWR_AND_RESIN"
		"(Simultaneous power key and reset line)",
	[6] = "Triggered from RESIN_N (Reset line/Volume Down Key)",
	[7] = "Triggered from KPDPWR_N (Long Power Key hold)",
	[8] = "N/A",
	[9] = "N/A",
	[10] = "N/A",
	[11] = "Triggered from CHARGER (Charger ENUM_TIMER, BOOT_DONE)",
	[12] = "Triggered from TFT (Thermal Fault Tolerance)",
	[13] = "Triggered from UVLO (Under Voltage Lock Out)",
	[14] = "Triggered from OTST3 (Overtemp)",
	[15] = "Triggered from STAGE3 (Stage 3 reset)",
};

/*
 * On the kernel command line specify
 * qpnp-power-on.warm_boot=1 to indicate a warm
 * boot of the device.
 */
static int warm_boot;
#ifdef CONFIG_HUAWEI_DSM
struct lcd_pwr_status_t lcd_pwr_status;
#endif
module_param(warm_boot, int, 0);
#ifdef  CONFIG_HUAWEI_KERNEL
static int hw_get_pwrkpd_flag(void)
{
	smem_exten_huawei_paramater *smem = NULL;
	static bool pwrkpd_flag = false;
	static int pwrkpd_reset = -1;
	if(pwrkpd_flag)
	{
		return pwrkpd_reset;
	}
	smem = smem_alloc(SMEM_ID_VENDOR1, sizeof(smem_exten_huawei_paramater),0,SMEM_ANY_HOST_FLAG);
	if(NULL != smem)
	{
		pwrkpd_reset = smem->pwrkpd_reset;
	}
	pwrkpd_flag = true;
	return pwrkpd_reset;
}
#endif
static int
qpnp_pon_masked_write(struct qpnp_pon *pon, u16 addr, u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
							addr, &reg, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to read from addr=%hx, rc(%d)\n",
			addr, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;
	rc = spmi_ext_register_writel(pon->spmi->ctrl, pon->spmi->sid,
							addr, &reg, 1);
	if (rc)
		dev_err(&pon->spmi->dev,
			"Unable to write to addr=%hx, rc(%d)\n", addr, rc);
	return rc;
}

/**
 * qpnp_pon_set_restart_reason - Store device restart reason in PMIC register.
 *
 * Returns = 0 if PMIC feature is not avaliable or store restart reason
 * successfully.
 * Returns > 0 for errors
 *
 * This function is used to store device restart reason in PMIC register.
 * It checks here to see if the restart reason register has been specified.
 * If it hasn't, this function should immediately return 0
 */
int qpnp_pon_set_restart_reason(enum pon_restart_reason reason)
{
	int rc = 0;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return 0;

	if (!pon->store_hard_reset_reason)
		return 0;

	rc = qpnp_pon_masked_write(pon, QPNP_PON_SOFT_RB_SPARE(pon->base),
					PON_MASK(7, 5), (reason << 5));
	if (rc)
		dev_err(&pon->spmi->dev,
				"Unable to write to addr=%x, rc(%d)\n",
				QPNP_PON_SOFT_RB_SPARE(pon->base), rc);
	return rc;
}
EXPORT_SYMBOL(qpnp_pon_set_restart_reason);

/*
 * qpnp_pon_check_hard_reset_stored - Checks if the PMIC need to
 * store hard reset reason.
 *
 * Returns true if reset reason can be stored, false if it cannot be stored
 *
 */
bool qpnp_pon_check_hard_reset_stored(void)
{
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return false;

	return pon->store_hard_reset_reason;
}
EXPORT_SYMBOL(qpnp_pon_check_hard_reset_stored);

static int qpnp_pon_set_dbc(struct qpnp_pon *pon, u32 delay)
{
	int rc = 0;
	u32 delay_reg;

	mutex_lock(&pon->pon_input->mutex);
	if (delay == pon->dbc)
		goto unlock;

	if (delay < QPNP_PON_MIN_DBC_US)
		delay = QPNP_PON_MIN_DBC_US;
	else if (delay > QPNP_PON_MAX_DBC_US)
		delay = QPNP_PON_MAX_DBC_US;

	delay_reg = (delay << QPNP_PON_DELAY_BIT_SHIFT) / USEC_PER_SEC;
	delay_reg = ilog2(delay_reg);
	rc = qpnp_pon_masked_write(pon, QPNP_PON_DBC_CTL(pon->base),
					QPNP_PON_DBC_DELAY_MASK, delay_reg);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to set PON debounce\n");
		goto unlock;
	}

	pon->dbc = delay;

unlock:
	mutex_unlock(&pon->pon_input->mutex);
	return rc;
}

static ssize_t qpnp_pon_dbc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct qpnp_pon *pon = dev_get_drvdata(dev);

	return snprintf(buf, QPNP_PON_BUFFER_SIZE, "%d\n", pon->dbc);
}

static ssize_t qpnp_pon_dbc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct qpnp_pon *pon = dev_get_drvdata(dev);
	u32 value;
	int rc;

	if (size > QPNP_PON_BUFFER_SIZE)
		return -EINVAL;

	rc = kstrtou32(buf, 10, &value);
	if (rc)
		return rc;

	rc = qpnp_pon_set_dbc(pon, value);
	if (rc < 0)
		return rc;

	return size;
}

static DEVICE_ATTR(debounce_us, 0664, qpnp_pon_dbc_show, qpnp_pon_dbc_store);

/**
 * qpnp_pon_system_pwr_off - Configure system-reset PMIC for shutdown or reset
 * @type: Determines the type of power off to perform - shutdown, reset, etc
 *
 * This function will only configure a single PMIC. The other PMICs in the
 * system are slaved off of it and require no explicit configuration. Once
 * the system-reset PMIC is configured properly, the MSM can drop PS_HOLD to
 * activate the specified configuration.
 */
int qpnp_pon_system_pwr_off(enum pon_power_off_type type)
{
	int rc;
	u8 reg;
	u16 rst_en_reg;
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
			QPNP_PON_REVISION2(pon->base), &reg, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to read addr=%x, rc(%d)\n",
			QPNP_PON_REVISION2(pon->base), rc);
		return rc;
	}

	if (reg == PON_REV2_VALUE)
		rst_en_reg = QPNP_PON_PS_HOLD_RST_CTL(pon->base);
	else
		rst_en_reg = QPNP_PON_PS_HOLD_RST_CTL2(pon->base);

	rc = qpnp_pon_masked_write(pon, rst_en_reg, QPNP_PON_RESET_EN, 0);
	if (rc)
		dev_err(&pon->spmi->dev,
			"Unable to write to addr=%hx, rc(%d)\n",
			rst_en_reg, rc);

	/*
	 * We need 10 sleep clock cycles here. But since the clock is
	 * internally generated, we need to add 50% tolerance to be
	 * conservative.
	 */
	udelay(500);

	rc = qpnp_pon_masked_write(pon, QPNP_PON_PS_HOLD_RST_CTL(pon->base),
				   QPNP_PON_POWER_OFF_MASK, type);
	if (rc)
		dev_err(&pon->spmi->dev,
			"Unable to write to addr=%x, rc(%d)\n",
				QPNP_PON_PS_HOLD_RST_CTL(pon->base), rc);

	rc = qpnp_pon_masked_write(pon, rst_en_reg, QPNP_PON_RESET_EN,
						    QPNP_PON_RESET_EN);
	if (rc)
		dev_err(&pon->spmi->dev,
			"Unable to write to addr=%hx, rc(%d)\n",
			rst_en_reg, rc);

	dev_dbg(&pon->spmi->dev, "power off type = 0x%02X\n", type);

	return rc;
}
EXPORT_SYMBOL(qpnp_pon_system_pwr_off);

/**
 * qpnp_pon_is_warm_reset - Checks if the PMIC went through a warm reset.
 *
 * Returns > 0 for warm resets, 0 for not warm reset, < 0 for errors
 *
 * Note that this function will only return the warm vs not-warm reset status
 * of the PMIC that is configured as the system-reset device.
 */
int qpnp_pon_is_warm_reset(void)
{
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -EPROBE_DEFER;

	return pon->warm_reset_reason1
		|| (pon->warm_reset_reason2 & QPNP_PON_WARM_RESET_TFT);
}
EXPORT_SYMBOL(qpnp_pon_is_warm_reset);

/**
 * qpnp_pon_wd_config - Disable the wd in a warm reset.
 * @enable: to enable or disable the PON watch dog
 *
 * Returns = 0 for operate successfully, < 0 for errors
 */
int qpnp_pon_wd_config(bool enable)
{
	struct qpnp_pon *pon = sys_reset_dev;
	int rc = 0;

	if (!pon)
		return -EPROBE_DEFER;

	rc = qpnp_pon_masked_write(pon, QPNP_PON_WD_RST_S2_CTL2(pon->base),
			QPNP_PON_WD_EN, enable ? QPNP_PON_WD_EN : 0);
	if (rc)
		dev_err(&pon->spmi->dev,
				"Unable to write to addr=%x, rc(%d)\n",
				QPNP_PON_WD_RST_S2_CTL2(pon->base), rc);

	return rc;
}
EXPORT_SYMBOL(qpnp_pon_wd_config);


/**
 * qpnp_pon_trigger_config - Configures (enable/disable) the PON trigger source
 * @pon_src: PON source to be configured
 * @enable: to enable or disable the PON trigger
 *
 * This function configures the power-on trigger capability of a
 * PON source. If a specific PON trigger is disabled it cannot act
 * as a power-on source to the PMIC.
 */

int qpnp_pon_trigger_config(enum pon_trigger_source pon_src, bool enable)
{
	struct qpnp_pon *pon = sys_reset_dev;
	int rc;

	if (!pon)
		return -EPROBE_DEFER;

	if (pon_src < PON_SMPL || pon_src > PON_KPDPWR_N) {
		dev_err(&pon->spmi->dev, "Invalid PON source\n");
		return -EINVAL;
	}

	rc = qpnp_pon_masked_write(pon, QPNP_PON_TRIGGER_EN(pon->base),
				BIT(pon_src), enable ? BIT(pon_src) : 0);
	if (rc)
		dev_err(&pon->spmi->dev, "Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_TRIGGER_EN(pon->base), rc);

	return rc;
}
EXPORT_SYMBOL(qpnp_pon_trigger_config);

/*
 * This function stores the PMIC warm reset reason register values. It also
 * clears these registers if the qcom,clear-warm-reset device tree property
 * is specified.
 */
static int qpnp_pon_store_and_clear_warm_reset(struct qpnp_pon *pon)
{
	int rc;
	u8 reg = 0;

	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
			QPNP_PON_WARM_RESET_REASON1(pon->base),
			&pon->warm_reset_reason1, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to read addr=%x, rc(%d)\n",
			QPNP_PON_WARM_RESET_REASON1(pon->base), rc);
		return rc;
	}

#ifdef CONFIG_HUAWEI_KERNEL	
	huawei_pon_regs[WARM_REASON_INDEX] = pon->warm_reset_reason1;
#endif	
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
			QPNP_PON_WARM_RESET_REASON2(pon->base),
			&pon->warm_reset_reason2, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to read addr=%x, rc(%d)\n",
			QPNP_PON_WARM_RESET_REASON2(pon->base), rc);
		return rc;
	}

	if (of_property_read_bool(pon->spmi->dev.of_node,
					"qcom,clear-warm-reset")) {
		rc = spmi_ext_register_writel(pon->spmi->ctrl, pon->spmi->sid,
			QPNP_PON_WARM_RESET_REASON1(pon->base), &reg, 1);
		if (rc)
			dev_err(&pon->spmi->dev,
				"Unable to write to addr=%hx, rc(%d)\n",
				QPNP_PON_WARM_RESET_REASON1(pon->base), rc);
	}

	return 0;
}

static struct qpnp_pon_config *
qpnp_get_cfg(struct qpnp_pon *pon, u32 pon_type)
{
	int i;

	for (i = 0; i < pon->num_pon_config; i++) {
		if (pon_type == pon->pon_cfg[i].pon_type)
			return  &pon->pon_cfg[i];
	}

	return NULL;
}

static int
qpnp_pon_input_dispatch(struct qpnp_pon *pon, u32 pon_type)
{
	int rc;
	struct qpnp_pon_config *cfg = NULL;
	u8 pon_rt_sts = 0, pon_rt_bit = 0;
	u32 key_status;

	cfg = qpnp_get_cfg(pon, pon_type);
	if (!cfg)
		return -EINVAL;

	/* Check if key reporting is supported */
	if (!cfg->key_code)
		return 0;

	/* check the RT status to get the current status of the line */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
				QPNP_PON_RT_STS(pon->base), &pon_rt_sts, 1);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to read PON RT status\n");
		return rc;
	}

	switch (cfg->pon_type) {
	case PON_KPDPWR:
		pon_rt_bit = QPNP_PON_KPDPWR_N_SET;
		break;
	case PON_RESIN:
		pon_rt_bit = QPNP_PON_RESIN_N_SET;
		break;
	case PON_CBLPWR:
		pon_rt_bit = QPNP_PON_CBLPWR_N_SET;
		break;
	case PON_KPDPWR_RESIN:
		pon_rt_bit = QPNP_PON_KPDPWR_RESIN_BARK_N_SET;
		break;
	default:
		return -EINVAL;
	}

#ifdef CONFIG_HUAWEI_KERNEL
	pr_err("PMIC input: code=%d, sts=0x%hhx\n",
					cfg->key_code, pon_rt_sts);
#else
	pr_debug("PMIC input: code=%d, sts=0x%hhx\n",
					cfg->key_code, pon_rt_sts);
#endif
	key_status = pon_rt_sts & pon_rt_bit;

	/* simulate press event in case release event occured
	 * without a press event
	 */
	if (!cfg->old_state && !key_status) {
		input_report_key(pon->pon_input, cfg->key_code, 1);
		input_sync(pon->pon_input);
	}

	input_report_key(pon->pon_input, cfg->key_code, key_status);
	input_sync(pon->pon_input);

#ifdef CONFIG_HUAWEI_DSM
	if(cfg->key_code == DSM_POWER_KEY_VAL && key_status){
		dsm_key_pressed(DSM_POW_KEY);
	}else if(cfg->key_code == DSM_VOL_DOWN_KEY_VAL && key_status){
		dsm_key_pressed(DSM_VOL_DOWN_KEY);
	}
#endif
#define POWERKEY_KEYCODE 116
#define KEY_DOWN_S 1


	if(cfg->key_code == POWERKEY_KEYCODE && key_status == KEY_DOWN_S)
	{
		power_key_ps = true;
	}
#ifdef CONFIG_HUAWEI_DSM
	/*when device wake up,we enable the timer,3 senconds later report the lcd power status if work abnormally*/
	if(cfg->key_code == POWERKEY_KEYCODE && key_status == KEY_DOWN_S && lcd_pwr_status.panel_power_on== 0)
	{
		del_timer(&lcd_pwr_status.lcd_dsm_t);
		lcd_pwr_status.lcd_dsm_t.function = lcd_dcm_pwr_status_handler;
		lcd_pwr_status.lcd_dsm_t.data = 0;
		lcd_pwr_status.lcd_dsm_t.expires = jiffies + 3*HZ;
		//lcd_pwr_status.lcd_dcm_pwr_status = 0;
		add_timer(&lcd_pwr_status.lcd_dsm_t);
	}
	/*(lcd_dcm_pwr_status & 0x000f) >> 3 == 1 means lcd  power on*/
	/*when device suspend,del the timer*/
	else if((cfg->key_code == POWERKEY_KEYCODE) && (key_status == KEY_DOWN_S) && (lcd_pwr_status.panel_power_on== 1) )
	{
		del_timer(&lcd_pwr_status.lcd_dsm_t);
	}
#endif
	cfg->old_state = !!key_status;

	return 0;
}

static irqreturn_t qpnp_kpdpwr_irq(int irq, void *_pon)
{
	int rc;
	struct qpnp_pon *pon = _pon;
#ifdef CONFIG_HUAWEI_KERNEL
	struct timeval tv_pwr;
	struct tm tm_pwr;
#endif

	rc = qpnp_pon_input_dispatch(pon, PON_KPDPWR);
	if (rc)
		dev_err(&pon->spmi->dev, "Unable to send input event\n");
#ifdef CONFIG_HUAWEI_KERNEL
	do_gettimeofday(&tv_pwr);
	time_to_tm(tv_pwr.tv_sec, 0, &tm_pwr);
	pr_info("Exit %s : [%ld-%d-%d]%d:%d:%d:%ld\n",__func__,tm_pwr.tm_year + 1900,tm_pwr.tm_mon+1,
						tm_pwr.tm_mday,tm_pwr.tm_hour,tm_pwr.tm_min,tm_pwr.tm_sec,tv_pwr.tv_usec%1000);
#endif
	return IRQ_HANDLED;
}

#ifdef CONFIG_MSM_DLOAD_MODE
#ifdef CONFIG_HUAWEI_KERNEL
extern void clear_dload_mode(void);
#endif
#endif

static irqreturn_t qpnp_kpdpwr_bark_irq(int irq, void *_pon)
{
#ifdef CONFIG_HUAWEI_KERNEL	
    int rc = 0;
	u8 pon_rt_sts = 0;
	u8 pon_rt_sts_kpdpwr_bark = (0x1 << PON_INT_RT_STS_KPDPWR_ON)|(0x1 << PON_INT_RT_STS_KPDPWR_BARK);
	struct qpnp_pon *pon = _pon;

	/* check the RT status to get the current status of the line */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
				QPNP_PON_RT_STS(pon->base), &pon_rt_sts, 1);
	if (rc)
	{
		dev_err(&pon->spmi->dev, "Unable to read PON RT status\n");
	}

	pr_err("PMIC input: sts=0x%hhx, when expect 0x%hhx.\n",  pon_rt_sts, pon_rt_sts_kpdpwr_bark);
#endif
#ifdef CONFIG_MSM_DLOAD_MODE
#ifdef CONFIG_HUAWEI_KERNEL
	/* clear dload mode to reduce false triggering dump*/
	clear_dload_mode();
#endif
#endif

#ifdef CONFIG_SRECORDER
     printk(KERN_ERR "long press power key have detected,reset normal\n");
     srecorder_notify_framework(RESET_NORMAL);
#ifdef CONFIG_HUAWEI_KERNEL
	if(pon_rt_sts == pon_rt_sts_kpdpwr_bark)
	{
#ifdef CONFIG_HUAWEI_RESET_DETECT
        clear_reset_magic();
#endif	
		printk(KERN_ERR "long press power key have detected!\n");
	}
#endif
#endif
	return IRQ_HANDLED;
}

static irqreturn_t qpnp_resin_irq(int irq, void *_pon)
{
	int rc;
	struct qpnp_pon *pon = _pon;

	rc = qpnp_pon_input_dispatch(pon, PON_RESIN);
	if (rc)
		dev_err(&pon->spmi->dev, "Unable to send input event\n");
	return IRQ_HANDLED;
}

static irqreturn_t qpnp_kpdpwr_resin_bark_irq(int irq, void *_pon)
{
	return IRQ_HANDLED;
}

static irqreturn_t qpnp_cblpwr_irq(int irq, void *_pon)
{
	int rc;
	struct qpnp_pon *pon = _pon;

	rc = qpnp_pon_input_dispatch(pon, PON_CBLPWR);
	if (rc)
		dev_err(&pon->spmi->dev, "Unable to send input event\n");

	return IRQ_HANDLED;
}

static void print_pon_reg(struct qpnp_pon *pon, u16 offset)
{
	int rc;
	u16 addr;
	u8 reg;

	addr = pon->base + offset;
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
			addr, &reg, 1);
	if (rc)
		dev_emerg(&pon->spmi->dev,
				"Unable to read reg at 0x%04hx\n", addr);
	else
		dev_emerg(&pon->spmi->dev, "reg@0x%04hx: %02hhx\n", addr, reg);
}

#define PON_PBL_STATUS			0x7
#define PON_PON_REASON1			0x8
#define PON_PON_REASON2			0x9
#define PON_WARM_RESET_REASON1		0xA
#define PON_WARM_RESET_REASON2		0xB
#define PON_POFF_REASON1		0xC
#define PON_POFF_REASON2		0xD
#define PON_SOFT_RESET_REASON1		0xE
#define PON_SOFT_RESET_REASON2		0xF
#define PON_PMIC_WD_RESET_S1_TIMER	0x54
#define PON_PMIC_WD_RESET_S2_TIMER	0x55
static irqreturn_t qpnp_pmic_wd_bark_irq(int irq, void *_pon)
{
	struct qpnp_pon *pon = _pon;

	print_pon_reg(pon, PON_PBL_STATUS);
	print_pon_reg(pon, PON_PBL_STATUS);
	print_pon_reg(pon, PON_PON_REASON1);
	print_pon_reg(pon, PON_PON_REASON2);
	print_pon_reg(pon, PON_WARM_RESET_REASON1);
	print_pon_reg(pon, PON_WARM_RESET_REASON2);
	print_pon_reg(pon, PON_POFF_REASON1);
	print_pon_reg(pon, PON_POFF_REASON2);
	print_pon_reg(pon, PON_SOFT_RESET_REASON1);
	print_pon_reg(pon, PON_SOFT_RESET_REASON2);
	print_pon_reg(pon, PON_PMIC_WD_RESET_S1_TIMER);
	print_pon_reg(pon, PON_PMIC_WD_RESET_S2_TIMER);
	panic("PMIC Watch dog triggered");

	return IRQ_HANDLED;
}

static void bark_work_func(struct work_struct *work)
{
	int rc;
	u8 pon_rt_sts = 0;
	struct qpnp_pon_config *cfg;
	struct qpnp_pon *pon =
		container_of(work, struct qpnp_pon, bark_work.work);

	cfg = qpnp_get_cfg(pon, PON_RESIN);
	if (!cfg) {
		dev_err(&pon->spmi->dev, "Invalid config pointer\n");
		goto err_return;
	}

	/* enable reset */
	rc = qpnp_pon_masked_write(pon, cfg->s2_cntl2_addr,
				QPNP_PON_S2_CNTL_EN, QPNP_PON_S2_CNTL_EN);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to configure S2 enable\n");
		goto err_return;
	}
	/* bark RT status update delay */
	msleep(100);
	/* read the bark RT status */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
				QPNP_PON_RT_STS(pon->base), &pon_rt_sts, 1);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to read PON RT status\n");
		goto err_return;
	}

	if (!(pon_rt_sts & QPNP_PON_RESIN_BARK_N_SET)) {
		/* report the key event and enable the bark IRQ */
		input_report_key(pon->pon_input, cfg->key_code, 0);
		input_sync(pon->pon_input);
		enable_irq(cfg->bark_irq);
	} else {
		/* disable reset */
		rc = qpnp_pon_masked_write(pon, cfg->s2_cntl2_addr,
				QPNP_PON_S2_CNTL_EN, 0);
		if (rc) {
			dev_err(&pon->spmi->dev,
				"Unable to configure S2 enable\n");
			goto err_return;
		}
		/* re-arm the work */
		schedule_delayed_work(&pon->bark_work, QPNP_KEY_STATUS_DELAY);
	}

err_return:
	return;
}

static irqreturn_t qpnp_resin_bark_irq(int irq, void *_pon)
{
	int rc;
	struct qpnp_pon *pon = _pon;
	struct qpnp_pon_config *cfg;

	/* disable the bark interrupt */
	disable_irq_nosync(irq);

	cfg = qpnp_get_cfg(pon, PON_RESIN);
	if (!cfg) {
		dev_err(&pon->spmi->dev, "Invalid config pointer\n");
		goto err_exit;
	}

	/* disable reset */
	rc = qpnp_pon_masked_write(pon, cfg->s2_cntl2_addr,
					QPNP_PON_S2_CNTL_EN, 0);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to configure S2 enable\n");
		goto err_exit;
	}

	/* report the key event */
	input_report_key(pon->pon_input, cfg->key_code, 1);
	input_sync(pon->pon_input);
	/* schedule work to check the bark status for key-release */
	schedule_delayed_work(&pon->bark_work, QPNP_KEY_STATUS_DELAY);
err_exit:
	return IRQ_HANDLED;
}

static int
qpnp_config_pull(struct qpnp_pon *pon, struct qpnp_pon_config *cfg)
{
	int rc;
	u8 pull_bit;

	switch (cfg->pon_type) {
	case PON_KPDPWR:
		pull_bit = QPNP_PON_KPDPWR_PULL_UP;
		break;
	case PON_RESIN:
		pull_bit = QPNP_PON_RESIN_PULL_UP;
		break;
	case PON_CBLPWR:
		pull_bit = QPNP_PON_CBLPWR_PULL_UP;
		break;
	case PON_KPDPWR_RESIN:
		pull_bit = QPNP_PON_KPDPWR_PULL_UP | QPNP_PON_RESIN_PULL_UP;
		break;
	default:
		return -EINVAL;
	}

	rc = qpnp_pon_masked_write(pon, QPNP_PON_PULL_CTL(pon->base),
				pull_bit, cfg->pull_up ? pull_bit : 0);
	if (rc)
		dev_err(&pon->spmi->dev, "Unable to config pull-up\n");

	return rc;
}

static int
qpnp_config_reset(struct qpnp_pon *pon, struct qpnp_pon_config *cfg)
{
	int rc;
	u8 i;
	u16 s1_timer_addr, s2_timer_addr;

	switch (cfg->pon_type) {
	case PON_KPDPWR:
		s1_timer_addr = QPNP_PON_KPDPWR_S1_TIMER(pon->base);
		s2_timer_addr = QPNP_PON_KPDPWR_S2_TIMER(pon->base);
		break;
	case PON_RESIN:
		s1_timer_addr = QPNP_PON_RESIN_S1_TIMER(pon->base);
		s2_timer_addr = QPNP_PON_RESIN_S2_TIMER(pon->base);
		break;
	case PON_KPDPWR_RESIN:
		s1_timer_addr = QPNP_PON_KPDPWR_RESIN_S1_TIMER(pon->base);
		s2_timer_addr = QPNP_PON_KPDPWR_RESIN_S2_TIMER(pon->base);
		break;
	default:
		return -EINVAL;
	}
	/* disable S2 reset */
	rc = qpnp_pon_masked_write(pon, cfg->s2_cntl2_addr,
				QPNP_PON_S2_CNTL_EN, 0);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to configure S2 enable\n");
		return rc;
	}

	usleep(100);

	/* configure s1 timer, s2 timer and reset type */
	for (i = 0; i < PON_S1_COUNT_MAX + 1; i++) {
		if (cfg->s1_timer <= s1_delay[i])
			break;
	}
	rc = qpnp_pon_masked_write(pon, s1_timer_addr,
				QPNP_PON_S1_TIMER_MASK, i);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to configure S1 timer\n");
		return rc;
	}

	i = 0;
	if (cfg->s2_timer) {
		i = cfg->s2_timer / 10;
		i = ilog2(i + 1);
	}

	rc = qpnp_pon_masked_write(pon, s2_timer_addr,
				QPNP_PON_S2_TIMER_MASK, i);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to configure S2 timer\n");
		return rc;
	}

	rc = qpnp_pon_masked_write(pon, cfg->s2_cntl_addr,
				QPNP_PON_S2_CNTL_TYPE_MASK, (u8)cfg->s2_type);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to configure S2 reset type\n");
		return rc;
	}

	/* enable S2 reset */
	rc = qpnp_pon_masked_write(pon, cfg->s2_cntl2_addr,
				QPNP_PON_S2_CNTL_EN, QPNP_PON_S2_CNTL_EN);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to configure S2 enable\n");
		return rc;
	}

	return 0;
}

static int
qpnp_pon_request_irqs(struct qpnp_pon *pon, struct qpnp_pon_config *cfg)
{
	int rc = 0;

	switch (cfg->pon_type) {
	case PON_KPDPWR:
		rc = devm_request_irq(&pon->spmi->dev, cfg->state_irq,
							qpnp_kpdpwr_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						"qpnp_kpdpwr_status", pon);
		if (rc < 0) {
			dev_err(&pon->spmi->dev, "Can't request %d IRQ\n",
							cfg->state_irq);
			return rc;
		}
		if (cfg->use_bark) {
			rc = devm_request_irq(&pon->spmi->dev, cfg->bark_irq,
						qpnp_kpdpwr_bark_irq,
						IRQF_TRIGGER_RISING,
						"qpnp_kpdpwr_bark", pon);
			if (rc < 0) {
				dev_err(&pon->spmi->dev,
					"Can't request %d IRQ\n",
						cfg->bark_irq);
				return rc;
			}
		}
		break;
	case PON_RESIN:
		rc = devm_request_irq(&pon->spmi->dev, cfg->state_irq,
							qpnp_resin_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						"qpnp_resin_status", pon);
		if (rc < 0) {
			dev_err(&pon->spmi->dev, "Can't request %d IRQ\n",
							cfg->state_irq);
			return rc;
		}
		if (cfg->use_bark) {
			rc = devm_request_irq(&pon->spmi->dev, cfg->bark_irq,
						qpnp_resin_bark_irq,
						IRQF_TRIGGER_RISING,
						"qpnp_resin_bark", pon);
			if (rc < 0) {
				dev_err(&pon->spmi->dev,
					"Can't request %d IRQ\n",
						cfg->bark_irq);
				return rc;
			}
		}
		break;
	case PON_CBLPWR:
		rc = devm_request_irq(&pon->spmi->dev, cfg->state_irq,
							qpnp_cblpwr_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					"qpnp_cblpwr_status", pon);
		if (rc < 0) {
			dev_err(&pon->spmi->dev, "Can't request %d IRQ\n",
							cfg->state_irq);
			return rc;
		}
		break;
	case PON_KPDPWR_RESIN:
		if (cfg->use_bark) {
			rc = devm_request_irq(&pon->spmi->dev, cfg->bark_irq,
					qpnp_kpdpwr_resin_bark_irq,
					IRQF_TRIGGER_RISING,
					"qpnp_kpdpwr_resin_bark", pon);
			if (rc < 0) {
				dev_err(&pon->spmi->dev,
					"Can't request %d IRQ\n",
						cfg->bark_irq);
				return rc;
			}
		}
		break;
	default:
		return -EINVAL;
	}

	/* mark the interrupts wakeable if they support linux-key */
	if (cfg->key_code) {
		enable_irq_wake(cfg->state_irq);
		/* special handling for RESIN due to a hardware bug */
		if (cfg->pon_type == PON_RESIN && cfg->support_reset)
			enable_irq_wake(cfg->bark_irq);
	}

	return rc;
}

static int
qpnp_pon_config_input(struct qpnp_pon *pon,  struct qpnp_pon_config *cfg)
{
	if (!pon->pon_input) {
		pon->pon_input = input_allocate_device();
		if (!pon->pon_input) {
			dev_err(&pon->spmi->dev,
				"Can't allocate pon input device\n");
			return -ENOMEM;
		}
		pon->pon_input->name = "qpnp_pon";
		pon->pon_input->phys = "qpnp_pon/input0";
	}

	/* don't send dummy release event when system resumes */
	__set_bit(INPUT_PROP_NO_DUMMY_RELEASE, pon->pon_input->propbit);
	input_set_capability(pon->pon_input, EV_KEY, cfg->key_code);

	return 0;
}

static int qpnp_pon_config_init(struct qpnp_pon *pon)
{
	int rc = 0, i = 0, pmic_wd_bark_irq;
	struct device_node *pp = NULL;
	struct qpnp_pon_config *cfg;
	u8 pon_ver;
	u8 pmic_type;
	u8 revid_rev4;

#ifdef  CONFIG_HUAWEI_KERNEL
	int pwrkpd_flag = 0;
#endif
	/* Check if it is rev B */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
			QPNP_PON_REVISION2(pon->base), &pon_ver, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to read addr=%x, rc(%d)\n",
			QPNP_PON_REVISION2(pon->base), rc);
		return rc;
	}

	/* iterate through the list of pon configs */
	while ((pp = of_get_next_child(pon->spmi->dev.of_node, pp))) {

		cfg = &pon->pon_cfg[i++];

		rc = of_property_read_u32(pp, "qcom,pon-type", &cfg->pon_type);
		if (rc) {
			dev_err(&pon->spmi->dev, "PON type not specified\n");
			return rc;
		}

		switch (cfg->pon_type) {
		case PON_KPDPWR:
			cfg->state_irq = spmi_get_irq_byname(pon->spmi,
							NULL, "kpdpwr");
			if (cfg->state_irq < 0) {
				dev_err(&pon->spmi->dev,
					"Unable to get kpdpwr irq\n");
				return cfg->state_irq;
			}

			rc = of_property_read_u32(pp, "qcom,support-reset",
							&cfg->support_reset);
#ifdef  CONFIG_HUAWEI_KERNEL
			pwrkpd_flag = hw_get_pwrkpd_flag();
            /*if disable pwrkpd flag is true, disable pwrkpd reset function*/
			if (PON_DIS_PWRKPD_RESET == pwrkpd_flag)
			{
				cfg->support_reset = 0;
			}
#endif
			if (rc && rc != -EINVAL) {
				dev_err(&pon->spmi->dev,
					"Unable to read 'support-reset'\n");
				return rc;
			}

			cfg->use_bark = of_property_read_bool(pp,
							"qcom,use-bark");
			if (cfg->use_bark) {
				cfg->bark_irq = spmi_get_irq_byname(pon->spmi,
							NULL, "kpdpwr-bark");
				if (cfg->bark_irq < 0) {
					dev_err(&pon->spmi->dev,
					"Unable to get kpdpwr-bark irq\n");
					return cfg->bark_irq;
				}
			}

			/* If the value read from REVISION2 register is 0x00,
			   then there is a single register to control s2 reset.
			   Otherwise there are separate registers for s2 reset
			   type and s2 reset enable */
			if (pon_ver == PON_REV2_VALUE) {
				cfg->s2_cntl_addr = cfg->s2_cntl2_addr =
					QPNP_PON_KPDPWR_S2_CNTL(pon->base);
			} else {
				cfg->s2_cntl_addr =
					QPNP_PON_KPDPWR_S2_CNTL(pon->base);
				cfg->s2_cntl2_addr =
					QPNP_PON_KPDPWR_S2_CNTL2(pon->base);
			}

			break;
		case PON_RESIN:
			cfg->state_irq = spmi_get_irq_byname(pon->spmi,
							NULL, "resin");
			if (cfg->state_irq < 0) {
				dev_err(&pon->spmi->dev,
					"Unable to get resin irq\n");
				return cfg->bark_irq;
			}

			rc = of_property_read_u32(pp, "qcom,support-reset",
							&cfg->support_reset);
#ifdef  CONFIG_HUAWEI_KERNEL
			pwrkpd_flag = hw_get_pwrkpd_flag();
			if (PON_DIS_PWRKPD_RESET == pwrkpd_flag)
			{
				cfg->support_reset = 0;
			}
#endif
			if (rc && rc != -EINVAL) {
				dev_err(&pon->spmi->dev,
					"Unable to read 'support-reset'\n");
				return rc;
			}

			cfg->use_bark = of_property_read_bool(pp,
							"qcom,use-bark");

			rc = spmi_ext_register_readl(pon->spmi->ctrl,
					pon->spmi->sid, PMIC_VERSION_REG,
							&pmic_type, 1);

			if (rc) {
				dev_err(&pon->spmi->dev,
					"Unable to read PMIC type\n");
				return rc;
			}

			if (pmic_type == PMIC_VER_8941) {

				rc = spmi_ext_register_readl(pon->spmi->ctrl,
					pon->spmi->sid, PMIC_VERSION_REV4_REG,
							&revid_rev4, 1);

				if (rc) {
					dev_err(&pon->spmi->dev,
					"Unable to read PMIC revision ID\n");
					return rc;
				}

				/*PM8941 V3 does not have harware bug. Hence
				bark is not required from PMIC versions 3.0*/
				if (!(revid_rev4 == PMIC8941_V1_REV4 ||
					revid_rev4 == PMIC8941_V2_REV4)) {
					cfg->support_reset = false;
					cfg->use_bark = false;
				}
			}

			if (cfg->use_bark) {
				cfg->bark_irq = spmi_get_irq_byname(pon->spmi,
							NULL, "resin-bark");
				if (cfg->bark_irq < 0) {
					dev_err(&pon->spmi->dev,
					"Unable to get resin-bark irq\n");
					return cfg->bark_irq;
				}
			}

			if (pon_ver == PON_REV2_VALUE) {
				cfg->s2_cntl_addr = cfg->s2_cntl2_addr =
					QPNP_PON_RESIN_S2_CNTL(pon->base);
			} else {
				cfg->s2_cntl_addr =
					QPNP_PON_RESIN_S2_CNTL(pon->base);
				cfg->s2_cntl2_addr =
					QPNP_PON_RESIN_S2_CNTL2(pon->base);
			}

			break;
		case PON_CBLPWR:
			cfg->state_irq = spmi_get_irq_byname(pon->spmi,
							NULL, "cblpwr");
			if (cfg->state_irq < 0) {
				dev_err(&pon->spmi->dev,
						"Unable to get cblpwr irq\n");
				return rc;
			}
			break;
		case PON_KPDPWR_RESIN:
			rc = of_property_read_u32(pp, "qcom,support-reset",
							&cfg->support_reset);
#ifdef  CONFIG_HUAWEI_KERNEL
			pwrkpd_flag = hw_get_pwrkpd_flag();
			if (PON_DIS_PWRKPD_RESET == pwrkpd_flag)
			{
				cfg->support_reset = 0;
			}
#endif
			if (rc && rc != -EINVAL) {
				dev_err(&pon->spmi->dev,
					"Unable to read 'support-reset'\n");
				return rc;
			}

			cfg->use_bark = of_property_read_bool(pp,
							"qcom,use-bark");
			if (cfg->use_bark) {
				cfg->bark_irq = spmi_get_irq_byname(pon->spmi,
						NULL, "kpdpwr-resin-bark");
				if (cfg->bark_irq < 0) {
					dev_err(&pon->spmi->dev,
					"Unable to get kpdpwr-resin-bark irq\n");
					return cfg->bark_irq;
				}
			}

			if (pon_ver == PON_REV2_VALUE) {
				cfg->s2_cntl_addr = cfg->s2_cntl2_addr =
				QPNP_PON_KPDPWR_RESIN_S2_CNTL(pon->base);
			} else {
				cfg->s2_cntl_addr =
				QPNP_PON_KPDPWR_RESIN_S2_CNTL(pon->base);
				cfg->s2_cntl2_addr =
				QPNP_PON_KPDPWR_RESIN_S2_CNTL2(pon->base);
			}

			break;
		default:
			dev_err(&pon->spmi->dev, "PON RESET %d not supported",
								cfg->pon_type);
			return -EINVAL;
		}

		if (cfg->support_reset) {
			/*
			 * Get the reset parameters (bark debounce time and
			 * reset debounce time) for the reset line.
			 */
			rc = of_property_read_u32(pp, "qcom,s1-timer",
							&cfg->s1_timer);
			if (rc) {
				dev_err(&pon->spmi->dev,
					"Unable to read s1-timer\n");
				return rc;
			}
			if (cfg->s1_timer > QPNP_PON_S1_TIMER_MAX) {
				dev_err(&pon->spmi->dev,
					"Incorrect S1 debounce time\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(pp, "qcom,s2-timer",
							&cfg->s2_timer);
			if (rc) {
				dev_err(&pon->spmi->dev,
					"Unable to read s2-timer\n");
				return rc;
			}
			if (cfg->s2_timer > QPNP_PON_S2_TIMER_MAX) {
				dev_err(&pon->spmi->dev,
					"Incorrect S2 debounce time\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(pp, "qcom,s2-type",
							&cfg->s2_type);
			if (rc) {
				dev_err(&pon->spmi->dev,
					"Unable to read s2-type\n");
				return rc;
			}
			if (cfg->s2_type > QPNP_PON_RESET_TYPE_MAX) {
				dev_err(&pon->spmi->dev,
					"Incorrect reset type specified\n");
				return -EINVAL;
			}

		}
		/*
		 * Get the standard-key parameters. This might not be
		 * specified if there is no key mapping on the reset line.
		 */
		rc = of_property_read_u32(pp, "linux,code", &cfg->key_code);
		if (rc && rc != -EINVAL) {
			dev_err(&pon->spmi->dev,
				"Unable to read key-code\n");
			return rc;
		}
		/* Register key configuration */
		if (cfg->key_code) {
			rc = qpnp_pon_config_input(pon, cfg);
			if (rc < 0)
				return rc;
		}
		/* get the pull-up configuration */
		rc = of_property_read_u32(pp, "qcom,pull-up", &cfg->pull_up);
		if (rc && rc != -EINVAL) {
			dev_err(&pon->spmi->dev, "Unable to read pull-up\n");
			return rc;
		}
	}

	pmic_wd_bark_irq = spmi_get_irq_byname(pon->spmi, NULL, "pmic-wd-bark");
	/* request the pmic-wd-bark irq only if it is defined */
	if (pmic_wd_bark_irq >= 0) {
		rc = devm_request_irq(&pon->spmi->dev, pmic_wd_bark_irq,
					qpnp_pmic_wd_bark_irq,
					IRQF_TRIGGER_RISING,
					"qpnp_pmic_wd_bark", pon);
		if (rc < 0) {
			dev_err(&pon->spmi->dev,
				"Can't request %d IRQ\n",
					pmic_wd_bark_irq);
			goto free_input_dev;
		}
	}

	/* register the input device */
	if (pon->pon_input) {
		rc = input_register_device(pon->pon_input);
		if (rc) {
			dev_err(&pon->spmi->dev,
				"Can't register pon key: %d\n", rc);
			goto free_input_dev;
		}
	}

	for (i = 0; i < pon->num_pon_config; i++) {
		cfg = &pon->pon_cfg[i];
		/* Configure the pull-up */
		rc = qpnp_config_pull(pon, cfg);
		if (rc) {
			dev_err(&pon->spmi->dev, "Unable to config pull-up\n");
			goto unreg_input_dev;
		}
		/* Configure the reset-configuration */
		if (cfg->support_reset) {
			rc = qpnp_config_reset(pon, cfg);
			if (rc) {
				dev_err(&pon->spmi->dev,
					"Unable to config pon reset\n");
				goto unreg_input_dev;
			}
		} else if (cfg->pon_type != PON_CBLPWR) {
			/* disable S2 reset */
			rc = qpnp_pon_masked_write(pon, cfg->s2_cntl2_addr,
						QPNP_PON_S2_CNTL_EN, 0);
			if (rc) {
				dev_err(&pon->spmi->dev,
					"Unable to disable S2 reset\n");
				goto unreg_input_dev;
			}
		}

		rc = qpnp_pon_request_irqs(pon, cfg);
		if (rc) {
			dev_err(&pon->spmi->dev, "Unable to request-irq's\n");
			goto unreg_input_dev;
		}
	}

	device_init_wakeup(&pon->spmi->dev, 1);

	return rc;

unreg_input_dev:
	if (pon->pon_input)
		input_unregister_device(pon->pon_input);
free_input_dev:
	if (pon->pon_input)
		input_free_device(pon->pon_input);
	return rc;
}

static bool dload_on_uvlo;

static int qpnp_pon_debugfs_uvlo_dload_get(char *buf,
		const struct kernel_param *kp)
{
	struct qpnp_pon *pon = sys_reset_dev;
	int rc = 0;
	u8 reg;

	if (!pon)
		return -ENODEV;

	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
			QPNP_PON_XVDD_RB_SPARE(pon->base), &reg, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to read addr=%x, rc(%d)\n",
			QPNP_PON_XVDD_RB_SPARE(pon->base), rc);
		return rc;
	}

	return snprintf(buf, PAGE_SIZE, "%d",
			!!(QPNP_PON_UVLO_DLOAD_EN & reg));
}

static int qpnp_pon_debugfs_uvlo_dload_set(const char *val,
		const struct kernel_param *kp)
{
	struct qpnp_pon *pon = sys_reset_dev;
	int rc = 0;
	u8 reg;

	if (!pon)
		return -ENODEV;

	rc = param_set_bool(val, kp);
	if (rc) {
		pr_err("Unable to set bms_reset: %d\n", rc);
		return rc;
	}

	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
			QPNP_PON_XVDD_RB_SPARE(pon->base), &reg, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to read addr=%x, rc(%d)\n",
			QPNP_PON_XVDD_RB_SPARE(pon->base), rc);
		return rc;
	}

	reg &= ~QPNP_PON_UVLO_DLOAD_EN;
	if (*(bool *)kp->arg)
		reg |= QPNP_PON_UVLO_DLOAD_EN;

	rc = spmi_ext_register_writel(pon->spmi->ctrl, pon->spmi->sid,
			QPNP_PON_XVDD_RB_SPARE(pon->base), &reg, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to write to addr=%hx, rc(%d)\n",
				QPNP_PON_XVDD_RB_SPARE(pon->base), rc);
		return rc;
	}

	return 0;
}

static struct kernel_param_ops dload_on_uvlo_ops = {
	.set = qpnp_pon_debugfs_uvlo_dload_set,
	.get = qpnp_pon_debugfs_uvlo_dload_get,
};

module_param_cb(dload_on_uvlo, &dload_on_uvlo_ops, &dload_on_uvlo, 0644);

#if defined(CONFIG_DEBUG_FS)

static int qpnp_pon_debugfs_uvlo_get(void *data, u64 *val)
{
	struct qpnp_pon *pon = (struct qpnp_pon *) data;

	*val = pon->uvlo;

	return 0;
}

static int qpnp_pon_debugfs_uvlo_set(void *data, u64 val)
{
	struct qpnp_pon *pon = (struct qpnp_pon *) data;

	if (pon->pon_trigger_reason == PON_SMPL ||
		pon->pon_power_off_reason == QPNP_POFF_REASON_UVLO)
		panic("An UVLO was occurred.\n");
	pon->uvlo = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(qpnp_pon_debugfs_uvlo_fops, qpnp_pon_debugfs_uvlo_get,
			qpnp_pon_debugfs_uvlo_set, "0x%02llx\n");

static void qpnp_pon_debugfs_init(struct spmi_device *spmi)
{
	struct qpnp_pon *pon = dev_get_drvdata(&spmi->dev);
	struct dentry *ent;

	pon->debugfs = debugfs_create_dir(dev_name(&spmi->dev), NULL);
	if (!pon->debugfs) {
		dev_err(&pon->spmi->dev, "Unable to create debugfs directory\n");
	} else {
		ent = debugfs_create_file("uvlo_panic",
				S_IFREG | S_IWUSR | S_IRUGO,
				pon->debugfs, pon, &qpnp_pon_debugfs_uvlo_fops);
		if (!ent)
			dev_err(&pon->spmi->dev, "Unable to create uvlo_panic debugfs file.\n");
	}
}

static void qpnp_pon_debugfs_remove(struct spmi_device *spmi)
{
	struct qpnp_pon *pon = dev_get_drvdata(&spmi->dev);

	debugfs_remove_recursive(pon->debugfs);
}

#else

static void qpnp_pon_debugfs_init(struct spmi_device *spmi)
{}

static void qpnp_pon_debugfs_remove(struct spmi_device *spmi)
{}
#endif

#ifdef CONFIG_HUAWEI_PMU_DSM
#define REASON_MAX		16
/* monitor power-on and power-off reasons, if abnormal, notify to dsm*/
static void monitor_power_on_off_reason(struct qpnp_pon *pon)
{
	int rc, index, index_warm;
	u8 pon_sts = 0, buf[2];
	u16 poff_sts = 0;
	u16 pon_warm_reset_reason = 0;  //0x80A,0x80B

	if(!pon)
		return;

	/* warm reset reason reg value */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
				QPNP_PON_WARM_RESET_REASON1(pon->base), (u8 *)&pon_warm_reset_reason, 2);
	if (rc) {
		pr_err("Unable to read WARM_RESET_RESASON reg\n");
		return;
	}

	index_warm = ffs(pon_warm_reset_reason);
	if ((index_warm > REASON_MAX) || (index_warm < 0))
		index_warm = 0;

	/* PON reason */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
				QPNP_PON_REASON1(pon->base), &pon_sts, 1);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to read PON_RESASON1 reg\n");
		return;
	}

	index = ffs(pon_sts) - 1;
	if (index >= ARRAY_SIZE(qpnp_pon_reason) || index < 0){
		dev_dbg(&pon->spmi->dev,
			"PMIC@SID%d Power-on reason: Unknown, reg:0x%x\n",
			pon->spmi->sid, pon_sts);
	}else{
		if(PON_REA_SMPL == index){
			dev_info(&pon->spmi->dev,
				"PMIC@SID%d Power-on reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_pon_reason[index], pon_sts);
			/* if power on reason is SMPL, record this log, and notify to the dsm server*/
			DSM_PMU_LOG(pmu_dclient, DSM_ABNORMAL_POWERON_REASON_1,
				"PMIC@SID%d Power-on reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_pon_reason[index], pon_sts);
		}
	}

	/* POFF reason */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
				QPNP_POFF_REASON1(pon->base),
				buf, 2);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to read POFF_RESASON regs\n");
		return;
	}
	poff_sts = buf[0] | (buf[1] << 8);
	index = ffs(poff_sts) - 1;
	if (index >= ARRAY_SIZE(qpnp_poff_reason) || index < 0){
		dev_info(&pon->spmi->dev,
				"PMIC@SID%d: Unknown power-off reason, reg:0x%x\n",
				pon->spmi->sid, poff_sts);
	}else{
		if(POFF_REA_TFT == index){
			dev_info(&pon->spmi->dev,
				"PMIC@SID%d Power-off reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_poff_reason[index], poff_sts);
			/* if power on reason is TFT, record this log, and notify to the dsm server*/
			DSM_PMU_LOG(pmu_dclient, DSM_ABNORMAL_POWEROFF_REASON_1,
				"PMIC@SID%d Power-off reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_poff_reason[index], poff_sts);
		}

		if(POFF_REA_UVLO == index){
			dev_info(&pon->spmi->dev,
				"PMIC@SID%d Power-off reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_poff_reason[index], poff_sts);
			/* if power on reason is UVLO, record this log, and notify to the dsm server*/
			DSM_PMU_LOG(pmu_dclient, DSM_ABNORMAL_POWEROFF_REASON_2,
				"PMIC@SID%d Power-off reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_poff_reason[index], poff_sts);
		}

		if(POFF_REA_PMIC_OVETEMP == index){
			dev_info(&pon->spmi->dev,
				"PMIC@SID%d Power-off reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_poff_reason[index], poff_sts);
			/* if power on reason is PMIC_OVETEMP, record this log, and notify to the dsm server*/
			DSM_PMU_LOG(pmu_dclient, DSM_ABNORMAL_POWEROFF_REASON_3,
				"PMIC@SID%d Power-off reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_poff_reason[index], poff_sts);
		}

		if(POFF_REA_STAGE3 == index){
			dev_info(&pon->spmi->dev,
				"PMIC@SID%d Power-off reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_poff_reason[index], poff_sts);
			/* if power on reason is STAGE3, record this log, and notify to the dsm server*/
			DSM_PMU_LOG(pmu_dclient, DSM_ABNORMAL_POWEROFF_REASON_4,
				"PMIC@SID%d Power-off reason: %s, reg:0x%x\n",
				pon->spmi->sid, qpnp_poff_reason[index], poff_sts);
		}
	}
}

/* monitor pa and cpu temeraure, if abnormal, notify to dsm*/
static void monitor_pa_cpu_temperature(void)
{
	int pa_temp = 0, cpu_temp = 0;
	long tsen_temp;
	uint32_t tsensor_id2;
	uint32_t tsensor_id5;
	int ret = 0;

	pa_temp = dsm_get_pa_temp();
	cpu_temp = dsm_get_cpu_temp();

	if(PA_CPU_TEMP_THRESH < pa_temp){
		/* if pa_temp is over 70 degree, record this log, and notify to the dsm server*/
		DSM_PMU_LOG(pmu_dclient, DSM_PA_OVERTEMP,
			"pa_temp is high, over 70 degree: pa_temp = %d\n", pa_temp);
	}

	if(PA_CPU_TEMP_THRESH < cpu_temp){
		/* if cpu_temp is over 70 degree, record this log, and notify to the dsm server*/
		DSM_PMU_LOG(pmu_dclient, DSM_CPU_OVERTEMP,
			"cpu_temp is high, over 70 degree: cpu_temp = %d\n", cpu_temp);
	}

	tsensor_id2 = TSENS_ID2;
	ret = get_tsens_temp(tsensor_id2, &tsen_temp);
	if(ret < 0){
		pr_err("Unable to read temperature for tsen_id:%d. err:%d\n",
			tsensor_id2, ret);
		return;
	}

	if(TSENS_TEMP_THRESH < tsen_temp){
		/* if zone2(tsens2) temp is over 90 degree, record this log, and notify to the dsm server*/
		pr_info("zone2 is high, over 90 degree: tsen_temp = %ld\n", tsen_temp);
		DSM_PMU_LOG(pmu_dclient, DSM_THERMAL_ZONE2_OVERTEMP,
			"zone2 is high, over 90 degree: tsen_temp = %d\n", (int)tsen_temp);
	}

	tsensor_id5 = TSENS_ID5;
	ret = get_tsens_temp(tsensor_id5, &tsen_temp);
	if(ret < 0){
		pr_err("Unable to read temperature for tsen_id:%d. err:%d\n",
			tsensor_id5, ret);
		return;
	}

	if(TSENS_TEMP_THRESH < tsen_temp){
		/* if zone4(tsens5) temp is over 90 degree, record this log, and notify to the dsm server*/
		pr_info("zone4 is high, over 90 degree: tsen_temp = %ld\n", tsen_temp);
		DSM_PMU_LOG(pmu_dclient, DSM_THERMAL_ZONE4_OVERTEMP,
			"zone4 is high, over 90 degree: tsen_temp = %d\n", (int)tsen_temp);
	}
}

/*===========================================
FUNCTION: qpnp_read_ldo_status_reg
DESCRIPTION: this function is used to read ldo_x status reg value
IPNUT: pon: point to qpnp_pon; status_no: status reg addr offset;
           ldo_no: ldo number; ldo_val: pointer to ldo status value
RETURN:	0: success; none 0: read failed
=============================================*/
static int qpnp_read_ldo_status_reg(struct qpnp_pon *pon, int status_no, int ldo_no, u8 *ldo_val )
{
	int rc = 0;
	u8 *ldo_status_val;
	u8 sid = 0;
	u16 addr = 0;

	if(ldo_val == NULL){
		pr_info("ldo_val is NULL pointer\n");
		return -EINVAL;
	}

	ldo_status_val = ldo_val;
	sid = ((LDO1_ADDR_BASE + status_no + ldo_no*LDO_OFFSET) >> 16) & 0xF;
	addr = (LDO1_ADDR_BASE + status_no + ldo_no*LDO_OFFSET) & 0xFFFF;
	rc = spmi_ext_register_readl(pon->spmi->ctrl, sid,addr, ldo_status_val, 1);
	if (rc) {
		pr_err("Unable to read LDO status1 reg value\n");
		return rc;
	}
	return rc;
}

/* monitor LDO1 to LDO18 output voltage, if voltage is below VREG_OK threshold, notify to dsm*/
static void monitor_ldo_voltage(struct qpnp_pon *pon)
{
	int i = 0;
	int rc = 0;
	u8 ldo_status1[LDO_NUM] = {0}, ldo_status2[LDO_NUM] = {0};

	if (!pon)
		return;

	for(i = 0; i < LDO_NUM; i++){
		rc = qpnp_read_ldo_status_reg(pon, STATUS2_OFFSET, i, &ldo_status2[i]);
		if (rc) {
			pr_err("Unable to read LDO status2 reg value\n");
			return;
		}

		if(LDO_ON_MASK & ldo_status2[i]){/* if LDO is on, then check status1 value*/
			rc = qpnp_read_ldo_status_reg(pon, STATUS1_OFFSET, i, &ldo_status1[i]);
			if (rc) {
				pr_err("Unable to read LDO status1 reg value\n");
				return;
			}

			if(!(LDO_VOL_MASK & ldo_status1[i])){
				msleep(1); //delay 1 ms to ensure LDO voltage can keep stably
				/*after 1ms,read LDO_ON reg again to make sure LDO_ON en*/
				rc = qpnp_read_ldo_status_reg(pon, STATUS2_OFFSET, i, &ldo_status2[i]);
				if (rc) {
					pr_err("Unable to read LDO status2 reg value\n");
					return;
				}
				if(!(LDO_ON_MASK & ldo_status2[i])){
					return;
				}
				/* after 1 ms, read status1 reg again to check VREG_OK*/
				rc = qpnp_read_ldo_status_reg(pon, STATUS1_OFFSET, i, &ldo_status1[i]);
				if (rc) {
					pr_err("Unable to read LDO status1 reg value\n");
					return;
				}

				if(!(LDO_VOL_MASK & ldo_status1[i])){
				/* LDOx voltage is below VREG_OK threshold, save the log, and notify to dsm server*/
					pr_info("LDO_%d voltage is below VREG_OK threshold"
						"LDO status1 and status2 regs val: 0x%x, 0x%x\n",
						(i+1), ldo_status1[i], ldo_status2[i]);
					DSM_PMU_LOG(pmu_dclient, (DSM_LDO1_VOLTAGE_LOW+i),
						"LDO_%d voltage is below VREG_OK threshold"
						"LDO status1 and status2 regs val: 0x%x, 0x%x\n",
						(i+1), ldo_status1[i], ldo_status2[i]);
				}
			}
		}
	}
}

static void dsm_pmu_work_func(struct work_struct *work)
{
	static int check_poweron_off_reason_flag = 0;
	struct qpnp_pon *pon =
		container_of(work, struct qpnp_pon, dsm_pmu_work.work);

	if(!check_poweron_off_reason_flag){
		monitor_power_on_off_reason(pon);
		check_poweron_off_reason_flag = 1;
	}

	monitor_pa_cpu_temperature();

	monitor_ldo_voltage(pon);

	schedule_delayed_work(&pon->dsm_pmu_work, CHECK_PMU_STATUS_DELAY);
}
#endif
static int qpnp_pon_probe(struct spmi_device *spmi)
{
	struct qpnp_pon *pon;
	struct resource *pon_resource;
	struct device_node *itr = NULL;
	u32 delay = 0, s3_debounce = 0;
	int rc, sys_reset, index;
	u8 pon_sts = 0, buf[2];
	u16 poff_sts = 0;
	const char *s3_src;
	u8 s3_src_reg;
#ifdef  CONFIG_HUAWEI_KERNEL
	int pwrkpd_flag = 0;
#endif

	pon = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_pon),
							GFP_KERNEL);
	if (!pon) {
		dev_err(&spmi->dev, "Can't allocate qpnp_pon\n");
		return -ENOMEM;
	}

	sys_reset = of_property_read_bool(spmi->dev.of_node,
						"qcom,system-reset");
	if (sys_reset && sys_reset_dev) {
		dev_err(&spmi->dev, "qcom,system-reset property can only be specified for one device on the system\n");
		return -EINVAL;
	} else if (sys_reset) {
		sys_reset_dev = pon;
	}

	pon->spmi = spmi;

	/* get the total number of pon configurations */
	while ((itr = of_get_next_child(spmi->dev.of_node, itr)))
		pon->num_pon_config++;

	if (!pon->num_pon_config) {
		/* No PON config., do not register the driver */
		dev_err(&spmi->dev, "No PON config. specified\n");
		return -EINVAL;
	}

	pon->pon_cfg = devm_kzalloc(&spmi->dev,
			sizeof(struct qpnp_pon_config) * pon->num_pon_config,
								GFP_KERNEL);

	pon_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
	if (!pon_resource) {
		dev_err(&spmi->dev, "Unable to get PON base address\n");
		return -ENXIO;
	}
	pon->base = pon_resource->start;

	rc = qpnp_pon_store_and_clear_warm_reset(pon);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to store/clear WARM_RESET_REASONx registers\n");
		return rc;
	}

	/* PON reason */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
				QPNP_PON_REASON1(pon->base), &pon_sts, 1);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to read PON_RESASON1 reg\n");
		return rc;
	}

	boot_reason = ffs(pon_sts);

	index = ffs(pon_sts) - 1;
	cold_boot = !qpnp_pon_is_warm_reset();
	if (index >= ARRAY_SIZE(qpnp_pon_reason) || index < 0) {
		dev_info(&pon->spmi->dev,
			"PMIC@SID%d Power-on reason: Unknown and '%s' boot\n",
			pon->spmi->sid, cold_boot ? "cold" : "warm");
	} else {
		pon->pon_trigger_reason = index;
		dev_info(&pon->spmi->dev,
			"PMIC@SID%d Power-on reason: %s and '%s' boot\n",
			pon->spmi->sid, qpnp_pon_reason[index],
			cold_boot ? "cold" : "warm");
	}
#ifdef CONFIG_HUAWEI_KERNEL
    huawei_pon_regs[PON_REASON_INDEX] = pon_sts;
#endif	

	/* POFF reason */
	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
				QPNP_POFF_REASON1(pon->base),
				buf, 2);
	if (rc) {
		dev_err(&pon->spmi->dev, "Unable to read POFF_RESASON regs\n");
		return rc;
	}
	poff_sts = buf[0] | (buf[1] << 8);
	index = ffs(poff_sts) - 1;
	if (index >= ARRAY_SIZE(qpnp_poff_reason) || index < 0) {
		dev_info(&pon->spmi->dev,
				"PMIC@SID%d: Unknown power-off reason\n",
				pon->spmi->sid);
	} else {
		pon->pon_power_off_reason = index;
		dev_info(&pon->spmi->dev,
				"PMIC@SID%d: Power-off reason: %s\n",
				pon->spmi->sid,
				qpnp_poff_reason[index]);
	}
#ifdef CONFIG_HUAWEI_KERNEL
    huawei_pon_regs[POFF_REASON_INDEX] = poff_sts;
#endif

	if (pon->pon_trigger_reason == PON_SMPL ||
		pon->pon_power_off_reason == QPNP_POFF_REASON_UVLO) {
		if (of_property_read_bool(spmi->dev.of_node,
						"qcom,uvlo-panic"))
			panic("An UVLO was occurred.");
	}

	/* program s3 debounce */
	rc = of_property_read_u32(pon->spmi->dev.of_node,
				"qcom,s3-debounce", &s3_debounce);
#ifdef  CONFIG_HUAWEI_KERNEL
    /*if disable pwrkpd flag is true, set s3 timer to 128s*/
	pwrkpd_flag = hw_get_pwrkpd_flag();
	if (PON_DIS_PWRKPD_RESET == pwrkpd_flag)
	{
		s3_debounce = QPNP_PON_S3_TIMER_SECS_MAX;
	}
#endif
	if (rc) {
		if (rc != -EINVAL) {
			dev_err(&pon->spmi->dev, "Unable to read s3 timer\n");
			return rc;
		}
	} else {
		if (s3_debounce > QPNP_PON_S3_TIMER_SECS_MAX) {
			dev_info(&pon->spmi->dev,
				"Exceeded S3 max value, set it to max\n");
			s3_debounce = QPNP_PON_S3_TIMER_SECS_MAX;
		}

		/* 0 is a special value to indicate instant s3 reset */
		if (s3_debounce != 0)
			s3_debounce = ilog2(s3_debounce);

		/* s3 debounce is SEC_ACCESS register */
		rc = qpnp_pon_masked_write(pon, QPNP_PON_SEC_ACCESS(pon->base),
					0xFF, QPNP_PON_SEC_UNLOCK);
		if (rc) {
			dev_err(&spmi->dev, "Unable to do SEC_ACCESS\n");
			return rc;
		}

		rc = qpnp_pon_masked_write(pon, QPNP_PON_S3_DBC_CTL(pon->base),
				QPNP_PON_S3_DBC_DELAY_MASK, s3_debounce);
		if (rc) {
			dev_err(&spmi->dev, "Unable to set S3 debounce\n");
			return rc;
		}
	}

	/* program s3 source */
	s3_src = "kpdpwr-and-resin";
	rc = of_property_read_string(pon->spmi->dev.of_node,
				"qcom,s3-src", &s3_src);
	if (rc && rc != -EINVAL) {
		dev_err(&pon->spmi->dev, "Unable to read s3 timer\n");
		return rc;
	}

	if (!strcmp(s3_src, "kpdpwr"))
		s3_src_reg = QPNP_PON_S3_SRC_KPDPWR;
	else if (!strcmp(s3_src, "resin"))
		s3_src_reg = QPNP_PON_S3_SRC_RESIN;
	else if (!strcmp(s3_src, "kpdpwr-or-resin"))
		s3_src_reg = QPNP_PON_S3_SRC_KPDPWR_OR_RESIN;
	else /* default combination */
		s3_src_reg = QPNP_PON_S3_SRC_KPDPWR_AND_RESIN;

#ifdef  CONFIG_HUAWEI_KERNEL
	/*set s3 timer trigger source to pwrkpd if pwrkpd reset flag is true*/
	pwrkpd_flag = hw_get_pwrkpd_flag();
	if (PON_DIS_PWRKPD_RESET == pwrkpd_flag)
	{
		s3_src_reg = QPNP_PON_S3_SRC_KPDPWR_AND_RESIN;
	}
#endif
	/* S3 source is a write once register. If the register has
	 * been configured by bootloader then this operation will
	 * not be effective. */
	rc = qpnp_pon_masked_write(pon, QPNP_PON_S3_SRC(pon->base),
			QPNP_PON_S3_SRC_MASK, s3_src_reg);
	if (rc) {
		dev_err(&spmi->dev,
			"Unable to program s3 source\n");
		return rc;
	}

	dev_set_drvdata(&spmi->dev, pon);

	INIT_DELAYED_WORK(&pon->bark_work, bark_work_func);

#ifdef CONFIG_HUAWEI_PMU_DSM
	if (!pmu_dclient) {
		pmu_dclient = dsm_register_client(&dsm_pmu);
	}

	INIT_DELAYED_WORK(&pon->dsm_pmu_work, dsm_pmu_work_func);
	schedule_delayed_work(&pon->dsm_pmu_work, CHECK_PMU_STATUS_DELAY);
#endif

#ifdef CONFIG_HUAWEI_DSM
	init_timer(&lcd_pwr_status.lcd_dsm_t);
#endif
	/* register the PON configurations */
	rc = qpnp_pon_config_init(pon);
	if (rc) {
		dev_err(&spmi->dev,
			"Unable to intialize PON configurations\n");
		return rc;
	}

	rc = of_property_read_u32(pon->spmi->dev.of_node,
				"qcom,pon-dbc-delay", &delay);
	if (rc) {
		if (rc != -EINVAL) {
			dev_err(&spmi->dev, "Unable to read debounce delay\n");
			return rc;
		}
	} else {
		rc = qpnp_pon_set_dbc(pon, delay);
		if (rc)
			return rc;
	}

	rc = device_create_file(&spmi->dev, &dev_attr_debounce_us);
	if (rc) {
		dev_err(&spmi->dev, "sys file creation failed\n");
		return rc;
	}

	/* config whether store the hard reset reason */
	pon->store_hard_reset_reason = of_property_read_bool(
					spmi->dev.of_node,
					"qcom,store-hard-reset-reason");

	qpnp_pon_debugfs_init(spmi);
	return rc;
}

static int qpnp_pon_remove(struct spmi_device *spmi)
{
	struct qpnp_pon *pon = dev_get_drvdata(&spmi->dev);

	device_remove_file(&spmi->dev, &dev_attr_debounce_us);

#ifdef CONFIG_HUAWEI_PMU_DSM
	cancel_delayed_work_sync(&pon->dsm_pmu_work);
#endif
	cancel_delayed_work_sync(&pon->bark_work);

	if (pon->pon_input)
		input_unregister_device(pon->pon_input);
	qpnp_pon_debugfs_remove(spmi);
	return 0;
}

#ifdef CONFIG_HUAWEI_PMU_DSM
static int qpnp_pon_suspend(struct device *dev)
{
	struct qpnp_pon *pon = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&pon->dsm_pmu_work);

	return 0;
}
static int qpnp_pon_resume(struct device *dev)
{
	struct qpnp_pon *pon = dev_get_drvdata(dev);

	schedule_delayed_work(&pon->dsm_pmu_work,
				msecs_to_jiffies(0));

	return 0;
}

static const struct dev_pm_ops qpnp_pon_pm_ops = {
	.suspend	= qpnp_pon_suspend,
	.resume	= qpnp_pon_resume,
};
#endif
static struct of_device_id spmi_match_table[] = {
	{ .compatible = "qcom,qpnp-power-on", },
	{}
};

static struct spmi_driver qpnp_pon_driver = {
	.driver		= {
		.name	= "qcom,qpnp-power-on",
		.of_match_table = spmi_match_table,
#ifdef CONFIG_HUAWEI_PMU_DSM
		.pm		= &qpnp_pon_pm_ops,
#endif
	},
	.probe		= qpnp_pon_probe,
	.remove		= qpnp_pon_remove,
};

static int __init qpnp_pon_init(void)
{
	return spmi_driver_register(&qpnp_pon_driver);
}
module_init(qpnp_pon_init);

static void __exit qpnp_pon_exit(void)
{
	return spmi_driver_unregister(&qpnp_pon_driver);
}
module_exit(qpnp_pon_exit);

MODULE_DESCRIPTION("QPNP PMIC POWER-ON driver");
MODULE_LICENSE("GPL v2");
