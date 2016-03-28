/*
 * Copyright (C) 2012-2015 HUAWEI
 * Author: L J H
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_BQ24296M_CHARGER_H
#define _LINUX_BQ24296M_CHARGER_H

#define MIN_INT                (1 << 31)
#define MAX_INT                (~(1 << 31))
#define TEMP_CTL_POINTS_NUM    (12)
#define TEMP_CTL_TYPE_NUM      (4)
#define TEMP_AREA_NUM          (13)
#define BQ2419x_EVENT_UNKOWN                   (0x00)
#define BQ2419x_NO_CHARGER_SOURCE              (0x00)
#define BQ2419x_NOT_CHARGING                   (0x10)
#define BQ2419x_START_CHARGING                 (0x20)
#define BQ2419x_START_AC_CHARGING              (0x30)
#define BQ2419x_START_USB_CHARGING             (0x40)
#define BQ2419x_CHARGE_DONE                    (0x50)
#define BQ2419x_STOP_CHARGING                  (0x60)
#define POWER_SUPPLY_STATE_FAULT               (0x70)
#define POWER_SUPPLY_OVERVOLTAGE               (0x80)
#define POWER_SUPPLY_WEAKSOURCE                (0x90)
#define BQ2419x_RESET_TIMER                    (0x38)


/*Input Source Control Register REG00(default 00110xxx, or 3x)*/
#define INPUT_SOURCE_REG00    0x00

#define BQ24296M_EN_HIZ_SHIFT          7
#define EN_HIZ                          (1)
#define DIS_HIZ                         (0)
#define BQ24296M_VINDPM_SHIFT          3
#define VINDPM_MIN_3880              (3880)
#define VINDPM_4200                  (4200)
#define VINDPM_4360                  (4360)
#define VINDPM_4440                  (4440)
#define VINDPM_4520                  (4520)
#define VINDPM_4600                  (4600)
#define VINDPM_MAX_5080              (5080)
#define VINDPM_STEP_80                 (80)
#define IINLIM_100                    (100)
#define IINLIM_150                    (150)
#define IINLIM_500                    (500)
#define IINLIM_900                    (900)
#define IINLIM_1000                  (1000)
#define IINLIM_1200                  (1200)
#define IINLIM_1500                  (1500)
#define IINLIM_2000                  (2000)
#define IINLIM_3000                  (3000)

/*Power-On Configuration Register REG01(default 00011011, or 1B)*/
#define POWER_ON_CONFIG_REG01    0x01

#define REG_RST               (1 << 7)
#define WATCHDOG_TIMER_RST    (1 << 6)
#define BQ_OTG_EN            (1<<5)
#define BQ24296M_EN_CHARGER_SHIFT    4
#define DIS_CHARGER                (0)
#define EN_CHARGER                 (1)
#define EN_CHARGER_OTG             (2)
#define BQ24296M_SYS_MIN_SHIFT       1
#define SYS_MIN_MIN_3000        (3000)
#define SYS_MIN_3500            (3500)
#define SYS_MIN_MAX_3700        (3700)
#define SYS_MIN_STEP_100        (100)
#define BOOST_LIM_1500            (1)
#define BOOST_LIM_1000             (0)

/*Charge Current Control Register REG02 (bq24190/191/192/193
   default 01100000, or 60; bq24192I default 00100000, or 20)*/
#define CHARGE_CURRENT_REG02    0x02

#define BQ24296M_ICHG_SHIFT    2
#define BQ24296M_BCOLD_SHIFT 1
#define ICHG_LOW_TEMP    (100)
#define ICHG_100         (100)
#define ICHG_512         (512)
#define ICHG_640         (640)
#define ICHG_700         (704)
#define ICHG_820         (820)/*768*/
#define ICHG_900         (900)/*896*/
#define ICHG_1000       (1000)/*960*/
#define ICHG_1024       (1024)
#define ICHG_1200       (1200)/*1152*/
#define ICHG_1500       (1500)/*1472*/
#define ICHG_1536       (1536)
#define ICHG_2000       (2000)/*1984*/
#define ICHG_2048       (2048)
#define ICHG_2500       (2500)/*2496*/
#define ICHG_2944       (2944)
#define ICHG_3000       (3000)/*3008*/
#define ICHG_MAX        (4544)
#define ICHG_STEP_64      (64)
#define BCOLD_0            (0)/*yp. 76% of REGN or -10.C w/ 103AT thermistor*/
#define BCOLD_1           (1)/*Typ. 79% of REGN or -20.C w/ 103AT thermistor*/
#define EN_FORCE_20PCT	   (1)   /*fast charge current as 20% of programmed value in REG02*/
#define DIS_FORCE_20PCT	   (0)   /* fast charge current as programmed value in REG02*/

/*Pre-Charge/Termination Current Control Register REG 03 (default 00010001, or 0x11)*/
#define PRECHARGE_TERM_CURRENT_REG03  0x03

#define BQ24296M_IPRECHRG_SHIFT    4
#define IPRECHRG_MIN_128    (128)
#define IPRECHRG_256        (256)
#define IPRECHRG_640        (640)
#define IPRECHRG_MAX_2048  (2048)
#define IPRECHRG_STEP_128   (128)
#define ITERM_MIN_128       (128)
#define ITERM_256           (256)
#define ITERM_MAX_1024     (1024)
#define ITERM_STEP_128      (128)
#define ITERM_MAX_512     (512)

/*Charge Voltage Control Register REG04 (default 10110010, or 0xB2))*/
#define CHARGE_VOLTAGE_REG04    0x04

#define BQ24296M_VCHARGE_SHIFT             2
#define VCHARGE_MIN_3504              (3504)
#define VCHARGE_4100                  (4100)
#define VCHARGE_4200                  (4208) //(4.192)
#define VCHARGE_4300                  (4304) //(4.288)
#define VCHARGE_4350                  (4352) //(4.336)
#define VCHARGE_MAX_4400              (4400)
#define VCHARGE_STEP_16               (16)

#define BATLOWV_3000                 (1 << 1)
#define BATLOWV_2800                 (0 << 1)
#define VRECHG_300                   (1 << 0)
#define VRECHG_100                   (0 << 0)

/*Charge Termination/Timer Control Register REG05 (default 10011010, or 9A)*/
#define CHARGE_TERM_TIMER_REG05    0x05

#define BQ24296M_EN_TERM_SHIFT       7
#define EN_TERM                         (1)
#define DIS_TERM                        (0)
#define TERM_STAT                  (0 << 6)
#define WATCHDOG_DIS               (0 << 4)
#define WATCHDOG_40                (1 << 4)
#define WATCHDOG_80                (1 << 5)
#define BQ24296M_EN_TIMER_SHIFT      3
#define EN_TIMER                        (1)
#define DIS_TIMER                       (0)
#define CHG_TIMER_8                (1 << 1)
#define CHG_TIMER_12               (2 << 1)
#define CHG_TIMER_20               (3 << 1)
//#define JEITA_ISET_HALF_ICHG            (0)
//#define JEITA_ISET                      (1)

/*Boost Voltage/Thermal Regulation Control Register REG06
   (default 01110011, or 0x73))*/

#define THERMAL_REGUALTION_REG06    0x06
#define BOOSTV_MIN_4550 4550
#define BOOSTV_DEFAULT		4998
#define BOOSTV_MAX_5510 5510
#define BOOSTV_SETP_64 64
#define BQ24296M_BOOSTV_SHIFT   4
#define BHOT_DEFAULT		0
#define TREG_120                   (3)
#define TREG_100                   (2)
#define TREG_80                    (1)
#define TREG_60                    (0)

/*Misc Operation Control Register REG07 (default 01001011, or 4B))*/
#define MISC_OPERATION_REG07   0x07

#define BQ24296M_DPDM_EN_SHIFT                7
#define DPDM_EN                            (1)
#define DPDM_DIS                           (0)
#define TMR2X_EN                      (1 << 6)
#define BQ24296M_BATFET_EN_SHIFT              5
#define EN_BATFET                          (0)
#define DIS_BATFET                         (1)
//#define JEITA_VSET_4050               (0 << 4)
//#define JEITA_VSET_4200               (1 << 4)
#define CHRG_FAULT_INT_DIS            (0 << 1)
#define CHRG_FAULT_INT_EN             (1 << 1)
#define BAT_FAULT_INT_DIS             (0 << 0)
#define BAT_FAULT_INT_EN              (1 << 0)

/*System Status Register REG08*/
#define SYS_STATUS_REG08    0x08

#define BQ24296M_VBUS_STAT_UNKNOWM        (0x00)
#define BQ24296M_VBUS_STAT_USB_HOST       (0x40)
#define BQ24296M_VBUS_STAT_ADATPTER       (0x80)
#define BQ24296M_VBUS_STAT_OTG            (0xC0)
#define BQ24296M_CHGR_STAT_NOT_CHARGING   (0x00)
#define BQ24296M_CHGR_STAT_PRE_CHARGING   (0x10)
#define BQ24296M_CHGR_STAT_FAST_CHARGING  (0x20)
#define BQ24296M_CHGR_STAT_CHAEGE_DONE    (0x30)
#define BQ24296M_NOT_DPM_STAT             (0x00)
#define BQ24296M_DPM_STAT                 (0x08)
#define BQ24296M_NOT_PG_STAT              (0x00)
#define BQ2429M_PG_STAT                  (0x04)
#define BQ24296M_THERM_STAT_NOEMAL        (0x00)
#define BQ24296M_THERM_STAT_TREG          (0x02)
#define BQ24296M_NOT_VSYS_STAT            (0x00)
#define BQ24296M_VSYS_STAT                (0x01)



/*Fault Register REG09*/
#define CHARGER_FAULT_REG09    0x09

#define BQ24296M_WATCHDOG_FAULT          (0x80)
#define BQ24296M_OTG_FAULT               (0x40)
#define BQ24296M_POWER_SUPPLY_OVP        (0x10)
#define BQ24296M_POWER_SUPPLY_OVP_MASK   (0x30)
#define BQ24296M_THERMAL_SHUTDOWM        (0x20)
#define BQ24296M_CHRG_TIMER_EXPIRED      (0x30)
#define BQ24296M_BAT_FAULT_OVP           (0x08)
#define BQ2419x_NTC_TS1_COLD            (0x01)
#define BQ2419x_NTC_TS1_HOT             (0x02)
#define BQ2419x_NTC_TS2_COLD            (0x03)
#define BQ2419x_NTC_TS2_HOT             (0x04)
#define BQ2419x_NTC_TS1_TS2_COLD        (0x05)
#define BQ2419x_NTC_TS1_TS2_HOT         (0x06)
#define BQ2419x_NTC_TS1_TS2_HOT_COLD    (0x07)

/*Vender / Part / Revision Status Register REG0A*/

#define PART_REVISION_REG0A     0x0A

#define BQ24296M                (0x20)

#define BQ24196        (0x28)
#define BQ_VID_MASK        (0x38)

#define TS_PROFILE_WINDOW       (0x00)
#define TS_PROFILE_JEITA        (1 << 2)

#define BQ24296M_WATCHDOG_TIMEOUT    (30000)

/*two stage charger*/
#define TWO_STAGE_CHARGE_FIRST_STAGE                 (0x00)
#define TWO_STAGE_CHARGE_SECOND_STAGE                 (0x01)

/*set gpio_074 to control CE pin to disable/enable bq24161 IC*/
#define ENABLE_BQ2419x_CHARGER        (GPIO_9_2)

/*(-10 ) battery temperature is -10 degree*/
#define BQ24296M_COLD_BATTERY_THRESHOLD     (-10)
 /*(0 ) battery temperature is 0 degree*/
#define BQ24296M_COOL_BATTERY_THRESHOLD      (0)
 /*( 5 ) battery temperature is 5 degree*/
#define BQ24296M_BATTERY_THRESHOLD_5       (5)
 /*( 10 ) battery temperature is 10 degree*/
#define BQ24296M_BATTERY_THRESHOLD_10       (10)
 /*( 45 ) battery temperature is 45 degree*/
#define BQ24296M_WARM_BATTERY_THRESHOLD     (45)
 /*( 50 ) battery temperature is 50 degree*/
#define BQ24296M_HOT_BATTERY_THRESHOLD      (50)
 /*( 3 ) battery temperature offset is 3 degree*/
#define BQ24296M_TEMPERATURE_OFFSET          (3)

 /* default battery capacity for error that can't get Capacity */
#define BQ24296M_DEFAULT_CAPACITY            (2000)
 /* default charge parameter when low temperature for error that can't get the parameter  */
#define DEFAULT_CHARGE_PARAM_LOW_TEMP       (3)

 /*(3.0V) battery preconditioning voltage is 3.0V*/

 /*low temperature charge termination voltage*/
#define BQ24296M_LOW_TEMP_TERM_VOLTAGE    (4000)
#define BQ24296M_LOW_TEMP_NOT_CHARGING_VOLTAGE    (4100)
#define BQ24296M_NORNAL_ICHRG_VOLTAGE     (3400)
#define CAPACITY_LEVEL_HIGH_THRESHOLD    (90)
#define POWR_SUPPLY_BATTERY_NAME   "battery"
#define POWR_SUPPLY_BMS_NAME    "bms"
#define BATT_DEFAULT_TEMP		250
#define BATT_DEFAULT_VOL		3800
#define BATT_DEFAULT_SOC		50
#define BATT_FULL			100 //battery full capactiy
#define SUSPEND_CURRENT		2
#define SDP_CURRENT_LIMIT		500
#define RT_DISCHARGE 		(1)
#define BQ24296_TEMPERATURE_OFFSET        (20)
#define BATT_TEMP_MAX    600
#define BQ24296_REG_NUM                              (11)
enum usb_charger_type
{
    CHARGER_TYPE_USB = 0,      //SDP
    CHARGER_TYPE_BC_USB,       //CDP
    CHARGER_TYPE_NON_STANDARD, //UNKNOW
    CHARGER_TYPE_STANDARD,     //DCP
    CHARGER_REMOVED,           //not connected
    USB_EVENT_OTG_ID,
};

struct bq24296m_temp_control_info {
    int    cold_bat_degree;    /*lowest temperature to stop charging */
    int    cool_bat_degree;    /*cool temprature to limit charging current and voltage*/
    int    imaxma_cool_bat;    /* max battery charging input current in ma */
    int    vmaxmv_cool_bat;	    /* max battery terminate voltage in mv*/
    int    warm_bat_degree;    /*warm temprature to limit charging current and voltage*/
    int    imaxma_warm_bat;    /* max battery charging input current in ma */
    int    vmaxmv_warm_bat;    /* max battery terminate voltage in mv */
    int    hot_bat_degree;    /*highest temperature to stop charging */
};

extern struct bq24296m_temp_control_info bq24296_temp_info;

/*Remove unuse code*/
int get_bq_charge_status(void);

/*Remove unuse code*/
#endif

