/*
 *
 * Zinitix zt7538 touch driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
 #ifndef __ZT7548_H__
#define  __ZT7548_H__

 #ifdef __cplusplus
extern "C" {
#endif
  
#include "stdlib.h"
#include "stdbool.h"

#define ZT7548_IC_CHIP_CODE	0xE548
#define ZT7548_SLAVE_ADDR 0x20


#define SEC_FACTORY_TEST
/* for run_read_all in factory cmd */
typedef void (*run_func_t)(void *);
enum data_type {
  DATA_UNSIGNED_CHAR,
  DATA_SIGNED_CHAR,
  DATA_UNSIGNED_SHORT,
  DATA_SIGNED_SHORT,
  DATA_UNSIGNED_INT,
  DATA_SIGNED_INT,
};

#ifdef CONFIG_SEC_FACTORY
#define REPORT_2D_Z
#endif

#ifdef REPORT_2D_Z
#define REAL_Z_MAX		3000
#define ZT7538_REAL_WIDTH		0x03A6
#endif

#define SUPPORTED_PALM_TOUCH
#define USE_CHECKSUM
#define TSP_MUIC_NOTIFICATION
#define ZINITIX_I2C_CHECKSUM

#define TS_DRVIER_VERSION		"1.0.18_1"
#define ZT7538_TS_DEVICE		"zt7538"

#define TOUCH_POINT_MODE		0
#define ZINITIX_MISC_DEBUG		1
#define CHECK_HWID			0
#define ZINITIX_DEBUG			0
#define TSP_INIT_TEST_RATIO		100
#define MAX_SUPPORTED_FINGER_NUM	5	/* max 10 */

#define MAX_SUPPORTED_BUTTON_NUM	2	/* max 8 */
#define SUPPORTED_BUTTON_NUM		0

/* resolution offset */
#define ABS_PT_OFFSET			(-1)
#define TOUCH_FORCE_UPGRADE		1
#define CHIP_OFF_DELAY			50	/*ms*/
#define DELAY_FOR_SIGNAL_DELAY		30	/*us*/
#define DELAY_FOR_TRANSCATION		50
#define DELAY_FOR_POST_TRANSCATION	10
#define RAWDATA_DELAY_FOR_HOST		100

/* PMIC Regulator based supply to TSP */
#define TSP_REGULATOR_SUPPLY		1
/* gpio controlled LDO based supply to TSP */
#define TSP_LDO_SUPPLY				0

/* ESD Protection */
/*second : if 0, no use. if you have to use, 3 is recommended*/
#define ESD_TIMER_INTERVAL			1
#define SCAN_RATE_HZ				100
#define CHECK_ESD_TIMER				3

#define DEF_OPTIONAL_STATE_CHECK

/*Test Mode (Monitoring Raw Data) */

#define TSP_INIT_TEST_RATIO  100

#define	SEC_MUTUAL_AMP_V_SEL	0x0232

#define	CHIP_ON_DELAY			200	/*ms*/
#define FIRMWARE_ON_DELAY		50	/*ms*/
#define	SEC_DND_N_COUNT			15
#define	SEC_DND_U_COUNT			18
#define	SEC_DND_FREQUENCY		169

#define	SEC_HFDND_N_COUNT		15
#define	SEC_HFDND_U_COUNT		18
#define	SEC_HFDND_FREQUENCY		112
#define	SEC_SX_AMP_V_SEL		0x0434
#define	SEC_SX_SUB_V_SEL		0x0055
#define	SEC_SY_AMP_V_SEL		0x0232
#define	SEC_SY_SUB_V_SEL		0x0022
#define	SEC_SHORT_N_COUNT		2
#define	SEC_SHORT_U_COUNT		1

//DND
#define SEC_DND_CP_CTRL_L			0x1fb3
#define SEC_DND_V_FORCE				0
#define SEC_DND_AMP_V_SEL			0x0141

#define MAX_RAW_DATA_SZ				36*22
#define MAX_TRAW_DATA_SZ	\
	(MAX_RAW_DATA_SZ + 4*MAX_SUPPORTED_FINGER_NUM + 2)


#define TOUCH_SEC_MODE				48
#define TOUCH_REF_MODE				10
#define TOUCH_NORMAL_MODE			5
#define TOUCH_DELTA_MODE			3
#define TOUCH_REFERENCE_MODE			8
#define TOUCH_DND_MODE				11
#define TOUCH_RXSHORT_MODE			12
#define TOUCH_TXSHORT_MODE			13

#define	PALM_REPORT_WIDTH	200
#define	PALM_REJECT_WIDTH	255

#define TOUCH_V_FLIP	0x01
#define TOUCH_H_FLIP	0x02
#define TOUCH_XY_SWAP	0x04

#define TSP_NORMAL_EVENT_MSG	1
#define I2C_RETRY_TIMES		8

/*  Other Things */
#define INIT_RETRY_CNT		3
#define I2C_SUCCESS		0
#define I2C_FAIL		1

#define TOUCH_IOCTL_BASE			0xbc
#define TOUCH_IOCTL_GET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 0, int)
#define TOUCH_IOCTL_SET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 1, int)
#define TOUCH_IOCTL_GET_CHIP_REVISION		_IOW(TOUCH_IOCTL_BASE, 2, int)
#define TOUCH_IOCTL_GET_FW_VERSION		_IOW(TOUCH_IOCTL_BASE, 3, int)
#define TOUCH_IOCTL_GET_REG_DATA_VERSION	_IOW(TOUCH_IOCTL_BASE, 4, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_SIZE		_IOW(TOUCH_IOCTL_BASE, 5, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_DATA		_IOW(TOUCH_IOCTL_BASE, 6, int)
#define TOUCH_IOCTL_START_UPGRADE		_IOW(TOUCH_IOCTL_BASE, 7, int)
#define TOUCH_IOCTL_GET_X_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 8, int)
#define TOUCH_IOCTL_GET_Y_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 9, int)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 10, int)
#define TOUCH_IOCTL_SET_RAW_DATA_MODE		_IOW(TOUCH_IOCTL_BASE, 11, int)
#define TOUCH_IOCTL_GET_RAW_DATA		_IOW(TOUCH_IOCTL_BASE, 12, int)
#define TOUCH_IOCTL_GET_X_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 13, int)
#define TOUCH_IOCTL_GET_Y_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 14, int)
#define TOUCH_IOCTL_HW_CALIBRAION		_IOW(TOUCH_IOCTL_BASE, 15, int)
#define TOUCH_IOCTL_GET_REG			_IOW(TOUCH_IOCTL_BASE, 16, int)
#define TOUCH_IOCTL_SET_REG			_IOW(TOUCH_IOCTL_BASE, 17, int)
#define TOUCH_IOCTL_SEND_SAVE_STATUS		_IOW(TOUCH_IOCTL_BASE, 18, int)
#define TOUCH_IOCTL_DONOT_TOUCH_EVENT		_IOW(TOUCH_IOCTL_BASE, 19, int)

/* Register Map*/
#define ZT7538_SWRESET_CMD				0x0000
#define ZT7538_WAKEUP_CMD				0x0001
#define ZT7538_IDLE_CMD					0x0004
#define ZT7538_SLEEP_CMD				0x0005
#define ZT7538_CLEAR_INT_STATUS_CMD			0x0003
#define ZT7538_CALIBRATE_CMD				0x0006
#define ZT7538_SAVE_STATUS_CMD				0x0007
#define ZT7538_SAVE_CALIBRATION_CMD			0x0008
#define ZT7538_RECALL_FACTORY_CMD			0x000f
#define ZT7538_THRESHOLD				0x0020
#define ZT7538_DEBUG_REG				0x0115 /* 0~7 */
#define ZT7538_TOUCH_MODE				0x0010
#define ZT7538_CHIP_REVISION				0x0011
#define ZT7538_FIRMWARE_VERSION				0x0012
#define ZT7538_MINOR_FW_VERSION				0x0121
#define ZT7538_VENDOR_ID				0x001C
#define ZT7538_HW_ID					0x0014
#define ZT7538_DATA_VERSION_REG				0x0013
#define ZT7538_SUPPORTED_FINGER_NUM			0x0015
#define ZT7538_EEPROM_INFO				0x0018
#define ZT7538_INITIAL_TOUCH_MODE			0x0019
#define ZT7538_TOTAL_NUMBER_OF_X			0x0060
#define ZT7538_TOTAL_NUMBER_OF_Y			0x0061
#define ZT7538_DELAY_RAW_FOR_HOST			0x007f
#define ZT7538_BUTTON_SUPPORTED_NUM			0x00B0
#define ZT7538_BUTTON_SENSITIVITY			0x00B2
#define ZT7538_DUMMY_BUTTON_SENSITIVITY			0X00C8
#define ZT7538_X_RESOLUTION				0x00C0
#define ZT7538_Y_RESOLUTION				0x00C1
#define ZT7538_POINT_STATUS_REG				0x0080
#define ZT7538_ICON_STATUS_REG				0x00AA

#define ZT7538_MUTUAL_AMP_V_SEL				0x02F9
#define ZT7538_DND_SHIFT_VALUE				0x012B
#define ZT7538_AFE_FREQUENCY				0x0100
#define ZT7538_DND_N_COUNT				0x0122
#define ZT7538_DND_U_COUNT				0x0135
#define ZT7538_DND_V_FORCE				0x02F1
#define ZT7538_DND_AMP_V_SEL				0x02F9
#define ZT7538_DND_CP_CTRL_L				0x02bd
#define ZT7538_RAWDATA_REG				0x0200
#define ZT7538_EEPROM_INFO_REG				0x0018
#define ZT7538_INT_ENABLE_FLAG				0x00f0
#define ZT7538_PERIODICAL_INTERRUPT_INTERVAL		0x00f1
#define ZT7538_BTN_WIDTH				0x0316
#define ZT7538_CHECKSUM_RESULT				0x012c
#define ZT7538_INIT_FLASH				0x01d0
#define ZT7538_WRITE_FLASH				0x01d1
#define ZT7538_READ_FLASH				0x01d2
#define ZINITIX_INTERNAL_FLAG_02			0x011e
#define ZT7538_DEBUG_REGSITER				0x0115
#define ZT7538_OPTIONAL_SETTING				0x0116
#define ZT75XX_SX_AMP_V_SEL				0x02DF
#define ZT75XX_SX_SUB_V_SEL				0x02E0
#define ZT75XX_SY_AMP_V_SEL				0x02EC
#define ZT75XX_SY_SUB_V_SEL				0x02ED
#define ZT75XX_RESOLUTION_EXPANDER			0x0186

/* Interrupt & status register flag bit
-------------------------------------------------
*/
#define BIT_PT_CNT_CHANGE	0
#define BIT_DOWN		1
#define BIT_MOVE		2
#define BIT_UP			3
#define BIT_PALM		4
#define BIT_PALM_REJECT		5
#define RESERVED_0		6
#define RESERVED_1		7
#define BIT_WEIGHT_CHANGE	8
#define BIT_PT_NO_CHANGE	9
#define BIT_REJECT		10
#define BIT_PT_EXIST		11
#define RESERVED_2		12
#define BIT_MUST_ZERO		13
#define BIT_DEBUG		14
#define BIT_ICON_EVENT		15

/* button */
#define BIT_O_ICON0_DOWN	0
#define BIT_O_ICON1_DOWN	1
#define BIT_O_ICON2_DOWN	2
#define BIT_O_ICON3_DOWN	3
#define BIT_O_ICON4_DOWN	4
#define BIT_O_ICON5_DOWN	5
#define BIT_O_ICON6_DOWN	6
#define BIT_O_ICON7_DOWN	7

#define BIT_O_ICON0_UP		8
#define BIT_O_ICON1_UP		9
#define BIT_O_ICON2_UP		10
#define BIT_O_ICON3_UP		11
#define BIT_O_ICON4_UP		12
#define BIT_O_ICON5_UP		13
#define BIT_O_ICON6_UP		14
#define BIT_O_ICON7_UP		15

#define SUB_BIT_EXIST		0
#define SUB_BIT_DOWN		1
#define SUB_BIT_MOVE		2
#define SUB_BIT_UP		3
#define SUB_BIT_UPDATE		4
#define SUB_BIT_WAIT		5

#define zinitix_bit_set(val, n)		((val) &= ~(1<<(n)), (val) |= (1<<(n)))
#define zinitix_bit_clr(val, n)		((val) &= ~(1<<(n)))
#define zinitix_bit_test(val, n)	((val) & (1<<(n)))
#define zinitix_swap_v(a, b, t)		((t) = (a), (a) = (b), (b) = (t))
#define zinitix_swap_16(s)		(((((s) & 0xff) << 8) | (((s) >> 8) & 0xff)))

void ZT7548_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ZT7548_H__ */
