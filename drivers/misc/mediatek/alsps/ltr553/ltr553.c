/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/ltr553.c - LTR553 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Change log
 *
 * Date		Author		Description
 * ----------------------------------------------------------------------------
 * 11/17/2011	chenqy		Initial modification from ltr502.
 * 01/03/2012	chenqy		Fix logical error in sensor enable function.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <linux/wakelock.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "ltr553.h"
#include <linux/hwmsen_helper.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
//@hfs---start  20130712 for ps up system cannot wake
#include <linux/wakelock.h>
//@hfs---end
#include "cust_ltr553.h"

#define POWER_NONE_MACRO -1

extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

/******************************************************************************
 * configuration
*******************************************************************************/
#define LTR553_DEV_NAME		"LTR553"
/*----------------------------------------------------------------------------*/
#define APS_TAG	"[ALS/PS] "
#define APS_DEBUG
#if defined(APS_DEBUG)
#define APS_FUN(f)		printk(KERN_DEBUG APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)	printk(KERN_ERR APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)	printk(KERN_DEBUG APS_TAG "%s(%d):" fmt, __FUNCTION__, __LINE__, ##args)
#define APS_DBG(fmt, args...)	printk(KERN_DEBUG fmt, ##args)
#else
#define APS_FUN(f)
#define APS_ERR(fmt, args...)
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#endif

struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;
static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
//add for ps calibration   end

#define ALS_DO_AVR    //als do average
#ifdef ALS_DO_AVR
static int als_times = 0;
static int als_temp[ALS_AVR_COUNT] = {0};
static int first_cycle = 1;
#endif

#ifdef WT_ALS_SWITCH_AUTO
static int first_switch = 1;
#endif

#ifdef WT_ALS_ENABLE_SKIP
static int als_lux_pre = -1;
static int als_skip_count = CUST_ALS_SKIP_COUNT;
#endif

static bool ps_interrupt_flag = false;
/******************************************************************************
 * extern functions
*******************************************************************************/
static struct i2c_client *ltr553_i2c_client = NULL;

//static struct wake_lock ps_wake_lock;
//static int ps_wakeup_timeout = 3;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr553_i2c_id[] = {{LTR553_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_ltr553={ I2C_BOARD_INFO(LTR553_DEV_NAME, (LTR553_I2C_SLAVE_ADDR>>1))};

/*----------------------------------------------------------------------------*/
static int ltr553_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ltr553_i2c_remove(struct i2c_client *client);
static int ltr553_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int ltr553_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr553_i2c_resume(struct i2c_client *client);

static struct ltr553_priv *g_ltr553_ptr = NULL;

/*----------------------------------------------------------------------------*/
//@hfs---start 20130712 for ps up system cannot wake
static struct wake_lock  ps_lock;
//@hfs---end
typedef enum {
	CMC_TRC_APS_DATA	= 0x0002,
	CMC_TRC_EINT		= 0x0004,
	CMC_TRC_IOCTL		= 0x0008,
	CMC_TRC_I2C		= 0x0010,
	CMC_TRC_CVT_ALS		= 0x0020,
	CMC_TRC_CVT_PS		= 0x0040,
	CMC_TRC_DEBUG		= 0x8000,
} CMC_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS		= 1,
	CMC_BIT_PS		= 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
struct ltr553_priv {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct work_struct eint_work;
	//struct timer_list first_read_ps_timer;
	//struct timer_list first_read_als_timer;

	/*misc*/
	atomic_t	trace;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on;	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;	/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;	/*indicates if the debounce is on*/
	atomic_t	ps_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
    atomic_t    ps_thd_val_h;   /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_l;   /*the cmd value can't be read, stored in ram*/


	/*data*/
	// u8		als;
	// u8		ps;
	int		als;
	int		ps;
	u8		_align;
	u16		als_level_num;
	u16		als_value_num;
	u32		als_level[C_CUST_ALS_LEVEL-1];
	u32		als_value[C_CUST_ALS_LEVEL];
#ifdef WT_ALS_SWITCH_AUTO
	u32		als_level_ing[C_CUST_ALS_LEVEL-1];
#endif
	bool		als_enable;	/*record current als status*/
	unsigned int	als_widow_loss;

	bool		ps_enable;	 /*record current ps status*/
	unsigned int	ps_thd_val;	 /*the cmd value can't be read, stored in ram*/
	ulong		enable;		 /*record HAL enalbe status*/
	ulong		pending_intr;	/*pending interrupt*/
	//ulong		first_read;	// record first read ps and als
	unsigned int	polling;
	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
#endif
};
static struct platform_driver ltr553_alsps_driver;

/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr553_i2c_driver = {
	.probe		= ltr553_i2c_probe,
	.remove		= ltr553_i2c_remove,
	.detect		= ltr553_i2c_detect,
	.suspend	= ltr553_i2c_suspend,
	.resume		= ltr553_i2c_resume,
	.id_table	= ltr553_i2c_id,
//	.address_data	= &ltr553_addr_data,
	.driver = {
//		.owner	= THIS_MODULE,
		.name	= LTR553_DEV_NAME,
	},
};

static struct ltr553_priv *ltr553_obj = NULL;

static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

#if defined(WT_DO_ALS_SMOOTH)
static unsigned int als_count = 0;
static int als_first_report = 1;
static int current_als_value = -1;
static int last_als_data = 0;
#endif
static int als_sleep_flag = 0; 

/*
 * The ps_trigger_xxx_table
 * controls the interrupt trigger range
 * bigger value means close to p-sensor
 * smaller value means far away from p-sensor
 */
#if 0
static int ps_trigger_high = 300;
static int ps_trigger_low = 250;
static int ps_trigger_delta = 0x0f;
#endif
static int ltr553_get_ps_value(struct ltr553_priv *obj, int ps);
static int ltr553_get_als_value(struct ltr553_priv *obj, int als);

/*----------------------------------------------------------------------------*/
static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
	u8 buf;
	int ret = 0;
	struct i2c_client client_read = *client;

	client_read.addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG |I2C_RS_FLAG;
	buf = addr;
	ret = i2c_master_send(&client_read, (const char*)&buf, 1<<8 | 1);
	if (ret < 0) {
		APS_ERR("send command error!!\n");
		return -EFAULT;
	}

	*data = buf;
	client->addr = client->addr& I2C_MASK_FLAG;
	return 0;
}

/*----------------------------------------------------------------------------*/
int ltr553_read_data_als(struct i2c_client *client, int *data)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
	int alsval_ch1_lo = 0;
	int alsval_ch1_hi = 0;
	int alsval_ch0_lo = 0;
	int alsval_ch0_hi = 0;
	int luxdata_int;
	int luxdata_flt;
	int ratio;
	int alsval_ch0;
	int alsval_ch1;

	if(client == NULL)
	{
		APS_ERR("ltr553_enable_als CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}
#if 0
	if(obj->als_enable != 1){
		*data = final_lux_val;
		return 0;
	}
#endif
	if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH1_0, &alsval_ch1_lo)) {
		APS_ERR("reads als data error (ch1 lo) = %d\n", alsval_ch1_lo);
		return -EFAULT;
	}
	if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH1_1, &alsval_ch1_hi)) {
		APS_ERR("reads aps data error (ch1 hi) = %d\n", alsval_ch1_hi);
		return -EFAULT;
	}
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
	
	if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH0_0, &alsval_ch0_lo)) {
		APS_ERR("reads als data error (ch0 lo) = %d\n", alsval_ch0_lo);
		return -EFAULT;
	}
	if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH0_1, &alsval_ch0_hi)) {
		APS_ERR("reads als data error (ch0 hi) = %d\n", alsval_ch0_hi);
		return -EFAULT;
	}
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;

	if (alsval_ch0==0 && alsval_ch1==0)
		{
			luxdata_int = 0;   //all 0, means dark lux=0
			goto err;
		}
	
	ratio = (alsval_ch1 * 1000) / (alsval_ch0 + alsval_ch1);

	if (ratio < 450){
		luxdata_flt = (17743 * alsval_ch0) - (-11059 * alsval_ch1);
		luxdata_flt = luxdata_flt / 10000;
	} else if ((ratio >= 450) && (ratio < 640)){
		luxdata_flt = (37725 * alsval_ch0) - (13363 * alsval_ch1);
		luxdata_flt = luxdata_flt / 10000;
	} else if ((ratio >= 640) && (ratio < 850)){
		luxdata_flt = (16900 * alsval_ch0) - (1690 * alsval_ch1);
		luxdata_flt = luxdata_flt / 10000;
	} else {
		luxdata_flt = 0;
	}

/*
	// For Range1
	if (als_gainrange == ALS_RANGE1_320)
		luxdata_flt = luxdata_flt / 150;
*/
	// convert float to integer;
	luxdata_int = luxdata_flt;
	if ((luxdata_flt - luxdata_int) > 0.5){
		luxdata_int = luxdata_int + 1;
	} else {
		luxdata_int = luxdata_flt;
	}

err:
	final_lux_val = luxdata_int;

	//------------------------------------
#ifdef ALS_DO_AVR
	int sum = 0;
	int i;
	als_temp[als_times]=luxdata_int;
    als_times++;

	if(first_cycle){
		for(i=0; i<als_times; i++){
			sum+=als_temp[i];
		}
		luxdata_int = sum / als_times;
	}else{
		for(i=0; i<ALS_AVR_COUNT; i++){
			sum+=als_temp[i];
		}
		luxdata_int = sum / ALS_AVR_COUNT;
	}

	if(als_times >= ALS_AVR_COUNT){
		als_times = 0;
		first_cycle = 0;
	}
#endif		
	//-------------------------------------
	if (atomic_read(&obj->trace) & CMC_TRC_APS_DATA){
			APS_LOG("ALS (CH0): 0x%04X\n", alsval_ch0);
			APS_LOG("ALS (CH1): 0x%04X\n", alsval_ch1);
			APS_LOG("ALS (Ratio): %d\n", ratio);
			APS_LOG("ALS: %d\n", luxdata_int);
		}

	*data = luxdata_int;
	
	//APS_LOG("luxdata_int=%x \n",luxdata_int);

	return 0;
}

int ltr553_read_data_ps(struct i2c_client *client, int *data)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
	int psval_lo = 0;
	int psval_hi = 0;
	int psdata = 0;

	if (hwmsen_read_byte_sr(client, APS_RO_PS_DATA_0, &psval_lo)) {
		printk("reads aps data = %d\n", psval_lo);
		return -EFAULT;
	}

	if (hwmsen_read_byte_sr(client, APS_RO_PS_DATA_1, &psval_hi)) {
		printk("reads aps hi data = %d\n", psval_hi);
		return -EFAULT;
	}

	psdata = ((psval_hi & 7) * 256) + psval_lo;
	//printk("psensor rawdata is:%d\n", psdata);
	if (atomic_read(&obj->trace) & CMC_TRC_CVT_PS){
				APS_LOG("PS (lo): %d\n", psval_lo);
				APS_LOG("PS (hi): %d\n", psval_hi);
				APS_LOG("PS: %d\n", psdata);
		}

	*data = psdata;
	final_prox_val = psdata;
	return 0;
}

/*----------------------------------------------------------------------------*/

int ltr553_init_device(struct i2c_client *client)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	APS_LOG("ltr553_init_device.........\r\n");
	u8 buf =0;
	int i = 0;
	int ret = 0;

	hwmsen_write_byte(client, APS_RW_PS_LED, CUST_LTR553_PS_LED); 		
	hwmsen_write_byte(client, APS_RW_PS_N_PULSES, CUST_LTR553_PS_N_PULSES);   
	hwmsen_write_byte(client, APS_RW_PS_MEAS_RATE, CUST_LTR553_PS_MEAS_RATE);	
	hwmsen_write_byte(client, APS_RW_INTERRUPT_PERSIST, CUST_LTR553_INTERRUPT_PERSIST); 

	if(0 == obj->hw->polling_mode_ps){
		hwmsen_write_byte(client, APS_RW_INTERRUPT, 0x01);		//enable ps for interrupt mode
		hwmsen_write_byte(client, APS_RW_PS_THRES_UP_0, (atomic_read(&obj->ps_thd_val_h)) & 0x00FF);
		hwmsen_write_byte(client, APS_RW_PS_THRES_UP_1, (atomic_read(&obj->ps_thd_val_h)>>8) & 0X07);

		hwmsen_write_byte(client, APS_RW_PS_THRES_LOW_0,(atomic_read(&obj->ps_thd_val_l)) & 0x00FF);
		hwmsen_write_byte(client, APS_RW_PS_THRES_LOW_1,(atomic_read(&obj->ps_thd_val_l)>>8) & 0X07);
	}
	
	mdelay(WAKEUP_DELAY);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr553_power(struct alsps_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;
	int status = 0; 

	APS_LOG("power %s\n", on ? "on" : "off");
	APS_LOG("power id:%d POWER_NONE_MACRO:%d\n", hw->power_id, POWER_NONE_MACRO);

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
			status = 0;
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "LTR553"))
			{
				APS_ERR("power on fails!!\n");
			}
			status =  1;
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "LTR553"))
			{
				APS_ERR("power off fail!!\n");
			}
			status =  -1;
		}
	}
	power_on = on;

	return status;
}
/*----------------------------------------------------------------------------*/
static int ltr553_enable_als(struct i2c_client *client, bool enable)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	int err=0;
	int trc = atomic_read(&obj->trace);
	u8 regdata=0;
	u8 regint=0;
	
#ifdef ALS_DO_AVR
	int i;
	als_times = 0;
	for(i=0;i<ALS_AVR_COUNT;i++){
		als_temp[i] = 0;
	}
	first_cycle = 1;
#endif

#ifdef WT_ALS_SWITCH_AUTO
	first_switch = 1;
#endif

#ifdef WT_ALS_ENABLE_SKIP
	als_skip_count = CUST_ALS_SKIP_COUNT;
#endif

	if(client == NULL)
	{
		APS_ERR("ltr553_enable_als CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	if(enable == true)
	{
		hwmsen_write_byte(client, APS_RW_ALS_CONTR, CUST_LTR553_ALS_CONTR_ENABLE);
		hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &regdata);
		if(regdata != CUST_LTR553_ALS_CONTR_ENABLE){
			hwmsen_write_byte(client, APS_RW_ALS_CONTR, CUST_LTR553_ALS_CONTR_ENABLE);
			hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &regdata);
		}
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		APS_LOG("ltr553_enable_als, APS_RW_ALS_CONTR: 0x%x!\n", regdata);
		
		mdelay(WAKEUP_DELAY);
	}
	else if(enable == false)
	{
		hwmsen_write_byte(client, APS_RW_ALS_CONTR, 0x00);
		hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &regdata);
		if(regdata != 0x00){
			hwmsen_write_byte(client, APS_RW_ALS_CONTR, 0x00);
			hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &regdata);
		}
		atomic_set(&obj->als_deb_on, 0);
		APS_LOG("ltr553_disable_als, APS_RW_ALS_CONTR: 0x%x!\n", regdata);
	}

	obj->als_enable = enable;

	if(trc & CMC_TRC_DEBUG)
	{
		APS_LOG("enable als (%d)\n", enable);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr553_enable_ps(struct i2c_client *client, bool enable)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	int err=0;
	int trc = atomic_read(&obj->trace);
	u8 regdata = 0;

	APS_LOG("ltr553_enable_ps: enable:%d, obj->ps_enable: %d\n",enable,obj->ps_enable);

	if(client == NULL)
	{
		APS_ERR("ltr553_enable_ps CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	if(enable == true)
	{
		ltr553_init_device(client);
	
		hwmsen_write_byte(client, APS_RW_PS_CONTR, CUST_LTR553_PS_CONTR_ENABLE);
		hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata);
		if(regdata != CUST_LTR553_PS_CONTR_ENABLE){
			hwmsen_write_byte(client, APS_RW_PS_CONTR, CUST_LTR553_PS_CONTR_ENABLE);
			hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata);
		}
		APS_LOG("ltr553_enable_ps, APS_RW_PS_CONTR: 0x%x!\n", regdata);

		mdelay(WAKEUP_DELAY);
  
		mt_eint_unmask(CUST_EINT_ALS_NUM); 	
	}
	else if(enable == false)
	{
		hwmsen_write_byte(client, APS_RW_PS_CONTR, 0x00);
		hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata);
		if(regdata != 0x00){
			hwmsen_write_byte(client, APS_RW_PS_CONTR, 0x00);
			hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata);
		}
		APS_LOG("ltr553_disable_ps, APS_RW_PS_CONTR: 0x%x!\n", regdata);
		
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt_eint_mask(CUST_EINT_ALS_NUM);
		}
	}
	obj->ps_enable = enable;
	if(trc & CMC_TRC_DEBUG)
	{
		APS_LOG("enable ps (%d)\n", enable);
	}

	return err;
}
/*----------------------------------------------------------------------------*/

static int ltr553_check_intr(struct i2c_client *client)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	int err;
	u8 data=0;

	// err = hwmsen_read_byte_sr(client,APS_INT_STATUS,&data);
	err = hwmsen_read_byte_sr(client,APS_RO_ALS_PS_STATUS,&data);
	APS_LOG("INT flage: = %x\n", data);

	if (err) {
		APS_ERR("WARNING: read int status: %d\n", err);
		return 0;
	}

	if (data & 0x08) {
		set_bit(CMC_BIT_ALS, &obj->pending_intr);
	} else {
		clear_bit(CMC_BIT_ALS, &obj->pending_intr);
	}

	if (data & 0x02) {
		set_bit(CMC_BIT_PS, &obj->pending_intr);
	} else {
		clear_bit(CMC_BIT_PS, &obj->pending_intr);
	}

	if (atomic_read(&obj->trace) & CMC_TRC_DEBUG) {
		//APS_LOG("check intr: 0x%08X\n", obj->pending_intr);
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
void ltr553_eint_func(void)
{
	struct ltr553_priv *obj = g_ltr553_ptr;
	//APS_LOG("ltr553 interrupt fuc\n");
	if(!obj)
	{
		return;
	}

	schedule_work(&obj->eint_work);
	//if(atomic_read(&obj->trace) & CMC_TRC_EINT)
	//{
	//	APS_LOG("eint: als/ps intrs\n");
	//}
}
/*----------------------------------------------------------------------------*/
static void ltr553_eint_work(struct work_struct *work)	
{
	struct ltr553_priv *obj = (struct ltr553_priv *)container_of(work, struct ltr553_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
	u8 buf;
	
	APS_FUN();
	memset(&sensor_data, 0, sizeof(sensor_data));

	err = ltr553_check_intr(obj->client);
	if(err) {
		APS_ERR("check intrs: %d\n", err);
	}
	//APS_LOG("ltr553_eint_work obj->pending_intr =%d, obj:%p\n",obj->pending_intr,obj);

	if ((1<<CMC_BIT_ALS) & obj->pending_intr) {
		// get raw data
		APS_LOG("ltr553 als INT\n");
		if (err = ltr553_read_data_als(obj->client, &obj->als)) {
			APS_ERR("ltr553 read als data: %d\n", err);
		}
		//map and store data to hwm_sensor_data
		while(-1 == ltr553_get_als_value(obj, obj->als)){
			ltr553_read_data_als(obj->client, &obj->als);
			msleep(50);
		}
 		sensor_data.values[0] = ltr553_get_als_value(obj, obj->als);
		//APS_LOG("ltr553_eint_work sensor_data.values[0] =%d\n",sensor_data.values[0]);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		// let up layer to know
		if (err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)) {
			APS_ERR("call hwmsen_get_interrupt_data light fail = %d\n", err);
		}
	}
	
	if ((1 << CMC_BIT_PS) & obj->pending_intr) {
		// get raw data
		APS_LOG("ltr553_eint_work, ltr553 ps INT\n");
		if (err = ltr553_read_data_ps(obj->client, &obj->ps)) {
			APS_ERR("ltr553 read ps data: %d\n", err);
		}
		//map and store data to hwm_sensor_data
		while(-1 == ltr553_get_ps_value(obj, obj->ps)) {
			ltr553_read_data_ps(obj->client, &obj->ps);
			msleep(50);
			APS_LOG("ltr553 read ps data delay\n");
		}
		sensor_data.values[0] = ltr553_get_ps_value(obj, obj->ps);
		APS_LOG("%s: ps_data=%d, ps_value=%d\n",__func__,obj->ps,sensor_data.values[0]);
		
//@hfs---start 20130712 for ps up system cannot wake
		if(1 == sensor_data.values[0] ) 
		{
			if( false == wake_lock_active(&ps_lock))
		    {
				wake_lock_timeout(&ps_lock, 5*HZ); //5s
			}
		}
		else	//wake unlock when close 
		{
			wake_unlock(&ps_lock);
		}
//@hfs---end 20130712 for ps up system cannot wake
		if(sensor_data.values[0] == 0)
		{
			hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,0xff);
			hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,0x07);
			
			hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,atomic_read(&obj->ps_thd_val_l) & 0xff);
			hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,(atomic_read(&obj->ps_thd_val_l)>>8) & 0X07);		
			//wake_unlock(&ps_wake_lock);
		}
		else if(sensor_data.values[0] == 1)
		{
			//wake_lock_timeout(&ps_wake_lock,ps_wakeup_timeout*HZ);
			hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,atomic_read(&obj->ps_thd_val_h) & 0xff);	
			hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,(atomic_read(&obj->ps_thd_val_h)>>8) & 0X07);
			
			hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,0x00);
			hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,0x00);	
		}

		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//let up layer to know
		//APS_LOG("ltr553 read ps data = %d \n",sensor_data.values[0]);
		if(err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)) {
			APS_ERR("call hwmsen_get_interrupt_data proximity fail = %d\n", err);
		}
		ps_interrupt_flag = true;
	}

	mt_eint_unmask(CUST_EINT_ALS_NUM);
}

int ltr553_setup_eint(struct i2c_client *client)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);

	APS_FUN();
	g_ltr553_ptr = obj;
#if 0 //jin
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, ltr553_eint_func, 0);
	mt_eint_unmask(CUST_EINT_ALS_NUM);	
#else
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, ltr553_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);	
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr553_get_partid(struct i2c_client *client)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	int i = 0;
	u8 regdata = 0;
	
	APS_LOG("ltr553_get_partid...\n");
	
	for(i=0; i<3; i++){
		if(hwmsen_read_byte_sr(client, APS_RO_PART_ID, &regdata)){
			APS_ERR("read i2c error! \n");
			return -EFAULT;
		}
		if(LTR553_PART_ID == regdata){
			APS_LOG("Get ltr553 partid OK!\n");
			return 0;
		}
	}
	APS_ERR("This is not ltr553, partid = 0x%x!\n",regdata);
	return -1; 
}

/*----------------------------------------------------------------------------*/
static int ltr553_init_client(struct i2c_client *client)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	int err=0;
	APS_LOG("ltr553_init_client.........\r\n");

	if(0 == obj->hw->polling_mode_ps){
		if((err = ltr553_setup_eint(client)))
		{
			APS_ERR("setup eint: %d\n", err);
			return err;
		}
	}
	if((err = ltr553_init_device(client)))
	{
		APS_ERR("init dev: %d\n", err);
		return err;
	}
	return err;
}
/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ltr553_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
		atomic_read(&ltr553_obj->i2c_retry), atomic_read(&ltr553_obj->als_debounce),
		atomic_read(&ltr553_obj->ps_mask), ltr553_obj->ps_thd_val, atomic_read(&ltr553_obj->ps_debounce));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_store_config(struct device_driver *ddri, char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}

	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{
		atomic_set(&ltr553_obj->i2c_retry, retry);
		atomic_set(&ltr553_obj->als_debounce, als_deb);
		atomic_set(&ltr553_obj->ps_mask, mask);
		ltr553_obj->ps_thd_val= thres;
		atomic_set(&ltr553_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr553_obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_store_trace(struct device_driver *ddri, char *buf, size_t count)
{
	int trace;
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&ltr553_obj->trace, trace);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_show_als(struct device_driver *ddri, char *buf)
{
	int als_data = 0;
	int als_enable, als_value;

	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}
	als_enable = test_bit(CMC_BIT_ALS, &ltr553_obj->enable)?(1):(0);
	if(als_enable){
		ltr553_read_data_als(ltr553_obj->client, &als_data);
	}else{
		return snprintf(buf, PAGE_SIZE, "ALS is disable\n");
	}
	if(als_data < 0){
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", als_data);
	}else{
		als_value = ltr553_get_als_value(ltr553_obj, als_data);
	}
	return snprintf(buf, PAGE_SIZE, "%d => %d lux\n",als_data,als_value);
	
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_show_ps(struct device_driver *ddri, char *buf)
{
	int	ps_data = 0;
	int ps_value, ps_enable;
	 
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}
	ps_enable = test_bit(CMC_BIT_PS, &ltr553_obj->enable)?(1):(0);
	if(ps_enable){
		ltr553_read_data_ps(ltr553_obj->client, &ps_data);
	}else{
		return snprintf(buf, PAGE_SIZE, "PS is disable\n");
	}		
	if(ps_data < 0){
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", ps_data);
	}else{
		ps_value = ltr553_get_ps_value(ltr553_obj, ps_data);
	}
	return snprintf(buf, PAGE_SIZE, "%d => 0x%04X\n",ps_data,ps_value);
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr553_show_ps_thres(struct device_driver *ddri, char *buf)
{
	int ps_thres_close, ps_thres_far, valid;

	struct alsps_hw *hw = get_cust_alsps_hw();
	
		if(ps_cali.valid == 1)
		{
			ps_thres_close = ps_cali.close;
			ps_thres_far = ps_cali.far_away;
			valid = ps_cali.valid;
		}
		else if(ps_cali.valid == 0){
			ps_thres_close = hw->ps_threshold_high;
			ps_thres_far = hw->ps_threshold_low;
			valid = ps_cali.valid;
		}
	return snprintf(buf, PAGE_SIZE, "%d=>%d,%d\n",valid,ps_thres_close,ps_thres_far);
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr553_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int ps_enable, als_enable;
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}
	als_enable = test_bit(CMC_BIT_ALS, &ltr553_obj->enable)?(1):(0);
	ps_enable = test_bit(CMC_BIT_PS, &ltr553_obj->enable)?(1):(0);
	
	if(ltr553_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr553_obj->hw->i2c_num, ltr553_obj->hw->power_id, ltr553_obj->hw->power_vol);
	}else{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "Enable: als:%d ps:%d\n",als_enable,ps_enable);
	return len;
}

#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr553_priv *obj, const char* buf, size_t count,
							 u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++;
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/


static ssize_t ltr553_show_reg(struct device_driver *ddri, char *buf)
{
	int i = 0;
	u8 bufdata;
	int count  = 0;
	
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}

	for(i = 0;i < 31 ;i++)
	{
		hwmsen_read_byte_sr(ltr553_obj->client,0x80+i,&bufdata);
		count+= sprintf(buf+count,"[%x] = (%x)\n",0x80+i,bufdata);
	}

	return count;
}

static ssize_t ltr553_store_reg(struct device_driver *ddri,char *buf,ssize_t count)
{
	u8 addr = 0,data = 0;
	
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null\n");
		return 0;
	}
	else if(0)//(2 != sscanf(buf,"0x%02X 0x%02X",&addr,&data))
	{
		APS_ERR("invalid format:%s\n",buf);
		return 0;
	}
	hwmsen_write_byte(ltr553_obj->client,addr,data);

	return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t ltr553_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}

	for(idx = 0; idx < ltr553_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr553_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_store_alslv(struct device_driver *ddri, char *buf, size_t count)
{
	struct ltr553_priv *obj;
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr553_obj->als_level, ltr553_obj->hw->als_level, sizeof(ltr553_obj->als_level));
	}
	else if(ltr553_obj->als_level_num != read_int_from_buf(ltr553_obj, buf, count,
			ltr553_obj->hw->als_level, ltr553_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}

	for(idx = 0; idx < ltr553_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr553_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_store_alsval(struct device_driver *ddri, char *buf, size_t count)
{
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr553_obj->als_value, ltr553_obj->hw->als_value, sizeof(ltr553_obj->als_value));
	}
	else if(ltr553_obj->als_value_num != read_int_from_buf(ltr553_obj, buf, count,
			ltr553_obj->hw->als_value, ltr553_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}

static ssize_t ltr553_show_enable_als(struct device_driver *ddrv,char *buf)
{
	ssize_t len =  0;
	int idx;
	if(!ltr553_obj)
	{
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}

	if(true == ltr553_obj->als_enable)
	{
		len = sprintf(buf,"%d\n",1);
	}
	else
	{
		len = sprintf(buf,"%d\n",0);
	}
	return len;

}
static  ssize_t ltr553_store_enable_als(struct device_driver *ddrv,char *buf, size_t count)
{
	int enable;
	if(!ltr553_obj)
	{	
		APS_ERR("ltr553_obj is null!!\n");
		return 0;
	}
	if(1 == sscanf(buf,"%d",&enable))
	{
		if(enable)
		{
			ltr553_enable_als(ltr553_obj->client,true);
		}
		else
		{
			ltr553_enable_als(ltr553_obj->client,false);
		}
	}
	else
	{
		APS_LOG("enable als fail\n");
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,	 S_IWUSR | S_IRUGO, ltr553_show_als, NULL);
static DRIVER_ATTR(ps,	 S_IWUSR | S_IRUGO, ltr553_show_ps,	NULL);
static DRIVER_ATTR(ps_thres, S_IWUSR | S_IRUGO, ltr553_show_ps_thres,	NULL);
static DRIVER_ATTR(alslv, S_IWUSR | S_IRUGO, ltr553_show_alslv, ltr553_store_alslv);
static DRIVER_ATTR(alsval, S_IWUSR | S_IRUGO, ltr553_show_alsval,ltr553_store_alsval);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, ltr553_show_trace, ltr553_store_trace);
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, ltr553_show_status, NULL);
static DRIVER_ATTR(reg,	 S_IWUSR | S_IRUGO, ltr553_show_reg, ltr553_store_reg);
static DRIVER_ATTR(enable_als,  S_IWUSR | S_IRUGO, ltr553_show_enable_als, ltr553_store_enable_als);
/*----------------------------------------------------------------------------*/
static struct device_attribute *ltr553_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
	&driver_attr_ps_thres,
	&driver_attr_trace,		/*trace log*/
	&driver_attr_alslv,
	&driver_attr_alsval,
	&driver_attr_status,
	&driver_attr_reg,
	&driver_attr_enable_als,
};
/*----------------------------------------------------------------------------*/
static int ltr553_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr553_attr_list)/sizeof(ltr553_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, ltr553_attr_list[idx]))
		{
			APS_ERR("driver_create_file (%s) = %d\n", ltr553_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
	static int ltr553_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr553_attr_list)/sizeof(ltr553_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, ltr553_attr_list[idx]);
	}

	return err;
}
/******************************************************************************
 * Function Configuration
******************************************************************************/
// static int ltr553_get_als_value(struct ltr553_priv *obj, u8 als)
static int ltr553_get_als_value(struct ltr553_priv *obj, int als)
{
	int idx;
	int invalid = 0;

#if defined(WT_DO_ALS_SMOOTH)
	unsigned int raw_data;
#endif

#ifdef WT_ALS_SWITCH_AUTO
	int idx_high = -1, idx_low = -1, i = 0;
	static int last_idx_high = -1, last_idx_low = -1;
	
	if(first_switch)
	{
		for(i = 0; i < obj->als_level_num; i++)
		{
			obj->als_level_ing[i] = obj->hw->als_level[i];
		}
		last_idx_high = -1;
		last_idx_low = -1;
		first_switch = 0;
	}
#endif

#ifdef WT_ALS_SWITCH_AUTO
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->als_level_ing[idx])
		{
			break;
		}
	}
#else
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
#endif

	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}

#ifdef WT_ALS_SWITCH_AUTO
	if(obj->hw->als_level_high[idx] != obj->hw->als_level[idx])
	{
		obj->als_level_ing[idx] = obj->hw->als_level_high[idx];
		idx_high = idx;
	}
	if((idx > 0)&&(obj->hw->als_level_low[idx-1] != obj->hw->als_level[idx-1]))
	{
		obj->als_level_ing[idx-1] = obj->hw->als_level_low[idx-1];
		idx_low = idx - 1;
	}
	if((idx_low!=last_idx_low)||(idx_high!=last_idx_high))
	{
		for(i=0; i<obj->als_level_num; i++)
		{
			if((i == idx_low)||(i == idx_high))
			{
				continue;
			}
			obj->als_level_ing[i] = obj->hw->als_level[i];
		}
		last_idx_low = idx_low;
		last_idx_high = idx_high;
	}
#endif
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}

		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
#if defined(WT_DO_ALS_SMOOTH)	
		if(als_sleep_flag == 1) 
			return current_als_value; // in sleep 

		raw_data = obj->hw->als_value[idx];

		if(raw_data == last_als_data)
		{
			als_count++;
		}
		else
		{
			als_count = 0;
			last_als_data = raw_data;
		}
		//normal
		if(als_count >= ALS_COUNT_FOR_SMOOTH)
		{
			als_count = 0;
			current_als_value = raw_data;
		}
		//after resume
		if(als_sleep_flag == 2 && als_count >= ALS_COUNT_FOR_SMOOTH/4)
		{
			als_count = 0;
			current_als_value = raw_data;
			als_sleep_flag = 0;
		}

		//the first time
		if(current_als_value == -1)
		{
			als_count = 0;
			last_als_data = raw_data;
			current_als_value = raw_data;
		}

		if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS) {
			APS_DBG("ALS: %05d => %05d, value:%d, count:%d, flag:%d\n", 
				als, obj->hw->als_value[idx], current_als_value, als_count, als_sleep_flag);
		}
		//printk("als_value :%d => %d => %d \n",als,obj->hw->als_value[idx],current_als_value);
		return current_als_value;
#else
		if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		}
		
#ifdef WT_ALS_ENABLE_SKIP
		if(als_lux_pre == -1) //the first time
		{
			als_lux_pre = obj->hw->als_value[idx];
			als_skip_count = 0;
		}
		if(als_sleep_flag == 1) //in sleep
		{
			return als_lux_pre;
		}
		if(als_skip_count)  //skip CUST_ALS_SKIP_COUNT times
		{
			als_skip_count--;
		}
		else	
		{
			als_lux_pre = obj->hw->als_value[idx];   // report real value
		}
		if(als_skip_count < 0)
		{
			als_skip_count = 0;
		}
		//printk("ALS :%d => %d, real = %d\n",als,als_lux_pre,obj->hw->als_value[idx]);
		return als_lux_pre;
#else
		return obj->hw->als_value[idx];
#endif

#endif
	}
	else
	{
		if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
		}
		return -1;
	}
}
/*----------------------------------------------------------------------------*/

static int ltr553_get_ps_value(struct ltr553_priv *obj, int ps)
{
	int val= -1;
	int invalid = 0;
	
	APS_DBG("%s enter\n",__func__);
	if (ps > atomic_read(&obj->ps_thd_val_h)) {
		// bigger value, close
		val = 0;
	} else if (ps < atomic_read(&obj->ps_thd_val_l)) {
		// smaller value, far away
		val = 1;
	} else {
		val = 1;
	}

	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}

		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			APS_DBG("PS: %05d => %05d\n", ps, val);
		}
		APS_DBG("%s: return val = %d\n",__func__,val); 
		return val;

	}
	else
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			APS_DBG("PS: %05d => %05d (-1)\n", ps, val);
		}
		return -1;
	}

}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int ltr553_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr553_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr553_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//add for ps calibration   start
static int ltr553_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
     int i=0 ,err = 0,j = 0,sum=0;
	 int data[21] = {0},data_cali;

	 for(i = 0;i<20;i++)
	 	{
	 		mdelay(100);//mdelay(5);fangliang
			if(!ltr553_read_data_ps(client,&data[i]))
			{
				sum += data[i];
			}
			else
			{
				break;
			}
	 	}
	 if(i == 20)
	 {
			data_cali = sum/20;
			printk("ltr553_read_data_for_cali data_cali = %d\n",data_cali);
			ps_data_cali->far_away = data_cali + CUST_LTR553_PS_THRES_FAR;
			ps_data_cali->close = data_cali + CUST_LTR553_PS_THRES_CLOSE;
			ps_data_cali->valid = 1;
			err= 0;
	 }
	 else
	 {
	 	ps_data_cali->valid = 0;
		APS_LOG("ltr553_read_data_for_cali get data error!\n");
	 	err=  -1;
	 }
	 
	 return err;
}
static void ltr553_WriteCalibration(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *data_cali)
{
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	struct alsps_hw *hw = get_cust_alsps_hw();
	u8 databuf[2];	   
   	u8 res;
 	if(data_cali->valid == 1)
  	{
		atomic_set(&obj->ps_thd_val_h,  data_cali->close);
		atomic_set(&obj->ps_thd_val_l,  data_cali->far_away);
		ps_cali.valid = 1;
	}
	else if(data_cali->valid == 0){
		atomic_set(&obj->ps_thd_val_h,  hw->ps_threshold_high);
		atomic_set(&obj->ps_thd_val_l,  hw->ps_threshold_low);
		ps_cali.valid = 0;	
	}
		ps_cali.close = atomic_read(&obj->ps_thd_val_h);
		ps_cali.far_away = atomic_read(&obj->ps_thd_val_l);
	
	if(0 == obj->hw->polling_mode_ps){
		hwmsen_write_byte(client, APS_RW_PS_THRES_UP_0, (atomic_read(&obj->ps_thd_val_h)) & 0x00FF);
		hwmsen_write_byte(client, APS_RW_PS_THRES_UP_1, (atomic_read(&obj->ps_thd_val_h)>>8) & 0X07);
	
		hwmsen_write_byte(client, APS_RW_PS_THRES_LOW_0,(atomic_read(&obj->ps_thd_val_l)) & 0x00FF);
		hwmsen_write_byte(client, APS_RW_PS_THRES_LOW_1,(atomic_read(&obj->ps_thd_val_l)>>8) & 0X07);
	}
	
}
//add for ps calibration   end

static long ltr553_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr553_priv *obj = i2c_get_clientdata(client);
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	struct PS_CALI_DATA_STRUCT ps_cali_temp;// add for ps calibration 

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = ltr553_enable_ps(obj->client, true))
				{
					APS_ERR("enable ps fail: %d\n", err);
					goto err_out;
				}
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if(err = ltr553_enable_ps(obj->client, false))
				{
					APS_ERR("disable ps fail: %d\n", err);
					goto err_out;
				}
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			if(ps_interrupt_flag){
			if(err = ltr553_read_data_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			dat = ltr553_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			ps_interrupt_flag = false;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			if(err = ltr553_read_data_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}

			dat = obj->ps;// & 0x80;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = ltr553_enable_als(obj->client, true))
				{
					APS_ERR("enable als fail: %d\n", err);
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if(err = ltr553_enable_als(obj->client, false))
				{
					APS_ERR("disable als fail: %d\n", err);
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA:
			if(err = ltr553_read_data_als(obj->client, &obj->als))
			{
				goto err_out;
			}
			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_RAW_DATA:
			if(err = ltr553_read_data_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = obj->als;	
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
//add for ps calibration   start
		case ALSPS_SET_PS_CALI:
			dat = (void __user*)arg;
			if(dat == NULL)
			{
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&ps_cali_temp,dat, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;	  
			}
			ltr553_WriteCalibration(obj->client,&ps_cali_temp);
			APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
			break;
		case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			ltr553_enable_ps(obj->client, 1);
			ltr553_read_data_for_cali(obj->client,&ps_cali_temp);
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}			   
			break;
//add for ps calibration   end
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;
}
/*----------------------------------------------------------------------------*/
static struct file_operations ltr553_fops = {
//	.owner = THIS_MODULE,
	.open = ltr553_open,
	.release = ltr553_release,
	.unlocked_ioctl = ltr553_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr553_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr553_fops,
};
/*----------------------------------------------------------------------------*/
static int ltr553_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	//struct ltr553_priv *obj = i2c_get_clientdata(client);
	//int err;
	APS_FUN();
#if 0
	if(msg.event == PM_EVENT_SUSPEND)
	{
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}

		atomic_set(&obj->als_suspend, 1);
		if(err = ltr553_enable_als(client, false))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}
		if(!obj->ps_enable)
			ltr553_power(obj->hw, 0);
	}
#endif	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr553_i2c_resume(struct i2c_client *client)
{
	//struct ltr553_priv *obj = i2c_get_clientdata(client);
	//int err;
	APS_FUN();
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	if(1 == ltr553_power(obj->hw, 1))
	{
		if(err = ltr553_init_device(client))
		{
			APS_ERR("initialize client fail!!\n");
			return err;
		}
		if(obj->ps_enable)
		{
			if(err = ltr553_enable_ps(client,true))
			{
				APS_ERR("enable ps fail: %d\n",err);
			}
			return err;
		}
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = ltr553_enable_als(client, true))
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}
	else
	{
		if(err = ltr553_enable_als(client, false))
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}
#endif	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void ltr553_early_suspend(struct early_suspend *h)
{
	/*early_suspend is only applied for ALS*/
	struct ltr553_priv *obj = container_of(h, struct ltr553_priv, early_drv);
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 1);
	if(err = ltr553_enable_als(obj->client, false))
	{
		APS_ERR("disable als fail: %d\n", err);
	}

	als_sleep_flag = 1;
#if defined(WT_DO_ALS_SMOOTH)
	als_count = 0;
#endif

}
/*----------------------------------------------------------------------------*/
static void ltr553_late_resume(struct early_suspend *h)
{
	/*early_suspend is only applied for ALS*/
	struct ltr553_priv *obj = container_of(h, struct ltr553_priv, early_drv);
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = ltr553_enable_als(obj->client, true))
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}
	else
	{
		if(err = ltr553_enable_als(obj->client, false))
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}

	als_sleep_flag = 2;
#if defined(WT_DO_ALS_SMOOTH)
	als_count = 0;
#endif
}

int ltr553_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr553_priv *obj = (struct ltr553_priv *)self;

	//APS_FUN(f);
	APS_LOG("ltr553_ps_operate command:%d\n",command);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				APS_DBG("%s: SENSOR_ENABLE, value = %d\n",__func__,value);
				if(value)
				{
					if(err = ltr553_enable_ps(obj->client, true))
					{
						APS_ERR("enable ps fail: %d\n", err);
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if(err = ltr553_enable_ps(obj->client, false))
					{
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if ((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			} else {
				sensor_data = (hwm_sensor_data *)buff_out;
				if (err = ltr553_read_data_ps(obj->client, &obj->ps)) {
					err = -1;
					break;
				} else {
					while(-1 == ltr553_get_ps_value(obj, obj->ps)) {
						ltr553_read_data_ps(obj->client, &obj->ps);
						msleep(50);
					}
					sensor_data->values[0] = ltr553_get_ps_value(obj, obj->ps);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					APS_LOG("ltr553 get ps raw_data = %d, sensor_data =%d\n",obj->ps, sensor_data->values[0]);
				}
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

int ltr553_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr553_priv *obj = (struct ltr553_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					if(err = ltr553_enable_als(obj->client, true))
					{
						APS_ERR("enable als fail: %d\n", err);
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if(err = ltr553_enable_als(obj->client, false))
					{
						APS_ERR("disable als fail: %d\n", err);
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			//APS_LOG("ltr553 get als data !!!!!!\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("ltr553_als_operate get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;

				if(err = ltr553_read_data_als(obj->client, &obj->als))
				{
					err = -1;
				}
				else
				{
					sensor_data->values[0] = ltr553_get_als_value(obj, obj->als);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					//APS_LOG("ltr553_als_operate get obj->als = %d, sensor_data =%d\n",obj->als, sensor_data->values[0]);
				}
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
static int ltr553_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	APS_FUN();
	strcpy(info->type, LTR553_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr553_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr553_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	u8 buf = 0;
	int addr = 1;
	int ret = 0;
	APS_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	ltr553_obj = obj;

	obj->hw = get_cust_alsps_hw();

	if(0 == obj->hw->polling_mode_ps){
		INIT_WORK(&obj->eint_work, ltr553_eint_work);
	}
	//@hfs---start  20130712 for ps up system cannot wake
	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
	//@hfs---end
	obj->client = client;
	APS_LOG("addr = %x\n",obj->client->addr);
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->als_debounce, 100);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 1000);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_h, obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_l, obj->hw->ps_threshold_low);

	obj->ps_enable = 0;
	obj->als_enable = 0;
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
#ifdef WT_ALS_SWITCH_AUTO
	memcpy(obj->als_level_ing, obj->hw->als_level, sizeof(obj->als_level_ing));
#endif
	//pre set ps threshold
	obj->ps_thd_val = obj->hw->ps_threshold;
	//pre set window loss
	obj->als_widow_loss = obj->hw->als_window_loss;

	ltr553_i2c_client = client;

	if(err = ltr553_get_partid(client))
	{
		goto exit_init_failed;
	}
	
	if(err = ltr553_init_client(client))
	{
		goto exit_init_failed;
	}

	if(err = ltr553_enable_als(client, false))
	{
		APS_ERR("disable als fail: %d\n", err);
	}
	if(err = ltr553_enable_ps(client, false))
	{
		APS_ERR("disable ps fail: %d\n", err);
	}

	if(err = misc_register(&ltr553_device))
	{
		APS_ERR("ltr553_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(err = ltr553_create_attr(&ltr553_alsps_driver.driver))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	obj_ps.self = ltr553_obj;
	APS_LOG("obj->hw->polling_mode:%d\n",obj->hw->polling_mode);
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;//interrupt mode
	}
	obj_ps.sensor_operate = ltr553_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach ID_PROXIMITY fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	obj_als.self = ltr553_obj;
	ltr553_obj->polling = obj->hw->polling_mode;
	if(1 == obj->hw->polling_mode_als)
	{
		obj_als.polling = 1;
		APS_LOG("polling mode\n");
	}
	else
	{
		obj_als.polling = 0;//interrupt mode
		APS_LOG("interrupt mode\n");
	}
	obj_als.sensor_operate = ltr553_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach ID_LIGHT fail = %d\n", err);
		goto exit_create_attr_failed;
	}
						
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB,
	obj->early_drv.suspend = ltr553_early_suspend,
	obj->early_drv.resume = ltr553_late_resume,
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
		ltr553_delete_attr(&ltr553_alsps_driver.driver);
	exit_misc_device_register_failed:
		misc_deregister(&ltr553_device);
	exit_init_failed:
	exit_kfree:
	kfree(obj);
	exit:
	ltr553_i2c_client = NULL;
	mt_eint_unmask(CUST_EINT_ALS_NUM);

	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr553_i2c_remove(struct i2c_client *client)
{
	int err;

	if(err = ltr553_delete_attr(&ltr553_alsps_driver.driver))
	{
		APS_ERR("ltr553_delete_attr fail: %d\n", err);
	}

	if(err = misc_deregister(&ltr553_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);
	}

	ltr553_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static int ltr553_probe(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();

	ltr553_power(hw,1);
	if(i2c_add_driver(&ltr553_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	return 0;
}

static int ltr553_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();
	ltr553_power(hw, 0);
	i2c_del_driver(&ltr553_i2c_driver);
	return 0;
}
#if 0
static struct platform_driver ltr553_alsps_driver = {
	.probe	= ltr553_probe,
	.remove	= ltr553_remove,
	.driver	= {
		.name	= "als_ps",
//		.owner	= THIS_MODULE,
	}
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{ .compatible = "mediatek,als_ps", },
	{},
};
#endif

static struct platform_driver ltr553_alsps_driver =
{
	.probe      = ltr553_probe,
	.remove     = ltr553_remove,    
	.driver     = 
	{
		.name = "als_ps",
        #ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
		#endif
	}
};
static int __init ltr553_init(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	APS_FUN();
	//wake_lock_init(&ps_wake_lock,WAKE_LOCK_SUSPEND,"ps module");
	i2c_register_board_info(hw->i2c_num, &i2c_ltr553, 1);
	if (platform_driver_register(&ltr553_alsps_driver)) {
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr553_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&ltr553_alsps_driver);
	//wake_lock_destroy(&ps_wake_lock);
}
/*----------------------------------------------------------------------------*/
module_init(ltr553_init);
module_exit(ltr553_exit);
/*----------------------------------------------------------------------------*/
MODULE_DESCRIPTION("LTR553 light sensor & p sensor driver");
MODULE_LICENSE("GPL");
