/************************************************************************
 * Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
 *
 * File Name: focaltech_driver.c
 *
 * Author:
 *
 * Created: 2015-01-01
 *
 * Abstract: Function for driver initial, report point, resume, suspend
 *
 ************************************************************************/
#include "tpd.h"
#include "tpd_custom_fts.h"
#include "cust_gpio_usage.h"
#include <cust_eint.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/dma-mapping.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#if WT_ADD_CTP_INFO
#include <linux/hardware_info.h>
static char tp_string_version[40];
#endif

#if CTP_CHARGER_DETECT
#include <mach/battery_common.h>

u8 prv_charger_status = 0;
u8 current_charger_status = 0;
#endif

#ifdef   CTP_OPEN_SHORT_TEST
#include  "mcap_5x46_test_lib.h"
#include  <linux/fs.h>
#include  <linux/device.h>     

#define ITO_TEST_AUTHORITY   0777
#define FT5446_PROC_MSG_ITO_TESE      "ctp_test"
#define FT5446_PROC_ITO_OPENTEST_DEBUG      "opentest"
#define FT5446_PROC_ITO_SHORTTEST_DEBUG      "shorttest"
#define FT5446_PROC_ITO_TTESTRESULT_DEBUG      "testresult"

struct  i2c_client *g_focalclient = NULL;
#endif

/*if need these function, pls enable this MACRO*/
//#define MT_PROTOCOL_B
//#define TPD_PROXIMITY

#define FTS_CTL_IIC
#define SYSFS_DEBUG
#define FTS_APK_DEBUG
//#define HIDTOI2C_DISABLE

//for tp esd check
//#define GTP_ESD_PROTECT  
#ifdef GTP_ESD_PROTECT
#define TPD_ESD_CHECK_CIRCLE        						200
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
//add for esd
static int count_irq = 0;
static unsigned long esd_check_circle = TPD_ESD_CHECK_CIRCLE;
static u8 run_check_91_register = 0;
#endif


#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef SYSFS_DEBUG
#include "focaltech_ex_fun.h"
#endif

/*PROXIMITY*/
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
static u8 tpd_proximity_flag 			= 0;
//add for tpd_proximity by wangdongfang
static u8 tpd_proximity_flag_one 		= 0; 
//0-->close ; 1--> far away
static u8 tpd_proximity_detect 		= 1;
#endif
/*GESTRUE*/
#if WT_CTP_GESTURE_SUPPORT
//#include "ft_gesture_lib.h"
#define FTS_GESTRUE_POINTS 				255
#define FTS_GESTRUE_POINTS_ONETIME  		62
#define FTS_GESTRUE_POINTS_HEADER 		8
#define FTS_GESTURE_OUTPUT_ADRESS 		0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 	4

#define KEY_GESTURE_U 						KEY_U
#define KEY_GESTURE_UP 						KEY_UP
#define KEY_GESTURE_DOWN 					KEY_DOWN
#define KEY_GESTURE_LEFT 					KEY_LEFT 
#define KEY_GESTURE_RIGHT 					KEY_RIGHT
#define KEY_GESTURE_O 						KEY_O
#define KEY_GESTURE_E 						KEY_E
#define KEY_GESTURE_M 						KEY_M 
#define KEY_GESTURE_L 						KEY_L
#define KEY_GESTURE_W 						KEY_W
#define KEY_GESTURE_S 						KEY_S 
#define KEY_GESTURE_V 						KEY_V
#define KEY_GESTURE_Z 						KEY_Z

#define GESTURE_LEFT						0x20
#define GESTURE_RIGHT						0x21
#define GESTURE_UP		    					0x22
#define GESTURE_DOWN						0x23
#define GESTURE_DOUBLECLICK				0x24
#define GESTURE_O		    					0x30
#define GESTURE_W		    					0x31
#define GESTURE_M		    					0x32
#define GESTURE_E		    					0x33
#define GESTURE_L		    					0x44
#define GESTURE_S		    					0x46
#define GESTURE_V		    					0x54
#define GESTURE_Z		    					0x41

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};

static void Ctp_Gesture_Fucntion_Proc_File(void);
static char gtp_gesture_coordinate[60]= {0X8140 >> 8, 0X8140 & 0xFF};
static char gtp_gesture_value = 0;
static char gtp_gesture_onoff = '0';
static char gtp_glove_onoff = '0';
const char gtp_gesture_type[]=GTP_GESTURE_TPYE_STR;
static const char gtp_glove_support_flag=GTP_GLOVE_SUPPORT_ONOFF;
static const char gtp_gesture_support_flag=GTP_GESTURE_SUPPORT_ONOFF;
static const char gtp_verson[] =GTP_PROC_DRIVER_VERSION;
static  char gtp_glove_support_flag_changed=NULL;
static  char gtp_gesture_support_flag_changed=NULL;
#endif

/*ic update info*/
struct Upgrade_Info fts_updateinfo[] =
{
	{0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 10, 2000},
	{0x58,"FT5x22",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x58, 0x2c, 20, 2000},
	{0x59,"FT5x26",TPD_MAXPOINTS_10,AUTO_CLB_NONEED,30, 50, 0x79, 0x10, 1, 2000},
};

struct Upgrade_Info fts_updateinfo_curr;
struct i2c_client *global_i2c_client = NULL;
struct task_struct *thread = NULL;
extern struct tpd_device *tpd;
struct tp_module_info tp_info;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);
static void tpd_eint_interrupt_handler(void);
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

/*dma declare, allocate and release*/
#define __MSG_DMA_MODE__
#ifdef __MSG_DMA_MODE__
u8 *g_dma_buff_va = NULL;
dma_addr_t *g_dma_buff_pa = NULL;
#endif

#ifdef __MSG_DMA_MODE__

static int msg_dma_alloct()
{
	g_dma_buff_va = (u8 *)dma_alloc_coherent(&global_i2c_client->dev, 4096, &g_dma_buff_pa, GFP_KERNEL);
	if(!g_dma_buff_va)
	{
		FT_CTP_ERROR("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
		return -1;
	}

	return 0;
}
static void msg_dma_release(){
	if(g_dma_buff_va)
	{
		dma_free_coherent(NULL, 4096, g_dma_buff_va, g_dma_buff_pa);
		g_dma_buff_va = NULL;
		g_dma_buff_pa = NULL;
		FT_CTP_ERROR("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
}
#endif
/*register define*/
#define FTS_RESET_PIN							GPIO_CTP_RST_PIN
#define TPD_OK 									0
#define DEVICE_MODE 							0x00
#define GEST_ID 									0x01
#define TD_STATUS 								0x02
#define TOUCH1_XH 								0x03
#define TOUCH1_XL 								0x04
#define TOUCH1_YH 								0x05
#define TOUCH1_YL 								0x06
#define TOUCH2_XH 								0x09
#define TOUCH2_XL 								0x0A
#define TOUCH2_YH 								0x0B
#define TOUCH2_YL 								0x0C
#define TOUCH3_XH 								0x0F
#define TOUCH3_XL 								0x10
#define TOUCH3_YH 								0x11
#define TOUCH3_YL 								0x12
#define TPD_MAX_RESET_COUNT 					3

/*touch event info*/
struct ts_event 
{
	u16 au16_x[CFG_MAX_TOUCH_POINTS];				/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];				/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];		/*touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];			/*touch ID */
	u16 pressure[CFG_MAX_TOUCH_POINTS];
	u16 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
	int touchs;
	u8 touch_point_num;
};


#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif
static int tpd_flag = 0;
static int tpd_halt=0;
static int point_num = 0;
static int p_point_num = 0;
static u8 buf_addr[2] = { 0 };
static u8 buf_value[2] = { 0 };
/*
 *	open/release/(I/O) control tpd device
 *
 */
//#define VELOCITY_CUSTOM_fts
#ifdef VELOCITY_CUSTOM_fts
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

/*for magnify velocity*/
#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 				10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 				10
#endif

#define TOUCH_IOC_MAGIC 						'A'
#define TPD_GET_VELOCITY_CUSTOM_X 			_IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y 			_IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;


/************************************************************************
 * Name: tpd_misc_open
 * Brief: open node
 * Input: node, file point
 * Output: no
 * Return: fail <0
 ***********************************************************************/
static int tpd_misc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}
/************************************************************************
 * Name: tpd_misc_release
 * Brief: release node
 * Input: node, file point
 * Output: no
 * Return: 0
 ***********************************************************************/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}
/************************************************************************
 * Name: tpd_unlocked_ioctl
 * Brief: I/O control for apk
 * Input: file point, command
 * Output: no
 * Return: fail <0
 ***********************************************************************/

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{

	void __user *data;

	long err = 0;

	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		FT_CTP_ERROR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			

			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			

			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;


		default:
			FT_CTP_ERROR("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;

	}

	return err;
}


static struct file_operations tpd_fops = {
	//.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};

static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};
#endif
/*touch info*/
struct touch_info {
	int y[10];
	int x[10];
	int p[10];
	int id[10];
	int count;
};

/*register driver and device info*/ 
static const struct i2c_device_id fts_tpd_id[] = {{"fts",0},{}};

static struct i2c_board_info __initdata fts_i2c_tpd={ I2C_BOARD_INFO("fts", (0x70>>1))};

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = "fts",
		//.owner = THIS_MODULE,
	},
	.probe = tpd_probe,
	.remove = __devexit_p(tpd_remove),
	.id_table = fts_tpd_id,
	.detect = tpd_detect,

};

/************************************************************************
 * Name: fts_i2c_Read
 * Brief: i2c read
 * Input: i2c info, write buf, write len, read buf, read len
 * Output: get data in the 3rd buf
 * Return: fail <0
 ***********************************************************************/
int fts_i2c_Read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret,i;

	// for DMA I2c transfer

	mutex_lock(&i2c_rw_access);

	if(writelen!=0)
	{
		//DMA Write
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
			//dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,*g_dma_buff_pa);
			FT_CTP_ERROR("i2c write failed\n");
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}

	//DMA Read 

	if(readlen!=0)

	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;

		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);

		memcpy(readbuf, g_dma_buff_va, readlen);

		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}

	mutex_unlock(&i2c_rw_access);

	return ret;

}

/************************************************************************
 * Name: fts_i2c_Write
 * Brief: i2c write
 * Input: i2c info, write buf, write len
 * Output: no
 * Return: fail <0
 ***********************************************************************/
int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	int i = 0;

	mutex_lock(&i2c_rw_access);
	
 	//client->addr = client->addr & I2C_MASK_FLAG;

	//ret = i2c_master_send(client, writebuf, writelen);
	memcpy(g_dma_buff_va, writebuf, writelen);

	client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
	if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
		//dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,*g_dma_buff_pa);
		FT_CTP_ERROR("i2c write failed\n");
	client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

	mutex_unlock(&i2c_rw_access);

	return ret;
}
/************************************************************************
 * Name: fts_write_reg
 * Brief: write register
 * Input: i2c info, reg address, reg value
 * Output: no
 * Return: fail <0
 ***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_Write(client, buf, sizeof(buf));
}
/************************************************************************
 * Name: fts_read_reg
 * Brief: read register
 * Input: i2c info, reg address, reg value
 * Output: get reg value
 * Return: fail <0
 ***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{

	return fts_i2c_Read(client, &regaddr, 1, regvalue, 1);

}
/************************************************************************
 * Name: fts_get_upgrade_array
 * Brief: decide which ic
 * Input: no
 * Output: get ic info in fts_updateinfo_curr
 * Return: no
 ***********************************************************************/
void fts_get_upgrade_array(void)
{

	u8 chip_id;
	u32 i;
	int ret = 0;

	ret = fts_read_reg(global_i2c_client, FTS_REG_CHIP_ID,&chip_id);
	if (ret<0) 
	{
		FT_CTP_ERROR("[Focal][Touch] read value fail");
	}
		
	FT_CTP_DMESG("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[10], sizeof(struct Upgrade_Info));
	}
}
/************************************************************************
 * Name: tpd_down
 * Brief: down info
 * Input: x pos, y pos, id number
 * Output: no
 * Return: no
 ***********************************************************************/
static void tpd_down(int x, int y, int p) {

	if(x > TPD_RES_X)
	{
		TPD_DEBUG("warning: IC have sampled wrong value.\n");;
		return;
	}
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0x3f);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//printk("tpd:D[%4d %4d %4d] ", x, y, p);
	/* track id Start 0 */
	//input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	input_mt_sync(tpd->dev);
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 1);  
	}
	if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	{
		//msleep(50);
		FT_CTP_ERROR("D virtual key \n");
	}
	TPD_EM_PRINT(x, y, x, y, p-1, 1);
}
/************************************************************************
 * Name: tpd_up
 * Brief: up info
 * Input: x pos, y pos, count
 * Output: no
 * Return: no
 ***********************************************************************/
static  void tpd_up(int x, int y,int *count)
{
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	//printk("U[%4d %4d %4d] ", x, y, 0);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, 0, 0);

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 0); 
	}   		 
}
/************************************************************************
 * Name: tpd_touchinfo
 * Brief: touch info
 * Input: touch info point, no use
 * Output: no
 * Return: success nonzero
 ***********************************************************************/
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[128] = {0};
	u16 high_byte,low_byte,reg;
	u8 report_rate =0;
	p_point_num = point_num;
	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
	mutex_lock(&i2c_access);

	reg = 0x00;
	fts_i2c_Read(global_i2c_client, &reg, 1, data, 64);
	mutex_unlock(&i2c_access);

	/*get the number of the touch points*/

	point_num= data[2] & 0x0f;

	for(i = 0; i < point_num; i++)  
	{
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
		cinfo->id[i] = data[3+6*i+2]>>4; //touch id
		/*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;	
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;
	}

	//printk(" tpd cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);
	return true;

};


/************************************************************************
 * Name: fts_read_Touchdata
 * Brief: report the point information
 * Input: event info
 * Output: get touch data in pinfo
 * Return: success is zero
 ***********************************************************************/
static unsigned int buf_count_add=0;
static unsigned int buf_count_neg=0;
//unsigned int buf_count_add1;
//unsigned int buf_count_neg1;
u8 buf_touch_data[30*POINT_READ_BUF] = { 0 };//0xFF
static int fts_read_Touchdata(struct ts_event *pinfo)
{
	u8 buf[POINT_READ_BUF] = { 0 };//0xFF
	int ret = -1;
	int i = 0;
	u8 pointid = FTS_MAX_ID;
	//u8 pt00f=0;
	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}

	mutex_lock(&i2c_access);
	ret = fts_i2c_Read(global_i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) 
	{
		dev_err(&global_i2c_client->dev, "%s read touchdata failed.\n",__func__);
		mutex_unlock(&i2c_access);
		return ret;
	}
	mutex_unlock(&i2c_access);

	buf_count_add++;
	//buf_count_add1=buf_count_add;
	memcpy( buf_touch_data+(((buf_count_add-1)%30)*POINT_READ_BUF), buf, sizeof(u8)*POINT_READ_BUF );





	return 0;
}

/************************************************************************
 * Name: fts_report_value
 * Brief: report the point information
 * Input: event info
 * Output: no
 * Return: success is zero
 ***********************************************************************/
static int fts_report_value(struct ts_event *data)
{
	//struct ts_event *event = NULL;
	int i = 0;
	int up_point = 0;
	int touchs = 0;
	u8 pointid = FTS_MAX_ID;
	u8 buf[POINT_READ_BUF] = { 0 };//0xFF

	buf_count_neg++;
	//buf_count_neg1=buf_count_neg;
	memcpy( buf,buf_touch_data+(((buf_count_neg-1)%30)*POINT_READ_BUF), sizeof(u8)*POINT_READ_BUF );


	memset(data, 0, sizeof(struct ts_event));
	data->touch_point_num=buf[FT_TOUCH_POINT_NUM] & 0x0F;

	data->touch_point = 0;
	//FT_CTP_DEBUG("tpd  fts_updateinfo_curr.TPD_MAX_POINTS=%d fts_updateinfo_curr.chihID=%d \n", fts_updateinfo_curr.TPD_MAX_POINTS,fts_updateinfo_curr.CHIP_ID);
	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)
	{
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			data->touch_point++;
		data->au16_x[i] =
			(s16) (buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F) <<
			8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];
		data->au16_y[i] =
			(s16) (buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F) <<
			8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];
		data->au8_touch_event[i] =
			buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
		data->au8_finger_id[i] =
			(buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;

		data->pressure[i] =
			(buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);//cannot constant value
		data->area[i] =
			(buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;
		if((data->au8_touch_event[i]==0 || data->au8_touch_event[i]==2)&&((data->touch_point_num==0)||(data->pressure[i]==0 && data->area[i]==0  )))
			return 1;



		//if ( pinfo->au16_x[i]==0 && pinfo->au16_y[i] ==0)
		//	pt00f++;
	}
	/*
	   if (pt00f>0)
	   {    
	   for(i=0;i<POINT_READ_BUF;i++)
	   {
	   printk(KERN_WARNING "The xy00 is %x \n",buf[i]);
	   }
	   printk(KERN_WARNING "\n");		
	   }
	   */			
	//event = data;
	for (i = 0; i < data->touch_point; i++) 
	{
		input_mt_slot(tpd->dev, data->au8_finger_id[i]);

		if (data->au8_touch_event[i]== 0 || data->au8_touch_event[i] == 2)
		{
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
			input_report_abs(tpd->dev, ABS_MT_PRESSURE,data->pressure[i]/*0x3f*/);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,data->area[i]/*0x05*/);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X,data->au16_x[i]);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y,data->au16_y[i]);
			touchs |= BIT(data->au8_finger_id[i]);
			data->touchs |= BIT(data->au8_finger_id[i]);
			//printk("tpd D x[%d] =%d,y[%d]= %d",i,event->au16_x[i],i,event->au16_y[i]);
		}
		else
		{
			up_point++;
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,false);
			data->touchs &= ~BIT(data->au8_finger_id[i]);
		}				 

	}
	if(unlikely(data->touchs ^ touchs)){
		for(i = 0; i < CFG_MAX_TOUCH_POINTS; i++){
			if(BIT(i) & (data->touchs ^ touchs)){
				input_mt_slot(tpd->dev, i);
				input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
			}
		}
	}
	data->touchs = touchs;
	if(data->touch_point == up_point)
		input_report_key(tpd->dev, BTN_TOUCH, 0);
	else
		input_report_key(tpd->dev, BTN_TOUCH, 1);

	input_sync(tpd->dev);
	return 0;
	//printk("tpd D x =%d,y= %d",event->au16_x[0],event->au16_y[0]);
}

#ifdef TPD_PROXIMITY
/************************************************************************
 * Name: tpd_read_ps
 * Brief: read proximity value
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
int tpd_read_ps(void)
{
	tpd_proximity_detect;
	return 0;    
}
/************************************************************************
 * Name: tpd_get_ps_value
 * Brief: get proximity value
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}
/************************************************************************
 * Name: tpd_enable_ps
 * Brief: enable proximity
 * Input: enable or not
 * Output: no
 * Return: 0
 ***********************************************************************/
static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;
	
	//i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);

	ret = fts_read_reg(global_i2c_client, 0xB0,&state);
	if (ret<0) 
	{
		printk("[Focal][Touch] read value fail");
		//return ret;
	}
	
	FT_CTP_DMESG("[proxi_fts]read: 999 0xb0's value is 0x%02X\n", state);

	if (enable)
	{
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is on\n");	
	}
	else
	{
		state &= 0x00;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is off\n");
	}
	
	//ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
	ret = fts_write_reg(global_i2c_client, 0xB0,&state);
	if (ret<0) 
	{
		FT_CTP_ERROR("[Focal][Touch] write value fail");
		//return ret;
	}
	TPD_PROXIMITY_DEBUG("[proxi_fts]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}
/************************************************************************
 * Name: tpd_ps_operate
 * Brief: operate function for proximity 
 * Input: point, which operation, buf_in , buf_in len, buf_out , buf_out len, no use
 * Output: buf_out
 * Return: fail <0
 ***********************************************************************/
int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,

		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_fts]command = 0x%02X\n", command);		

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
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				if((err = tpd_read_ps()))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = tpd_get_ps_value();
					TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
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
#endif

#if WT_CTP_GESTURE_SUPPORT
/************************************************************************
 * Name: check_gesture
 * Brief: report gesture id
 * Input: gesture id
 * Output: no
 * Return: no
 ***********************************************************************/
static void check_gesture(int gesture_id)
{
	//printk("fts gesture_id==0x%x\n ",gesture_id);
	switch(gesture_id)
	{
		case GESTURE_LEFT:
			gtp_gesture_value = 'L';
			input_report_key(tpd->dev, KEY_GESTURE, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE, 0);
			input_sync(tpd->dev);
			break;
		case GESTURE_RIGHT:
			gtp_gesture_value = 'R';
			input_report_key(tpd->dev, KEY_GESTURE, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE, 0);
			input_sync(tpd->dev);
			break;
		case GESTURE_DOUBLECLICK:
			gtp_gesture_value = 'K';
			input_report_key(tpd->dev, KEY_GESTURE, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE, 0);
			input_sync(tpd->dev);
			break;
		case GESTURE_O:
			gtp_gesture_value = 'o';
			input_report_key(tpd->dev, KEY_GESTURE, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE, 0);
			input_sync(tpd->dev);
			break;
		case GESTURE_V:
			gtp_gesture_value = 'v';
			input_report_key(tpd->dev, KEY_GESTURE, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE, 0);
			input_sync(tpd->dev);
			break;
		default:
			break;
	}
}
/************************************************************************
 * Name: fts_read_Gestruedata
 * Brief: read data from TP register
 * Input: no
 * Output: no
 * Return: fail <0
 ***********************************************************************/
static int fts_read_Gestruedata(void)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;
	int gestrue_id = 0;
	short pointnum = 0;

	pointnum = 0;
	ret = fts_i2c_Read(global_i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	//printk( "tpd read FTS_GESTRUE_POINTS_HEADER.\n");
	if (ret < 0)
	{
		FT_CTP_ERROR( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	//Ë«»÷»½ÐÑ
	if (0x24 == buf[0])
	{
		gestrue_id = 0x24;
		check_gesture(gestrue_id);
		FT_CTP_ERROR( "tpd %d check_gesture gestrue_id.\n", gestrue_id);
		return -1;
	}

	/* FW */
	if (fts_updateinfo_curr.CHIP_ID==0x54|| fts_updateinfo_curr.CHIP_ID==0x58)
	{
		gestrue_id = buf[0];
		pointnum = (short)(buf[1]) & 0xff;
		buf[0] = 0xd3;

		if((pointnum * 4 + 8)<255)
		{
			ret = fts_i2c_Read(global_i2c_client, buf, 1, buf, (pointnum * 4 + 8));
		}
		else
		{
			ret = fts_i2c_Read(global_i2c_client, buf, 1, buf, 255);
			ret = fts_i2c_Read(global_i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
		}
		if (ret < 0)
		{
			FT_CTP_ERROR( "%s read touchdata failed.\n", __func__);
			return ret;
		}
		check_gesture(gestrue_id);
		for(i = 0;i < pointnum;i++)
		{
			coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
				8 | (((s16) buf[1 + (4 * i)])& 0xFF);
			coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
				8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
		}
		return -1;
	}

	return -1;
}
#endif

#if CTP_CHARGER_DETECT
void tp_charger_detect_op(void){
	current_charger_status = (u8)bat_is_charger_exist();
	//FT_CTP_DEBUG("charger_status:%d", current_charger_status);
	if(current_charger_status != prv_charger_status){
		prv_charger_status = current_charger_status;
		fts_write_reg(global_i2c_client,0x8B,current_charger_status);
	}
}
#endif

/************************************************************************
 * Name: touch_event_handler
 * Brief: interrupt event from TP, and read/report data to Android system 
 * Input: no use
 * Output: no
 * Return: 0
 ***********************************************************************/
static int touch_event_handler(void *unused)
{
	struct touch_info cinfo, pinfo;
	struct ts_event pevent;
	int i=0;
	int ret = 0;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
#endif
	u8 state;
	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

#if CTP_CHARGER_DETECT
		tp_charger_detect_op();
#endif

#if WT_CTP_GESTURE_SUPPORT
		if(gtp_gesture_onoff == '1'){
			ret = fts_read_reg(global_i2c_client, 0xd0,&state);
			FT_CTP_DEBUG("in event gesture:%d\n", ret);
			if (ret<0) 
			{
				FT_CTP_ERROR("read value fail");
			}
			if(state ==1)
			{
				fts_read_Gestruedata();
				continue;
			}
		}
#endif

#ifdef TPD_PROXIMITY

		if (tpd_proximity_flag == 1)
		{

				//i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);

				ret = fts_read_reg(global_i2c_client, 0xB0,&state);
				if (ret<0) 
				{
					FT_CTP_ERROR("read value fail");
					//return ret;
				}
			TPD_PROXIMITY_DEBUG("proxi_fts 0xB0 state value is 1131 0x%02X\n", state);
			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}
				//i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
				ret = fts_read_reg(global_i2c_client, 0x01,&proximity_status);
				if (ret<0) 
				{
					FT_CTP_ERROR(" read value fail");
					//return ret;
				}
			TPD_PROXIMITY_DEBUG("proxi_fts 0x01 value is 1139 0x%02X\n", proximity_status);
			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);
			if ((err = tpd_read_ps()))
			{
				TPD_PROXIMITY_DMESG("proxi_fts read ps data 1156: %d\n", err);	
			}
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			//if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			//{
			//	TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);	
			//}
		}  

#endif

#ifdef MT_PROTOCOL_B
		{
			ret = fts_read_Touchdata(&pevent);
			//if (ret == 0)
			fts_report_value(&pevent);
		}
#else
		{
			if (tpd_touchinfo(&cinfo, &pinfo)) 
			{
				//FT_CTP_DEBUG("tpd point_num = %d\n",point_num);
				TPD_DEBUG_SET_TIME;
				if(point_num >0) 
				{
					for(i =0; i<point_num; i++)//only support 3 point
					{
						FT_CTP_DEBUG("TouchFinger:%d,x:%d, y:%d\n",i+1,cinfo.x[i], cinfo.y[i] );
						tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
					}
					input_sync(tpd->dev);
				}
				else  
				{
					tpd_up(cinfo.x[0], cinfo.y[0],&cinfo.id[0]);
					FT_CTP_DEBUG("release --->\n");         	   
					input_sync(tpd->dev);
				}
			}
		}
#endif
	}while(!kthread_should_stop());
	return 0;
}
/************************************************************************
 * Name: fts_reset_tp
 * Brief: reset TP
 * Input: pull low or high
 * Output: no
 * Return: 0
 ***********************************************************************/
void fts_reset_tp(int HighOrLow)
{
	if(HighOrLow)
	{
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
	}
	else
	{
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	}
}
/************************************************************************
 * Name: tpd_detect
 * Brief: copy device name
 * Input: i2c info, board info
 * Output: no
 * Return: 0
 ***********************************************************************/
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);	
	return 0;
}
/************************************************************************
 * Name: tpd_eint_interrupt_handler
 * Brief: deal with the interrupt event
 * Input: no
 * Output: no
 * Return: no
 ***********************************************************************/
static void tpd_eint_interrupt_handler(void)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	 #ifdef GTP_ESD_PROTECT
		count_irq ++;
	 #endif
	wake_up_interruptible(&waiter);
}
/************************************************************************
 * Name: fts_init_gpio_hw
 * Brief: initial gpio
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
static int fts_init_gpio_hw(void)
{

	int ret = 0;
	int i = 0;

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	return ret;
}


#ifdef   CTP_OPEN_SHORT_TEST                                                                                                                                                                                                                             
static ssize_t  opentest_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos);
static ssize_t  opentest_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos);
static  const struct file_operations  proc_opentest_file_fops = 
{
    .write = opentest_proc_write,
    .read = opentest_proc_read,
    //.open = simple_open,
    .owner = THIS_MODULE,
};

static int focal_i2c_Read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen)
{
		int ret;  
		
		ret=fts_i2c_Read(g_focalclient, writebuf,writelen, readbuf, readlen);
		if (ret < 0)
			//dev_err(&g_focalclient->dev, "%s:i2c read error.\n", __func__);
			FT_CTP_DEBUG( "focal_i2c_Read:i2c read error\n");

	       return ret;
}

static int focal_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret;

	  ret=fts_i2c_Write(g_focalclient,writebuf,  writelen);
	  if (ret < 0)
		//dev_err(&g_focalclient->dev, "%s i2c write error.\n", __func__);
		FT_CTP_DEBUG( "focal_i2c_Read:i2c write error\n");
	
	return ret;
}

#define  FT5446_INI_FILEPATH    "/system/etc/"        
static int ft5446_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5446_INI_FILEPATH , config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5446_ReadInIData(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5446_INI_FILEPATH, config_name);
	
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
 
	return 0;
}

static int ft5446_get_testparam_from_ini(char *config_name)
{
	char *filedata = NULL;
  	int inisize;

	  inisize = ft5446_GetInISize(config_name);
	  FT_CTP_DEBUG("*** inisize = %d  *** \n ", inisize); 
		if (inisize <= 0)
		{
		   FT_CTP_DEBUG(" *** %s ERROR:Get firmware size failed *** \n",
					__func__);
		   return -EIO;
	          }

	filedata = kmalloc(inisize + 1, GFP_KERNEL);
	
	if (ft5446_ReadInIData(config_name, filedata)) {
		FT_CTP_DEBUG("  *** %s() - ERROR: request_firmware failed ***\n",
					__func__);
		kfree(filedata);
		return -EIO;
	} else {
		FT_CTP_DEBUG("*** ft5446_ReadInIData successful *** \n");
	}
	SetParamData(filedata);
	return 0;
}

static ssize_t  opentest_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
		return -1;
}

static ssize_t  opentest_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
    char *ptr = buf;
    char  *cfgname = NULL;
    unsigned char  uc_reg_addr;
     unsigned char  ctp_vendor_id;
    int  retval = -1;
  
    if(*ppos)
    {
        FT_CTP_DEBUG("tp test again return\n");
        return 0;
    }
    *ppos += count;

      uc_reg_addr = FTS_REG_VENDOR_ID;
      fts_i2c_Write(global_i2c_client, &uc_reg_addr, 1);
      retval=fts_i2c_Read(global_i2c_client, &uc_reg_addr, 0, &ctp_vendor_id, 1);
      FT_CTP_DEBUG("vendor  id = 0x%x.\n",ctp_vendor_id);
     if(retval<0)
	{
		FT_CTP_ERROR("Read I2C error! driver NOt load!! CTP chip id is %d.\n", ctp_vendor_id);
		return 0;
	}

     switch(ctp_vendor_id)
  	{
	  	case 0x02:                                                                           /*shenyue_black TP module*/
			cfgname = "ft5446_pard.ini";
		break;
		case 0xFD:                                                                          /*shenyue_white TP module*/
			cfgname = "shenyue_white_pard.ini";
		break;
		case 0xAE:                                                                          /*oufei_black TP module*/
			cfgname = "oufei_black_pard.ini";
		break;
		case 0x51:                                                                          /*oufei_white TP module*/
			cfgname = "oufei_white_pard.ini";
		break;
  	  }

    FT_CTP_DEBUG(" *** cfgname_1 is: %s *** \n", cfgname); 
    
    Init_I2C_Write_Func(focal_i2c_Write);
    Init_I2C_Read_Func(focal_i2c_Read);
    
    if(ft5446_get_testparam_from_ini(cfgname) <0)                
    {
        FT_CTP_DEBUG("get testparam from ini failure\n");
        sprintf(ptr, "result=%d\n", 0);
    }
    else 
    	{
        if(true == StartTestTP())                                                       
        {
            FT_CTP_DEBUG("tp_openclose_test pass\n");
            sprintf(ptr, "result=%d\n", 1);
        }
        else
        {
            FT_CTP_DEBUG("tp_openclose_test failure\n");
            sprintf(ptr, "result=%d\n", 0);
        }
		
        FreeTestParamData();
    }
    return count;
}

static ssize_t  shorttest_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos);
static ssize_t  shorttest_proc_write(struct file *file, char __user *buf,size_t count, loff_t *ppos);
static  const struct file_operations  proc_shorttest_file_fops = 
{
	  .write = shorttest_proc_write,
    .read = shorttest_proc_read,
    //.open = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t  shorttest_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
		char *ptr = buf;
		
		if(*ppos)
    {
        return 0;
    }
    *ppos += count;
    
    FT_CTP_DEBUG("shorttest read");
    return sprintf(ptr, "no test\n");
}

static ssize_t  shorttest_proc_write(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{    
	  FT_CTP_DEBUG("tpd_shorttest write");
    return -1;
}

static void create_tp_proc(void)
{
	struct proc_dir_entry  *msg_tp_test = NULL;
	struct proc_dir_entry  *opentest = NULL;
	struct proc_dir_entry *shorttest = NULL;
	struct proc_dir_entry *testresult = NULL;
        
        msg_tp_test = proc_mkdir(FT5446_PROC_MSG_ITO_TESE, NULL);
         if (NULL==msg_tp_test ) 
    	  {
       	  		FT_CTP_DEBUG("creating  /proc/ctp/  failed\n");
   	    }      
   	    
    	 opentest = proc_create( FT5446_PROC_ITO_OPENTEST_DEBUG, ITO_TEST_AUTHORITY, msg_tp_test,&proc_opentest_file_fops);
			 if (NULL==opentest) 
    	  {
       	  	FT_CTP_DEBUG("creating  /proc/ctp/openteset  failed\n");
   	 		 } 

    	 shorttest= proc_create(FT5446_PROC_ITO_SHORTTEST_DEBUG,ITO_TEST_AUTHORITY, msg_tp_test, &proc_shorttest_file_fops);
  		 if (NULL == shorttest)
   		 {
       		 FT_CTP_DEBUG("create /proc/ctp/shorttest fail\n");
   		 }
	 #ifdef TP_TESTRESULTPROC
   		 testresult= proc_create(FT5446_PROC_ITO_TTESTRESULT_DEBUG,ITO_TEST_AUTHORITY, msg_tp_test, &ctp_testresult_procs_fops);
  		 if (NULL == testresult)
   		 {
       		 FT_CTP_DEBUG("create /proc/ctp_test/testresult fail\n");
   		 }
      #endif
}
#endif


#if WT_CTP_GESTURE_SUPPORT

static int proc_gesture_data_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	int num = 0;
	if(*ppos) //CMD call again
		return 0;
    printk("proc_gesture_data_read=%d\n",gtp_gesture_value);
    num =  sprintf(buffer,"%c\n",gtp_gesture_value);
	*ppos += num;
	return num;
}

static int proc_gesture_data_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    sscanf(buffer,"%c",&gtp_gesture_value);
    return count;
}

static int proc_gesture_type_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	int num;
	if(*ppos) //CMD call again
	{	printk("%s\n",__func__);
		return 0;
		}
    num = sprintf(buffer,"%s\n",gtp_gesture_type);
	*ppos += num;
	return num;

}

static int proc_gesture_type_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{

    return -1;
}

static int proc_gesture_onoff_read(struct file *file, char *page, size_t count, loff_t *ppos)
{
	FT_CTP_DMESG("in proc_gesture_onoff_read \n");
	int num;
	if(*ppos) //CMD call again
		return 0;
    if(NULL==gtp_gesture_support_flag_changed) 
		num = sprintf(page,"%c\n",gtp_gesture_support_flag);
    else 
		num = sprintf(page,"%c\n",gtp_gesture_onoff);
	*ppos += num;
	return num;

}

static int proc_gesture_onoff_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	FT_CTP_DMESG("in proc_gesture_onoff_write \n");
    sscanf(buffer,"%c",&gtp_gesture_onoff);
    gtp_gesture_support_flag_changed=1;
	FT_CTP_DMESG("in proc_gesture_onoff_write %c \n", gtp_gesture_onoff);
    return count;
}
static int proc_glove_onoff_read(struct file *file, char *page, size_t count, loff_t *ppos)
{
	int num;
	if(*ppos) //CMD call again
		return 0;
    if(NULL==gtp_glove_support_flag_changed) 
		num = sprintf(page,"%c\n",gtp_glove_support_flag);
    else 
		num = sprintf(page,"%c\n",gtp_glove_onoff);
	*ppos += num;
	return num;

}

static int proc_glove_onoff_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    sscanf(buffer,"%c",&gtp_glove_onoff);
    gtp_glove_support_flag_changed=1;
    msleep(100);
    return count;
}

static int proc_version_read(struct file *file, char *page, size_t count, loff_t *ppos)
{
	int num;
	if(*ppos) //CMD call again
		return 0;
    num = sprintf(page,"%s\n",gtp_verson);
	*ppos += num;
	return num;
}

static int proc_version_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    return count;
}


static int proc_gesture_coordinate_read(struct file *file, char *page, size_t count, loff_t *ppos)
{
	if(*ppos) //CMD call again
		return 0;
	memcpy(page,gtp_gesture_coordinate,60);
	printk("GTP gtp_ges 0x%02x\n", gtp_gesture_coordinate[13]);
	memset(gtp_gesture_coordinate, 0, 60);
	*ppos += 60;
	return 60;
}

static int proc_gesture_coordinate_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    return count;
}

static const struct file_operations gt_gesture_var_proc_fops = { 
    .write = proc_gesture_data_write,
    .read = proc_gesture_data_read,
};

static const struct file_operations gt_gesture_type_proc_fops = { 
    .write = proc_gesture_type_write,
    .read = proc_gesture_type_read,
};

static const struct file_operations gt_gesture_onoff_proc_fops = { 
    .write = proc_gesture_onoff_write,
    .read = proc_gesture_onoff_read,
};

static const struct file_operations gt_glove_onoff_proc_fops = { 
    .write = proc_glove_onoff_write,
    .read = proc_glove_onoff_read,
};

static const struct file_operations gt_version_proc_fops = { 
    .write = proc_version_write,
    .read = proc_version_read,
};

static const struct file_operations gt_coordinate_proc_fops = { 
    .write = proc_gesture_coordinate_write,
    .read = proc_gesture_coordinate_read,
};


void Ctp_Gesture_Fucntion_Proc_File(void)
{
    struct proc_dir_entry *ctp_device_proc = NULL;
    struct proc_dir_entry *ctp_gesture_var_proc = NULL;
    struct proc_dir_entry *ctp_gesture_type_proc = NULL;
    struct proc_dir_entry *ctp_gesture_onoff_proc = NULL;
    struct proc_dir_entry *ctp_glove_onoff_proc = NULL;
    struct proc_dir_entry *ctp_version_proc = NULL;
    struct proc_dir_entry *ctp_coordinate_proc = NULL;
#define CTP_GESTURE_FUNCTION_AUTHORITY_PROC 0777

    ctp_device_proc = proc_mkdir("touchscreen_feature", NULL);

    ctp_gesture_var_proc = proc_create("gesture_data", 0444, ctp_device_proc, &gt_gesture_var_proc_fops);
    if (ctp_gesture_var_proc == NULL)
    {
        TPD_DEBUG("ctp_gesture_var_proc create failed\n");
    }
 
    ctp_gesture_type_proc = proc_create("gesture_type", 0444, ctp_device_proc, &gt_gesture_type_proc_fops);
    if (ctp_gesture_type_proc == NULL)
    {
        TPD_DEBUG("ctp_gesture_type_proc create failed\n");
    }
   
    ctp_gesture_onoff_proc = proc_create("gesture_onoff", 0666, ctp_device_proc, &gt_gesture_onoff_proc_fops);
    if (ctp_gesture_onoff_proc == NULL)
    {
        TPD_DEBUG("ctp_gesture_onoff_proc create failed\n");
    }

    ctp_glove_onoff_proc = proc_create("glove_onoff", 0666, ctp_device_proc, &gt_glove_onoff_proc_fops);
    if (ctp_glove_onoff_proc == NULL)
    {
        TPD_DEBUG("ctp_gesture_onoff_proc create failed\n");
    }
  
    ///////////////uupdate
    ctp_version_proc = proc_create("version", 0444, ctp_device_proc, &gt_version_proc_fops);
    if (ctp_version_proc == NULL)
    {
        TPD_DEBUG("create_proc_entry version failed\n");
    }

    ctp_coordinate_proc = proc_create("gesture_coordinate", 0777, ctp_device_proc, &gt_coordinate_proc_fops);
    if (ctp_coordinate_proc == NULL)
    {
        TPD_DEBUG("create_proc_entry version failed\n");
    }

}


#endif

/************************************************************************
 * Name: tpd_probe
 * Brief: driver entrance function for initial/power on/create channel 
 * Input: i2c info, device id
 * Output: no
 * Return: 0
 ***********************************************************************/
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	int retval = -1;
	char data;
	u8 report_rate=0;
	//int err=0;
	int reset_count = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
#ifdef TPD_PROXIMITY
	int err;
	struct hwmsen_object obj_ps;
#endif


#ifdef  CTP_OPEN_SHORT_TEST
      g_focalclient = client;                                     
#endif

	global_i2c_client = client;
	if(global_i2c_client == NULL){
		FT_CTP_DMESG("client is null ,fatal error");
		goto out;
	}
reset_proc:   
	//step 1: pull down reset to reset IC
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(10);
		

	//step 2:power on
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 


	//step 3:let ic run
	msleep(10);
    	FT_CTP_DMESG(" fts reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	msleep(150);

	
	FT_CTP_DEBUG("begin dma alloc");
	retval = msg_dma_alloct();
	if(retval < 0){
		goto out;
	}

	fts_init_gpio_hw();

	uc_reg_addr = FTS_REG_POINT_RATE;				
	fts_i2c_Write(global_i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(global_i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	FT_CTP_DEBUG("report rate is %dHz.\n",uc_reg_value * 10);

	uc_reg_addr = FTS_REG_FW_VER;
	fts_i2c_Write(global_i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(global_i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	FT_CTP_DMESG("Firmware version = 0x%x\n", uc_reg_value);
	tp_info.fw_version = uc_reg_value;

	uc_reg_addr = FTS_REG_VENDOR_ID;
	fts_i2c_Write(global_i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(global_i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	FT_CTP_DMESG("tp vendor id = 0x%x\n", uc_reg_value);
	tp_info.vendor_id = uc_reg_value;

	uc_reg_addr = FTS_REG_CHIP_ID;
	fts_i2c_Write(global_i2c_client, &uc_reg_addr, 1);
	retval=fts_i2c_Read(global_i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	FT_CTP_DMESG("chip id = 0x%x.\n",uc_reg_value);
	tp_info.chip_id = uc_reg_value;
	
	if(retval<0)
	{
		FT_CTP_ERROR("Read I2C error! driver NOt load!! CTP chip id is %d.\n",uc_reg_value);
		return 0;
	}

#if WT_ADD_CTP_INFO
	if(tp_info.vendor_id== 0x02){
		sprintf(tp_string_version,"FT5446,shenyue black,fw_ver:%2x", tp_info.fw_version);
	}else if(tp_info.vendor_id== 0xfd){
		sprintf(tp_string_version,"FT5446,shenyue white,fw_ver:%2x", tp_info.fw_version);
	}else if(tp_info.vendor_id== 0x51){
		sprintf(tp_string_version,"FT5446,oufei white,fw_ver:%2x", tp_info.fw_version);
	}else if(tp_info.vendor_id== 0xae){
		sprintf(tp_string_version,"FT5446,oufe black,fw_ver:%2x", tp_info.fw_version);
	}else{
		sprintf(tp_string_version,"FT5446,unknwon,fw_ver:%2x", tp_info.fw_version);
	}
	
	hardwareinfo_set_prop(HARDWARE_TP, tp_string_version);
#endif

	tpd_load_status = 1;
	/*
	   mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	   mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	   mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	   mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	   */
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef  CTP_OPEN_SHORT_TEST
	create_tp_proc();  
#endif



#ifdef VELOCITY_CUSTOM_fts
	if((err = misc_register(&tpd_misc_device)))
	{
		FT_CTP_ERROR("mtk_tpd: tpd_misc_device register failed\n");

	}
#endif

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{ 
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}



#ifdef SYSFS_DEBUG
	fts_create_sysfs(global_i2c_client);
#endif

	fts_get_upgrade_array();

#ifdef FTS_CTL_IIC
	if (fts_rw_iic_drv_init(global_i2c_client) < 0)
		FT_CTP_ERROR("%s:[FTS] create fts control iic driver failed\n", __func__);
#endif

#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(global_i2c_client);
#endif

#if TPD_AUTO_UPGRADE
	FT_CTP_DMESG("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(global_i2c_client);
#endif

#ifdef TPD_PROXIMITY
	{
		obj_ps.polling = 1; //0--interrupt mode;1--polling mode;
		obj_ps.sensor_operate = tpd_ps_operate;
		if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
		{
			TPD_DEBUG("hwmsen attach fail, return:%d.", err);
		}
	}
#endif
#ifdef GTP_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
	queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif

#if WT_CTP_GESTURE_SUPPORT
	//init_para(480,854,60,0,0);

	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE);

	__set_bit(KEY_GESTURE, tpd->dev->keybit);

	Ctp_Gesture_Fucntion_Proc_File();

#endif

#ifdef MT_PROTOCOL_B
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
		input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS);
	#endif
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
out:
   	FT_CTP_DMESG("fts Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
   	return 0;
   
 }
/************************************************************************
 * Name: tpd_remove
 * Brief: remove driver/channel
 * Input: i2c info
 * Output: no
 * Return: 0
 ***********************************************************************/
static int __devexit tpd_remove(struct i2c_client *client)

{
	msg_dma_release();

#ifdef FTS_CTL_IIC
	FT_CTP_DEBUG("ii2c_drv_exit----1");
	fts_rw_iic_drv_exit();
	FT_CTP_DEBUG("ii2c_drv_exit----2");
#endif
	
#ifdef SYSFS_DEBUG
	FT_CTP_DEBUG("release_sysfs----1");
	fts_release_sysfs(client);
	FT_CTP_DEBUG("release_sysfs----2");
#endif


#ifdef GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

#ifdef FTS_APK_DEBUG
	fts_release_apk_debug_channel();
#endif

	TPD_DEBUG("TPD removed\n");

	return 0;
}
#ifdef GTP_ESD_PROTECT
/************************************************************************
 * Name: force_reset_guitar
 * Brief: reset
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
static void force_reset_guitar(void)
{
	s32 i;
	s32 ret;

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(10);
    	TPD_DMESG("force_reset_guitar\n");

	hwPowerDown(MT6323_POWER_LDO_VGP1,  "TP");
	msleep(200);
	hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
	msleep(5);

	
	msleep(10);
	TPD_DMESG(" fts ic reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	msleep(300);

#ifdef TPD_PROXIMITY
	if (FT_PROXIMITY_ENABLE == tpd_proximity_flag) 
	{
		tpd_enable_ps(FT_PROXIMITY_ENABLE);
	}
#endif
}
//0 for no apk upgrade, 1 for apk upgrade
extern int apk_debug_flag; 
#define A3_REG_VALUE								0x54
#define RESET_91_REGVALUE_SAMECOUNT 				5
static u8 g_old_91_Reg_Value = 0x00;
static u8 g_first_read_91 = 0x01;
static u8 g_91value_same_count = 0;
/************************************************************************
 * Name: gtp_esd_check_func
 * Brief: esd check function
 * Input: struct work_struct
 * Output: no
 * Return: 0
 ***********************************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	int i;
	int ret = -1;
	u8 data, data_old;
	u8 flag_error = 0;
	int reset_flag = 0;
	u8 check_91_reg_flag = 0;

	if (tpd_halt ) 
	{
		return;
	}
	if(apk_debug_flag) 
	{
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
		return;
	}

	run_check_91_register = 0;
	for (i = 0; i < 3; i++) 
	{
		//ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0xA3, 1, &data);
		ret = fts_read_reg(global_i2c_client, 0xA3,&data);
		if (ret<0) 
		{
			FT_CTP_ERROR("[Focal][Touch] read value fail");
			//return ret;
		}
		if (ret==1 && A3_REG_VALUE==data) 
		{
			break;
		}
	}

	if (i >= 3) 
	{
		force_reset_guitar();
		FT_CTP_DEBUG("focal--tpd reset. i >= 3  ret = %d	A3_Reg_Value = 0x%02x\n ", ret, data);
		reset_flag = 1;
		goto FOCAL_RESET_A3_REGISTER;
	}

	//esd check for count
  	//ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x8F, 1, &data);
	ret = fts_read_reg(global_i2c_client, 0x8F,&data);
	if (ret<0) 
	{
		FT_CTP_ERROR("[Focal][Touch] read value fail");
		//return ret;
	}
	FT_CTP_DEBUG("0x8F:%d, count_irq is %d\n", data, count_irq);

	flag_error = 0;
	if((count_irq - data) > 10) 
	{
		if((data+200) > (count_irq+10) )
		{
			flag_error = 1;
		}
	}

	if((data - count_irq ) > 10) 
	{
		flag_error = 1;		
	}

	if(1 == flag_error) 
	{	
		FT_CTP_DEBUG("focal--tpd reset.1 == flag_error...data=%d	count_irq\n ", data, count_irq);
		force_reset_guitar();
		reset_flag = 1;
		goto FOCAL_RESET_INT;
	}

	run_check_91_register = 1;
	//ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x91, 1, &data);
	ret = fts_read_reg(global_i2c_client, 0x91,&data);
	if (ret<0) 
	{
		FT_CTP_ERROR("[Focal][Touch] read value fail");
		//return ret;
	}
	FT_CTP_DEBUG("focal---------91 register value = 0x%02x	old value = 0x%02x\n",	data, g_old_91_Reg_Value);
	if(0x01 == g_first_read_91) 
	{
		g_old_91_Reg_Value = data;
		g_first_read_91 = 0x00;
	} 
	else 
	{
		if(g_old_91_Reg_Value == data)
		{
			g_91value_same_count++;
			FT_CTP_DEBUG("focal 91 value ==============, g_91value_same_count=%d\n", g_91value_same_count);
			if(RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count) 
			{
				force_reset_guitar();
				FT_CTP_DEBUG("focal--tpd reset. g_91value_same_count = 5\n");
				g_91value_same_count = 0;
				reset_flag = 1;
			}

			//run_check_91_register = 1;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE / 2;
			g_old_91_Reg_Value = data;
		} 
		else 
		{
			g_old_91_Reg_Value = data;
			g_91value_same_count = 0;
			//run_check_91_register = 0;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE;
		}
	}
FOCAL_RESET_INT:
FOCAL_RESET_A3_REGISTER:
	count_irq=0;
	data=0;
	//fts_i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
	ret = fts_write_reg(global_i2c_client, 0x8F,data);
	if (ret<0) 
	{
		printk("[Focal][Touch] write value fail");
		//return ret;
	}
	if(0 == run_check_91_register)
	{
		g_91value_same_count = 0;
	}
	#ifdef TPD_PROXIMITY
	if( (1 == reset_flag) && ( FT_PROXIMITY_ENABLE == tpd_proximity_flag) )
	{
		if((tpd_enable_ps(FT_PROXIMITY_ENABLE) != 0))
		{
			APS_ERR("enable ps fail\n"); 
			return -1;
		}
	}
	#endif
	//end esd check for count

	if (!tpd_halt)
	{
		//queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
	}

	return;
}
#endif

/************************************************************************
 * Name: tpd_local_init
 * Brief: add driver info
 * Input: no
 * Output: no
 * Return: fail <0
 ***********************************************************************/
static int tpd_local_init(void)
{
	TPD_DMESG("Focaltech fts I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TPD_DMESG("fts unable to add i2c driver.\n");
		return -1;
	}
	if(tpd_load_status == 0) 
	{
		TPD_DMESG("fts add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	//TINNO_TOUCH_TRACK_IDS <--- finger number
	//TINNO_TOUCH_TRACK_IDS	5
	#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0))
		//for linux 3.8
		input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (TPD_MAX_POINTS_5-1), 0, 0);
	#endif

	
   	#ifdef TPD_HAVE_BUTTON     
		// initialize tpd button data
    		tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
	#endif   
  
	#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    		TPD_DO_WARP = 1;
    		memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    		memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
	#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
	return 0; 
}
/************************************************************************
 * Name: tpd_resume
 * Brief: system wake up 
 * Input: no use
 * Output: no
 * Return: no
 ***********************************************************************/
static void tpd_resume( struct early_suspend *h )
{
	int i=0,ret = 0;

	TPD_DMESG("TPD wake up\n");
	buf_addr[0]=0xC0;
	buf_addr[1]=0x8B;

	for(i=0;i<2;i++)
	{
		ret = fts_write_reg(global_i2c_client, buf_addr[i], buf_value[i]);
		if (ret<0) 
		{
			FT_CTP_ERROR("[Focal][Touch] write value fail");
			//return ret;
		}
	}

#ifdef TPD_PROXIMITY	
	if (tpd_proximity_flag == 1)
	{
		if(tpd_proximity_flag_one == 1)
		{
			tpd_proximity_flag_one = 0;	
			TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
			return;
		}
	}
#endif	

#if CTP_CHARGER_DETECT
	tp_charger_detect_op();
#endif


#if WT_CTP_GESTURE_SUPPORT
	fts_write_reg(global_i2c_client,0xD0,0x00);
	FT_CTP_DEBUG("in resume gesture\n");
#endif
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
#else

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(5);  
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	msleep(30);
	tpd_halt = 0;
	
	#ifdef GTP_ESD_PROTECT
		count_irq = 0;
    		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
	#endif

	TPD_DMESG("TPD wake up done\n");

}
/************************************************************************
 * Name: tpd_suspend
 * Brief: system sleep
 * Input: no use
 * Output: no
 * Return: no
 ***********************************************************************/
static void tpd_suspend( struct early_suspend *h )
{
	char data = 0x3;
	int i=0,ret = 0;
	buf_addr[0]=0xC0;
	buf_addr[1]=0x8B;

	for(i=0;i<2;i++)
	{
		ret = fts_read_reg(global_i2c_client, buf_addr[i], (buf_value+i));
		if (ret<0) 
		{
			FT_CTP_ERROR("[Focal][Touch] read value fail");
			//return ret;
		}
	}

#if CTP_CHARGER_DETECT
	tp_charger_detect_op();
#endif


	FT_CTP_DMESG("TPD enter sleep\n");
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_proximity_flag_one = 1;	
		return;
	}
#endif

#if WT_CTP_GESTURE_SUPPORT
	FT_CTP_DEBUG("suspend gesture onoff %c", gtp_gesture_onoff);
	if(gtp_gesture_onoff == '1'){
		fts_write_reg(global_i2c_client, 0xd0, 0x01);
		if (fts_updateinfo_curr.CHIP_ID==0x54 || fts_updateinfo_curr.CHIP_ID==0x58)
		{
			fts_write_reg(global_i2c_client, 0xd1, 0xff);
			fts_write_reg(global_i2c_client, 0xd2, 0xff);
			fts_write_reg(global_i2c_client, 0xd5, 0xff);
			fts_write_reg(global_i2c_client, 0xd6, 0xff);
			fts_write_reg(global_i2c_client, 0xd7, 0xff);
			fts_write_reg(global_i2c_client, 0xd8, 0xff);
		}
		FT_CTP_DEBUG("in suspend gesture\n");
		return;
	}
#endif
#ifdef GTP_ESD_PROTECT
	cancel_delayed_work_sync(&gtp_esd_check_work);
#endif
	tpd_halt = 1;

	 mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	 FT_CTP_DMESG("mask interrupt");
	 mutex_lock(&i2c_access);
	#ifdef TPD_CLOSE_POWER_IN_SLEEP	
		hwPowerDown(TPD_POWER_SOURCE,"TP");
	#else
		FT_CTP_DEBUG("write data %x", data);
		ret = fts_write_reg(global_i2c_client, 0xA5, data);
		if (ret<0) 
		{
			FT_CTP_ERROR("[Focal][Touch] write value fail");
			//return ret;
		}
	#endif
	mutex_unlock(&i2c_access);

	FT_CTP_DMESG("TPD enter sleep done\n");

} 


static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "fts",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
};

/************************************************************************
 * Name: tpd_suspend
 * Brief:  called when loaded into kernel
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
static int __init tpd_driver_init(void) {
	FT_CTP_DMESG("MediaTek fts touch panel driver init\n");
	i2c_register_board_info(TPD_I2C_NUMBER, &fts_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add fts driver failed\n");
	return 0;
}


/************************************************************************
 * Name: tpd_driver_exit
 * Brief:  should never be called
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
static void __exit tpd_driver_exit(void) 
{
	TPD_DMESG("MediaTek fts touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


