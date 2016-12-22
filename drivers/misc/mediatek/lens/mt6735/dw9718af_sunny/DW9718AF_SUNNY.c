/*
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "DW9718AF_SUNNY.h"
#include "../camera/kd_camera_hw.h"
#include <linux/xlog.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif


#define LENS_I2C_BUSNUM 0
//static struct i2c_board_info kd_lens_dev __initdata = { I2C_BOARD_INFO("DW9718AF_SUNNY", 0x18) };
static struct i2c_board_info kd_lens_dev __initdata = { I2C_BOARD_INFO("DW9718AF_SUNNY", 0x19) };


#define DW9718AF_SUNNY_DRVNAME "DW9718AF_SUNNY"
#define DW9718AF_SUNNY_VCM_WRITE_ID           0x18
#define PLATFORM_DRIVER_NAME "lens_actuator_dw9718af_sunny"

#define DW9718AF_SUNNY_DEBUG
#ifdef DW9718AF_SUNNY_DEBUG
#define DW9718AF_SUNNYDB pr_debug
#else
#define DW9718AF_SUNNYDB(x, ...)
#endif

static spinlock_t g_DW9718AF_SUNNY_SpinLock;

static struct i2c_client *g_pstDW9718AF_SUNNY_I2Cclient;

static dev_t g_DW9718AF_SUNNY_devno;
static struct cdev *g_pDW9718AF_SUNNY_CharDrv;
static struct class *actuator_class;

static int g_s4DW9718AF_SUNNY_Opened;
static long g_i4MotorStatus;
static long g_i4Dir;
static unsigned long g_u4DW9718AF_SUNNY_INF;
static unsigned long g_u4DW9718AF_SUNNY_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int g_sr = 3;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[1] = { (char)(a_u2Addr) };
	i4RetValue = i2c_master_send(g_pstDW9718AF_SUNNY_I2Cclient, puReadCmd, 1);
	if (i4RetValue != 2) {
		DW9718AF_SUNNYDB(" I2C write failed!!\n");
		return -1;
	}
	/*  */
	i4RetValue = i2c_master_recv(g_pstDW9718AF_SUNNY_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
		DW9718AF_SUNNYDB(" I2C read failed!!\n");
		return -1;
	}

	return 0;
}

static u8 read_data(u8 addr)
{
	u8 get_byte = 0;
	i2c_read(addr, &get_byte);
	DW9718AF_SUNNYDB("[DW9718AF_SUNNY]  get_byte %d\n", get_byte);
	return get_byte;
}

static int s4DW9718AF_SUNNY_ReadReg(unsigned short *a_pu2Result)
{
	/* int  i4RetValue = 0; */
	/* char pBuff[2]; */

	*a_pu2Result = (read_data(0x02) << 8) + (read_data(0x03) & 0xff);

	DW9718AF_SUNNYDB("[DW9718AF_SUNNY]  s4DW9718AF_SUNNY_ReadReg %d\n", *a_pu2Result);
	return 0;
}

static int s4DW9718AF_SUNNY_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = { 0x02, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };

	DW9718AF_SUNNYDB("[DW9718AF_SUNNY]  write %d\n", a_u2Data);

	g_pstDW9718AF_SUNNY_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
	i4RetValue = i2c_master_send(g_pstDW9718AF_SUNNY_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] I2C send failed!!\n");
		return -1;
	}

	return 0;
}

inline static int getDW9718AF_SUNNYInfo(__user stDW9718AF_SUNNY_MotorInfo * pstMotorInfo)
{
	stDW9718AF_SUNNY_MotorInfo stMotorInfo;
	stMotorInfo.u4MacroPosition = g_u4DW9718AF_SUNNY_MACRO;
	stMotorInfo.u4InfPosition = g_u4DW9718AF_SUNNY_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = TRUE;

	if (g_i4MotorStatus == 1) {
		stMotorInfo.bIsMotorMoving = 1;
	} else {
		stMotorInfo.bIsMotorMoving = 0;
	}

	if (g_s4DW9718AF_SUNNY_Opened >= 1) {
		stMotorInfo.bIsMotorOpen = 1;
	} else {
		stMotorInfo.bIsMotorOpen = 0;
	}

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stDW9718AF_SUNNY_MotorInfo))) {
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] copy to user failed when getting motor information\n");
	}

	return 0;
}
static void initdrv()
{
	char puSendCmd2[2] = { 0x01, 0x39 };
	char puSendCmd3[2] = { 0x05, 0x65 };
	i2c_master_send(g_pstDW9718AF_SUNNY_I2Cclient, puSendCmd2, 2);
	i2c_master_send(g_pstDW9718AF_SUNNY_I2Cclient, puSendCmd3, 2);
}

inline static int moveDW9718AF_SUNNY(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4DW9718AF_SUNNY_MACRO) || (a_u4Position < g_u4DW9718AF_SUNNY_INF)) {
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] out of range\n");
		return -EINVAL;
	}

	if (g_s4DW9718AF_SUNNY_Opened == 1) {
		unsigned short InitPos;
		initdrv();
		ret = s4DW9718AF_SUNNY_ReadReg(&InitPos);

		if (ret == 0) {
			DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Init Pos %6d\n", InitPos);

			spin_lock(&g_DW9718AF_SUNNY_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
		} else {
			spin_lock(&g_DW9718AF_SUNNY_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
		}

		spin_lock(&g_DW9718AF_SUNNY_SpinLock);
		g_s4DW9718AF_SUNNY_Opened = 2;
		spin_unlock(&g_DW9718AF_SUNNY_SpinLock);

	}

	if (g_u4CurrPosition < a_u4Position) {
		spin_lock(&g_DW9718AF_SUNNY_SpinLock);
		g_i4Dir = 1;
		spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
	} else if (g_u4CurrPosition > a_u4Position) {
		spin_lock(&g_DW9718AF_SUNNY_SpinLock);
		g_i4Dir = -1;
		spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
	} else {
		return 0;
	}

	spin_lock(&g_DW9718AF_SUNNY_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(&g_DW9718AF_SUNNY_SpinLock);

	/* DW9718AF_SUNNYDB("[DW9718AF_SUNNY] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */

	spin_lock(&g_DW9718AF_SUNNY_SpinLock);
	g_sr = 3;
	g_i4MotorStatus = 0;
	spin_unlock(&g_DW9718AF_SUNNY_SpinLock);

	if (s4DW9718AF_SUNNY_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(&g_DW9718AF_SUNNY_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
	} else {
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] set I2C failed when moving the motor\n");
		spin_lock(&g_DW9718AF_SUNNY_SpinLock);
		g_i4MotorStatus = -1;
		spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
	}

	return 0;
}

inline static int setDW9718AF_SUNNYInf(unsigned long a_u4Position)
{
	spin_lock(&g_DW9718AF_SUNNY_SpinLock);
	g_u4DW9718AF_SUNNY_INF = a_u4Position;
	spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
	return 0;
}

inline static int setDW9718AF_SUNNYMacro(unsigned long a_u4Position)
{
	spin_lock(&g_DW9718AF_SUNNY_SpinLock);
	g_u4DW9718AF_SUNNY_MACRO = a_u4Position;
	spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
static long DW9718AF_SUNNY_Ioctl(struct file *a_pstFile,
			   unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case DW9718AF_SUNNYIOC_G_MOTORINFO:
		i4RetValue = getDW9718AF_SUNNYInfo((__user stDW9718AF_SUNNY_MotorInfo *) (a_u4Param));
		break;

	case DW9718AF_SUNNYIOC_T_MOVETO:
		i4RetValue = moveDW9718AF_SUNNY(a_u4Param);
		break;

	case DW9718AF_SUNNYIOC_T_SETINFPOS:
		i4RetValue = setDW9718AF_SUNNYInf(a_u4Param);
		break;

	case DW9718AF_SUNNYIOC_T_SETMACROPOS:
		i4RetValue = setDW9718AF_SUNNYMacro(a_u4Param);
		break;

	default:
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}


/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
/* 3.Update f_op pointer. */
/* 4.Fill data structures into private_data */
/* CAM_RESET */
static int DW9718AF_SUNNY_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] DW9718AF_SUNNY_Open - Start\n");
	if (g_s4DW9718AF_SUNNY_Opened) {
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] the device is opened\n");
		return -EBUSY;
	}
	spin_lock(&g_DW9718AF_SUNNY_SpinLock);
	g_s4DW9718AF_SUNNY_Opened = 1;
	spin_unlock(&g_DW9718AF_SUNNY_SpinLock);
	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] DW9718AF_SUNNY_Open - End\n");
	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int DW9718AF_SUNNY_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] DW9718AF_SUNNY_Release - Start\n");

    if (g_s4DW9718AF_SUNNY_Opened == 2)
    {
		g_sr = 5;    
    }

	if (g_s4DW9718AF_SUNNY_Opened) {
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] feee\n");

		spin_lock(&g_DW9718AF_SUNNY_SpinLock);
		g_s4DW9718AF_SUNNY_Opened = 0;
		spin_unlock(&g_DW9718AF_SUNNY_SpinLock);

	}
	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] DW9718AF_SUNNY_Release - End\n");

	return 0;
}

static const struct file_operations g_stDW9718AF_SUNNY_fops = {
	.owner = THIS_MODULE,
	.open = DW9718AF_SUNNY_Open,
	.release = DW9718AF_SUNNY_Release,
	.unlocked_ioctl = DW9718AF_SUNNY_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = DW9718AF_SUNNY_Ioctl,
#endif
};

inline static int Register_DW9718AF_SUNNY_CharDrv(void)
{
	struct device *vcm_device = NULL;

	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Register_DW9718AF_SUNNY_CharDrv - Start\n");

	/* Allocate char driver no. */
	if (alloc_chrdev_region(&g_DW9718AF_SUNNY_devno, 0, 1, DW9718AF_SUNNY_DRVNAME)) {
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Allocate device no failed\n");

		return -EAGAIN;
	}
	/* Allocate driver */
	g_pDW9718AF_SUNNY_CharDrv = cdev_alloc();

	if (NULL == g_pDW9718AF_SUNNY_CharDrv) {
		unregister_chrdev_region(g_DW9718AF_SUNNY_devno, 1);

		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(g_pDW9718AF_SUNNY_CharDrv, &g_stDW9718AF_SUNNY_fops);

	g_pDW9718AF_SUNNY_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pDW9718AF_SUNNY_CharDrv, g_DW9718AF_SUNNY_devno, 1)) {
		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Attatch file operation failed\n");

		unregister_chrdev_region(g_DW9718AF_SUNNY_devno, 1);

		return -EAGAIN;
	}

	actuator_class = class_create(THIS_MODULE, "actuatordrvDW9718AF_SUNNY");
	if (IS_ERR(actuator_class)) {
		int ret = PTR_ERR(actuator_class);
		DW9718AF_SUNNYDB("Unable to create class, err = %d\n", ret);
		return ret;
	}

	vcm_device = device_create(actuator_class, NULL, g_DW9718AF_SUNNY_devno, NULL, DW9718AF_SUNNY_DRVNAME);

	if (NULL == vcm_device) {
		return -EIO;
	}

	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Register_DW9718AF_SUNNY_CharDrv - End\n");
	return 0;
}

inline static void Unregister_DW9718AF_SUNNY_CharDrv(void)
{
	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Unregister_DW9718AF_SUNNY_CharDrv - Start\n");

	/* Release char driver */
	cdev_del(g_pDW9718AF_SUNNY_CharDrv);

	unregister_chrdev_region(g_DW9718AF_SUNNY_devno, 1);

	device_destroy(actuator_class, g_DW9718AF_SUNNY_devno);

	class_destroy(actuator_class);

	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Unregister_DW9718AF_SUNNY_CharDrv - End\n");
}

/* //////////////////////////////////////////////////////////////////// */

static int DW9718AF_SUNNY_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int DW9718AF_SUNNY_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id DW9718AF_SUNNY_i2c_id[] = { {DW9718AF_SUNNY_DRVNAME, 0}, {} };

struct i2c_driver DW9718AF_SUNNY_i2c_driver = {
	.probe = DW9718AF_SUNNY_i2c_probe,
	.remove = DW9718AF_SUNNY_i2c_remove,
	.driver.name = DW9718AF_SUNNY_DRVNAME,
	.id_table = DW9718AF_SUNNY_i2c_id,
};

#if 0
static int DW9718AF_SUNNY_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, DW9718AF_SUNNY_DRVNAME);
	return 0;
}
#endif
static int DW9718AF_SUNNY_i2c_remove(struct i2c_client *client)
{
	return 0;
}

/* Kirby: add new-style driver {*/
static int DW9718AF_SUNNY_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] DW9718AF_SUNNY_i2c_probe\n");

	/* Kirby: add new-style driver { */
	g_pstDW9718AF_SUNNY_I2Cclient = client;

	g_pstDW9718AF_SUNNY_I2Cclient->addr = g_pstDW9718AF_SUNNY_I2Cclient->addr >> 1;

	/* Register char driver */
	i4RetValue = Register_DW9718AF_SUNNY_CharDrv();

	if (i4RetValue) {

		DW9718AF_SUNNYDB("[DW9718AF_SUNNY] register char device failed!\n");

		return i4RetValue;
	}

	spin_lock_init(&g_DW9718AF_SUNNY_SpinLock);

	DW9718AF_SUNNYDB("[DW9718AF_SUNNY] Attached!!\n");

	return 0;
}

static int DW9718AF_SUNNY_probe(struct platform_device *pdev)
{
	return i2c_add_driver(&DW9718AF_SUNNY_i2c_driver);
}

static int DW9718AF_SUNNY_remove(struct platform_device *pdev)
{
	i2c_del_driver(&DW9718AF_SUNNY_i2c_driver);
	return 0;
}

static int DW9718AF_SUNNY_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int DW9718AF_SUNNY_resume(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver g_stDW9718AF_SUNNY_Driver = {
	.probe = DW9718AF_SUNNY_probe,
	.remove = DW9718AF_SUNNY_remove,
	.suspend = DW9718AF_SUNNY_suspend,
	.resume = DW9718AF_SUNNY_resume,
	.driver = {
		   .name = PLATFORM_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   }
};
static struct platform_device g_stDW9718AF_SUNNY_device = {
    .name = PLATFORM_DRIVER_NAME,
    .id = 0,
    .dev = {}
};
static int __init DW9718AF_SUNNY_i2C_init(void)
{
	i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
  if(platform_device_register(&g_stDW9718AF_SUNNY_device)){
    DW9718AF_SUNNYDB("failed to register AF driver\n");
    return -ENODEV;
  }
	if (platform_driver_register(&g_stDW9718AF_SUNNY_Driver)) {
		DW9718AF_SUNNYDB("failed to register DW9718AF_SUNNY driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit DW9718AF_SUNNY_i2C_exit(void)
{
	platform_driver_unregister(&g_stDW9718AF_SUNNY_Driver);
}
module_init(DW9718AF_SUNNY_i2C_init);
module_exit(DW9718AF_SUNNY_i2C_exit);

MODULE_DESCRIPTION("DW9718AF_SUNNY lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");
