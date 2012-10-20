	/*
	* drivers/media/video/msm/mt9m114.c
	*
	* Refer to drivers/media/video/msm/mt9d112.c
	* For MT9M114: 2.0Mp, 1/5-Inch System-On-A-Chip (SOC) CMOS Digital Image Sensor
	*
	* Copyright (C) 2009-2010 ZTE Corporation.
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
	* Created by jia.jia@zte.com.cn
	*/

	/*-----------------------------------------------------------------------------------------
	when         who          what, where, why                         comment tag
	--------     ----         -------------------------------------    ----------------------
	2011-06-24   wangtao      add sensor for 727d40                    ZTE_CAM_WT_20110706 
	2011-06-24   lijing       optimize adaptor flow                    ZTE_CAM_LJ_20110624 
	2010-12-09   jia          add failure process to avoid standby     ZTE_JIA_CAM_20101209
	                    current exception problem
	2010-12-06   jia          add support for exposure compensation    ZTE_CAM_JIA_20101206
	2010-09-08   jia          add exception process of i2c_del_driver  ZTE_JIA_CAM_20100908
	2010-08-04   li.jing      update ISO settings                      ZTE_LJ_CAM_20100804
	2010-07-23   li.jing      update sensor settings                   ZTE_LJ_CAM_20100723
	2010-07-05   li.jing      update sensor reg settings               ZTE_CAM_LIJING_20100705
	                    set MCLK and MT9M114_MODEL_ID
	2010-06-29   li.jing      created                                  ZTE_CAMERA_LIJING_20100629
	------------------------------------------------------------------------------------------*/

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include "mt9m114.h"
#include <linux/slab.h>

	/*-----------------------------------------------------------------------------------------
	*
	* MACRO DEFINITION
	*
	*----------------------------------------------------------------------------------------*/
	/*
	* For Sensor Init During System Startup
	* otherwise, sensor is initialized during
	* camera app startup
	*/

	static int MT9D114_CSI_CONFIG = 0;
#define MT9M114_PROBE_WORKQUEUE
#include <linux/workqueue.h>
	static struct platform_device *pdev_wq = NULL;
	static struct workqueue_struct *mt9m114_wq = NULL;
	static void mt9m114_workqueue(struct work_struct *work);
	static DECLARE_WORK(mt9m114_cb_work, mt9m114_workqueue);

#define ENOINIT 100 /*have not power up,so don't need to power down*/

	/*
	* CAMIO Input MCLK (MHz)
	*
	* MCLK: 6-54 MHz
	*
	* maximum frame rate: 
	* 15 fps at full resolution (JPEG),
	* 30 fps in preview mode
	*
	* when MCLK=40MHZ, PCLK=48MHZ (PLL is enabled by sensor)
	* when MCLK=48MHZ, PCLK=48MHZ (PLL is disabled by sensor)
	*
	* 54MHz is the maximum value accepted by sensor
	*/
#define MT9M114_CAMIO_MCLK  24000000

	/*
	* Micron MT9M114 Registers and their values
	*/
	/* Sensor I2C Board Name */
#define MT9M114_I2C_BOARD_NAME "mt9m114"

	/* Sensor I2C Bus Number (Master I2C Controller: 0) */
#define MT9M114_I2C_BUS_ID  (0)

	/* Sensor I2C Slave Address */
#define MT9M114_SLAVE_WR_ADDR 0x78 /* replaced by "msm_i2c_devices.addr" */
#define MT9M114_SLAVE_RD_ADDR 0x79 /* replaced by "msm_i2c_devices.addr" */

	/* Sensor I2C Device ID */
#define REG_MT9M114_MODEL_ID    0x0000
#define MT9M114_MODEL_ID        0x2481

	/* Sensor I2C Device Sub ID */
#define REG_MT9M114_MODEL_ID_SUB    0x31FE
#define MT9M114_MODEL_ID_SUB        0x0011
	/* SOC Registers */
#define REG_MT9M114_STANDBY_CONTROL     0x0018

	/*
	* GPIO For Sensor Clock Switch
	*/
#if defined(CONFIG_MACH_R750) || defined(CONFIG_MACH_JOE)
#define MT9M114_GPIO_SWITCH_CTL     39
#define MT9M114_GPIO_SWITCH_VAL     0
#elif defined(CONFIG_MACH_V9)
#define MT9M114_GPIO_SWITCH_CTL     107
#define MT9M114_GPIO_SWITCH_VAL     0
#elif defined(CONFIG_MACH_SAILBOAT)
#define MT9M114_GPIO_SWITCH_CTL     30
#define MT9M114_GPIO_SWITCH_VAL     0
#else
#undef MT9M114_GPIO_SWITCH_CTL
#undef MT9M114_GPIO_SWITCH_VAL
#endif

	/* 
	* GPIO For Lowest-Power mode (SHUTDOWN mode)
	* #define MT9M114_GPIO_SHUTDOWN_CTL   Dummy
	*/

	/*-----------------------------------------------------------------------------------------
	*
	* TYPE DECLARATION
	*
	*----------------------------------------------------------------------------------------*/
	struct mt9m114_work_t {
	struct work_struct work;
	};

	struct mt9m114_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
	};

	/*-----------------------------------------------------------------------------------------
	*
	* GLOBAL VARIABLE DEFINITION
	*
	*----------------------------------------------------------------------------------------*/
	static struct mt9m114_work_t *mt9m114_sensorw = NULL;
	static struct i2c_client *mt9m114_client = NULL;
	static struct mt9m114_ctrl_t *mt9m114_ctrl = NULL;

	/*
	* For coexistence of MT9T111, MT9T112 and MT9M114
	*/
	static uint16_t model_id;
	/* ZTE_ZT_CAM_20110107
	* Set the global value of exposure & brightness 
	*/
	//static int8_t current_brightness = CAMERA_BRIGHTNESS_3;
	//static int8_t current_exposure = CAMERA_EXPOSURE_2;

	//DECLARE_MUTEX(mt9m114_sem);

	//static struct wake_lock mt9m114_wake_lock;

	/*-----------------------------------------------------------------------------------------
	*
	* FUNCTION DECLARATION
	*
	*----------------------------------------------------------------------------------------*/
	static int mt9m114_sensor_init(const struct msm_camera_sensor_info *data);
	static int mt9m114_sensor_config(void __user *argp);
	static int mt9m114_sensor_release(void);
	//static int mt9m114_sensor_release_internal(void);
	static int32_t mt9m114_i2c_add_driver(void);
	static void mt9m114_i2c_del_driver(void);

	extern int32_t msm_camera_power_backend(enum msm_camera_pwr_mode_t pwr_mode);
	extern int msm_camera_clk_switch(const struct msm_camera_sensor_info *data,
	                                 uint32_t gpio_switch,
	                                 uint32_t switch_val);

	/*
	* Get FTM flag to adjust 
	* the initialize process 
	* of camera
	*/
#ifdef CONFIG_ZTE_PLATFORM
#ifdef CONFIG_ZTE_FTM_FLAG_SUPPORT
	extern int zte_get_ftm_flag(void);
#endif
#endif

	/*-----------------------------------------------------------------------------------------
	*
	* FUNCTION DEFINITION
	*
	*----------------------------------------------------------------------------------------*/

	/*
	* Hard reset: RESET_BAR pin (active LOW)
	* Hard reset has the same effect as the soft reset.
	*/

	static int mt9m114_hard_reset(const struct msm_camera_sensor_info *dev)
	{
	int rc = 0;

	pr_err("%s: entry\n", __func__);

	rc = gpio_request(dev->sensor_reset, "mt9m114");
	if (0 == rc)
	{
	/* ignore "rc" */
	rc = gpio_direction_output(dev->sensor_reset, 1);  
	mdelay(10);
	/* ignore "rc" */
	rc = gpio_direction_output(dev->sensor_reset, 0);
	/*
	  * RESET_BAR pulse width: Min 70 EXTCLKs
	  * EXTCLKs: = MCLK (i.e., MT9M114_CAMIO_MCLK)
	  */  
	mdelay(10);
	/* ignore "rc" */
	rc = gpio_direction_output(dev->sensor_reset, 1);

	/*
	  * Time delay before first serial write: Min 100 EXTCLKs
	  * EXTCLKs: = MCLK (i.e., MT9M114_CAMIO_MCLK)
	  */
	msleep(20);
	}

	gpio_free(dev->sensor_reset);

	return rc;
	}

	static int32_t mt9m114_i2c_txdata(unsigned short saddr,
	                               unsigned char *txdata,
	                               int length)
	{
	struct i2c_msg msg[] = {
	{
	    .addr  = saddr,
	    .flags = 0,
	    .len   = length,
	    .buf   = txdata,
	},
	};

	if (i2c_transfer(mt9m114_client->adapter, msg, 1) < 0)
	{
	pr_err("%s: failed!\n", __func__);
	return -EIO;
	}

	return 0;
	}

	static int32_t mt9m114_i2c_write(unsigned short saddr,
	                              unsigned short waddr,
	                              unsigned short wdata,
	                              enum mt9m114_width_t width)
	{
	int32_t rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));

	switch (width)
	{
	case WORD_LEN:
	{
	    buf[0] = (waddr & 0xFF00) >> 8;
	    buf[1] = (waddr & 0x00FF);
	    buf[2] = (wdata & 0xFF00) >> 8;
	    buf[3] = (wdata & 0x00FF);

	    rc = mt9m114_i2c_txdata(saddr, buf, 4);
	}
	break;

	case BYTE_LEN:
	{
	    buf[0] = (waddr & 0xFF00) >> 8;
	    buf[1] = (waddr & 0x00FF);
	    buf[2] = wdata;

	    rc = mt9m114_i2c_txdata(saddr, buf, 3);
	}
	break;

	default:
	{
	    rc = -EFAULT;
	}
	break;
	}

	if (rc < 0)
	{
	pr_err("%s: waddr = 0x%x, wdata = 0x%x, failed!\n", __func__, waddr, wdata);
	}

	return rc;
	}

	static int32_t mt9m114_i2c_write_table(struct mt9m114_i2c_reg_conf const *reg_conf_tbl,
	                                     int len)
	{
	uint32_t i;
	int32_t rc = 0;

	for (i = 0; i < len; i++)
	{
	rc = mt9m114_i2c_write(mt9m114_client->addr,
	                       reg_conf_tbl[i].waddr,
	                       reg_conf_tbl[i].wdata,
	                       reg_conf_tbl[i].width);
	if (rc < 0)
	{
	    break;
	}

	if (reg_conf_tbl[i].mdelay_time != 0)
	{
	    msleep(reg_conf_tbl[i].mdelay_time);
	}

	/*
	  * To fix the bug of preview failure
	  * time delay of 1ms is recommended after writing per 16 items (0x10)
	  */
	//if (0x00 == (!(i | 0xFFFFFFE0) && 0x0F))
	//{
	   // mdelay(1);
	//}
	}
	return rc;
	}

	static int mt9m114_i2c_rxdata(unsigned short saddr,
	                           unsigned char *rxdata,
	                           int length)
	{
	struct i2c_msg msgs[] = {
	{
	    .addr  = saddr,
	    .flags = 0,
	    .len   = 2,
	    .buf   = rxdata,
	},
	{
	    .addr  = saddr,
	    .flags = I2C_M_RD,
	    .len   = length,
	    .buf   = rxdata,
	},
	};

	if (i2c_transfer(mt9m114_client->adapter, msgs, 2) < 0)
	{
	pr_err("%s: failed!\n", __func__);
	return -EIO;
	}

	return 0;
	}

	static int32_t mt9m114_i2c_read(unsigned short saddr,
	                             unsigned short raddr,
	                             unsigned short *rdata,
	                             enum mt9m114_width_t width)
	{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
	{
	pr_err("%s: rdata points to NULL!\n", __func__);
	return -EIO;
	}

	memset(buf, 0, sizeof(buf));

	switch (width)
	{
	case WORD_LEN:
	{
	    buf[0] = (raddr & 0xFF00) >> 8;
	    buf[1] = (raddr & 0x00FF);

	    rc = mt9m114_i2c_rxdata(saddr, buf, 2);
	    if (rc < 0)
	    {
	        return rc;
	    }

	    *rdata = buf[0] << 8 | buf[1];
	}
	break;

	case BYTE_LEN:
	{
	    buf[0] = (raddr & 0xFF00) >> 8;
	    buf[1] = (raddr & 0x00FF);

	    rc = mt9m114_i2c_rxdata(saddr, buf, 2);
	    if (rc < 0)
	    {
	        return rc;
	    }

	    *rdata = buf[0] ;
	}
	break;
	default:
	{
	    rc = -EFAULT;
	}
	break;
	}

	if (rc < 0)
	{
	pr_err("%s: failed!\n", __func__);
	}

	return rc;
	}

	/*
	* Auto Focus Trigger (not supported)
	*/
	static int32_t __attribute__((unused)) mt9m114_af_trigger(void)
	{
	pr_err("%s: not supported!\n", __func__);
	return 0;
	}

	/* ZTE_ZT_CAM_20110107
	* Adjust the configurations according to the value of expusure & brightness
	*/
#if 0
	static int32_t mt9m114_set_exposure_brightness(int8_t exposure, int8_t brightness)
	{
	int32_t rc = 0;

	pr_err("%s: entry: exposure=%d, brightness=%d\n", __func__, exposure, brightness);

	rc = mt9m114_i2c_write_table(mt9m114_regs.brightness_exposure_tbl[exposure*CAMERA_BRIGHTNESS_MAX+brightness],
	                                 mt9m114_regs.brightness_exposure_tbl_sz[exposure*CAMERA_BRIGHTNESS_MAX+brightness]); 

	return rc;
	}
#endif
	/*
	* White Balance Setting
	*/
	static int32_t mt9m114_set_wb(int8_t wb_mode)
	{
	int32_t rc = 0;

	pr_err("%s: entry: wb_mode=%d\n", __func__, wb_mode);
#if 0
	switch (wb_mode)
	{
	case CAMERA_WB_MODE_AWB:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.wb_auto_tbl, 
	                                 mt9m114_regs.wb_auto_tbl_sz);
	}
	break;

	case CAMERA_WB_MODE_SUNLIGHT:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.wb_daylight_tbl,
	                                 mt9m114_regs.wb_daylight_tbl_sz);
	}
	break;

	case CAMERA_WB_MODE_INCANDESCENT:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.wb_incandescent_tbl,
	                                 mt9m114_regs.wb_incandescent_tbl_sz);
	}
	break;

	case CAMERA_WB_MODE_FLUORESCENT:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.wb_flourescant_tbl,
	                                 mt9m114_regs.wb_flourescant_tbl_sz);
	}
	break; 

	case CAMERA_WB_MODE_CLOUDY:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.wb_cloudy_tbl,
	                                 mt9m114_regs.wb_cloudy_tbl_sz);
	}
	break;

	default:
	{
	    pr_err("%s: parameter error!\n", __func__);
	    rc = -EFAULT;
	}     
	}

	/*
	* Attention
	*
	* Time delay of 100ms or more is required by sensor,
	*
	* WB config will have no effect after setting 
	* without time delay of 100ms or more
	*/
	mdelay(100);
#endif
	return rc;
	}    

	static int32_t mt9m114_set_contrast(int8_t contrast)
	{
	int32_t rc = 0;

	pr_err("%s: entry: contrast=%d\n", __func__, contrast);
#if 0
	switch (contrast)
	{
	case CAMERA_CONTRAST_0:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.contrast_tbl[0],
	                                 mt9m114_regs.contrast_tbl_sz[0]);
	}
	break;

	case CAMERA_CONTRAST_1:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.contrast_tbl[1],
	                                 mt9m114_regs.contrast_tbl_sz[1]);
	}
	break;

	case CAMERA_CONTRAST_2:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.contrast_tbl[2],
	                                 mt9m114_regs.contrast_tbl_sz[2]);
	}
	break;

	case CAMERA_CONTRAST_3:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.contrast_tbl[3],
	                                 mt9m114_regs.contrast_tbl_sz[3]);
	}
	break; 

	case CAMERA_CONTRAST_4:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.contrast_tbl[4],
	                                 mt9m114_regs.contrast_tbl_sz[4]);
	}
	break;

	default:
	{
	    pr_err("%s: parameter error!\n", __func__);
	    rc = -EFAULT;
	}     
	}

	/*
	* Attention
	*
	* Time delay of 100ms or more is required by sensor,
	*
	* Contrast config will have no effect after setting 
	* without time delay of 100ms or more
	*/
	mdelay(100);
#endif
	return rc;
	}

	static int32_t mt9m114_set_brightness(int8_t brightness)
	{

	int32_t rc = 0;
	//unsigned short brightness_lev = 0;	
	pr_err("%s: entry: brightness=%d\n", __func__, brightness);	

				
	switch (brightness)
	{
	case CAMERA_BRIGHTNESS_0:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.brightness_tbl[0],
	                                 mt9m114_regs.brightness_tbl_sz[0] );				
	  //rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC870, &brightness_lev, BYTE_LEN);	
	 //pr_err("yanwei brightness_0 %s: entry: 0xC870=%x\n", __func__, brightness_lev);
	}
	break;

	case CAMERA_BRIGHTNESS_1:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.brightness_tbl[1],
	                                 mt9m114_regs.brightness_tbl_sz[1] );		
	 // rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC870, &brightness_lev, BYTE_LEN);	
	// pr_err("yanwei brightness_1 %s: entry: 0xC870=%x\n", __func__, brightness_lev);		
	}
	break;

	case CAMERA_BRIGHTNESS_2:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.brightness_tbl[2],
	                                 mt9m114_regs.brightness_tbl_sz[2] );		
	 // rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC870, &brightness_lev, BYTE_LEN);	
	//pr_err("yanwei brightness_2%s: entry: 0xC870=%x\n", __func__, brightness_lev);			
	}
	break;

	case CAMERA_BRIGHTNESS_3:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.brightness_tbl[3],
	                                 mt9m114_regs.brightness_tbl_sz[3] );			
	  // rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC870, &brightness_lev, BYTE_LEN);	
	//pr_err("yanwei brightness_3%s: entry: 0xC870=%x\n", __func__, brightness_lev);                                         
	}
	break; 

	case CAMERA_BRIGHTNESS_4:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.brightness_tbl[4],
	                                 mt9m114_regs.brightness_tbl_sz[4] );		
	  //rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC870, &brightness_lev, BYTE_LEN);	
	//pr_err("yanwei brightness_4%s: entry: 0xC870=%x\n", __func__, brightness_lev);				
	}
	break;

	case CAMERA_BRIGHTNESS_5:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.brightness_tbl[5],
	                                 mt9m114_regs.brightness_tbl_sz[5] );	
	 // rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC870, &brightness_lev, BYTE_LEN);	
	//pr_err("yanwei brightness_5 %s: entry: 0xC870=%x\n", __func__, brightness_lev);
	}
	break; 

	case CAMERA_BRIGHTNESS_6:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.brightness_tbl[6],
	                                 mt9m114_regs.brightness_tbl_sz[6] );		
	 // rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC870, &brightness_lev, BYTE_LEN);	
	//pr_err("yanwei brightness_6 %s: entry: 0xC870=%x\n", __func__, brightness_lev);				
	}
	break;

	default:
	{
	    pr_err("yanwei %s: parameter error!\n", __func__);
	    rc = -EFAULT;
	}     
	}
	return rc;
	}   
	static int32_t mt9m114_set_saturation(int8_t saturation)
	{
	int32_t rc = 0;

	pr_err("%s: entry: saturation=%d\n", __func__, saturation);
#if 0
	switch (saturation)
	{
	case CAMERA_SATURATION_0:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.saturation_tbl[0],
	                                 mt9m114_regs.saturation_tbl_sz[0]);
	}
	break;

	case CAMERA_SATURATION_1:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.saturation_tbl[1],
	                                 mt9m114_regs.saturation_tbl_sz[1]);
	}
	break;

	case CAMERA_SATURATION_2:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.saturation_tbl[2],
	                                 mt9m114_regs.saturation_tbl_sz[2]);
	}
	break;

	case CAMERA_SATURATION_3:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.saturation_tbl[3],
	                                 mt9m114_regs.saturation_tbl_sz[3]);
	}
	break; 

	case CAMERA_SATURATION_4:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.saturation_tbl[4],
	                                 mt9m114_regs.saturation_tbl_sz[4]);
	}
	break;

	default:
	{
	    pr_err("%s: parameter error!\n", __func__);
	    rc = -EFAULT;
	}     
	}

	/*
	* Attention
	*
	* Time delay of 100ms or more is required by sensor,
	*
	* Saturation config will have no effect after setting 
	* without time delay of 100ms or more
	*/
	mdelay(100);
#endif
	return rc;
	}    

	static int32_t mt9m114_set_sharpness(int8_t sharpness)
	{
	int32_t rc = 0;

	pr_err("%s: entry: sharpness=%d\n", __func__, sharpness);
#if 0
	switch (sharpness)
	{
	case CAMERA_SHARPNESS_0:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.sharpness_tbl[0],
	                                 mt9m114_regs.sharpness_tbl_sz[0]);
	}
	break;

	case CAMERA_SHARPNESS_1:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.sharpness_tbl[1],
	                                 mt9m114_regs.sharpness_tbl_sz[1]);
	}
	break;

	case CAMERA_SHARPNESS_2:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.sharpness_tbl[2],
	                                 mt9m114_regs.sharpness_tbl_sz[2]);
	}
	break;

	case CAMERA_SHARPNESS_3:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.sharpness_tbl[3],
	                                 mt9m114_regs.sharpness_tbl_sz[3]);
	}
	break; 

	case CAMERA_SHARPNESS_4:
	{
	    rc = mt9m114_i2c_write_table(mt9m114_regs.sharpness_tbl[4],
	                                 mt9m114_regs.sharpness_tbl_sz[4]);
	}
	break;        

	default:
	{
	    pr_err("%s: parameter error!\n", __func__);
	    rc = -EFAULT;
	}     
	}
#endif
	return rc;
	}    

	/*
	* ISO Setting
	*/
	static int32_t mt9m114_set_iso(int8_t iso_val)
	{
	int32_t rc = 0;
       unsigned short ISO_lev;
	pr_err("%s: entry: iso_val=%d\n", __func__, iso_val);

	switch (iso_val)
	{
	case CAMERA_ISO_SET_AUTO:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E,  0x4884, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC884, 0x0020, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC886, 0x0100, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("ISO_auto %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	}
	break;

	case CAMERA_ISO_SET_100:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E,  0x4884, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC884, 0x0020, WORD_LEN);
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("yanwei ISO_100 %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC886, 0x00F0, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("ISO_100 %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	}
	case CAMERA_ISO_SET_HJR:
	{
	     pr_err("%s: not supported!\n", __func__);
	     rc = -EFAULT;
	}
	break;

	case CAMERA_ISO_SET_200:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E,  0x4884, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC884, 0x0028, WORD_LEN);
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("yanwei ISO_200 %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC886, 0x00F0, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }  
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("yanwei ISO_200 %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	}
	break;

	case CAMERA_ISO_SET_400:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E,  0x4884, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC884, 0x0030, WORD_LEN);
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("yanwei ISO_400 %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC886, 0x00F0, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("yanwei ISO_400 %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	}
	break;

	case CAMERA_ISO_SET_800:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E,  0x4884, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC884, 0x0038, WORD_LEN);
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("yanwei ISO_800 %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC886, 0x00F0, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_read(mt9m114_client->addr, 0xC884, &ISO_lev, WORD_LEN);	
	    pr_err("yanwei ISO_800 %s: entry: 0xC884=%x\n", __func__, ISO_lev);			
	}
	break;
	default:
	{
	    pr_err("%s: parameter error!\n", __func__);
	    rc = -EFAULT;
	}     
	}

	/*
	* Attention
	*
	* Time delay of 100ms or more is required by sensor,
	*
	* ISO config will have no effect after setting 
	* without time delay of 100ms or more
	*/
	msleep(10);
	return rc;
	} 

	/*
	* Antibanding Setting
	*/
	static int32_t  mt9m114_set_antibanding(int8_t antibanding)
	{
	int32_t rc = 0;

	pr_err("%s: entry: antibanding=%d\n", __func__, antibanding);
#if 0
	switch (antibanding)
	{
	case CAMERA_ANTIBANDING_SET_OFF:
	{
	    pr_err("%s: CAMERA_ANTIBANDING_SET_OFF NOT supported!\n", __func__);
	}
	break;

	case CAMERA_ANTIBANDING_SET_60HZ:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA118, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0002, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA11E, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0002, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA124, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0002, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA12A, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0002, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA404, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }  
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x00A0, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA103, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0005, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	}            
	break;
	case CAMERA_ANTIBANDING_SET_50HZ:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA118, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0002, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA11E, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0002, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA124, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0002, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }    
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA12A, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0002, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }    
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA404, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x00E0, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA103, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0005, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }  
	}
	break;

	case CAMERA_ANTIBANDING_SET_AUTO:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA118, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0001, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA11E, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0001, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    } 
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA124, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0000, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }    
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA12A, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0001, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }  
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098C, 0xA103, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0990, 0x0005, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }  
	}
	break;

	default:
	{
	    pr_err("%s: parameter error!\n", __func__);
	    rc = -EFAULT;
	}     
	}

	/*
	* Attention
	*
	* Time delay of 100ms or more is required by sensor,
	*
	* Antibanding config will have no effect after setting 
	* without time delay of 100ms or more
	*/
	mdelay(100);
#endif
	return rc;
	} 

	/*
	* Len Shading Setting
	*/
	static int32_t __attribute__((unused))mt9m114_set_lensshading(int8_t lensshading)
	{
#if 0
	int32_t rc = 0;
	uint16_t brightness_lev = 0;

	pr_err("%s: entry: lensshading=%d\n", __func__, lensshading);

	if (0 == lensshading)
	{
	pr_err("%s: lens shading is disabled!\n", __func__);
	return rc;
	}

	/*
	* Set lens shading value according to brightness level
	*/
	rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E, 0x3835, WORD_LEN);
	if (rc < 0)
	{
	return rc;
	}

	rc = mt9m114_i2c_read(mt9m114_client->addr, 0x0990, &brightness_lev, WORD_LEN);
	if (rc < 0)
	{
	return rc;
	}

	if (brightness_lev < 5)
	{
	rc = mt9m114_i2c_write_table(mt9m114_regs.lens_for_outdoor_tbl, mt9m114_regs.lens_for_outdoor_tbl_sz);
	if (rc < 0)
	{
	    return rc;
	} 
	}
	else
	{
	rc = mt9m114_i2c_write_table(mt9m114_regs.lens_for_indoor_tbl, mt9m114_regs.lens_for_indoor_tbl_sz);
	if (rc < 0)
	{
	    return rc;
	} 
	}

	return rc;
#else
	return 0;
#endif
	}

	static long mt9m114_set_exposure_compensation(int8_t exposure)
	{
	long rc = 0;

	pr_err("%s: entry: exposure=%d\n", __func__, exposure);
#if 0
	current_exposure = exposure;

	rc = (int32_t)mt9m114_set_exposure_brightness(current_exposure, 
	current_brightness);
#endif
	return rc;
	}

	static long mt9m114_reg_init(void)
	{
	long rc;

	pr_err("yanwei%s: entry\n", __func__);

	rc = mt9m114_i2c_write_table(mt9m114_regs.prevsnap_tbl, mt9m114_regs.prevsnap_tbl_sz);
	if (rc < 0)
	{
	return rc;
	}

	return 0;
	}

	static long mt9m114_set_sensor_mode(int32_t mode)
	{
	long rc = 0;
	unsigned short status;
	struct msm_camera_csi_params mt9m114_csi_params;
	pr_err("yanwei %s:entry\n", __func__);	

	switch (mode)
	{
	case SENSOR_PREVIEW_MODE:
	{
	pr_err("yanwei %s: mt9m114_csi_params entry\n", __func__);			
	if(!MT9D114_CSI_CONFIG) {
	rc = mt9m114_i2c_write_table(mt9m114_regs.pll_tbl, mt9m114_regs.pll_tbl_sz);
	if (rc < 0)
	{
	return rc;
	}
	mt9m114_csi_params.data_format = CSI_8BIT;
	mt9m114_csi_params.lane_cnt = 1;
	mt9m114_csi_params.lane_assign = 0xe4;
	mt9m114_csi_params.dpcm_scheme = 0;
	mt9m114_csi_params.settle_cnt = 0x7;

	rc = msm_camio_csi_config(&mt9m114_csi_params);		
	MT9D114_CSI_CONFIG = 1;
	mdelay(10);
	pr_err("yanwei %s: msm_camio_csi_config_over\n", __func__);			
	}

	rc = mt9m114_reg_init();
	if (rc < 0)
	{
	pr_err("mt9m114_reg_init failed!\n");
	}
	mdelay(10);	
	rc = mt9m114_i2c_read(mt9m114_client->addr, 0x3c42, &status, WORD_LEN);	
	pr_err("yanwei mt9m114: 0x3c42=%x\n", status);	


	}
	break;

	case SENSOR_SNAPSHOT_MODE:
	break;

	default:
	{
	    return -EFAULT;
	}
	}
		pr_err("yanwei %s exit\n", __func__);
	return 0;
	}

	static long mt9m114_set_effect(int32_t mode, int32_t effect)
	{
	// uint16_t __attribute__((unused)) reg_addr;
	// uint16_t __attribute__((unused)) reg_val;
	long rc = 0;

	switch (mode)
	{
	case SENSOR_PREVIEW_MODE:
	{
	    /* Context A Special Effects */
	    /* add code here
	         e.g.
	         reg_addr = 0xXXXX;
	       */
	}
	break;

	case SENSOR_SNAPSHOT_MODE:
	{
	    /* Context B Special Effects */
	    /* add code here
	         e.g.
	         reg_addr = 0xXXXX;
	       */
	}
	break;

	default:
	{
	    /* add code here
	         e.g.
	         reg_addr = 0xXXXX;
	       */
	}
	break;
	}

	switch (effect)
	{
	case CAMERA_EFFECT_OFF:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E, 0xC874, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC874, 0x00, BYTE_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xDC00, 0x28, BYTE_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0080, 0x8004, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }	
	mdelay(10);	
	}            
	break;

	case CAMERA_EFFECT_MONO:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E, 0xC874, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC874, 0x01, BYTE_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xDC00, 0x28, BYTE_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0080, 0x8004, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }    
	mdelay(10);			
	}
	break;

	case CAMERA_EFFECT_NEGATIVE:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E, 0xC874, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC874, 0x03, BYTE_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xDC00, 0x28, BYTE_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0080, 0x8004, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }   
	mdelay(10);			
	}
	break;

	case CAMERA_EFFECT_SOLARIZE:
	{
	}            
	break;

	case CAMERA_EFFECT_SEPIA:
	{
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x098E, 0xC874, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xC874, 0x02, BYTE_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0xDC00, 0x28, BYTE_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }
	    rc = mt9m114_i2c_write(mt9m114_client->addr, 0x0080, 0x8004, WORD_LEN);
	    if (rc < 0)
	    {
	        return rc;
	    }   
	mdelay(10);			
	}         
	break;

	default:
	{
	    /* add code here
	         e.g.
	         reg_val = 0xXXXX;

	         rc = mt9m114_i2c_write(mt9m114_client->addr, 0xXXXX, reg_addr, WORD_LEN);
	         if (rc < 0)
	         {
	            return rc;
	         }
	       */

	    return -EFAULT;
	}
	}

	/*
	* Attention
	*
	* Time delay of 100ms or more is required by sensor,
	*
	* Effect config will have no effect after setting 
	* without time delay of 100ms or more
	*/
	//mdelay(100);

	return rc;
	}

	/*
	* Power-up Process
	*/
	static long mt9m114_power_up(void)
	{
	pr_err("%s: not supported!\n", __func__);
	return 0;
	}

	/*
	* Power-down Process
	*/
	static long mt9m114_power_down(void)
	{
	pr_err("%s: not supported!\n", __func__);
	return 0;
	}

	/*
	* Set lowest-power mode (SHUTDOWN mode)
	*
	* MT9M114_GPIO_SHUTDOWN_CTL: 0, to quit lowest-power mode, or
	*                            1, to enter lowest-power mode
	*/
#if 0 /* Dummy */
	static int mt9m114_power_shutdown(uint32_t on)
	{
	int rc;

	pr_err("%s: entry\n", __func__);

	rc = gpio_request(MT9M114_GPIO_SHUTDOWN_CTL, "mt9m114");
	if (0 == rc)
	{
	/* ignore "rc" */
	rc = gpio_direction_output(MT9M114_GPIO_SHUTDOWN_CTL, on);

	/* time delay */
	mdelay(1);
	}

	gpio_free(MT9M114_GPIO_SHUTDOWN_CTL);

	return rc;
	}
#endif

	static int mt9m114_sensor_init(const struct msm_camera_sensor_info *data)
	{
	int rc;
	pr_err("yanwei%s: entry\n", __func__);

	if (!data || strcmp(data->sensor_name, "mt9m114"))
	{
	pr_err("%s: invalid parameters!\n", __func__);
	rc = -ENODEV;
	goto probe_init_fail;
	}
	mt9m114_ctrl = kzalloc(sizeof(struct mt9m114_ctrl_t), GFP_KERNEL);
	if (!mt9m114_ctrl)
	{
	pr_err("%s: kzalloc failed!\n", __func__);
	rc = -ENOMEM;
	goto probe_init_fail;
	}
	msm_camio_clk_rate_set(MT9M114_CAMIO_MCLK);
	mdelay(10);
	rc = mt9m114_hard_reset(data);
	if (rc < 0)
	{
	pr_err("hard reset failed!\n");

	}

       return 0;
	probe_init_fail:
	/*
	* To shut sensor down
	* Ignore "rc"
	*/

	if(mt9m114_ctrl)
	{
	kfree(mt9m114_ctrl);
	}
	return rc;
	}


	static int mt9m114_sensor_config(void __user *argp)
	{
	struct sensor_cfg_data cfg_data;
	long rc = 0;

	pr_err("%s: entry\n", __func__);

	if (copy_from_user(&cfg_data, (void *)argp, sizeof(struct sensor_cfg_data)))
	{
	pr_err("%s: copy_from_user failed!\n", __func__);
	return -EFAULT;
	}

	/* down(&mt9m114_sem); */

	pr_err("%s: cfgtype = %d, mode = %d\n", __func__, cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype)
	{

	case CFG_SET_MODE:
	{
	    rc = mt9m114_set_sensor_mode(cfg_data.mode);
	}
	break;

	case CFG_SET_EFFECT:
	{
	    rc = mt9m114_set_effect(cfg_data.mode, cfg_data.cfg.effect);
	}
	break;

	case CFG_PWR_UP:
	{
	    rc = mt9m114_power_up();
	}
	break;

	case CFG_PWR_DOWN:
	{
	    rc = mt9m114_power_down();
	}
	break;

	case CFG_SET_WB:
	{
	    rc = mt9m114_set_wb(cfg_data.cfg.wb_mode);
	}
	break;

	case CFG_SET_AF:
	{
	    /*
	       * Not supported by FF module
	       * ignore "rc"
	       */
	    //rc = mt9m114_set_lensshading(1);
	    //rc = mt9m114_af_trigger();
	    rc = 0;
	}
	break;

	case CFG_SET_ISO:
	{
	    rc = mt9m114_set_iso(cfg_data.cfg.iso_val);
	}
	break;

	case CFG_SET_ANTIBANDING:
	{
	    rc = mt9m114_set_antibanding(cfg_data.cfg.antibanding);
	}
	break;

	case CFG_SET_BRIGHTNESS:
	{
	    rc = mt9m114_set_brightness(cfg_data.cfg.brightness);
	}
	break;

	case CFG_SET_SATURATION:
	{
	    rc = mt9m114_set_saturation(cfg_data.cfg.saturation);
	}
	break;

	case CFG_SET_CONTRAST:
	{
	    rc = mt9m114_set_contrast(cfg_data.cfg.contrast);
	}
	break;

	case CFG_SET_SHARPNESS:
	{
	    rc = mt9m114_set_sharpness(cfg_data.cfg.sharpness);
	}
	break;

	case CFG_SET_LENS_SHADING:
	{
	    /*
	       * no code here
	       */
	    //rc = mt9m114_set_lensshading(cfg_data.cfg.lensshading);
	    rc = 0;
	}
	break;

	case CFG_SET_EXPOSURE_COMPENSATION:
	{
	    rc = mt9m114_set_exposure_compensation(cfg_data.cfg.exposure);
	}
	break;

	default:
	{
	    rc = -EFAULT;
	}
	break;
	}

	/*
	* Wake lock to prevent suspend
	*/
	//mt9m114_prevent_suspend();

	/* up(&mt9m114_sem); */
	return rc;
	}

	static int mt9m114_sensor_release(void)
	{
	int rc;
	pr_err("%s: entry\n", __func__);

	rc = gpio_request(3, "mt9m114");
	if (0 == rc)
	{
	/* ignore "rc" */
	pr_err("%s: entry,set reset to 0\n", __func__);
	rc = gpio_direction_output(3, 0);
	gpio_free(3);
	}
	MT9D114_CSI_CONFIG = 0;
	return rc;
	}
	static int mt9m114_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
	{
	int rc = 0;

	pr_err("%s: entry\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
	rc = -ENOTSUPP;
	goto probe_failure;
	}

	mt9m114_sensorw = kzalloc(sizeof(struct mt9m114_work_t), GFP_KERNEL);
	if (!mt9m114_sensorw)
	{
	rc = -ENOMEM;
	goto probe_failure;
	}

	i2c_set_clientdata(client, mt9m114_sensorw);

	mt9m114_client = client;

	return 0;

	probe_failure:
	kfree(mt9m114_sensorw);
	mt9m114_sensorw = NULL;
	pr_err("%s: rc = %d, failed!\n", __func__, rc);
	return rc;
	}

	static int __exit mt9m114_i2c_remove(struct i2c_client *client)
	{
	struct mt9m114_work_t *sensorw = i2c_get_clientdata(client);

	pr_err("%s: entry\n", __func__);

	free_irq(client->irq, sensorw);   
	kfree(sensorw);

	/*
	* Wake lock to prevent suspend
	*/
	//mt9m114_deinit_suspend();

	mt9m114_client = NULL;
	mt9m114_sensorw = NULL;

	return 0;
	}

	static const struct i2c_device_id mt9m114_id[] = {
	{ "mt9m114", 0},
	{ },
	};

	static struct i2c_driver mt9m114_driver = {
	.id_table = mt9m114_id,
	.probe  = mt9m114_i2c_probe,
	.remove = __exit_p(mt9m114_i2c_remove),
	.driver = {
	.name = MT9M114_I2C_BOARD_NAME,
	},
	};

	static int32_t mt9m114_i2c_add_driver(void)
	{
	int32_t rc = 0;
	pr_err("yan %s: E\n", __func__);

	rc = i2c_add_driver(&mt9m114_driver);
	if (IS_ERR_VALUE(rc))
	{
	goto init_failure;
	}

	return rc;

	init_failure:
	pr_err("%s: rc = %d, failed!\n", __func__, rc);
	return rc;
	}

	static void mt9m114_i2c_del_driver(void)
	{
	i2c_del_driver(&mt9m114_driver);
	}

	void mt9m114_exit(void)
	{
	pr_err("%s: entry\n", __func__);
	mt9m114_i2c_del_driver();
	}

	int mt9m114_sensor_probe(const struct msm_camera_sensor_info *info,
	                        struct msm_sensor_ctrl *s)
	{
	int rc;

	pr_err("yanwei %s: entry\n", __func__);

	rc = mt9m114_i2c_add_driver();
	if (rc < 0)
	{
	goto probe_failed;
	}
	rc = gpio_request(1, "ov5640");
	if (rc < 0)
	{
	pr_err("%s: set ov5640 pwd to 0!\n", __func__);
	return rc;	
	}
	pr_err("%s: set reset to 0\n", __func__);
	rc = gpio_direction_output(1, 1);
	gpio_free(1);

	if (!info || strcmp(info->sensor_name, "mt9m114"))
	{
	pr_err("%s: invalid parameters!\n", __func__);
	rc = -ENODEV;

	}

	mt9m114_ctrl = kzalloc(sizeof(struct mt9m114_ctrl_t), GFP_KERNEL);
	if (!mt9m114_ctrl)
	{
	pr_err("%s: kzalloc failed!\n", __func__);
	rc = -ENOMEM;

	}
	mt9m114_ctrl->sensordata = info;

	rc = msm_camera_power_backend(MSM_CAMERA_PWRUP_MODE);
	if (rc < 0)
	{
	pr_err("%s: camera_power_backend failed!\n", __func__);

	}
	msm_camio_clk_rate_set(MT9M114_CAMIO_MCLK);
	mdelay(10);
	rc = mt9m114_hard_reset(info);
	if (rc < 0)
	{
	pr_err("hard reset failed!\n");
	}

	if(mt9m114_client == NULL){
	pr_err("%s: mt9m114_client == NULL\n", __func__);

	}
	/* Read the Model ID of the sensor */
	model_id = 0x0000;
	rc = mt9m114_i2c_read(mt9m114_client->addr, REG_MT9M114_MODEL_ID, &model_id, WORD_LEN);
	pr_err("%s: model_id = 0x%x\n", __func__, model_id);

	if (model_id != MT9M114_MODEL_ID)
	{
	rc = -EFAULT;
	goto probe_failed;
	} 
 
	/*
	* Wake lock to prevent suspend
	*/
	//mt9m114_init_suspend();


	s->s_mount_angle = 0;
	s->s_camera_type = FRONT_CAMERA_2D;//BACK_CAMERA_2D;

	s->s_init       = mt9m114_sensor_init;
	s->s_config     = mt9m114_sensor_config;
	s->s_release    = mt9m114_sensor_release;

	mt9m114_sensor_release();
	return 0;

	probe_failed:
	pr_err("%s: rc = %d, failed!\n", __func__, rc);

	mt9m114_i2c_del_driver();

	return rc;
	}

	/* To implement the parallel init process */
	static void mt9m114_workqueue(struct work_struct *work)
	{
	int32_t rc;

	if (!pdev_wq)
	{
	pr_err("%s: pdev_wq is NULL!\n", __func__);
	return;
	}
	rc = msm_camera_drv_start(pdev_wq, mt9m114_sensor_probe);
	}

	static int32_t mt9m114_probe_workqueue(void)
	{
	int32_t rc;

	mt9m114_wq = create_singlethread_workqueue("mt9m114_wq");

	if (!mt9m114_wq)
	{
	pr_err("%s: mt9m114_wq is NULL!\n", __func__);
	return -EFAULT;
	}

	/*
	* Ignore "rc"
	* "queue_work"'s rc:
	* 0: already in work queue
	* 1: added into work queue
	*/   
	rc = queue_work(mt9m114_wq, &mt9m114_cb_work);

	return 0;
	}

	static int __mt9m114_probe(struct platform_device *pdev)
	{
	int32_t rc;

	pdev_wq = pdev;
	rc = mt9m114_probe_workqueue();
	return rc;
	}

	static struct platform_driver msm_camera_driver = {
	.probe = __mt9m114_probe,
	.driver = {
	.name = "msm_camera_mt9m114",
	.owner = THIS_MODULE,
	},
	};

	static int __init mt9m114_init(void)
	{
	pr_err("%s: E!\n", __func__);
	return platform_driver_register(&msm_camera_driver);
	}

	module_init(mt9m114_init);

