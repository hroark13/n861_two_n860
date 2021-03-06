/* linux/arch/arm/mach-msm/board-zte-wifi.c
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/if.h> /*For IFHWADDRLEN */

#include "devices.h"
#include "proc_comm.h"  /* For msm_proc_cmd */
#include <linux/gpio.h>

#include <linux/random.h>
#include <linux/jiffies.h>

#include <linux/proc_fs.h>


static void zte_wifi_create_random_MAC(unsigned char *ptr_mac)
{	
	uint rand_mac;
	
	srandom32((uint)jiffies);
	rand_mac = random32();
	ptr_mac[0] = 0x00;
	ptr_mac[1] = 0xd0;
	ptr_mac[2] = 0xd0;
	ptr_mac[3] = (unsigned char)rand_mac;
	ptr_mac[4] = (unsigned char)(rand_mac >> 8);
	ptr_mac[5] = (unsigned char)(rand_mac >> 16);
	
	printk("use Random MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
		ptr_mac[0], ptr_mac[1], ptr_mac[2],
		ptr_mac[3], ptr_mac[4], ptr_mac[5]);
}

static int zte_wifi_get_mac_addr(unsigned char *buf)
{
	int rc = 0;	
	uint32_t data1,data2;	
	static unsigned long count;
	static unsigned char saved_mac[IFHWADDRLEN];

	data2 = (1<<31);
	
	if (!buf){
		printk("zte_wifi_get_mac_addr(), null parameter !!\n");	
		return -EFAULT;
	}
	
	printk("Getting MAC the %ldth time. count = %ld.\n", count+1, count);
	
	if(count++ == 0){
		printk("Reading MAC from NV item 4678......\n");
		rc = msm_proc_comm(PCOM_CUSTOMER_CMD1, &data1, &data2);		
			
		if(!rc){
			saved_mac[5] = (unsigned char)((data2>>8)&0xff);
			saved_mac[4] = (unsigned char)(data2&0xff);
			saved_mac[3] = (unsigned char)((data1>>24)&0xff);
			saved_mac[2] = (unsigned char)((data1>>16)&0xff);
			saved_mac[1] = (unsigned char)((data1>>8)&0xff);
			saved_mac[0] = (unsigned char)(data1&0xff);

			printk("MAC from NV: %02X:%02X:%02X:%02X:%02X:%02X\n",
				saved_mac[0], saved_mac[1], saved_mac[2],
				saved_mac[3], saved_mac[4], saved_mac[5]);	

			if(saved_mac[0] == 0x11
					&& saved_mac[1] == 0x22
					&& saved_mac[2] == 0x33
					&& saved_mac[3] == 0x44
					&& saved_mac[4] == 0x55
					&& saved_mac[5] == 0x66){
					printk("MAC from NV is illegal\n");
					zte_wifi_create_random_MAC(saved_mac);					
			}
		}else{
			printk("Fail to read NV item 4678\n");
			zte_wifi_create_random_MAC(saved_mac);
			return -EFAULT;
		}
	}	
	printk("Use MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
				saved_mac[0], saved_mac[1], saved_mac[2],
				saved_mac[3], saved_mac[4], saved_mac[5]);	
	memcpy(buf, saved_mac, sizeof(unsigned char) * IFHWADDRLEN);
	return 0;
}

#if defined(CONFIG_BCM_WIFI)
#define ZTE_GPIO_WIFI_SHUTDOWN_N	30
#define ZTE_GPIO_WIFI_IRQ		42 

int zte_wifi_power(int on);
int zte_wifi_reset(int on);
int zte_wifi_set_carddetect(int on);

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;


static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *zte_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init zte_init_wifi_mem(void)
{
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}

static int zte_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

int zte_wifi_status_register(
			void (*callback)(int card_present, void *dev_id),
			void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

unsigned int zte_wifi_status(struct device *dev)
{
	return zte_wifi_cd;
}


int zte_wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	zte_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int zte_wifi_power_state;

int zte_wifi_power(int on)
{
	printk("%s:  shaohua power %d \n", __func__, on);

	if((gpio_request(ZTE_GPIO_WIFI_SHUTDOWN_N,"shaohua")) < 0) {
		printk("request fail \n");
	} else {
		printk("shaohua request reg_on  success \n");
		gpio_direction_output(ZTE_GPIO_WIFI_SHUTDOWN_N, 0);
		mdelay(10);
		gpio_direction_output(ZTE_GPIO_WIFI_SHUTDOWN_N, on);
		mdelay(200);
		gpio_free(ZTE_GPIO_WIFI_SHUTDOWN_N);
	}

	printk("shaohua gpio_set_value = %d \n", 
		gpio_get_value(ZTE_GPIO_WIFI_SHUTDOWN_N));

	zte_wifi_power_state = on;
	return 0;
}

static int zte_wifi_reset_state;

int zte_wifi_reset(int on)
{
	printk("%s: do nothing\n", __func__);
	zte_wifi_reset_state = on;
	return 0;
}

static struct resource zte_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= MSM_GPIO_TO_INT(ZTE_GPIO_WIFI_IRQ),
		.end		= MSM_GPIO_TO_INT(ZTE_GPIO_WIFI_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
		//.flags          = IRQF_TRIGGER_FALLING,

	},
};

static struct wifi_platform_data zte_wifi_control = {
	.set_power      = zte_wifi_power,
	.set_reset      = zte_wifi_reset,
	.set_carddetect = zte_wifi_set_carddetect,
	.mem_prealloc	= zte_wifi_mem_prealloc, 
	.get_mac_addr	= zte_wifi_get_mac_addr, 
};

static struct platform_device zte_wifi_device = {
        .name           = "bcmdhd_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(zte_wifi_resources),
        .resource       = zte_wifi_resources,
        .dev            = {
                .platform_data = &zte_wifi_control,
        },
};
static int __init zte_wifi_init(void)
{
	int ret;

	pr_err("++++%s++++ \n", __func__);
	
	gpio_set_value(ZTE_GPIO_WIFI_SHUTDOWN_N, 0);
	zte_init_wifi_mem();
	ret = platform_device_register(&zte_wifi_device);
	pr_err("----%s----\n", __func__);
        return ret;
}

//late_initcall(zte_wifi_init);
#elif defined(CONFIG_ATH_WIFI)

#if defined(CONFIG_MACH_ATLAS40)
#include <mach/pmic.h>
#define WLAN_CHIP_PWD_PIN  113
#else                                                   //mooncake2, roamer2 and so on
#define WLAN_CHIP_PWD_PIN  93
#endif

#define WLAN_CHIP_WOW_PIN     114
//#define WLAN_3V3_EN		116
#define WLAN_1V8_EN             117
#define ZTE_PROC_COMM_CMD3_PMIC_GPIO 16

static int zte_wifi_power(int on)
{
	gpio_request(WLAN_CHIP_PWD_PIN, "WLAN_CHIP_PWD");

	if (on) {
	    pr_info("ar6005 back to live======================\n");
	    gpio_direction_output(WLAN_CHIP_PWD_PIN, 1);
	} else {
	    pr_info("ar6005 power cut======================\n");        
	    gpio_direction_output(WLAN_CHIP_PWD_PIN, 0);
	}		

	gpio_free(WLAN_CHIP_PWD_PIN);
	return 0;
}

static struct wifi_platform_data zte_wifi_control = {
	.set_power      = zte_wifi_power,
	.get_mac_addr	= zte_wifi_get_mac_addr,
};

static struct platform_device zte_wifi_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
	.dev            = {
				.platform_data = &zte_wifi_control,
	},
};

static int __init zte_wifi_init(void)
{
    int ret = 0;
	unsigned int pmic_gpio_config = (PMIC_GPIO_9 << 28)|	\
									(CONFIG_CMOS << 24)|	\
									(PMIC_GPIO_VIN7 << 20)|	\
									(SOURCE_GND << 16)|		\
									(BUFFER_MEDIUM << 12)|	\
									(0 << 8)|				\
									(1 << 4)|				\
									0x0;
	unsigned int pmic_gpio_cmd = ZTE_PROC_COMM_CMD3_PMIC_GPIO;

	do {
		pr_info("%s() enter\n", __func__);
		ret = gpio_tlmm_config(GPIO_CFG(WLAN_CHIP_WOW_PIN, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

       	if(ret) {
            		printk(KERN_ERR"WLAN_CHIP_WOW_PIN config failed\n");
	    		break;
        	} 
		gpio_request(WLAN_CHIP_WOW_PIN, "WLAN_WOW");
		//gpio_direction_output(WLAN_CHIP_WOW_PIN, 0);
		gpio_free(WLAN_CHIP_WOW_PIN);        

		printk("PCOM_CUSTOMER_CMD3 send/n");
		ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &pmic_gpio_config, &pmic_gpio_cmd);
		if(ret) {
            		printk(KERN_ERR"Failed to turn on LDO 3.3v\n");
        	}
		mdelay(50);
		
        ret = gpio_tlmm_config(GPIO_CFG(WLAN_1V8_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
        if(ret) {
        	printk(KERN_ERR"WLAN_1V8_EN config failed\n");
	 	break;
        } 
		gpio_request(WLAN_1V8_EN, "WLAN_1V8_EN");
		gpio_direction_output(WLAN_1V8_EN, 1);
        	gpio_free(WLAN_1V8_EN); 

        pr_info("%s() VREG 1.8v On\n", __func__);
        mdelay(100);
        pr_info("%s() Pull low CHIP PWD\n", __func__);
        /*
         * Pull low Chip power down pin
         */		
		ret = gpio_tlmm_config(GPIO_CFG(WLAN_CHIP_PWD_PIN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
        	if(ret) {
            		printk(KERN_ERR"WLAN_CHIP_PWD_PIN config failed\n");
	    		break;
        	} 
		gpio_request(WLAN_CHIP_PWD_PIN, "WLAN_CHIP_PWD");
		gpio_direction_output(WLAN_CHIP_PWD_PIN, 0);
		gpio_free(WLAN_CHIP_PWD_PIN);
		
		platform_device_register(&zte_wifi_device);
		
		return 0;

    } while (0);

    return ret;
}
#endif

static int common_read_proc(
char *page, char **start, off_t off, int count, int *eof, void *data, char *inputbuf , int inputlen )
{
		int len = inputlen;
		static int goff=0;
		if(off==0)
			goff=0;
        if (off >= len)
                return 0;
        if (count > len - off)
                count = len - off;		
		*start=page;		
        memcpy(page, inputbuf + goff, count);
		goff+=count;		
		printk("read:*start=0x%x len=%d off=%d count=%d\n", (unsigned int)*start,len,(int)off,count);	
		//return 0;
		if(off+count==len)
			*eof=1;
        return count;
}


static int proc_read_wifi_mac(
char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char mac_buf[6];
	char mac_formated[20];
	
	printk("%s, enter!\n", __func__);
	
	zte_wifi_get_mac_addr(mac_buf);
	
	sprintf(mac_formated, "%02X:%02X:%02X:%02X:%02X:%02X",
		mac_buf[0], mac_buf[1], mac_buf[2],
		mac_buf[3], mac_buf[4], mac_buf[5]);

	len = strlen(mac_formated);

	printk("buf=%s\n", mac_formated);
	printk("len=%d\n", len);
	
	return common_read_proc(page, start, off, count, eof, data, mac_formated, len);	
 
}

int wlan_init_power(void)
{
	struct proc_dir_entry * d_entry;

	d_entry = create_proc_entry("WIFI_MAC_ADDR", 0, NULL);
	if (d_entry)
	{
		d_entry->read_proc = proc_read_wifi_mac;
		d_entry->write_proc = NULL;
		d_entry->data = NULL;
	}


	return zte_wifi_init();
}
