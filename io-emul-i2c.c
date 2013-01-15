#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/sys_config.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c-dev.h>
#include <linux/delay.h>
#include <linux/cdev.h>

#define DRIVER_NAME "io_emul_i2c"
#define I2C_GET_GPIO 0x707
#define I2C_SET_GPIO 0x708

#define GPIO_KEY	0
#define GPIO_LED_1	1
#define GPIO_LED_2	2
#define GPIO_LED_3	3
#define GPIO_RX		4
#define GPIO_TX		5

#define GPIO_PINS_SET "lcd0_param"
#define GPIO_NUM 6
static char* gpio_pins[GPIO_NUM] = 
{
	"lcdd0", //for key
	"lcdd1", // for led1
	"lcdd2", // for led2
	"lcdd3", //for led3
	"lcdd4", //for tx
	"lcdd5", //for rx
};

struct gpio_setting_t
{
	u32 gpio;
	u32 value;
};
struct i2c_emul_platform_data
{
	u32 sda_handle;
	u32 scl_handle;
	
	u32 gpio_handles[GPIO_NUM];

	char*   pin_set;
	char*   sda_pin;
	char*   scl_pin;
	int		udelay;
	int		timeout;
	unsigned int	sda_is_open_drain:1;
	unsigned int	scl_is_open_drain:1;
	unsigned int	scl_is_output_only:1;
};

/* Toggle SDA by changing the direction of the pin */
static void i2c_gpio_setsda_dir(void *data, int state)
{
	struct i2c_emul_platform_data *pdata = data;
	int ret;
	if (state) {
		ret = gpio_set_one_pin_io_status(pdata->sda_handle, 0, NULL); // input
	} else {
		ret = gpio_set_one_pin_io_status(pdata->sda_handle, 1, NULL); // output
		gpio_write_one_pin_value(pdata->sda_handle, 0, NULL);
	}
}

/*
 * Toggle SDA by changing the output value of the pin. This is only
 * valid for pins configured as open drain (i.e. setting the value
 * high effectively turns off the output driver.)
 */
static void i2c_gpio_setsda_val(void *data, int state)
{
	struct i2c_emul_platform_data *pdata = data;

	gpio_write_one_pin_value(pdata->sda_handle, state, NULL);
}

/* Toggle SCL by changing the direction of the pin. */
static void i2c_gpio_setscl_dir(void *data, int state)
{
	struct i2c_emul_platform_data *pdata = data;
	
	if (state) {
		gpio_set_one_pin_io_status(pdata->scl_handle, 0, NULL);
	} else {
		gpio_set_one_pin_io_status(pdata->scl_handle, 1, NULL);
		gpio_write_one_pin_value(pdata->scl_handle, 0, NULL);
	}
}

/*
 * Toggle SCL by changing the output value of the pin. This is used
 * for pins that are configured as open drain and for output-only
 * pins. The latter case will break the i2c protocol, but it will
 * often work in practice.
 */
static void i2c_gpio_setscl_val(void *data, int state)
{
	struct i2c_emul_platform_data *pdata = data;

	gpio_write_one_pin_value(pdata->scl_handle, state, NULL);
}

static int i2c_gpio_getsda(void *data)
{
	struct i2c_emul_platform_data *pdata = data;

	int ret = gpio_read_one_pin_value(pdata->sda_handle, NULL);
    	return ret;
}

static int i2c_gpio_getscl(void *data)
{
	struct i2c_emul_platform_data *pdata = data;

	int ret = gpio_read_one_pin_value(pdata->scl_handle, NULL);
    	return ret;
}


#ifdef DEBUG
#define bit_dbg(level, dev, format, args...) \
	do { \
		if (i2c_debug >= level) \
			dev_dbg(dev, format, ##args); \
	} while (0)
#else
#define bit_dbg(level, dev, format, args...) \
	do {} while (0)
#endif /* DEBUG */

static struct i2c_adapter *g_adap = NULL;
#define setsda(adap, val)	adap->setsda(adap->data, val)
#define setscl(adap, val)	adap->setscl(adap->data, val)
#define getsda(adap)		adap->getsda(adap->data)
#define getscl(adap)		adap->getscl(adap->data)

static inline void sdalo(struct i2c_algo_bit_data *adap)
{
	setsda(adap, 0);
	udelay((adap->udelay + 1) / 2);
}

static inline void sdahi(struct i2c_algo_bit_data *adap)
{
	setsda(adap, 1);
	udelay((adap->udelay + 1) / 2);
}

static inline void scllo(struct i2c_algo_bit_data *adap)
{
	setscl(adap, 0);
	udelay(adap->udelay / 2);
}

static int sclhi(struct i2c_algo_bit_data *adap)
{
	unsigned long start;

	setscl(adap, 1);

	if (!adap->getscl)
		goto done;

	start = jiffies;
	while (!getscl(adap)) {
		if (time_after(jiffies, start + adap->timeout))
			return -ETIMEDOUT;
		cond_resched();
	}

done:
	udelay(adap->udelay);
	return 0;
}

static void i2c_start(struct i2c_algo_bit_data *adap)
{
	setsda(adap, 0);
	udelay(adap->udelay);
	scllo(adap);
}

static void i2c_repstart(struct i2c_algo_bit_data *adap)
{
	sdahi(adap);
	sclhi(adap);
	setsda(adap, 0);
	udelay(adap->udelay);
	scllo(adap);
}


static void i2c_stop(struct i2c_algo_bit_data *adap)
{
	sdalo(adap);
	sclhi(adap);
	setsda(adap, 1);
	udelay(adap->udelay);
}


static int i2c_outb(struct i2c_adapter *i2c_adap, unsigned char c)
{
	int i;
	int sb;
	int ack;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	for (i = 7; i >= 0; i--) {
		sb = (c >> i) & 1;
		setsda(adap, sb);
		udelay((adap->udelay + 1) / 2);
		if (sclhi(adap) < 0) {
			bit_dbg(1, &i2c_adap->dev, "i2c_outb: 0x%02x, "
				"timeout at bit #%d\n", (int)c, i);
			return -ETIMEDOUT;
		}

		scllo(adap);
	}
	sdahi(adap);
	if (sclhi(adap) < 0) {
		bit_dbg(1, &i2c_adap->dev, "i2c_outb: 0x%02x, "
			"timeout at ack\n", (int)c);
		return -ETIMEDOUT;
	}

	ack = !getsda(adap);
	bit_dbg(2, &i2c_adap->dev, "i2c_outb: 0x%02x %s\n", (int)c,
		ack ? "A" : "NA");

	scllo(adap);
	return ack;
}


static int i2c_inb(struct i2c_adapter *i2c_adap)
{
	int i;
	unsigned char indata = 0;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	sdahi(adap);
	for (i = 0; i < 8; i++) {
		if (sclhi(adap) < 0) {
			bit_dbg(1, &i2c_adap->dev, "i2c_inb: timeout at bit "
				"#%d\n", 7 - i);
			return -ETIMEDOUT;
		}
		indata *= 2;
		if (getsda(adap))
			indata |= 0x01;
		setscl(adap, 0);
		udelay(i == 7 ? adap->udelay / 2 : adap->udelay);
	}
	return indata;
}

static int try_address(struct i2c_adapter *i2c_adap,
		       unsigned char addr, int retries)
{
	struct i2c_algo_bit_data *adap = g_adap->algo_data;
	int i, ret = 0;

	for (i = 0; i <= retries; i++) {
		ret = i2c_outb(i2c_adap, addr);
		if (ret == 1 || i == retries)
			break;
		bit_dbg(3, &i2c_adap->dev, "emitting stop condition\n");
		i2c_stop(adap);
		udelay(adap->udelay);
		yield();
		bit_dbg(3, &i2c_adap->dev, "emitting start condition\n");
		i2c_start(adap);
	}
	if (i && ret)
		bit_dbg(1, &i2c_adap->dev, "Used %d tries to %s client at "
			"0x%02x: %s\n", i + 1,
			addr & 1 ? "read from" : "write to", addr >> 1,
			ret == 1 ? "success" : "failed, timeout?");
	return ret;
}

static int sendbytes(struct i2c_adapter *i2c_adap, unsigned char* buf, int len)
{
	const unsigned char *temp = buf;
	int count = len;
	unsigned short nak_ok = 1;
	int retval;
	int wrcount = 0;

	while (count > 0) {
		retval = i2c_outb(i2c_adap, *temp);

		if ((retval > 0) || (nak_ok && (retval == 0))) {
			count--;
			temp++;
			wrcount++;

		} else if (retval == 0) {
			dev_err(&i2c_adap->dev, "sendbytes: NAK bailout.\n");
			return -EIO;

		} else {
			dev_err(&i2c_adap->dev, "sendbytes: error %d\n",
					retval);
			return retval;
		}
	}
	return wrcount;
}

static int acknak(struct i2c_adapter *i2c_adap, int is_ack)
{
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	if (is_ack)
		setsda(adap, 0);
	udelay((adap->udelay + 1) / 2);
	if (sclhi(adap) < 0) {
		dev_err(&i2c_adap->dev, "readbytes: ack/nak timeout\n");
		return -ETIMEDOUT;
	}
	scllo(adap);
	return 0;
}

static int readbytes(struct i2c_adapter *i2c_adap, unsigned char* buf, int len)
{
	int inval;
	int rdcount = 0;
	unsigned char *temp = buf;
	int count = len;
	const unsigned flags = 0;

	while (count > 0) {
		inval = i2c_inb(i2c_adap);
		if (inval >= 0) {
			*temp = inval;
			rdcount++;
		} else {
			break;
		}

		temp++;
		count--;

		bit_dbg(2, &i2c_adap->dev, "readbytes: 0x%02x %s\n",
			inval,
			(flags & I2C_M_NO_RD_ACK)
				? "(no ack/nak)"
				: (count ? "A" : "NA"));

		if (!(flags & I2C_M_NO_RD_ACK)) {
			inval = acknak(i2c_adap, count);
			if (inval < 0)
				return inval;
		}
	}
	return rdcount;
}

static ssize_t i2cdev_read(struct file *file, char __user *buf, size_t count,
		loff_t *offset)
{
	char *tmp;
	int ret;

	unsigned char addr = (unsigned char)file->private_data;

	if (count > 8192)
		count = 8192;

	
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	i2c_start(g_adap->algo_data);

	
	if (try_address(g_adap, (addr << 1) | 1,3)) {
		ret = readbytes(g_adap, tmp, count);
		
		if (ret >= 0)
			ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;

	} else {
		ret = EFAULT;
	}
	i2c_stop(g_adap->algo_data);
	
	kfree(tmp);
	return ret;
}

static ssize_t i2cdev_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	int ret;
	char *tmp;
	unsigned char addr = (unsigned char)file->private_data;

	if (count > 8192)
		count = 8192;

	
	
	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	i2c_start(g_adap->algo_data);

	if (try_address(g_adap, (addr << 1) | 0,3)) {
		
		ret = sendbytes(g_adap, tmp, count);
		
	} else {
		ret = EFAULT;	
	}
	i2c_stop(g_adap->algo_data);
	
	kfree(tmp);
	return ret;
}

static long i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned long funcs;
	struct i2c_algo_bit_data *bit_data = g_adap->algo_data;
	struct i2c_emul_platform_data *pdata = bit_data->data;

	switch (cmd) {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
		file->private_data = (void*)arg;
		return 0;
	case I2C_GET_GPIO:
		{
			struct gpio_setting_t g;
			__copy_from_user(&g, arg, sizeof(struct gpio_setting_t));
			
			if (g.gpio != GPIO_KEY) {
				printk("request gpio error");
				return -EINVAL;
			}
			
			g.value = gpio_read_one_pin_value(pdata->gpio_handles[g.gpio], NULL);
			__copy_to_user(arg, &g, sizeof(struct gpio_setting_t));
		}
	break;
	case I2C_SET_GPIO:
		{
			struct gpio_setting_t g;
			__copy_from_user(&g, arg, sizeof(struct gpio_setting_t));
			
			if (g.gpio = GPIO_KEY || g.gpio >= GPIO_NUM) {
				printk("request gpio error");
				return -EINVAL;
			}
			
			gpio_write_one_pin_value(pdata->gpio_handles[g.gpio], g.value, NULL);
		}
	break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static int i2cdev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int i2cdev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations i2cdev_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= i2cdev_read,
	.write		= i2cdev_write,
	.unlocked_ioctl	= i2cdev_ioctl,
	.open		= i2cdev_open,
	.release	= i2cdev_release,
};

static struct cdev *i2c_cdev;
static dev_t devid ;
static struct class *i2c_class;

static int __devinit i2c_gpio_probe(struct platform_device *pdev)
{
	struct i2c_emul_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	int ret, i;

	
	alloc_chrdev_region(&devid, 0, 1, DRIVER_NAME);
	i2c_cdev = cdev_alloc();
	cdev_init(i2c_cdev, &i2cdev_fops);
	i2c_cdev->owner = THIS_MODULE;
	ret = cdev_add(i2c_cdev, devid, 1);

	if (ret) {
		printk("failed to register character device /dev/io_emul_i2c\n");
		return -1;
	}
	
	i2c_class = class_create(THIS_MODULE, DRIVER_NAME);
	device_create(i2c_class, NULL,
	              devid, NULL, DRIVER_NAME);   
		
	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;
		
	for (i = 0; i < GPIO_NUM; i++) {
		pdata->gpio_handles[i] = gpio_request_ex(GPIO_PINS_SET, gpio_pins[i]);
		if (!pdata->gpio_handles[i]) {
			printk("failed to get %s handle\n", gpio_pins[i]);
			return -1;	
		}
	}
	
	gpio_set_one_pin_io_status(pdata->gpio_handles[GPIO_KEY], 0, NULL); //input
	gpio_set_one_pin_io_status(pdata->gpio_handles[GPIO_LED_1], 1, NULL); //output
	gpio_set_one_pin_io_status(pdata->gpio_handles[GPIO_LED_2], 1, NULL); //output
	gpio_set_one_pin_io_status(pdata->gpio_handles[GPIO_LED_3], 1, NULL); //output
	gpio_set_one_pin_io_status(pdata->gpio_handles[GPIO_TX], 1, NULL); //output
	gpio_set_one_pin_io_status(pdata->gpio_handles[GPIO_TX], 1, NULL); //output

	pdata->sda_handle = gpio_request_ex(pdata->pin_set, pdata->sda_pin);
	pdata->scl_handle = gpio_request_ex(pdata->pin_set, pdata->scl_pin);
	if (!pdata->sda_handle || !pdata->scl_handle) {
		printk("failed to get %s handle\n", pdata->pin_set);
		return -1;
	}


	ret = -ENOMEM;
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap)
		goto err_alloc_adap;
	bit_data = kzalloc(sizeof(struct i2c_algo_bit_data), GFP_KERNEL);
	if (!bit_data)
		goto err_alloc_bit_data;

	if (pdata->sda_is_open_drain) {
		gpio_write_one_pin_value(pdata->sda_handle, 1, NULL);
		bit_data->setsda = i2c_gpio_setsda_val;
	} else {
		gpio_set_one_pin_io_status(pdata->sda_handle, 0, NULL);
		bit_data->setsda = i2c_gpio_setsda_dir;
	}

	if (pdata->scl_is_open_drain || pdata->scl_is_output_only) {
		gpio_write_one_pin_value(pdata->scl_handle, 1, NULL);
		bit_data->setscl = i2c_gpio_setscl_val;
	} else {
		gpio_set_one_pin_io_status(pdata->scl_handle, 0, NULL);
		bit_data->setscl = i2c_gpio_setscl_dir;
	}
	
	
	

	if (!pdata->scl_is_output_only)
		bit_data->getscl = i2c_gpio_getscl;

	// set pull : 0 for pull disabled, 1 for pull up, 2 for pull down
	gpio_set_one_pin_pull(pdata->sda_handle, 1, NULL);
	gpio_set_one_pin_pull(pdata->scl_handle, 1, NULL);

	bit_data->getsda = i2c_gpio_getsda;

	if (pdata->udelay)
		bit_data->udelay = pdata->udelay;
	else if (pdata->scl_is_output_only)
		bit_data->udelay = 50;			/* 10 kHz */
	else
		bit_data->udelay = 5;			/* 100 kHz */

	if (pdata->timeout)
		bit_data->timeout = pdata->timeout;
	else
		bit_data->timeout = HZ / 10;		/* 100 ms */

	bit_data->data = pdata;

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "ioemul-i2c%d", pdev->id);
	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	adap->nr = (pdev->id != -1) ? pdev->id : 0;

  	g_adap = adap;
 
	platform_set_drvdata(pdev, adap);

	dev_info(&pdev->dev, "using pins %s (SDA) and %s (SCL%s)\n",
			pdata->sda_pin, pdata->scl_pin,
			pdata->scl_is_output_only
			? ", no clock stretching" : "");

	return 0;

err_add_bus:
err_request_scl:
	gpio_release(pdata->sda_handle, 0);
	gpio_release(pdata->scl_handle, 0);
err_request_sda:
	kfree(bit_data);
err_alloc_bit_data:
	kfree(adap);
err_alloc_adap:
	class_destroy(i2c_class);
	cdev_del(i2c_cdev);
	return ret;
}

static int __devexit i2c_gpio_remove(struct platform_device *pdev)
{
	struct i2c_emul_platform_data *pdata;
	struct i2c_adapter *adap;

	adap = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	i2c_del_adapter(adap);
	gpio_release(pdata->sda_handle, 0);
	gpio_release(pdata->scl_handle, 0);
	kfree(adap->algo_data);
	kfree(adap);
	class_destroy(i2c_class);
	cdev_del(i2c_cdev);

	return 0;
}

static struct platform_driver i2c_emul_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= i2c_gpio_probe,
	.remove		= __devexit_p(i2c_gpio_remove),
};

static struct i2c_emul_platform_data i2c_emul_data  = {
#if 1
	.pin_set = "mmc0_para",
	.sda_pin = "sdc_d0",	
	.scl_pin = "sdc_d1",
#else
	.pin_set = "twi0_para",
	.sda_pin = "twi0_sda",	
	.scl_pin = "twi0_scl",	
#endif
	.udelay = 5
};

struct platform_device i2c_emul_device = {
	.name	= DRIVER_NAME,	
	.dev = { .platform_data = &i2c_emul_data },
};


static int i2c_emul_init()
{
	int ret = platform_device_register(&i2c_emul_device);
	if (ret) {
		printk("failed to register i2c emulation device!!\n");
		return -1;
	}

	ret = platform_driver_register(&i2c_emul_driver);
	if (ret) {
		platform_device_unregister(&i2c_emul_device);
		printk(KERN_ERR "i2c-emul: probe failed: %d\n", ret);
		return -1;
	}

	return 0;
}

static void i2c_emul_deinit()
{
	platform_driver_unregister(&i2c_emul_driver);
	platform_device_unregister(&i2c_emul_device);
}


static int __init ioemul_init(void)
{
	printk("%s\n", __func__);
	i2c_emul_init();

	return 0;
}

static void __exit ioemul_exit(void)
{
	printk("%s\n", __func__);
}

module_init(ioemul_init);
module_exit(ioemul_exit);

MODULE_AUTHOR("Derek Quan");
MODULE_DESCRIPTION("i2c emulation module");
MODULE_LICENSE("GPL");

