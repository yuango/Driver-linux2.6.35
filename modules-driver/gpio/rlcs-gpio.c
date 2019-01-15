/*
** 2019-01-11 First release
** Driver for GPF0_4 ~ GPF2_7, 20 ports totally
** ``Character Device Driver``
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>


MODULE_AUTHOR("Jiangyuan <yuango.jiang@gmail.com>");
MODULE_DESCRIPTION("RLCS GPIO Driver");
MODULE_LICENSE("GPL v2");

#define RLCS_GPIO_NUM 20
#define RLCS_DEV_NAME "rlcs_gpio_dev"
#define RLCS_CLASS_NAME "rlcs_gpio_class"
#define RLCS_DEVICE_NAME "rlcs_gpio_device"

int rlcs_major;
struct class *rlcs_class = NULL;
struct device *rlcs_device = NULL;

struct gpio rlcs_gpio_table[RLCS_GPIO_NUM] =
{
	{S5PV210_GPF0(4), GPIOF_OUT_INIT_HIGH, "OUT0"},
	{S5PV210_GPF0(5), GPIOF_OUT_INIT_HIGH, "OUT1"},
	{S5PV210_GPF0(6), GPIOF_OUT_INIT_HIGH, "OUT2"},
	{S5PV210_GPF0(7), GPIOF_OUT_INIT_HIGH, "OUT3"},
	{S5PV210_GPF1(0), GPIOF_OUT_INIT_HIGH, "OUT4"},
	{S5PV210_GPF1(1), GPIOF_OUT_INIT_HIGH, "OUT5"},
	{S5PV210_GPF1(2), GPIOF_OUT_INIT_HIGH, "OUT6"},
	{S5PV210_GPF1(3), GPIOF_OUT_INIT_HIGH, "OUT7"},
	{S5PV210_GPF1(4), GPIOF_OUT_INIT_HIGH, "OUT8"},
	{S5PV210_GPF1(5), GPIOF_OUT_INIT_HIGH, "OUT9"},
	{S5PV210_GPF1(6), GPIOF_OUT_INIT_HIGH, "OUT10"},
	{S5PV210_GPF1(7), GPIOF_OUT_INIT_HIGH, "OUT11"},
	{S5PV210_GPF2(0), GPIOF_OUT_INIT_HIGH, "OUT12"},
	{S5PV210_GPF2(1), GPIOF_OUT_INIT_HIGH, "OUT13"},
	{S5PV210_GPF2(2), GPIOF_OUT_INIT_HIGH, "OUT14"},
	{S5PV210_GPF2(3), GPIOF_OUT_INIT_HIGH, "OUT15"},
	{S5PV210_GPF2(4), GPIOF_OUT_INIT_HIGH, "OUT16"},
	{S5PV210_GPF2(5), GPIOF_OUT_INIT_HIGH, "OUT17"},
	{S5PV210_GPF2(6), GPIOF_OUT_INIT_HIGH, "OUT18"},
	{S5PV210_GPF2(7), GPIOF_OUT_INIT_HIGH, "OUT19"},
};

int rlcs_open(struct inode *inode, struct file *filp)
{
	printk("open success!\n");
	return 0;
}

long rlcs_ioctl(struct file *filp, unsigned int cmd, unsigned long index)
{
	if(index < 0 || index >= RLCS_GPIO_NUM)
	{
		printk("Invalid GPIO Number!\n");
		return -EINVAL;
	}
	switch(cmd)
	{
		case 1:
			gpio_set_value(rlcs_gpio_table[index].gpio, 1);
			break;
		case 0:
			gpio_set_value(rlcs_gpio_table[index].gpio, 0);
			break;
		default:
			printk("Invalid value set for GPIO!\n");
			return -EINVAL;			
	}

	return 0;
}

int rlcs_release(struct inode *inode, struct file *filp)
{
	
	int i;
	for(i=0; i<RLCS_GPIO_NUM; i++)
 	{
 		gpio_set_value(rlcs_gpio_table[i].gpio, 1);// reset the gpio port
 	}
	printk("close success!\n");
	return 0;
}

struct file_operations rlcs_fops = {
	.owner = THIS_MODULE,
	.open = rlcs_open,
	.unlocked_ioctl = rlcs_ioctl,
	.release = rlcs_release,
};

int __init rlcs_gpio_init(void)
{
	int err;
	rlcs_major = register_chrdev(0, RLCS_DEV_NAME, &rlcs_fops);
	if(rlcs_major < 0)
	{
		printk("register_chrdev failed!\n");
		goto failure_register_chrdev;
	}

	rlcs_class = class_create(THIS_MODULE, RLCS_CLASS_NAME);
	if(IS_ERR(rlcs_class))
	{
		printk("class_create failed!\n");
		goto failure_create_class;
	}

	rlcs_device = device_create(rlcs_class, NULL, MKDEV(rlcs_major, 0), NULL, RLCS_DEVICE_NAME);
	if(IS_ERR(rlcs_device))
	{
		printk("device_create failed!\n");
		goto failure_create_device;
	}

	err = gpio_request_array(rlcs_gpio_table, RLCS_GPIO_NUM);
	if(err)
	{
		printk("gpio_request_array failed!\n");
		goto failure_request_gpio_array;
	}

	return 0;

failure_request_gpio_array:
	device_destroy(rlcs_class, MKDEV(rlcs_major, 0));
failure_create_device:
	class_destroy(rlcs_class);
failure_create_class:
	unregister_chrdev(rlcs_major, RLCS_DEV_NAME);
failure_register_chrdev:
	return -1;
	
}

void __exit rlcs_gpio_exit(void)
{
	gpio_free_array(rlcs_gpio_table, RLCS_GPIO_NUM);
	device_destroy(rlcs_class, MKDEV(rlcs_major, 0));
	class_destroy(rlcs_class);
	unregister_chrdev(rlcs_major, RLCS_DEV_NAME);
}

module_init(rlcs_gpio_init);
module_exit(rlcs_gpio_exit);
