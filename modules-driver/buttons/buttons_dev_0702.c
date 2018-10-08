#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>

#include "gpio_buttons.h"

MODULE_LICENSE("GPL");

struct resource button_resource[]=
{//��������Դ,4������(һ����Դ��ʾ���ĸ���Դ��ʾ��ʲô����?)
	{
		.start = IRQ_EINT0,
		.end  = IRQ_EINT0,
		/*������Դ������Ϊ�ж�������Դ*/
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_EINT1,
		.end  = IRQ_EINT1,
		/*������Դ������Ϊ�ж�������Դ*/
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_EINT2,
		.end  = IRQ_EINT2,
		/*������Դ������Ϊ�ж�������Դ*/
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_EINT3,
		.end  = IRQ_EINT3,
		/*������Դ������Ϊ�ж�������Դ*/
		.flags = IORESOURCE_IRQ,
	},
};

struct gpio_button gpio_buttons[]=
{
	{
		.code = KEY_UP,
		.desc = "UP",
		.gpio = S5PV210_GPH0(0),
	},
	{
		.code = KEY_DOWN,
		.desc = "DOWN",
		.gpio = S5PV210_GPH0(1),
	},
	{
		.code = KEY_LEFT,
		.desc = "LEFT",
		.gpio = S5PV210_GPH0(2),
	},
	{
		.code = KEY_RIGHT,
		.desc = "RIGNT",
		.gpio = S5PV210_GPH0(3),
	},
};

void gpio_button_release(struct device *dev)
{
	printk("enter gpio_button_release!\n");
}

struct platform_device button_dev = 
{//ARRAY_SIZE��������Ԫ�صĸ���
	.name = "mybutton",
	.resource = button_resource,
	.num_resources = ARRAY_SIZE(button_resource),
	/*�����Ƕ������Ա��ֵ��
	*�����ǶԸ����Ա������ֵ
	*/
	.dev = 
	{
		.platform_data = gpio_buttons,//˽������
		.release = gpio_button_release,
	},
};

int __init button_dev_init(void)
{
	platform_device_register(&button_dev);
	return 0;
}

void __exit button_dev_exit(void)
{
	platform_device_unregister(&button_dev);
}

module_init(button_dev_init);
module_exit(button_dev_exit);
