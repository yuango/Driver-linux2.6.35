#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include "gpio_buttons.h"

MODULE_LICENSE("GPL v2");

struct input_dev *buttons_dev = NULL;
struct timer_list buttons_timer;

volatile struct gpio_button *irq_button_data = NULL;

irqreturn_t buttons_isr(int irq,void *dev_id)
{
	irq_button_data = (volatile struct gpio_button *)dev_id;

	mod_timer(&buttons_timer,jiffies+HZ/100);

	return IRQ_HANDLED;
}

void buttons_timer_function(unsigned long data)
{
	unsigned int pinval;

	volatile struct gpio_button *pindesc = irq_button_data;

	if(!pindesc)
	{
		return;
	}

	pinval = gpio_get_value(pindesc->gpio);

	if(pinval)
	{
		input_event(buttons_dev,EV_KEY,pindesc->code,0);//�ͷ�
		input_event(buttons_dev,EV_SYN,0,0);
	}
	else
	{
		input_event(buttons_dev,EV_KEY,pindesc->code,1);//����
		input_event(buttons_dev,EV_SYN,0,0);
	}
}

int button_probe(struct platform_device *pdev )
{//�豸���豸����ƥ��ɹ����ں˾ͻ���õ��ú���,���Ӳ���ľ���ʵ�־���������ʵ��
	int i = 0;
	int ret = 0;

	struct resource *res = NULL;

	/*�����Ĳ�����ָ��ƥ��ɹ���button_dev*/
	struct gpio_button *data = pdev->dev.platform_data;

	/*����input��ϵͳ��ʽд����*/
	buttons_dev = input_allocate_device();

	/*���ø������豸*/
	set_bit(EV_KEY,buttons_dev->evbit);
	set_bit(EV_SYN,buttons_dev->evbit);
	set_bit(EV_REP,buttons_dev->evbit);

	for(;i<pdev->num_resources;i++)
	{//num_resource�б���ĸ������ǰ����ĸ���,��ߴ��������
		set_bit(data[i].code,buttons_dev->keybit);

		/*
		*��ȡ��Դ,
		*dev,ָ����ȡ��Դ����Դ
		*type,��ȡ��Դ����:IORESOURCE_IRQ/IORESOURCE_MEM
		*num,��ȡtype������Դ�еĵڼ���
		*/
		res = platform_get_resource(pdev,IORESOURCE_IRQ,i);
		/*ע���ж�,��һ������ͬpdev->resource[i].start*/
		ret = request_irq(res->start,buttons_isr,IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,data[i].desc,&(data[i]));
	}

	ret = input_register_device(buttons_dev);

	init_timer(&buttons_timer);
	buttons_timer.function = buttons_timer_function;
	add_timer(&buttons_timer);

	return 0;
}

int button_remove(struct platform_device *pdev)
{
	int i = 0;
	struct resource *res = NULL;
	struct gpio_button *data = pdev->dev.platform_data;

	del_timer(&buttons_timer);

	input_unregister_device(buttons_dev);

	for(;i<pdev->num_resources;i++)
	{
		res = platform_get_resource(pdev,IORESOURCE_IRQ,i);

		free_irq(res->start,data+i);
	}

	input_free_device(buttons_dev);

	return 0;
}

struct platform_driver button_drv =
{
	.probe = button_probe,
	.remove = button_remove,
	.driver =
	{
		.name = "mybutton",
	},
};

int __init button_drv_init(void)
{
	platform_driver_register(&button_drv);
	return 0;
}

void __exit button_drv_exit(void)
{
	platform_driver_unregister(&button_drv);
}

module_init(button_drv_init);
module_exit(button_drv_exit);
