#ifndef __GPIO_BUTTONS_H__
#define __GPIO_BUTTONS_H__

struct gpio_button
{
	int code;//按键值
	char *desc;//描述信息
	int gpio;//对应的管脚编号
};


#endif
