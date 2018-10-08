#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

MODULE_LICENSE("GPL");

const struct i2c_device_id at24cxx_id[] =
{	/*名称匹配添加client时的名字,
	*挂在I2C0上
	**/
	{"at24cxx",0},
	{}//结束符
};

dev_t dev;//用于保存动态申请的设备号
struct at24cxx_dev
{
	struct cdev cdev;
	struct i2c_client *client;
};

struct at24cxx_dev *at24cxx_devp = NULL;
struct class *dev_class = NULL;
struct device *dev_device = NULL;

/*
*为实现测试程序命令:./a.out w 0 0x55
*向从设备的偏移地址0处写值0x55
**/
int at24cxx_write(struct file *filp,const char __user *buf,
					size_t size,loff_t *offset)
{
	unsigned char val[3];
	int ret = 0;
	struct i2c_msg msg[1];

	if(size != 3)
	{
		return -EINVAL;
	}

	ret = copy_from_user(val,buf,size);//返回传输的数据个数size

	/*组织要发送的消息,依据时序图过程*/
	msg[0].addr = at24cxx_devp->client->addr;//从设备地址(clien是probe中匹配成功的地址)
	msg[0].flags = 0;//表示写信号
	/*发送要写入的从设备内的偏移地址0(占8bit一字节即val[0])
	*发送要写入从设备的数据0x50(占8bit一字节即val[1])
	*两个数据同时发
	**/
	msg[0].buf = val;
	msg[0].len = 3;//发送的数据长度

	/*内核中已经实现了I2C驱动程序
	*相应的i2c_adapter就存在(同cdev,一一对应),并提供了收发函数
	*功能:START..ADDR+W..ACK..offset..ACK..val..ACK..STOP
	*其中ADDR+W,offset,val需要我们组织好,其他的该函数都能自动实现
	*client结构体是由内核帮忙创建,而其成员adapter就是存放的I2C0适配器
	*I2C0是我们在内核板载信息文件中添加client时指定的
	**/
	i2c_transfer(at24cxx_devp->client->adapter,msg,1);//只有一个msg要发,每个msg都要一个start

	return 3;/*2表示写2字节*/
}

/*用户空间读命令:./a.out r 100
*(100是用户指定需要读取的从设备的偏移地址,通过buf传递到内核中)
*/
int at24cxx_read(struct file *filp,char __user *buf,
					size_t size,loff_t *offset)
{
	unsigned char address;
	unsigned char data[size];
	/*个数参考start信号*/
	struct i2c_msg msg[2];
	int ret;

	ret = copy_from_user(&address,buf,1);//先把buf中的偏移地址传到内核

	msg[0].addr = at24cxx_devp->client->addr;//从设备地址
	msg[0].flags = 0;
	msg[0].buf = &address;
	msg[0].len = 1;//buf中要发送的数据字节数

	msg[1].addr = at24cxx_devp->client->addr;
	msg[1].flags = 1;//read
	msg[1].buf = data;//找从设备要的数据放到data中
	msg[1].len = size;//每次读1字节

	/*两个msg就需要两个start信号
	*start,ACK,STOP信号都由I2C控制器驱动实现
	*即i2c_transfer可以自动添加以上信号,
	*作用是依据读写时序向I2C总线发送读写信号
	*/
	ret = i2c_transfer(at24cxx_devp->client->adapter,msg,ARRAY_SIZE(msg));

	/*将读取到的从设备数据传递到用户态*/
	ret = copy_to_user(buf,data,size);

	return ret;
}


struct file_operations at24cxx_fops =
{
	.owner = THIS_MODULE,
	.read = at24cxx_read,
	.write = at24cxx_write,
};

int at24cxx_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{/*参数1为匹配成功的client结构体地址*/
	/*注册设备号*/
	alloc_chrdev_region(&dev,0,1,"AT24C01");

	/*申请cdev空间*/
	at24cxx_devp = kzalloc(sizeof(struct at24cxx_dev),GFP_KERNEL);

	at24cxx_devp->client = client;//将匹配成功的client结构体地址保存起来以后用

	/*初始化cdev并添加进内核*/
	cdev_init(&at24cxx_devp->cdev,&at24cxx_fops);
	cdev_add(&at24cxx_devp->cdev,dev,1);

	/*创建设备节点文件*/
	dev_class = class_create(THIS_MODULE,"AT24CXX");
	dev_device = device_create(dev_class,NULL,dev,NULL,"at24cxx0");

	return 0;
}

int at24cxx_remove(struct i2c_client *client)
{
	device_destroy(dev_class,dev);
	class_destroy(dev_class);
	cdev_del(&at24cxx_devp->cdev);
	kfree(at24cxx_devp);
	unregister_chrdev_region(dev,1);
	return 0;
}

struct i2c_driver at24cxx_driver =
{
	.driver =
	{/*这里的名称并不会用于匹配*/
		.name = "AT24C01",
		.owner = THIS_MODULE,
	},

	.probe = at24cxx_probe,
	.remove = at24cxx_remove,

	/*i2c match use below,table里面放可以支持的所有设备*/
	.id_table = at24cxx_id,
};

int __init at24cxx_init(void)
{
	/**/
	i2c_add_driver(&at24cxx_driver);

	return 0;
}

void __exit at24cxx_exit(void)
{
	i2c_del_driver(&at24cxx_driver);
}

module_init(at24cxx_init);
module_exit(at24cxx_exit);
