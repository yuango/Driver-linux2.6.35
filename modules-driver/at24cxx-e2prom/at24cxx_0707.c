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
{	/*����ƥ�����clientʱ������,
	*����I2C0��
	**/
	{"at24cxx",0},
	{}//������
};

dev_t dev;//���ڱ��涯̬������豸��
struct at24cxx_dev
{
	struct cdev cdev;
	struct i2c_client *client;
};

struct at24cxx_dev *at24cxx_devp = NULL;
struct class *dev_class = NULL;
struct device *dev_device = NULL;

/*
*Ϊʵ�ֲ��Գ�������:./a.out w 0 0x55
*����豸��ƫ�Ƶ�ַ0��дֵ0x55
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

	ret = copy_from_user(val,buf,size);//���ش�������ݸ���size

	/*��֯Ҫ���͵���Ϣ,����ʱ��ͼ����*/
	msg[0].addr = at24cxx_devp->client->addr;//���豸��ַ(clien��probe��ƥ��ɹ��ĵ�ַ)
	msg[0].flags = 0;//��ʾд�ź�
	/*����Ҫд��Ĵ��豸�ڵ�ƫ�Ƶ�ַ0(ռ8bitһ�ֽڼ�val[0])
	*����Ҫд����豸������0x50(ռ8bitһ�ֽڼ�val[1])
	*��������ͬʱ��
	**/
	msg[0].buf = val;
	msg[0].len = 3;//���͵����ݳ���

	/*�ں����Ѿ�ʵ����I2C��������
	*��Ӧ��i2c_adapter�ʹ���(ͬcdev,һһ��Ӧ),���ṩ���շ�����
	*����:START..ADDR+W..ACK..offset..ACK..val..ACK..STOP
	*����ADDR+W,offset,val��Ҫ������֯��,�����ĸú��������Զ�ʵ��
	*client�ṹ�������ں˰�æ����,�����Աadapter���Ǵ�ŵ�I2C0������
	*I2C0���������ں˰�����Ϣ�ļ������clientʱָ����
	**/
	i2c_transfer(at24cxx_devp->client->adapter,msg,1);//ֻ��һ��msgҪ��,ÿ��msg��Ҫһ��start

	return 3;/*2��ʾд2�ֽ�*/
}

/*�û��ռ������:./a.out r 100
*(100���û�ָ����Ҫ��ȡ�Ĵ��豸��ƫ�Ƶ�ַ,ͨ��buf���ݵ��ں���)
*/
int at24cxx_read(struct file *filp,char __user *buf,
					size_t size,loff_t *offset)
{
	unsigned char address;
	unsigned char data[size];
	/*�����ο�start�ź�*/
	struct i2c_msg msg[2];
	int ret;

	ret = copy_from_user(&address,buf,1);//�Ȱ�buf�е�ƫ�Ƶ�ַ�����ں�

	msg[0].addr = at24cxx_devp->client->addr;//���豸��ַ
	msg[0].flags = 0;
	msg[0].buf = &address;
	msg[0].len = 1;//buf��Ҫ���͵������ֽ���

	msg[1].addr = at24cxx_devp->client->addr;
	msg[1].flags = 1;//read
	msg[1].buf = data;//�Ҵ��豸Ҫ�����ݷŵ�data��
	msg[1].len = size;//ÿ�ζ�1�ֽ�

	/*����msg����Ҫ����start�ź�
	*start,ACK,STOP�źŶ���I2C����������ʵ��
	*��i2c_transfer�����Զ���������ź�,
	*���������ݶ�дʱ����I2C���߷��Ͷ�д�ź�
	*/
	ret = i2c_transfer(at24cxx_devp->client->adapter,msg,ARRAY_SIZE(msg));

	/*����ȡ���Ĵ��豸���ݴ��ݵ��û�̬*/
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
{/*����1Ϊƥ��ɹ���client�ṹ���ַ*/
	/*ע���豸��*/
	alloc_chrdev_region(&dev,0,1,"AT24C01");

	/*����cdev�ռ�*/
	at24cxx_devp = kzalloc(sizeof(struct at24cxx_dev),GFP_KERNEL);

	at24cxx_devp->client = client;//��ƥ��ɹ���client�ṹ���ַ���������Ժ���

	/*��ʼ��cdev����ӽ��ں�*/
	cdev_init(&at24cxx_devp->cdev,&at24cxx_fops);
	cdev_add(&at24cxx_devp->cdev,dev,1);

	/*�����豸�ڵ��ļ�*/
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
	{/*��������Ʋ���������ƥ��*/
		.name = "AT24C01",
		.owner = THIS_MODULE,
	},

	.probe = at24cxx_probe,
	.remove = at24cxx_remove,

	/*i2c match use below,table����ſ���֧�ֵ������豸*/
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
