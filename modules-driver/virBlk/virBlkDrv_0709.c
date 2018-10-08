#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/blkdev.h>

MODULE_LICENSE("GPL v2");

int blk_dev = 0;
struct _virblk_device
{
	struct gendisk *gd;
	unsigned long size;//device�Ĵ洢����
	u8 *data;//���ڴ�ģ��nand,dataָ��洢λ��
	spinlock_t lock;//��request_queue���������л�ʹ�õ�
};

struct _virblk_device virblk_device;

int logical_sector_size = 512;//һ������512�ֽ�
int nsectors = 1024;//���豸����1024������

struct request_queue *virblk_queue = NULL;

struct block_device_operations virblk_fops =
{
	.owner = THIS_MODULE,
};

/*
*sector,��requestҪ��������ʼ����
*nsect,��requestҪ������������������
*buffer,��request������Դ��Ŀ�껺��
*write,1��ʾ�Ը��豸д,
*		0��ʾ�Ը��豸��
*/
void virblk_transfer(sector_t sector,unsigned long nsect,
							char *buffer,int write)
{
	unsigned long offset = sector * logical_sector_size;//ȡ������ʼλ��ƫ�Ƶ�ַ
	unsigned long nbytes = nsect * logical_sector_size;//�����������ֽ���
	/*��������Խ��*/
	if((offset+nbytes) > virblk_device.size)
	{
		printk("beyond end!\n");

		return;
	}

	/*Ӳ������*/
	if(write)
	{
		memcpy(virblk_device.data+offset,buffer,nbytes);
	}
	else
	{
		memcpy(buffer,virblk_device.data+offset,nbytes);
	}
}

void virblk_request_func(struct request_queue *q)
{
	struct request *req = NULL;
	
	/*��ĳ�������л�ȡһ������,�ɹ����ػ�ȡ�������׵�ַ*/
	req = blk_fetch_request(q);

	/*�����п����ж��������Ҫ����*/
	while(req != NULL)
	{
		/*�����еĺ�������ȡ��req�е�ĳ����Ա
		*1 return rq->__sector
		*4 req->cmd_flags & 1,
		*cmd_flags�д�ŵ�ֵ�����req�����ݷ���
		*��������cmd_flagsֵ�Ӻζ���??
		*/
		virblk_transfer(blk_rq_pos(req),blk_rq_cur_sectors(req),req->buffer,rq_data_dir(req));//����д�����崦����

		/*����ɹ���Ҫ֪ͨ�ں˴Ӷ����н�������ڵ�ȥ��
		*__blk_end_request_curִ�гɹ�֮��ű�ʾ�������
		*�����б�ɾ��,����false˵����������,
		*true˵����û�д���������,����0��ʾ�޴�����
		*/
		if(!__blk_end_request_cur(req,0))
		{
			/*ȡ��һ������,�ѵ�ָ������������ָ������Զ��ƶ�?*/
			req = blk_fetch_request(q);
		}
	}
}

int __init virblk_init(void)
{
	/*�����豸�ռ�,���Ժ����������Ķ�д����*/
	virblk_device.size = nsectors * logical_sector_size;
	virblk_device.data = vmalloc(virblk_device.size);

	if(IS_ERR(virblk_device.data))
	{
		return -ENOMEM;
	}

	/*������豸��,����豸��,Ϊgendisk����*/
	blk_dev = register_blkdev(0,"virblk");//0�Զ������豸�Ŵ浽����ֵ��

	/*����gendisk�ռ�*/
	virblk_device.gd= alloc_disk(1);//һ��gendisk

	/*����gendisk*/
	virblk_device.gd->major = blk_dev;//���豸��
	virblk_device.gd->first_minor = 0;//��ʼ�豸��
	virblk_device.gd->fops = &virblk_fops;
	strcpy(virblk_device.gd->disk_name,"virblk0");

	/*���ø��豸��������������
	*size,��λ����������
	*/
	set_capacity(virblk_device.gd,nsectors);

	/*����request_queue,����request�Ĵ�������������*/

	spin_lock_init(&virblk_device.lock);//����֮ǰ��Ҫ�ȳ�ʼ��
	virblk_queue = blk_init_queue(virblk_request_func,&virblk_device.lock);//�ɹ����ض���ͷָ��

	/*����gendisk��Ӧ��request_queue*/
	virblk_device.gd->queue = virblk_queue;

	/*���gendisk���ں�*/
	add_disk(virblk_device.gd);

	return 0;
}

void __exit virblk_exit(void)
{
	del_gendisk(virblk_device.gd);

	/*���ü�����һ*/
	put_disk(virblk_device.gd);

	/*����������*/
	blk_cleanup_queue(virblk_queue);

	unregister_blkdev(blk_dev,"virblk");

	vfree(virblk_device.data);
}

module_init(virblk_init);
module_exit(virblk_exit);
