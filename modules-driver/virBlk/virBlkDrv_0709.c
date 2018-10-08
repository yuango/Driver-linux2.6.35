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
	unsigned long size;//device的存储容量
	u8 *data;//拿内存模拟nand,data指向存储位置
	spinlock_t lock;//邋request_queue操作过程中会使用到
};

struct _virblk_device virblk_device;

int logical_sector_size = 512;//一个扇区512字节
int nsectors = 1024;//该设备共有1024个扇区

struct request_queue *virblk_queue = NULL;

struct block_device_operations virblk_fops =
{
	.owner = THIS_MODULE,
};

/*
*sector,该request要操作的起始扇区
*nsect,该request要连续操作的扇区个数
*buffer,该request操作的源或目标缓存
*write,1表示对该设备写,
*		0表示对该设备读
*/
void virblk_transfer(sector_t sector,unsigned long nsect,
							char *buffer,int write)
{
	unsigned long offset = sector * logical_sector_size;//取扇区起始位置偏移地址
	unsigned long nbytes = nsect * logical_sector_size;//连续操作的字节数
	/*操作不能越界*/
	if((offset+nbytes) > virblk_device.size)
	{
		printk("beyond end!\n");

		return;
	}

	/*硬件操作*/
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
	
	/*从某个队列中获取一个请求,成功返回获取的请求首地址*/
	req = blk_fetch_request(q);

	/*队列中可能有多个请求需要处理*/
	while(req != NULL)
	{
		/*参数中的函数都是取得req中的某个成员
		*1 return rq->__sector
		*4 req->cmd_flags & 1,
		*cmd_flags中存放的值代表该req的数据方向
		*但是这里cmd_flags值从何而来??
		*/
		virblk_transfer(blk_rq_pos(req),blk_rq_cur_sectors(req),req->buffer,rq_data_dir(req));//单独写个具体处理函数

		/*处理成功后要通知内核从队列中将该请求节点去掉
		*__blk_end_request_cur执行成功之后才表示该请求从
		*队列中被删除,返回false说明请求被做完,
		*true说明还没有处理完请求,参数0表示无错误处理
		*/
		if(!__blk_end_request_cur(req,0))
		{
			/*取下一个请求,难道指向队列中请求的指针可以自动移动?*/
			req = blk_fetch_request(q);
		}
	}
}

int __init virblk_init(void)
{
	/*申请设备空间,用以后续对扇区的读写操作*/
	virblk_device.size = nsectors * logical_sector_size;
	virblk_device.data = vmalloc(virblk_device.size);

	if(IS_ERR(virblk_device.data))
	{
		return -ENOMEM;
	}

	/*申请块设备号,起块设备名,为gendisk服务*/
	blk_dev = register_blkdev(0,"virblk");//0自动分配设备号存到返回值中

	/*申请gendisk空间*/
	virblk_device.gd= alloc_disk(1);//一个gendisk

	/*设置gendisk*/
	virblk_device.gd->major = blk_dev;//主设备号
	virblk_device.gd->first_minor = 0;//起始设备号
	virblk_device.gd->fops = &virblk_fops;
	strcpy(virblk_device.gd->disk_name,"virblk0");

	/*设置该设备的容量和扇区数
	*size,单位是扇区个数
	*/
	set_capacity(virblk_device.gd,nsectors);

	/*创建request_queue,包含request的处理函数和自旋锁*/

	spin_lock_init(&virblk_device.lock);//用锁之前需要先初始化
	virblk_queue = blk_init_queue(virblk_request_func,&virblk_device.lock);//成功返回队列头指针

	/*关联gendisk对应的request_queue*/
	virblk_device.gd->queue = virblk_queue;

	/*添加gendisk到内核*/
	add_disk(virblk_device.gd);

	return 0;
}

void __exit virblk_exit(void)
{
	del_gendisk(virblk_device.gd);

	/*引用计数减一*/
	put_disk(virblk_device.gd);

	/*清空请求队列*/
	blk_cleanup_queue(virblk_queue);

	unregister_blkdev(blk_dev,"virblk");

	vfree(virblk_device.data);
}

module_init(virblk_init);
module_exit(virblk_exit);
