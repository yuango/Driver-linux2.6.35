#include <linux/init.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

MODULE_LICENSE("GPL");

struct net_device *virnet_card = NULL;

int virnet_card_init(struct net_device *dev)
{
	printk("enter card_init!\n");

	/*和以太网相关的进一步初始化*/
	ether_setup(dev);

	strcpy(dev->name,"virnet");//网卡设备名

	return 0;
}

int virnet_card_open(struct net_device *dev)
{
	printk("enter card_open!\n");

	return 0;
}

int	virnet_card_xmit(struct sk_buff *skb,struct net_device *dev)
{
	printk("enter card_xmit!\n");

	/*通过硬件发送数据*/

	/*发送完成后释放传入的skb空间(在无关层申请的skb空间)*/
	dev_kfree_skb(skb);

	return 0;
}


struct net_device_ops vir_netdev_ops=
{
	/*进一步初始化*/
	.ndo_init = virnet_card_init,
	.ndo_open = virnet_card_open,
	.ndo_start_xmit = virnet_card_xmit,
};

int __init virnet_init(void)
{
	/*申请net_device空间,100+0个该结构体空间*/
	virnet_card = alloc_etherdev(0);

	/*设置net_device*/
	virnet_card->netdev_ops = &vir_netdev_ops;

	/*注册net_device*/
	register_netdev(virnet_card);

	/*注册中断,中断处理*/

	return 0;
}

void __exit virnet_exit(void)
{
	unregister_netdev(virnet_card);

	free_netdev(virnet_card);
}

module_init(virnet_init);
module_exit(virnet_exit);
