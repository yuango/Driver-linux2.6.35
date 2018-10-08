#include <linux/init.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

MODULE_LICENSE("GPL");

struct net_device *virnet_card = NULL;

int virnet_card_init(struct net_device *dev)
{
	printk("enter card_init!\n");

	/*����̫����صĽ�һ����ʼ��*/
	ether_setup(dev);

	strcpy(dev->name,"virnet");//�����豸��

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

	/*ͨ��Ӳ����������*/

	/*������ɺ��ͷŴ����skb�ռ�(���޹ز������skb�ռ�)*/
	dev_kfree_skb(skb);

	return 0;
}


struct net_device_ops vir_netdev_ops=
{
	/*��һ����ʼ��*/
	.ndo_init = virnet_card_init,
	.ndo_open = virnet_card_open,
	.ndo_start_xmit = virnet_card_xmit,
};

int __init virnet_init(void)
{
	/*����net_device�ռ�,100+0���ýṹ��ռ�*/
	virnet_card = alloc_etherdev(0);

	/*����net_device*/
	virnet_card->netdev_ops = &vir_netdev_ops;

	/*ע��net_device*/
	register_netdev(virnet_card);

	/*ע���ж�,�жϴ���*/

	return 0;
}

void __exit virnet_exit(void)
{
	unregister_netdev(virnet_card);

	free_netdev(virnet_card);
}

module_init(virnet_init);
module_exit(virnet_exit);
