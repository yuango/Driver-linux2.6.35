/*
** 2018-05-07 First release
** Driver for CAN contoller -- mcp2515
** ``Character Device Driver``
*/
#include <linux/can.h>
#include <linux/delay.h>//mdelay
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>//user&kernel communicate
#include <linux/slab.h>//kzalloc
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/sched.h>//schedule()
#include <linux/interrupt.h>
#include <linux/mutex.h>
//#include <linux/spinlock_types.h>


MODULE_AUTHOR("Jiangyuan <yuango.jiang@gmail.com>");
MODULE_DESCRIPTION("MCP2515 CAN controller driver");
MODULE_LICENSE("GPL v2");

#define MCP2515_BUF_LEN	1000
#define SPI_TRANSFER_BUF_LEN 14

/* SPI interface instruction set */
#define INSTRUCTION_RESET   0xc0
#define INSTRUCTION_READ    0x03
#define INSTRUCTION_READ_RXB(n) ((n==0) ? 0x90 : 0x94)//RXB(0) or RXB(1) from RXBnSIDH
#define INSTRUCTION_WRITE   0x02
#define INSTRUCTION_LOAD_TXB(n) (2*n + 0x40)//TXB0 or TXB1 or TXB2 from TXBnSIDH
#define INSTRUCTION_READ_STATUS	0xa0
#define INSTRUCTION_READ_RXSTATUS	0xb0
#define INSTRUCTION_BIT_MODIFY  0x05

/* MCP2515 registers set */
#define CANSTAT 0x0e
#define CANCTRL 0x0f
#define TEC 0x1c
#define REC 0x1d
#define CANINTE 0x2b
#define CANINTF 0x2c
#define EFLG    0x2d
#define TXBCTRL(n)  (n*0x10 + 0x30)//n=0,1,2
#define RXBCTRL(n)  (n*0x10 + 0x60)//n=0,1

//CONFIG MODEL
#define CNF1    0x2a
#define CNF2    0x29
#define CNF3    0x28
//filtering registers--n=0,1,2,4,5,6(no 3)
#define RXFSIDH(n) ((n) * 4)//
#define RXFSIDL(n) ((n) * 4 + 1)
#define RXFEID8(n) ((n) * 4 + 2)
#define RXFEID0(n) ((n) * 4 + 3)
//mask registers--n=1,0
#define RXMSIDH(n) ((n) * 4 + 0x20)
#define RXMSIDL(n) ((n) * 4 + 0x21)
#define RXMEID8(n) ((n) * 4 + 0x22)
#define RXMEID0(n) ((n) * 4 + 0x23)
//mcp2515 model
#define CTRLMODEL_NORMAL	0x01
#define CTRLMODEL_LOOPBACK	0x02
#define CTRLMODEL_LISTENONLY	0x04
#define CTRLMODEL_SLEEP	0x08
#define CTRLMODEL_CONFIG	0x10

#define SET_BANDRATE	1
#define SET_MODEL	5//in ioctl, 2 will lead to system error(conflict)
#define SET_FILTER_REG	3
#define SET_MASK_REG		4


struct mcp2515_dev{
	struct cdev cdev;
	struct spi_device *spi;
	struct work_struct worker;

	wait_queue_head_t wqh;//read can with blocking
	u8 *spi_transfer_buf;

	struct can_frame spi_tx_buf[MCP2515_BUF_LEN];
	struct can_frame spi_rx_buf[MCP2515_BUF_LEN];

	u32 rxbin;//count, received data from can bus
	u32 rxbout;//count, read data from spi_rx_buf
	u32 txbin;
	u32 txbout;
	//not used below
	int bandrate;
	u8 can_model;
	u32 filter_id;
	u32 mask_id;
};

struct mutex m_lock;
//spinlock_t lock;
dev_t dev;
struct class *dev_class = NULL;
struct device *dev_device = NULL;

static int mcp2515_spi_trans(struct spi_device *spi, const u8 *txbuf, unsigned n_tx, u8 *rxbuf, unsigned n_rx)
{//there are something wrong with spi_write_then_read(), it will fail. But this work well.
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);
	struct spi_transfer t = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len = n_tx + n_rx,
		.cs_change = 0,
	};
	struct spi_message m;
	int ret;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if(ret)
	{
		printk("spi transfer failed!\n");
	}

	return ret;
}

static u8 mcp2515_read_reg(struct spi_device *spi, u8 reg)
{
	u8 txbuf[2] = {0};
	u8 rxbuf[3] = {0};
	u8 val;

	txbuf[0] = INSTRUCTION_READ;
	txbuf[1] = reg;
	mcp2515_spi_trans(spi, txbuf, 2, rxbuf, 1);
	val = rxbuf[2];

	return val;
}

static u8 mcp2515_read_status(struct spi_device *spi, u8 cmd)
{
	u8 txbuf[1] = {0};
	u8 rxbuf[2] = {0};
	u8 val;

	txbuf[0] = cmd;//INSTRUCTION_READ_STATUS or INSTRUCTION_READ_RXSTATUS
	mcp2515_spi_trans(spi, txbuf, 1, rxbuf, 1);
	val = rxbuf[1];

	return val;
}

static void mcp2515_write_reg(struct spi_device *spi, u8 reg, u8 val)
{
	u8 txbuf[3] = {0};
	int ret;

	txbuf[0] = INSTRUCTION_WRITE;
	txbuf[1] = reg;
	txbuf[2] = val;
	ret = spi_write(spi, txbuf, 3);
	if(ret < 0)
	{
		printk("Write register error!\n");
	}
}

static void mcp2515_bit_modify(struct spi_device *spi, u8 reg, u8 mask, u8 val)
{
	u8 txbuf[4] = {0};
	int ret;

	txbuf[0] = INSTRUCTION_BIT_MODIFY;
	txbuf[1] = reg;
	txbuf[2] = mask;
	txbuf[3] = val;
	ret = spi_write(spi, txbuf, 4);
	if(ret < 0)
	{
		printk("Bit modify error!\n");
	}
}

static int mcp2515_hw_reset(struct spi_device *spi)
{
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);

	u8 txbuf[1];
	int ret;
	u8 canstat = 0;
	u8 canctrl = 0;

	txbuf[0] = INSTRUCTION_RESET;
	ret = spi_write(spi, txbuf, 1);
	if(ret < 0)
	{
		printk("Reset error!\n");
	}

	mdelay(10);//delay for mcp2515 reset ready

	devp->can_model = CTRLMODEL_CONFIG;

	//check whether reset ready via reading the status of CANSTAT and CANCTRL registers
	canstat = mcp2515_read_reg(spi, CANSTAT) & 0xEE;
	canctrl = mcp2515_read_reg(spi, CANCTRL) & 0x17;// after reset, there might be some 'magic values' within CANCTRL bits

	return (canstat == 0x80 && canctrl == 0x07) ? 0 : 1;
}

static int mcp2515_hw_sleep(struct spi_device *spi)
{
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);

	mcp2515_write_reg(spi, CANCTRL, 0x20);//request go to sleep model

	devp->can_model = CTRLMODEL_SLEEP;

	return 0;
}

static int mcp2515_hw_wakeup(struct spi_device *spi)
{
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);

	mcp2515_bit_modify(spi, CANINTE, 0x40, 0x40);//enable CANINTE.WAKIE
	mcp2515_bit_modify(spi, CANINTF, 0x40, 0x40);//set CANINTF.WAKIF = 1
	mdelay(1);//delay for OSC ready work

	//reset WAKIF interrupt--no need, in irq handler, CANINTF.WAKIF will be clear to 0
//	mcp2515_bit_modify(spi, CANINTE, 0x40, 0x00);
//	mcp2515_bit_modify(spi, CANINTF, 0x40, 0x00);

	devp->can_model = CTRLMODEL_LISTENONLY;

	return 0;
}

static int mcp2515_set_bandrate(struct spi_device* spi, int bandrate)
{//make sure in CONFIG MODEL
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);

	u8 canctrl;
	u8 cnf1 = 0;
	u8 cnf2 = 0;
	u8 cnf3 = 0;

	switch(bandrate)
	{
		case 500:
			cnf1 = 0x00; cnf2 = 0x91; cnf3 = 0x01;
			break;
		case 250:
			cnf1 = 0x00; cnf2 = 0x96; cnf3 = 0x04;
			break;
		case 125:
			cnf1 = 0x00; cnf2 = 0x97; cnf3 = 0x03;
			break;
		default:
			return -1;

	}

	devp->bandrate = bandrate;

	canctrl = mcp2515_read_reg(spi, CANCTRL);//save CANCTRL value
	mcp2515_bit_modify(spi, CANCTRL, 0xe0, 0x80);//CONFIG MODEL

	mcp2515_write_reg(spi, CNF1, cnf1);
	mdelay(1);//in case spi busy??
	mcp2515_write_reg(spi, CNF2, cnf2);
	mdelay(1);
	mcp2515_write_reg(spi, CNF3, cnf3);
	mdelay(1);

	//resume canctrl
	mcp2515_write_reg(spi, CANCTRL, canctrl);

	return 0;
}

static void mcp2515_get_canid(struct spi_device *spi, canid_t *address, canid_t id, int ext)
{//ext = 1 extend id, ext = 0 standard id
	canid_t idbuf = 0;

	if(ext)
	{
		id &= 0x1fffffff;
		idbuf = ((id & 0x1fe00000) >> 21) | ((id & 0x00030000) >> 8) |(1 << 11) |
			((id & 0x001c0000) >> 5) | ((id & 0x0000ff00) << 8) | ((id & 0x000000ff) << 24);//high bits first to SH->SL->EH->EL
	}
	else
	{
		id &= 0x7ff;
		idbuf = (((id & 0x7f8) >> 3) | ((id & 0x7) << 13)) & 0xf7ff;//one bit must be 0 means standard ID
	}

	memcpy(address, &idbuf, sizeof(canid_t));
}

static void mcp2515_set_filter(struct spi_device *spi, canid_t filter_id)
{//make sure in CONFIG MODEL
	canid_t idbuf;
	u8 canctrl;
	int i;
	int j;

	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);
	devp->filter_id = filter_id;

	mcp2515_get_canid(spi, &idbuf, filter_id, 1);

	canctrl = mcp2515_read_reg(spi, CANCTRL);//save CANCTRL value
	mcp2515_bit_modify(spi, CANCTRL, 0xe0, 0x80);//CONFIG MODEL
	//set filter registers
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 4; j++)
		{
			mcp2515_write_reg(spi, RXFSIDH(i)+j, (idbuf >> (j*8)) & 0xff);
			mdelay(1);
		}
	}
	for(i = 4; i < 7; i++)
	{
		for(j = 0; j < 4; j++)
		{
			mcp2515_write_reg(spi, RXFSIDH(i)+j, (idbuf >> (j*8)) & 0xff);
			mdelay(1);
		}
	}

	//resume canctrl
	mcp2515_write_reg(spi, CANCTRL, canctrl);
}

static void mcp2515_set_mask(struct spi_device *spi, canid_t mask_id)
{//make sure in CONFIG MODEL
	canid_t idbuf;
	u8 canctrl;
	int i;
	int j;

	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);
	devp->mask_id = mask_id;

	mcp2515_get_canid(spi, &idbuf, mask_id, 1);
	idbuf &= 0xffe3ffff;//get bits needed for mask id

	canctrl = mcp2515_read_reg(spi, CANCTRL);//save CANCTRL value
	mcp2515_bit_modify(spi, CANCTRL, 0xe0, 0x80);//CONFIG MODEL
	//set mask registers
	for(i = 0; i < 2; i++)
	{
		for(j = 0; j < 4; j++)
		{
			mcp2515_write_reg(spi, RXMSIDH(i)+j, (idbuf >> (j*8)) & 0xff);
			mdelay(1);
		}
	}

	//resume canctrl
	mcp2515_write_reg(spi, CANCTRL, canctrl);
}

static int mcp2515_hw_setup(struct spi_device *spi)
{
	int ret;

	//bandrate, filter and mask registers default values set
	ret = mcp2515_set_bandrate(spi, 500);//default set bandrate 500kbps(8M OSC)
	if(ret)
	{
		return -1;
	}

	//set RXB0CTRL.BUKT(rollover) and RXBnCTRL.RXM(only receive Extend ID)
	mcp2515_bit_modify(spi, RXBCTRL(0), 0x64, 0x44);
	mcp2515_bit_modify(spi, RXBCTRL(1), 0x60, 0x40);

	/*
	**Note that after we reset, RXBnCTRL registers are set default 0.
	**Then RXBn will only accept datas(ID) that comfort filter and mask registers.
	*/
	mcp2515_set_mask(spi, 0x00000000);//if set mask to 0, means the bits should be masked and accept all type ID
	//mcp2515_set_filter(spi,u8 filter);//no need here

	return 0;
}

static int mcp2515_set_model(struct spi_device *spi, u8 ctrlmodel)
{
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);

	//enable interrupts
	mcp2515_write_reg(spi, CANINTE, 0x3f);//enable interrupts except MERRE and WAKIE

	if(ctrlmodel & CTRLMODEL_NORMAL)
	{
		mcp2515_write_reg(spi, CANCTRL, 0x00);
	}
	else if(ctrlmodel & CTRLMODEL_LOOPBACK)
	{
		mcp2515_write_reg(spi, CANCTRL, 0x40);
	}
	else if(ctrlmodel & CTRLMODEL_LISTENONLY)
	{
		//enable CANINTE.MERRE only in LISTENONLY MODEL
		mcp2515_bit_modify(spi, CANINTE, 0x80, 0x80);
		mcp2515_write_reg(spi, CANCTRL, 0x60);
		
	}
	else
	{
		printk("Configurable Models: normal, loopback and listen-only!\n");

		return -1;
	}
	mdelay(1);

	devp->can_model = ctrlmodel;
	
	return 0;
}

static void mcp2515_hw_rx(struct spi_device *spi, int pos)
{//receive data from RXB(n), once interrupt happend, recieve data
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);
	u8 txbuf[1];//spi instruction is needed
	u8 *rxbuf = devp->spi_transfer_buf;
	struct can_frame *frame = &devp->spi_rx_buf[devp->rxbin];
	//memset *txbuf
	memset(rxbuf, 0, SPI_TRANSFER_BUF_LEN);

	txbuf[0] = INSTRUCTION_READ_RXB(pos);
	mcp2515_spi_trans(spi, txbuf, 1, rxbuf, SPI_TRANSFER_BUF_LEN - 1);

	frame->can_id = (rxbuf[1] << 21) | ((rxbuf[2] & 0xe0) << 13) | ((rxbuf[2] & 0x03) << 16) | (rxbuf[3] << 8) |rxbuf[4];//in spi_rx_buf and spi_tx_buf, the 11/29 bits ID should be saved
	frame->can_dlc = rxbuf[5] & 0x0f;//the high bits include RTR(remote thansfer request) bit
	memcpy(frame->data, rxbuf + 6, frame->can_dlc);

	devp->rxbin ++;
	if(devp->rxbin >= MCP2515_BUF_LEN)
	{
		devp->rxbin = 0;
	}

	if(devp->rxbout != devp->rxbin)
	{
		wake_up_interruptible(&devp->wqh);//interruptible, wake up wait queue in mcp2515_read()
	}
}

static void mcp2515_hw_tx(struct spi_device *spi, int pos)
{//send data to TXB(n), once interrupt happend, only when there is data, can we send it.
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);
	u8 *txbuf = devp->spi_transfer_buf;
	struct can_frame *frame;
	//canid_t idbuf;
	int ret;
//	u32 txbout;

	//memset(&idbuf, 0, sizeof(canid_t));

	//lock on devp->txbout
//	mutex_lock(&m_lock);
//	txbout = devp->txbout;
//	if(devp->txbout != devp->txbin)
//	{
//		devp->txbout ++;
//		if(devp->txbout >= MCP2515_BUF_LEN)
//			devp->txbout = 0;
//	}
	/*
	**mutex lock could figure out the problem: send each data in turn in buf and each data will just be send once.
	**besides, this function will be called in interrupt handler, so there should be no messeges printed to ttys by printk.
	*/
	mutex_lock(&m_lock);
//	spin_lock(&lock);
	if(devp->txbout != devp->txbin)
	{
//		printk("txbout: %d, txbin: %d!\n", devp->txbout, devp->txbin);
		memset(txbuf, 0, SPI_TRANSFER_BUF_LEN);

		frame = &devp->spi_tx_buf[devp->txbout];
		devp->txbout ++;
		if(devp->txbout >= MCP2515_BUF_LEN)
		{
			devp->txbout = 0;
		}
		mutex_unlock(&m_lock);
//		spin_unlock(&lock);

		if(frame->can_dlc > 8)
		{
			frame->can_dlc = 8;
		}

		txbuf[0] = INSTRUCTION_LOAD_TXB(pos);
//		printk("ID: %08x", frame->can_id);
		mcp2515_get_canid(spi, (canid_t *)(txbuf+1), frame->can_id, 1);//error happened in txbuf+1!

//		printk("canid: ");
//		int i;
//		for(i=1; i<5; i++)
//		{
//			printk("%02x ", txbuf[i]);
//		}
//		printk("\n");

		//memcpy(txbuf + 1, &idbuf, 4);
		txbuf[5] = frame->can_dlc & 0x0f;//in TXBnDLC, the RTR bit should also be specified
		memcpy(txbuf + 6, frame->data, frame->can_dlc);//Note that can_dlc in struct can_frame just save 0~8

		ret = spi_write(spi, txbuf, SPI_TRANSFER_BUF_LEN);
		if(ret < 0)
		{
			printk("Load TXB failed!\n");
			return ;
		}
		printk("\n-----Send: %#x\n", frame->data[0]);

//		devp->txbout ++;
//		if(devp->txbout >= MCP2515_BUF_LEN)
//		{
//			devp->txbout = 0;
//		}

		//request RXB(pos) transfer
		mcp2515_bit_modify(spi, TXBCTRL(pos), 0x08, 0x08);
	}
	else
		mutex_unlock(&m_lock);
//		spin_unlock(&lock);
}

static int mcp2515_open(struct inode *inode, struct file *filp)
{
	//in inode, cdev adress was saved
	printk("MCP2515 Driver open!\n");

	int ret = 0;
	struct mcp2515_dev *devp = NULL;
	devp = container_of(inode->i_cdev, struct mcp2515_dev, cdev);
	filp->private_data = devp;

	devp->txbin = 0;
	devp->txbout = 0;
	devp->rxbin = 0;
	devp->rxbout = 0;
	/*
	**上电或者复位时器件自动进入配置模式，所有错误计数器都被清零
	**配置模式下才能对如下寄存器进行修改：
	**CNF1、CNF2、CNF3、TXRTSCTRL(引脚控制和状态寄存器)、验收过滤/屏蔽寄存器
	*/

	//do some init works, such as config registers
	struct spi_device *spi = devp->spi;
	mcp2515_hw_wakeup(spi);//if mcp2515 is in SLEEP MODEL, then registers just can be read not wrote via spi.
	ret = mcp2515_hw_reset(spi);//to CONFIG MODEL
	if(ret)
	{
		printk("reset hw failed when open dev file!\n");
		mcp2515_hw_sleep(spi);

		goto error_open;
	}

	//set default value of bandrate(500kbps), filter register and mask register(all 1 means recv all type ID).
	ret = mcp2515_hw_setup(spi);
	if(ret)
	{
		printk("setup mcp2515 failed!\n");
		mcp2515_hw_sleep(spi);

		goto error_open;
	}
	ret = mcp2515_set_model(spi, CTRLMODEL_NORMAL);
	if(ret)
	{
		printk("set normal model failed!\n");
		mcp2515_hw_sleep(spi);

		goto error_open;
	}

	printk("kernel open success!!\n");

error_open:
	return ret;
}

static ssize_t mcp2515_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
//	printk("Enter mcp2515 read!\n");

	struct mcp2515_dev *devp = filp->private_data;
//	struct spi_device *spi = devp->spi;
	struct can_frame *frame;
	int ret;

	if(count != sizeof(struct can_frame))
		return -EINVAL;

//	printk("rxbout: %d, rxbin: %d!\n", devp->rxbout, devp->rxbin);

	while(devp->rxbout == devp->rxbin)
	{//while and if are both ok here
		if(filp->f_flags & O_NONBLOCK)//cannot read with non-block way because we will need to sleep wait.
		{
			printk("file flag is O_NONBLOCK!!\n");
			return -EINVAL;//invalid argument
		}
		if(wait_event_interruptible(devp->wqh, devp->rxbout != devp->rxbin))
			return -ERESTARTSYS;
	}

	frame = &devp->spi_rx_buf[devp->rxbout];
	ret = copy_to_user(buf, frame, count);//success return 0, fail return the size of failed data.
	if(ret)
		return -EFAULT;//bad address

	devp->rxbout ++;
	if(devp->rxbout >= MCP2515_BUF_LEN)
		devp->rxbout = 0;

//	printk("kernel mcp2515 read successfully!\n");

	return frame->can_dlc & 0x0f;//return the size of datas(not include ID or DLC)
}

static ssize_t mcp2515_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
//	printk("Enter mcp2515 write!\n");

	struct mcp2515_dev *devp = filp->private_data;
	struct spi_device *spi = devp->spi;
	struct can_frame *frame;
	int ret = 0;
	u8 rxtx_status;

	if(count != sizeof(struct can_frame))
		return -EINVAL;

	frame = &devp->spi_tx_buf[devp->txbin];
	ret = copy_from_user(frame, buf, count);
	if(ret)
		return -EFAULT;

	devp->txbin ++;
	if(devp->txbin >= MCP2515_BUF_LEN)
		devp->txbin = 0;

	//find which TXB is empty, then use it or else wait for TXBINTF interrupt to transmit the can_frame data in spi_tx_buf
	rxtx_status = mcp2515_read_status(spi, INSTRUCTION_READ_STATUS);

	if(!(rxtx_status & 0x04))
		mcp2515_hw_tx(spi, 0);
	if(!(rxtx_status & 0x10))//not else if, because we want to transmit more datas with less time. Maybe
		mcp2515_hw_tx(spi, 1);
	if(!(rxtx_status & 0x40))
		mcp2515_hw_tx(spi, 2);

//	printk("kernel mcp2515 write successfully!\n");

	return frame->can_dlc & 0x0f;
}

static int mcp2515_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	printk("Enter mcp2515 ioctl!\n");

	struct mcp2515_dev *devp = filp->private_data;
	struct spi_device *spi = devp->spi;
	int ret = 0;

	switch(cmd){
	case SET_BANDRATE:
		//set bandrate
		ret = mcp2515_set_bandrate(spi, arg);//arg should be in (500, 250, 125)
		break;
	case SET_MODEL:
		//set model
		ret = mcp2515_set_model(spi, arg);//arg should be in (0x01-NORMAL, 0x02-LOOPBACK, 0x04-LISTENONLY)
		break;
	case SET_FILTER_REG:
		//set filter registers
		mcp2515_set_filter(spi, (canid_t)arg);//u64->u32
		break;
	case SET_MASK_REG:
		//set mask registers
		mcp2515_set_mask(spi, (canid_t)arg);
		break;
	default:
		//cannot get info
		break;
	}

	return ret;
}

static int mcp2515_release(struct inode *inode, struct file *filp)
{
	printk("MCP2515 Driver Release(in SLEEP MODEL)!\n");

	struct mcp2515_dev *devp = NULL;
	devp = container_of(inode->i_cdev, struct mcp2515_dev, cdev);
	struct spi_device *spi = devp->spi;

	memset(devp->spi_tx_buf, 0, MCP2515_BUF_LEN);
	memset(devp->spi_rx_buf, 0, MCP2515_BUF_LEN);

	//clear pending interrupts
	mcp2515_write_reg(spi, CANINTE, 0x00);
	mcp2515_write_reg(spi, CANINTF, 0x00);

	//go to SLEEP MODEL
	mcp2515_hw_sleep(spi);

	return 0;
}

static void mcp2515_work_func(struct work_struct *work)
{
	// Param work: address point to the member worker of mcp2515_dev
	// through the address of *work, get mcp2515_dev
	struct mcp2515_dev *devp = container_of(work, struct mcp2515_dev, worker);
	struct spi_device *spi = devp->spi;
	u8 canintf;

	//handle CANINTF
	while(1)
	{//until INT reset
		canintf = mcp2515_read_reg(spi, CANINTF);
//		printk("\n-------INTF: %#x\n", canintf);
//		mdelay(1000);
		if(!canintf)
		{
			break;//no interrupts, reset INT
		}

		if(canintf & 0x80)//MERRF
			/*这里一旦复位，则无法正常工作，需要mcp2515_hw_setup后再设定工作模式
			**??何时出现MERRF,以及应该怎样处理这种情况??
			**--总线错误时，该设备下线，但是不影响总线上其他设备正常工作
			**--理论上处理方式是根据错误标志找出错误位置，解决后再重启设备(手动或延时启动)
			**延时重启的目地是为了防止出错设备一直重启发送主动错误从而会占用总线带宽
			*/
			mcp2515_hw_reset(spi);//in NORMAL MODEL, we do not know how to handle MERRF interrupt
		if(canintf & 0x20)//ERRIF
			mcp2515_write_reg(spi, EFLG, 0x00);//do nothing but set 0 to EFLG register
//		if(canintf & 0x40)//WAKIF
//			printk("why CANINTF.WAKIF set 1!\n");
		if(canintf & 0x04)//TX0IF
			mcp2515_hw_tx(spi, 0);
		if(canintf & 0x08)//TX1IF
			mcp2515_hw_tx(spi, 1);
		if(canintf & 0x10)//TX2IF
			mcp2515_hw_tx(spi, 2);
		if(canintf & 0x01)//RX0IF
			mcp2515_hw_rx(spi, 0);//in mcp2515+hw_rx, we try to wake up wait queue if exist
		if(canintf & 0x02)//RX1IF
			mcp2515_hw_rx(spi, 1);

		//mcp2515_bit_modify(spi, CANINTF, canintf, canintf);//here is the bug!! Why set the bit to 1 instead of 0??
		mcp2515_bit_modify(spi, CANINTF, canintf, 0x00);
	}
//	printk("interrupt handle finished!\n");
}

static irqreturn_t mcp2515_irq_handler(int irq, void *dev_id)
{
	//get worker
	struct mcp2515_dev *devp = dev_id;

	schedule_work(&devp->worker);

	return IRQ_HANDLED;
}

struct file_operations mcp2515_fops = {
	.owner = THIS_MODULE,
	.open = mcp2515_open,
	.read = mcp2515_read,
	.write = mcp2515_write,
	.ioctl = mcp2515_ioctl,
	.release = mcp2515_release,
};

static int __devinit mcp2515_probe(struct spi_device *spi)
{
	/*
	** Param spi: refer to the matched spi_device data
	*/
	int ret = 0;//return value
	struct mcp2515_dev *devp = NULL;

	//allocate device number
	ret = alloc_chrdev_region(&dev,0,1,"MCP2515");
	if(ret < 0)
	{
		printk("alloc_chrdev_region failed!\n");

		goto failure_alloc_chrdev;
	}
	
	//allocate cdev space
	devp = kzalloc(sizeof(struct mcp2515_dev), GFP_KERNEL);//init 0
	if(IS_ERR(devp))
	{
		printk("kzalloc devp failed!\n");
		ret = PTR_ERR(devp);

		goto failure_kzalloc_devp;
	}

	//spi->dev.p->driver_data = devp
	//then we can use *devp data through spi
	dev_set_drvdata(&spi->dev, devp);
	devp->spi = spi;
	devp->rxbin = 0;
	devp->rxbout = 0;
	devp->txbin = 0;
	devp->txbout = 0;
	devp->bandrate = 0;
	devp->can_model = 0;
	devp->filter_id = 0;
	devp->mask_id = 0;
	devp->spi_transfer_buf = kzalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
	if(IS_ERR(devp->spi_transfer_buf))
	{
		printk("kzalloc spi transfer buf failed!\n");
		ret = PTR_ERR(devp->spi_transfer_buf);

		goto failuer_kzalloc_buf;
	}

	mutex_init(&m_lock);
//	spin_lock_init(&lock);

	//init worker (for the bottom half of interrupt)--usage: init then schedule
	INIT_WORK(&devp->worker, mcp2515_work_func);

	//irq
	ret = request_irq(IRQ_EINT(20), mcp2515_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "can_read_interrupt", devp);
	if(ret<0)
	{
		printk("request irq failed!\n");

		goto failure_request_irq;
	}

	//init wait queue head (for read block)--usage: init then schedule
	init_waitqueue_head(&devp->wqh);
	
	//initial cdev and add it to kernel, each device needs a cdev struct
	cdev_init(&devp->cdev, &mcp2515_fops);
	devp->cdev.owner = THIS_MODULE;
	ret = cdev_add(&devp->cdev, dev, 1);//link dev and cdev
	if(ret < 0)
	{
		printk("cdev_add failed!\n");

		goto failure_cdev_add;
	}
	
	//create dev node file--class & device
	dev_class = class_create(THIS_MODULE, "MCP2515");//new sys.class.MCP2515 directory
	if(IS_ERR(dev_class))
	{
		printk("dev class create failed!\n");
		ret = PTR_ERR(dev_class);

		goto failure_dev_class;
	}

	// new sys.class.MCP2515.mcp2515-0 file and new dev.mcp2515-0 file
	dev_device = device_create(dev_class, NULL, dev, NULL, "mcp2515-0");//link dev_class, dev and cdev
	if(IS_ERR(dev_device))
	{
		printk("device create failed!\n");
		ret = PTR_ERR(dev_device);

		goto failure_dev_device;
	}

	//firstly, we should set up spi bus
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	//reset mcp2515
	ret = mcp2515_hw_reset(spi);
	if(ret)
	{
		printk("reset hw failed!\n");

		goto failure_reset_hw;
	}

	//make mcp2515 in SLEEP MODEL
	mcp2515_hw_sleep(spi);

	printk("MCP2515: CAN Device Driver Success(SLEEP MODEL)!\n");
	return 0;

failure_reset_hw:
	device_destroy(dev_class, dev);
failure_dev_device:
	class_destroy(dev_class);
failure_dev_class:
	cdev_del(&devp->cdev);
failure_cdev_add:
	free_irq(IRQ_EINT(20), devp);
failure_request_irq:
	kfree(devp->spi_transfer_buf);
failuer_kzalloc_buf:
	kfree(devp);
failure_kzalloc_devp:
	unregister_chrdev_region(dev, 1);
failure_alloc_chrdev:
	return ret;
}

static int mcp2515_suspend(struct spi_device *spi, pm_message_t mesg)
{
	mcp2515_hw_sleep(spi);

	return 0;
}

static int mcp2515_resume(struct spi_device *spi)
{
	mcp2515_hw_wakeup(spi);

	return 0;
}

static int __devexit mcp2515_remove(struct spi_device *spi)
{
	struct mcp2515_dev *devp = dev_get_drvdata(&spi->dev);

	device_destroy(dev_class, dev);
	class_destroy(dev_class);
	cdev_del(&devp->cdev);
	kfree(devp);
	free_irq(IRQ_EINT(20), devp);
	kfree(devp->spi_transfer_buf);
	unregister_chrdev_region(dev, 1);

	printk("MCP2515 Driver removed!\n");
	return 0;
}

static struct spi_device_id mcp2515_id[] = {
	{"mcp2515",	0},//name match and on spi0
	{},//EOF
};

struct spi_driver mcp2515_driver = {
	.driver = {
		.name = "MCP2515",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},

	.id_table = mcp2515_id,
	.probe = mcp2515_probe,
	.suspend = mcp2515_suspend,
	.resume = mcp2515_resume,
	.remove = __devexit_p(mcp2515_remove),
};

static int __init mcp2515_init(void)
{
	return spi_register_driver(&mcp2515_driver);
}

static void __exit mcp2515_exit(void)
{
	spi_unregister_driver(&mcp2515_driver);
}

module_init(mcp2515_init);
module_exit(mcp2515_exit);
