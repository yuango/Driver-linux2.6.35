#include <stdio.h>
#include <stdlib.h>//exit
#include <string.h>
#include <pthread.h>
#include <unistd.h>//alarm
#include <sys/ioctl.h>
#include <sys/socket.h>//something wrong without socket.h
#include <linux/can.h>
#include <malloc.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>//from man 2 open
//for time
#include <signal.h>
#include <time.h>

struct Mcp2515_data{
	int fd;
	struct can_frame frame_read;
};

int g_fd;
unsigned int send_count;


void *func_read(void *p)
{
	struct Mcp2515_data *pr = p;
	struct can_frame *frame = &pr->frame_read;

	printf("Start reading...\n");
	printf("fd=%d\n", pr->fd);

	unsigned int count = 0;
	while(1)
	{
//		sleep(1);
		memset(frame, 0, sizeof(struct can_frame));

		int n = read(pr->fd, frame, sizeof(struct can_frame));
		if(n < 0)
		{
			perror("read can bus data failed!\n");
			break;
		}

		printf("%u  RECV: ID=%08x,data length=%d: %#x\n", ++count, frame->can_id, frame->can_dlc, frame->data[0]);//定时1秒连续发送数据包(13bytes)，理论上可以接收到大约4k～5k个包每秒
		usleep(1000);
//		int i;
//		for(i=0; i<frame->can_dlc; i++)
//		{
//			printf("%#x ", frame->data[i]);
//		}
//		printf("\n");
	}

	printf("exit read can bus thread!\n");
	close(pr->fd);
	free(pr);
	exit(-3);//in case no return value alarm!
}

void func_exit()
{
	printf("---------------------send count: %d\n", send_count);
	sleep(5);
	close(g_fd);
	exit(0);//强行退出进程
}

void func_write(int fd, struct can_frame *frame)
{
	while(1)
	{
		usleep(100000);
		printf("Please input data to can bus(max size: 8 bytes):");

		char buf[16] = {0};
		scanf(" %[^\n]", buf);//从屏幕输入字符串到换行结束
		if(!strcmp(buf, "quit"))
		{
			printf("Quit all thread!\n");
			break;
		}
		if(!strcmp(buf, "set loopback"))
		{
			if(ioctl(fd, 5, 2))
			{
				perror("set loopback model failed!\n");
			}
			continue;
		}
		if(!strcmp(buf, "set normal"))
		{
			if(ioctl(fd, 5, 1))
				perror("set normal model failed!\n");
			continue;
		}
		if(!strcmp(buf, "set mask filter"))
		{//can_id&mask == filter&mask
			if(ioctl(fd, 4, 0x00000001))
				perror("set mask register failed!\n");

			if(ioctl(fd, 3, 0x1234567e))
				perror("set filter register failed!\n");

			printf("after set mask filter, you need to set good canid to test ID whether good.\n");
			continue;
		}
		if(!strcmp(buf, "set good canid"))
		{
			frame->can_id = 0x1030507e;
			printf("after set mask and filter register, the 0x1234567f ID will be filtered but 0x1030507e will pass!\n");
			continue;
		}

		if(!strcmp(buf, "test"))
		{
			int j;
			unsigned int count = 0;
			for(j=0; j<frame->can_dlc; j++)
			{
				frame->data[j] = (1<<j)&0xff;//should be 0x80 40 20 10 08 04 02 01, 8bytes data
			}
			//这里应该定时执行
			signal(SIGALRM, func_exit);
			alarm(1);//定时一秒发送数据，测试发送速度和接收的数据是否丢失
			while(1)
			{
				if(write(fd, frame, sizeof(struct can_frame)) < 0)
				{
					perror("test error: write data to can bus failed!\n");
					break;
				}
				frame->data[0] = (frame->data[0]+1)&0xff;
				++send_count;
//				usleep(1000);
			}
			break;
		}

		int i;
		for(i=0; i<frame->can_dlc; i++)
		{
			frame->data[i] = (unsigned char)buf[i];
//在printf函数打印格式中，
//%X是以十六进制打印，并且a~f打印出来的是大写的A、B、C、D、E、F
//如果是%#X，则会在打印的十六进制结果前面加上0X
			printf("Send Num %d bytes to can bus: %#x\n", i, frame->data[i]);
		}

		int nw = write(fd, frame, sizeof(struct can_frame));
		if(nw < 0)
		{
			perror("write data to can bus failed!\n");
			break;
		}
	}
}

int main(int argc, char *argv[])
{
	int fd;
	int ret;
	struct Mcp2515_data *p = NULL;
	struct can_frame frame_write;

	if(argc != 1)
	{
		printf("command error!\n");
		return -1;
	}

	p = malloc(sizeof(struct Mcp2515_data));
	if(p == NULL)
	{
		perror("malloc failed!\n");

		return -1;
	}

	fd = open("/dev/mcp2515-0", O_RDWR);
	if(fd < 0)
	{
		perror("open failed!\n");

		return -1;
	}
	printf("open success! fd=%d\n", fd);//once open success, mcp2515 should be set default(NORMAL MODEL and recv any extend ID)

	printf("Start a thread to receive data from can bus!\n");
	pthread_t id;
	g_fd = fd;
	p->fd = fd;
	ret = pthread_create(&id, NULL, func_read, p);
	if(ret)
	{
		printf("create read thread failed!\n");

		return ret;
	}

	//main thread will be used to send message to can bus
	frame_write.can_id = 0x1234567f;
	frame_write.can_dlc = 8;
	func_write(fd, &frame_write);

	close(fd);
	return 0;
}
