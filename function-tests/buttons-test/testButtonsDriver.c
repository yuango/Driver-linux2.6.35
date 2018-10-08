#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/input.h>

struct inputevent
{
	struct timeval time;
	__u16 type;
	__u16 code;
	__u32 value;
};

int fd;

int main()
{
	struct inputevent ev_key;
	fd = open("/dev/input/event3",O_RDWR);

	while(1)
	{
		read(fd,&ev_key,sizeof(ev_key));

		if(ev_key.type == EV_KEY)
		{
			printf("time:%ld,%ld,type:%d code:%d value:%d\n",
					ev_key.time.tv_sec,ev_key.time.tv_usec,
					ev_key.type,ev_key.code,ev_key.value);
		}
	}

	return 0;
}
