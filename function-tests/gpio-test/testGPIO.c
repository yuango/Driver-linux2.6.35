#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main()
{
	int fd = 0;
	int len = 0;
	int port, value;
	
	fd = open("/dev/rlcs_gpio_device", O_WRONLY);
	if(fd < 0)
	{
		printf("open char device file failed!\n");

		return -1;
	}

	while(1)
	{
		printf("Input command: ");
		char buf[10] = {0};
		scanf(" %[^\n]", buf);

		if(!strcmp(buf, "quit"))
		{
			printf("Bye!\n");
			break;
		}

		len = strlen(buf);
		if(len<3 ||len>4)
		{
			printf("Invalid command! Please input again\n----example: port value(19 1)\n");
		}
		else if(len(buf) == 3)
		{
			if(buf[0]<'0' || buf[0]>'9' || buf[2]<'0' || buf[2]>'9' || buf[1]!=' ')
			{
				printf("Invalid command! Please input again\n----example: port value(19 1)\n");
			}
			else
			{
				port = buf[0] - 48;//invalid port? 0~19
				value = buf[2] - 48;//invalid value? 0 and 1
				ioctl(fd, value, port);
			}
		}
		else
		{
			if(buf[0]<'0' || buf[0]>'9' || buf[1]<'0' || buf[1]>'9' || buf[2]!=' ' || buf[3]<'0' || buf[3]>'9')
			{
				printf("Invalid command! Please input again\n----example: port value(19 1)\n");
			}
			else
			{
				port = (buf[0] - 48) * 10 + buf[1] - 48;
				value = buf[3] - 48;
				ioctl(fd, value, port);
			}
		}
	}

	close(fd);

	return 0;
}
