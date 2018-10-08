#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/*
*�����ʽ:
*./a.out w 10 0x55
*./a.out r 20
**/

void print_usage(char *file)
{
	printf("%s r addr num\n",file);
	printf("%s w addr val1 val2\n",file);
}

int main(int argc,char *argv[])
{
	int fd;
	int size;
	int i;
	unsigned char buf[5];

	if(argc!=4 && argc!=5)
	{
		print_usage(argv[0]);

		return -1;
	}

	fd = open("/dev/at24cxx0",O_RDWR);

	if(fd<0)
	{
		printf("open failed!\n");

		return -1;
	}

	printf("open success!\n");

	if((argc == 4) && (strcmp(argv[1],"r") == 0))
	{
		/*��Ҫ�Ƚ��ַ���ת��ʵ����ֵ*/
		buf[0] = strtoul(argv[2],NULL,0);
		size = strtoul(argv[3],NULL,0);
		if(size>5)
			size = 5;
		/*���ں�����ȡ��buf[0]�д�ŵ�ƫ�Ƶ�ַ��������
		*Ȼ���ٽ��Ӵ��豸���������ݴ浽buf��
		*/
		read(fd,buf,size);
		for(i=0;i<size;i++)
		{
			printf("data%d:%d 0x%2x\n",i,buf[i],buf[i]);
		}
	}
	else if((argc == 5) && (strcmp(argv[1],"w") == 0))
	{
		buf[0] = strtoul(argv[2],NULL,0);
		buf[1] = strtoul(argv[3],NULL,0);
		buf[2] = strtoul(argv[4],NULL,0);
		write(fd,buf,3);
	}
	else
	{
		print_usage(argv[0]);

		return -1;
	}

	return 0;
}
