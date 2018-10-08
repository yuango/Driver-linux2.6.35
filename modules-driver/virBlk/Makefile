
KERNELDIR 	?=/root/android-kernel-samsung-dev/

obj-m 		+= virBlkDrv_0709.o

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	@rm -f *.o *.ko *.order *.sy* *.mod*
