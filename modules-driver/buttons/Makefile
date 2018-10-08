
KERNELDIR 	?=/root/android-kernel-samsung-dev/

obj-m 		+= buttons_dev_0702.o
obj-m 		+= buttons_drv_0702.o
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	@rm -f *.o *.ko *.order *.sy* *.mod*
