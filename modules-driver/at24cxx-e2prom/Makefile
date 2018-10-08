
KERNELDIR 	?=/root/android-kernel-samsung-dev/

obj-m 		+= at24cxx_0707.o

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	@rm -f *.o *.ko *.order *.sy* *.mod*
