
KERNELDIR 	?=/root/android-kernel-samsung-dev/

obj-m 		+= mcp2515-3.o

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	@rm -f *.o *.ko *.order *.sy* *.mod*
