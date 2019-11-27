obj-m += px200.o
all:
	sudo make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c
