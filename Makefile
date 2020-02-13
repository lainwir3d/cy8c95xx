ccflags-y :=  -O2 -std=gnu99 -Wno-declaration-after-statement

obj-m += gpio-cy8c95xx.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	cp gpio-cy8c95xx.ko /lib/modules/$(shell uname -r)/

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

