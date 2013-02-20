TARGET = mzc_usb
OBJS = mzc_usb.o
MDIR = drivers/misc

CURRENT = $(shell uname -r)
KDIR = /lib/modules/$(CURRENT)/build
PWD = $(shell pwd)
DEST = /lib/modules/$(CURRENT)/kernel/$(MDIR)

obj-m := $(TARGET).o

default:
	make -C $(KDIR) SUBDIRS=$(PWD) modules

$(TARGET).o: $(OBJS)
	$(LD) $(LD_RFLAG) -r -o $@ $(OBJS)

clean:
	-rm -f *.o *.ko .*.cmd .*.flags *.mod.c *.symvers

-include $(KDIR)/Rules.make
