tbs6812-objs	:= tbsecp3-core.o tbsecp3-cards.o tbsecp3-i2c.o tbsecp3-dma.o tbsecp3-dvb.o tbsecp3-asi.o
cxd2857-objs	:= cxd2857er.o
# tbsecp3-ca.o
obj-m	:= tbs6812.o cxd2857.o

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) tbs6812.ko cxd2857.ko

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
