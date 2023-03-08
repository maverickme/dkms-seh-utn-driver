DEBFLAGS = -O2

ifndef $(LDDINC)                                                                
  LDDINC:=$(shell pwd)
endif

ccflags-y += $(DEBFLAGS) -DHAVE_HCD_H
ccflags-y += -I$(LDDINC)

TARGET = seh_vhcd

#ifneq ($(KERNELRELEASE),)

seh_vhcd-objs := vhci_sd_boost.o vhci_hcd.o vhci_event.o vhci_char_dev.o  vhci_to_service.o vhci_xmit.o sha1.o

obj-m	:= $(TARGET).o

#else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) LDDINC=$(PWD) EXTRA_CFLAGS="-DHAVE_HCD_H" modules
	@cp seh_vhcd.ko seh_vhcd-$(KVER)_$(shell uname -m).ko

#endif


install:
	install -d $(INSTALLDIR)
	install -c $(TARGET).o $(INSTALLDIR)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers *.order *.a *.mod *.dwo
	find ./legacy -name '*.o' -exec rm {} \;
	find ./legacy -name '*.dwo' -exec rm {} \;
	find ./legacy -name '*.cmd' -exec rm {} \;

depend .depend dep:
	$(CC) -M *.c > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif
