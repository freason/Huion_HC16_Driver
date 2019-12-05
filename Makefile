#obj-m := hc16_test.o
obj-m := hc16_device.o
KVERSION := $(shell uname -r)
KDIR := /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)
modules modules_install clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) $@
install: modules_install
	install -D -m 0644 99-hc16-device.conf /etc/modprobe.d/99-hc16-device.conf
	#install -D -m 0644 99-hc16-device.rules /etc/udev/rules.d/99-hc16-device.rules
	udevadm control --reload
	depmod -a
uninstall:
	./uninstall.bash
