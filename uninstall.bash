#!/bin/bash

DRIVER_NAME=hc16_device
#DRIVER_NAME=hc16_test

[ ! -z "$(lsmod | grep hc16)" ] && rmmod ${DRIVER_NAME}
rm -vf /lib/modules/*/extra/${DRIVER_NAME}.ko*
rm -vf /etc/modprobe.d/99-hc16-device.conf
#rm -vf /etc/udev/rules.d/99-hc16-device.rules
udevadm control --reload
depmod -a
