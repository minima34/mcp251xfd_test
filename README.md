This is module for testing the spi connection to mcp251xfd family via writing and reading the OSC register of the device.
Tested on linux v4.9.170 @ allwinner a133 board with mcp2518fd

Device tree confguration is the same as the mcp251xfd driver

`
~$ git clone https://github.com/minima34/mcp251xfd_test.git
~$ cp -r mcp251xfd_test/drivers/mcptest linux-source/drivers/
`

You must add this line to linux-source/drivers/Kconfig
`
source "drivers/mcptest/Kconfig"
`

And this line to linux-source/drivers/Makefile
`
obj-y	+= mcptest/
`

Add this line to linux-source/.config
`
CONFIG_CAN_MCPTEST=m
`

Build your new kernel and copy the module to your destination rootfs:
`
$ copy drivers/mcptest/mcptest.ko /mnt/sdcard_rootfs/root
`

And run these:
`
$ sudo -i
# dmesg -C && insmod /root/mcptest.ko && dmesg
`