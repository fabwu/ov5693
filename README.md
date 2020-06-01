# OmniVision ov5693

Experimental driver for Surface Book 2.

## Install

Make sure you have a linux kernel somwhere. Then run

```
make
insmod ov5693.ko
```

## Debug

Enable debugging with the following command

```
echo 'module ov5693 +p' > /sys/kernel/debug/dynamic_debug/control
```

Check `dmesg` for debug output.
