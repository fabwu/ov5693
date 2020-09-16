# Surface Cam

## Installation

1. Apply patches from `patches/` to kernel tree (`git am <patch>`).

2. Set `CONFIG_INIT_STACK_NONE` to `no automatic initialization (weakest)` (only arch distros)

3. Set `VIDEO_ATOMISP_OV5693` to `n`

4. Boot into new kernel

5. Load camera driver for ov5693

```
cd ov5693/
make
sudo insmod ov5693.ko
```

6. Load cio2-bridge

```
cd cio2-bridge
make
sudo insmod cio2-bridge.ko
```

7. Test camera

```
sudo cam -l
sudo qcam
```

## References

- https://github.com/jhand2/surface-camera
- https://github.com/kitakar5525/surface-ipu3-cameras
- https://github.com/djrscally/miix-510-cameras
