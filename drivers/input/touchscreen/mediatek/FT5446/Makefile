include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/FT5446/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/FT5446/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/FT5446/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y += focaltech_ctl.o
obj-y += focaltech_driver.o
obj-y += focaltech_ex_fun.o
obj-y += ft_gesture_lib.a
obj-y += mcap_5x46_test_lib_64bit_lib.a