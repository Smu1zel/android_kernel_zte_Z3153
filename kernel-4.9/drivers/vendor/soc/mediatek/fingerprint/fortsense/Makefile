

######################### fortsense driver config begin #########################

ifdef CONFIG_VENDOR_SOC_MTK_COMPILE
#just for mtk platform head file, kernel version larger than kernle 4.0
#mtk平台，内核版本大于4.0参考头文件
#MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/spi
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include

#just for beanpod v2 compatible
#豆荚 V2 TEE平台，兼容头文件
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/V1.0/tz_vfs

#甁砵 TEE 平台，兼容头文件
#subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/core
#subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/include/

# Android L
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
endif

#fortsense driver file
# 如果没有移植 Kconfig 导致驱动没编译到，请打开下面的 obj-y 并屏蔽 obj-$(CONFIG_FINGERPRINT_FORTSENSE)
#obj-y                                     += fortsense_fp.o
obj-$(CONFIG_FINGERPRINT_FORTSENSE)       += fortsense_fp.o
fortsense_fp-objs                           := sf_ctl.o sf_hw.o

########################## fortsense driver config end ##########################

