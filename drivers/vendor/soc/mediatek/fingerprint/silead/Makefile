#
# Silead in MTK
#
ifdef CONFIG_VENDOR_SOC_MTK_COMPILE
	ccflags-y += -I$(srctree)/drivers/spi
	ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
	ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
	ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
endif


obj-$(CONFIG_MTK_FINGERPRINT_SILEAD)	+= silead_fp.o silead_fp_platform.o
