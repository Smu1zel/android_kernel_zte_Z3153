
# FortSense 2-1: Please edit kernel-3.18/drivers/input/fingerprint/Kconfig to add FINGERPRINT_FORTSENSE option.
config FINGERPRINT_FORTSENSE
	tristate "FortSense Fingerprint"
	default y
	help
	  If you say Y to this option, support will be included for 
	  the FortSense's fingerprint sensor. This driver supports 
	  both REE and TEE. If in REE, CONFIG_SPI_SPIDEV must be set 
	  to use the standard 'spidev' driver.
	
	  This driver can also be built as a module. If so, the module
	  will be called 'fortsense_driver_all_in_one'.


# FortSense 2-2: Please edit kernel-3.18/drivers/input/fingerprint/Makefile to add FINGERPRINT_FORTSENSE option.
obj-$(CONFIG_FINGERPRINT_FORTSENSE) += fortsense_driver_all_in_one/

