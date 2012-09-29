# arch/arm/mach-exynos4/Makefile
#
# Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
#		http://www.samsung.com/
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

# Core support for EXYNOS4 system

obj-$(CONFIG_CPU_EXYNOS4210)	+= cpu.o init.o clock.o irq-combiner.o
<<<<<<< HEAD
obj-$(CONFIG_CPU_EXYNOS4210)	+= setup-i2c0.o irq-eint.o dma.o
obj-$(CONFIG_PM)		+= pm.o sleep.o
obj-$(CONFIG_CPU_FREQ)		+= cpufreq.o
=======
obj-$(CONFIG_CPU_EXYNOS4210)	+= setup-i2c0.o irq-eint.o dma.o pmu.o
obj-$(CONFIG_PM)		+= pm.o sleep.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
obj-$(CONFIG_CPU_IDLE)		+= cpuidle.o

obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o

<<<<<<< HEAD
ifeq ($(CONFIG_EXYNOS4_MCT),y)
obj-y				+= mct.o
else
obj-y				+= time.o
obj-$(CONFIG_LOCAL_TIMERS)	+= localtimer.o
endif
=======
obj-$(CONFIG_EXYNOS4_MCT)	+= mct.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o

# machine support

obj-$(CONFIG_MACH_SMDKC210)		+= mach-smdkc210.o
obj-$(CONFIG_MACH_SMDKV310)		+= mach-smdkv310.o
obj-$(CONFIG_MACH_ARMLEX4210)		+= mach-armlex4210.o
obj-$(CONFIG_MACH_UNIVERSAL_C210)	+= mach-universal_c210.o
obj-$(CONFIG_MACH_NURI)			+= mach-nuri.o

# device support

obj-y					+= dev-audio.o
obj-$(CONFIG_EXYNOS4_DEV_AHCI)		+= dev-ahci.o
obj-$(CONFIG_EXYNOS4_DEV_PD)		+= dev-pd.o
obj-$(CONFIG_EXYNOS4_DEV_SYSMMU)	+= dev-sysmmu.o
<<<<<<< HEAD

obj-$(CONFIG_EXYNOS4_SETUP_FIMC)	+= setup-fimc.o
=======
obj-$(CONFIG_EXYNOS4_DEV_DWMCI)	+= dev-dwmci.o

obj-$(CONFIG_EXYNOS4_SETUP_FIMC)	+= setup-fimc.o
obj-$(CONFIG_EXYNOS4_SETUP_FIMD0)	+= setup-fimd0.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
obj-$(CONFIG_EXYNOS4_SETUP_I2C1)	+= setup-i2c1.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C2)	+= setup-i2c2.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C3)	+= setup-i2c3.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C4)	+= setup-i2c4.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C5)	+= setup-i2c5.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C6)	+= setup-i2c6.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C7)	+= setup-i2c7.o
obj-$(CONFIG_EXYNOS4_SETUP_KEYPAD)	+= setup-keypad.o
obj-$(CONFIG_EXYNOS4_SETUP_SDHCI)	+= setup-sdhci.o
obj-$(CONFIG_EXYNOS4_SETUP_SDHCI_GPIO)	+= setup-sdhci-gpio.o

obj-$(CONFIG_EXYNOS4_SETUP_USB_PHY)	+= setup-usb-phy.o
