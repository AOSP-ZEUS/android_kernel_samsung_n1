<<<<<<< HEAD
bcma-y					+= main.o scan.o core.o
bcma-y					+= driver_chipcommon.o driver_chipcommon_pmu.o
bcma-y					+= driver_pci.o
=======
bcma-y					+= main.o scan.o core.o sprom.o
bcma-y					+= driver_chipcommon.o driver_chipcommon_pmu.o
bcma-y					+= driver_pci.o
bcma-$(CONFIG_BCMA_DRIVER_PCI_HOSTMODE)	+= driver_pci_host.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
bcma-$(CONFIG_BCMA_HOST_PCI)		+= host_pci.o
obj-$(CONFIG_BCMA)			+= bcma.o

ccflags-$(CONFIG_BCMA_DEBUG)		:= -DDEBUG
