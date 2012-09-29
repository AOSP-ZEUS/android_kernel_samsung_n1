obj-y	+= grant-table.o features.o events.o manage.o balloon.o
obj-y	+= xenbus/
<<<<<<< HEAD
obj-y	+= tmem.o
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

nostackp := $(call cc-option, -fno-stack-protector)
CFLAGS_features.o			:= $(nostackp)

obj-$(CONFIG_BLOCK)			+= biomerge.o
obj-$(CONFIG_HOTPLUG_CPU)		+= cpu_hotplug.o
obj-$(CONFIG_XEN_XENCOMM)		+= xencomm.o
obj-$(CONFIG_XEN_BALLOON)		+= xen-balloon.o
<<<<<<< HEAD
=======
obj-$(CONFIG_XEN_SELFBALLOONING)	+= xen-selfballoon.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
obj-$(CONFIG_XEN_DEV_EVTCHN)		+= xen-evtchn.o
obj-$(CONFIG_XEN_GNTDEV)		+= xen-gntdev.o
obj-$(CONFIG_XEN_GRANT_DEV_ALLOC)	+= xen-gntalloc.o
obj-$(CONFIG_XENFS)			+= xenfs/
obj-$(CONFIG_XEN_SYS_HYPERVISOR)	+= sys-hypervisor.o
obj-$(CONFIG_XEN_PLATFORM_PCI)		+= xen-platform-pci.o
<<<<<<< HEAD
obj-$(CONFIG_SWIOTLB_XEN)		+= swiotlb-xen.o
obj-$(CONFIG_XEN_DOM0)			+= pci.o
=======
obj-$(CONFIG_XEN_TMEM)			+= tmem.o
obj-$(CONFIG_SWIOTLB_XEN)		+= swiotlb-xen.o
obj-$(CONFIG_XEN_DOM0)			+= pci.o
obj-$(CONFIG_XEN_PCIDEV_BACKEND)	+= xen-pciback/
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

xen-evtchn-y				:= evtchn.o
xen-gntdev-y				:= gntdev.o
xen-gntalloc-y				:= gntalloc.o

xen-platform-pci-y			:= platform-pci.o
