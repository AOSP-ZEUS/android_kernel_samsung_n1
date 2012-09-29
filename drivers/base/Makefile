# Makefile for the Linux device tree

obj-y			:= core.o sys.o bus.o dd.o syscore.o \
			   driver.o class.o platform.o \
			   cpu.o firmware.o init.o map.o devres.o \
			   attribute_container.o transport_class.o
obj-$(CONFIG_DEVTMPFS)	+= devtmpfs.o
obj-y			+= power/
obj-$(CONFIG_HAS_DMA)	+= dma-mapping.o
obj-$(CONFIG_HAVE_GENERIC_DMA_COHERENT) += dma-coherent.o
<<<<<<< HEAD
=======
obj-$(CONFIG_DMA_SHARED_BUFFER) += dma-buf.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
obj-$(CONFIG_ISA)	+= isa.o
obj-$(CONFIG_FW_LOADER)	+= firmware_class.o
obj-$(CONFIG_NUMA)	+= node.o
obj-$(CONFIG_MEMORY_HOTPLUG_SPARSE) += memory.o
obj-$(CONFIG_SMP)	+= topology.o
<<<<<<< HEAD
obj-$(CONFIG_IOMMU_API) += iommu.o
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
ifeq ($(CONFIG_SYSFS),y)
obj-$(CONFIG_MODULES)	+= module.o
endif
obj-$(CONFIG_SYS_HYPERVISOR) += hypervisor.o
<<<<<<< HEAD

obj-$(CONFIG_SYNC)	+= sync.o
obj-$(CONFIG_SW_SYNC)	+= sw_sync.o
=======
obj-$(CONFIG_REGMAP)	+= regmap/
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

ccflags-$(CONFIG_DEBUG_DRIVER) := -DDEBUG

