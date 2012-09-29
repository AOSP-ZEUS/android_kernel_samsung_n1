#
# ia64/platform/dig/Makefile
#
# Copyright (C) 1999 Silicon Graphics, Inc.
# Copyright (C) Srinivasa Thirumalachar (sprasad@engr.sgi.com)
#

obj-y := setup.o
<<<<<<< HEAD
ifeq ($(CONFIG_DMAR), y)
=======
ifeq ($(CONFIG_INTEL_IOMMU), y)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
obj-$(CONFIG_IA64_GENERIC) += machvec.o machvec_vtd.o
else
obj-$(CONFIG_IA64_GENERIC) += machvec.o
endif

