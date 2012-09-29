<<<<<<< HEAD
EXTRA_CFLAGS += -I$(srctree)/drivers/target/ \
		-I$(srctree)/drivers/scsi/ \
		-I$(srctree)/include/scsi/ \
		-I$(srctree)/drivers/target/tcm_fc/

tcm_fc-y +=	tfc_cmd.o \
		tfc_conf.o \
		tfc_io.o \
		tfc_sess.o

obj-$(CONFIG_TCM_FC)	+= tcm_fc.o

ifdef CONFIGFS_TCM_FC_DEBUG
EXTRA_CFLAGS	+= -DTCM_FC_DEBUG
endif
=======
tcm_fc-y +=		tfc_cmd.o \
			tfc_conf.o \
			tfc_io.o \
			tfc_sess.o

obj-$(CONFIG_TCM_FC)	+= tcm_fc.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
