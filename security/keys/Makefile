#
# Makefile for key management
#

obj-y := \
	gc.o \
	key.o \
	keyring.o \
	keyctl.o \
	permission.o \
	process_keys.o \
	request_key.o \
	request_key_auth.o \
	user_defined.o

obj-$(CONFIG_TRUSTED_KEYS) += trusted.o
<<<<<<< HEAD
obj-$(CONFIG_ENCRYPTED_KEYS) += encrypted.o
=======
obj-$(CONFIG_ENCRYPTED_KEYS) += ecryptfs_format.o encrypted.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
obj-$(CONFIG_KEYS_COMPAT) += compat.o
obj-$(CONFIG_PROC_FS) += proc.o
obj-$(CONFIG_SYSCTL) += sysctl.o
