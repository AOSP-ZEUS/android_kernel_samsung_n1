#
# Makefile for the kernel security code
#

obj-$(CONFIG_KEYS)			+= keys/
subdir-$(CONFIG_SECURITY_SELINUX)	+= selinux
subdir-$(CONFIG_SECURITY_SMACK)		+= smack
subdir-$(CONFIG_SECURITY_TOMOYO)        += tomoyo
subdir-$(CONFIG_SECURITY_APPARMOR)	+= apparmor
<<<<<<< HEAD
=======
subdir-$(CONFIG_TRUSTED_FOUNDATIONS)	+= tf_driver
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

# always enable default capabilities
obj-y					+= commoncap.o
obj-$(CONFIG_MMU)			+= min_addr.o

# Object file lists
obj-$(CONFIG_SECURITY)			+= security.o capability.o
obj-$(CONFIG_SECURITYFS)		+= inode.o
# Must precede capability.o in order to stack properly.
obj-$(CONFIG_SECURITY_SELINUX)		+= selinux/built-in.o
obj-$(CONFIG_SECURITY_SMACK)		+= smack/built-in.o
obj-$(CONFIG_AUDIT)			+= lsm_audit.o
obj-$(CONFIG_SECURITY_TOMOYO)		+= tomoyo/built-in.o
obj-$(CONFIG_SECURITY_APPARMOR)		+= apparmor/built-in.o
obj-$(CONFIG_CGROUP_DEVICE)		+= device_cgroup.o
<<<<<<< HEAD
=======
obj-$(CONFIG_TRUSTED_FOUNDATIONS)	+= tf_driver/built-in.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

# Object integrity file lists
subdir-$(CONFIG_IMA)			+= integrity/ima
obj-$(CONFIG_IMA)			+= integrity/ima/built-in.o
