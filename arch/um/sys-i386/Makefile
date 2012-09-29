#
# Copyright (C) 2002 - 2007 Jeff Dike (jdike@{addtoit,linux.intel}.com)
#

obj-y = bug.o bugs.o checksum.o delay.o fault.o ksyms.o ldt.o ptrace.o \
	ptrace_user.o setjmp.o signal.o stub.o stub_segv.o syscalls.o sysrq.o \
<<<<<<< HEAD
	sys_call_table.o tls.o atomic64_cx8_32.o

obj-$(CONFIG_BINFMT_ELF) += elfcore.o

subarch-obj-y = lib/semaphore_32.o lib/string_32.o
=======
	sys_call_table.o tls.o atomic64_cx8_32.o mem.o

obj-$(CONFIG_BINFMT_ELF) += elfcore.o

subarch-obj-y = lib/string_32.o
subarch-obj-$(CONFIG_RWSEM_XCHGADD_ALGORITHM) += lib/rwsem.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
subarch-obj-$(CONFIG_HIGHMEM) += mm/highmem_32.o
subarch-obj-$(CONFIG_MODULES) += kernel/module.o

USER_OBJS := bugs.o ptrace_user.o fault.o

USER_OBJS += user-offsets.s
extra-y += user-offsets.s

UNPROFILE_OBJS := stub_segv.o
CFLAGS_stub_segv.o := $(CFLAGS_NO_HARDENING)

include arch/um/scripts/Makefile.rules
