#
# Copyright 2003 PathScale, Inc.
#
# Licensed under the GPL
#

<<<<<<< HEAD
obj-y = bug.o bugs.o delay.o fault.o ldt.o mem.o ptrace.o ptrace_user.o \
	setjmp.o signal.o stub.o stub_segv.o syscalls.o syscall_table.o \
	sysrq.o ksyms.o tls.o

subarch-obj-y = lib/csum-partial_64.o lib/memcpy_64.o lib/thunk_64.o \
		lib/rwsem_64.o
=======
obj-y = bug.o bugs.o delay.o fault.o ldt.o ptrace.o ptrace_user.o mem.o \
	setjmp.o signal.o stub.o stub_segv.o syscalls.o syscall_table.o \
	sysrq.o ksyms.o tls.o

obj-y += vdso/

subarch-obj-y = lib/csum-partial_64.o lib/memcpy_64.o lib/thunk_64.o \
		lib/rwsem.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
subarch-obj-$(CONFIG_MODULES) += kernel/module.o

ldt-y = ../sys-i386/ldt.o

USER_OBJS := ptrace_user.o

USER_OBJS += user-offsets.s
extra-y += user-offsets.s

UNPROFILE_OBJS := stub_segv.o
CFLAGS_stub_segv.o := $(CFLAGS_NO_HARDENING)

include arch/um/scripts/Makefile.rules
