cmd_arch/arm/lib/backtrace.o := /home/dman3285/CM10/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-gcc -Wp,-MD,arch/arm/lib/.backtrace.o.d  -nostdinc -isystem /home/dman3285/CM10/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include -I/home/dman3285/Kernerl-Source/kernel_hammer_v1/arch/arm/include -Iarch/arm/include/generated -Iinclude  -include /home/dman3285/Kernerl-Source/kernel_hammer_v1/include/linux/kconfig.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-tegra/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables  -D__LINUX_ARM_ARCH__=7 -march=armv7-a  -include asm/unified.h -msoft-float -gdwarf-2        -c -o arch/arm/lib/backtrace.o arch/arm/lib/backtrace.S

source_arch/arm/lib/backtrace.o := arch/arm/lib/backtrace.S

deps_arch/arm/lib/backtrace.o := \
    $(wildcard include/config/cpu.h) \
    $(wildcard include/config/frame/pointer.h) \
    $(wildcard include/config/printk.h) \
  /home/dman3285/Kernerl-Source/kernel_hammer_v1/include/linux/kconfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/.h) \
    $(wildcard include/config/foo.h) \
  /home/dman3285/Kernerl-Source/kernel_hammer_v1/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
    $(wildcard include/config/thumb2/kernel.h) \
  include/linux/linkage.h \
  include/linux/compiler.h \
    $(wildcard include/config/sparse/rcu/pointer.h) \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  /home/dman3285/Kernerl-Source/kernel_hammer_v1/arch/arm/include/asm/linkage.h \
  /home/dman3285/Kernerl-Source/kernel_hammer_v1/arch/arm/include/asm/assembler.h \
    $(wildcard include/config/cpu/feroceon.h) \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/smp.h) \
  /home/dman3285/Kernerl-Source/kernel_hammer_v1/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/dman3285/Kernerl-Source/kernel_hammer_v1/arch/arm/include/asm/hwcap.h \
  /home/dman3285/Kernerl-Source/kernel_hammer_v1/arch/arm/include/asm/domain.h \
    $(wildcard include/config/io/36.h) \
    $(wildcard include/config/cpu/use/domains.h) \

arch/arm/lib/backtrace.o: $(deps_arch/arm/lib/backtrace.o)

$(deps_arch/arm/lib/backtrace.o):
