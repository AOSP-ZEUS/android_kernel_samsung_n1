
#
# Makefile for m68k-specific library files..
#

lib-y	:= ashldi3.o ashrdi3.o lshrdi3.o muldi3.o \
	   memcpy.o memset.o memmove.o

ifdef CONFIG_MMU
lib-y	+= string.o uaccess.o checksum_mm.o
else
<<<<<<< HEAD
lib-y	+= mulsi3.o divsi3.o udivsi3.o modsi3.o umodsi3.o delay.o checksum_no.o
=======
lib-y	+= mulsi3.o divsi3.o udivsi3.o modsi3.o umodsi3.o checksum_no.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
endif

