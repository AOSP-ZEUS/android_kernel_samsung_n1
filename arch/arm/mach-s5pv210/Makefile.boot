<<<<<<< HEAD
zreladdr-y	:= 0x20008000
params_phys-y	:= 0x20000100

# override for Herring
zreladdr-$(CONFIG_MACH_HERRING)	:= 0x30008000
params_phys-$(CONFIG_MACH_HERRING)	:= 0x30000100
=======
   zreladdr-y	:= 0x20008000
params_phys-y	:= 0x20000100
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
