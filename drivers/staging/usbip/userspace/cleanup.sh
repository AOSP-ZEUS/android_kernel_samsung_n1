<<<<<<< HEAD
#!/bin/sh -x

=======
#!/bin/sh
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

if [ -r Makefile ]; then
	make distclean
fi

<<<<<<< HEAD
FILES="configure cscope.out Makefile.in depcomp compile config.guess config.sub config.h.in~ config.log config.status ltmain.sh libtool config.h.in autom4te.cache missing aclocal.m4 install-sh cmd/Makefile.in lib/Makefile.in Makefile lib/Makefile cmd/Makefile"

rm -Rf $FILES
=======
FILES="aclocal.m4 autom4te.cache compile config.guess config.h.in config.log \
       config.status config.sub configure cscope.out depcomp install-sh      \
       libsrc/Makefile libsrc/Makefile.in libtool ltmain.sh Makefile         \
       Makefile.in missing src/Makefile src/Makefile.in"

rm -vRf $FILES
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
