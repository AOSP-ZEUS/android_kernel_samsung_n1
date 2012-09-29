#!/bin/sh
<<<<<<< HEAD
# Test for gcc 'asm goto' suport
=======
# Test for gcc 'asm goto' support
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
# Copyright (C) 2010, Jason Baron <jbaron@redhat.com>

echo "int main(void) { entry: asm goto (\"\"::::entry); return 0; }" | $@ -x c - -c -o /dev/null >/dev/null 2>&1 && echo "y"
