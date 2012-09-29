
#include "../../../arch/x86/lib/memcpy_64.S"
<<<<<<< HEAD
/*
 * We need to provide note.GNU-stack section, saying that we want
 * NOT executable stack. Otherwise the final linking will assume that
 * the ELF stack should not be restricted at all and set it RWX.
 */
.section .note.GNU-stack,"",@progbits
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
