#ifndef LINUX_GENERIC_ACL_H
#define LINUX_GENERIC_ACL_H

#include <linux/xattr.h>

struct inode;

extern const struct xattr_handler generic_acl_access_handler;
extern const struct xattr_handler generic_acl_default_handler;

int generic_acl_init(struct inode *, struct inode *);
int generic_acl_chmod(struct inode *);
<<<<<<< HEAD
int generic_check_acl(struct inode *inode, int mask, unsigned int flags);
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#endif /* LINUX_GENERIC_ACL_H */
