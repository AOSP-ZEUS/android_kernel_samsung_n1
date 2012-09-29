/*
 * Copyright (c) 2000-2003,2005 Silicon Graphics, Inc.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write the Free Software Foundation,
 * Inc.,  51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#ifndef __XFS_H__
#define __XFS_H__

#ifdef CONFIG_XFS_DEBUG
#define STATIC
#define DEBUG 1
#define XFS_BUF_LOCK_TRACKING 1
<<<<<<< HEAD
/* #define QUOTADEBUG 1 */
#endif

#include <linux-2.6/xfs_linux.h>
=======
#endif

#include "xfs_linux.h"

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif	/* __XFS_H__ */
