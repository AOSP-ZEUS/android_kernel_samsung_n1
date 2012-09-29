#include <linux/ceph/ceph_debug.h>

#include <linux/err.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#include <linux/ceph/msgpool.h>

<<<<<<< HEAD
static void *alloc_fn(gfp_t gfp_mask, void *arg)
{
	struct ceph_msgpool *pool = arg;
	void *p;

	p = ceph_msg_new(0, pool->front_len, gfp_mask);
	if (!p)
		pr_err("msgpool %s alloc failed\n", pool->name);
	return p;
}

static void free_fn(void *element, void *arg)
{
	ceph_msg_put(element);
=======
static void *msgpool_alloc(gfp_t gfp_mask, void *arg)
{
	struct ceph_msgpool *pool = arg;
	struct ceph_msg *msg;

	msg = ceph_msg_new(0, pool->front_len, gfp_mask);
	if (!msg) {
		dout("msgpool_alloc %s failed\n", pool->name);
	} else {
		dout("msgpool_alloc %s %p\n", pool->name, msg);
		msg->pool = pool;
	}
	return msg;
}

static void msgpool_free(void *element, void *arg)
{
	struct ceph_msgpool *pool = arg;
	struct ceph_msg *msg = element;

	dout("msgpool_release %s %p\n", pool->name, msg);
	msg->pool = NULL;
	ceph_msg_put(msg);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

int ceph_msgpool_init(struct ceph_msgpool *pool,
		      int front_len, int size, bool blocking, const char *name)
{
<<<<<<< HEAD
	pool->front_len = front_len;
	pool->pool = mempool_create(size, alloc_fn, free_fn, pool);
=======
	dout("msgpool %s init\n", name);
	pool->front_len = front_len;
	pool->pool = mempool_create(size, msgpool_alloc, msgpool_free, pool);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (!pool->pool)
		return -ENOMEM;
	pool->name = name;
	return 0;
}

void ceph_msgpool_destroy(struct ceph_msgpool *pool)
{
<<<<<<< HEAD
=======
	dout("msgpool %s destroy\n", pool->name);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	mempool_destroy(pool->pool);
}

struct ceph_msg *ceph_msgpool_get(struct ceph_msgpool *pool,
				  int front_len)
{
<<<<<<< HEAD
	if (front_len > pool->front_len) {
		pr_err("msgpool_get pool %s need front %d, pool size is %d\n",
=======
	struct ceph_msg *msg;

	if (front_len > pool->front_len) {
		dout("msgpool_get %s need front %d, pool size is %d\n",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		       pool->name, front_len, pool->front_len);
		WARN_ON(1);

		/* try to alloc a fresh message */
		return ceph_msg_new(0, front_len, GFP_NOFS);
	}

<<<<<<< HEAD
	return mempool_alloc(pool->pool, GFP_NOFS);
=======
	msg = mempool_alloc(pool->pool, GFP_NOFS);
	dout("msgpool_get %s %p\n", pool->name, msg);
	return msg;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

void ceph_msgpool_put(struct ceph_msgpool *pool, struct ceph_msg *msg)
{
<<<<<<< HEAD
=======
	dout("msgpool_put %s %p\n", pool->name, msg);

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* reset msg front_len; user may have changed it */
	msg->front.iov_len = pool->front_len;
	msg->hdr.front_len = cpu_to_le32(pool->front_len);

	kref_init(&msg->kref);  /* retake single ref */
<<<<<<< HEAD
=======
	mempool_free(msg, pool->pool);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}
