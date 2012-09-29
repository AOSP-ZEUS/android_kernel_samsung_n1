extern void	pstore_set_kmsg_bytes(int);
extern void	pstore_get_records(void);
extern int	pstore_mkfile(enum pstore_type_id, char *psname, u64 id,
			      char *data, size_t size,
<<<<<<< HEAD
			      struct timespec time, int (*erase)(u64));
=======
			      struct timespec time, struct pstore_info *psi);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
extern int	pstore_is_mounted(void);
