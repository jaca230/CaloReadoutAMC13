#ifndef fe_compress_z_mt_h
#define fe_compress_z_mt_h

#ifdef tcp_thread_c
#define EXTERN
#else
#define EXTERN extern
#endif

/* make functions callable from a C++ program */
#ifdef __cplusplus
extern "C" {
#endif

EXTERN INT fe_compress_z_init();
EXTERN INT fe_compress_z_exit();
EXTERN INT fe_compress_z_bor(INT run_number, char *error);
EXTERN INT fe_compress_z_eor(INT run_number, char *error);
EXTERN INT fe_compress_z(char *pevent, char *inp, unsigned int size, unsigned int space, int thread);
EXTERN INT fe_compress_z2(char *pevent, char *inp, unsigned int size, unsigned int space, int thread, char *zname);

#ifdef __cplusplus
}
#endif 

#undef EXTERN

#endif /* fe_compress_z_mt_h defined */
