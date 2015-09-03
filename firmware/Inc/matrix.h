void sub_mtrx_nxn(int dim, const float *A, const float *B, float *C );
void add_mtrx_nxn(int dim, const float *A, const float *B, float *C );
void mult_mtrx_nxn( int dim, const float A[dim][dim], const float B[dim][dim], float C[dim][dim]);
void array_to_mtrx_nxn(float *Arry, float *Mtrx, int dim);
int invert_mtrx_4x4(const float m[16], float invOut[16]);
void print_mtrx_nxn(int dim, float *A);
void transpose_mtrx_nxn(int dim, const float A[dim][dim], float transOut[dim][dim]);
float det_mtrx_4x4(float *m);
float det_mtrx_2x2(float *m);