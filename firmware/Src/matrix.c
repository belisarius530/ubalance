#include <math.h>
#include <stdlib.h>
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "matrix.h"

//int main(void) {
//    float Mtrx[2][2] = {0.0};
//    float B[2][2] = {{19.3,1.3},{1.1,5.8}};
//    float A[2][2] = {{-1.1,0.0},{0.0,-1.1}};
//    float C[2][2] = {0.0};
//    float A4x4[4][4] = {{1,2,3,4},
//                         {0,2,7,3},
//                         {0,0,8,9},
//                         {0,0,0,7}};
//    float B4x4[4][4] = {0};
//    float C4x4[4][4] = {0};
//    invert_mtrx_4x4(&A4x4[0][0],&B4x4[0][0]);
//    mult_mtrx_nxn(4,A4x4,B4x4,C4x4);
//    mult_mtrx_nxn(2, A, B, C);
//    float Arry[4] = {2.1, 2.3, 2.4, 8.1};
//    print_mtrx_nxn(4, &A4x4[0][0]);
//    print_mtrx_nxn(4, &B4x4[0][0]);
//    print_mtrx_nxn(4, &C4x4[0][0]);
//   // float **fpp;
//   // fpp = Mtrx;
//    //printf("\n\rMtrx[1,0] = %f\n\r",fpp[1][0]);
//    array_to_mtrx_nxn(&Arry[0], &Mtrx[0][0], 4);
//    print_mtrx_nxn(2,C);
//  	transpose_mtrx_nxn(2, &B[0][0], &C[0][0]);
//    print_mtrx_nxn(2,C);
//    transpose_mtrx_nxn(4,&A4x4[0][0], &C4x4[0][0] );
//    print_mtrx_nxn(4, &C4x4[0][0]);
//    printf("Determinant of matrix C4x4 = %lf", det_mtrx_4x4(&C4x4[0][0]));
//	return 0;
//}

void array_to_mtrx_nxn(float *Arry, float *Mtrx, int dim)
{
	int a;
	for(a=0; a<dim*dim; a++)
	{
	    Mtrx[a] = Arry[a];
	}
	return;
}

void transpose_mtrx_nxn(int dim, const float A[dim][dim], float transOut[dim][dim])
{
	int i, j;
	for(j=0;j<dim;j++)
	{
		for(i=0;i<dim;i++)
		{
			transOut[i][j] = A[j][i];
		}
	}
}

void mult_mtrx_nxn( int dim,const float A[dim][dim],const float B[dim][dim], float C[dim][dim])
{
	int i, j, s;
	float sum;
	for(i=0;i<dim;i++)
	{
		for(j=0;j<dim;j++)
		{
			sum = 0;
			for(s=0;s<dim;s++)
			{
				sum += A[i][s]*B[s][j];	
			}
			C[i][j] = sum;
		}
	}
	return;
}

int invert_mtrx_4x4(const float m[16], float invOut[16])
{
    float inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return 0;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return 1;
}

int invert_mtrx_2x2(const float m[4], float invOut[4])
{
	float det = m[0]*m[3] - m[1]*m[2];
	if (det == 0.0)
	    return 0;
	det = 1.0/det;
	invOut[0] = m[3]*det;
	invOut[1] = -m[1];
	invOut[2] = -m[2];
	invOut[3] = m[0]*det;
	
	return 1;
}

void print_mtrx_nxn(int dim, float *A)
{
	int i;
	for(i = 0;i<dim*dim;i++)
	{
		
		if( !(i%dim) )
		    printf("\n\r");
		printf("%lf  ", A[i]);
	}
	printf("\n\r");
}

float det_mtrx_4x4(float *m){
	return
         m[12] * m[9]  * m[6]  * m[3]   -  m[8] * m[13] * m[6]  * m[3]   -
         m[12] * m[5]  * m[10] * m[3]   +  m[4] * m[13] * m[10] * m[3]   +
         m[8]  * m[5]  * m[14] * m[3]   -  m[4] * m[9]  * m[14] * m[3]   -
         m[12] * m[9]  * m[2]  * m[7]   +  m[8] * m[13] * m[2]  * m[7]   +
         m[12] * m[1]  * m[10] * m[7]   -  m[0] * m[13] * m[10] * m[7]   -
         m[8]  * m[1]  * m[14] * m[7]   +  m[0] * m[9]  * m[14] * m[7]   +
         m[12] * m[5]  * m[2]  * m[11]  -  m[4] * m[13] * m[2]  * m[11]  -
         m[12] * m[1]  * m[6]  * m[11]  +  m[0] * m[13] * m[6]  * m[11]  +
         m[4]  * m[1]  * m[14] * m[11]  -  m[0] * m[5]  * m[14] * m[11]  -
         m[8]  * m[5]  * m[2]  * m[15]  +  m[4] * m[9]  * m[2]  * m[15]  +
         m[8]  * m[1]  * m[6]  * m[15]  -  m[0] * m[9]  * m[6]  * m[15]  -
         m[4]  * m[1]  * m[10] * m[15]  +  m[0] * m[5]  * m[10] * m[15];
}
float det_mtrx_2x2(float *m)
{
    return m[0]*m[3] - m[1]*m[2];
}

	
void add_mtrx_nxn(int dim, const float *A, const float *B, float *C )
{
	int i;
	for(i=0; i<dim*dim;i++)
	{
		C[i] = A[i]+B[i];
	}
	return;
}	
void sub_mtrx_nxn(int dim, const float *A, const float *B, float *C )
{
	int i;
	for(i=0; i<dim*dim;i++)
	{
		C[i] = A[i] - B[i];
	}
	return;
}