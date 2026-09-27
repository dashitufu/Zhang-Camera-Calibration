#include "stdio.h"
#include <iostream>
#include "assert.h"
#include "memory.h"
#include <stdarg.h>
//#include "Common.h"
#include "Matrix.h"
#include "float.h"
#ifdef ESP32
	#include "esp_dsp.h"
#else
	#include "immintrin.h"
	//#include "../esp_dsp_pc/esp_dsp.h"
#endif

//实例化
#ifndef ESP32
template void Matrix_Multiply<float>(float* A, int ma, int na, float* B, int nb, float* C);
template void Matrix_Multiply<double>(double* A, int ma, int na, double* B, int nb, double* C);
template void Matrix_Multiply<int>(int* A, int ma, int na, int* B, int nb, int* C);

template void Reshape_Row_Mul_Align<float>(float A[], int m, int n, int iBlock_Size, float** ppA1, int bTranspose);
template void Reshape_Row_Mul_Align<double>(double A[], int m, int n, int iBlock_Size, double** ppA1, int bTranspose);
template void Reshape_Col_Mul_Align<float>(float B[], int m, int n, int iBlock_Size, float** ppB1, int bTranspose);
template void Reshape_Col_Mul_Align<double>(double B[], int m, int n, int iBlock_Size, double** ppB1, int bTranspose);
#endif

#ifndef ESP32
void Matrix_Multiply_Reshape_double(double A[], int ma, int na, double B[], int nb,
	double C[], int mc, int nc)
{//注意，ma, na, nb一定是扩展后的长度，8对齐
	const int iBlock_Size = 8,
		iBlock_Size_Sqr = iBlock_Size * iBlock_Size;
	unsigned char iMask = (1 << (nc % iBlock_Size)) - 1;
	for (int y0 = 0; y0 < ma; y0 += iBlock_Size)
	{
		for (int x0 = 0; x0 < nb; x0 += iBlock_Size)
		{
			__m512d Sum[iBlock_Size] = {},
				B_Block[iBlock_Size];

			/*if (x0 == 8)
				printf("here");*/
				//将A的行块乘以B的列块
			double* pA_Block = &A[y0 * na],
				* pB_Block = &B[x0 * na];
			for (int x1 = 0; x1 < na; x1 += iBlock_Size)
			{
				B_Block[0] = _mm512_loadu_pd(&pB_Block[0]);
				B_Block[1] = _mm512_loadu_pd(&pB_Block[1 * iBlock_Size]);
				B_Block[2] = _mm512_loadu_pd(&pB_Block[2 * iBlock_Size]);
				B_Block[3] = _mm512_loadu_pd(&pB_Block[3 * iBlock_Size]);
				B_Block[4] = _mm512_loadu_pd(&pB_Block[4 * iBlock_Size]);
				B_Block[5] = _mm512_loadu_pd(&pB_Block[5 * iBlock_Size]);
				B_Block[6] = _mm512_loadu_pd(&pB_Block[6 * iBlock_Size]);
				B_Block[7] = _mm512_loadu_pd(&pB_Block[7 * iBlock_Size]);

				//据说放在加载完的地方合适
				_mm_prefetch((const char*)(pB_Block + iBlock_Size_Sqr), _MM_HINT_T0);
				for (int i = 0; i < iBlock_Size; i++)
				{
					Sum[i] = _mm512_fmadd_pd(_mm512_set1_pd(pA_Block[0]), B_Block[0], Sum[i]);
					Sum[i] = _mm512_fmadd_pd(_mm512_set1_pd(pA_Block[1]), B_Block[1], Sum[i]);
					Sum[i] = _mm512_fmadd_pd(_mm512_set1_pd(pA_Block[2]), B_Block[2], Sum[i]);
					Sum[i] = _mm512_fmadd_pd(_mm512_set1_pd(pA_Block[3]), B_Block[3], Sum[i]);
					Sum[i] = _mm512_fmadd_pd(_mm512_set1_pd(pA_Block[4]), B_Block[4], Sum[i]);
					Sum[i] = _mm512_fmadd_pd(_mm512_set1_pd(pA_Block[5]), B_Block[5], Sum[i]);
					Sum[i] = _mm512_fmadd_pd(_mm512_set1_pd(pA_Block[6]), B_Block[6], Sum[i]);
					Sum[i] = _mm512_fmadd_pd(_mm512_set1_pd(pA_Block[7]), B_Block[7], Sum[i]);
					_mm_prefetch((const char*)(pA_Block + iBlock_Size), _MM_HINT_T0);
					pA_Block += iBlock_Size;
				}
				pB_Block += iBlock_Size_Sqr;
			}

			//将Block 放回道c中
			double* pC_Block = &C[y0 * nc + x0];
			int iRemain_y = mc - y0,
				iRemain_x = nc - x0;
			if (iRemain_y >= iBlock_Size && iRemain_x >= iBlock_Size)
			{
				_mm512_storeu_pd(pC_Block, Sum[0]);
				_mm512_storeu_pd(&pC_Block[1 * nc], Sum[1]);
				_mm512_storeu_pd(&pC_Block[2 * nc], Sum[2]);
				_mm512_storeu_pd(&pC_Block[3 * nc], Sum[3]);
				_mm512_storeu_pd(&pC_Block[4 * nc], Sum[4]);
				_mm512_storeu_pd(&pC_Block[5 * nc], Sum[5]);
				_mm512_storeu_pd(&pC_Block[6 * nc], Sum[6]);
				_mm512_storeu_pd(&pC_Block[7 * nc], Sum[7]);
			}
			else
			{

				if (iRemain_y > iBlock_Size)
					iRemain_y = iBlock_Size;

				if (iRemain_x >= iBlock_Size)
				{
					for (int y = 0; y < iRemain_y; y++)
						_mm512_storeu_pd(&pC_Block[y * nc], Sum[y]);
				}
				else
				{
					for (int y = 0; y < iRemain_y; y++)
						_mm512_mask_storeu_pd(&pC_Block[y * nc], iMask, Sum[y]);
				}
			}
		}
	}
	return;
}


template void At_x_B(float A[], int ma, int na, float B[], int nb, float C[]);
template void At_x_B(double A[], int ma, int na, double B[], int nb, double C[]);
template<typename _T>void At_x_B(_T A[], int ma, int na, _T B[], int nb, _T C[])
{
	int mb = ma, mc = na, nc = nb;
	int iBlock_Size = 64 / sizeof(_T);
	int ma1, na1, mb1, nb1;
	_T* pA1, * pB1;

	if (sizeof(_T) == 8)
	{
		ma1 = ALIGN_SIZE_8(ma);
		na1 = ALIGN_SIZE_8(na);
		mb1 = na1;
		nb1 = ALIGN_SIZE_8(nb);
	}
	else
	{
		ma1 = ALIGN_SIZE_16(ma);
		na1 = ALIGN_SIZE_16(na);
		mb1 = na1;
		nb1 = ALIGN_SIZE_16(nb);
	}

	int iSize = (ma1 * na1 + mb1 * nb1) * sizeof(_T);
	pA1 = (_T*)pMalloc(iSize);
	pB1 = pA1 + ma1 * na1;

	Reshape_Row_Mul_Align<_T>(A, ma, na, iBlock_Size, &pA1, 1);
	Reshape_Col_Mul_Align<_T>(B, mb, nb, iBlock_Size, &pB1);
	//注意，ma, na, nb一定是扩展后的长度，8对齐

	if (typeid(_T) == typeid(double))
		Matrix_Multiply_Reshape_double((double*)pA1, na1, ma1, (double*)pB1, nb1, (double*)C, mc, nc);
	else
		Matrix_Multiply_Reshape_float((float*)pA1, na1, ma1, (float*)pB1, nb1, (float*)C, mc, nc);

	if (pA1)
		Free(pA1);

	//别删，用来对数据
	//int mb = ma, mc = na, nc = nb;
	//_T* pC1;
	//if (C == A || C == B)
	//	pC1 = (_T*)pMalloc(mc * nc * sizeof(_T));
	//else
	//	pC1 = C;

	////用A的每一列乘以B的每一列
	//for (int y = 0; y < mc; y++)
	//{
	//	for (int x = 0; x < nc; x++)
	//	{
	//		_T fTotal = 0;
	//		for (int i = 0; i < mb; i++)
	//			fTotal += A[i * na + y] * B[x + i * nb];
	//		pC1[y * nc + x] = fTotal;
	//	}
	//}
	//if (C == A || C == B)
	//{
	//	memcpy(C, pC1, mc * nc * sizeof(_T));
	//	Free(pC1);
	//}
	return;
}

template<typename _T>void Reshape_Col_Mul_Align(_T B[], int m, int n, int iBlock_Size, _T** ppB1, int bTranspose)
{//用于重拍B矩阵，按列重排，8数据对齐
	int w1 = ((n + iBlock_Size - 1) / iBlock_Size) * iBlock_Size,
		h1 = ((m + iBlock_Size - 1) / iBlock_Size) * iBlock_Size;

	_T* pB1, * pB1_Cur, * pB_Cur;
	int iRemain_x, iRemain_y;
	if (!(*ppB1))
		pB1 = (_T*)pMalloc(w1 * h1 * sizeof(_T));
	else
		pB1 = *ppB1;
	pB1_Cur = pB1;

	if (!pB1)
	{
		printf("Insufficient memory in Reshape_Col_Mul_Align_8\n");
		return;
	}

	memset(pB1, 0, w1 * h1 * sizeof(_T));
	for (int x0 = 0; x0 < w1; x0 += iBlock_Size)
	{
		iRemain_x = n - x0;
		if (iRemain_x > iBlock_Size)
			iRemain_x = iBlock_Size;
		for (int y0 = 0; y0 < h1; y0 += iBlock_Size)
		{
			iRemain_y = m - y0;
			if (iRemain_y > iBlock_Size)
				iRemain_y = iBlock_Size;

			pB_Cur = &B[y0 * n + x0];
			if (bTranspose)
			{//需要转置
				pB1_Cur = &pB1[x0 * iBlock_Size + y0 * w1];
				for (int y1 = 0; y1 < iRemain_y; y1++)
					for (int x1 = 0; x1 < iRemain_x; x1++)
						pB1_Cur[x1 * iBlock_Size + y1] = pB_Cur[y1 * n + x1];
				//Disp(pB1_Cur, 8, 8, "Block");
			}
			else
			{
				pB1_Cur = &pB1[x0 * h1 + y0 * iBlock_Size];
				for (int i = 0; i < iRemain_y; i++, pB1_Cur += iBlock_Size, pB_Cur += n)
					memcpy(pB1_Cur, pB_Cur, iRemain_x * sizeof(_T));
			}
		}
	}

	if (!(*ppB1))
		*ppB1 = pB1;
	return;
}

template<typename _T>void Reshape_Row_Mul_Align(_T A[], int m, int n, int iBlock_Size,
	_T** ppA1, int bTranspose)
{//将二维数组改 每组为iStride的形式。 A[w/iStride][h][iStide]
	int w1 = ((n + iBlock_Size - 1) / iBlock_Size) * iBlock_Size,
		h1 = ((m + iBlock_Size - 1) / iBlock_Size) * iBlock_Size;

	_T* pA1, * pA1_Cur, * pA_Cur;
	int iRemain_x, iRemain_y;

	if (!(*ppA1))
		pA1 = (_T*)pMalloc(w1 * h1 * sizeof(_T));
	else
		pA1 = *ppA1;

	pA1_Cur = pA1;
	memset(pA1, 0, w1 * h1 * sizeof(_T));
	for (int y0 = 0; y0 < h1; y0 += iBlock_Size)
	{
		iRemain_y = m - y0;
		if (iRemain_y > iBlock_Size)
			iRemain_y = iBlock_Size;

		for (int x0 = 0; x0 < w1; x0 += iBlock_Size)
		{
			iRemain_x = n - x0;
			if (iRemain_x > iBlock_Size)
				iRemain_x = iBlock_Size;
			pA_Cur = &A[y0 * n + x0];
			if (bTranspose)
			{//需要转置
				pA1_Cur = &pA1[x0 * h1 + y0 * iBlock_Size];
				for (int y1 = 0; y1 < iRemain_y; y1++)
					for (int x1 = 0; x1 < iRemain_x; x1++)
						pA1_Cur[x1 * iBlock_Size + y1] = pA_Cur[y1 * n + x1];
			}
			else
			{
				pA1_Cur = &pA1[y0 * w1 + x0 * iBlock_Size];
				for (int i = 0; i < iRemain_y; i++, pA1_Cur += iBlock_Size, pA_Cur += n)
					memcpy(pA1_Cur, pA_Cur, iRemain_x * sizeof(_T));
			}
		}
	}

	if (!*ppA1)
		*ppA1 = pA1;
	return;
}

template<typename _T>void Block_Multiply(_T A[], _T B[], int iBlock_Size, _T Sum[])
{
	const int iSub_Block_Size = 4;
	int Sub_Block_Count = iBlock_Size >> 2;

	//块内求和
	_T* pSum_1 = Sum;
	for (int y2 = 0; y2 < iBlock_Size; y2++, pSum_1 += iBlock_Size)
	{
		_T* pB_Block_1 = B;
		for (int y3 = 0; y3 < iBlock_Size; y3++)
		{
			_T fValue_A = *(A++);
			for (int x3 = 0; x3 < iBlock_Size; x3++)
				pSum_1[x3] += *(pB_Block_1++) * fValue_A;
		}
	}
}

template<typename _T>void Matrix_Multiply_Reshape(_T A[], int ma, int na, _T B[], int nb,
	_T C[], int mc, int nc, int iBlock_Size)
{//纯CPU分块
	const int iBlock_Size_Sqr = iBlock_Size * iBlock_Size;
	_T* Sum = (_T*)pMalloc(iBlock_Size_Sqr * sizeof(_T));

	for (int y0 = 0; y0 < ma; y0 += iBlock_Size)
	{
		for (int x0 = 0; x0 < nb; x0 += iBlock_Size)
		{
			//_T Sum[iBlock_Size_Sqr] = {};
			memset(Sum, 0, iBlock_Size_Sqr * sizeof(_T));
			//将A的行块乘以B的列块
			_T* pA_Block = &A[y0 * na],
				* pB_Block = &B[x0 * na];

			for (int x1 = 0; x1 < na; x1 += iBlock_Size)
			{
//#ifdef ESP32
//				Block_Multiply_esp32(pA_Block, pB_Block, iBlock_Size, Sum);
//#else
				Block_Multiply_2(pA_Block, pB_Block, iBlock_Size, Sum);
//#endif
				pB_Block += iBlock_Size_Sqr;
				pA_Block += iBlock_Size_Sqr;
			}

			//将Block 放回道c中
			_T* pC_Block = &C[y0 * nc + x0];
			int iRemain_y = mc - y0,
				iRemain_x = nc - x0;

			if (iRemain_y > iBlock_Size)
				iRemain_y = iBlock_Size;
			if (iRemain_x > iBlock_Size)
				iRemain_x = iBlock_Size;

			for (int y = 0; y < iRemain_y; y++)
				for (int x = 0; x < iRemain_x; x++)
					pC_Block[y * nc + x] = Sum[y * iBlock_Size + x];
		}
	}
	if (Sum)
		Free(Sum);
	return;
}

void Matrix_Multiply_Reshape_float(float A[], int ma, int na, float B[], int nb,
	float C[], int mc, int nc)
{//已经比avx256快很多
	const int iBlock_Size = 16,
		iBlock_Size_Sqr = iBlock_Size * iBlock_Size;
	unsigned short iMask = (1 << (nc % iBlock_Size)) - 1;

	for (int y0 = 0; y0 < ma; y0 += iBlock_Size)
	{
		for (int x0 = 0; x0 < nb; x0 += iBlock_Size)
		{
			__m512 Sum[iBlock_Size] = {},
				B_Block[iBlock_Size];
			//将A的行块乘以B的列块
			float* pA_Block = &A[y0 * na],
				* pB_Block = &B[x0 * na];


			for (int x1 = 0; x1 < na; x1 += iBlock_Size)
			{
				B_Block[0] = _mm512_loadu_ps(&pB_Block[0]);
				B_Block[1] = _mm512_loadu_ps(&pB_Block[1 * iBlock_Size]);
				B_Block[2] = _mm512_loadu_ps(&pB_Block[2 * iBlock_Size]);
				B_Block[3] = _mm512_loadu_ps(&pB_Block[3 * iBlock_Size]);
				B_Block[4] = _mm512_loadu_ps(&pB_Block[4 * iBlock_Size]);
				B_Block[5] = _mm512_loadu_ps(&pB_Block[5 * iBlock_Size]);
				B_Block[6] = _mm512_loadu_ps(&pB_Block[6 * iBlock_Size]);
				B_Block[7] = _mm512_loadu_ps(&pB_Block[7 * iBlock_Size]);
				B_Block[8] = _mm512_loadu_ps(&pB_Block[8 * iBlock_Size]);
				B_Block[9] = _mm512_loadu_ps(&pB_Block[9 * iBlock_Size]);
				B_Block[10] = _mm512_loadu_ps(&pB_Block[10 * iBlock_Size]);
				B_Block[11] = _mm512_loadu_ps(&pB_Block[11 * iBlock_Size]);
				B_Block[12] = _mm512_loadu_ps(&pB_Block[12 * iBlock_Size]);
				B_Block[13] = _mm512_loadu_ps(&pB_Block[13 * iBlock_Size]);
				B_Block[14] = _mm512_loadu_ps(&pB_Block[14 * iBlock_Size]);
				B_Block[15] = _mm512_loadu_ps(&pB_Block[15 * iBlock_Size]);

				//据说放在加载完的地方合适
				_mm_prefetch((const char*)(pB_Block + iBlock_Size_Sqr), _MM_HINT_T0);
				for (int i = 0; i < iBlock_Size; i++)
				{
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[0]), B_Block[0], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[1]), B_Block[1], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[2]), B_Block[2], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[3]), B_Block[3], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[4]), B_Block[4], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[5]), B_Block[5], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[6]), B_Block[6], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[7]), B_Block[7], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[8]), B_Block[8], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[9]), B_Block[9], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[10]), B_Block[10], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[11]), B_Block[11], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[12]), B_Block[12], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[13]), B_Block[13], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[14]), B_Block[14], Sum[i]);
					Sum[i] = _mm512_fmadd_ps(_mm512_set1_ps(pA_Block[15]), B_Block[15], Sum[i]);

					_mm_prefetch((const char*)(pA_Block + iBlock_Size), _MM_HINT_T0);
					pA_Block += iBlock_Size;
				}
				pB_Block += iBlock_Size_Sqr;
			}

			//将Block 放回道c中
			float* pC_Block = &C[y0 * nc + x0];
			int iRemain_y = mc - y0,
				iRemain_x = nc - x0;
			if (iRemain_y > iBlock_Size && iRemain_x > iBlock_Size)
			{
				_mm512_storeu_ps(pC_Block, Sum[0]);
				_mm512_storeu_ps(&pC_Block[1 * nc], Sum[1]);
				_mm512_storeu_ps(&pC_Block[2 * nc], Sum[2]);
				_mm512_storeu_ps(&pC_Block[3 * nc], Sum[3]);
				_mm512_storeu_ps(&pC_Block[4 * nc], Sum[4]);
				_mm512_storeu_ps(&pC_Block[5 * nc], Sum[5]);
				_mm512_storeu_ps(&pC_Block[6 * nc], Sum[6]);
				_mm512_storeu_ps(&pC_Block[7 * nc], Sum[7]);
				_mm512_storeu_ps(&pC_Block[8 * nc], Sum[8]);
				_mm512_storeu_ps(&pC_Block[9 * nc], Sum[9]);
				_mm512_storeu_ps(&pC_Block[10 * nc], Sum[10]);
				_mm512_storeu_ps(&pC_Block[11 * nc], Sum[11]);
				_mm512_storeu_ps(&pC_Block[12 * nc], Sum[12]);
				_mm512_storeu_ps(&pC_Block[13 * nc], Sum[13]);
				_mm512_storeu_ps(&pC_Block[14 * nc], Sum[14]);
				_mm512_storeu_ps(&pC_Block[15 * nc], Sum[15]);
			}
			else
			{
				if (iRemain_y > iBlock_Size)
					iRemain_y = iBlock_Size;

				if (iRemain_x >= iBlock_Size)
				{
					for (int y = 0; y < iRemain_y; y++)
						_mm512_storeu_ps(&pC_Block[y * nc], Sum[y]);
				}
				else
				{
					for (int y = 0; y < iRemain_y; y++)
						_mm512_mask_storeu_ps(&pC_Block[y * nc], iMask, Sum[y]);
				}
			}
		}
	}
	return;
}

void Matrix_Multiply_float_1(float A[], int ma, int na, float B[], int nb, float C[])
{
	float* pA1 = NULL, * pB1 = NULL;
	const int iBlock_Size = 16;
	int ma1 = ((ma + iBlock_Size - 1) / iBlock_Size) * iBlock_Size,
		na1 = ((na + iBlock_Size - 1) / iBlock_Size) * iBlock_Size,
		nb1 = ((nb + iBlock_Size - 1) / iBlock_Size) * iBlock_Size;

	unsigned int iSize = ALIGN_SIZE_128(ma1 * na1 * sizeof(float)) +
		na1 * nb1 * sizeof(float);
	pA1 = (float*)pMalloc(iSize);
	pB1 = (float*)((unsigned char*)pA1 + ALIGN_SIZE_128(ma1 * na1 * sizeof(float)));

	Reshape_Row_Mul_Align(A, ma, na, iBlock_Size, &pA1);
	Reshape_Col_Mul_Align(B, na, nb, iBlock_Size, &pB1);
	//Disp(A, ma, na, "A");
	//Disp(B, na, nb, "B");
	Matrix_Multiply_Reshape_float(pA1, ma1, na1, pB1, nb1, C, ma, nb);
	//Matrix_Multiply_Reshape<float>(pA1, ma1, na1, pB1, nb1, C, ma, nb,iBlock_Size);
	if (pA1)
		Free(pA1);
	return;
}

void Transpose_Multiply_AVX_double(double A[], int m, int n, double B[])
{//求AA'
	int n1 = (n >> 3) << 3,
		iMask = (1 << (n - n1)) - 1;
	for (int y = 0; y < m; y++)
	{
		double* pA_Cur = &A[y * n];
		double* pAt_Cur = pA_Cur;
		for (int y1 = y; y1 < m; y1++)
		{
			__m512d Sum_8 = {}, A_8, At_8;
			int x;
			for (x = 0; x < n1; x += 8)
			{
				A_8 = _mm512_load_pd(&pA_Cur[x]);
				At_8 = _mm512_load_pd(&pAt_Cur[x]);
				Sum_8 = _mm512_add_pd(Sum_8, _mm512_mul_pd(A_8, At_8));
			}
			if (iMask)
			{
				A_8 = _mm512_load_pd(&pA_Cur[x]);
				At_8 = _mm512_load_pd(&pAt_Cur[x]);
				Sum_8 = _mm512_add_pd(Sum_8, _mm512_maskz_mul_pd(iMask, A_8, At_8));
			}
			//_mm_prefetch((const char*)(pAt_Cur + n), _MM_HINT_T0);
			B[y1 * m + y] = B[y * m + y1] = _mm512_reduce_add_pd(Sum_8);
			pAt_Cur += n;
		}
	}
	return;
}

void Transpose_Multiply_AVX_float(float A[], int m, int n, float B[])
{//求AA'
	int n1 = (n >> 4) << 4,
		iMask = (1 << (n - n1)) - 1;

	for (int y = 0; y < m; y++)
	{
		float* pA_Cur = &A[y * n];
		float* pAt_Cur = pA_Cur;
		for (int y1 = y; y1 < m; y1++)
		{
			__m512 Sum_16 = {}, A_16, At_16;
			int x;
			for (x = 0; x < n1; x += 16)
			{
				A_16 = _mm512_load_ps(&pA_Cur[x]);
				At_16 = _mm512_load_ps(&pAt_Cur[x]);
				Sum_16 = _mm512_add_ps(Sum_16, _mm512_mul_ps(A_16, At_16));
			}
			if (iMask)
			{
				A_16 = _mm512_load_ps(&pA_Cur[x]);
				At_16 = _mm512_load_ps(&pAt_Cur[x]);
				Sum_16 = _mm512_add_ps(Sum_16, _mm512_maskz_mul_ps(iMask, A_16, At_16));
			}
			//_mm_prefetch((const char*)(pAt_Cur + n), _MM_HINT_T0);
			B[y1 * m + y] = B[y * m + y1] = _mm512_reduce_add_ps(Sum_16);
			pAt_Cur += n;
		}
	}
	return;
}
template void Transpose_Multiply(float A[], int m, int n, float B[], int bAAt);
template void Transpose_Multiply(double A[], int m, int n, double B[], int bAAt);
template<typename _T> void Transpose_Multiply(_T A[], int m, int n, _T B[], int bAAt)
{//iFlag=0 时 B = A'A iFlag=1 时 B= AA'
	_T* At = NULL;;
	_T* B1;
	if (!bAAt)
	{//A'A 的情况，先转置，再用AA'
		At = (_T*)pMalloc(n * m * sizeof(_T));
		Matrix_Transpose(A, m, n, At);
		//再调用一次，懒得写
		Transpose_Multiply(At, n, m, B, 1);	//AtA
		Free(At);
	}
	else
	{//此处尝试优化一下，用A的行来推进
		int x, y;
		if (A == B)
			B1 = (_T*)pMalloc(m * m * sizeof(_T));
		else
			B1 = B;

		if (!B1)
		{
			printf("Invalid B1 in Transpose_Multiply\n");
			return;
		}

		if (typeid(_T) == typeid(double) && n >= 8)
			Transpose_Multiply_AVX_double((double*)A, m, n, (double*)B1);
		else if (typeid(_T) == typeid(float) && n >= 16)
			Transpose_Multiply_AVX_float((float*)A, m, n, (float*)B1);
		else
		{
			//此处应该利用对称性减少计算量
			for (y = 0; y < m; y++)
			{
				for (x = y; x < m; x++)
				{	//(y,x)为目标地址
					int iPos_r = y * n,
						iPos_c = x * n;
					_T fValue = 0;
					for (int x1 = 0; x1 < n; x1++)
						fValue += A[iPos_r++] * A[iPos_c++];
					B1[x * m + y] = B1[y * m + x] = fValue;
				}
			}
		}

		if (A == B)
		{
			memcpy(B, B1, m * m * sizeof(_T));
			Free(B1);
		}
	}
}

template<typename _T> void Matrix_Multiply(_T* A, int ma, int na, _T* B, int nb, _T* C)
{//Amn x Bno = Cmo
	_T* C_Dup;
	if (C == A || C == B)
	{
		C_Dup = (_T*)pMalloc(ma * nb * sizeof(_T));
		if (!C_Dup)
		{
			printf("Fail to Malloc_1 in Matrix_Multiply\n");
			return;
		}
	}
	else
		C_Dup = C;

	int bAVX = 1;
	if (nb >= 64 / sizeof(_T))
	{
		if (std::is_same<_T, float>::value)
			Matrix_Multiply_float_1((float*)A, ma, na, (float*)B, nb, (float*)C_Dup);
		/*else if (std::is_same<_T, double>::value)
			Matrix_Multiply_double_1((double*)A, ma, na, (double*)B, nb, (double*)C_Dup);*/
		else
			bAVX = 0;
	}
	else
		bAVX = 0;

	if (!bAVX)
	{
		int y, x, i;;
		_T fValue;
		for (y = 0; y < ma; y++)
		{//从规模上看，此处计算是大头，只优化内存顺序并不一定有好结果
			for (x = 0; x < nb; x++)
			{
				for (fValue = 0, i = 0; i < na; i++)
					fValue += A[y * na + i] * B[i * nb + x];
				C_Dup[y * nb + x] = fValue;
			}
		}
	}
	//Disp(C_Dup, ma, na);
	if (C == A || C == B)
	{
		memcpy(C, C_Dup, ma * nb * sizeof(_T));
		Free(C_Dup);
	}
	return;
}

template void Matrix_Multiply_3x1(float A[3 * 3], float B[3], float C[3]);
template void Matrix_Multiply_3x1(double A[3 * 3], double B[3], double C[3]);
template<typename _T>void Matrix_Multiply_3x1(_T A[3 * 3], _T B[3], _T C[3])
{//矩阵乘以列向量，得3x1列向量
	if (B == C)
	{
		_T C1[3];
		C1[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
		C1[1] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
		C1[2] = A[6] * B[0] + A[7] * B[1] + A[8] * B[2];
		C[0] = C1[0], C[1] = C1[1], C[2] = C1[2];
	}
	else
	{
		C[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
		C[1] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
		C[2] = A[6] * B[0] + A[7] * B[1] + A[8] * B[2];
	}
	return;
}

template void Matrix_Multiply_3x3(float A[3 * 3], float B[3 * 3], float C[3 * 3]);
template void Matrix_Multiply_3x3(double A[3 * 3], double B[3 * 3], double C[3 * 3]);
template<typename _T>void Matrix_Multiply_3x3(_T A[3 * 3], _T B[3 * 3], _T C[3 * 3])
{//计算C=AxB
 //Light_Ptr oPtr = oMatrix_Mem;
	_T* C_1, C_2[9];
	C_1 = A == C || B == C ? C_2 : C;

	//Malloc_1(oPtr, 3 * 3 * sizeof(_T),C_1);
	C_1[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	C_1[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	C_1[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

	C_1[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	C_1[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	C_1[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

	C_1[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	C_1[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	C_1[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

	if (A == C || B == C)
		memcpy(C, C_1, 9 * sizeof(_T));
	return;
}

#endif
template void Copy_Matrix_Partial(float Source[], int m, int n, float Dest[], int iDest_Stride, int x, int y);
template void Copy_Matrix_Partial(double Source[], int m, int n, double Dest[], int iDest_Stride, int x, int y);
template<typename _T>void Copy_Matrix_Partial(_T Source[], int m, int n, _T Dest[], int iDest_Stride, int x, int y)
{//将整个Source抄到 Dest对应位置上
	int x1, y1;
	//此处还得改进，要象Place_Image那样来考虑各种情况，现在太粗糙
	for (y1 = 0; y1 < m; y1++)
		for (x1 = 0; x1 < n; x1++)
			Dest[(y1 + y) * iDest_Stride + x1 + x] = Source[y1 * n + x1];
	return;
}

template void Copy_Matrix_Partial(float Source[], int iSource_Stride, int x, int y, float Dest[], int m, int n);
template void Copy_Matrix_Partial(double Source[], int iSource_Stride, int x, int y, double Dest[], int m, int n);
template<typename _T>void Copy_Matrix_Partial(_T Source[], int iSource_Stride, int x, int y, _T Dest[], int m, int n)
{//刚好相反，将矩阵得一部分拷贝到目标矩阵上
	int x1, y1;
	for (y1 = 0; y1 < m; y1++)
		for (x1 = 0; x1 < n; x1++)
			Dest[y1 * n + x1] = Source[(y1 + y) * iSource_Stride + x1 + x];
}

template void Add_I_Matrix(float A[], int iOrder, float ramda);
template void Add_I_Matrix(double A[], int iOrder, double ramda);
template<typename _T>void Add_I_Matrix(_T A[], int iOrder, _T ramda)
{//专用于列文-马夸方法，给定的A矩阵加上一个I阵
	int i;
	for (i = 0; i < iOrder; i++)
		A[i * iOrder + i] += ramda;
	return;
}

template<typename _T>void Matrix_Minus(_T A[], _T B[], int iOrder, _T C[])
{//矩阵加法，此处是方阵
	int i;
	for (i = 0; i < iOrder * iOrder; i++)
		C[i] = A[i] - B[i];
	return;
}

template void Matrix_Add(float A[], float B[], int iOrder, float C[]);
template void Matrix_Add(double A[], double B[], int iOrder, double C[]);
template<typename _T>void Matrix_Add(_T A[], _T B[], int iOrder, _T C[])
{//矩阵加法，此处是方阵
	int i;
	for (i = 0; i < iOrder * iOrder; i++)
		C[i] = A[i] + B[i];
	return;
}

//*****************一组向量操作********************************/
template void Vector_Minus(float A[], float B[], int n, float C[]);
template void Vector_Minus(double A[], double B[], int n, double C[]);
template<typename _T>void Vector_Minus(_T A[], _T B[], int n, _T C[])
{
	for (int i = 0; i < n; i++)
		C[i] = A[i] - B[i];
}

template void Vector_Add(float A[], float B[], int n, float C[]);
template void Vector_Add(double A[], double B[], int n, double C[]);
template<typename _T>void Vector_Add(_T A[], _T B[], int n, _T C[])
{
	for (int i = 0; i < n; i++)
		C[i] = A[i] + B[i];
}

template void Vector_Multiply(float A[], int n, float B[], float C[]);
template void Vector_Multiply(double A[], int n, double B[], double C[]);
template<typename _T>void Vector_Multiply(_T A[], int n, _T B[], _T C[])
{//对应位置相乘，安全函数
	for (int i = 0; i < n; i++)
		C[i] = A[i] * B[i];
}

template void Vector_Multiply(float A[], int n, float a, float B[]);
template void Vector_Multiply(double A[], int n, double a, double B[]);
template<typename _T>void Vector_Multiply(_T A[], int n, _T a, _T B[])
{//B = aA
	for (int i = 0; i < n; i++)
		B[i] = A[i] * a;
}

template float fGet_Theta_2D(float v0[], float v1[]);
template<typename _T>_T fGet_Theta_2D(_T v0[], _T v1[])
{//取值范围 (-pi, pi)，右手法则
	_T fDot_Product = fDot(v0, v1, 2),
		fCross_Product = fCross_Product_2D(v0, v1);
	return (_T)atan2(fCross_Product, fDot_Product);
}

template float fDot(float V0[], float V1[], int iDim);
template<typename _T>_T fDot(_T V0[], _T V1[], int iDim)
{//求内积, a.b = |a|.|b|.cos(theta)
	_T fTotal = 0;
	for (int i = 0; i < iDim; i++)
		fTotal += V0[i] * V1[i];
	return fTotal;
}

template void Cross_Product(float V0[], float V1[], float V2[]);
template void Cross_Product(double V0[], double V1[], double V2[]);
template<typename _T>void Cross_Product(_T V0[], _T V1[], _T V2[])
{//计算叉乘，外积，向量积， V2=V0xV1，只干三维 a×b=（aybz-azby)i + (azbx-axbz)j + (axby-aybx)k
	_T Temp[3];
	Temp[0] = V0[1] * V1[2] - V0[2] * V1[1];
	Temp[1] = V0[2] * V1[0] - V0[0] * V1[2];
	Temp[2] = V0[0] * V1[1] - V0[1] * V1[0];
	V2[0] = Temp[0], V2[1] = Temp[1], V2[2] = Temp[2];
	return;
}

template float fCross_Product_2D(float V0[], float V1[]);
template double fCross_Product_2D(double V0[], double V1[]);
template<typename _T>_T fCross_Product_2D(_T V0[], _T V1[])
{
	return V0[0] * V1[1] - V0[1] * V1[0];
}

template float fGet_Sqr_Sum(float V[], int n);
template double fGet_Sqr_Sum(double V[], int n);
template<typename _T>_T fGet_Sqr_Sum(_T V[], int n)
{
	_T fSum;
	int i;
	for (fSum = 0, i = 0; i < n; i++)
		fSum += V[i] * V[i];
	return fSum;
}
template float fGet_Mod(float V[], int n);
template double fGet_Mod(double V[], int n);
template<typename _T> _T fGet_Mod(_T V[], int n)
{
	return sqrt(fGet_Sqr_Sum(V, n));
}
template int bIs_Finite(float V[], int n);
template int bIs_Finite(double V[], int n);
template<typename _T>int bIs_Finite(_T V[], int n)
{//测试一组数据是否存在烂数据
	for (int i = 0; i < n; i++)
		if (!isfinite(V[i]))
			return 0;
	return 1;
}
template void Normalize(float V[], int n, float V_1[]);
template void Normalize(double V[], int n, double V_1[]);
template<typename _T> void Normalize(_T V[], int n, _T V_1[])
{//对向量进行规格化
//并非每种向量都能规格化，当向量是0向量时，规定还是0向量
	_T fMod;
	int i;
#define eps 1e-10
	fMod = (_T)fGet_Mod(V, n);
	if (fMod > eps)
	{
		for (i = 0; i < n; i++)
			V_1[i] = V[i] / fMod;
	}
	else
		memset(V_1, 0, n * sizeof(_T));
#undef eps
}
//*****************一组向量操作********************************/

//******************转置函数***********************************/
#ifndef ESP32
void Transpose_AVX_float(float* A, int iStride_A, float* B, int iStride_B)
{
	union {
		struct {
			__m256 r0, r1, r2, r3, r4, r5, r6, r7;
		};
		struct {
			__m256 tt0, tt1, tt2, tt3, tt4, tt5, tt6, tt7;
		};
	};

	r0 = *(__m256*)(&A[0 * iStride_A]);
	r1 = *(__m256*)(&A[1 * iStride_A]);
	r2 = *(__m256*)(&A[2 * iStride_A]);
	r3 = *(__m256*)(&A[3 * iStride_A]);
	r4 = *(__m256*)(&A[4 * iStride_A]);
	r5 = *(__m256*)(&A[5 * iStride_A]);
	r6 = *(__m256*)(&A[6 * iStride_A]);
	r7 = *(__m256*)(&A[7 * iStride_A]);

	//此处是有作用的，但与体系有关
	_mm_prefetch((const char*)&A[8 * iStride_A], _MM_HINT_T0);
	_mm_prefetch((const char*)&A[9 * iStride_A], _MM_HINT_T0);

	//将两行的元素并在每两个一起
	__m256 t0 = _mm256_unpacklo_ps(r0, r1); // [a0, b0, a1, b1, a4, b4, a5, b5]
	__m256 t1 = _mm256_unpackhi_ps(r0, r1);
	__m256 t2 = _mm256_unpacklo_ps(r2, r3);
	__m256 t3 = _mm256_unpackhi_ps(r2, r3);
	__m256 t4 = _mm256_unpacklo_ps(r4, r5);
	__m256 t5 = _mm256_unpackhi_ps(r4, r5);
	__m256 t6 = _mm256_unpacklo_ps(r6, r7);
	__m256 t7 = _mm256_unpackhi_ps(r6, r7);

	//将两行并到每4个一起
	tt0 = _mm256_shuffle_ps(t0, t2, _MM_SHUFFLE(1, 0, 1, 0));
	tt1 = _mm256_shuffle_ps(t0, t2, _MM_SHUFFLE(3, 2, 3, 2));
	tt2 = _mm256_shuffle_ps(t1, t3, _MM_SHUFFLE(1, 0, 1, 0));
	tt3 = _mm256_shuffle_ps(t1, t3, _MM_SHUFFLE(3, 2, 3, 2));
	tt4 = _mm256_shuffle_ps(t4, t6, _MM_SHUFFLE(1, 0, 1, 0));
	tt5 = _mm256_shuffle_ps(t4, t6, _MM_SHUFFLE(3, 2, 3, 2));
	tt6 = _mm256_shuffle_ps(t5, t7, _MM_SHUFFLE(1, 0, 1, 0));
	tt7 = _mm256_shuffle_ps(t5, t7, _MM_SHUFFLE(3, 2, 3, 2));

	//每八个一起
	*(__m256*)(&B[0 * iStride_B]) = _mm256_permute2f128_ps(tt0, tt4, 0x20);
	*(__m256*)(&B[1 * iStride_B]) = _mm256_permute2f128_ps(tt1, tt5, 0x20);
	*(__m256*)(&B[2 * iStride_B]) = _mm256_permute2f128_ps(tt2, tt6, 0x20);
	*(__m256*)(&B[3 * iStride_B]) = _mm256_permute2f128_ps(tt3, tt7, 0x20);
	*(__m256*)(&B[4 * iStride_B]) = _mm256_permute2f128_ps(tt0, tt4, 0x31);
	*(__m256*)(&B[5 * iStride_B]) = _mm256_permute2f128_ps(tt1, tt5, 0x31);
	*(__m256*)(&B[6 * iStride_B]) = _mm256_permute2f128_ps(tt2, tt6, 0x31);
	*(__m256*)(&B[7 * iStride_B]) = _mm256_permute2f128_ps(tt3, tt7, 0x31);
}
template<typename _T>void Transpose_AVX_4_byte(_T A[], int m, int n, _T B[])
{//对于4字节数据类型，用8x8块
	int m1 = (m / 8) * 8, n1 = (n / 8) * 8;
	int x, y;

	for (x = 0; x < n1; x += 8)
	{
		for (y = 0; y < m1; y += 8)
			Transpose_AVX_float((float*)&A[y * n + x], n, (float*)&B[x * m + y], m);

		//朴下边
		for (; y < m; y++)	//剩下的行直接搞算了
			for (int x1 = x; x1 < x + 8; x1++)
				B[x1 * m + y] = A[y * n + x1];
	}
	//Disp(B, n, m, "B");

	//右边所剩
	for (; x < n; x++)
		for (y = 0; y < m; y++)
			B[x * m + y] = A[y * n + x];
	return;
}

void Transpose_AVX_double(const double* A, int iStride_A, double* B, int iStride_B)
{
	//装入4x4数据
	__m256d r0, r1, r2, r3;
	r0 = _mm256_load_pd(A + 0 * iStride_A);
	r1 = _mm256_load_pd(A + 1 * iStride_A);
	r2 = _mm256_load_pd(A + 2 * iStride_A);
	r3 = _mm256_load_pd(A + 3 * iStride_A);

	//两个并一组
	__m256d t0 = _mm256_unpacklo_pd(r0, r1);
	__m256d t1 = _mm256_unpackhi_pd(r0, r1);
	__m256d t2 = _mm256_unpacklo_pd(r2, r3);
	__m256d t3 = _mm256_unpackhi_pd(r2, r3);

	_mm256_store_pd(B + 0 * iStride_B, _mm256_permute2f128_pd(t0, t2, 0x20));
	_mm256_store_pd(B + 1 * iStride_B, _mm256_permute2f128_pd(t1, t3, 0x20));
	_mm256_store_pd(B + 2 * iStride_B, _mm256_permute2f128_pd(t0, t2, 0x31));
	_mm256_store_pd(B + 3 * iStride_B, _mm256_permute2f128_pd(t1, t3, 0x31));
}

template<typename _T>void Transpose_AVX_8_byte(_T A[], int m, int n, _T B[])
{
	const int iBlock_Size = 4;
	int m1 = (m / iBlock_Size) * iBlock_Size,
		n1 = (n / iBlock_Size) * iBlock_Size;
	int x, y;

	//很奇怪，轮到8字节数据，则行优先更优
	for (y = 0; y < m1; y += iBlock_Size)
	{
		for (x = 0; x < n1; x += iBlock_Size)
			Transpose_AVX_double((double*)&A[y * n + x], n, (double*)&B[x * m + y], m);

		for (; x < n; x++)
			for (int y1 = y; y1 < y + iBlock_Size; y1++)
				B[x * m + y1] = A[y1 * n + x];
	}
	for (; y < m; y++)
		for (x = 0; x < n; x++)
			B[x * m + y] = A[y * n + x];

	return;
}
#endif

template<typename _T>void Matrix_Transpose(_T* A, int ma, int na, _T* At)
{//矩阵转置，安全函数，源可以等于目标
	int y, x;
	_T* At_1;
	if (abs((int)(A - At)) < ma * na)
		At_1 = (_T*)pMalloc(ma * na * sizeof(_T));
	else
		At_1 = At;

#ifndef ESP32
	if (ma >= 8 && na >= 8)
	{
		if (sizeof(_T) == 4)		//快接近3倍
			Transpose_AVX_4_byte(A, ma, na, At_1);
		else if (sizeof(_T) == 8)	//快一辈多
			Transpose_AVX_8_byte(A, ma, na, At_1);
		else
			printf("Not Implemented\n");
	}
	else
#endif
	{
		for (y = 0; y < ma; y++)
			for (x = 0; x < na; x++)
				At_1[x * ma + y] = A[y * na + x];
	}

	if (abs((int)(A - At)) < ma * na)
	{
		memcpy(At, At_1, ma * na * sizeof(_T));
		Free(At_1);
	}
	return;
}

template<typename _T>void Schmidt_Orthogon(_T* A, int m, int n, _T* B)
{//m行n列，m个向量正交化
	_T fValue, * bi, * pB = (_T*)malloc(m * n * sizeof(_T));
	int i, j, k;
	//Disp(A, m, n);
	for (i = 0; i < m; i++)
	{//每次求一个Bi
		bi = &pB[i * n];
		for (j = 0; j < n; j++)
			bi[j] = A[i * n + j];	//bi=ai;
		for (j = 0; j < i; j++)
		{
			fValue = fDot(&pB[j * n], &A[i * n], n);
			fValue /= fDot(&pB[j * n], &pB[j * n], n);
			for (k = 0; k < n; k++)
				bi[k] -= fValue * pB[j * n + k];
		}
	}

	//再单位化
	for (i = 0; i < m; i++)
	{
		bi = &pB[i * n];
		Normalize(bi, n, bi);
		//Disp(bi, 1, n);
	}
	memcpy(B, pB, m * n * sizeof(_T));
	free(pB);
	return;
}
//******************转置函数***********************************/
/********************一组矩阵类型判断函数***********************/
template<typename _T>int bIs_Orthogonal(_T* A, int h, int w)
{//判断一个矩阵是否为正交，本来正交矩阵是方阵，但是可以进一步放宽扩展，让它成为一般矩阵，
	int y, x, iMin;
	_T* At, * S;
	if (w == 0)
		w = h;
	//Malloc_1(oPtr, w * h * sizeof(_T), At);
	At = (_T*)pMalloc(w * h * sizeof(_T));
	if (!At)
		return 0;
	Matrix_Transpose(A, h, w, At);
	iMin = Min(h, w);
	//Malloc_1(oPtr, iMin * iMin * sizeof(_T), S);
	S = (_T*)pMalloc(iMin * iMin * sizeof(_T));
	if (w > h)	//宽大于高
		Matrix_Multiply(A, h, w, At, h, S);
	else
		Matrix_Multiply(At, w, h, A, w, S);
	//Disp(S, iMin, iMin, "S");
	for (y = 0; y < iMin; y++)
	{
		for (x = 0; x < iMin; x++)
		{
			if (y == x)
			{//对角线
				if (abs(S[y * iMin + x] - 1) > ZERO_APPROCIATE)
					return 0;
			}
			else
			{
				if (abs(S[y * iMin + x]) > ZERO_APPROCIATE)
					return 0;
			}
		}
	}
	Free(At);
	Free(S);
	return 1;
}

template<typename _T>int bIs_R(_T R[9], _T eps)
{
	_T Total[3];
	Total[0] = R[0] * R[1] + R[3] * R[4] + R[6] * R[7];
	Total[1] = R[0] * R[2] + R[3] * R[5] + R[6] * R[8];
	Total[2] = R[1] * R[2] + R[4] * R[5] + R[7] * R[8];
	if (!bIs_Orthogonal(R, 3, 3))
	{
		printf("非正交矩阵\n");
		return 0;
	}
	if (abs(Total[0]) > eps || abs(Total[1]) > eps || abs(Total[2]) > eps)
	{
		Disp(Total, 1, 3, "Not regid transform");
		return 0;
	}
	return 1;
}
template int bIs_Symmetric_Matrix(float A[], int iOrder, const float eps);
template int bIs_Symmetric_Matrix(double A[], int iOrder, const double eps);
template<typename _T>int bIs_Symmetric_Matrix(_T A[], int iOrder, const _T eps)
{
	int y, x, bRet = 1;
	float fA, fB, fMin, fDiff;
	for (y = 0; y < iOrder; y++)
	{
		for (x = 0; x < iOrder; x++)
		{
			fA = (float)abs(A[y * iOrder + x]);
			fB = (float)abs(A[x * iOrder + y]);
			fMin = Min(fA, fB);
			fDiff = abs(fA - fB);
			if (fDiff / fMin > eps)
			{
				if (std::is_same<_T, float>::value)
				//if (typeid(_T) == typeid(float))
					printf("%f %f\n", (float)A[y * iOrder + x], (float)A[x * iOrder + y]);
				if (std::is_same<_T, double>::value)
				//else if (typeid(_T) == typeid(double))
					printf("%lf %lf\n", (double)A[y * iOrder + x], (double)A[x * iOrder + y]);
				bRet = 0;
			}
		}
	}
	return bRet;
}
/********************一组矩阵类型判断函数***********************/
//*******************线性方程***********************************/
template void Test_Linear(float A[], int iOrder, float X[], float B[], float* pfError_Sum);
template void Test_Linear(double A[], int iOrder, double X[], double B[], double* pfError_Sum);
template<typename _T>void Test_Linear(_T A[], int iOrder, _T X[], _T B[],  _T* pfError_Sum)
{//验算Ax=b
	_T* B1 = (_T*)pMalloc(iOrder * sizeof(_T));
	_T fError_Sum;
	Matrix_Multiply(A, iOrder, iOrder, X, 1, B1);
	if (B)
		fError_Sum = fGet_Distance(B1, B, iOrder);
	else
		fError_Sum = fGet_Mod(B1, iOrder);
	printf("Error sum:%f\n", fError_Sum);
	if (B1)
		Free(B1);
	if (pfError_Sum)
		*pfError_Sum += fError_Sum;
	return;
}

template void Test_Linear_Contradictory(float A[], int m, int n, float X[], float B[], float* pfError_Sum);
template void Test_Linear_Contradictory(double A[], int m, int n, double X[], double B[], double* pfError_Sum);
template<typename _T>void Test_Linear_Contradictory(_T A[], int m, int n, _T X[], _T B[], _T* pfError_Sum)
{
	_T* pB1 = (_T*)pMalloc(m * sizeof(_T));
	int bAllocate = 0;
	if (!B)
	{
		B = (_T*)pMalloc(m * sizeof(_T));
		memset(B, 0, m * sizeof(_T));
		bAllocate = 1;
	}

	if (!pB1)
	{
		printf("Fail to allocate memory in Test_Linear_Contradictory\n");
		return;
	}
	Matrix_Multiply(A, m, n, X, 1, pB1);
	_T fSum = fGet_Distance(pB1, B, m);
	printf("Error Sum:%f\n", fSum);
	if(pfError_Sum)
		*pfError_Sum = fSum;

	if (bAllocate)
		Free(B);
	Free(pB1);
	return;
}

template<typename _T>void Elementary_Row_Operation_1(_T A[], int m, int n, _T A_2[], int* piRank, _T** ppBasic_Solution, _T** ppSpecial_Solution)
{//这个与Elementary_Row_Operation有什么区别忘了 :)

	typedef struct Q_Item {
		unsigned short m_iRow_Index;	//当前列对应的列主元所在行索引
		unsigned short m_iCol_Index;	//列主元对应的列索引，x索引
	}Q_Item;

	int y, x, x_1, i, iRank = 0, iPos, iMax;
	Q_Item* Q = NULL, iTemp;
	_T* pBasic_Solution = NULL, * pSpecial_Solution = NULL;
	short* pMap_Row_2_x_Index = NULL, * pMap_x_2_Basic_Solution_Index = NULL;
	int j, iRank_Basic_Solution;

	_T fValue, fMax, * A_1 = NULL;
	union {
		_T* pfMax_Row;
		_T* pfBottom_Row;
		_T* pfCur_Row;
	};
	if (piRank)
		*piRank = 0;
	if (m > 65535)
	{
		printf("Too large row count:%d\n", m);
		goto END;
	}
	A_1 = (_T*)pMalloc(m * n * sizeof(_T));
	Q = (Q_Item*)pMalloc(m * sizeof(Q_Item));
	if (A_1)
		memcpy(A_1, A, m * n * sizeof(_T));
	iPos = 0;
	for (y = 0; y < m; y++)
		Q[y] = { (unsigned short)y };	//每次主元所在的行

	//Disp(A_1, m, n, "\n");
	for (x_1 = 0, y = 0; y < m; y++)
	{//这个方法y与x独立推进，各不相干
		while (1)
		{
			iMax = y;
			fMax = A_1[Q[iMax].m_iRow_Index * n + x_1];
			for (i = y + 1; i < m; i++)
			{
				if (abs(A_1[iPos = Q[i].m_iRow_Index * n + x_1]) > abs(fMax))
				{
					fMax = A_1[iPos];
					iMax = i;
				}
			}
			if (abs(fMax) <= ZERO_APPROCIATE && x_1 < n - 1)
				x_1++;
			else
				break;
		}

		if (abs(fMax) < ZERO_APPROCIATE)
		{//列主元为0，显然不满秩，该方程没有唯一解
			//Disp(A_1, m, n,"\n");
			break;
		}

		//将最大元SWAP到Q的当前位置上
		iTemp = Q[y];
		Q[y] = Q[iMax];
		Q[iMax] = iTemp;
		Q[y].m_iCol_Index = x_1;
		iRank++;

		//Disp(A_1, m, n, "\n");
		pfMax_Row = &A_1[Q[y].m_iRow_Index * n];
		pfMax_Row[x_1] = 1.f;
		for (x = x_1 + 1; x < n; x++)
			pfMax_Row[x] /= fMax;
		//Disp(A_1, m, n, "\n");

		//对后面所有行代入
		for (i = y + 1; i < m; i++)
			//for (i = 0; i < m; i++)
		{//i表示第i行
			iPos = Q[i].m_iRow_Index * n;
			if (((fValue = A_1[iPos + x_1]) != 0) && i != y)
			{//对于对应元不为0才有算的意义
				for (x = x_1; x < n; x++)
					A_1[iPos + x] -= fValue * pfMax_Row[x];
				A_1[iPos + x_1] = 0;	//此处也不是必须的，置零只是好看
			}
			//Disp(Ai, iOrder, iOrder + 1, "\n");
		}
		//Disp(A_1, m, n, "\n");
		x_1++;
	}

	//Disp(A_1, m, n,"初等行变换");
	int y1;	//已经得知矩阵的秩
	//然后顺着最下一行向上变换，这段跟高斯解线性方程不一样
	//完成以后A_1将变成最简形
	for (y = iRank - 1; y > 0; y--)
	{//逻辑上从最下一行向上，实际上由Q指路
		pfBottom_Row = &A_1[Q[y].m_iRow_Index * n];
		x_1 = Q[y].m_iCol_Index;	//前面已经得到该行列主元位置

		for (y1 = y - 1; y1 >= 0; y1--)
		{
			//iPos = Q[y_1] * iRow_Size;
			iPos = Q[y1].m_iRow_Index * n;
			x = x_1;	//上面行的x位置
			fValue = A_1[iPos + x];
			A_1[iPos + x] = 0;
			for (x++; x < n; x++)
				A_1[iPos + x] -= fValue * pfBottom_Row[x];
			//Disp(A_1, m, n, "\n");
		}
	}
	//Disp(A_1, m, n, "A_1");
	if (piRank)
		*piRank = iRank;
	if (A_2)
		memcpy(A_2, A_1, m * n * sizeof(_T));
	iRank_Basic_Solution = n - 1 - iRank;	//基础解系的秩
	if (!ppBasic_Solution || !ppSpecial_Solution)
		goto END;

	//最后一步，搞齐次方程组基础解系，按照理论用列向量，为n-1维列向量，共n-1- Rank个
	pBasic_Solution = (_T*)pMalloc(iRank_Basic_Solution * n * sizeof(_T));
	//以下为给定的解x对应哪个解向量，一个Map
	pMap_Row_2_x_Index = (short*)pMalloc((n - 1) * sizeof(short));
	pMap_x_2_Basic_Solution_Index = (short*)pMalloc((n - 1) * sizeof(short));

	memset(pBasic_Solution, 0, iRank_Basic_Solution * n * sizeof(_T));
	memset(pMap_Row_2_x_Index, 0, (n - 1) * sizeof(short));
	memset(pMap_x_2_Basic_Solution_Index, 0, (n - 1) * sizeof(short));

	//先置前面列主元的x位置为-1
	for (i = 0; i < iRank; i++)
		pMap_Row_2_x_Index[Q[i].m_iCol_Index] = -1;

	//剩下的值为0的就是基础解系各向量对应的x位置
	for (j = 0, i = 0; i < n - 1; i++)
	{
		if (pMap_Row_2_x_Index[i] == 0)
		{
			pMap_x_2_Basic_Solution_Index[i] = j;
			pBasic_Solution[i * iRank_Basic_Solution + j] = 1;
			j++;
		}
	}

	//最后，构成齐次方程基础解析
	for (y = 0; y < n; y++)
	{
		iPos = Q[y].m_iRow_Index * n;
		pfCur_Row = &A_1[iPos];
		x_1 = Q[y].m_iCol_Index + 1;
		for (; x_1 < n - 1; x_1++)
		{
			if (Abs(pfCur_Row[x_1]) > ZERO_APPROCIATE)
			{
				//本来是行号，但是行号又与列号相关，既然取不到行号就拿列号
				pBasic_Solution[Q[y].m_iCol_Index * iRank_Basic_Solution + pMap_x_2_Basic_Solution_Index[x_1]] = -A_1[iPos + x_1];
			}
		}
	}

	if (pSpecial_Solution)
	{
		memset(pSpecial_Solution, 0, (n - 1) * sizeof(_T));
		for (i = 0; i < iRank; i++)
			pSpecial_Solution[Q[i].m_iCol_Index] = A_1[Q[i].m_iRow_Index * n + (n - 1)];
	}
END:
	if (A_1)
		Free(A_1);
	if (pMap_Row_2_x_Index)
		Free(pMap_Row_2_x_Index);
	if (pMap_x_2_Basic_Solution_Index)
		Free(pMap_x_2_Basic_Solution_Index);
	if (ppBasic_Solution)
		*ppBasic_Solution = pBasic_Solution;
	if (pBasic_Solution && !ppBasic_Solution)
		Free(pBasic_Solution);
	if (ppSpecial_Solution)
		*ppSpecial_Solution = pSpecial_Solution;
	if (pSpecial_Solution && !ppSpecial_Solution)
		Free(pSpecial_Solution);
	if (Q)
		Free(Q);
	return;
}
template void Solve_Linear_Gause_AAt(float* A, int iOrder, float* B, float* X, int* pbSuccess);
template void Solve_Linear_Gause_AAt(double* A, int iOrder, double* B, double* X, int* pbSuccess);
template<typename _T>void Solve_Linear_Gause_AAt(_T* A, int iOrder, _T* B, _T* X, int* pbSuccess)
{//解对称矩阵方程，用上三角构成增广矩阵，快很多
//与列主元相比，在double下，即数字精度足够条件下，其与列主元法不相上下
//但在float下，数字精度不够下，与列主元法差距很大。慎用
	int y, x, i;
	union {
		int iRow_Size;
		int iPos;
	};

	_T fPivot, * pfPivot_Row;
	const _T eps = (_T)1e-10;
	int bSuccess = 1;
	_T* Ai = (_T*)pMalloc((iOrder + 1 + 2) * iOrder / 2 * sizeof(_T));
	if (!Ai)
	{
		bSuccess = 0;
		goto END;
	}
	iPos = 0;
	for (y = 0; y < iOrder; y++)
	{
		for (x = y; x < iOrder; x++, iPos++)
			Ai[iPos] = A[y * iOrder + x];
		Ai[iPos++] = B[y];
	}
	//Disp_Ai(Ai, iOrder);

	iRow_Size = iOrder + 1;
	//为了便于理解，以下iMax表示Q中的索引，而不是Ai中的行号
	pfPivot_Row = Ai;

	_T* pfCur_Row;
	int iCur_Row_Size;
	for (y = 0; y < iOrder; y++)
	{
		//主元就是对角线元素
		fPivot = *pfPivot_Row;
		if (abs(fPivot) < eps)
		{//列主元为0，显然不满秩，该方程没有唯一解
			printf("列主元为：%f\n", fPivot);
			bSuccess = 0;
			goto END;
		}
		//为了快点，尝试用倒数
		fPivot = 1.f / fPivot;

		//试一下此处算一半了事
		pfCur_Row = pfPivot_Row + iRow_Size;
		iCur_Row_Size = iRow_Size - 1;

		//此处好像有点问题，是不是应该从y+1到iOrder呢？
		//for (i = 1; i < iOrder; i++)
		for (i = 1; iCur_Row_Size; i++)
		{//i表示第i行
			union {
				_T fFactor;
				_T fRow_Head;
			};
			if (fRow_Head = pfPivot_Row[i])
			{//对于对应元不为0才有算的意义
				//fFactor = fRow_Head / fPivot;
				fFactor = fRow_Head * fPivot;
				for (x = 0; x < iCur_Row_Size; x++)
				{
					pfCur_Row[x] -= fFactor * pfPivot_Row[x + i];
				}
			}
			pfCur_Row += iCur_Row_Size;
			iCur_Row_Size--;
		}

		//pfPivot_Row[0] = 1.f;	//这个也可能不需要
		for (x = 1; x < iRow_Size; x++)
			//pfPivot_Row[x] /= fPivot;
			pfPivot_Row[x] *= fPivot;
		pfPivot_Row += iRow_Size;
		iRow_Size--;
	}
	//Disp_Ai(Ai, iOrder);	
	//回代，从Q[iOrder-1]开始回代，从最下一行向上回代
	pfCur_Row = &Ai[(iOrder + 1 + 2) * iOrder / 2 - 1];
	//第一个解
	X[iOrder - 1] = pfCur_Row[0];

	//从以下行开始，iCur_Row_Size不包括 1
	//pfCur_Row从1后的第一个元素开始
	iCur_Row_Size = 1;
	pfCur_Row -= 3;
	for (y = iOrder - 2; y >= 0; y--)
	{
		//将解向上回代
		//fValue=b
		_T fValue = pfCur_Row[iCur_Row_Size];
		for (x = 0; x < iCur_Row_Size; x++, iPos++)
			fValue -= pfCur_Row[x] * X[iOrder - iCur_Row_Size + x];
		//Ai[iPos + x] = 0;		//此处不是必须的，算完置0，好看一些而已
		X[y] = fValue;
		//Ai[iPos + iOrder] = fValue;	//此处不是必须，好看而已
		iCur_Row_Size++;
		pfCur_Row -= iCur_Row_Size + 2;	//1占一个，b占一个
	}

	//Disp(X, iOrder, 1, "X");
	//验算
	//fLinear_Equation_Check(A, iOrder, B, X, (_T)ZERO_APPROCIATE);
END:
	*pbSuccess = bSuccess;
	if (Ai)
		Free(Ai);

	return;
}

template<typename _T> void Solve_Linear_Solution_Construction_1(_T* A, int m, int n, _T B[], int* pbSuccess, _T* pBasic_Solution, int* piBasic_Solution_Count, _T* pSpecial_Solution)
{//获得一个解的结构
	//此处给增广矩阵分配内存. m行n列的增广矩阵，加一列，为 m*(n+1)个元素
	//_T* Ai = (_T*)malloc(m * (n + 1) * sizeof(_T));
	_T* Ai = (_T*)pMalloc(m * (n + 1) * sizeof(_T));
	_T* pBasic_Solution_1 = NULL, * pSpecial_Solution_1 = NULL;
	int y, x, iRow_Size = n + 1;
	int iRank = 0;	//系数矩阵的秩

	for (y = 0; y < m; y++)
	{//Ai为增广矩阵
		for (x = 0; x < n; x++)
			Ai[y * iRow_Size + x] = A[y * n + x];
		Ai[y * iRow_Size + n] = B ? B[y] : 0;
	}

	//此处进行初等行变换，而且用更靠谱的列主元
	Elementary_Row_Operation_1(Ai, m, n + 1, Ai, &iRank, &pBasic_Solution_1, &pSpecial_Solution_1);

	for (y = 0; y < m; y++)
	{
		int bIs_Zero = 1;
		for (x = 0; x < n; x++)
		{
			if (abs(Ai[y * (n + 1) + x]) > ZERO_APPROCIATE)
			{
				bIs_Zero = 0;
				break;
			}
		}
		if (bIs_Zero && Ai[y * (n + 1) + n] > ZERO_APPROCIATE)
		{
			printf("该方程无解，经过初等行变换以后，第%d行的常数项为：%f\n", y, Ai[y * (n + 1) + n]);
			Disp(Ai, m, n + 1);
			*pbSuccess = 0;
			return;
		}
	}
	int bIs_Homo = 1;
	int iBasic_Solution_Count;
	if (B)
	{
		for (y = 0; y < m; y++)
		{
			if (B[y])
			{//如果常数列b不为0，则为非齐次方程
				bIs_Homo = 0;
				break;
			}
		}
	}

	if (bIs_Homo)
	{//齐次方程组的解，形如 X= c0*v0 + c1*v1 + ... + c(n-r) * v(n-r)
		if (iRank == n)
		{
			printf("系数矩阵满秩，只有零解\n");
			//if (piBasic_Solution_Count)
				//*piBasic_Solution_Count = 0;
			iBasic_Solution_Count = 0;
		}
		else
			iBasic_Solution_Count = n - iRank;
	}
	else
		iBasic_Solution_Count = n - iRank;

	if (piBasic_Solution_Count)
		*piBasic_Solution_Count = iBasic_Solution_Count;

	if (pBasic_Solution && pBasic_Solution_1)
	{
		Matrix_Transpose(pBasic_Solution_1, n, n - iRank, pBasic_Solution_1);
		//此处有一事不明，一定要做个正交化吗？正交化会不变改变了基的夹角？
		Schmidt_Orthogon(pBasic_Solution_1, n - iRank, n, pBasic_Solution_1);
		if (iBasic_Solution_Count)
			memcpy(pBasic_Solution, pBasic_Solution_1, (n - iRank) * n * sizeof(_T));
		else
			memset(pBasic_Solution, 0, m * sizeof(_T));
	}
	if (!bIs_Homo && pSpecial_Solution)
		memcpy(pSpecial_Solution, pSpecial_Solution_1, n * sizeof(_T));

	if (pSpecial_Solution_1)
		Free(pSpecial_Solution_1);
	if (pBasic_Solution_1)
		Free(pBasic_Solution_1);
	if (Ai)
		Free(Ai);
	*pbSuccess = 1;
}

template<typename _T>void Solve_Linear_Gause(_T* A, int iOrder, _T* B, _T* X, int* pbSuccess)
{//用高斯列主元法求解线性方程组, 要点：
	//1，高斯法完全等价人肉行变换，只不过没有用人肉的公倍数法，而是步步都是将主元变为1
	//2，选列主元的原因是保证在 其他系数/aij时分母不至于太小。分母小误差大
	//3,在列主元太小（<eps)的情况下算不满秩，退出。实际上主元太小会导致后面的除法严重误差
	int y, x, i, iRow_Size;
	int iMax, iTemp, iPos, * pQ;
	_T fMax, * pfMax_Row, fValue;
	const _T eps = (_T)1e-10;
	int bSuccess = 1;
	pQ = (int*)pMalloc(iOrder * sizeof(int));
	_T* Ai = (_T*)pMalloc((iOrder + 1) * iOrder * sizeof(_T));
	if (!pQ || !Ai)
	{
		bSuccess = 0;
		goto END;
	}

	iPos = 0;
	for (y = 0; y < iOrder; y++)
	{
		for (x = 0; x < iOrder; x++, iPos++)
			Ai[iPos] = A[y * iOrder + x];
		Ai[iPos++] = B[y];
		pQ[y] = y;	//每次主元所在的行
	}

	//Disp(Ai, iOrder, iOrder + 1,"\n");
	iRow_Size = iOrder + 1;
	//为了便于理解，以下iMax表示Q中的索引，而不是Ai中的行号
	for (y = 0; y < iOrder; y++)
	{
		iMax = y;	//感觉这里错了，应该是 iMax = pQ[i]
		fMax = Ai[pQ[iMax] * iRow_Size + y];
		for (i = y + 1; i < iOrder; i++)
		{//寻找列主元
			if (abs(Ai[iPos = pQ[i] * iRow_Size + y]) > abs(fMax))
			{
				fMax = Ai[iPos];
				iMax = i;
			}
		}
		/*if (iMax != y)
			printf("%d\n", iMax);*/

		if (abs(fMax) < eps)
		{//列主元为0，显然不满秩，该方程没有唯一解
			printf("不满秩,列主元为：%f\n", fMax);
			bSuccess = 0;
			goto END;
		}

		//将最大元SWAP到Q的当前位置上
		iTemp = pQ[y];
		pQ[y] = pQ[iMax];
		pQ[iMax] = iTemp;

		//对iMax所在的行进行系数计算，新系数/=A[y][y]
		pfMax_Row = &Ai[pQ[y] * iRow_Size];
		pfMax_Row[y] = 1.f;
		for (x = y + 1; x < iRow_Size; x++)
			pfMax_Row[x] /= fMax;

		//Disp(Ai, iOrder, iOrder + 1, "\n");

		//对后面所有行代入
		for (i = y + 1; i < iOrder; i++)
		{//i表示第i行
			iPos = pQ[i] * iRow_Size;
			if ((fValue = Ai[iPos + y]) != 0)
			{//对于对应元不为0才有算的意义
				for (x = y + 1; x < iRow_Size; x++)
					Ai[iPos + x] -= fValue * pfMax_Row[x];
				Ai[iPos + y] = 0;	//此处也不是必须的，置零只是好看
			}
			//Disp(Ai, iOrder, iOrder + 1, "\n");
		}
	}

	//第一个解
	X[iOrder - 1] = Ai[pQ[iOrder - 1] * iRow_Size + iOrder];

	//回代，从Q[iOrder-1]开始回代，从最下一行向上回代
	for (y = iOrder - 2; y >= 0; y--)
	{
		//将解向上回代
		iPos = pQ[y] * iRow_Size;
		//fValue=b
		fValue = Ai[iPos + iOrder];
		for (x = y + 1; x < iOrder; x++)
		{
			fValue -= Ai[iPos + x] * X[x];
			Ai[iPos + x] = 0;		//此处不是必须的，算完置0，好看一些而已
		}
		X[y] = fValue;
		Ai[iPos + iOrder] = fValue;	//此处不是必须，好看而已
	}

END:
	//Disp(Ai, iOrder, iOrder + 1, "\n");
	//*pbSuccess = 1;
	*pbSuccess = bSuccess;
	if (pQ)
		Free(pQ);
	if (Ai)
		Free(Ai);
	//验算
	//Linear_Equation_Check(A, iOrder, B, X, (_T)ZERO_APPROCIATE);
	return;
}

template void Solve_Homo_Linear_SVD(float A[], int m, int n, float X[], int* pbSuccess);
template void Solve_Homo_Linear_SVD(double A[], int m, int n, double X[], int* pbSuccess);
template<typename _T>void Solve_Homo_Linear_SVD(_T A[], int m, int n, _T X[], int* pbSuccess)
{//只用sVD Ax =0
	SVD_Info oSVD;
	SVD_Alloc(m, n, &oSVD, A);
	svd_3(A, oSVD, pbSuccess);
	SVD_Get_Solution(oSVD, X);
	Free_SVD(&oSVD);
	return;
}
template<typename _T>void Get_Diag_Max_Min(_T A[], int n, _T* pfMax=NULL, _T* pfMin=NULL)
{
	_T fAbs_Max, fAbs_Min;	//fMax, , fMin;
	//fMax = fMin = A[0];
	fAbs_Max = fAbs_Min = Abs(A[0]);

	for (int i = 1; i < n; i++)
	{
		_T fValue = abs(A[i * n + i]);
		if (fValue < fAbs_Min)
			fAbs_Min = fValue;
		else
			fAbs_Max = fValue;
	}
	if (pfMax)*pfMax = fAbs_Max;
	if (pfMin)*pfMin = fAbs_Min;
	return;
}
template<typename _T>void Adjust_AtA(_T A[], int n, _T B[],_T eps = 1e-6)
{
	_T fMax, fMin;
	Get_Diag_Max_Min(A, n, &fMax, &fMin);
	if (fMin && fMax / fMin > 1000)
		Add_I_Matrix<_T>(A, n, eps);
	return;
}

template void Solve_Linear_Contradictory(float A[], int m, int n, float B[], float X[],  int* pbSuccess, float fAdj_eps);
template void Solve_Linear_Contradictory(double A[], int m, int n, double B[], double X[], int* pbSuccess, double fAdj_eps);
template<typename _T> void Solve_Linear_Contradictory(_T A[], int m, int n, _T B[], _T X[], int* pbSuccess,_T fAdj_eps)
{//尝试用最小二乘法解矛盾方程组。关键是求 A'Ax=A'B, 若有解，则x为 Ax=B的最小二乘解
//所有拟合问题尽可能通过两边变形化为先行问题，然后用解矛盾方程组的方法来解就好办
	_T* At, * AtA, * AtB;
	int bResult = 1;

	if (B)	//非齐次方程
	{
		At = (_T*)pMalloc((m * n + n * n + n) * sizeof(_T));
		AtA = At + m * n;
		AtB = AtA + n * n;
		Matrix_Transpose(A, m, n, At);
		Transpose_Multiply(At, n, m, AtA);

		//注意，此处要用 岭回归 调整病态方程
		Adjust_AtA(AtA, n, AtA, fAdj_eps);
		Matrix_Multiply(At, n, m, B, 1, AtB);
		Solve_Linear_Gause(AtA, n, AtB, X, &bResult);
		Free(At);
	}else
	{
		//先用快速方法冲一下
		AtA = (_T*)pMalloc((n * n) * sizeof(_T));
		Transpose_Multiply(A, m, n, AtA, 0);
		if (!bInverse_Power(AtA, n, (_T*)NULL, X))
		{
			bResult = 0;
			memset(X, 0, n * sizeof(_T));
		}
		Free(AtA);

		if (!bResult)//尚未搞定，SVD做最后挣扎
			Solve_Homo_Linear_SVD(A, m, n, X, &bResult);

		//由于对反幂法了解不深，暂时没有足够的样本看出哪种更健壮。关键
		//在于1，精度谁更好；2，是否存在某些svd能分解但反幂法算不出来的
		//情况。反幂法中最大的问题是LU分解要求矩阵符合一定条件
	}

	if (pbSuccess)
		*pbSuccess = bResult;
}
//*******************线性方程***********************************/

//********************一组矩阵求逆******************************/
template void Get_Inv_Matrix_Row_Op(float* pM, float* pInv, int iOrder, int* pbSuccess);
template void Get_Inv_Matrix_Row_Op(double* pM, double* pInv, int iOrder, int* pbSuccess);
template<typename _T> void Get_Inv_Matrix_Row_Op(_T* pM, _T* pInv, int iOrder, int* pbSuccess)
{//列主元法求逆
	//由于用伴随矩阵求逆矩阵太慢，此处用初等行变换搞，其实就是高斯求方程的一种
	//用一个大Buffer装好了M再行变换
	_T* pAux;
	int y, x, i, iRow_Size = iOrder * 2, iPos, bRet = 1;
	_T fMax, * pfCur_Row, fValue;

	pAux = (_T*)pMalloc(iOrder * 2 * iOrder * sizeof(_T));
	if (!pAux)
		return;

	//初始化值
	for (y = 0; y < iOrder; y++)
	{
		iPos = y * iRow_Size;
		for (x = 0; x < iOrder; x++)
			pAux[iPos + x] = pM[y * iOrder + x];
		iPos += iOrder;
		for (x = 0; x < iOrder; x++)
			pAux[iPos + x] = y == x ? 1.f : 0.f;
	}

	//Disp(pAux, iOrder, iOrder * 2,"\n");
	const _T eps = (_T)1e-10;
	for (y = 0; y < iOrder; y++)
	{
		pfCur_Row = &pAux[y * iRow_Size];
		fMax = pAux[y * iRow_Size + y];
		if (Abs(fMax) < eps)
		{
			bRet = 0;
			printf("列主元为0\n");
			goto END;
		}
		pfCur_Row[y] = 1.f;
		for (x = y + 1; x < iRow_Size; x++)
			pfCur_Row[x] /= fMax;
		//Disp(pAux, iOrder, iOrder * 2, "\n");

		//对后面所有行代入
		for (i = y + 1; i < iOrder; i++)
		{//i表示第i行
			//iPos = Q[i] * iRow_Size;
			iPos = i * iRow_Size;
			if ((fValue = pAux[iPos + y]) != 0)
			{//对于对应元不为0才有算的意义
				for (x = y + 1; x < iRow_Size; x++)
					pAux[iPos + x] -= fValue * pfCur_Row[x];
				pAux[iPos + y] = 0;	//此处也不是必须的，置零只是好看
			}
			//Disp(pAux, iOrder, iOrder * 2, "\n");
		}
	}
	//Disp(pAux, iOrder, iOrder * 2, "\n");

	_T* pfBottom_Row;
	int y_1;
	//然后顺着最下一行向上变换，这段跟高斯解线性方程不一样
	for (y = iOrder - 1; y > 0; y--)
	{//逻辑上从最下一行向上，实际上由Q指路
		pfBottom_Row = &pAux[y * iRow_Size];
		for (y_1 = y - 1; y_1 >= 0; y_1--)
		{
			//iPos = Q[y_1] * iRow_Size;
			iPos = y_1 * iRow_Size;
			x = y;	//上面行的x位置
			fValue = pAux[iPos + x];
			pAux[iPos + x] = 0;
			for (x++; x < iRow_Size; x++)
				pAux[iPos + x] -= fValue * pfBottom_Row[x];
			//Disp(pAux, iOrder, iRow_Size, "\n");
		}
	}

	//最后抄到目标矩阵
	for (y = 0; y < iOrder; y++)
	{
		iPos = y * iRow_Size + iOrder;
		for (x = 0; x < iOrder; x++, iPos++, pInv++)
			*pInv = pAux[iPos];
	}

END:
	if (pbSuccess)
		*pbSuccess = bRet;
	Free(pAux);
	//free(pAux);
	return;
}

template<typename _T>_T fGet_Determinant(_T* A, int iOrder)
{//求行列式，M为方阵，iOrder为行列式的阶,iStrike为M的行大小，一行元素个数， Get n-order determinant
//此算法不求速度，但求验证数学原理，所以采用按行展开法则。 D=ai1*Ai1+ai2*Ai2+...+ainAin
//优化思路：对行列式用列主元法进行初等行变换，转换为上三角方阵，再求其对角线乘积
	_T* pCur, * pCofactor; //余子式
	int i, j, k;
	_T fDeterminant, fTotal;
	if (iOrder == 2)//递归到最深一层， 2阶行列式
		return A[0] * A[3] - A[1] * A[2];

	pCofactor = (_T*)pMalloc((iOrder - 1) * (iOrder - 1) * sizeof(_T)); //余子式
	A -= (iOrder + 1);	//为了与数学原理相对应，此处改变地址令下标从1开始
	//按第一行展开
	for (fTotal = 0, i = 1; i <= iOrder; i++)
	{
		//求第一行第i列元素的余子式
		for (pCur = pCofactor, j = 2; j <= iOrder; j++)
		{
			for (k = 1; k <= iOrder; k++)
			{
				if (k == i)
					continue;	//划去
				*pCur++ = A[j * iOrder + k];
			}
		}
		fDeterminant = fGet_Determinant(pCofactor, iOrder - 1);
		fTotal += (_T)(A[1 * iOrder + i] * pow(-1, 1 + i) * fDeterminant);
		//Disp((float*)pCofactor, iOrder - 1, iOrder - 1);
	}
	Free(pCofactor);
	return fTotal;
}
template<typename _T>void Gen_Cofactor(_T* pM, int iOrder, _T* pCofactor, int i, int j)
{//构造aij对应的余子式Aij, iOrrder为pM的阶
	int x, y;
	_T* pCur = pCofactor;
	for (y = 0; y < iOrder; y++)
	{
		for (x = 0; x < iOrder; x++)
		{
			if (y == i || x == j)
				continue;	//将第i行，第j列划去
			*pCur++ = pM[y * iOrder + x];
		}
	}
	return;
}
template<typename _T>void Get_Adjoint_Matrix(_T* pA, int iOrder, _T* pAdjoint_Matrix)
{//求伴随矩阵。伴随矩阵由矩阵A各元素的代数余子数组成
	_T* pCofactor = (_T*)pMalloc((iOrder - 1) * (iOrder - 1) * sizeof(_T));
	int i, j, iFlag;
	_T* pAdj = (_T*)pMalloc(iOrder * iOrder * sizeof(float));
	//Disp(pA, iOrder, iOrder);
	//构造伴随矩阵，计算iOrder*iOrder个代数余子式
	for (i = 0; i < iOrder; i++)
	{
		for (j = 0; j < iOrder; j++)
		{
			iFlag = (int)pow(-1, (i + j) % 2);
			if (iOrder >= 3)
			{
				Gen_Cofactor(pA, iOrder, pCofactor, i, j);
				//printf("%d %d\n", i, j);
				//Disp(pCofactor, iOrder - 1, iOrder - 1);
				//printf("Determinant:%f\n\n", fGet_Determinant(pCofactor, iOrder - 1));
				pAdj[i * iOrder + j] = iFlag * fGet_Determinant(pCofactor, iOrder - 1);
			}
			else
				pAdj[i * iOrder + j] = iFlag * pA[((i + 1) % 2) * 2 + (j + 1) % 2];
		}
	}

	//Disp(pAdj, iOrder, iOrder);
	memcpy(pAdjoint_Matrix, pAdj, iOrder * iOrder * sizeof(_T));
	Free(pAdj);
	Free(pCofactor);
	return;
}
template void Get_Inv_Matrix(double* pM, double* pInv, int iOrder, int* pbSuccess);
template void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess);
template<typename _T>void Get_Inv_Matrix(_T* pM, _T* pInv, int iOrder, int* pbSuccess)
{//对pM求逆矩阵，用伴随矩阵法  Inv(A)= (A*)/|A| 暂时只干3阶以上的矩阵
//虽然老，仍然有研究价值
	_T fDet = fGet_Determinant(pM, iOrder);
	int i, j;
	if (abs(fDet) < ZERO_APPROCIATE)
	{
		*pbSuccess = 0;
		return;
	}
	_T* pAdjoint_Matrix = (_T*)pMalloc(iOrder * iOrder * sizeof(_T));
	Get_Adjoint_Matrix(pM, iOrder, pAdjoint_Matrix);
	//至此，伴随矩阵已经OK，放在pAdjoint_Matrix里，然而，最终结果不能跟教科书
	//准确的算法应该是 A(-1)= (A*)'/|A|	, 伴随矩阵要转置一下方可
	for (i = 0; i < iOrder; i++)
		for (j = 0; j < iOrder; j++)
			pInv[i * iOrder + j] = pAdjoint_Matrix[j * iOrder + i] / fDet;
	*pbSuccess = 1;

	Free(pAdjoint_Matrix);
	return;
}
//********************一组矩阵求逆******************************/

//********************一组LU分解********************************/
template void Cholosky_Decompose(float A[], int iOrder, float B[], int* pbSuccess);
template void Cholosky_Decompose(double A[], int iOrder, double B[], int* pbSuccess);
template<typename _T>void Cholosky_Decompose(_T A[], int iOrder, _T B[], int* pbSuccess)
{//对正定矩阵做Cholosky分解，结果放在啊B A=BxB'
//小范围测试没问题
//注意，A必须对称正定，对称未必一定就是正定，此时分解失败
	int i, j, k, iPos_kk, iPos_ik, iPos_ij, iPos_jk;
	_T* B1 = (_T*)pMalloc(iOrder * iOrder * sizeof(_T));
	_T d = 1;
	memcpy(B1, A, iOrder * iOrder * sizeof(_T));
	const _T eps = (_T)1e-10;

	for (k = 0; k < iOrder; k++)
	{
		iPos_kk = k * iOrder + k;
		d *= B1[iPos_kk];
		//if (B1[iPos_kk] <= 0.f )
		if (B1[iPos_kk] <= eps)
		{//实践发现存在对称但不正定的情况
			if (pbSuccess)
				*pbSuccess = 0;
			printf("Not positive-definite Matrix in Cholosky_Decompose \n");
			return;
		}

		B1[iPos_kk] = sqrt(B1[iPos_kk]);	//对角线等于其开方，显而易见
		for (i = k + 1; i < iOrder; i++)
		{//扫一列，这列就是刚才所算的kk列，除以上面搞出来的对角线元素
			iPos_ik = i * iOrder + k;	//似乎在搞下三角第k列
			B1[iPos_ik] /= B1[iPos_kk];
		}

		for (j = k + 1; j < iOrder; j++)
		{//对该列右边的元素计算
			iPos_jk = j * iOrder + k;
			for (i = j; i < iOrder; i++)
			{
				iPos_ij = i * iOrder + j;
				B1[iPos_ij] -= B1[i * iOrder + k] * B1[iPos_jk];
			}
		}
		//Disp(B1, iOrder, iOrder, "B1");
	}

	//显然，上三角置0
	if (B)
	{
		for (i = 0; i < iOrder; i++)
			for (j = i + 1; j < iOrder; j++)
				B1[i * iOrder + j] = 0;
		memcpy(B, B1, iOrder * iOrder * sizeof(_T));
	}
	//printf("d:%f\n", d);
	if (pbSuccess)
		*pbSuccess = 1;
	if (B1)
		Free(B1);
	return;
}

template void LLt_Decompose(float A[], int iOrder, float B[], int* pbSuccess);
template void LLt_Decompose(double A[], int iOrder, double B[], int* pbSuccess);
template<typename _T>void LLt_Decompose(_T A[], int iOrder, _T B[], int* pbSuccess)
{//对正定矩阵做LLt分解，不需要完全正定，进一步放松要求
//其实和Cholosky 分解等价
//小范围测试没问题
	int i, j, k, iPos_kk, iPos_ik, iPos_ij, iPos_jk;
	_T* B1 = (_T*)pMalloc(iOrder * iOrder * sizeof(_T));
	_T d = 1;
	memcpy(B1, A, iOrder * iOrder * sizeof(_T));
	for (k = 0; k < iOrder; k++)
	{
		iPos_kk = k * iOrder + k;
		d *= B1[iPos_kk];
		if (B1[iPos_kk] < 0.f)
		{//实践发现存在对称但不正定的情况
			if (pbSuccess)
				*pbSuccess = 0;
			printf("Invalid value in LLt_Decompose\n");
			return;
		}
		B1[iPos_kk] = sqrt(B1[iPos_kk]);	//对角线等于其开方，显而易见
		for (i = k + 1; i < iOrder; i++)
		{//扫一列，这列就是刚才所算的kk列，除以上面搞出来的对角线元素
			iPos_ik = i * iOrder + k;	//似乎在搞下三角第k列
			//还得分情况
			if (B1[iPos_ik] == 0)
			{//这里是可以不管上面的主元是否为0的，可以继续分解
			}
			else if (B1[iPos_ik] != 0.f)
				B1[iPos_ik] /= B1[iPos_kk];
			else
			{
				if (pbSuccess)
					*pbSuccess = 0;
				printf("Divided by 0 in Cholosky_Decompose\n");
				return;
			}
		}

		for (j = k + 1; j < iOrder; j++)
		{//对该列右边的元素计算
			iPos_jk = j * iOrder + k;
			for (i = j; i < iOrder; i++)
			{
				iPos_ij = i * iOrder + j;
				//if (B1[iPos_ij] != 0.f)
				B1[iPos_ij] -= B1[i * iOrder + k] * B1[iPos_jk];
				/*else
				{
					if (pbSuccess)
						*pbSuccess = 0;
					printf("Divided by 0 in Cholosky_Decompose\n");
					return;
				}*/
			}
		}
		//Disp(B1, iOrder, iOrder, "B1");
	}

	//显然，讲上三角置0
	if (B)
	{
		for (i = 0; i < iOrder; i++)
			for (j = i + 1; j < iOrder; j++)
				B1[i * iOrder + j] = 0;
		memcpy(B, B1, iOrder * iOrder * sizeof(_T));
	}
	//printf("d:%f\n", d);
	if (pbSuccess)
		*pbSuccess = 1;
	if (B1)
		Free(B1);
	return;
}

//应该没啥效果，退休了
#define LU_Decompose_1(A, L, U, bSucceed) \
{ \
	if (A[0][0] == 0) \
	{ \
		bSucceed = 0; \
		goto END; \
	} \
	U[0][0] = A[0][0]; \
	L[1][0] = A[1][0] / U[0][0]; \
	U[0][1] = A[0][1]; \
	U[1][1] = A[1][1] - L[1][0] * U[0][1]; \
	if (U[1][1] == 0) \
	{ \
		bSucceed = 0; \
		goto END; \
	} \
	L[2][0] = A[2][0] / U[0][0]; \
	L[2][1] = (A[2][1] - L[2][0] * U[0][1]) / U[1][1]; \
	U[0][2] = A[0][2]; \
	U[1][2] = A[1][2] - L[1][0] * U[0][2]; \
	U[2][2] = A[2][2] - L[2][0] * U[0][2] - L[2][1] * U[1][2]; \
	bSucceed = 1; \
END: \
	; \
}
#define  Init_LU(L, U) \
{/*初始化L(下三角） U（上三角）*/ \
	L[0][0] = L[1][1] = L[2][2] = 1; \
	L[0][1] = L[0][2] = L[1][2] = 0; \
	U[1][0] = U[2][0] = U[2][1] = 0; \
}

template<typename _T>void LU_Decompose_3x3(_T A[3][3], _T L[3][3], _T U[3][3], int* pbSucceed)
{//尝试将一个矩阵A分解为LxU=A。分解过程走位风骚，步步皆可求解
//限制：仅对非奇异矩阵（满秩）有效
	if (A[0][0] == 0)
	{
		*pbSucceed = 0;
		return;
	}
	Init_LU(L, U);
	U[0][0] = A[0][0];
	L[1][0] = A[1][0] / U[0][0];
	U[0][1] = A[0][1];
	U[1][1] = A[1][1] - L[1][0] * U[0][1];
	if (U[1][1] == 0)
	{
		*pbSucceed = 0;
		return;
	}
	L[2][0] = A[2][0] / U[0][0];
	L[2][1] = (A[2][1] - L[2][0] * U[0][1]) / U[1][1];
	U[0][2] = A[0][2];
	U[1][2] = A[1][2] - L[1][0] * U[0][2];
	U[2][2] = A[2][2] - L[2][0] * U[0][2] - L[2][1] * U[1][2];
	/*double Temp[3][3];
	Matrix_Multiply_3x3(L, U, Temp);
	Disp(Temp, "Temp");
	Disp(L, (char*)"L");
	Disp(U, (char*)"U");*/
	if (pbSucceed)
		*pbSucceed = 1;
	return;
}
template<typename _T>void LU_Decompose(_T A[], _T L[], _T U[], int n, int* pbSuccess)
{//一般LU分解，L出来符合单位下三角，具有唯一性
	int i, j, k, r;
	int iResult = 1;
	memset(L, 0, n * n * sizeof(_T));
	memset(U, 0, n * n * sizeof(_T));
	for (i = 0; i < n; i++)
		L[i * n + i] = 1;

	for (i = 0; i < n; i++/*, iFlag_rc = ~iFlag_rc*/)
	{
		_T fValue;

		//先搞第i列,向下搞
		for (j = 0; j <= i; j++)
		{
			fValue =/* aji =*/ A[j * n + i];
			//if (j == 1 && i == 1)
				//printf("here");
			//以此aij作为引导，算出一个uij
			for (k = 0; k < j; k++)
				fValue -= L[j * n + k] * U[k * n + i];
			U[j * n + i] = fValue;
		}

		//再搞第i+1行,向右搞
		r = i + 1;
		if (r >= n)
			break;  //结束了，后面没了

		for (j = 0; j < r; j++)
		{//向右只算L元素，
			fValue = /*arj =*/ A[r * n + j];
			/*if (r == 2 && j == 1)
				printf("here");*/
			for (k = 0; k < j; k++)
				fValue -= L[r * n + k] * U[k * n + j];
			if (U[j * n + j] == 0)
			{//主要此处会形成分解限制
				iResult = 0;
				goto END;
			}
			L[r * n + j] = fValue / U[j * n + j];
		}
	}
END:
	if (pbSuccess)
		*pbSuccess = iResult;
	return;
}
//********************一组LU分解********************************/

//*****************一组SVD 函数********************************/
template<typename _T>void Solve_Uk(_T L[3][3], _T Z[3], _T U[3])
{//求解LU=Z种的U
	U[0] = Z[0];
	U[1] = Z[1] - L[1][0] * U[0];
	U[2] = Z[2] - L[2][0] * U[0] - L[2][1] * U[1];
	return;
}
template<typename _T>void Solve_yk(_T U[3][3], _T yk[3], _T uk[3])
{//求解Uyk=uk中的yk
	yk[2] = uk[2] / U[2][2];
	yk[1] = (uk[1] - U[1][2] * yk[2]) / U[1][1];
	yk[0] = (uk[0] - U[0][1] * yk[1] - U[0][2] * yk[2]) / U[0][0];
	return;
}

template<typename _T>int bInverse_Power(_T A[3][3], _T* pfEigen_Value, _T Eigen_Vector[3], _T eps)
//int bInverse_Power(double A[3][3], double* pfEigen_Value, double Eigen_Vector[3])
{//求个最小的特征值所对应的特征向量，小范围数据已经成功收敛
//成功，返回1，失败:返回0
#define MAX_ITER_COUNT 100
	_T L[3][3], U[3][3], yk[3], zk[3] = { 1,1,1 }, z_pre[3] = { MAX_FLOAT,MAX_FLOAT,MAX_FLOAT }, uk[3];
	_T fDiff, fDiff_Total, tau, fSum;
	//Init_LU(L, U);
	int bSucceed = 1, i, iCount = 0;

	//double tm1[3][3],tv1[3];
	//iRank(A);

	//这个算法有缺陷，必须先判断矩阵是否可逆（满秩）
	LU_Decompose_3x3(A, L, U, &bSucceed);
	//出来以后A=L*U

	Matrix_Multiply((double*)L, 3, 3, (double*)U, 3, (double*)A);
	//Disp((double*)A, 3, 3, "A");
	//Disp(A,"A");
	if (!bSucceed || U[0][0] == 0 || U[1][1] == 0 || U[2][2] == 0)
	{//会有出现除以0
		*pfEigen_Value = Eigen_Vector[0] = Eigen_Vector[1] = Eigen_Vector[2] = 0;
		return 0;
	}

	while (1)
	{//以下两个解线性方程用的是高斯消元法，存在某些样本没有通用的反幂法精度高的原因
		//有可能是应为那些算法用的是列主元法，稍微好些
		//先求解Luk=Zk
		Solve_Uk(L, zk, uk);

		//再求 Uyk=uk
		Solve_yk(U, yk, uk);

		//Get_Max(yk, tau);
		Get_Max_1(yk, tau, i);
		tau = zk[i] >= 0 ? 1 / tau : -1.f / tau;	//乘法比除法快
		zk[0] = yk[0] * tau;
		zk[1] = yk[1] * tau;
		zk[2] = yk[2] * tau;

		//Disp(zk, "zk");
		fDiff_Total = 0;
		fDiff = zk[0] - z_pre[0];
		fDiff_Total = Abs(fDiff);
		fDiff = zk[1] - z_pre[1];
		fDiff_Total += Abs(fDiff);
		fDiff = zk[2] - z_pre[2];
		fDiff_Total += Abs(fDiff);
		//printf("Diff:%f\n", fDiff_Total);
		if (fDiff_Total < eps)
			//if(abs(zk[0] - z_pre[0])+abs(zk[1] - z_pre[1]) + abs(zk[2] - z_pre[2])< ZERO_APPROCIATE)
			break;
		else if ((iCount++) >= MAX_ITER_COUNT)
		{//必收敛，之不够不够彻底，可以跳出了事
			//printf("Diff:%f\n", fDiff_Total);
			//*pfEigen_Value = Eigen_Vector[0] = Eigen_Vector[1] = Eigen_Vector[2] = 0;
			//return 0;
			break;
		}

		z_pre[0] = zk[0];
		z_pre[1] = zk[1];
		z_pre[2] = zk[2];
	}

	//规格化
	fSum = zk[0] * zk[0] + zk[1] * zk[1] + zk[2] * zk[2];
	if (fSum < ZERO_APPROCIATE)
		return 0;
	fSum = 1 / sqrt(fSum);
	Eigen_Vector[0] = zk[0] *= fSum;
	Eigen_Vector[1] = zk[1] *= fSum;
	Eigen_Vector[2] = zk[2] *= fSum;
	*pfEigen_Value = tau;

	////临时代码验算
	//double A1[3][3],Temp[3],fErr;
	//memcpy(A1, A, 3 * 3 * sizeof(double));
	//A1[0][0] -= *pfEigen_Value;
	//A1[1][1] -= *pfEigen_Value;
	//A1[2][2] -= *pfEigen_Value;
	////Disp(A1, "A1");
	//Matrix_x_Vector(A1, Eigen_Vector,Temp);
	//if ((fErr=abs(Temp[0]) + abs(Temp[1]) + abs(Temp[2])) > 1)
	//	printf("err:%f\n",fErr);

	return 1;
#undef MAX_ITER_COUNT
}

template int bInverse_Power(float A[], int n, float* pfEigen_Value, float Eigen_Vector[], float eps);
template int bInverse_Power(double A[], int n, double* pfEigen_Value, double Eigen_Vector[], double eps);
template<typename _T>int bInverse_Power(_T A[], int n, _T* pfEigen_Value, _T Eigen_Vector[], _T eps)
{//求个最小的特征值所对应的特征向量，小范围数据已经成功收敛
//成功，返回1，失败:返回0
	//先来些短路优化
	if (n == 3)
		return bInverse_Power((_T(*)[3])A, pfEigen_Value, Eigen_Vector, eps);

#define MAX_ITER_COUNT 200
	_T* L, * U, * yk, * zk, * z_pre, * uk;
	_T* pBuffer = (_T*)pMalloc((2 * n * n + n * 4) * sizeof(_T));
	_T fDiff_Total;
	union {
		_T fMax;
		_T tau;
	};
	int bSucceed = 1, i, iCount = 0;

	L = pBuffer;			//n*n
	U = L + n * n;			//n*n
	yk = U + n * n;			//n
	zk = yk + n;			//n
	z_pre = zk + n;			//n
	uk = z_pre + n;			//n

	for (i = 0; i < n; i++)
		zk[i] = 1, z_pre[i] = MAX_FLOAT;

	//这个算法有缺陷，必须先判断矩阵是否可逆（满秩）
	LU_Decompose(A, L, U, n, &bSucceed);
	if (!bSucceed)
		goto END;
	//出来以后A=L*U

	while (1)
	{
		//先求解Luk=Zk
		//Solve_Uk(_T L[], _T Z[], _T U[], int n)//求解LU=Z种的U
		Solve_Linear_Gause(L, n, zk, uk, &bSucceed);
		if (!bSucceed)
			break;

		//再求 Uyk=uk	Solve_yk(U, yk, uk);
		Solve_Linear_Gause(U, n, uk, yk, &bSucceed);
		if (!bSucceed)
			break;

		//再找出yk中绝对值最大者
		int iMax = 0;
		for (i = 0; i < n; i++)
			if (abs(yk[i]) > fMax)
				fMax = abs(yk[i]), iMax = i;

		tau = zk[iMax] >= 0 ? 1 / yk[iMax] : -1.f / yk[iMax];	//乘法比除法快
		for (i = 0; i < n; i++)
			zk[i] = yk[i] * tau;

		fDiff_Total = 0;
		for (i = 0; i < n; i++)
			fDiff_Total += abs(zk[i] - z_pre[i]);

		//printf("iter:%d Diff:%f\n",iCount, fDiff_Total);
		if (fDiff_Total/n < eps)
			break;
		else if ((iCount++) >= MAX_ITER_COUNT)
		{//必收敛，之不够不够彻底，可以跳出了事
			break;
		}
		memcpy(z_pre, zk, n * sizeof(_T));
	}
	if (iCount > MAX_ITER_COUNT && fDiff_Total > eps)
		bSucceed = 0;

	//printf("Iter:%d Result:%d\n", iCount, bSucceed);
	//规格化
	Normalize(zk, n, Eigen_Vector);
	if (pfEigen_Value)
		*pfEigen_Value = tau;
END:
	if (pBuffer)
		Free(pBuffer);
	return bSucceed;
#undef MAX_ITER_COUNT
}

template void SVD_Alloc(int h, int w, SVD_Info* poInfo, float* A);
template void SVD_Alloc(int h, int w, SVD_Info* poInfo, double* A);
template<typename _T>void SVD_Alloc(int h, int w, SVD_Info* poInfo, _T* A)
{//更简接口，打算换成这个, 注意，用法为 SVD_Alloc<_T>(...)
	_T* U, * S, * Vt;
	SVD_Info oInfo;
	oInfo.A = A;
	oInfo.h_A = h;
	oInfo.w_A = w;
	if (h >= w)
	{//标准形
		U = (_T*)pMalloc(w * h * sizeof(_T));
		oInfo.h_Min_U = h;
		oInfo.w_Min_U = w;

		Vt = (_T*)pMalloc(w * w * sizeof(_T));
		oInfo.h_Min_Vt = oInfo.w_Min_Vt = w;
	}else
	{//
		U = (_T*)pMalloc(h * h * sizeof(_T));
		oInfo.h_Min_U = oInfo.w_Min_U = h;
		Vt = (_T*)pMalloc((h + 1) * w * sizeof(_T));
		oInfo.h_Min_Vt = h + 1;
		oInfo.w_Min_Vt = w;
	}
	oInfo.U = U;
	oInfo.Vt = Vt;
	S = (_T*)pMalloc(oInfo.w_Min_S = Min(h, w));
	oInfo.S = S;
	*poInfo = oInfo;
}

template<typename _T> void _svd_3(_T At[], int m, int n, int n1, _T Sigma[], _T Vt[], int* pbSuccess, double eps)
{
	int i, j, k, iter, max_iter = 30;
	_T sd;
	_T s, c;
	int iMax_Size = Max(m, n);
	memset(Sigma, 0, iMax_Size * sizeof(_T));
	//memset(Vt, 0, m * n * sizeof(_T));

	int astep = m, vstep = n;

	for (i = 0; i < n; i++)
	{
		for (k = 0, sd = 0; k < m; k++)
		{
			_T t = At[i * astep + k];
			sd += (_T)t * t;
		}
		Sigma[i] = sd;

		if (Vt)
		{
			for (k = 0; k < n; k++)
			{
				//if (i * vstep + k >= 90)
					//printf("err");
				Vt[i * vstep + k] = 0;
			}
			//if (i* vstep + i>= 90)
				//printf("here");
			Vt[i * vstep + i] = 1;
		}
	}
	//Disp(Vt, 3, vstep, "Vt");
	for (iter = 0; iter < max_iter; iter++)
	{
		int changed = false;
		for (i = 0; i < n - 1; i++)
		{
			for (j = i + 1; j < n; j++)
			{
				_T* Ai = At + i * astep, * Aj = At + j * astep;
				_T a = Sigma[i], p = 0, b = Sigma[j];

				for (k = 0; k < m; k++)
					p += (_T)Ai[k] * Aj[k];

				if (std::abs(p) <= eps * std::sqrt((double)a * b))
					continue;

				p *= 2;
				double beta = a - b, gamma = hypot((double)p, beta);
				if (beta < 0)
				{
					double delta = (gamma - beta) * 0.5;
					s = (_T)std::sqrt(delta / gamma);
					c = (_T)(p / (gamma * s * 2));
				}
				else
				{
					c = (_T)std::sqrt((gamma + beta) / (gamma * 2));
					s = (_T)(p / (gamma * c * 2));
				}

				a = b = 0;
				for (k = 0; k < m; k++)
				{
					_T t0 = c * Ai[k] + s * Aj[k];
					_T t1 = -s * Ai[k] + c * Aj[k];
					Ai[k] = t0; Aj[k] = t1;
					a += (_T)t0 * t0; b += (_T)t1 * t1;
				}
				Sigma[i] = a; Sigma[j] = b;

				changed = true;

				if (Vt)
				{
					_T* Vi = Vt + i * vstep, * Vj = Vt + j * vstep;
					k = 0;	//vblas.givens(Vi, Vj, n, c, s);
					for (; k < n; k++)
					{
						_T t0 = c * Vi[k] + s * Vj[k];
						_T t1 = -s * Vi[k] + c * Vj[k];
						Vi[k] = t0; Vj[k] = t1;
						//if (&Vi[i] - Vt >= 90)
							//printf("here");
					}
					//printf("iter:%d i:%d j:%d Vt[0]:%f\n", iter, i, j, Vt[0]);
				}
			}
		}
		if (!changed)
			break;
	}

	for (i = 0; i < n; i++)
	{
		for (k = 0, sd = 0; k < m; k++)
		{
			_T t = At[i * astep + k];
			sd += (_T)t * t;
		}
		Sigma[i] = std::sqrt(sd);
	}

	for (i = 0; i < n - 1; i++)
	{
		j = i;
		for (k = i + 1; k < n; k++)
		{
			if (Sigma[j] < Sigma[k])
				j = k;
		}
		if (i != j)
		{
			std::swap(Sigma[i], Sigma[j]);
			if (Vt)
			{
				for (k = 0; k < m; k++)
					std::swap(At[i * astep + k], At[j * astep + k]);

				for (k = 0; k < n; k++)
				{
					//if (i * vstep + k >= 90 || j * vstep + k >= 90)
						//printf("here");
					std::swap(Vt[i * vstep + k], Vt[j * vstep + k]);
				}
			}
		}
	}
	if (!Vt)
		return;
	//Disp(Vt, m, vstep,"Vt");

	unsigned long long iRandom_State = 0x12345678;
	for (i = 0; i < n1; i++)
	{
		sd = i < n ? Sigma[i] : 0;

		for (int ii = 0; ii < 100 && sd <= DBL_MIN; ii++)
		{
			// if we got a zero singular value, then in order to get the corresponding left singular vector
			// we generate a random vector, project it to the previously computed left singular vectors,
			// subtract the projection and normalize the difference.
			const _T val0 = (_T)(1. / m);
			for (k = 0; k < m; k++)
			{
				iGet_Random_No_cv(&iRandom_State);
				_T val = (iRandom_State & 256) != 0 ? val0 : -val0;
				At[i * astep + k] = val;
			}
			for (iter = 0; iter < 2; iter++)
			{
				for (j = 0; j < i; j++)
				{
					sd = 0;
					for (k = 0; k < m; k++)
						sd += At[i * astep + k] * At[j * astep + k];
					_T asum = 0;
					for (k = 0; k < m; k++)
					{
						_T t = (_T)(At[i * astep + k] - sd * At[j * astep + k]);
						At[i * astep + k] = t;
						asum += std::abs(t);
					}
					asum = asum > eps * 100 ? 1 / asum : 0;
					for (k = 0; k < m; k++)
						At[i * astep + k] *= asum;
				}
			}
			sd = 0;
			for (k = 0; k < m; k++)
			{
				_T t = At[i * astep + k];
				sd += (_T)t * t;
			}
			sd = std::sqrt(sd);
		}

		s = (_T)(sd > DBL_MIN ? 1 / sd : 0.);
		for (k = 0; k < m; k++)
			At[i * astep + k] *= s;
	}
}
template void svd_3(double* A, SVD_Info oSVD, int* pbSuccess, double eps);
template void svd_3(float* A, SVD_Info oSVD, int* pbSuccess, float eps);
template<typename _T> void svd_3(_T* A, SVD_Info oSVD, int* pbSuccess, _T eps)
{///*再写一次SVD，几个要点：
//1，所有的矩阵都通过转置统一为		nnnnn	即高比宽大的形状
//									nnnnn
//									nnnnn
//									nnnnn
//									nnnnn
//2, 对于原矩阵已经是高比宽大的形状，即如上，对于Ahxw, 暂时Vt只出wxw, U出到wxh
//3，对于形如 mmmmmmmmmmmmmmm	即宽比高大的矩阵，按时Vt只出到 (h+1)*w, U出到wxw
//			mmmmmmmmmmmmmmm
//			mmmmmmmmmmmmmmm
//4,对于方阵，暂时U Vt都是方阵
//5,S，暂时只出到一个min(m,n)
//*/
	_T* A_1, * Vt_1 = NULL, * Sigma;
	int bHor;
	int h = oSVD.h_A, w = oSVD.w_A;
	int iTemp, iMax_Size = Max(h, w), iMin_Size = Min(h, w);

	//这个A由于要参与到后面的真正计算中，还没有功夫考虑它的内存安排
	//A_1 = (_T*)pMalloc(&oMatrix_Mem, iMax_Size * iMax_Size * sizeof(_T));
	A_1 = (_T*)pMalloc(iMax_Size * iMax_Size * 2 * sizeof(_T));
	Sigma = (_T*)pMalloc(iMax_Size * sizeof(_T));
	if (!A_1 || !Sigma)
	{
		oSVD.m_bSuccess = 0;
		goto END;
	}
	else
		oSVD.m_bSuccess = 1;
	memset(A_1, 0, iMax_Size * iMax_Size * sizeof(_T));
	if (h < w)
	{//要转置成为高比宽大
		bHor = 1;
		iTemp = h;
		h = w;
		w = iTemp;
		memcpy(A_1, A, h * w * sizeof(_T));
	}
	else
	{//竖形 此时，A_1=At
		Matrix_Transpose(A, h, w, A_1);
		bHor = 0;
	}

	Vt_1 = (_T*)pMalloc(w * (w + 1) * sizeof(_T));
	memset(Vt_1, 0, w * (w + 1) * sizeof(_T));
	//U另做安排，可能直接写入U中
	//Disp(A, 3, 3, "A");
	//Disp(A_1, 3, 3, "A_1");
	_svd_3(A_1, h, w, w + 1, Sigma, Vt_1, &oSVD.m_bSuccess, eps);
	//Disp(A_1, h, h, "U");
	//Disp(Sigma,1,w,"S");
	//Disp(Vt_1, w, w, "Vt");
	//Disp(&A_1[2*3], 1, oSVD.h_A);
	memcpy(oSVD.S, Sigma, iMin_Size * sizeof(_T));
	if (!bHor)
	{//对A_1转置就是U，然而，此时U只取到形如	mmmm
	//											mmmm
	//											mmmm
	//											mmmm
		if (oSVD.U)
			Matrix_Transpose(A_1, w, h, (_T*)oSVD.U);
		//Disp((_T*)oInfo.U, oInfo.h_Min_U, oInfo.w_Min_U, "U");
		if (oSVD.Vt)
			memcpy(oSVD.Vt, Vt_1, oSVD.h_Min_Vt * oSVD.h_Min_Vt * sizeof(_T));
	}
	else
	{//
		//此时，Vt的值是U
		//Disp((_T*)oInfo.Vt, h, h);
		Matrix_Transpose(Vt_1, w, w, (_T*)oSVD.U);
		memcpy(oSVD.Vt, A_1, (w + 1) * h * sizeof(_T));
	}
	if (pbSuccess)
		*pbSuccess = oSVD.m_bSuccess;
END:
	if (A_1)
		Free(A_1);
	if (Sigma)
		Free(Sigma);
	if (Vt_1)
		Free(Vt_1);
}

template void SVD_Get_Solution(SVD_Info oSVD, float X[]);
template void SVD_Get_Solution(SVD_Info oSVD, double X[]);
template<typename _T>void SVD_Get_Solution(SVD_Info oSVD, _T X[])
{//将svd分解后的vt最右列分离出来，放在X中
	memcpy(X, &((_T*)oSVD.Vt)[(oSVD.w_A - 1) * oSVD.w_A], oSVD.w_A * sizeof(_T));
}

template float fGet_Cond_Num(float A[], int n);
template double fGet_Cond_Num(double A[], int n);
template<typename _T> _T fGet_Cond_Num(_T A[], int n)
{//对一个方阵进行SVD分解，得到一组奇异值（特征值），最大奇异值除以最小奇异值就是条件数
	//很一般来说，条件数越大，方程越病态
	_T fCond_Num = 0;
	SVD_Info oSVD;
	int iResult;
	SVD_Alloc<_T>(n * 2, 9, &oSVD);
	svd_3<_T>(A, oSVD, &iResult);

	_T* S = (_T*)oSVD.S;
	fCond_Num = S[0] / S[oSVD.w_Min_S - 1];
	Free_SVD(&oSVD);
	return fCond_Num;
}
template void Test_SVD(float A[], SVD_Info oSVD, int* piResult, float eps);
template void Test_SVD(double A[], SVD_Info oSVD, int* piResult, double eps);
template<typename _T>void Test_SVD(_T A[], SVD_Info oSVD, int* piResult, _T eps)
{//另一种简化表示，此处验算一下分解结果是否符合预期
	int y,  iResult = 1;
	_T* A_1 = NULL;
	union {
		_T* S;
		_T* SVt;
		_T* US;
	};
	_T fCon_Num = 0;
	if (oSVD.h_A > oSVD.w_A)
	{//竖形，A= U x (S x Vt) 先算后面，再算前面
		SVt = (_T*)pMalloc(oSVD.w_Min_S * oSVD.w_Min_S * sizeof(_T));
		memset(S, 0, oSVD.w_Min_S * oSVD.w_Min_S * sizeof(_T));
		for (y = 0; y < oSVD.w_Min_S; y++)
			S[y * oSVD.w_Min_S + y] = ((_T*)oSVD.S)[y];
		//Disp(S, oSVD.w_Min_S, oSVD.w_Min_S, "S");
		fCon_Num = S[0] / S[oSVD.w_Min_S * oSVD.w_Min_S - 1];
		Matrix_Multiply(S, oSVD.w_Min_S, oSVD.w_Min_S, (_T*)oSVD.Vt, oSVD.h_Min_Vt, SVt);
		//Disp(SVt, oSVD.w_Min_S, oSVD.w_Min_S, "SVt");
		A_1 = (_T*)pMalloc(oSVD.h_A * oSVD.w_A * sizeof(_T));
		Matrix_Multiply((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U, SVt, oSVD.w_Min_Vt, A_1);
		//Disp(A_1, oSVD.h_A, oSVD.w_A, "A_1");
		Free(SVt);
	}
	else
	{//横形
		S = (_T*)pMalloc(oSVD.w_Min_S * oSVD.w_Min_S * sizeof(_T));
		memset(S, 0, oSVD.w_Min_S * oSVD.w_Min_S * sizeof(_T));
		for (y = 0; y < oSVD.w_Min_S; y++)
			((_T*)S)[y * oSVD.w_Min_S + y] = ((_T*)oSVD.S)[y];
		//Disp(S, oSVD.w_Min_S, oSVD.w_Min_S, "S");
		Matrix_Multiply((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U, S, oSVD.w_Min_S, US);
		Disp(US, oSVD.h_Min_U, oSVD.w_Min_S, "UxS");
		A_1 = (_T*)pMalloc(oSVD.h_A * oSVD.w_A * sizeof(_T));
		Matrix_Multiply(US, oSVD.h_Min_U, oSVD.w_Min_S, (_T*)oSVD.Vt, oSVD.w_Min_Vt, A_1);
		Disp(A_1, oSVD.h_A, oSVD.w_A, "A_1");
		Free(US);
	}
	if (bIs_Orthogonal((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U))
		printf("U 正交\n");
	else
		printf("U 非正交\n");

	if (bIs_Orthogonal((_T*)oSVD.Vt, oSVD.h_Min_Vt, oSVD.w_Min_Vt))
		printf("Vt 正交\n");
	else
		printf("Vt 非正交\n");
	printf("条件数：%f\n", fCon_Num);

	/*for (y = 0; y < oSVD.h_A; y++)
	{
		for (x = 0; x < oSVD.w_A; x++)
		{
			if (abs(A_1[y * oSVD.w_A + x] - A[y * oSVD.w_A + x]) > eps)
			{
				printf("Correct:%f Error:%f\n", A[y * oSVD.w_A + x], A_1[y * oSVD.w_A + x]);
				iResult = 0;
			}
		}
	}*/

	if (A_1)
		Free(A_1);
	if (piResult)
		*piResult = iResult;
}
template<typename _T>void Test_SVD(_T A[], int h, int w, _T U[], _T S[], _T Vt[], int* piResult, double eps)
{//验算SVD分解结果
	_T* pTemp = (_T*)pMalloc(h * w * sizeof(_T));
	int y, x, iResult;
	Disp(S, h, w, "S,留心观察奇异值,凡是0值的位置可以无视Vt对应的行");
	Matrix_Multiply(U, h, h, S, w, pTemp);
	Disp(pTemp, h, w, "UxS");
	Matrix_Multiply(pTemp, h, w, Vt, w, pTemp);
	Disp(pTemp, h, w, "USVt");
	iResult = 1;
	if (bIs_Orthogonal(U, h))
		printf("U 正交\n");
	else
		printf("U 非正交\n");

	if (bIs_Orthogonal(Vt, w))
		printf("Vt 正交\n");
	else
		printf("Vt 非正交\n");

	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			if (abs(pTemp[y * w + x] - A[y * w + x]) > eps)
			{
				printf("Correct:%f Error:%f\n", A[y * w + x], pTemp[y * w + x]);
				iResult = 0;
				//goto END;
			}
		}
	}

	Free(pTemp);
	if (piResult)
		*piResult = iResult;
	return;
}
void Free_SVD(SVD_Info* poInfo)
{
	Free(poInfo->U);
	Free(poInfo->S);
	Free(poInfo->Vt);
}
//*****************一组SVD 函数********************************/

/*******************统计函数***********************************/
template void Get_E_2d(float Point[][2], int iCount, float E[]);
template void Get_E_2d(double Point[][2], int iCount, double E[]);
template<typename _T>void Get_E_2d(_T Point[][2], int iCount, _T E[])
{//计算期望
	E[0] = E[1] = 0;
	for (int i = 0; i < iCount; i++)
	{
		E[0] += Point[i][0];
		E[1] += Point[i][1];
	}
	E[0] /= iCount, E[1] /= iCount;
}

template void Get_Dev_2d(float Point[][2], int n, float E[2], float Dev[2]);
template void Get_Dev_2d(double Point[][2], int n, double E[2], double Dev[2]);
template<typename _T>void Get_Dev_2d(_T Point[][2], int n, _T E[2], _T Dev[2])
{
	_T Dev_1[2] = { 0 };
	for (int i = 0; i < n; i++)
		Dev_1[0] += (_T)abs(Point[i][0] - E[0]),
		Dev_1[1] += (_T)abs(Point[i][1] - E[1]);
	Dev[0] = Dev_1[0]/n, Dev[1] = Dev_1[1]/n;
	return;
}
template<typename _T>float fGet_Cov_2d(_T Point[][2], int iCount, _T E[2])
{//利用期望与样本计算协方差
	_T fTotal = 0;
	for (int i = 0; i < iCount; i++)	//由点与期望(0,0)围成一个正方形，累加面积
		fTotal += (Point[i][0] - E[0]) * (Point[i][1] - E[1]);
	return fTotal / iCount;
}
template<typename _T>void Get_Var_2d(_T Point[][2], int iCount, _T E[2], _T Var[2])
{//算方差
	Var[0] = Var[1] = 0;
	for (int i = 0; i < iCount; i++)
	{
		Var[0] += (Point[i][0] - E[0]) * (Point[i][0] - E[0]);
		Var[1] += (Point[i][1] - E[1]) * (Point[i][1] - E[1]);
	}
	Var[0] /= iCount;
	Var[1] /= iCount;
}

template<typename _T>_T fGet_Corr_Coef_2d(_T Point[][2], int iCount)
{//求相关系数
	_T E[2], Var[2];
	Get_E_2(Point, iCount, E);
	_T fCov = fGet_Cov_2(Point, iCount, E);
	Get_Var_2(Point, iCount, E, Var);

	//相关系数
	return (_T)(fCov / sqrt(Var[0] * Var[1]));
}
template<typename _T>void Gen_Cov_Matrix_2d(_T Point[][2], int iCount, _T A[2 * 2])
{//求协方差矩阵
	_T E[2], Var[2];
	Get_E_2(Point, iCount, E);
	_T fCov = fGet_Cov_2(Point, iCount, E);
	Get_Var_2(Point, iCount, E, Var);

	A[0] = Var[0], A[3] = Var[1];
	A[1] = A[2] = fCov;
}
template<typename _T>_T fGet_Mah_Dist_2d(_T Cov[2 * 2], _T Point_1[2], _T Point_2[2])
{//求马氏距离，一点与一个点集的关系
	_T Delta[2] = { Point_2[0] - Point_1[0],Point_2[1] - Point_1[1] };
	Matrix_Multiply(Cov, 2, 2, Delta, 1, Delta);

	_T fDist = fDot(Delta, Delta, 2);
	return (_T)sqrt(fDist);
}
template<typename _T>_T fGet_Norm_Dist(_T x)
{//返回标准正太分布（积分）
	return (_T)(0.5 * erfc(-x * sqrt(0.5)));
}

template<typename _T>_T fGet_Norm_Dist(_T e, _T sigma, _T x)
{//对于一般正太分布，返回积分值
//基于一个原理，若 x ~ N(e,sigma^2) 则
//F(x) = Phi( (x-e)/sigma )
	return fGet_Norm_Dist((x - e) / sigma);
}
/*******************统计函数***********************************/