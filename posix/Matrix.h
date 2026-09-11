//由于数学库同时应对float/double两种类型，故此该版开始，全变为模板函数
#pragma once 
#include <typeinfo>
#include "Common.h"
//using namespace std;

#define sign(x) (x>=0?1:-1)

#define Get_Max_1(V, fMax,i) \
{ \
	double fAbs_Max; \
	fAbs_Max = Abs(fMax=V[0]); \
	i=0; \
	if (Abs(V[1]) > fAbs_Max) \
	{ \
		fAbs_Max = Abs(V[1]), fMax = V[1]; \
		i=1;	\
	} \
	if ((Abs(V[2]) > fAbs_Max)) \
	{ \
		fAbs_Max = Abs(V[2]), fMax = V[2]; \
		i=2;	\
	} \
}

//#define PI 3.14159265358979323846
#define Abs(A) ((A)>=0?(A):(-(A)))
#define MAX_FLOAT ((float)0xFFFFFFFFFFFFFFFF)	//仅仅给一个足够大的数字，并不是iEEE的浮点数最大值
#define ZERO_APPROCIATE	0.00001f

#define Get_Homo_Pos(m_Pos, Homo_Pos) Homo_Pos[0]=m_Pos[0], Homo_Pos[1]=m_Pos[1], Homo_Pos[2]=m_Pos[2],Homo_Pos[3]=1;

typedef struct SVD_Info {	//有必要给SVD做一个单独的参数结构
	void* A;		//原矩阵
	void* U;
	void* S;
	void* Vt;
	int h_A, w_A,
		h_Min_U, w_Min_U,
		w_Min_S,
		h_Min_Vt, w_Min_Vt;
	int m_bSuccess;
}SVD_Info;

template<typename _T>void Matrix_Add(_T A[], _T B[], int iOrder, _T C[]);
template<typename _T>void Matrix_Minus(_T A[], _T B[], int iOrder, _T C[]);
template<typename _T>void Add_I_Matrix(_T A[], int iOrder, _T ramda);
template<typename _T>void Copy_Matrix_Partial(_T Source[], int iSource_Stride, int x, int y, _T Dest[], int m, int n);
template<typename _T>void Copy_Matrix_Partial(_T Source[], int m, int n, _T Dest[], int iDest_Stride, int x, int y);
template<typename _T> void Transpose_Multiply(_T A[], int m, int n, _T B[], int bAAt = 1);
template<typename _T> void Matrix_Multiply(_T* A, int ma, int na, _T* B, int nb, _T* C);
template<typename _T>void Matrix_Multiply_3x3(_T A[3 * 3], _T B[3 * 3], _T C[3 * 3]);
template<typename _T>void Matrix_Multiply_3x1(_T A[3 * 3], _T B[3], _T C[3]);
void Matrix_Multiply_float_1(float A[], int ma, int na, float B[], int nb, float C[]);
template<typename _T>void Reshape_Row_Mul_Align(_T A[], int m, int n, int iBlock_Size, _T** ppA1, int bTranspose = 0);
template<typename _T>void Reshape_Col_Mul_Align(_T B[], int m, int n, int iBlock_Size, _T** ppB1, int bTranspose = 0);
void Matrix_Multiply_Reshape_float(float A[], int ma, int na, float B[], int nb,
	float C[], int mc, int nc);
template<typename _T>void At_x_B(_T A[], int ma, int na, _T B[], int nb, _T C[]);
//矩阵求逆
template<typename _T> void Get_Inv_Matrix_Row_Op(_T* pM, _T* pInv, int iOrder, int* pbSuccess = NULL);	//矩阵求逆
template<typename _T>void Get_Inv_AAt_Row_Op(_T* pM, _T* pInv, int iOrder, int* pbSuccess = NULL);
template<typename _T>void Get_Inv_AAt_3x3(_T M[3 * 3], _T Inv[3 * 3], int* pbSuccess=NULL);
template<typename _T>void Get_Inv_Matrix(_T* pM, _T* pInv, int iOrder, int* pbSuccess = NULL);
template<typename _T>_T fGet_Determinant(_T* A, int iOrder);			//求行列式

//矩阵函数
template<typename _T>void Matrix_Transpose(_T* A, int ma, int na, _T* At);

//向量函数
template<typename _T>int bIs_Finite(_T V[], int n);
template<typename _T> _T fGet_Mod(_T V[], int n);
template<typename _T>_T fGet_Sqr_Sum(_T V[], int n);
template<typename _T>_T fDot(_T V0[], _T V1[], int iDim);
template<typename _T>void Cross_Product(_T V0[], _T V1[], _T V2[]);
template<typename _T>_T fCross_Product_2D(_T V0[], _T V1[]);
template<typename _T>_T fGet_Theta_2D(_T v0[], _T v1[]);
template<typename _T>void Vector_Minus(_T A[], _T B[], int n, _T C[]);
template<typename _T>void Vector_Add(_T A[], _T B[], int n, _T C[]);
template<typename _T>void Vector_Multiply(_T A[], int n, _T B[], _T C[]);
template<typename _T>void Vector_Multiply(_T A[], int n, _T a, _T B[]);
template<typename _T> void Normalize(_T V[], int n, _T V_1[]);

//SVD
template<typename _T>void SVD_Alloc(int h, int w, SVD_Info* poInfo, _T* A = NULL);
template<typename _T> void svd_3(_T* A, SVD_Info oSVD, int* pbSuccess = NULL, _T eps = 2.2204460492503131e-15);
template<typename _T>void Test_SVD(_T A[], SVD_Info oSVD, int* piResult = NULL, _T eps = 2.2204460492503131e-15);
template<typename _T>void SVD_Get_Solution(SVD_Info oSVD, _T X[]);
void Free_SVD(SVD_Info* poInfo);
template<typename _T>int bInverse_Power(_T A[], int n, _T* pfEigen_Value, _T Eigen_Vector[], _T eps = 1e-5f);
template<typename _T>int bInverse_Power(_T A[3][3], _T* pfEigen_Value, _T Eigen_Vector[3], _T eps = 1e-9f);
template<typename _T> _T fGet_Cond_Num(_T A[], int n);

//线性方程组
template<typename _T>void Test_Linear(_T A[], int iOrder, _T X[], _T B[] = NULL, _T* pfError_Sum = NULL);
template<typename _T>void Test_Linear_Contradictory(_T A[], int m, int n, _T X[], _T B[] = NULL, _T* pfError_Sum = NULL);
template<typename _T> void Solve_Linear_Solution_Construction_1(_T* A, int m, int n, _T B[], int* pbSuccess, _T* pBasic_Solution = NULL, int* piBasic_Solution_Count = NULL, _T* pSpecial_Solution = NULL);
template<typename _T> void Solve_Linear_Contradictory(_T A[], int m, int n, _T B[], _T X[], int* pbSuccess = NULL, _T fAdj_eps = 1e-6);
template<typename _T>void Solve_Linear_Gause_AAt(_T* A, int iOrder, _T* B, _T* X, int* pbSuccess);
template<typename _T>void Solve_Homo_Linear_SVD(_T A[], int m, int n, _T X[], int* pbSuccess = NULL);

//一组统计函数
template<typename _T>void Get_E_2d(_T Point[][2], int iCount, _T E[]);
template<typename _T>void Get_Dev_2d(_T Point[][2], int n, _T E[2], _T Dev[2]);
template<typename _T>float fGet_Cov_2d(_T Point[][2], int iCount, _T E[2]);
template<typename _T>void Get_Var_2d(_T Point[][2], int iCount, _T E[2], _T Var[2]);
template<typename _T>_T fGet_Corr_Coef_2d(_T Point[][2], int iCount);
template<typename _T>void Gen_Cov_Matrix_2d(_T Point[][2], int iCount, _T A[2 * 2]);
template<typename _T>_T fGet_Mah_Dist_2d(_T Cov[2 * 2], _T Point_1[2], _T Point_2[2]);
template<typename _T>_T fGet_Norm_Dist(_T x);
template<typename _T>_T fGet_Norm_Dist(_T e, _T sigma, _T x);

//一组矩阵分解
template<typename _T>void LLt_Decompose(_T A[], int iOrder, _T B[], int* pbSuccess = NULL);
template<typename _T>void Cholosky_Decompose(_T A[], int iOrder, _T B[] = NULL, int* pbSuccess = NULL);
template<typename _T>void LU_Decompose(_T A[], _T L[], _T U[], int n, int* pbSuccess = NULL);
template<typename _T>void LU_Decompose_3x3(_T A[3][3], _T L[3][3], _T U[3][3], int* pbSucceed=NULL);


template<typename _T>int bIs_Symmetric_Matrix(_T A[], int iOrder, const _T eps=1e-7);