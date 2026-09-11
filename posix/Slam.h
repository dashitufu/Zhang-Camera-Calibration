#pragma once
#include "Matrix.h"

typedef enum {
	Hartley,		//最优
	Dev,			//稍逊
	Bounding_Box,	//最差
	None			//不归一化
}Normalize_Method;

template<typename _T> struct Point_2D {
	unsigned int m_iCamera_Index;
	unsigned int m_iPoint_Index;
	_T m_Pos[2];
};

template<typename _T>void Get_K4_by_eq_focal(_T eq_f, _T w, _T h, _T K[4]);
template<typename _T>void Get_K3_by_eq_focal(_T eq_f, _T w, _T h, _T K[3]);
template<typename _T>void Get_K9_by_eq_focal(_T eq_f, _T w, _T h, _T K[3 * 3]);
template<typename _T>void K3_Proj(_T K[3], _T P[3], _T uv[2], int bHas_z = 1);
template<typename _T>void K3_Inv(_T K[3], _T K_Inv[3]);	//K3 快速求逆
template<typename _T>void K9_2_K4(_T K9[9], _T K4[4]);
template<typename _T>void K4_2_K9(_T K4[9], _T K9[4]);

//一组归一化函数
template<typename _T>void Normalize_by_B_Box_2d(_T P[][2], int n, _T fBox_Size, _T Norm[][2], _T K[3] = NULL, _T K_Inv[] = NULL);	//求一bounding_Box, 将图片搞里头
template<typename _T>void Normalize_Hartley_2d(_T P[][2], int n, _T Norm[][2], _T K[3] = NULL, _T K_Inv[] = NULL);
template<typename _T>void Normalize_Dev_2d(_T P[][2], int n, _T Norm[][2], _T K[4] = NULL, _T K_Inv[4] = NULL);
template<typename _T>void Normalize_2d(_T P[][2], int n, _T Norm[][2],
	Normalize_Method iMethod = Hartley, _T K[4] = NULL, _T K_Inv[4] = NULL);

//H矩阵估计函数
template<typename _T>int Estimate_H_Ref(_T P[][2], _T uv[][2], int n, _T H[3 * 3], int bNormalize = 1, int bUse_SVD = 1);
template<typename _T>int Estimate_H_Zhang(_T Norm_P[][2], _T uv[][2], int n, _T H[3 * 3],
	_T K_Norm[], int bNormalize = 1, Normalize_Method iMethod = Dev);
template<typename _T>void Gen_H_Coeff_z_0(_T P[][2], _T uv[][2], int n, _T A[]);
template<typename _T>_T Test_H_2d(_T P[][2], _T uv[][2], int n, _T H[3 * 3]);

//四元数,旋转矩阵，旋转向量互换函数
template<typename _T>void Quaternion_2_Rotation_Matrix(_T Q[4], _T R[]);	//四元数转旋转矩阵
template<typename _T>void Quaternion_2_Rotation_Vector(_T Q[4], _T V[4]);	//四元数转旋转向量
void Quaternion_Add(float Q_1[], float Q_2[], float Q_3[]);	//四元数加
void Quaternion_Minus(float Q_1[], float Q_2[], float Q_3[]);	//四元数减，其实这两个函数可以用普通向量加减
void Quaternion_Conj(float Q_1[], float Q_2[]);				//简单求个共轭
void Quaternion_Multiply(float Q_1[], float Q_2[], float Q_3[]);	//四元数乘法
void Quaternion_Inv(float Q_1[], float Q_2[]);	//四元数求逆
template<typename _T>void Rotation_Matrix_2_Quaternion(_T R[], _T Q[]);		//旋转矩阵转换四元数
template<typename _T>void Rotation_Matrix_2_Vector(_T R[3 * 3], _T V[4]);		//旋转矩阵转旋转向量
template<typename _T>void Rotation_Matrix_2_Vector_3(_T R[3 * 3], _T V[3]);
template<typename _T>void Rotation_Matrix_2_Vector_4(_T R[3 * 3], _T V[4]);
template<typename _T>void Rotation_Vector_4_2_Matrix(_T V[4], _T R[3 * 3]);		//旋转向量转换旋转矩阵
template<typename _T>void Rotation_Vector_3_2_Matrix(_T V[3], _T R[3 * 3]);		//旋转向量转换旋转矩阵
template<typename _T>void Rotation_Vector_2_Quaternion(_T V[4], _T Q[4]);	//旋转向量转换四元数
template<typename _T>void Rotation_Vector_3_2_4(_T V[], _T V1[]);
template<typename _T>void Rotation_Vector_4_2_3(_T V[], _T V1[]);

//各种投影函数
template<typename _T>void Get_uv_Ref(_T P[4], _T T[4 * 4], _T K[3 * 3], _T D[5], _T uv[2]);
template<typename _T>void Get_Distort_Coeff(_T x, _T y, _T D[5], _T dc[2 * 5]);

//李群李代数
template<typename _T>void Get_PnP_Deriv(_T P[4], _T uv_Ref[2], _T T[4 * 4], _T K[3 * 3], _T D[5],
	_T dE_dK[2*4]=NULL, _T dE_dKsi[2*6] = NULL, _T dE_dD[2*5] = NULL, _T dE_dP[2*3] = NULL, _T E[2]=NULL);
template<typename _T>void Get_dTP_dKsi(_T Pt[3], _T Deriv[4 * 6]);
template<typename _T>void Get_dTP_dKsi(_T T[3 * 4], _T P[2], _T Deriv[4 * 6]);
template<typename _T>void Get_J_E_uv(_T P[4], _T uv[2], _T T[3 * 4], _T K[4], _T D[5], _T J[2][15], _T E[2]);
template<typename _T>void Get_J_E_Norm(_T P[4], _T uv[2], _T T[3 * 4], _T K[4], _T D[5], _T J[2][15], _T E[2]);
template<typename _T>void Gen_Pose_By_R_t(_T R[], _T t[], _T T[]);
template<typename _T>void T_2_R9_t(_T T[3 * 4], _T R[3 * 3], _T t[3]);
template<typename _T>void Gen_Pose_By_V3_t(_T V3[], _T t[], _T T[]);
template<typename _T>void Get_R_t(_T T[4 * 4], _T R[3 * 3], _T t[3]);
template<typename _T>void Hat(_T V[], _T M[]);
template<typename _T>void Vee(_T M[], _T V[3]);
template<typename _T>void se3_2_SE3(_T Ksi[6], _T T[]);

//一气呵成构造H 矩阵
template<typename _T>void Get_H_JtE(_T T[][3 * 4], _T K[4], _T D[5], _T P[][2], Point_2D<_T> uv[],
	int iObservation_Count, int iOrder, _T H[], _T JtE[]);
template<typename _T>void Get_H_JtE(_T T[][3 * 4], _T K[4], _T D[5], _T P[][2], Point_2D<_T> uv[],
	int iObservation_Count, int iOrder, _T H[], _T JtE[]);
