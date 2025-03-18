//重建用的代码 
#pragma once
#include "iostream"
#include "Reconstruct.h"
using namespace std;

#define SQR(Value) (Value)*(Value)

template<typename _T> void Sample_XY(_T Point_1[][2], _T Point_2[][2], short Sample_Index[], int iCount, _T X_rand[][2], _T Y_rand[][2], int iRan_Count = 8)
{//从一堆点集中抽出八个点放在最前面
	int i, iNo_1,iNo_2, iLast_Index = iCount - 1;
	for (i = 0; i < iRan_Count; i++)
	{
		iNo_2 = iRandom(i, iLast_Index);
		iNo_1 = Sample_Index[iNo_2];
		X_rand[i][0] = Point_1[iNo_1][0];
		X_rand[i][1] = Point_1[iNo_1][1];
		Y_rand[i][0] = Point_2[iNo_1][0];
		Y_rand[i][1] = Point_2[iNo_1][1];
		swap(Sample_Index[i], Sample_Index[iNo_2]);		
	}
	//Disp(Sample_Index, iRan_Count, 1);
	return;
}

template<typename _T> void Normalize_Point_3(_T Point[][2], int iCount, _T M[3][3], _T Norm_Point[][2])
{//将一组点，投影到边长为 2* sqrt(2)的投影平面上,那么所有的点都已经实现某种意义上的归一化
	_T Centroid[2] = { 0 };
	int i;
	//先求个重心
	for (i = 0; i < iCount; i++)
		Centroid[0] += Point[i][0], Centroid[1] += Point[i][1];
	Centroid[0] /= iCount, Centroid[1] /= iCount;

	//每一点到重心有个距离，求8点的距离平方和
	_T rms_mean_dist = 0;
	for (i = 0; i < iCount; i++)
		rms_mean_dist += SQR(Point[i][0] - Centroid[0]) + SQR(Point[i][1] - Centroid[1]);
	rms_mean_dist = (_T)sqrt(rms_mean_dist / iCount);	//此处先除以8再开方才对，别瞎改
	//算完以后，rms_mean_dist就是8点到重心的平均距离

	//如何理解norm_factor？先简化一下，所有的点到重心距离都是rms_mean_dist，
	//可组成一个正方形,边长为2*rms_mean_dist。我们将这个正方形（像素平面）
	//设为2*sqrt(2)大小。 那么rms_mean_dist对应sqrt(2)这么大。然后，norm_factor
	//就是指每一份原距离的像素数量.回顾一下自己做像素平面的方法：
	//a = 1920 / (fMax_x - fMin_x);	
	//b = -1080 / (fMax_y - fMin_y);	有相似之处，都是拿屏幕的大小作为分子
	_T norm_factor = (_T)sqrt(2.0) / rms_mean_dist;
	//这个M矩阵长得很像相机内参，后面的算法会进一步印证
	// fx 0  cx
	//	0 fy cy
	//	0 0  1
	M[0][0] = norm_factor, M[0][1] = 0, M[0][2] = -norm_factor * Centroid[0];
	M[1][0] = 0, M[1][1] = norm_factor, M[1][2] = -norm_factor * Centroid[1];
	M[2][0] = 0, M[2][1] = 0, M[2][2] = 1;
	//Disp((float*)M, 3, 3, "M");

	//规格化
	for (i = 0; i < iCount; i++)
	{
		Norm_Point[i][0] = Point[i][0] * M[0][0] + M[0][2];
		Norm_Point[i][1] = Point[i][1] * M[1][1] + M[1][2];
		//printf("%f %f\n", Norm_Point[i][0], Norm_Point[i][1]);
	}
	return;
}
template<typename _T> void Normalize_Point_1(_T Point[][2], int iCount, _T M[3][3], _T Norm_Point[][2])
{//将点做一次规格化，形成一个相机内参M
	_T Centroid[2] = { 0 };
	int i;
	//先求个重心
	for (i = 0; i < iCount; i++)
		Centroid[0] += Point[i][0], Centroid[1] += Point[i][1];
	Centroid[0] /= iCount, Centroid[1] /= iCount;

	//每一点到重心有个距离，求8点的距离平方和
	_T rms_mean_dist = 0;
	for (i = 0; i < iCount; i++)
		rms_mean_dist += SQR(Point[i][0] - Centroid[0]) + SQR(Point[i][1] - Centroid[1]);
	rms_mean_dist = (_T)sqrt(rms_mean_dist / iCount);	//此处先除以8再开方才对，别瞎改
	//算完以后，rms_mean_dist就是8点到重心的平均距离

	//如何理解norm_factor？先简化一下，所有的点到重心距离都是rms_mean_dist，
	//可组成一个正方形,边长为2*rms_mean_dist。我们将这个正方形（像素平面）
	//设为2*sqrt(2)大小。 那么rms_mean_dist对应sqrt(2)这么大。然后，norm_factor
	//就是指每一份原距离的像素数量.回顾一下自己做像素平面的方法：
	//a = 1920 / (fMax_x - fMin_x);	
	//b = -1080 / (fMax_y - fMin_y);	有相似之处，都是拿屏幕的大小作为分子
	_T norm_factor = (_T)sqrt(2.0) / rms_mean_dist;
	//这个M矩阵长得很像相机内参，后面的算法会进一步印证
	// fx 0  cx
	//	0 fy cy
	//	0 0  1
	M[0][0] = norm_factor, M[0][1] = 0, M[0][2] = -norm_factor * Centroid[0];
	M[1][0] = 0, M[1][1] = norm_factor, M[1][2] = -norm_factor * Centroid[1];
	M[2][0] = 0, M[2][1] = 0, M[2][2] = 1;
	//Disp((float*)M, 3, 3, "M");

	//规格化
	for (i = 0; i < iCount; i++)
	{
		Norm_Point[i][0] = Point[i][0] * M[0][0] + M[0][2];
		Norm_Point[i][1] = Point[i][1] * M[1][1] + M[1][2];
		//printf("%f %f\n", Norm_Point[i][0], Norm_Point[i][1]);
	}
	//Disp((float*)Norm_Point, iCount, 2, "Norm Point");
	return;
}
template<typename _T> void Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, _T F[3 * 3], _T(*Norm_Point_1)[2], _T(*Norm_Point_2)[2], Light_Ptr oPtr)
{//理论上这个函数已经完备，给定的一组点已经能找到一个矩阵F，使得p2'*F*p1=0，已经油最简形式，不需要Norm_Point空间
	_T M_1[3][3], M_2[3][3];
	unsigned char* pCur;
	int i, iSize, bUse_Matrix_Mem;
	
	bUse_Matrix_Mem = (!Norm_Point_1 || !Norm_Point_2 || !oPtr.m_pBuffer) ? 1 : 0;
	if (bUse_Matrix_Mem)
	{
		iSize = iCount * 4 * sizeof(_T) +	//Norm_Point
			iCount * 9 * sizeof(_T) +		//A
			128 * 3;
		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(&oMatrix_Mem, iSize), iSize, -1);
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_1 = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_2 = (_T(*)[2])pCur;
	}
	Normalize_Point_1(Point_1, iCount, M_1, Norm_Point_1);
	Normalize_Point_1(Point_2, iCount, M_2, Norm_Point_2);

	//从以下可以看到，经过相对中心的规格化后，两者数值进一步接近
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, iCount, 2, "Norm_Point_2");
	//Disp((_T*)Point_2, iCount, 2, "Point_2");
	//构造系数系数矩阵
	//Point_1 x F(3x3) x Point_2' 展开，为行向量
	//Disp((float*)pNorm_Point_1, iCount, 2);
	_T(*A)[9];	// , B[8] = { 0 };
	Malloc(oPtr, iCount * 9 * sizeof(_T), pCur);
	A = (_T(*)[9])pCur;
	for (i = 0; i < iCount; i++)
	{
		A[i][0] = Norm_Point_2[i][0] * Norm_Point_1[i][0];
		A[i][1] = Norm_Point_2[i][0] * Norm_Point_1[i][1];
		A[i][2] = Norm_Point_2[i][0];
		A[i][3] = Norm_Point_2[i][1] * Norm_Point_1[i][0];
		A[i][4] = Norm_Point_2[i][1] * Norm_Point_1[i][1];
		A[i][5] = Norm_Point_2[i][1];
		A[i][6] = Norm_Point_1[i][0];
		A[i][7] = Norm_Point_1[i][1];
		A[i][8] = 1;
	}

	//正式求解
	_T Basic_Solution[7 * 9];
	//int iBasic_Solution_Count, iResult;
	//注意，当Ax=0 中的A为横形，此时可以不用svd求解，就是个齐次方程求基础解系的过程，也许更快更准
	//Solve_Linear_Solution_Construction((float*)A, iCount, 9, B, &iResult, Basic_Solution, &iBasic_Solution_Count, NULL);
	SVD_Info oSVD;
	SVD_Alloc<_T>(iCount, 9, &oSVD);
	//Disp((_T*)A, 8, 9, "A");
	svd_3((_T*)A, oSVD, &oSVD.m_bSuccess);
	//Disp((_T*)oSVD.Vt, 9, 9, "Vt");
	memcpy(Basic_Solution, ((_T(*)[9])oSVD.Vt)[8], 9 * sizeof(_T));
	//第一次SVD已经结束使命
	Free_SVD(&oSVD);

	//由解再组合成一个F矩阵
	_T Temp_1[3 * 3], 
		F_1[3 * 3] = { Basic_Solution[0],Basic_Solution[1],Basic_Solution[2],
		Basic_Solution[3],Basic_Solution[4],Basic_Solution[5],
		Basic_Solution[6],Basic_Solution[7],Basic_Solution[8] };
	//Disp((_T*)E_t, 3, 3, "Et");
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(F_1, oSVD,&oSVD.m_bSuccess);

	//对于F矩阵，奇异值的处理稍微没那么多，仅仅把最后一个奇异值置零
	_T S[3 * 3] = { ((_T*)oSVD.S)[0],0,0,
				0,((_T*)oSVD.S)[1],0,
				0,0,0 };
	Matrix_Multiply((_T*)oSVD.U, 3, 3 ,S, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)oSVD.Vt, 3, F);

	//在算 M2' * F * M1恢复原坐标
	Matrix_Transpose((_T*)M_2, 3, 3, (_T*)M_2);
	Matrix_Multiply((_T*)M_2, 3, 3, F, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)M_1, 3, F);
	Free_SVD(&oSVD);
	if (bUse_Matrix_Mem)
		Free(&oMatrix_Mem, oPtr.m_pBuffer);
	//Disp(F, 3, 3, "F");
	//Disp_Mem(&oMatrix_Mem, 0);
}
template<typename _T> void Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T (*Norm_Point_1)[2], _T (*Norm_Point_2)[2], Light_Ptr oPtr)
{//做一个更清爽的接口，Norm_Point_1 ，Norm_Point_2可以不开空间，oPtr可以没有
//理论上这已经是完备接口，给与一组点 x1,x2，求得一个矩阵E， 使得 x2'*E*x1=0
	_T M_1[3][3], M_2[3][3];
	unsigned char* pCur;
	int i, iSize,bUse_Matrix_Mem;

	bUse_Matrix_Mem = (!Norm_Point_1 || !Norm_Point_2 || !oPtr.m_pBuffer) ? 1 : 0;
	if (bUse_Matrix_Mem)
	{
		iSize = iCount * 4 * sizeof(_T) +	//Norm_Point
			iCount * 9 * sizeof(_T) +		//A
			128 * 3;

		Attach_Light_Ptr(oPtr,(unsigned char*)pMalloc(&oMatrix_Mem,iSize), iSize, -1);
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_1 = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_2= (_T(*)[2])pCur;
	}
	
	Normalize_Point_1(Point_1, iCount, M_1, Norm_Point_1);
	Normalize_Point_1(Point_2, iCount, M_2, Norm_Point_2);
	
	//从以下可以看到，经过相对中心的规格化后，两者数值进一步接近
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, iCount, 2, "Norm_Point_2");
	//Disp((_T*)Point_2, iCount, 2, "Point_2");
	//构造系数系数矩阵
	//Point_1 x E(3x3) x Point_2' 展开，为行向量
	//Disp((float*)pNorm_Point_1, iCount, 2);
	_T(*A)[9];	//, B[8] = { 0 };
	Malloc(oPtr, iCount * 9 * sizeof(_T), pCur);
	A = (_T(*)[9])pCur;
	for (i = 0; i < iCount; i++)
	{
		A[i][0] = Norm_Point_2[i][0] * Norm_Point_1[i][0];
		A[i][1] = Norm_Point_2[i][0] * Norm_Point_1[i][1];
		A[i][2] = Norm_Point_2[i][0];
		A[i][3] = Norm_Point_2[i][1] * Norm_Point_1[i][0];
		A[i][4] = Norm_Point_2[i][1] * Norm_Point_1[i][1];
		A[i][5] = Norm_Point_2[i][1];
		A[i][6] = Norm_Point_1[i][0];
		A[i][7] = Norm_Point_1[i][1];
		A[i][8] = 1;
	}
	
	//Disp((_T*)A, 8, 9);
	//正式求解
	_T Basic_Solution[9 * 9];
	SVD_Info oSVD;
	if (iCount == 8)
	{	//注意，当Ax=0 中的A为横形，此时可以不用svd求解，就是个齐次方程求基础解系的过程，也许更快更准
		int iBasic_Solution_Count, iResult;
		_T B[9] = { 0 };
		Solve_Linear_Solution_Construction_1((_T*)A, iCount, 9, B, &iResult, Basic_Solution, &iBasic_Solution_Count);
		//Disp(Basic_Solution, 3, 3);
	}else
	{
		SVD_Alloc<_T>(iCount, 9, &oSVD);
		//Disp((_T*)A, 8, 9, "A");
		svd_3((_T*)A, oSVD, &oSVD.m_bSuccess);
		//Disp((_T*)oSVD.Vt, 9, 9, "Vt");
		memcpy(Basic_Solution, ((_T(*)[9])oSVD.Vt)[8], 9 * sizeof(_T));
		//Disp(Basic_Solution, 3, 3);
		//第一次SVD已经结束使命
		Free_SVD(&oSVD);
	}
	
	//由解再组合成一个E'矩阵，注意，这里搞了点多此一举的转置造成后面费解
	//正确的E = [	e1 e2 e3		而此处是 [	e1 e4 e7	所以这是E'
	//				e3 e4 e5					e2 e5 e8
	//				e6 e7 e8 ]					e3 e6 e9]
	_T E_raw[3*3], Temp_1[3 * 3], Temp_2[3 * 3],
		E_t[3 * 3] = { Basic_Solution[0],Basic_Solution[3],Basic_Solution[6],
		Basic_Solution[1],Basic_Solution[4],Basic_Solution[7],
		Basic_Solution[2],Basic_Solution[5],Basic_Solution[8] };
	//Disp((_T*)E_t, 3, 3, "Et");

	//要推导以下以下E_raw是个什么东西
	// NP_2' * E * NP_1 =0		这是最初的最优化问题，在规格化点上做
	//展开规格化点  (M_2 * P2)' * E * M_1 * P1=0  又根据矩阵乘的转置等于转置后矩阵的乘有
	//=>  P2' * M_2' * E * M_1 * P1=0	可以将中间视为整体E_raw
	//=》 E_raw = M_2' * E * M_1
	// De-normalize to image points.
	Matrix_Transpose((_T*)M_2, 3, 3, Temp_1);
	Matrix_Transpose(E_t, 3, 3, Temp_2);
	Matrix_Multiply(Temp_1, 3, 3, Temp_2, 3, E_raw);
	Matrix_Multiply(E_raw, 3, 3, (_T*)M_1, 3, E_raw);
	
	//Disp(E_raw, 3, 3, "E_raw");
	//第二次SVD, 为E_raw
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(E_raw, oSVD);
	//Disp((_T*)oSVD.S, 1, 3,"S");
	//Disp((_T*)oSVD.Vt, 3, 3, "Vt");

	_T S[3 * 3] = {};
	//然后进行一个骚操作，对角化的三个元素进行调整，暂时不知其理。
	//但是在QR分解中则有一种情况，因为计算误差两个重根存在些许差，此时
	//应该将可疑重根求和再平均，此时误差最小
	S[0] = S[4] = ( ((_T*)oSVD.S)[0]  + ((_T*)oSVD.S)[1]) / 2.f;
	S[8] = 0.f;

	//Disp(S, 3, 3, "S");
	Matrix_Multiply((_T*)oSVD.U, 3, 3, S, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)oSVD.Vt, 3, E);
	Free_SVD(&oSVD);
	if (bUse_Matrix_Mem)
		Free(&oMatrix_Mem, oPtr.m_pBuffer);
	//Disp(E, 3, 3, "E");
	return;
}

template<typename _T> void Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T H[3 * 3], _T (*Norm_Point_1)[2], _T (*Norm_Point_2)[2], Light_Ptr oPtr)
{//给定的数据求解一个Homograph矩阵，满足 min ( ||P1*X-P2||^2)
//理论上该接口已经完备，给与一组点，存在一个矩阵H，s.t. H*p1=p2
	_T M_1[3][3], M_2[3][3];
	int i, j,iResult,bAllocate_Large;
	
	bAllocate_Large = (!Norm_Point_1 || !Norm_Point_2 || !oPtr.m_pBuffer) ? 1 : 0;
	if (bAllocate_Large)
	{
		unsigned char* pCur;
		int iSize = iCount * 4 * sizeof(_T) +	//Norm_Point
			iCount * 9 *2 * sizeof(_T) +		//A
			128 * 3;

		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(&oMatrix_Mem, iSize), iSize, -1);
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_1 = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_2 = (_T(*)[2])pCur;
	}else
	{
		//unsigned char* pCur;
		int iSize = iCount * 9 *2 * sizeof(_T) +		//A
			128;
		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(&oMatrix_Mem, iSize), iSize, -1);
	}

	//从以下可以看到，经过相对中心的规格化后，两者数值进一步接近
	Normalize_Point_3(Point_1, iCount, M_1, Norm_Point_1);
	Normalize_Point_3(Point_2, iCount, M_2, Norm_Point_2);
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, 4, 2, "Norm_Point_2");

	_T(*A)[9];	// , * S, * Vt;
	unsigned char* pCur;

	//此处分配有问题
	Malloc(oPtr, iCount * 2 * 9 * sizeof(_T), pCur);
	A = (_T(*)[9])pCur;

	memset(A, 0, iCount * 2 * 9 * sizeof(_T));
	for (i = 0, j = iCount; i < iCount; i++, j++)
	{
		_T s_0 = Norm_Point_1[i][0];
		_T s_1 = Norm_Point_1[i][1];
		_T d_0 = Norm_Point_2[i][0];
		_T d_1 = Norm_Point_2[i][1];

		A[i][0] = -s_0;
		A[i][1] = -s_1;
		A[i][2] = -1;
		A[i][6] = s_0 * d_0;
		A[i][7] = s_1 * d_0;
		A[i][8] = d_0;

		A[j][3] = -s_0;
		A[j][4] = -s_1;
		A[j][5] = -1;
		A[j][6] = s_0 * d_1;
		A[j][7] = s_1 * d_1;
		A[j][8] = d_1;
	}
	//Disp((_T*)A, 8, 9, "A");
	SVD_Info oSVD;
	//SVD_Allocate( (_T*)(A) , iCount*2, 9, &oSVD);
	SVD_Alloc<_T>(iCount * 2, 9, &oSVD);

	svd_3( (_T*)A, oSVD, &iResult);
	//Disp((_T*)oSVD.Vt, oSVD.h_Min_Vt, oSVD.w_Min_Vt, "Vt");
	
	_T H_t[3 * 3];
	memcpy(H_t, &((_T*)oSVD.Vt)[8 * 9], 9 * sizeof(_T));
	//Disp(H_t, 3, 3, "H");

	Free_SVD(&oSVD);
	_T Temp_1[3 * 3];
	Get_Inv_Matrix_Row_Op((_T*)M_2, (_T*)M_2, 3, &iResult);
	Matrix_Multiply((_T*)M_2, 3, 3, H_t, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)M_1, 3, H);
	//Disp(H, 3, 3, "H");
	Free(&oMatrix_Mem, oPtr.m_pBuffer);
	return;
}

template<typename _T>
void Get_Residual_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T H[3 * 3], _T Residual[])
{//对H矩阵的误差估计不能用Sampson距离，用的还是欧氏距离，待验证
	_T H_00 = H[0], H_01 = H[1], H_02 = H[2], H_10 = H[3],
		H_11 = H[4], H_12 = H[5], H_20 = H[6], H_21 = H[7], H_22 = H[8];

	for (int i = 0; i < iCount; i++)
	{
		_T s_0 = Point_1[i][0];
		_T s_1 = Point_1[i][1];
		_T d_0 = Point_2[i][0];
		_T d_1 = Point_2[i][1];

		//此处向做了个H * Point_1 的变换
		_T pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
		_T pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
		_T pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

		//此处像投影
		_T inv_pd_2 = 1.f / pd_2;
		_T dd_0 = d_0 - pd_0 * inv_pd_2;
		_T dd_1 = d_1 - pd_1 * inv_pd_2;

		//欧氏距离
		Residual[i] = dd_0 * dd_0 + dd_1 * dd_1;
	}
}
template<typename _T>
void Get_Support(_T Residual[], int iCount, _T fMax_Residual, int* piInlier_Count, float* pfResidual_Sum)
{//根本没什么营养，就是比个误差，
	int iInlier_Count = 0;
	_T fSum = 0;
	for (int i = 0; i < iCount; i++)
		if (Residual[i] <= fMax_Residual)
			iInlier_Count++, fSum += Residual[i];
	if (piInlier_Count)
		*piInlier_Count = iInlier_Count;
	if (pfResidual_Sum)
		*pfResidual_Sum = (float)fSum;
}
int iCompute_Num_Trials(int iInlier_Count, int iSample_Count, int kMinNumSamples)
{//计算dyn_max_num_trials,动态测试次数
	//当Inlier增大，fInlier_Ratio增大，fInlier_Ratio就是找到匹配点的比例
	float fInlier_Ratio = (float)iInlier_Count / iSample_Count;
	float nom = 1.f - 0.99f;
	//Inlier增大，denom减少。 fInlier_Ration ^ (4|8)，高次函数x^(4|8)看[0,1]区间图像即可
	float denom = 1.f - (float)pow(fInlier_Ratio, kMinNumSamples);
	int iRet;
	if (denom <= 0)
		return 1;
	if (denom == 1.0)	//此时表示Inlier_Ratio=0，找不到任何匹配点
		return (int)(std::numeric_limits<size_t>::max());	//返回个最大值
	
	//printf("nom:%f denom:%f log(nom):%f log(denom):%f\n",nom,denom,log(nom),log(denom));
	//log(nom)= -4.605171, 总为负 denom<1, log（denom) 总为负
	//inlier 增大，denom递减，|log(denom)| 递增，结果递减
	//其实从总体上来看，无非就是匹配的点数越多，继续迭代的次数酒约少。有兴趣可以自己搞个曲线
	iRet = (int)ceil(log(nom) / log(denom) * 3.f);
	return iRet;
}
template<typename _T>void Normalize_Point(_T(*pPoint_1)[2], _T(*pPoint_2)[2], int iCount, _T(*pNorm_Point_1)[2], _T(*pNorm_Point_2)[2], float f, float c1, float c2)
{//用相机内参将屏幕坐标投影到归一化平面上，就是焦距为1的成像平面
	for (int i = 0; i < iCount; i++)
	{	//按照u= a*f*x/z + cx => x= z* (u-cx) /af	//假设a=1
		//然而，源代码用的式子是： (x - c1) / f， 那么z=1.此处存疑
		//猜想假定每个点的z距离是1？
		//OK了，这个过程就是通过相机内参将所有的像素点投影到归一化平面上。
		// 矩阵形式就是 x= K(-1) * p	其中P为像素点位置，x为归一化后位置， K(-1)为 K的逆
		//目前，能想象到的的变换是物体里相机光心为1像素。焦距为768像素，恢复中心点
		//故此物体在在世界的坐标为 (x-c1)/f 坐标 在 -0.5,0.5之间
		pNorm_Point_1[i][0] = (pPoint_1[i][0] - c1) / f;
		pNorm_Point_1[i][1] = (pPoint_1[i][1] - c2) / f;

		pNorm_Point_2[i][0] = (pPoint_2[i][0] - c1) / f;
		pNorm_Point_2[i][1] = (pPoint_2[i][1] - c2) / f;
		//printf("%f %f\n", pNorm_Point_1[i][0], pNorm_Point_1[i][1]);
	}	
	//Disp((float*)pNorm_Point_2, iMatch_Count, 2, "Point_2");
}

template<typename _T> void Compute_Squared_Sampson_Error(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T Residual[])
{//计算Sampson距离
	const _T E_00 = E[0 * 3 + 0], E_01 = E[0 * 3 + 1], E_02 = E[0 * 3 + 2],
		E_10 = E[1 * 3 + 0], E_11 = E[1 * 3 + 1], E_12 = E[1 * 3 + 2],
		E_20 = E[2 * 3 + 0], E_21 = E[2 * 3 + 1], E_22 = E[2 * 3 + 2];
	//float fSum = 0;
	for (int i = 0; i < iCount; i++)
	{
		_T x1_0 = Point_1[i][0], x1_1 = Point_1[i][1],
			x2_0 = Point_2[i][0], x2_1 = Point_2[i][1];

		//求 E*x1
		// Ex1 = E * points1[i].homogeneous();
		_T Ex1_0 = E_00 * x1_0 + E_01 * x1_1 + E_02,
			Ex1_1 = E_10 * x1_0 + E_11 * x1_1 + E_12,
			Ex1_2 = E_20 * x1_0 + E_21 * x1_1 + E_22;

		//求 E'*x2
		// Etx2 = E.transpose() * points2[i].homogeneous();
		_T Etx2_0 = E_00 * x2_0 + E_10 * x2_1 + E_20,
			Etx2_1 = E_01 * x2_0 + E_11 * x2_1 + E_21;

		//求 x2'*E*x1 这个是E的定义
		// x2tEx1 = points2[i].homogeneous().transpose() * Ex1;
		_T x2tEx1 = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;

		// Sampson distance 这个距离待理解
		Residual[i] = x2tEx1 * x2tEx1 / (Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1 + Etx2_0 * Etx2_0 + Etx2_1 * Etx2_1);
	}
	
	////猜该距离的实际数学表示
	//for (int i = 0; i < iCount; i++)
	//{
	//	_T Temp_1[3] = { Point_1[i][0],Point_1[i][1],1 };
	//	_T Temp_2[3] = { Point_2[i][0],Point_2[i][1],1 };
	//	_T Temp[3];
	//	Matrix_Multiply(Temp_1, 1, 3, E, 3, Temp);
	//	Matrix_Multiply(Temp, 1, 3, Temp_2, 1, Temp);
	//	printf("%f\n", Temp[0]);
	//}
}
template<typename _T> void Ransac_Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr)
{//按照算法，算的是 p0*E*x1=0， 其中p0,p1是原位置
#define SAMPLE_COUNT 8	//估计这个模型所需要的最小样本数
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//为了速度，Norm_Point 先分配内存

	_T X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], F[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial, dyn_max_num_trials = 0, bAbort = 0;
	//所谓Local指的是用粗糙的8点法求出的变换为基础，将所有的点都加入进来，一起迭代多次求得的变换
	Ransac_Support oLocal_Support = {}, oCur_Support = {};	//够大就行
	Ransac_Report oReport = {};

	//所谓 Best 指的是最优解
	_T Best_Modal[3 * 3];
	//int bBest_Model_is_Local;
	Ransac_Support oBest_Support = { 0,(float)1e30 };

	//Ransac要素1, 必须指定一个最大误差阈值。大于此阈值的样本不计入内
	_T fMax_Residual = 4 * 4;
	{//以下分配内存，有点蠢，但又必需
		unsigned char* pCur;
		int iSize = ALIGN_SIZE_128(iCount * sizeof(short) +	//Sample Index
			iCount * 5 * sizeof(_T) +					//Residual, Norm_Point_1,Norm_Point_2,pDup_Point_1,pDup_Point_2
			iCount * 9 * sizeof(_T) +					//Estimate_E
			128 * 4);
		if (pMem_Mgr)
			poMem_Mgr = pMem_Mgr;
		else
		{
			poMem_Mgr = (Mem_Mgr*)malloc(sizeof(Mem_Mgr));
			//iSize = ALIGN_SIZE_128(iCount * sizeof(short) + iCount * 5 * sizeof(_T) + iCount * iCount + 128 * 4);
			Init_Mem_Mgr(poMem_Mgr, iSize, 1024, 997);
			if (!poMem_Mgr->m_pBuffer)
				return;
		}
		//Report那点空间放在最底
		if (pMem_Mgr)
			oReport.m_pInlier_Mask = (unsigned char*)pMalloc(poMem_Mgr, iCount * sizeof(unsigned char));	//oReport.m_pInlier_Mask
		else
			oReport.m_pInlier_Mask = (unsigned char*)malloc(iCount * sizeof(unsigned char));
		oReport.m_iFloat_Size = sizeof(_T);

		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(poMem_Mgr, iSize), iSize, -1);

		if (!oPtr.m_pBuffer)
		{
			printf("Fail to allocate memory\n");
			return;
		}
		Malloc(oPtr, iCount * sizeof(short), pCur);		//Sample Index
		pSample_Index = (short*)pCur;
		Malloc_1(oPtr, iCount * sizeof(_T), pResidual);		//Residual
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_1
		pNorm_Point_1 = pX_Inlier = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_2
		pNorm_Point_2 = pY_Inlier = (_T(*)[2])pCur;
	}

	for (i = 0; i < iCount; i++)	//初始化样本索引，后面是随机选取样本，不断打乱。这里保证索引乱点不乱
		pSample_Index[i] = i;

	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}
		//Ransac要素2，随机取一组点构造一个粗糙的解。这个解对于这小数量点是成立的，
		//但对所有的其他点集是粗糙的		
		Sample_XY(Point_1, Point_2, pSample_Index, iCount, X_rand, Y_rand, SAMPLE_COUNT);
		//Ransac要素3，求解算法。每种数据都有不同的求解方法，不一而足，故此Ransac只有
		//程序结构上的意义，没有细节上的意义，此处要自己写自己的求解方法
		Estimate_F(X_rand, Y_rand, SAMPLE_COUNT, F, pNorm_Point_1, pNorm_Point_2, oPtr);
		//Estimate_F(X_rand, Y_rand, SAMPLE_COUNT, F);
		
		//求Sampson误差
		Compute_Squared_Sampson_Error(Point_1, Point_2, iCount, F, pResidual);
		Get_Support(pResidual, iCount, fMax_Residual, &oCur_Support.m_iInlier_Count, &oCur_Support.m_fResidual_Sum);
		if (oCur_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oCur_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oCur_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
		{
			oBest_Support = oCur_Support;
			//bBest_Model_is_Local = 0;
			memcpy(Best_Modal, F, 9 * sizeof(_T));
			int prev_best_num_inliers = oBest_Support.m_iInlier_Count;
			if (oCur_Support.m_iInlier_Count > SAMPLE_COUNT)
			{
				for (iTrial = 0; iTrial < 10; iTrial++)
				{
					//再把m_iInlier_Count个样本做一次E估计，这次就不是用八点了
					for (j = i = 0; i < iCount; i++)
					{
						if (pResidual[i] < fMax_Residual)
						{
							pX_Inlier[j][0] = Point_1[i][0];
							pX_Inlier[j][1] = Point_1[i][1];
							pY_Inlier[j][0] = Point_2[i][0];
							pY_Inlier[j][1] = Point_2[i][1];
							j++;
						}
					}
					prev_best_num_inliers = oBest_Support.m_iInlier_Count;
					Estimate_F(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, F, pNorm_Point_1, pNorm_Point_2, oPtr);
					//Estimate_F(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, F);
					Compute_Squared_Sampson_Error(Point_1, Point_2, iCount, F, pResidual);
					Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
					if (oLocal_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oLocal_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oLocal_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
					{
						oBest_Support = oLocal_Support;
						memcpy(Best_Modal, F, 9 * sizeof(_T));
						//bBest_Model_is_Local = 1;
					}
					if (oBest_Support.m_iInlier_Count <= prev_best_num_inliers)
						break;
				}
				dyn_max_num_trials = iCompute_Num_Trials(oBest_Support.m_iInlier_Count, iCount, SAMPLE_COUNT);
			}			
		}
		if (oReport.m_iTrial_Count >= dyn_max_num_trials && oReport.m_iTrial_Count >= 30)
		{
			bAbort = true;
			break;
		}
	}
	oReport.m_oSupport = oBest_Support;
	memcpy(oReport.m_Modal, Best_Modal, 9 * sizeof(_T));

	if (oReport.m_oSupport.m_iInlier_Count > 8)
		oReport.m_bSuccess = 1;

	Compute_Squared_Sampson_Error(Point_1, Point_2, iCount, Best_Modal, pResidual);
	Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
	for (i = 0; i < iCount; i++)
	{
		if (pResidual[i] <= fMax_Residual)
			oReport.m_pInlier_Mask[i] = 1;
		else
			oReport.m_pInlier_Mask[i] = 0;
	}
	oReport.m_iSample_Count = iCount;
	if (poReport)
		*poReport = oReport;

	//Disp((_T*)oReport.m_Modal, 3, 3);
	//最后释放
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//自己开的Mem_Mgr自己负责释放
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
}
template<typename _T> void Ransac_Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, float f,float c1,  float c2, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr)
{//注意，此处应当与求解H矩阵的方法有别，来的Point_1，Point_2已经是Normalized
//按照算法，算的是 x0*E*x1=0, 其中 x0,x1是归一化后的点

#define SAMPLE_COUNT 8	//估计这个模型所需要的最小样本数
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual,*pBest_Residual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pDup_Point_1)[2], (*pDup_Point_2)[2],		//这是Point_1, Point_2恢复到归一化平面的坐标
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//为了速度，Norm_Point 先分配内存

	_T Best_Modal[3 * 3], X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], E[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial,/* bBest_Model_is_Local,*/ dyn_max_num_trials = 0, bAbort = 0;
	Ransac_Support oLocal_Support = {}, oCur_Support = {}, oBest_Support = { 0,(float)1e30 };	//够大就行
	Ransac_Report oReport = {};

	//Ransac要素1, 必须指定一个最大误差阈值。大于此阈值的样本不计入内
	_T fMax_Residual = 0.005208333333333333f * 0.005208333333333333f;

	{//以下分配内存，有点蠢，但又必需
		unsigned char* pCur;
		int iSize = ALIGN_SIZE_128(iCount * 2 * sizeof(short) +	//Sample Index
			iCount * 9 * sizeof(_T) * 2 +					//Residual, Norm_Point_1,Norm_Point_2,pDup_Point_1,pDup_Point_2
			iCount * 9 * sizeof(_T) +						//Estimate_E
			128 * 4);	
		if (pMem_Mgr)
			poMem_Mgr = pMem_Mgr;
		else
		{
			poMem_Mgr = (Mem_Mgr*)malloc(sizeof(Mem_Mgr));
			//iSize = ALIGN_SIZE_128(iCount * sizeof(short) + iCount * 5 * sizeof(_T) + iCount * iCount + 128 * 4);
			Init_Mem_Mgr(poMem_Mgr, iSize, 1024, 997);
			if (!poMem_Mgr->m_pBuffer)
				return;
		}
		//Report那点空间放在最底
		if (pMem_Mgr)
			oReport.m_pInlier_Mask = (unsigned char*)pMalloc(poMem_Mgr, iCount * sizeof(unsigned char));	//oReport.m_pInlier_Mask
		else
			oReport.m_pInlier_Mask = (unsigned char*)malloc(iCount * sizeof(unsigned char));
		oReport.m_iFloat_Size = sizeof(_T);

		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(poMem_Mgr, iSize), iSize, -1);
		
		if (!oPtr.m_pBuffer)
		{
			printf("Fail to allocate memory\n");
			return;
		}
		Malloc(oPtr, iCount * sizeof(short), pCur);		//Sample Index
		pSample_Index = (short*)pCur;
		Malloc_1(oPtr, iCount * sizeof(_T), pResidual);		//Residual
		Malloc_1(oPtr, iCount * sizeof(_T), pBest_Residual);	//Best Residule
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_1
		pNorm_Point_1 = pX_Inlier = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_2
		pNorm_Point_2 = pY_Inlier = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//pDup_Point_1
		pDup_Point_1 = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//pDup_Point_2
		pDup_Point_2 = (_T(*)[2])pCur;
	}
	for (i = 0; i < iCount; i++)	//初始化样本索引，后面是随机选取样本，不断打乱。这里保证索引乱点不乱
		pSample_Index[i] = i;

	//将点恢复到归一化平面
	Normalize_Point(Point_1, Point_2, iCount, pDup_Point_1, pDup_Point_2, f, c1, c2);

	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}
		//Ransac要素2，随机取一组点构造一个粗糙的解。这个解对于这小数量点是成立的，
		//但对所有的其他点集是粗糙的		
		Sample_XY(pDup_Point_1, pDup_Point_2, pSample_Index, iCount, X_rand, Y_rand, SAMPLE_COUNT);

		//Ransac要素3，求解算法。每种数据都有不同的求解方法，不一而足，故此Ransac只有
		//程序结构上的意义，没有细节上的意义，此处要自己写自己的求解方法
		//Estimate_E(X_rand, Y_rand, SAMPLE_COUNT, E, pNorm_Point_1, pNorm_Point_2, oPtr);
		Estimate_E(X_rand, Y_rand, SAMPLE_COUNT, E);
		
		//求Sampson误差
		Compute_Squared_Sampson_Error(pDup_Point_1, pDup_Point_2, iCount, E, pResidual);
		Get_Support(pResidual, iCount, fMax_Residual, &oCur_Support.m_iInlier_Count, &oCur_Support.m_fResidual_Sum);
		//if (oCur_Support.m_iInlier_Count == 8)
			//printf("Here");
		if (oCur_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oCur_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oCur_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
		{
			//if (oCur_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oCur_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oCur_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
			{
				memcpy(Best_Modal, E, 9 * sizeof(_T));
				memcpy(pBest_Residual, pResidual, iCount * sizeof(_T));
				//bBest_Model_is_Local = 1;
			}
			oBest_Support = oCur_Support;
			
			//bBest_Model_is_Local = 0;
			//memcpy(Best_Modal, E, 9 * sizeof(_T));
			int prev_best_num_inliers = oBest_Support.m_iInlier_Count;
			if (oCur_Support.m_iInlier_Count > SAMPLE_COUNT)
			{
				for (iTrial = 0; iTrial < 10; iTrial++)
				{
					//再把m_iInlier_Count个样本做一次E估计，这次就不是用八点了
					for (j = i = 0; i < iCount; i++)
					{
						if (pResidual[i] < fMax_Residual)
						{
							pX_Inlier[j][0] = pDup_Point_1[i][0];
							pX_Inlier[j][1] = pDup_Point_1[i][1];
							pY_Inlier[j][0] = pDup_Point_2[i][0];
							pY_Inlier[j][1] = pDup_Point_2[i][1];
							j++;
						}
					}
					prev_best_num_inliers = oBest_Support.m_iInlier_Count;
					//Estimate_E(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, E, pNorm_Point_1, pNorm_Point_2, oPtr);
					Estimate_E(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, E);
					//Disp(E, 3, 3, "E");
					Compute_Squared_Sampson_Error(pDup_Point_1, pDup_Point_2, iCount, E, pResidual);
					Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
					if (oLocal_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oLocal_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oLocal_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
					{
						oBest_Support = oLocal_Support;
						memcpy(Best_Modal, E, 9 * sizeof(_T));
						memcpy(pBest_Residual, pResidual, iCount * sizeof(_T));
						//bBest_Model_is_Local = 1;
					}
					if (oBest_Support.m_iInlier_Count <= prev_best_num_inliers)
						break;
				}
				dyn_max_num_trials = iCompute_Num_Trials(oBest_Support.m_iInlier_Count, iCount, SAMPLE_COUNT);
			}			
		}
		if (oReport.m_iTrial_Count >= dyn_max_num_trials && oReport.m_iTrial_Count >= 30)
		{
			bAbort = true;
			break;
		}
	}

	oReport.m_oSupport = oBest_Support;
	memcpy(oReport.m_Modal, Best_Modal, 9 * sizeof(_T));

	if (oReport.m_oSupport.m_iInlier_Count > 8 || (iCount==8 && oBest_Support.m_iInlier_Count==8))
		oReport.m_bSuccess = 1;

	for (i = 0; i < iCount; i++)
	{
		if (pBest_Residual[i] <= fMax_Residual)
			oReport.m_pInlier_Mask[i] = 1;
		else
			oReport.m_pInlier_Mask[i] = 0;
	}

	oReport.m_iSample_Count = iCount;
	if (poReport)
		*poReport = oReport;

	//Disp((_T*)oReport.m_Modal, 3, 3);
	//最后释放
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//自己开的Mem_Mgr自己负责释放
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
	return;
#undef SAMPLE_COUNT
}
//template<typename _T>void Estimate_s_by_H(_T Point_1[][2], _T Point_2[][2], _T H[3 * 3], int iCount, _T* ps)
//{//根据样本优化s, p2 = sHp1
//}
template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr, _T fMax_Residual)
{//返回的Report统一为double类型
#define SAMPLE_COUNT 4
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, * pBest_Residual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//为了速度，Norm_Point 先分配内存

	_T Best_Modal[3 * 3], X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], H[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial,/* bBest_Model_is_Local, */dyn_max_num_trials = 0, bAbort = 0;
	int prev_best_num_inliers;
	Ransac_Support oLocal_Support = {}, oCur_Support = {}, oBest_Support = { 0,(float)1e30 };	//够大就行
	Ransac_Report oReport = {};

	//Ransac要素1, 必须指定一个最大误差阈值。大于此阈值的样本不计入内
	//_T fMax_Residual = 4 * 4;

	unsigned char* pCur;
	int iSize;

	{//以下分配内存，有点蠢，但又必需
		iSize = ALIGN_SIZE_128(iCount * sizeof(short) + 
			iCount * 5 * sizeof(_T) +
			iCount * 9 * 2 * sizeof(_T)+		//A Vt S //iCount * iCount +
			128 * 4);						//补余数凑128字节对齐
		if (pMem_Mgr)
			poMem_Mgr = pMem_Mgr;
		else
		{
			poMem_Mgr = (Mem_Mgr*)malloc(sizeof(Mem_Mgr));
			//iSize = ALIGN_SIZE_128(iCount * sizeof(short) + iCount * 5 * sizeof(_T) + iCount * iCount + 128 * 4);
			Init_Mem_Mgr(poMem_Mgr, iSize, 1024, 997);
			if (!poMem_Mgr->m_pBuffer)
				return;
		}
		if (pMem_Mgr)
			oReport.m_pInlier_Mask = (unsigned char*)pMalloc(poMem_Mgr, iCount * sizeof(unsigned char));	//oReport.m_pInlier_Mask
		else
			oReport.m_pInlier_Mask = (unsigned char*)malloc(iCount * sizeof(unsigned char));
		oReport.m_iFloat_Size = sizeof(_T);

		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(poMem_Mgr, iSize), iSize, -1);
		if (!oPtr.m_pBuffer)
		{
			printf("Fail to allocate memory\n");
			return;
		}
		Malloc(oPtr, iCount * sizeof(short), pCur);		//Sample Index
		pSample_Index = (short*)pCur;
		Malloc(oPtr, iCount * sizeof(_T), pCur);		//Residual
		pResidual = (_T*)pCur;
		Malloc_1(oPtr, iCount * sizeof(_T), pBest_Residual);	//Best Residule
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_1
		pNorm_Point_1 = pX_Inlier = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_2
		pNorm_Point_2 = pY_Inlier = (_T(*)[2])pCur;
		//Malloc(oPtr, iCount * sizeof(unsigned char), pCur);
	}	
	for (i = 0; i < iCount; i++)	//初始化样本索引，后面是随机选取样本，不断打乱。这里保证索引乱点不乱
		pSample_Index[i] = i;
	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}

		//Ransac要素2，随机取一组点构造一个粗糙的解。这个解对于这小数量点是成立的，
		//但对所有的其他点集是粗糙的		
		Sample_XY(Point_1, Point_2, pSample_Index, iCount, X_rand, Y_rand, 4);

		//Ransac要素3，求解算法。每种数据都有不同的求解方法，不一而足，故此Ransac只有
		//程序结构上的意义，没有细节上的意义，此处要自己写自己的求解方法
		Estimate_H(X_rand, Y_rand, 4, H, pNorm_Point_1, pNorm_Point_2,oPtr);
		//Disp(H, 3, 3, "H");

		//此处似乎用欧氏距离
		Get_Residual_H(Point_1, Point_2, iCount, H, pResidual);
		Get_Support(pResidual, iCount, fMax_Residual, &oCur_Support.m_iInlier_Count, &oCur_Support.m_fResidual_Sum);
		//printf("4 Point Trial:%d Inlier:%d Residual:%f\n", oReport.m_iTrial_Count, oCur_Support.m_iInlier_Count, oCur_Support.m_fResidual_Sum);

		if (oCur_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oCur_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oCur_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
		{
			oBest_Support = oCur_Support;
			//bBest_Model_is_Local = 0;
			memcpy(Best_Modal, H, 9 * sizeof(_T));
			memcpy(pBest_Residual, pResidual, iCount * sizeof(_T));

			prev_best_num_inliers = oBest_Support.m_iInlier_Count;

			if (oCur_Support.m_iInlier_Count > 4)
			{
				for (iTrial = 0; iTrial < 10; iTrial++)
				{//此处的结构很明了，从一个Modal出发，不断用全部点调整这个Modal，直到点数不再增加
				//才跳出迭代。后面搞了个迭代次数衰减的参数，也没啥营养，完全可以自己拍脑袋
					//再把m_iInlier_Count个样本做一次E估计，这次就不是用八点了
					for (j = i = 0; i < iCount; i++)
					{
						if (pResidual[i] < fMax_Residual)
						{
							pX_Inlier[j][0] = Point_1[i][0];
							pX_Inlier[j][1] = Point_1[i][1];
							pY_Inlier[j][0] = Point_2[i][0];
							pY_Inlier[j][1] = Point_2[i][1];
							j++;
						}
					}
					prev_best_num_inliers = oBest_Support.m_iInlier_Count;
					Estimate_H(pX_Inlier, pY_Inlier,oBest_Support.m_iInlier_Count, H, pNorm_Point_1, pNorm_Point_2, oPtr);
					Get_Residual_H(Point_1, Point_2, iCount, H, pResidual);
					Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
					
					if (oLocal_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oLocal_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oLocal_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
					{
						oBest_Support = oLocal_Support;
						memcpy(Best_Modal, H, 9 * sizeof(_T));
						memcpy(pBest_Residual, pResidual, iCount * sizeof(_T));
						//bBest_Model_is_Local = 1;
					}
					
					if (oBest_Support.m_iInlier_Count <= prev_best_num_inliers)
						break;
					//printf("4 points Trial:%d Full Trial:%d Inlier:%d Residual:%f\n", oReport.m_iTrial_Count,iTrial,
						//oBest_Support.m_iInlier_Count,oBest_Support.m_fResidual_Sum);
				}
			}
			//完全可以自己拍脑袋
			dyn_max_num_trials = iCompute_Num_Trials(oBest_Support.m_iInlier_Count, iCount, 4);
			//printf("dyn_max_num_trials:%d\n", dyn_max_num_trials);
		}
		if (oReport.m_iTrial_Count >= dyn_max_num_trials && oReport.m_iTrial_Count >= 30)
		{
			bAbort = 1;
			break;
		}
	}

	oReport.m_oSupport = oBest_Support;
	memcpy(oReport.m_Modal, Best_Modal, 9 * sizeof(_T));
	
	if (oReport.m_oSupport.m_iInlier_Count > 8)
		oReport.m_bSuccess = 1;
	
	//Get_Residual_H(Point_1, Point_2, iCount, Best_Modal, pResidual);
	//Get_Support(pResidual, iCount, fMax_Residual, &oBest_Support.m_iInlier_Count, &oBest_Support.m_fResidual_Sum);

	int iTemp_Count = 0;

	for (i = 0; i < iCount; i++)
	{
		if (pBest_Residual[i] <= fMax_Residual)
			oReport.m_pInlier_Mask[i] = 1, iTemp_Count++;
		else
			oReport.m_pInlier_Mask[i] = 0;
	}

	////以下想寻找一个统一的s使得 p2 = sHp1，结果不靠谱
	//_T f0=0, f1=0,s;
	//for (i = 0; i < iCount; i++)
	//{
	//	if (oReport.m_pInlier_Mask[i])
	//	{
	//		_T p1[] = {Point_1[i][0],Point_1[i][1],1.f},
	//			p2[] = {Point_2[i][0],Point_2[i][1],1};
	//		Matrix_Multiply_3x1((_T*)oReport.m_Modal, p1, p1);
	//		f0 += p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2];
	//		f1 += p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2];
	//		printf("%f\n", (p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2]) / (p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2]));
	//	}
	//}
	//s = f0 / f1;

	if (iTemp_Count != oReport.m_oSupport.m_iInlier_Count)
		printf("Err in Ransac_Estimate_H\n");

	oReport.m_iSample_Count = iCount;
	if (poReport)
		*poReport = oReport;

	//最后释放
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//自己开的Mem_Mgr自己负责释放
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
#undef SAMPLE_COUNT
}
void Free_Report(Ransac_Report oReport, Mem_Mgr* poMem_Mgr)
{
	if (oReport.m_pInlier_Mask)
	{
		if (poMem_Mgr)
			Free(poMem_Mgr, oReport.m_pInlier_Mask);
		else
			free(oReport.m_pInlier_Mask);
	}
	return;
}

void Disp_Report(Ransac_Report oReport)
{
	printf("%s\n", oReport.m_bSuccess ? "Success" : "Fail");
	printf("Sample Count:%d Trial_Count:%d\n", oReport.m_iSample_Count, oReport.m_iTrial_Count);
	printf("Inlier Count:%d Residual:%f\n", oReport.m_oSupport.m_iInlier_Count, oReport.m_oSupport.m_fResidual_Sum);
	if (oReport.m_iFloat_Size == 4)
		Disp(oReport.m_Modal_f, 3, 3, "Modal");
	else
		Disp(oReport.m_Modal_d, 3, 3, "Modal");
	return;
}
template<typename _T>_T ComputeOppositeOfMinor(_T matrix[], const size_t row, const size_t col)
{
	const size_t col1 = col == 0 ? 1 : 0;
	const size_t col2 = col == 2 ? 1 : 2;
	const size_t row1 = row == 0 ? 1 : 0;
	const size_t row2 = row == 2 ? 1 : 2;
	return (matrix[row1*3 + col2] * matrix[row2 * 3 + col1] -
		matrix[row1 * 3 + col1] * matrix[row2 * 3 + col2]);
}
#define SignOfNumber(fValue) (int)((0.f < fValue) - (fValue < 0.f))

template<typename _T>void ComputeHomographyRotation(_T H_normalized[3 * 3], _T tstar[3], _T n[3], _T v,_T R[])
{
	_T Temp[3 * 3];
	Matrix_Multiply(tstar, 3, 1, n, 3, Temp);
	Matrix_Multiply(Temp, 3, 3, (_T)- 2.f / v, Temp);
	Add_I_Matrix(Temp, 3);
	Matrix_Multiply(H_normalized,3,3, Temp,3, R);
	//Disp(R, 3, 3, "R");
}
template<typename _T>void Decompose_H(_T H[3 * 3], _T R1[3 * 3], _T R2[3 * 3], _T t1[3], _T t2[3],_T K1[],_T K2[])
{//对H矩阵分解为R,t
	
	_T H_normalized[3 * 3];
	_T S[3 * 3];
	_T Temp[3 * 3];
	int i,iResult;

	//Temp = K2(-1) H K1
	Get_Inv_Matrix_Row_Op_2(K2?K2:K1, Temp, 3, &iResult);
	Matrix_Multiply(Temp, 3, 3, H, 3, Temp );
	Matrix_Multiply(Temp, 3, 3, K1, 3, H_normalized);

	//Disp(Temp, 3, 3);
	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(H_normalized, oSVD, &oSVD.m_bSuccess);
	Matrix_Multiply(H_normalized, 3, 3, 1.f/((_T*)oSVD.S)[1], H_normalized);
	Free_SVD(&oSVD);

	//Disp(H_normalized, 3, 3);	
	if (fGet_Determinant(H_normalized, 3) < 0)
		Matrix_Multiply(H_normalized, 3, 3, (_T)-1.f, H_normalized);
	//printf("%f", fGet_Determinant(H_normalized, 3));
	Transpose_Multiply(H_normalized, 3, 3, S,0);
	Add_I_Matrix(S, 3,(_T)-1.f);

	_T kMinInfinityNorm = 1e-3f;
	_T fMax = 0;
	//找出整个AtA矩阵的绝对值最大值
	for (i = 0; i < 3 * 3; i++)
		if (abs(S[i]) > fMax)
			fMax = abs(S[i]);
	if (fMax < kMinInfinityNorm)
	{
		memcpy(R1, H_normalized, 3 * 3 * sizeof(_T));
		memcpy(R2, H_normalized, 3 * 3 * sizeof(_T));
		memset(t1, 0, 3 * sizeof(_T));
		memset(t2, 0, 3 * sizeof(_T));
		return;
	}

	_T M00 = ComputeOppositeOfMinor(S, 0, 0);
	_T M11 = ComputeOppositeOfMinor(S, 1, 1);
	_T M22 = ComputeOppositeOfMinor(S, 2, 2);

	_T rtM00 = (_T)sqrt(M00);
	_T rtM11 = (_T)sqrt(M11);
	_T rtM22 = (_T)sqrt(M22);

	_T M01 = ComputeOppositeOfMinor(S, 0, 1);
	_T M12 = ComputeOppositeOfMinor(S, 1, 2);
	_T M02 = ComputeOppositeOfMinor(S, 0, 2);

	int e12 = SignOfNumber(M12);
	int e02 = SignOfNumber(M02);
	int e01 = SignOfNumber(M01);
	
	_T nS00 = Abs(S[0 * 3 + 0]);
	_T nS11 = Abs(S[1 * 3 + 1]);
	_T nS22 = Abs(S[2 * 3 + 2]);

	_T nS[] = { nS00, nS11, nS22 };
	int idx;
	fMax = 0;
	for (i = 0; i < 3; i++)
		if (nS[i] > fMax)
			fMax = nS[i], idx = i;
	_T np1[3], np2[3];
	if (idx == 0)
	{
		np1[0] = S[0*3+ 0];
		np2[0] = S[0 * 3 + 0];
		np1[1] = S[0 * 3 + 1] + rtM22;
		np2[1] = S[0 * 3 + 1] - rtM22;
		np1[2] = S[0 * 3 + 2] + e12 * rtM11;
		np2[2] = S[0 * 3 + 2] - e12 * rtM11;
	}
	else if (idx == 1)
	{
		np1[0] = S[0 * 3 + 1] + rtM22;
		np2[0] = S[0 * 3 + 1] - rtM22;
		np1[1] = S[1 * 3 + 1];
		np2[1] = S[1 * 3 + 1];
		np1[2] = S[1 * 3 + 2] - e02 * rtM00;
		np2[2] = S[1 * 3 + 2] + e02 * rtM00;
	}
	else if (idx == 2)
	{
		np1[0] = S[0 * 3 + 2] + e01 * rtM11;
		np2[0] = S[0 * 3 + 2] - e01 * rtM11;
		np1[1] = S[1 * 3 + 2] + rtM00;
		np2[1] = S[1 * 3 + 2] - rtM00;
		np1[2] = S[2 * 3 + 2];
		np2[2] = S[2 * 3 + 2];
	}
	_T traceS = fGet_Tr(S,3);
	_T v = 2.f * (_T)sqrt(1.f + traceS - M00 - M11 - M22);
	int ESii = SignOfNumber(S[idx*3+ idx]);
	_T r_2 = 2 + traceS + v;
	_T nt_2 = 2 + traceS - v;

	_T r = (_T)sqrt(r_2);
	_T n_t = (_T)sqrt(nt_2);

	_T* n1 = np1, * n2 = np2;
	Normalize(n1,3,n1); Normalize(n2, 3, n2);

	_T half_nt = 0.5f * n_t;
	_T esii_t_r = ESii * r;
	_T fValue_1, fValue_2;
	fValue_1 = half_nt * esii_t_r;
	fValue_2 = half_nt * n_t;
	_T t1_star[3], t2_star[3];
	Matrix_Multiply(n2, 1, 3, fValue_1, t1_star);
	Matrix_Multiply(n1, 1, 3, fValue_2, Temp);
	Vector_Minus(t1_star, Temp, 3, t1_star);

	Matrix_Multiply(n1, 1, 3, fValue_1, t2_star);
	Matrix_Multiply(n2, 1, 3, fValue_2, Temp);
	Vector_Minus(t2_star, Temp, 3, t2_star);

	//Disp(t1_star, 1,3,"t1");
	//Disp(t2_star, 1, 3, "t2");

	//ComputeHomographyRotation(H_normalized, t1_star, n1, v);
	//H_normalized* (Eigen::Matrix3d::Identity() - (2.0 / v) * tstar * n.transpose());
	//_T R1[3 * 3], R2[3 * 3],t1[3],t2[3];
	ComputeHomographyRotation(H_normalized, t1_star, n1, v, R1);
	//t1 = R1 * t1_star;
	Matrix_Multiply(R1, 3, 3, t1_star, 1, t1);
	//ComputeHomographyRotation(H_normalized, t2_star, n2, v);
	ComputeHomographyRotation(H_normalized, t2_star, n2, v, R2);
	//t2 = R2 * t2_star;
	Matrix_Multiply(R2, 3, 3, t2_star, 1, t2);

	return;
}
template<typename _T>void Decompose_E(_T E[3 * 3], _T R1[3 * 3], _T R2[3 * 3], _T t1[3], _T t2[3], int bNormalize_t)
{//从一个E矩阵中分解出两个R 和两个t
//注意，这里的分解不是完全按照定义来 E = t^ * R , 而是将t归一化了
	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(E, oSVD, &oSVD.m_bSuccess);
	if (fGet_Determinant((_T*)oSVD.U, 3) < 0)
		Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T)(-1), (_T*)oSVD.U);

	if (fGet_Determinant((_T*)oSVD.Vt, 3) < 0)
		Matrix_Multiply((_T*)oSVD.Vt, 3, 3,(_T)(- 1), (_T*)oSVD.Vt);

	//算没问题，但是尚未详细推导，这个分解是怎样实现的，如何和前面的(s0,s0,0)分解结合起来
	//以下为Col_Map代码
	_T W[3 * 3] = { 0, 1, 0, -1, 0, 0, 0, 0, 1 };

	//R_1 = U * W * V
	Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T*)W, 3, R1);
	Matrix_Multiply(R1, 3, 3, (_T*)oSVD.Vt, 3, R1);

	Matrix_Transpose(W, 3, 3, W);
	Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T*)W, 3, R2);
	Matrix_Multiply(R2, 3, 3, (_T*)oSVD.Vt, 3, R2);

	t1[0] = ((_T*)oSVD.U)[2], t1[1] = ((_T*)oSVD.U)[5], t1[2] = ((_T*)oSVD.U)[8];
	Normalize(t1, 3, t1);
	Matrix_Multiply(t1, 1, 3, (_T)(-1.f), t2);
	if (!bNormalize_t)
	{
		Matrix_Multiply(t1, 1, 3, (_T)((_T*)oSVD.S)[0], t1);
		Matrix_Multiply(t2, 1, 3, (_T)((_T*)oSVD.S)[0], t2);
	}
	Free_SVD(&oSVD);

	//验算一下是否 E= R_1 * t^, 失败，存疑
	//Test_Decompose_E(E, R_2, t);

	//Disp(R_1, 3, 3, "R_1");
	//Disp(R_2, 3, 3, "R_2");

	////尝试用理论上的方法
	//float R_z_t_1[3 * 3],		//Rz(pi/2)		后续做成常量
	//    R_z_t_2[3 * 3];		    //Rz(-pi/2)
	//float V[] = { 0,0,1,PI / 2.f };
	//Rotation_Vector_2_Matrix_3D(V, R_z_t_1);
	//Disp(R_z_t_1, 3, 3);
	////可以看出， R_z_t_1={  0, -1, 0,
	////                      1, 0, 0,
	////                      0, 0,0 }

	//V[3] = -PI / 2.f;
	//Rotation_Vector_2_Matrix_3D(V, R_z_t_2);
	//Disp(R_z_t_2, 3, 3);
	////而 R_z_t_2={   0, 1, 0,
	////              -1, 0, 0,
	////               0, 0, 0 }
	////可见，两个转换结果一样

	//Disp(t, 1, 3, "t");
}
template<typename _T>void Get_J_Inv(_T eij[6], _T J_Inv[6 * 6])
{//求 J(-1)(eij)，用于位姿图估计
	//= I + 1/2 *   phi^    rho^
	//              0       phi^
	_T hat[3 * 3];
	Hat(eij, hat);  //得rho^
	memset(J_Inv, 0, 36 * sizeof(_T));
	//Disp(hat, 3, 3, "hat");
	J_Inv[3] = hat[0], J_Inv[4] = hat[1], J_Inv[5] = hat[2];
	J_Inv[9] = hat[3], J_Inv[10] = hat[4], J_Inv[11] = hat[5];
	J_Inv[15] = hat[6], J_Inv[16] = hat[7], J_Inv[17] = hat[8];

	Hat(&eij[3], hat);  //得phi^
	J_Inv[0] = J_Inv[21] = hat[0], J_Inv[1] = J_Inv[22] = hat[1], J_Inv[2] = J_Inv[23] = hat[2];
	J_Inv[6] = J_Inv[27] = hat[3], J_Inv[7] = J_Inv[28] = hat[4], J_Inv[8] = J_Inv[29] = hat[5];
	J_Inv[12] = J_Inv[33] = hat[6], J_Inv[13] = J_Inv[34] = hat[7], J_Inv[14] = J_Inv[35] = hat[8];
	//Disp(J_Inv, 6, 6, "J_Inv");
	Matrix_Multiply(J_Inv, 6, 6, (_T)0.5f, J_Inv);
	Add_I_Matrix(J_Inv, 6);
	//Disp(J_Inv, 6, 6, "J_Inv");
	return;
}
template<typename _T>void Get_Adj(_T Rt[4 * 4], _T Adj[6 * 6])
{//主要利用伴随性质，对给定的齐次矩阵Rt生成伴随矩阵 R  t^R
//                                                  0   R
	_T R[3 * 3],
		t_R[3 * 3],  //= t^R
		t[3];
	memset(Adj, 0, 36 * sizeof(_T));
	Get_R_t(Rt, R, t);
	Adj[0] = Adj[21] = R[0], Adj[1] = Adj[22] = R[1], Adj[2] = Adj[23] = R[2];
	Adj[6] = Adj[27] = R[3], Adj[7] = Adj[28] = R[4], Adj[8] = Adj[29] = R[6];
	Adj[12] = Adj[33] = R[6], Adj[13] = Adj[34] = R[7], Adj[14] = Adj[35] = R[8];

	Hat(t, t_R);
	Matrix_Multiply(t_R, 3, 3, R, 3, t_R);

	Adj[3] = t_R[0], Adj[4] = t_R[1], Adj[5] = t_R[2];
	Adj[9] = t_R[3], Adj[10] = t_R[4], Adj[11] = t_R[5];
	Adj[15] = t_R[6], Adj[16] = t_R[7], Adj[17] = t_R[8];
	//Disp(Adj, 6, 6, "Adj");
	return;
}

//应该搞一组损失函数，专门计算估计出来的参数的误差
template<typename _T>_T Test_T(_T Point_3D_0[][3], _T Point_3D_1[][3], _T T[], int iCount)
{//算一个变换的误差
	int i;
	_T R[3 * 3], t[3], fTotal;
	Get_R_t(T, R, t);

	for (fTotal = 0, i = 0; i < iCount; i++)
	{
		_T Temp[3];
		Matrix_Multiply(R, 3, 3, Point_3D_0[i], 1, Temp);
		Vector_Add(Temp, t, 3, Temp);
		_T fDist = fGet_Distance(Temp, Point_3D_1[i], 3);
		fTotal += fDist;
		//printf("Expected:%f %f %f   Actural:%f %f %f    Error:%f\n", Point_3D_1[i][0], Point_3D_1[i][1], Point_3D_1[i][2], Temp[0], Temp[1], Temp[2], fDist);
	}
	return fTotal / iCount;
}


void SB_Reconstruct()
{//这就是个傻逼方法，用来欺骗template
	bSave_PLY(NULL, Point_Cloud < double>{});
	bSave_PLY(NULL, Point_Cloud < float>{});

	Init_Point_Cloud( (Point_Cloud<double>*)NULL, 0, 0);
	Init_Point_Cloud((Point_Cloud<float>*)NULL, 0, 0);

	Free_Point_Cloud((Point_Cloud<double>*)NULL);
	Free_Point_Cloud((Point_Cloud<float>*)NULL);

	Draw_Sphere((Point_Cloud<double>*)NULL, (double)0, (double)0, (double)0);
	Draw_Sphere((Point_Cloud<float>*)NULL, (float)0, (float)0, (float)0);

	Draw_Line((Point_Cloud<double>*)NULL, (double)0, (double)0, (double)0, (double)0, (double)0, (double)0);
	Draw_Line((Point_Cloud<float>*)NULL, (float)0, (float)0, (float)0, (float)0, (float)0, (float)0);
	
	Draw_Camera((Point_Cloud<double>*)NULL, (double*)NULL);
	Draw_Camera((Point_Cloud<float>*)NULL, (float*)NULL);
	
	Draw_Image((Point_Cloud<double>*)NULL, Image{}, (double*)NULL, (double*)NULL, 0);
	Draw_Image((Point_Cloud<float>*)NULL, Image{}, (float*)NULL, (float*)NULL, 0);

	Get_Deriv_E_P((double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL);
	Get_Deriv_E_P((float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL);

	Bundle_Adjust((double*)NULL, 0, (double*)NULL, 0, 0, (double*)NULL, 3,(double)0.f, 0,(void(*)(double*, double*, double*, double*, double*,...))NULL);
	Bundle_Adjust((float*)NULL, 0, (float*)NULL, 0, 0, (float*)NULL, 3, (float)0.f, 0,(void(*)(float*, float*, float*, float*, float*,...))NULL);

	Init_LM_Param( (LM_Param_g2o<double>*) NULL);
	Init_LM_Param( (LM_Param_g2o<float>*) NULL);

	Solve_Linear_LM((double*)NULL, 0, (double*)NULL, (double*)NULL, NULL, (LM_Param_g2o<double>*) NULL);
	Solve_Linear_LM((float*)NULL, 0, (float*)NULL, (float*)NULL, NULL, (LM_Param_g2o<float>*) NULL);;

	Check_Cheirality((double(*)[2])NULL, (double(*)[2])NULL, (int*)NULL, (double*)NULL, (double*)NULL, (double(*)[3])NULL);
	Check_Cheirality((float(*)[2])NULL, (float(*)[2])NULL, (int*)NULL, (float*)NULL, (float*)NULL, (float(*)[3])NULL);

	Disp_T((double*)NULL);
	Disp_T((float*)NULL);

	Test_Triangulate((double(*)[3])NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0, (double*)NULL, (double*)NULL);
	Test_Triangulate((float(*)[3])NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0, (float*)NULL, (float*)NULL);

	Match_2_Image((const char*)NULL, (const char*)NULL, (const char*)NULL, (const char*)NULL, (double)0.f, 
		(double*)NULL, NULL, (double(**)[2])NULL, (double(**)[2])NULL,
		(double(**)[2])NULL, (double(**)[2])NULL,	(double(**)[3])NULL, (double(**)[3])NULL,(Image*)NULL, (Image*)NULL, (Image*)NULL);
	Match_2_Image((const char*)NULL, (const char*)NULL, (const char*)NULL, (const char*)NULL, (float)0.f,
		(float*)NULL, NULL, (float(**)[2])NULL, (float(**)[2])NULL,
		(float(**)[2])NULL, (float(**)[2])NULL, (float(**)[3])NULL, (float(**)[3])NULL, (Image*)NULL, (Image*)NULL, (Image*)NULL);


	Estimate_2_Image_T(NULL, NULL, (double*)NULL, (Image_Match_Param<double>*)NULL);
	Estimate_2_Image_T(NULL, NULL, (float*)NULL, (Image_Match_Param<float>*)NULL);

	Free_2_Image_Match((Image_Match_Param<double>*)NULL);
	Free_2_Image_Match((Image_Match_Param<float>*)NULL);

	fGet_Error((double*)NULL, 0, 0, (double*)NULL);
	fGet_Error((float*)NULL, 0, 0, (float*)NULL);

	DLT((double(*)[3])NULL, (double(*)[3])NULL, 0, (double*)NULL);
	DLT((float(*)[3])NULL, (float(*)[3])NULL, 0, (float*)NULL);

	DLT_svd((double(*)[3])NULL, (double(*)[3])NULL, (double(*)[2])NULL, 0, (double*)NULL);
	DLT_svd((float(*)[3])NULL, (float(*)[3])NULL, (float(*)[2])NULL, 0, (float*)NULL);

	Test_T((double(*)[3])NULL, (double(*)[3])NULL,(double*)NULL,0);
	Test_T((float(*)[3])NULL, (float(*)[3])NULL, (float*)NULL, 0);

	Test_T((double(*)[3])NULL, (double(*)[2])NULL,(double*)NULL,0);
	Test_T((float(*)[3])NULL, (float(*)[2])NULL, (float*)NULL, 0);

	Match_2_Image((const char*)NULL, (const char*)NULL, (const char*)NULL, (const char*)NULL, (double)0.f, (double*)NULL, NULL, (double(**)[2])NULL, (double(**)[2])NULL, (double(**)[2])NULL, (double(**)[2])NULL, (double(**)[3])NULL, (double(**)[3])NULL, NULL,NULL,NULL);
	Match_2_Image((const char*)NULL, (const char*)NULL, (const char*)NULL, (const char*)NULL, (float)0.f, (float*)NULL, NULL, (float(**)[2])NULL, (float(**)[2])NULL, (float(**)[2])NULL, (float(**)[2])NULL, (float(**)[3])NULL, (float(**)[3])NULL, NULL,NULL,NULL);

	Gen_Pose((double*)NULL, (double)0.f, (double)0.f, (double)0.f, (double)0.f, (double)0.f, (double)0.f, (double)0.f);
	Gen_Pose((float*)NULL, (float)0.f, (float)0.f, (float)0.f, (float)0.f, (float)0.f, (float)0.f, (float)0.f);

	Get_K_Inv((double*)NULL, (double*)NULL);
	Get_K_Inv((float*)NULL, (float*)NULL);

	Get_K_Inv_With_gama((double*)NULL, (double*)NULL);
	Get_K_Inv_With_gama((float*)NULL, (float*)NULL);

	Gen_Plane((double(**)[3])NULL, NULL);
	Gen_Plane((float(**)[3])NULL, NULL);

	Gen_Plane_z0((double(**)[3])NULL, NULL);
	Gen_Plane_z0((float(**)[3])NULL, NULL);

	Gen_Sphere((double(**)[3])NULL, NULL);
	Gen_Sphere((float(**)[3])NULL, NULL);

	Gen_Cube((double(**)[3])NULL, NULL,1.f);
	Gen_Cube((float(**)[3])NULL, NULL, 1.f);

	Gen_Cube((double(*)[4])NULL, 0);
	Gen_Cube((float(*)[4])NULL, 0);

	Reset_Pose_Graph(Pose_Graph_Sigma_H<double>{});
	Reset_Pose_Graph(Pose_Graph_Sigma_H<float>{});

	Copy_Data_2_Sparse(Pose_Graph_Sigma_H<double>{}, (Sparse_Matrix<double>*)NULL);
	Copy_Data_2_Sparse(Pose_Graph_Sigma_H<float>{}, (Sparse_Matrix<float>*)NULL);

	Init_Pose_Graph((Measurement<double>*)NULL, 0, 0, (Pose_Graph_Sigma_H<double>*)NULL );
	Init_Pose_Graph((Measurement<float>*)NULL, 0, 0, (Pose_Graph_Sigma_H<float>*)NULL);

	Get_J_Inv((double*)NULL, (double*)NULL);
	Get_J_Inv((float*)NULL, (float*)NULL);

	Undistort_uv((double*)NULL, (double*)NULL, (double*)NULL, 0, (double*)NULL);
	Undistort_uv((float*)NULL, (float*)NULL, (float*)NULL, 0, (float*)NULL);

	TQ_2_Rt((double*)NULL, (double*)NULL);
	TQ_2_Rt((float*)NULL, (float*)NULL);

	Get_Adj((double*)NULL, (double*)NULL);
	Get_Adj((float*)NULL, (float*)NULL);

	Distribute_Data(Pose_Graph_Sigma_H<double>{}, (double*)NULL, 0, 0);
	Distribute_Data(Pose_Graph_Sigma_H<float>{}, (float*)NULL, 0, 0);

	Solve_Linear_Schur(Schur_Camera_Data<double>{}, (double*)NULL, (double*)NULL);
	Solve_Linear_Schur(Schur_Camera_Data<float>{}, (float*)NULL, (float*)NULL);

	Free((Schur_Camera_Data<double>*)NULL);
	Free((Schur_Camera_Data<float>*)NULL);

	Copy_Data_2_Sparse(Schur_Camera_Data<double>{}, (Sparse_Matrix<double>*)NULL);
	Copy_Data_2_Sparse(Schur_Camera_Data<float>{}, (Sparse_Matrix<float>*)NULL);

	Init_All_Camera_Data((Schur_Camera_Data<double>*)NULL, NULL, 0, 0);
	Init_All_Camera_Data((Schur_Camera_Data<float>*)NULL, NULL, 0, 0);

	Distribute_Data(Schur_Camera_Data<double>{}, (double*)NULL, 0, 0);
	Distribute_Data(Schur_Camera_Data<float>{}, (float*)NULL, 0, 0);

	Get_Drive_UV_P((double*)NULL, (double*)NULL, (double*)NULL);
	Get_Drive_UV_P((float*)NULL, (float*)NULL, (float*)NULL);

	Get_Drive_UV_P((double)0.f, (double)0.f, (double*)NULL,(double*)NULL);
	Get_Drive_UV_P((float)0.f, (float)0.f, (float*)NULL, (float*)NULL);

	Get_Delta_Pose((double*)NULL, (double*)NULL, (double*)NULL);
	Get_Delta_Pose((float*)NULL, (float*)NULL, (float*)NULL);

	Disp_Error((double(*)[3])NULL, (double(*)[3])NULL, 0, (double*)NULL);
	Disp_Error((float(*)[3])NULL, (float(*)[3])NULL, 0, (float*)NULL);

	Optical_Flow_1({}, {}, (double(*)[2])NULL, (double(*)[2])NULL, 0, NULL);
	Optical_Flow_1({}, {}, (float(*)[2])NULL, (float(*)[2])NULL, 0, NULL);

	ICP_SVD((double(*)[3])NULL, (double(*)[3])NULL, 0, (double*)NULL, NULL);
	ICP_SVD((float(*)[3])NULL, (float(*)[3])NULL, 0, (float*)NULL, NULL);

	ICP_BA_2_Image_1((double(*)[3])NULL, (double(*)[3])NULL, 0, (double*)NULL, NULL);
	ICP_BA_2_Image_1((float(*)[3])NULL, (float(*)[3])NULL, 0, (float*)NULL, NULL);

	ICP_BA_2_Image_2((double(*)[3])NULL, (double(*)[3])NULL, 0, (double*)NULL, NULL);
	ICP_BA_2_Image_2((float(*)[3])NULL, (float(*)[3])NULL, 0, (float*)NULL, NULL);

	Get_Deriv_TP_Ksi_1((double*)NULL, (double*)NULL, (double*)NULL);
	Get_Deriv_TP_Ksi_1((float*)NULL, (float*)NULL, (float*)NULL);

	Get_Deriv_TP_Ksi((double*)NULL, (double*)NULL, (double*)NULL);
	Get_Deriv_TP_Ksi((float*)NULL, (float*)NULL, (float*)NULL);

	Get_Deriv_E_Ksi((double*)NULL, (double*)NULL, (double*)NULL);
	Get_Deriv_E_Ksi((float*)NULL, (float*)NULL, (float*)NULL);

	Bundle_Adjust_3D2D_1((double(*)[3])NULL, (double(*)[2])NULL, 0, (double*)NULL, (double*)NULL, NULL);
	Bundle_Adjust_3D2D_1((float(*)[3])NULL, (float(*)[2])NULL, 0, (float*)NULL, (float*)NULL, NULL);

	Image_Pos_2_Norm((double(*)[2])NULL, 0, (double*)NULL, (double(*)[2])NULL);
	Image_Pos_2_Norm((float(*)[2])NULL, 0, (float*)NULL, (float(*)[2])NULL);

	Image_Pos_2_3D((double(*)[2])NULL,NULL,0, 0, (double*)NULL, (double)0, (double(*)[3])NULL);
	Image_Pos_2_3D((float(*)[2])NULL, NULL,0,0, (float*)NULL, (float)0, (float(*)[3])NULL);

	Image_Pos_2_3D((double(*)[3])NULL, 0, (double*)NULL,(double)0, (double(*)[3])NULL);
	Image_Pos_2_3D((float(*)[3])NULL, 0, (float*)NULL, (float)0, (float(*)[3])NULL);

	RGBD_2_Point_3D({}, NULL, (double(*)[3])NULL, (double)0, (double(*)[3])NULL, NULL);
	RGBD_2_Point_3D({}, NULL, (float(*)[3])NULL, (float)0, (float(*)[3])NULL, NULL);

	Determine_Confg(NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0, (double(**)[2])NULL, (double(**)[2])NULL);
	Determine_Confg(NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0, (float(**)[2])NULL, (float(**)[2])NULL);

	Estimate_Relative_Pose({}, NULL, NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0, NULL);
	Estimate_Relative_Pose({}, NULL, NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0, NULL);

	Gen_Camera_Intrinsic((double*)NULL, 0, 0, 0, 0, 0);
	Gen_Camera_Intrinsic((float*)NULL, 0, 0, 0, 0, 0);

	Test_E((double*)NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0);
	Test_E((float*)NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0);

	Triangulate_Point((double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL);
	Triangulate_Point((float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL);

	Compute_Squared_Sampson_Error((double(*)[2])NULL, (double(*)[2])NULL, 0, (double*)NULL, (double*)NULL);
	Compute_Squared_Sampson_Error((float(*)[2])NULL, (float(*)[2])NULL, 0, (float*)NULL, (float*)NULL);

	Ransac_Estimate_H((double(*)[2])NULL, (double(*)[2])NULL, 0, NULL);
	Ransac_Estimate_H((float(*)[2])NULL, (float(*)[2])NULL, 0, NULL);

	Ransac_Estimate_E((double(*)[2])NULL, (double(*)[2])NULL, 0, 0, 0, 0, NULL);
	Ransac_Estimate_E((float(*)[2])NULL, (float(*)[2])NULL, 0, 0, 0, 0, NULL);

	Ransac_Estimate_F((double(*)[2])NULL, (double(*)[2])NULL, 0, NULL);
	Ransac_Estimate_F((float(*)[2])NULL, (float(*)[2])NULL, 0, NULL);

	Normalize_Point((double(*)[2])NULL, (double(*)[2])NULL, 0, (double(*)[2])NULL, (double(*)[2])NULL, 0, 0, 0);
	Normalize_Point((float(*)[2])NULL, (float(*)[2])NULL, 0, (float(*)[2])NULL, (float(*)[2])NULL, 0, 0, 0);

	Decompose_E((double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL);
	Decompose_E((float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL);

	Decompose_H((double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL);
	Decompose_H((float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL);

	E_2_R_t((double*)NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0,(double*)NULL,(double*)NULL, (double(*)[3])NULL);
	E_2_R_t((float*)NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0, (float*)NULL, (float*)NULL, (float(*)[3])NULL);

	E_2_R_t_Pixel_Pos((double*)NULL, (double(*)[2])NULL, (double(*)[2])NULL,(double*)NULL, 0, (double*)NULL, (double*)NULL, (double(*)[3])NULL);
	E_2_R_t_Pixel_Pos((float*)NULL, (float(*)[2])NULL, (float(*)[2])NULL, (float*)NULL, 0, (float*)NULL, (float*)NULL, (float(*)[3])NULL);

}
template<typename _T>void Triangulate_Point_1(_T x1[2],  _T R1[], _T t1[], _T x2[2], _T R2[], _T t2[], _T Point_3D[])
{//此处通过纯数学推导解z2x2 = R2 * R1(-1)* (z1x1 -t1)
	//union {
	//	_T R1_Inv[3 * 3];
	//	_T Temp[9];
	//	_T A[3 * 2];
	//};
	//Matrix_Transpose(R1,3, 3, R1_Inv);
	////算R2 * R1(-1)
	//Matrix_Multiply(R2, 3, 3, R1_Inv, 3, Temp);

	//_T V1[3]={x1[0],x1[1],1.f}, V2[3], X[3];
	//int iResult;
	//Matrix_Multiply(Temp, 3, 3, V1, 1, V1);
	//Matrix_Multiply(Temp, 3, 3, t1, 1, V2);
	//Vector_Minus(V2, t2,3,V2);
	//A[0] = V1[0], A[1] = -x2[0];
	//A[2] = V1[1], A[3] = -x2[1];
	//A[4] = V1[2], A[5] = -1.f;

	//Solve_Linear_Contradictory(A, 3, 2, V2, X, &iResult);
	//
	///*Point_3D[0] = X[0] * x1[0];
	//Point_3D[1] = X[0] * x1[1];
	//Point_3D[2] = X[0] * 1.f;*/
	//Vector_Minus(R2, R1, 3 * 3, Temp);
	//V1[0] = X[0] * x1[0], V1[1] = X[0] * x1[1], V1[2] = X[0]*1.f;
	//V2[0] = X[1] * x2[0], V2[1] = X[1] * x2[1], V2[2] = X[1]*1.f;
	//
	//Disp(V1, 1, 3, "x1 to Point");
	//Disp(V2, 1, 3, "x2 to Point");
	//Point_3D[0] = (V1[0] + V2[0]) / 2.f;
	//Point_3D[1] = (V1[1] + V2[1]) / 2.f;
	//Point_3D[2] = (V1[2] + V2[2]) / 2.f;

	////连z1,z2,P0,P1,P2一起算
	//_T A[6 * 5] = { 0 }, B[6], X[5];
	//int x, y,iResult;
	////第一列，x1为参数
	//A[0 * 5] = x1[0], A[1 * 5] = x1[1], A[2 * 5] = 1.f;
	//A[3 * 5 + 1] = x2[0], A[4 * 5 + 1] = x2[1], A[5 * 5 + 1] = 1.f;
	//for (y = 0; y < 3; y++)
	//{
	//	int iPos_A_1 = y * 5 + 2, iPos_A_2 = (y + 3) * 5 + 2,
	//		iPos_R = y * 3;
	//		for (x = 0; x < 3; x++, iPos_R++, iPos_A_1++, iPos_A_2++)
	//			A[iPos_A_1] = -R1[iPos_R], A[iPos_A_2] = -R2[iPos_R];
	//	B[y] = t1[y];
	//	B[y + 3] = t2[y];
	//}
	///*Disp(R1, 3, 3, "R1");
	//Disp(R2, 3, 3, "R2");
	//Disp(t1, 1, 3, "t1");
	//Disp(t2, 1, 3, "t2");
	//Disp(A, 6, 5, "A");
	//Disp(B, 1, 6, "B");*/
	//Solve_Linear_Contradictory(A, 6, 5, B, X, &iResult);
	//memcpy(Point_3D, &X[2], 3 * sizeof(_T));

	_T A[4 * 3],B[4],X[3];
	int iResult;
	//第一行
	A[0] = R1[6] * x1[0] - R1[0];
	A[1] = R1[7] * x1[0] - R1[1];
	A[2] = R1[8] * x1[0] - R1[2];
	B[0] = t1[0] - x1[0] * t1[2];

	//第二行
	A[3] = R1[6] * x1[1] - R1[3];
	A[4] = R1[7] * x1[1] - R1[4];
	A[5] = R1[8] * x1[1] - R1[5];
	B[1] = t1[1] - x1[1] * t1[2];

	//第三行
	A[6] = R2[6] * x2[0] - R2[0];
	A[7] = R2[7] * x2[0] - R2[1];
	A[8] = R2[8] * x2[0] - R2[2];
	B[2] = t2[0] - x2[0] * t2[2];

	//第四行
	A[9] = R2[6] * x2[1] - R2[3];
	A[10] = R2[7] * x2[1] - R2[4];
	A[11] = R2[8] * x2[1] - R2[5];
	B[3] = t2[1] - x2[1] * t2[2];

	Solve_Linear_Contradictory(A, 4, 3, B, X, &iResult);
	memcpy(Point_3D, X, 3 * sizeof(_T));
	return;
}
template<typename _T>void Triangulate_Point(_T x0[2], _T x1[2], _T KP_0[], _T KP_1[], _T Point_3D[])
{//这个方程有异议，对于 x = PX中， 如果X为空间点，那么P应为相机参数，最后投影到归一化平面上
	//注意，此处的相机参数KP可以是外参（平行投影），可以是内参矩阵乘以外参矩阵。如果有内参
	//x0 = (1/z) * KP * P	z完全不影响恢复

	int x;
	_T A[4 * 4] = {};    //这个是系数矩阵，最后我们求一个最小二乘问题， 求Ax=0的最小二乘解
	////第一行
	//A[0] = -1, A[2] = Point_1[0];
	////第二行
	//A[1*4+ 1] = -1, A[1*4+2] = Point_1[1];
	////第三行
	//for (x = 0; x < 4; x++)
	//    A[2 * 4 + x] = Point_2[0] * P_2[2 * 4 + x] - P_2[0 * 4 + x];
	////第4行
	//for (x = 0; x < 4; x++)
	//    A[3 * 4 + x] = Point_2[1] * P_2[2 * 4 + x] - P_2[1 * 4 + x];

	//Disp(A, 4, 4, "A");
	//按理论自己搞一下A，实践证明，部分符号取反之外，差别不大，解一样
	for (x = 0; x < 4; x++)
	{
		A[0 * 4 + x] = x0[1] * KP_0[2 * 4 + x] - KP_0[1 * 4 + x];
		A[1 * 4 + x] = KP_0[0 * 4 + x] - x0[0] * KP_0[2 * 4 + x];

		A[2 * 4 + x] = x1[1] * KP_1[2 * 4 + x] - KP_1[1 * 4 + x];
		A[3 * 4 + x] = KP_1[0 * 4 + x] - x1[0] * KP_1[2 * 4 + x];
	}
	//Disp(A, 4, 4, "A");

   /* Disp(A, 4, 4, "A");
	Disp(Point_1, 1, 2,"Point_1");
	Disp(Point_2, 1, 2, "Point_2");
	Disp(P_1, 4, 4, "P1");
	Disp(P_2, 4, 4, "P2");*/

	//然后求解 Ax的最小二乘解
	SVD_Info oSVD;
	SVD_Alloc<_T>(4, 4, &oSVD);
	svd_3(A, oSVD, &oSVD.m_bSuccess);
	//Disp(&((_T*)oSVD.Vt)[3 * 4], 1, 4,"Vt");
	//Test_SVD(A,oSVD);
	Homo_Normalize(&((_T*)oSVD.Vt)[3 * 4], 4, Point_3D);
	Point_3D[3] = 1.f;
	//Disp(New_Point, 1, 4, "New_Point");
	Free_SVD(&oSVD);
	return;
}
template<typename _T>_T fTriangulate_Error(_T Point_1[], _T Point_2[], _T T1[], _T T2[], _T New_Point[])
{
	_T Temp[4];
	int i;
	_T fTotal = 0;
	Matrix_Multiply(T1,4,4, New_Point,1, Temp);
	for (i = 0; i < 2; i++)
		Temp[i] /= Temp[2];
	fTotal += fGet_Distance(Point_1, Temp, 2);

	Matrix_Multiply(T2, 4, 4, New_Point, 1, Temp);
	for (i = 0; i < 2; i++)
		Temp[i] /= Temp[2];
	fTotal += fGet_Distance(Point_2, Temp, 2);

	return fTotal;
}
template<typename _T>void Check_Cheirality(_T Point_1[][2], _T Point_2[][2], int* piCount, _T R[], _T t[], _T Point_3d[][3],unsigned char * pInlier_Mask)
{//对给定的R,t检验是否符合现实
//注意，此处是一种偷懒算法，没有把相机内参带进来，因为其初衷仅仅是验证几个 R t组合中哪个能满足 z>0 
//故此恢复出来的Poin_3D根本就不是原来的点坐标，而是省略相机内参情况下的空间点坐标

	_T T1[4 * 4], T2[4 * 4], Temp_1[4 * 4];
	_T New_Point[4] = {};   //计算出来的点P，对应矩阵E的远处点

	//此处把一个Scale(奇异值)去掉了
	Gen_Homo_Matrix(R, t, T2);
	//Disp(P2, 4, 4, "P2");

	_T fMax_Depth, kMinDepth = 2.2204460492503131e-16;

	//P1为I，说得过去，就当视点（相机中心）到像素的方向与归一化平面垂直（正交）
	Gen_I_Matrix(T1, 4, 4);

	//算个fMax_Depth;
	Matrix_Transpose(R, 3, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, t, 1, Temp_1);
	fMax_Depth = 1000.f * fGet_Mod(Temp_1, 3);

	//Disp(P2, 4, 4, "P2");

	_T fDepth_1, fDepth_2, fMod_P2_Col_2;
	int i, j, iCount = *piCount;
	//这步是否有必要，P2是否正交？
	_T V[4] = { T2[2],T2[6],T2[10],T2[14] };
	fMod_P2_Col_2 = fGet_Mod(V, 4);

	Rotation_Matrix_2_Vector(R, V);
	
	//Disp(V, 1, 4, "V");
	New_Point[3] = 1;
	_T R1[3 * 3] = { 1,0,0,0,1,0,0,0,1 }, t1[3] = { 0 };
	_T fError_Sum = 0;
	if(pInlier_Mask)
		memset(pInlier_Mask, 0, iCount);
	for (i = j = 0; i < iCount; i++)
	{
		Triangulate_Point(Point_1[i], Point_2[i], T1, T2, New_Point);
		//Disp(New_Point, 1, 4);
		//printf("Error: %f\n",fTriangulate_Error(Point_1[i], Point_2[i], T1, T2, New_Point));
		/*Disp(R, 3, 3, "R");
		Disp(t, 1, 3, "t");*/
		//Triangulate_Point_1(Point_1[i], R1, t1, Point_2[i], R, t, New_Point);
		//Disp(New_Point, 1, 4);
		//printf("Error: %f\n", fTriangulate_Error(Point_1[i], Point_2[i], T1, T2, New_Point));
		//fError_Sum += fTriangulate_Error(Point_1[i], Point_2[i], P1, P2, New_Point);

		//再算深度, P1的第二行为(0,0,1,0), 所以，别搞那么复杂，直接赋值
		fDepth_1 = New_Point[2];
		if (fDepth_1 > kMinDepth /*&& fDepth_1 < fMax_Depth*/)
		{
			//此处，P*X=x => P的第二行点乘X就是(x,y,z)中的z
			//后面再乘一个列向量的模待考，目前只是1
			fDepth_2 = fDot(&T2[2 * 4], New_Point, 4) * fMod_P2_Col_2;
			if (fDepth_2 > kMinDepth /*&& fDepth_2 < fMax_Depth*/)
			{
				//if (j == 71)
					//printf("here");
				if (Point_3d)
					memcpy(Point_3d[j], New_Point, 3 * sizeof(_T));
				if(pInlier_Mask)
					pInlier_Mask[i] = 1;
				j++;
			}
		}
	}

	//printf("Error_Sum:%f\n", fError_Sum);
	*piCount = j;
	return;
}

template<typename _T>void E_2_R_t_Pixel_Pos(_T E[3 * 3], _T Point_1[][2], _T Point_2[][2], _T K[], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3])
{//未做完，还得继续
	int i;
	for (i = 0; i < iCount; i++)
	{
		Point_1[i][0] -= K[2], Point_1[i][1] -= K[5];
		Point_2[i][0] -= K[2], Point_2[i][1] -= K[5];
	}
	
	_T R1[3 * 3], R2[3 * 3], t1[3], t2[3];
	_T* Comb[4][2] = { {R1,t1},{R2,t1},{R1,t2},{R2,t2} };

	Decompose_E(E, R1, R2, t1, t2); //E = a * t^ * R
	
	//组成4对 (R1,t1), (R2,t1),(R1,t2),(R2,t2),分别检验转换后结果的对错
	int Count_1[4], iMax_Count = 0, iMax_Index;;
	for (i = 0; i < 4; i++)
	{
		Count_1[i] = iCount;
		Check_Cheirality(Point_1, Point_2, &Count_1[i], Comb[i][0], Comb[i][1], Point_3D);
		if (Count_1[i] > iMax_Count)
		{
			iMax_Count = Count_1[i];
			iMax_Index = i;
		}
		if (Count_1[i] == iCount)
			break;  //找到了

		/*for (int j = 0; j < iCount; j++)
		{
			_T x1[2], x2[2];
			
			x1[0] = Point_1[0] / K[0];
			x1[1] = Point_1[1] / K[0];
			x2[0] = Point_2[0] / K[0];
			x2[1] = Point_2[1] / K[0];
			
		}*/
	}

	return;
}
template<typename _T>void E_2_R_t(_T E[3 * 3], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3],int *piCount,unsigned char *pInlier_Mask)
{//从E中恢复R矩阵与 位移 t, 注意了，由于向量可行可列，在列向量前提下，
	//此处用归一化坐标
	//E= t^ * R 这才跟原来一直的计算对齐，先旋转后位移。否则天下大乱
	//并且，准确的表达是 E = a * t^ * R, 其中 a是E的特征值。否则数值不对
	_T R1[3 * 3], R2[3 * 3], t1[3], t2[3];
	_T* Comb[4][2] = { {R1,t1},{R2,t1},{R1,t2},{R2,t2} };
	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
	unsigned char* pInlier_Mask_1 = (unsigned char*)pMalloc(iCount);

	int i;
	Decompose_E(E, R1, R2, t1, t2); //E = a * t^ * R

	//组成4对 (R1,t1), (R2,t1),(R1,t2),(R2,t2),分别检验转换后结果的对错
	int Count_1[4], iMax_Count=0,iMax_Index;;
	for (i = 0; i < 4; i++)
	{
		Count_1[i] = iCount;
		//_T v[4];
		//fsRotation_Matrix_2_Vector(Comb[i][0],v);
		//Disp(v, 1, 4,"V");
		Check_Cheirality(Norm_Point_1, Norm_Point_2, &Count_1[i], Comb[i][0], Comb[i][1], pPoint_3D, pInlier_Mask_1);
					
		if (Count_1[i] > iMax_Count)
		{
			iMax_Count = Count_1[i];
			iMax_Index = i;
			if(Point_3D)
				memcpy(Point_3D, pPoint_3D, iMax_Count * 3 * sizeof(_T));
			if (pInlier_Mask)
				memcpy(pInlier_Mask, pInlier_Mask_1, iCount);
		}
		if (Count_1[i] == iCount)
			break;  //找到了
	}

	if (iMax_Count)
	{
		memcpy(R, Comb[iMax_Index][0], 3 * 3 * sizeof(_T));
		memcpy(t, Comb[iMax_Index][1], 3 * sizeof(_T));
	}
	if (piCount)
		*piCount = iMax_Count;

	Free(pPoint_3D);
	Free(pInlier_Mask_1);
	return;
}
template<typename _T>void Test_E(_T E[], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount)
{//这里验算一下给定的E 重新算一次 R t，主要验算 NP2= (x1/x2) * Rt * NP1
//注意，此处的t是归一化向量
//这个验算的初衷是通过三角化求出两匹配点对应的三维点X，这个X对于 x与x0来说是相等值，然后
//通过这个关系展开，不断得进行验算得到结果
	_T R[3 * 3], t[4],Rt[4*4],I[4*4];
	_T Temp_1[4 * 4],Point_3D[4], z1, z2;
	int i;

	E_2_R_t(E, Norm_Point_1, Norm_Point_2, iCount, R, t);

	Disp(E, 3, 3, "E");

	//这是一个很有用的实验，此处揭示了尺度不变性,意思是只要镜头方向不变，把三角按相似性质拉开有限倍，不改变投影结果
	//Matrix_Multiply(t, 3, 1, (_T)1000.f, t);

	Gen_Homo_Matrix(R, t, Rt);
	Gen_I_Matrix(I, 4, 4);
	for (i = 0; i < iCount; i++)
	{
		//先来个三角化，否则得不到深度z1,z2
		Triangulate_Point(Norm_Point_1[i], Norm_Point_2[i], I, Rt, Point_3D);

		//对于相机1，相机参数可以视为单位矩阵，那么深度z就是该点的z值
		Matrix_Multiply(I, 4, 4, Point_3D, 1, Temp_1);
		z1 = Temp_1[2];

		//对于相机二，KP相机参数中缺个K，就仅用外参Rt
		Matrix_Multiply(Rt, 4, 4, Point_3D, 1, Temp_1);
		z2 = Temp_1[2];
		
		//设x为A图在归一化平面上的点，x'是B图在归一化平面上对应x的点
		//正确的关系是 x' = (z1/z2) * Rt * x, 此时，z1,z2的参与必不可少，因为要恢复齐次
		//而z1,z2只有三角化以后才有，故此要验算这个结果，必须逐步恢复齐次坐标
		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Matrix_Multiply(Temp_1, 1, 2, z1, Temp_1);	//第一次化齐次坐标 (x,y,1) * z1
		Temp_1[2] = z1, Temp_1[3] = 1;						//第二次化齐次坐标 (z1*x, z1*y, z1, 1)

		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);		//再算 Rt * x
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2, Temp_1);	//最后再除以 z2, 此时就符合三角化公式了
		Disp(Norm_Point_2[i], 1, 2, "NP2");
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");
		//总结一下，x' = (z1/z2) * Rt * x， 但看这条式子是有问题的，因为x是2d点，Rt是三维变换阵，驴唇对马嘴
		//所以，要两者能一起运算，关键是将x变为三维点
		//x= (x,y,1)*z1 变成(x*z1,y*z1,z1) 再补1，最后 x=（x*z1,y*z1,z1,1) 这就可以参与运算了
		//最后x'的结果  x' = (z1/z2) * Rt * x 这就对了
	}
	return;
}

template<typename _T>void Gen_Camera_Intrinsic(_T K[3 * 3], float fFocal, float a, float b, float cx, float cy)
{//此处生成一个相机内参K,要彻底搞明白内参的来龙去脉
//fFocal: 针孔相机的焦距，这个与Z方向的距离相关。
//a:	一个相机有个成像平面，不管这个区域用什么单位，米还是厘米，都会落实
//		到每个单位对应多少个像素。这个值就是每单位水平方向上对应多少个像素
//b:	每单位在垂直方向上对用多少个像素。经常会a=b
//cx:	加上一个偏移量构成屏幕坐标。cx为水平偏移量
//cy:	垂直偏移量。 对于一个w*h的屏幕， (cx,cy)=(w/2,h/2)
//然而，只有K坐标还不足以由一个空间点坐标(x,y,z)直接推导出其对应的像素坐标(u,v,1)，还缺个Z
//因为还有远小近大的投影关系。 所以，对于空间中的一点(x,y,z)，有 (u,v)'=1/z * K * (x,y,z)'
	memset(K, 0, 3 * 3 * sizeof(_T));
	K[0] = a * fFocal;
	K[4] = b * fFocal;
	K[2] = cx;
	K[5] = cy;
	K[8] = 1.f;
	return;
}

template<typename _T> void Determine_Confg(Two_View_Geometry* poGeo, _T Point_1[][2], _T Point_2[][2], int iCount,
	_T(**ppNew_Point_1)[2], _T(**ppNew_Point_2)[2], Mem_Mgr* poMem_Mgr)
{//这里搞到很啰嗦
	float fEF_Ratio = (float)poGeo->m_oReport_E.m_oSupport.m_iInlier_Count / poGeo->m_oReport_F.m_oSupport.m_iInlier_Count,
		fHF_Ratio = (float)poGeo->m_oReport_H.m_oSupport.m_iInlier_Count / poGeo->m_oReport_F.m_oSupport.m_iInlier_Count,
		fHE_Ratio = (float)poGeo->m_oReport_H.m_oSupport.m_iInlier_Count / poGeo->m_oReport_E.m_oSupport.m_iInlier_Count;
	int num_inliers;
	unsigned char* best_inlier_mask = NULL;

	if (poGeo->m_oReport_E.m_bSuccess && fEF_Ratio > 0.95f && poGeo->m_oReport_E.m_oSupport.m_iInlier_Count >= 15)
	{//0.95与15都是经验值
		// Calibrated configuration.
		if (poGeo->m_oReport_E.m_oSupport.m_iInlier_Count >= poGeo->m_oReport_F.m_oSupport.m_iInlier_Count)
		{
			num_inliers = poGeo->m_oReport_E.m_oSupport.m_iInlier_Count;
			best_inlier_mask = poGeo->m_oReport_E.m_pInlier_Mask;
		}
		else
		{
			num_inliers = poGeo->m_oReport_F.m_oSupport.m_iInlier_Count;
			best_inlier_mask = poGeo->m_oReport_F.m_pInlier_Mask;
		}

		if (fHE_Ratio > 0.8f)
		{
			poGeo->m_iConfig = Two_View_Geometry::PLANAR_OR_PANORAMIC;
			if (poGeo->m_oReport_H.m_oSupport.m_iInlier_Count > num_inliers)
			{
				num_inliers = poGeo->m_oReport_H.m_oSupport.m_iInlier_Count;
				best_inlier_mask = poGeo->m_oReport_H.m_pInlier_Mask;
			}
		}
		else
			poGeo->m_iConfig = Two_View_Geometry::CALIBRATED;
	}
	else if (poGeo->m_oReport_F.m_bSuccess && poGeo->m_oReport_F.m_oSupport.m_iInlier_Count >= 15)
	{
		// Uncalibrated configuration.
		num_inliers = poGeo->m_oReport_F.m_oSupport.m_iInlier_Count;
		best_inlier_mask = poGeo->m_oReport_F.m_pInlier_Mask;
		if (fHF_Ratio > 0.8f)
		{
			poGeo->m_iConfig = Two_View_Geometry::PLANAR_OR_PANORAMIC;
			if (poGeo->m_oReport_H.m_oSupport.m_iInlier_Count > num_inliers)
			{
				num_inliers = poGeo->m_oReport_H.m_oSupport.m_iInlier_Count;
				best_inlier_mask = poGeo->m_oReport_H.m_pInlier_Mask;
			}
		}
		else
			poGeo->m_iConfig = Two_View_Geometry::UNCALIBRATED;
	}
	else if (poGeo->m_oReport_H.m_bSuccess && poGeo->m_oReport_H.m_oSupport.m_iInlier_Count >= 15)
	{
		num_inliers = poGeo->m_oReport_H.m_oSupport.m_iInlier_Count;
		best_inlier_mask = poGeo->m_oReport_H.m_pInlier_Mask;
		poGeo->m_iConfig = Two_View_Geometry::PLANAR_OR_PANORAMIC;
	}
	else
		poGeo->m_iConfig = Two_View_Geometry::DEGENERATE;

	_T(*pNew_Point_1)[2] = NULL, (*pNew_Point_2)[2] = NULL;
	int i, j;
	if (best_inlier_mask)
	{//此处将找到的最优inlier对找出来，形成新点集
		if (poMem_Mgr)
			pNew_Point_1 = (_T(*)[2])pMalloc(poMem_Mgr, num_inliers * 4 * sizeof(_T));
		else
			pNew_Point_1 = (_T(*)[2])malloc(num_inliers * 4 * sizeof(_T));
		pNew_Point_2 = pNew_Point_1 + num_inliers;
		for (i = 0, j = 0; i < iCount; i++)
		{
			if (best_inlier_mask[i])
			{
				pNew_Point_1[j][0] = Point_1[i][0];
				pNew_Point_1[j][1] = Point_1[i][1];
				pNew_Point_2[j][0] = Point_2[i][0];
				pNew_Point_2[j][1] = Point_2[i][1];
				j++;
			}
			//printf("%d ", best_inlier_mask[i]);
		}
	}
	*ppNew_Point_1 = pNew_Point_1;
	*ppNew_Point_2 = pNew_Point_2;
	poGeo->num_inliers = num_inliers;
	return;
}
template<typename _T> void Calculate_Triangulation_Angles(_T R[3 * 3], _T t[3], _T Point_3D[][3], int iCount, Two_View_Geometry::Config_Type iConfig, _T* pfAngle)
{
	_T Temp_1[4], Q[4], Center_1[3] = {}, Center_2[3];
	_T fBaseline_Length_Square = fGet_Mod(t, 3);

	//转换为4元组
	Rotation_Matrix_2_Quaternion(R, Q);
	//Disp(Q, 1, 4, "Q");

	//对R = -R'
	Matrix_Transpose(R, 3, 3, R);
	Matrix_Multiply(R, 3, 3, (_T)-1, R);
	//Disp(R, 3, 3, "R");

	Matrix_Multiply(R, 3, 3, t, 1, Center_2);
	//Disp(Center_2, 1, 3,"C2");

	_T* pAngle = (_T*)pMalloc(&oMatrix_Mem, iCount * sizeof(_T));
	_T ray_length_squared1, ray_length_squared2, fDenominator, fNominator, fAngle;

	for (int i = 0; i < iCount; i++)
	{
		//向量乘方求和用点积了事
		ray_length_squared1 = fDot(Point_3D[i], Point_3D[i], 3);
		Vector_Minus(Point_3D[i], Center_2, 3, Temp_1);
		ray_length_squared2 = fDot(Temp_1, Temp_1, 3);

		fDenominator =(_T)(2.f * sqrt(ray_length_squared1 * ray_length_squared2));
		fNominator = ray_length_squared1 + ray_length_squared2 - fBaseline_Length_Square;
		fAngle =(_T)abs(acos(fNominator / fDenominator));
		pAngle[i] = std::min(fAngle, PI - fAngle);
	}

	//Quick_Sort(pAngle, 0, iCount - 1);
	//Disp(pAngle, iCount, 1, "Angle");
	_T fMid = oGet_Nth_Elem(pAngle, iCount,iCount / 2);
	if (iConfig == Two_View_Geometry::PLANAR_OR_PANORAMIC)
	{
		printf("Not implemented\n");
		return;
	}
	*pfAngle = fMid;
	if (pAngle)
		Free(&oMatrix_Mem, pAngle);
	return;
}
template<typename _T> void Estimate_Relative_Pose(Two_View_Geometry oGeo, float Camera_1[3], float Camera_2[2], _T Point_1[][2], _T Point_2[][2], int iCount, Mem_Mgr* poMem_Mgr)
{// Camera: f, c1,c2,
	if (oGeo.m_iConfig != Two_View_Geometry::CALIBRATED && oGeo.m_iConfig != Two_View_Geometry::UNCALIBRATED && oGeo.m_iConfig != Two_View_Geometry::PLANAR &&
		oGeo.m_iConfig != Two_View_Geometry::PANORAMIC && oGeo.m_iConfig != Two_View_Geometry::PLANAR_OR_PANORAMIC)
		return;
	_T(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];
	pNorm_Point_1 = (_T(*)[2])pMalloc(poMem_Mgr, iCount * 4 * sizeof(_T));
	pNorm_Point_2 = pNorm_Point_1 + iCount;
	Normalize_Point(Point_1, Point_2, iCount, pNorm_Point_1, pNorm_Point_2, Camera_1[0], Camera_1[1], Camera_1[2]);

	_T R[3 * 3], t[3];
	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(poMem_Mgr, iCount * 3 * sizeof(_T));
	if (oGeo.m_iConfig == Two_View_Geometry::CALIBRATED || oGeo.m_iConfig == Two_View_Geometry::UNCALIBRATED)
		E_2_R_t((_T*)oGeo.m_oReport_E.m_Modal, pNorm_Point_1, pNorm_Point_2, iCount, R, t, pPoint_3D);
	//Disp((_T*)pPoint_3D, iCount, 3);
	//Disp(oGeo.m_oReport_E.m_Modal_d, 3, 3, "E");

	//验证未遂
	//E_Test_1(pNorm_Point_1, pNorm_Point_2,iCount, (_T*)oGeo.m_oReport_E.m_Modal, R, t);
	_T fAngle;
	int bSuccess;
	if (iCount)
		Calculate_Triangulation_Angles(R, t, pPoint_3D, iCount, oGeo.m_iConfig, &fAngle);
		
	if (iCount > 100 && t[2] < 0.95 && fAngle > 16.f * 0.0174532925199432954743716805978692718781530857086181640625)
		bSuccess = 1;
	else
		bSuccess = 0;
	//Disp(Q, 1, 4);

	if (pNorm_Point_1)
		Free(poMem_Mgr, pNorm_Point_1);
	if (pPoint_3D)
		Free(poMem_Mgr, pPoint_3D);
	return;
}

template<typename _T> void RGBD_2_Point_3D(Image oImage, unsigned short* pDepth, _T K[][3], _T fDepth_Factor, _T Point_3D[][3], int* piPoint_Count, unsigned char Color[][3])
{//通过相机内参和像素平面上的坐标还原空间点
	int y, x, i, iPoint_Count = 0, iDepth;
	for (y = i = 0; y < oImage.m_iHeight; y++)
	{
		for (x = 0; x < oImage.m_iWidth; x++, i++)
		{
			if (pDepth[i])
			{
				iDepth = (unsigned short)((pDepth[i] >> 8) + (pDepth[i] << 8));
				Point_3D[iPoint_Count][2] = (_T)iDepth / fDepth_Factor;
				Point_3D[iPoint_Count][0] = ((x - K[0][2]) * Point_3D[iPoint_Count][2]) / K[0][0];
				Point_3D[iPoint_Count][1] = ((y - K[1][2]) * Point_3D[iPoint_Count][2]) / K[1][1];
				//先看u,v的来历，像素平面上的坐标
				//u = X* f * s / Z
				//X = (u / f*s)*Z
				if (Color)
				{
					Color[iPoint_Count][0] = oImage.m_pChannel[0][i];
					Color[iPoint_Count][1] = oImage.m_pChannel[1][i];
					Color[iPoint_Count][2] = oImage.m_pChannel[2][i];
				}
				iPoint_Count++;
			}
		}
	}
	if (piPoint_Count)
		*piPoint_Count = iPoint_Count;
	return;
}
template<typename _T>void Image_Pos_2_Norm(_T Image_Pos[][2], int iCount, _T K[], _T(*pNorm_Point)[2])
{//将屏幕左边转换为归一化平面坐标
	//	x1 = (p1x - cx) / fx
	//	y1 = (p1y - cy) / fy
	_T fx = K[0], fy = K[4],
		cx = K[2], cy = K[5];

	for (int i = 0; i < iCount; i++)
	{
		pNorm_Point[i][0] = (Image_Pos[i][0] - cx) / fx;
		pNorm_Point[i][1] = (Image_Pos[i][1] - cy) / fy;
	}
	return;
}
template<typename _T>void Image_Pos_2_3D(_T Image_Pos[][2], unsigned short Depth[], int iCount, int iWidth, _T K[], _T fDepth_Factor, _T Pos_3D[][3])
{//之所以要用这个，是因为很多时候，Impage_Pos[2] = 1, 故此Impage_Pos只有x,y两项
	//Image_Pos: 像素平面上的点坐标(x,y)及深度图上的d信息, K 相机内参； fDepth_Factor:深度图量化因子
	//感觉这个函数也很实用，很多时候只关心几何信息，从第一手信息恢复空间坐标很重要
	for (int i = 0; i < iCount; i++)
	{
		int iPos =(int)( ((int)Image_Pos[i][1]) * iWidth + Image_Pos[i][0]);
		int iDepth = (unsigned short)((Depth[iPos] >> 8) + (Depth[iPos] << 8));
		Pos_3D[i][2] = (_T)iDepth / fDepth_Factor;
		//根据 p' = KP	,其中p'为像素平面上的点，P为空间点
		//p'x = K[0,0]*Px + K[0,1]
		//p'y = K[1,1]*Py + K[1,2]
		//Px = (p'x - K[0,1])/K[0,0]
		//Py = (p'y - K[1,2])/K[1,1]
		Pos_3D[i][0] = (Image_Pos[i][0] - K[2]) * Pos_3D[i][2] / K[0];
		Pos_3D[i][1] = (Image_Pos[i][1] - K[5]) * Pos_3D[i][2] / K[4];
		/*if (Depth[iPos] != 0)
			Disp(Pos_3D[i], 1, 3, "Point");*/
	}
	//Disp((_T*)Pos_3D,iCount,3);
	return;
}

template<typename _T>void Image_Pos_2_3D(_T Image_Pos[][3], int iCount,_T K[], _T fDepth_Factor,_T Pos_3D[][3])
{//Image_Pos: 像素平面上的点坐标(x,y)及深度图上的d信息, K 相机内参； fDepth_Factor:深度图量化因子
	//感觉这个函数也很实用，很多时候只关心几何信息，从第一手信息恢复空间坐标很重要
	int i;
	for (i = 0; i < iCount; i++)
	{
		Pos_3D[i][2] = (_T)Image_Pos[i][2] / fDepth_Factor;
		//根据 p' = KP	,其中p'为像素平面上的点，P为空间点
		//p'x = K[0,0]*Px + K[0,1]
		//p'y = K[1,1]*Py + K[1,2]
		//Px = (p'x - K[0,1])/K[0,0]
		//Py = (p'y - K[1,2])/K[1,1]
		Pos_3D[i][0] = ((Image_Pos[i][0] - K[2]) * Image_Pos[i][2]) / K[0];
		Pos_3D[i][1] = ((Image_Pos[i][1] - K[5]) * Image_Pos[i][2]) / K[4];
	}
	return;
}

template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_Source_1[][3], _T Point_2D_Source_2[][2], int iCount, _T K[], _T Pose[], int* piResult)
{//用高斯牛顿法搞BA估计，最简形式，只考虑 Ksi六元组的偏导
	//给定条件： Point_3D_1：空间点集1，也可以视为相机1观察到的空间点集
	//Point_2D_2，像素平面上的点集2
	//K： 相机1和相机2同一内参
	_T Pose_Estimate[4 * 4], Delta_Pose[4 * 4], Pose_Pre[4 * 4],
		Delta_Ksi[6];
	_T fSum_e, fSum_e_Pre = 1e10,
		E[2], JJt[6 * 6], H_Inv[6 * 6];
	int i, iResult = 1, iIter;
	const _T eps = (_T)1e-10;
	_T fx = K[0], fy = K[1 * 3 + 1], cx = K[2], cy = K[1 * 3 + 2];
	//初始条件下，迭代格中的位置为单位矩阵，表示无移动
	Gen_I_Matrix(Pose_Estimate, 4, 4);
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		_T  Sigma_H[6 * 6] = { 0 }, //H=∑J'J
			Sigma_JE[6] = { 0 };	//∑JE
		for (i = 0; i < iCount; i++)
		{
			_T Point_3D_1[4], * pPoint_2D_2;
			memcpy(Point_3D_1, Point_3D_Source_1[i], 3 * sizeof(_T));
			Point_3D_1[3] = 1;
			//用上次迭代得到的新位姿计算点集1的新位置
			Matrix_Multiply(Pose_Estimate, 4, 4, Point_3D_1, 1, Point_3D_1);
			if (abs(Point_3D_1[2]) < eps)
				continue;	//病态数据不算
			//用相机内参K投影到像素平面上，最理想是与点集2位置重合
			_T Point_2D_1[2] = { fx * Point_3D_1[0] / Point_3D_1[2] + cx, fy * Point_3D_1[1] / Point_3D_1[2] + cy };
			pPoint_2D_2 = Point_2D_Source_2[i];
			E[0] = pPoint_2D_2[0] - Point_2D_1[0];	//对应点i的数值差
			E[1] = pPoint_2D_2[1] - Point_2D_1[1];

			fSum_e += E[0] * E[0] + E[1] * E[1];

			_T  X_Sqr = Point_3D_1[0] * Point_3D_1[0],
				Y_Sqr = Point_3D_1[1] * Point_3D_1[1],
				Z_Sqr = Point_3D_1[2] * Point_3D_1[2];

			_T JE[6], J[6 * 2], //以下才是真正的Jt，和书上一致。源代码中是跳步走，不利于学习
				Jt[2 * 6] = { fx / Point_3D_1[2], 0 , -fx * Point_3D_1[0] / Z_Sqr, -fx * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, fx + fx * X_Sqr / Z_Sqr, -fx * Point_3D_1[1] / Point_3D_1[2],
								0, fy / Point_3D_1[2], -fy * Point_3D_1[1] / Z_Sqr, -fy - fy * Y_Sqr / Z_Sqr, fy * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, fy * Point_3D_1[0] / Point_3D_1[2] };
			Matrix_Multiply(Jt, 2, 6, (_T)-1.f, Jt);      //乘以-1后，这个就是所需的Jt,雅可比是求解的关键

			Matrix_Transpose(Jt, 2, 6, J);
			Matrix_Multiply(J, 6, 2, E, 1, JE);             //JE
			Vector_Add(Sigma_JE, JE, 6, Sigma_JE);

			Matrix_Multiply(J, 6, 2, Jt, 6, JJt);           //JJ'
			Matrix_Add(Sigma_H, JJt, 6, Sigma_H);           //
		}

		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 6, &iResult);		//求H(-1)
		Matrix_Multiply(H_Inv, 6, 6, Sigma_JE, 1, Delta_Ksi);	//H(-1)*JE
		Matrix_Multiply(Delta_Ksi, 1, 6, (_T)-1, Delta_Ksi);
		if (fSum_e_Pre <= fSum_e || !iResult)
			break;

		//接着从ξ恢复T, 将增量还原为齐次矩阵
		se3_2_SE3(Delta_Ksi, Delta_Pose);

		memcpy(Pose_Pre, Pose_Estimate, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose, 4, 4, Pose_Estimate, 4, Pose_Estimate);

		fSum_e_Pre = fSum_e;
	}
	if (iResult)
		*piResult = 1;
	memcpy(Pose, Pose_Pre, 4 * 4 * sizeof(_T));
}

//template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_Source_1[][3], _T Point_2D_Source_2[][2],int iCount,_T K[],_T Pose[],int *piResult)
//{//用高斯牛顿法搞BA估计，最简形式，只考虑 Ksi六元组的偏导
//	//给定条件： Point_3D_1：空间点集1，也可以视为相机1观察到的空间点集
//	//Point_2D_2，像素平面上的点集2
//	//K： 相机1和相机2同一内参
//	_T e[3], fx = K[0], fy = K[1 * 3 + 1], cx = K[2], cy = K[1 * 3 + 2];
//	_T fSum_e, fSum_e_Pre = 1e10, Temp[6 * 6], Delta_Ksi[6];
//	_T Pose_Pre[4 * 4], Pose_Estimate[4 * 4], Delta_Pose[4 * 4];
//	int i,iResult=1,iIter;
//	const _T eps = (_T)1e-10;
//	//初始条件下，迭代格中的位置为单位矩阵，表示无移动
//	Gen_I_Matrix(Pose_Estimate, 4, 4);
//
//	for (iIter = 0;; iIter++)
//	{
//		fSum_e = 0;
//		_T  H[6 * 6] = { 0 }, //H=∑J'J
//			b[6] = { 0 };
//		for (i = 0; i < iCount; i++)
//		{
//			_T Point_3D_1[4], *pPoint_2D_2;
//			memcpy(Point_3D_1, Point_3D_Source_1[i], 3 * sizeof(_T));
//			Point_3D_1[3] = 1;
//			//用上次迭代得到的新位姿计算点集1的新位置
//			Matrix_Multiply(Pose_Estimate, 4, 4, Point_3D_1, 1, Point_3D_1);
//			if ( abs(Point_3D_1[2]) < eps)
//				continue;	//病态数据不算
//			//用相机内参K投影到像素平面上，最理想是与点集2位置重合
//			_T Point_2D_1[2] = { fx * Point_3D_1[0] / Point_3D_1[2] + cx, fy * Point_3D_1[1] / Point_3D_1[2] + cy };
//			pPoint_2D_2 = Point_2D_Source_2[i];
//			e[0] = pPoint_2D_2[0] - Point_2D_1[0];	//对应点i的数值差
//			e[1] = pPoint_2D_2[1] - Point_2D_1[1];
//
//			fSum_e += e[0] * e[0] + e[1] * e[1];
//			_T  X_Sqr = Point_3D_1[0] * Point_3D_1[0],
//				Y_Sqr = Point_3D_1[1] * Point_3D_1[1],
//				Z_Sqr = Point_3D_1[2] * Point_3D_1[2];
//
//			_T Jt[6 * 2], //注意，下面这个才是真正的Jt。 书上那个是跳步，把符号搞里头
//				J[2 * 6] = { -fx / Point_3D_1[2], 0 , fx * Point_3D_1[0] / Z_Sqr, fx * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, -fx - fx * X_Sqr / Z_Sqr, fx * Point_3D_1[1] / Point_3D_1[2],
//				0, -fy / Point_3D_1[2], fy * Point_3D_1[1] / Z_Sqr, fy + fy * Y_Sqr / Z_Sqr, -fy * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, -fy * Point_3D_1[0] / Point_3D_1[2] };
//
//			Matrix_Transpose(J, 2, 6, Jt);
//			Matrix_Multiply(Jt, 6, 2, J, 6, Temp);
//			Matrix_Add(H, Temp, 6, H);  //H += J'J;
//
//			Matrix_Multiply(Jt, 6, 2, e, 1, Temp);
//			Vector_Add(b, Temp, 6, b);
//		}
//		Matrix_Multiply(b, 1, 6, (_T)-1, b);
//		
//		//解方程 Hx = b
//		Solve_Linear_Gause(H, 6, b, Delta_Ksi, &iResult);
//
//		//此处的停机条件有讲究，因为Delta_Ksi是前位移后旋转，所以不能用传统的|Δξ|≈ 0 完事
//		//只能用误差不发散为准
//		if (fSum_e >= fSum_e_Pre || !iResult)
//			break;
//
//		//将增量还原为齐次矩阵
//		se3_2_SE3(Delta_Ksi, Delta_Pose);
//
//		memcpy(Pose_Pre, Pose_Estimate, 4 * 4 * sizeof(_T));
//		Matrix_Multiply(Delta_Pose, 4, 4, Pose_Estimate, 4, Pose_Estimate);
//		//Disp(Pose, 4, 4, "Pose");
//		fSum_e_Pre = fSum_e;
//	}
//
//	*piResult = iResult;
//	if (iResult)
//		memcpy(Pose, Pose_Pre, 4 * 4 * sizeof(_T));
//}

template<typename _T>void Get_Drive_UV_P(_T K[3 * 3], _T P[3], _T J[2 * 3])
{//空间某一点的扰动对uv的变化。 uv二维，P为3维，最终的梯度维2x3矩阵
	_T fZ_Sqr = P[2] * P[2];
	J[0] = K[0 * 3 + 0] / P[2], J[2] = -K[0 * 3 + 0] * P[0] / fZ_Sqr;
	J[1*3+0] = K[1 * 3 + 1] / P[2], J[1*3+2] = -K[1 * 3 + 1] * P[1] / fZ_Sqr;
	J[1] = J[1 * 3 + 0] = 0;
}

template<typename _T>void Get_Drive_UV_P(_T fx, _T fy, _T P[3], _T J[2 * 3])
{//空间某一点的扰动对uv的变化。 uv二维，P为3维，最终的梯度维2x3矩阵
	//注意，只与焦距有关，与位移围观
	_T fZ_Sqr = P[2] * P[2];
	J[0] = fx / P[2], J[2] = -fx * P[0] / fZ_Sqr;
	J[1 * 3 + 1] = fy / P[2], J[1 * 3 + 2] = -fy*P[1] / fZ_Sqr;
	J[1] = J[1 * 3 + 0] = 0;
}

template<typename _T>void Get_Deriv_E_Ksi(_T K[3 * 3], _T TP[3],_T J[2*6])
{//TP=P', 求 de/dksi，用于PnP_3D_2D估计中。给定一个P',焦距，求雅可比
	_T z_recip = 1.f / TP[2];
	_T z_recip_sqr = z_recip * z_recip;
	J[0] = K[0] * z_recip;  //fx/Z'
	J[7] = K[4] * z_recip;  //fy/Z'
	J[1] = J[6] = 0;

	J[2] = -K[0] * TP[0] * z_recip_sqr; //- fx*X'/Z'^2
	J[5] = -J[0] * TP[1];               //- fy*Y'/Z'^2
	J[3] = J[2] * TP[1];                //- fx*X'Y'/Z'^2
	J[4] = K[0] - J[2] * TP[0];         //fx + fx*X'^2/Z'^2

	J[8] = -K[4] * TP[1] * z_recip_sqr; //-fy*Y'/Z'^2
	J[9] = -K[4] + J[8] * TP[1];        //-fy - fy*Y'/Z'^2
	J[10] = -J[8] * TP[0];
	J[11] = J[7] * TP[0];
	return;
}
template<typename _T>void Get_Deriv_TP_Ksi(_T T[4 * 4], _T TP[3], _T Deriv[4 * 6])
{//这个才是真正的dTP/dksi，来的是TP,不是P
	_T P1[4]= { TP[0],TP[1],TP[2],1 };
	_T P1_M[3 * 3];

	//∂TP/∂ξ = ∂P'/∂ξ= I -P'^
	Hat(P1, P1_M);

	//I
	Deriv[0 * 6 + 0] = 1; Deriv[0 * 6 + 1] = 0; Deriv[0 * 6 + 2] = 0;
	Deriv[1 * 6 + 0] = 0; Deriv[1 * 6 + 1] = 1; Deriv[1 * 6 + 2] = 0;
	Deriv[2 * 6 + 0] = 0; Deriv[2 * 6 + 1] = 0; Deriv[2 * 6 + 2] = 1;
	//-P'^
	Deriv[0 * 6 + 3] = -P1_M[0]; Deriv[0 * 6 + 4] = -P1_M[1]; Deriv[0 * 6 + 5] = -P1_M[2];
	Deriv[1 * 6 + 3] = -P1_M[3]; Deriv[1 * 6 + 4] = -P1_M[4]; Deriv[1 * 6 + 5] = -P1_M[5];
	Deriv[2 * 6 + 3] = -P1_M[6]; Deriv[2 * 6 + 4] = -P1_M[7]; Deriv[2 * 6 + 5] = -P1_M[8];

	//以下只具有理论意义，一般用不上，所以注掉
	//memset(&Deriv[3 * 6], 0, 6 * sizeof(_T));
	return;
}

template<typename _T>void Get_Deriv_TP_Ksi_1(_T T[4 * 4], _T P[3], _T Deriv[4 * 6])
{//有必要把扰动模型也做出来。其实就是∂TP/∂ξ。给定一点P及其变换T。给T一点扰动，看变化率。
//此时，ξ就是6维向量变量，但是隐含了。TP为4维齐次坐标。所以，目标为4x6矩阵
//注意，这个明明有点头疼，来的是原空间点，求的确实dTP/dksi,要小心

	_T _P[4] = { P[0],P[1],P[2],1 };
	_T P1[4];
	_T P1_M[3 * 3];

	Matrix_Multiply(T, 4,4,_P,1,P1);
	//∂TP/∂ξ = ∂P'/∂ξ= I -P'^
	Hat(P1, P1_M);

	//I
	Deriv[0 * 6 + 0] = 1; Deriv[0 * 6 + 1] = 0; Deriv[0 * 6 + 2] = 0;
	Deriv[1 * 6 + 0] = 0; Deriv[1 * 6 + 1] = 1; Deriv[1 * 6 + 2] = 0;
	Deriv[2 * 6 + 0] = 0; Deriv[2 * 6 + 1] = 0; Deriv[2 * 6 + 2] = 1;
	//-P'^
	Deriv[0 * 6 + 3] = -P1_M[0]; Deriv[0 * 6 + 4] = -P1_M[1]; Deriv[0 * 6 + 5] = -P1_M[2];
	Deriv[1 * 6 + 3] = -P1_M[3]; Deriv[1 * 6 + 4] = -P1_M[4]; Deriv[1 * 6 + 5] = -P1_M[5];
	Deriv[2 * 6 + 3] = -P1_M[6]; Deriv[2 * 6 + 4] = -P1_M[7]; Deriv[2 * 6 + 5] = -P1_M[8];

	//以下只具有理论意义，一般用不上，所以注掉
	//memset(&Deriv[3 * 6], 0, 6 * sizeof(_T));
	return;
}
template<typename _T>void ICP_BA_2_Image_1(_T P1[][3], _T P2[][3], int iCount, _T Pose[],int *piResult)
{//简单两图ICP，只做位姿调整，不做原点集位置调整
//用高斯牛顿法解ICP，紧咬高斯牛顿法形式
	_T Pose_Estimate[4 * 4], Delta_Pose[4 * 4], Pose_Pre[4 * 4],
		Delta_Ksi[6];
	_T fSum_e, fSum_e_Pre = 1e10,
		E[3], JJt[6 * 6], H_Inv[6 * 6];
	_T P_11[4]; //P1'
	int i, iResult, iIter;
	//Gen_I_Matrix(Pose_Estimate, 4, 4);
	memcpy(Pose_Estimate, Pose, 4 * 4 * sizeof(_T));

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		_T  Sigma_H[6 * 6] = { 0 }, //H=∑J'J
			Sigma_JE[6] = { 0 };  //∑JE
		for (i = 0; i < iCount; i++)
		{
			memcpy(P_11, P1[i], 3 * sizeof(_T));
			P_11[3] = 1;
			Matrix_Multiply(Pose_Estimate, 4, 4, P_11, 1, P_11);
			Vector_Minus(P2[i], P_11, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];

			_T Jt[4 * 6], J[6 * 3], JE[6];    //注意，J和Jt没有本质上的不同，只是行跟列的排列问题
			Get_Deriv_TP_Ksi_1(Pose_Estimate, P1[i], Jt);    //这个还不算Jt，因为他是g(x)的Jt,要求 e(x)的jt
			Matrix_Multiply(Jt, 3, 6, (_T)-1.f, Jt);      //乘以-1后，这个就是所需的Jt,雅可比是求解的关键
			//只要链导法结果正确，后面就能算出来

			Matrix_Transpose(Jt, 3, 6, J);
			Matrix_Multiply(J, 6, 3, E, 1, JE);             //JE
			Vector_Add(Sigma_JE, JE, 6, Sigma_JE);

			Matrix_Multiply(J, 6, 3, Jt, 6, JJt);           //JJ'
			Matrix_Add(Sigma_H, JJt, 6, Sigma_H);           //
		}

		Get_Inv_Matrix_Row_Op_2(Sigma_H, H_Inv, 6, &iResult);		//求H(-1)
		Matrix_Multiply(H_Inv, 6, 6, Sigma_JE, 1, Delta_Ksi);	//H(-1)*JE
		Matrix_Multiply(Delta_Ksi, 1, 6, (_T)-1, Delta_Ksi);	//

		if ( (fSum_e_Pre <= fSum_e && iIter>0) || !iResult)
			break;

		//接着从ξ恢复T, 将增量还原为齐次矩阵
		se3_2_SE3(Delta_Ksi, Delta_Pose);

		memcpy(Pose_Pre, Pose_Estimate, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose, 4, 4, Pose_Estimate, 4, Pose_Estimate);
		//Disp(Pose_Estimate, 4, 4, "Pose");
		printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}
	memcpy(Pose, Pose_Pre, 4 * 4 * sizeof(_T));
	return;
}

template<typename _T>void ICP_SVD(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[], int* piResult)
{//用SVD方法解ICP问题，这个速度应该快很多
	_T P_1_Centroid[3] = { 0 },
		P_2_Centroid[3] = { 0 };
	_T w[3 * 3] = { 0 }, q1[3], q2[3], q2q1t[3 * 3];
	int i, iResult = 1;

	for (i = 0; i < iCount; i++)
	{
		Vector_Add(P_1_Centroid, P_1[i], 3, P_1_Centroid);
		Vector_Add(P_2_Centroid, P_2[i], 3, P_2_Centroid);
	}
	Matrix_Multiply(P_1_Centroid, 1, 3, (_T)1.f / iCount, P_1_Centroid);
	Matrix_Multiply(P_2_Centroid, 1, 3, (_T)1.f / iCount, P_2_Centroid);

	for (i = 0; i < iCount; i++)
	{
		Vector_Minus(P_1[i], P_1_Centroid, 3, q1);
		Vector_Minus(P_2[i], P_2_Centroid, 3, q2);
		Matrix_Multiply(q2, 3, 1, q1, 3, q2q1t);
		Matrix_Add(w, q2q1t, 3, w);
	}

	//Disp(w, 3, 3, "w");
	_T R[3 * 3];
		
	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(w, oSVD);
	Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T*)oSVD.Vt, 3, R);
	Free_SVD(&oSVD);

	//此处要做一个行列式的判断
	if (fGet_Determinant(R, 3) < 0)
		Matrix_Multiply(R, 3, 3, (_T)(-1.f), R);
	//Disp(R, 3, 3, "R");

	//根据 p - Rp' -t =0 求t, t= p - Rp'
	_T t[3], Rp1[3 * 3];
	Matrix_Multiply(R, 3, 3, P_1_Centroid, 1, Rp1);
	Vector_Minus(P_2_Centroid, Rp1, 3, t);

	Gen_Homo_Matrix(R, t, Pose);
	*piResult = iResult;
}

int iGet_Pixel(Image oImage, int x, int y)
{
	return oImage.m_pChannel[0][y * oImage.m_iWidth + x];
}
float fGet_Pixel(Image oImage, float x, float y)
{
	const int iWidth_Minus_1 = oImage.m_iWidth - 1,
		iHeight_Minus_1 = oImage.m_iHeight - 1;
	float x1 = Clip3(0, iWidth_Minus_1, x),
		y1 = Clip3(0, iHeight_Minus_1, y);
	float xx = x - (int)x1;
	float yy = y - (int)y1;
	if (xx == 0 && yy == 0)
		return (float)iGet_Pixel(oImage, (int)x1, (int)y1);

	int x_a1 = min(iWidth_Minus_1, int(x1) + 1);
	int y_a1 = min(iHeight_Minus_1, int(y) + 1);

	//双线性而已，不想自己搞了，没营养
	return (1 - xx) * (1 - yy) * iGet_Pixel(oImage, (int)x1, (int)y1)
		+ xx * (1 - yy) * iGet_Pixel(oImage, (int)x_a1, (int)y1)
		+ (1 - xx) * yy * iGet_Pixel(oImage, (int)x, (int)y_a1)
		+ xx * yy * iGet_Pixel(oImage, (int)x_a1, (int)y_a1);
}

template<typename _T> void Optical_Flow_1(Image oImage_1, Image oImage_2, _T KP_1[][2], _T KP_2[][2], int iCount, int* piMatch_Count, int bHas_Initial, int bInverse)
{//以光流算法计算KP_1 对应在图二中的位置，未必全都找到
	//pbHas_Initial: 暂时不知意义，只知道在多层判断时用上
	int i, iResult, iIter, x, y, iMatch_Count = 0;
	_T x1, y1;
	_T dx, dy;
	const int r = 4, iIter_Count = 10;
	const int iWidth_Minus_1 = oImage_1.m_iWidth - 1,
		iHeight_Minus_1 = oImage_1.m_iHeight - 1;
	for (i = 0; i < iCount; i++)
	{
		dx = dy = 0;
		_T Keypoint_1[2] = { KP_1[i][0], KP_1[i][1] };
		if (bHas_Initial)
		{
			_T Keypoint_2[2] = { KP_2[i][0], KP_2[i][1] };
			dx = Keypoint_2[0] - Keypoint_1[0];
			dy = Keypoint_2[1] - Keypoint_1[1];
		}
		_T fCost, fPre_Cost = 0, fError;
		iResult = 1;
		for (iIter = 0; iIter < iIter_Count; iIter++)
		{
			_T H[2 * 2] = { 0 }, J[2], JJt[2 * 2], JE[2], b[2] = { 0 };    //这三件宝一出就知道是高斯牛顿法
			_T Delta_x[2];
			fCost = 0;
			for (x = -r; x < r; x++)
			{
				x1 = Keypoint_1[0] + x;
				for (y = -r; y < r; y++)
				{
					y1 = Keypoint_1[1] + y;
					fError = fGet_Pixel(oImage_1, (float)x1, (float)y1) - fGet_Pixel(oImage_2, (float)(x1 + dx), (float)(y1 + dy));
					if (bInverse == false)
					{//此处直接x-1
						J[0] = -(fGet_Pixel(oImage_2, (float)(x1 + dx + 1), (float)(y1 + dy)) - fGet_Pixel(oImage_2, (float)(x1 + dx - 1), (float)(y1 + dy))) * 0.5f;
						J[1] = -(fGet_Pixel(oImage_2, (float)(x1 + dx), (float)(y1 + dy + 1)) - fGet_Pixel(oImage_2, (float)(x1 + dx), (float)(y1 + dy - 1))) * 0.5f;
					}
					else if (iIter = -0)
					{
						J[0] = -(fGet_Pixel(oImage_1, (float)(x1 + 1), (float)(y1)) - fGet_Pixel(oImage_1, (float)(x1 - 1), (float)(y1)));
						J[1] = -(fGet_Pixel(oImage_1, (float)(x1), (float)(y1 + 1)) - fGet_Pixel(oImage_1, (float)(x1), (float)(y1 - 1)));
					}

					Matrix_Multiply(J, 1, 2, -fError, JE);
					Vector_Add(b, JE, 2, b);
					fCost += fError * fError;
					if (bInverse == false || iIter == 0)
					{
						Transpose_Multiply(J, 2, 1, JJt);//H += JJt
						Matrix_Add(H, JJt, 2, H);
					}
				}
			}
			//解个方程
			Solve_Linear_Gause(H, 2, b, Delta_x, &iResult);

			//未必每个方程都有解
			if (!iResult)
			{
				if (fError)
				{//真错
					printf("error");
					break;
				}
				else
					iResult = 1;
			}
			if (iIter > 0 && fCost > fPre_Cost)
				break;

			dx += Delta_x[0];
			dy += Delta_x[1];
			fPre_Cost = fCost;
			if (fGet_Mod(Delta_x, 2) < 1e-2)
				break;// converge
		}

		if (iResult)
		{
			KP_2[i][0] = KP_1[i][0] + dx;
			KP_2[i][1] = KP_1[i][1] + dy;
		}
		else
			KP_2[i][0] = KP_2[i][1] = 1e10;
	}

	//调整Keypoint, 两图都调
	for (i = 0; i < iCount; i++)
	{
		if (KP_2[i][0] == 1e10)
		{//失配
			swap(KP_1[i][0], KP_1[iCount - 1][0]);
			swap(KP_1[i][1], KP_1[iCount - 1][1]);
			swap(KP_2[i][0], KP_2[iCount - 1][0]);
			swap(KP_2[i][1], KP_2[iCount - 1][1]);
			iCount--;
		}
	}
	iMatch_Count = iCount;
	if (piMatch_Count)
		*piMatch_Count = iMatch_Count;
	return;
}

template<typename _T>void ICP_BA_2_Image_2(_T P1[][3], _T P2[][3], int iCount, _T Pose[], int* piResult)
{//简单两图ICP，既做位姿调整，也做原点集位置调整。用高斯牛顿法解ICP，紧咬高斯牛顿法形式
	_T E[3], R[3 * 3], P11[4], fSum_e, fSum_e_Pre = 1e10, X[9];
	_T Pose_Estimate[4 * 4], Delta_Pose[4 * 4], Pose_Pre[4 * 4],
		Jct[4 * 6], Jt[3 * 9], J[9 * 3], JEt[9], H_Inv[9 * 9];
	union {
		_T H[9 * 9];
		_T I[9 * 9];
	};
	_T Sigma_H[9 * 9], Sigma_JEt[9];
	_T(*P1_Pre)[3];

	int iIter, i, j, k, iResult;
	Gen_I_Matrix(Pose_Estimate, 4, 4);
	P1_Pre = (_T(*)[3])pMalloc(&oMatrix_Mem, iCount * 3 * sizeof(_T));

	for (iIter = 0;; iIter++)
	{
		//把R分离出来，因为这是 ∂P'/∂P
		Get_R_t(Pose_Estimate, R);
		fSum_e = 0;
		memset(Sigma_H, 0, 9 * 9 * sizeof(_T));
		memset(Sigma_JEt, 0, 9 * sizeof(_T));
		for (i = 0; i < iCount; i++)
		{
			Get_Homo_Pos(P1[i], P11);
			Matrix_Multiply(Pose_Estimate, 4, 4, P11, 1, P11);
			Vector_Minus(P2[i], P11, 3, E);         //求得误差E
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];

			Get_Deriv_TP_Ksi_1(Pose_Estimate, P1[i], Jct);    //∂TP/∂ξ
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
			{
				for (k = 0; k < 6; k++)
					Jt[j * 9 + k] = Jct[j * 6 + k];
				for (k = 0; k < 3; k++)
					Jt[j * 9 + 6 + k] = R[j * 3 + k];
			}
			Matrix_Multiply(Jt, 3, 9, (_T)-1.f, Jt);    //J'已经到位

			Matrix_Transpose(Jt, 3, 9, J);
			Matrix_Multiply(J, 9, 3, E, 1, JEt);    //JE'到位

			Matrix_Multiply(J, 9, 3, Jt, 9, H);

			Matrix_Add(Sigma_H, H, 9, Sigma_H);         //∑H JJ'到位

			Vector_Add(Sigma_JEt, JEt, 9, Sigma_JEt);   //∑JE'
		}

		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 9, &iResult);
		if (!iResult)
		{//此处对∑H的调整是关键，否则很有可能不满秩，注意，此处调整量应该是
			// H + λI，只不过H 没有呈现比较大的数值，故此λ=1已经够用
			Gen_I_Matrix(I, 9, 9);
			Matrix_Add(Sigma_H, I, 9, Sigma_H);
			Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 9, &iResult);
		}

		//Δx= -(∑H)(-1) * ∑JE'	 (E'为3x1)
		Matrix_Multiply(H_Inv, 9, 9, Sigma_JEt, 1, X);
		Matrix_Multiply(X, 1, 9, (_T)-1, X);

		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;

		//先更新一下Pose_Estimate
		se3_2_SE3(X, Delta_Pose);

		memcpy(Pose_Pre, Pose_Estimate, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose, 4, 4, Pose_Estimate, 4, Pose_Estimate);

		//备份P1_Pre
		memcpy(P1_Pre, P1, iCount * 3 * sizeof(_T));

		//继续调整P1
		for (i = 0; i < iCount; i++)
			Vector_Add(P1[i], &X[6], 3, P1[i]);

		//Disp(Pose_Estimate, 4, 4, "Pose");
		printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}

	memcpy(Pose, Pose_Pre, 4 * 4 * sizeof(_T));
	memcpy(P1, P1_Pre, iCount * 3 * sizeof(_T));
	Free(&oMatrix_Mem, P1_Pre);
	Disp(Pose, 4, 4, "Pose");
	return;
}
template<typename _T>void Disp_Error(_T P1[][3], _T P2[][3], int iCount, _T Pose[4 * 4])
{
	int i;
	_T P11[4];
	_T fError = 0;
	for (i = 0; i < iCount; i++)
	{
		memcpy(P11, P1[i], 4 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose, 4, 4, P11, 1, P11);
		fError += (P11[0] - P2[i][0]) * (P11[0] - P2[i][0]) +
			(P11[1] - P2[i][1]) * (P11[1] - P2[i][1]) +
			(P11[2] - P2[i][2]) * (P11[2] - P2[i][2]);
	}
	printf("Error:%f\n", fError);
	return;
}

template<typename _T>void Get_Delta_Pose(_T Pose_1[4 * 4], _T Pose_2[4 * 4], _T Delta_Pose[4 * 4])
{//已知Pose_1经过Rt变换到 Pose_2, 求这个变换
	//其实就是解方程 ΔPose * Pose_1 = Pose_2, 两边右乘 Pose_1(-1)即可，问题是，玩意是不是一个Homo_Matrix
	_T Pose_1_Inv[4 * 4];
	int iResult;
	Get_Inv_Matrix_Row_Op(Pose_1, Pose_1_Inv, 4, &iResult);
	Matrix_Multiply(Pose_2, 4, 4, Pose_1_Inv, 4, Delta_Pose);

	////验算Delta_Pose是不是一个Rt，如果是个一般矩阵有个毛用
	//_T R[3 * 3],R1[3*3], t[3], Rotation_Vector[4];
	//Get_R_t(Pose_1, R, t);
	//Disp(R, 3, 3, "R");
	//Rotation_Matrix_2_Vector(R, Rotation_Vector);
	//Rotation_Vector_2_Matrix(Rotation_Vector, R1);
	//if (fGet_Distance(R, R1, 9) > 0.0001f)
	//    printf("err");

	return;
}
template<typename _T>void Init_All_Camera_Data(Schur_Camera_Data<_T>* poData, int Point_Count_Each_Camera[], int iCamera_Count, int iPoint_Count)
{
	poData->m_iCamera_Count = iCamera_Count;
	poData->m_iPoint_Count = iPoint_Count;
	unsigned char* pBuffer, * pStart;
	int i, iObservation_Count = 0, iSize;
	for (i = 0; i < iCamera_Count; i++)
		iObservation_Count += Point_Count_Each_Camera[i];
	iSize = iCamera_Count * sizeof(Schur_Camera_Data<_T>::One_Camera_Data) +
		iPoint_Count * 9 * sizeof(_T) +
		iObservation_Count * sizeof(Schur_Camera_Data<_T>::One_Camera_Data::Point_Data);
	pBuffer = pStart = poData->m_pBuffer = (unsigned char*)pMalloc(&oMatrix_Mem, iSize);
	memset(pBuffer, 0, iSize);
	pBuffer += iCamera_Count * sizeof(Schur_Camera_Data<_T>::One_Camera_Data);
	poData->m_pData_3x3 = (_T(*)[9])pBuffer;
	pBuffer += iPoint_Count * 9 * sizeof(_T);
	typename Schur_Camera_Data<_T>::One_Camera_Data* poCamera;
	for (i = 0; i < iCamera_Count; i++)
	{
		poCamera = &poData->m_pCamera_Data[i];
		poCamera->m_iPoint_Count = Point_Count_Each_Camera[i];
		poCamera->m_pPoint_Data = (typename Schur_Camera_Data<_T>::One_Camera_Data::Point_Data*)pBuffer;
		pBuffer += Point_Count_Each_Camera[i] * sizeof(Schur_Camera_Data<_T>::One_Camera_Data::Point_Data);
	}
	poData->m_iObservation_Count = iObservation_Count;
	return;
}
template<typename _T>void Distribute_Data(Schur_Camera_Data<_T> oData, _T JJt[9 * 9], int iCamera_ID, int iPoint_ID)
{//将JJt 6x6部分累加，其余部分设值
	typename Schur_Camera_Data<_T>::One_Camera_Data* poCamera_Data = &oData.m_pCamera_Data[iCamera_ID];
	_T* pData_6x6 = poCamera_Data->m_Data_6x6;
	_T* pCur, * pCur_End = JJt + 6 * 9;
	int iCur;
	if (poCamera_Data->m_iCur >= poCamera_Data->m_iPoint_Count)
	{
		printf("exceed the point count of camera in Distribute_Data\n");
		return;
	}
	for (pCur = JJt, iCur = 0; pCur < pCur_End; pCur += 9)
		for (int x = 0; x < 6; x++, iCur++)
			pData_6x6[iCur] += pCur[x];

	//第二部分
	typename Schur_Camera_Data<_T>::One_Camera_Data::Point_Data* poPoint_Data = &poCamera_Data->m_pPoint_Data[poCamera_Data->m_iCur++];
	poPoint_Data->m_iPoint_Index = iPoint_ID;
	_T* pData_6x3 = poPoint_Data->m_Data_6x3;
	for (pCur = JJt + 6, iCur = 0; pCur < pCur_End; pCur += 9)
		for (int x = 0; x < 3; x++, iCur++)
			pData_6x3[iCur] = pCur[x];

	_T* pData_3x3 = oData.m_pData_3x3[iPoint_ID];
	pCur_End = JJt + 9 * 9;
	for (pCur = JJt + 6 * 9 + 6, iCur = 0; pCur < pCur_End; pCur += 9)
		for (int x = 0; x < 3; x++, iCur++)
			pData_3x3[iCur] += pCur[x];
	return;
}
template<typename _T>void Copy_Data_2_Sparse(Schur_Camera_Data<_T> oData, Sparse_Matrix<_T>* poA)
{//将All_Camera_Data抄到稀疏矩阵中
	int i, j, k, * pPoint_Index_2_Pos = (int*)pMalloc(&oMatrix_Mem, oData.m_iPoint_Count * sizeof(int));    //已知一个点索引，求其位置索引
	if (!pPoint_Index_2_Pos)
	{
		printf("Fail to allocate mem in Copy_Data_2_Sparse\n");
		return;
	}
	Sparse_Matrix<_T> oA = *poA;
	for (i = 0; i < oData.m_iCamera_Count; i++)
	{
		memset(pPoint_Index_2_Pos, -1, oData.m_iPoint_Count * sizeof(int));
		typename Schur_Camera_Data<_T>::One_Camera_Data* poCamera_Data = &oData.m_pCamera_Data[i];
		typename Schur_Camera_Data<_T>::One_Camera_Data::Point_Data* poPoint_Data;
		if (!poCamera_Data->m_iCur)
			continue;
		//做一个Revers Lookup 表,对Point进行排序，不用Quick Sort
		for (j = 0; j < poCamera_Data->m_iCur; j++)
		{
			poPoint_Data = &poCamera_Data->m_pPoint_Data[j];
			pPoint_Index_2_Pos[poPoint_Data->m_iPoint_Index] = j;
		}
		//往前挤
		for (j = k = 0; j < oData.m_iPoint_Count; j++)
		{
			if (pPoint_Index_2_Pos[j] >= 0)
				pPoint_Index_2_Pos[k++] = pPoint_Index_2_Pos[j];
		}

		int iCur_Camera_y = i * 6, iCur_Camera_x = iCur_Camera_y;
		for (j = 0; j < 6; j++, iCur_Camera_y++)
		{//逐行搞
			_T* pData = &poCamera_Data->m_Data_6x6[j * 6];
			int iPre_Item = oA.m_iCur_Item;

			//相机中的一行
			for (int i = 0; i < 6; i++)
				if (pData[i] != 0)
					oA.m_pBuffer[oA.m_iCur_Item++] = { (unsigned int)iCur_Camera_x + i,(unsigned int)iCur_Camera_y,pData[i],oA.m_iCur_Item + 1 };

			for (k = 0; k < poCamera_Data->m_iCur; k++)
			{
				poPoint_Data = &poCamera_Data->m_pPoint_Data[pPoint_Index_2_Pos[k]];
				int iPoint_Start_x = oData.m_iCamera_Count * 6 + poPoint_Data->m_iPoint_Index * 3;
				pData = &poCamera_Data->m_pPoint_Data[pPoint_Index_2_Pos[k]].m_Data_6x3[j * 3];
				if (oA.m_iCur_Item + 3 >= oA.m_iMax_Item_Count)
				{
					printf("Insufficient space in Coyy_Data_2_Sparse\n");
					return;
				}
				for (int i = 0; i < 3; i++)
					if (pData[i] != 0)
						oA.m_pBuffer[oA.m_iCur_Item++] = { (unsigned int)iPoint_Start_x + i,(unsigned int)iCur_Camera_y,pData[i],oA.m_iCur_Item + 1 };
			}
			if (oA.m_iCur_Item > iPre_Item) //防止全0情况
			{
				oA.m_pRow[iCur_Camera_y] = iPre_Item;
				oA.m_pBuffer[oA.m_iCur_Item - 1].m_iRow_Next = 0;
			}
		}
	}
	Build_Link_Col(oA);
	//Disp_Fillness(oA);
	//然后按照对称性把点数据从列方向抄到行方向
	int iCamera_End_x = oData.m_iCamera_Count * 6;
	for (i = 0; i < oData.m_iPoint_Count; i++)
	{//最外围以点推进
		int iPoint_Start_x = iCamera_End_x + i * 3;
		int iPoint_Start_y = iPoint_Start_x;
		for (j = 0; j < 3; j++)
		{
			int iPre_Item = oA.m_iCur_Item;
			if (oA.m_pCol[iPoint_Start_x + j])
			{//有东西
				typename Sparse_Matrix<_T>::Item* poItem = &oA.m_pBuffer[oA.m_pCol[iPoint_Start_x + j]];
				while (1)
				{
					if (poItem->y >= (unsigned int)iPoint_Start_y)
						break;
					if (poItem->m_fValue != 0)
						oA.m_pBuffer[oA.m_iCur_Item++] = { poItem->y,poItem->x,poItem->m_fValue,oA.m_iCur_Item + 1 };
					if (poItem->m_iCol_Next)
						poItem = &oA.m_pBuffer[poItem->m_iCol_Next];
					else
						break;
				}
			}
			_T* pData = &oData.m_pData_3x3[i][j * 3];
			for (int i = 0; i < 3; i++)
				if (pData[i] != 0)
					oA.m_pBuffer[oA.m_iCur_Item++] = { (unsigned int)iPoint_Start_x + i,(unsigned int)iPoint_Start_y + j,pData[i],oA.m_iCur_Item + 1 };
			
			if (iPre_Item != oA.m_iCur_Item)
			{
				oA.m_pRow[iPoint_Start_y + j] = iPre_Item;
				oA.m_pBuffer[oA.m_iCur_Item - 1].m_iRow_Next = 0;
			}
		}
	}
	Build_Link_Col(oA);
	//Disp_Fillness(oA);
	Free(&oMatrix_Mem, pPoint_Index_2_Pos);
	*poA = oA;
	return;
}
template<typename _T>void Free(Schur_Camera_Data<_T>* poData)
{
	if (poData && poData->m_pBuffer)
		Free(&oMatrix_Mem, poData->m_pBuffer);
	return;
}
template<typename _T>void Add_I_Matrix(Schur_Camera_Data<_T> oData, _T fRamda = 1.f)
{//列文-马夸方法需要对角线上加上fRamda
	int i;
	union {
		_T* pData_6x6;
		_T* pData_3x3;
	};
	//先搞Camera_Data
	for (i = 0; i < oData.m_iCamera_Count; i++)
	{
		pData_6x6 = oData.m_pCamera_Data[i].m_Data_6x6;
		pData_6x6[0] += fRamda, pData_6x6[1 * 6 + 1] += fRamda;
		pData_6x6[2 * 6 + 2] += fRamda, pData_6x6[3 * 6 + 3] += fRamda;
		pData_6x6[4 * 6 + 4] += fRamda, pData_6x6[5 * 6 + 5] += fRamda;
	}
	//再搞3x3 Data 
	for (i = 0; i < oData.m_iPoint_Count; i++)
	{
		pData_3x3 = oData.m_pData_3x3[i];
		pData_3x3[0] += fRamda, pData_3x3[4] += fRamda, pData_3x3[8] += fRamda;
	}
}
template<typename _T>void Schur_Get_C_Inv(Schur_Camera_Data<_T> oData)
{//对3x3块分别求逆，直接填回到3x3位置上了事，后面这些数据全不要了
	int i, iResult;
	for (i = 0; i < oData.m_iPoint_Count; i++)  //此处不用列主元法快很多，所以要对比精度
		Get_Inv_Matrix_Row_Op_2(oData.m_pData_3x3[i], oData.m_pData_3x3[i], 3, &iResult);
}
template<typename _T>void Schur_Gen_B_E_Cinv(Schur_Camera_Data<_T> oData, Sparse_Matrix<_T>* poB, Sparse_Matrix<_T>* poC_Inv, Sparse_Matrix<_T>* poE)
{
	int i, j, k, w_B = oData.m_iCamera_Count * 6,
		w_C = oData.m_iPoint_Count * 3;
	Sparse_Matrix<_T> oB, oC_Inv, oE;
	Init_Sparse_Matrix(&oB, oData.m_iCamera_Count * 6 * 6 + 2, w_B, w_B);
	Init_Sparse_Matrix(&oC_Inv, oData.m_iPoint_Count * 3 * 3 + 2, w_C, w_C);
	Init_Sparse_Matrix(&oE, oData.m_iObservation_Count * 6 * 3 + 2, w_C, w_B);

	int* pPoint_Index_2_Pos = (int*)pMalloc(&oMatrix_Mem, oData.m_iPoint_Count * sizeof(int));    //已知一个点索引，求其位置索引

	//生成oB, oE
	for (i = 0; i < oData.m_iCamera_Count; i++)
	{
		typename Schur_Camera_Data<_T>::One_Camera_Data* poCamera_Data = &oData.m_pCamera_Data[i];
		typename Schur_Camera_Data<_T>::One_Camera_Data::Point_Data* poPoint_Data;
		//if (!poCamera_Data->m_iCur)
		  //  continue;
		if (poCamera_Data->m_iCur)
		{
			memset(pPoint_Index_2_Pos, -1, oData.m_iPoint_Count * sizeof(int));
			//做一个Revers Lookup 表,对Point进行排序，不用Quick Sort
			for (j = 0; j < poCamera_Data->m_iCur; j++)
			{
				poPoint_Data = &poCamera_Data->m_pPoint_Data[j];
				pPoint_Index_2_Pos[poPoint_Data->m_iPoint_Index] = j;
			}
			//往前挤
			for (j = k = 0; j < oData.m_iPoint_Count; j++)
			{
				if (pPoint_Index_2_Pos[j] >= 0)
					pPoint_Index_2_Pos[k++] = pPoint_Index_2_Pos[j];
			}
		}
		int iCur_Camera_y = i * 6, iCur_Camera_x = iCur_Camera_y;
		for (j = 0; j < 6; j++, iCur_Camera_y++)
		{//逐行搞
			_T* pData = &poCamera_Data->m_Data_6x6[j * 6];
			int iPre_Item = oB.m_iCur_Item;

			if (oB.m_iCur_Item + 3 >= oB.m_iMax_Item_Count)
			{
				printf("Insufficient space in Coyy_Data_2_Sparse\n");
				return;
			}
			//相机中的一行
			for (int i = 0; i < 6; i++)
				if (pData[i] != 0)
					oB.m_pBuffer[oB.m_iCur_Item++] = { (unsigned int)iCur_Camera_x + i,(unsigned int)iCur_Camera_y,pData[i],oB.m_iCur_Item + 1 };
			if (oB.m_iCur_Item > iPre_Item) //防止全0情况
			{
				oB.m_pRow[iCur_Camera_y] = iPre_Item;
				oB.m_pBuffer[oB.m_iCur_Item - 1].m_iRow_Next = 0;
			}

			if (poCamera_Data->m_iCur)
			{
				iPre_Item = oE.m_iCur_Item;
				for (k = 0; k < poCamera_Data->m_iCur; k++)
				{
					poPoint_Data = &poCamera_Data->m_pPoint_Data[pPoint_Index_2_Pos[k]];
					int iPoint_Start_x = poPoint_Data->m_iPoint_Index * 3;
					pData = &poCamera_Data->m_pPoint_Data[pPoint_Index_2_Pos[k]].m_Data_6x3[j * 3];
					if (oE.m_iCur_Item + 3 >= oE.m_iMax_Item_Count)
					{
						printf("Insufficient space in Coyy_Data_2_Sparse\n");
						return;
					}
					/*if (oE.m_iCur_Item + 3 >= 47526)
						printf("here");*/
					for (int i = 0; i < 3; i++)
						if (pData[i] != 0)
							oE.m_pBuffer[oE.m_iCur_Item++] = { (unsigned int)iPoint_Start_x + i,(unsigned int)iCur_Camera_y,pData[i],oE.m_iCur_Item + 1 };
				}
				if (oE.m_iCur_Item > iPre_Item) //防止全0情况
				{
					oE.m_pRow[iCur_Camera_y] = iPre_Item;
					oE.m_pBuffer[oE.m_iCur_Item - 1].m_iRow_Next = 0;
				}
			}
		}
	}
	Build_Link_Col(oB);
	Build_Link_Col(oE);

	//Disp_Fillness(oB);
	//生成oC_Inv
	for (i = 0; i < oData.m_iPoint_Count; i++)
	{//逐个空间点搞
		_T* pData = oData.m_pData_3x3[i];
		int y = i * 3;
		int x = y;
		for (j = 0; j < 3; j++, pData += 3, y++)
		{//逐行
			oC_Inv.m_pRow[i * 3 + j] = oC_Inv.m_iCur_Item;
			int iPre_Item = oC_Inv.m_iCur_Item;
			for (k = 0; k < 3; k++)
			{//逐个行元素元素
				if (pData[k] != 0)
					oC_Inv.m_pBuffer[oC_Inv.m_iCur_Item++] = { (unsigned int)x + k,(unsigned int)y,pData[k],oC_Inv.m_iCur_Item + 1 };
			}
			if (iPre_Item != oC_Inv.m_iCur_Item)
			{
				oC_Inv.m_pRow[y] = iPre_Item;
				oC_Inv.m_pBuffer[oC_Inv.m_iCur_Item - 1].m_iRow_Next = 0;
			}
		}
	}
	Build_Link_Col(oC_Inv);

	//Disp_Fillness(oC_Inv);
	if (pPoint_Index_2_Pos)
		Free(&oMatrix_Mem, pPoint_Index_2_Pos);
	Compact_Sparse_Matrix(&oB);
	Compact_Sparse_Matrix(&oC_Inv);
	Compact_Sparse_Matrix(&oE);
	*poB = oB;
	*poC_Inv = oC_Inv;
	*poE = oE;
	return;
}
template<typename _T>void Schur_Get_E_Cinv(Schur_Camera_Data<_T> oData, Sparse_Matrix<_T>* poE_Cinv)
{//尝试直接在此E * C(-1)
	int i, j, w = oData.m_iPoint_Count * 3, h = oData.m_iCamera_Count * 6;
	typename Schur_Camera_Data<_T>::One_Camera_Data oCamera;
	typename  Schur_Camera_Data<_T>::One_Camera_Data::Point_Data oPoint;
	Sparse_Matrix<_T> oE_Cinv = *poE_Cinv;

	_T A[6 * 3], * E_Cinv;
	if (oE_Cinv.m_iMax_Item_Count)
		Reset_Sparse_Matrix(&oE_Cinv);
	else
		Init_Sparse_Matrix(&oE_Cinv, w * h, w, h);
	E_Cinv = (_T*)pMalloc(&oMatrix_Mem, w * h * sizeof(_T));
	memset(E_Cinv, 0, w * h * sizeof(_T));
	for (i = 0; i < oData.m_iCamera_Count; i++)
	{
		oCamera = oData.m_pCamera_Data[i];
		for (j = 0; j < oCamera.m_iPoint_Count; j++)
		{
			oPoint = oCamera.m_pPoint_Data[j];
			Matrix_Multiply(oPoint.m_Data_6x3, 6, 3, oData.m_pData_3x3[oPoint.m_iPoint_Index], 3, A);
			//Disp_Fillness(E_Cinv, h, w);
			Copy_Matrix_Partial(A, 6, 3, E_Cinv, w, oPoint.m_iPoint_Index * 3, i * 6);
		}
	}
	//Disp_Fillness(E_Cinv, h, w);
	Dense_2_Sparse(E_Cinv, h, w, &oE_Cinv);
	Free(&oMatrix_Mem, E_Cinv);
	*poE_Cinv = oE_Cinv;
	return;
}
template<typename _T>void Schur_Get_Xc(Schur_Camera_Data<_T> oData, Sparse_Matrix<_T>oEt_Delta_Xc, _T Xp[])
{//计算Matrix_Multiply(oC_Inv, oEt_Delta_Xc, &oXp);    //C(-1)*(w-Et* Delta Xc)
	_T* Et_Delta_Xc = (_T*)pMalloc(&oMatrix_Mem, oEt_Delta_Xc.m_iRow_Count * sizeof(_T));
	Sparse_2_Dense(oEt_Delta_Xc, Et_Delta_Xc);
	int i;
	_T* pData_3x3;
	for (i = 0; i < oData.m_iPoint_Count; i++)
	{
		pData_3x3 = oData.m_pData_3x3[i];
		Matrix_Multiply(pData_3x3, 3, 3, &Et_Delta_Xc[i * 3], 1, &Xp[i * 3]);
	}
	Free(&oMatrix_Mem, Et_Delta_Xc);
	return;
}
template<typename _T>void Solve_Linear_Schur(Schur_Camera_Data<_T> oData, _T Sigma_JE[], _T X[], int* pbSuccess)
{//用All_Camera_Data表示Slam数据，解出来以后放在X里
	//此处可能要多一步，用列文-马夸的方式，Add_I_Matrix
	Add_I_Matrix(oData);
	//unsigned long long tStart = iGet_Tick_Count();
	Schur_Get_C_Inv(oData);
	//Disp_Mem(&oMatrix_Mem, 0);
	//printf("%lld\n", iGet_Tick_Count() - tStart);
	//矩阵A高度稀疏，可分为4部分 A =    B   E
	//                                  E'  C
	//分离出4个稀疏矩阵
	int iResult, w_Xc = oData.m_iCamera_Count * 6,
		w_Xp = oData.m_iPoint_Count * 3;
	Sparse_Matrix<_T> oB, oC_Inv, oE, oEt;
	Schur_Gen_B_E_Cinv(oData, &oB, &oC_Inv, &oE);

	//下面就是经典Schur的一顿计算
	//算出EC(-1)
	Sparse_Matrix<_T> oE_Cinv, ov_E_Cinv_w, ov, ow, oXc, oXp;
	Schur_Get_E_Cinv(oData, &oE_Cinv);
	//Disp(oE_Cinv, "oE_Cinv");
	//Matrix_Multiply(oE, oC_Inv, &oE_Cinv);
	//Disp(oE_Cinv, "oE_Cinv");

	Init_Sparse_Matrix(&ov, w_Xc + 1, 1, w_Xc);
	Dense_2_Sparse(Sigma_JE, w_Xc, 1, &ov);

	Init_Sparse_Matrix(&ow, w_Xp + 1, 1, w_Xp);
	Dense_2_Sparse(Sigma_JE + w_Xc, w_Xp, 1, &ow);

	//EC(-1)w
	Matrix_Multiply(oE_Cinv, ow, &ov_E_Cinv_w); //EC(-1)w
	Matrix_Multiply(ov_E_Cinv_w, (_T)- 1.f);
	Matrix_Add(ov, ov_E_Cinv_w, &ov_E_Cinv_w);  //v-EC(-1)w
	//Disp(ov_E_Cinv_w);
	//Free_Sparse_Matrix(&ow);

	//再算B-EC(-1)*E'
	Sparse_Matrix<_T> oB_E_Cinv_Et, oEt_Delta_Xc;
	_T* v_E_Cinv_w;
	Init_Sparse_Matrix(&oEt, oE.m_iCur_Item, oE.m_iRow_Count, oE.m_iCol_Count);
	Matrix_Transpose_1(oE, &oEt);
	//Disp_Fillness(oEt);
	//此处也要优化
	Matrix_Multiply(oE_Cinv, oEt, &oB_E_Cinv_Et);    //E*C(-1)*E'
	Free_Sparse_Matrix(&oE_Cinv);
	Matrix_Multiply(oB_E_Cinv_Et, (_T)-1.f);
	Matrix_Add(oB, oB_E_Cinv_Et, &oB_E_Cinv_Et);//B-EC(-1)*E'
	Free_Sparse_Matrix(&oE);
	v_E_Cinv_w = (_T*)pMalloc(&oMatrix_Mem, ov_E_Cinv_w.m_iRow_Count * sizeof(_T));
	Sparse_2_Dense(ov_E_Cinv_w, v_E_Cinv_w);
	//Disp(v_E_Cinv_w, 1, oData.m_iCamera_Count * 6);
	//Disp(oB_E_Cinv_Et);
	Solve_Linear_Gause_1(oB_E_Cinv_Et, v_E_Cinv_w, X, &iResult);
	//此处的稀疏性放映了两个相机之间的关系。如果两相机交点处不为0，则表示有共同观察
	//Disp_Fillness(oB_E_Cinv_Et);
	Free_Sparse_Matrix(&oB_E_Cinv_Et);
	Free_Sparse_Matrix(&ov_E_Cinv_w);
	Free(&oMatrix_Mem, v_E_Cinv_w);
	Free_Sparse_Matrix(&oB);
	//Disp(X,1, ov_E_Cinv_w.m_iRow_Count);

	//最后求Delta_Xp
	Init_Sparse_Matrix(&oXc, ov.m_iRow_Count, 1, w_Xc);
	Dense_2_Sparse(X, w_Xc, 1, &oXc);
	Matrix_Multiply(oEt, oXc, &oEt_Delta_Xc, 0);
	Free_Sparse_Matrix(&oEt);
	Free_Sparse_Matrix(&oXc);
	Matrix_Minus(ow, oEt_Delta_Xc, &oEt_Delta_Xc);    //w-Et* Delta Xc
	Free_Sparse_Matrix(&ow);

	Init_Sparse_Matrix(&oXp, w_Xp, 1, w_Xp);
	//这个计算也要优化
	//Matrix_Multiply(oC_Inv, oEt_Delta_Xc, &oXp);    //C(-1)*(w-Et* Delta Xc)
	//Sparse_2_Dense(oXp, X + w_Xc);
	//优化成以下就快很多
	Schur_Get_Xc(oData, oEt_Delta_Xc, X + w_Xc);

	Free_Sparse_Matrix(&oXp);
	Free_Sparse_Matrix(&oC_Inv);
	Free_Sparse_Matrix(&oEt_Delta_Xc);
	Free_Sparse_Matrix(&ov);
	*pbSuccess = iResult;
	//Disp_Mem(&oMatrix_Mem, 0);
	return;
}
template<typename _T>void Init_Pose_Graph(Measurement<_T>* pMeasurement, int iMeasurement_Count, int iCamera_Count, Pose_Graph_Sigma_H<_T>* poPose_Graph)
{//暂时权宜之计，不是什么健壮性很高的算法，要重写。目的是先把位姿图搞定了事，其余以后再收拾
	Pose_Graph_Sigma_H<_T> oPose_Graph;
	int i, iSize;
	oPose_Graph.m_iCamera_Data_Count = iMeasurement_Count + iCamera_Count;
	oPose_Graph.m_iCamera_Count = iCamera_Count;

	iSize = oPose_Graph.m_iCamera_Data_Count * sizeof(Pose_Graph_Sigma_H<_T>::Camera_Data) +
		iCamera_Count * sizeof(Pose_Graph_Sigma_H<_T>::Camera_Line);
	oPose_Graph.m_pBuffer = (unsigned char*)pMalloc(&oMatrix_Mem, iSize);
	oPose_Graph.m_pLine = (typename Pose_Graph_Sigma_H<_T>::Camera_Line*)(oPose_Graph.m_pCamera_Data + oPose_Graph.m_iCamera_Data_Count);
	memset(oPose_Graph.m_pLine, 0, iCamera_Count * sizeof(Pose_Graph_Sigma_H<_T>::Camera_Line));
	int iSum = 0;

	for (i = 0; i < iMeasurement_Count; i++)
	{
		Measurement<_T> oM = pMeasurement[i];
		if (oM.m_Camera_Index[0] >= iCamera_Count || oM.m_Camera_Index[1] >= iCamera_Count)
		{
			printf("Camera Index exceed the Camera Count in Init_Pose_Graph\n");
			return;
		}
		oPose_Graph.m_pLine[oM.m_Camera_Index[0]].m_iCount++;
	}	

	typename Pose_Graph_Sigma_H<_T>::Camera_Data* pCur = oPose_Graph.m_pCamera_Data;
	typename Pose_Graph_Sigma_H<_T>::Camera_Line* poLine;
	for (i = 0; i < iCamera_Count; i++)
	{
		poLine = &oPose_Graph.m_pLine[i];
		poLine->m_pCamera_Data = pCur;
		pCur += poLine->m_iCount + 1;
		poLine->m_iCount = 0;   //后面有用
	}

	for (i = 0; i < iMeasurement_Count; i++)
	{
		Measurement<_T> oM = pMeasurement[i];
		poLine = &oPose_Graph.m_pLine[oM.m_Camera_Index[0]];
		if (poLine->m_iCount)
		{
			if (poLine->m_pCamera_Data[poLine->m_iCount - 1].m_iIndex >= oM.m_Camera_Index[1])
			{
				printf("error in Init_Pose_Graph");
				return;
			}
			poLine->m_pCamera_Data[poLine->m_iCount].m_iIndex = oM.m_Camera_Index[1];
			poLine->m_iCount++;
		}
		else
		{//第一次加到这行，连加两个
			poLine->m_pCamera_Data[0].m_iIndex = oM.m_Camera_Index[0];
			poLine->m_pCamera_Data[1].m_iIndex = oM.m_Camera_Index[1];
			if (oM.m_Camera_Index[0] >= oM.m_Camera_Index[1])
			{
				printf("error in Init_Pose_Graph");
				return;
			}
			poLine->m_iCount += 2;
		}
	}

	poLine =&oPose_Graph.m_pLine[oPose_Graph.m_iCamera_Count - 1];
	if (!poLine->m_iCount)
	{
		poLine->m_iCount = 1;
		poLine->m_pCamera_Data[0].m_iIndex = oPose_Graph.m_iCamera_Count - 1;

	}
		
	*poPose_Graph = oPose_Graph;
	return;
}
template<typename _T>void Reset_Pose_Graph(Pose_Graph_Sigma_H<_T> oSigma_H)
{
	int i;
	for (i = 0; i < oSigma_H.m_iCamera_Data_Count; i++)
		memset(oSigma_H.m_pCamera_Data[i].m_Data_6x6, 0, 6 * 6 * sizeof(_T));
	return;
}
template<typename _T>void Distribute_Data(Pose_Graph_Sigma_H<_T> oSigma_H, _T JJt[12 * 12], int iCamera_i, int iCamera_j)
{//注意对角线累加，其余直接设值
	//整个JJt劈开4部分，先搞对角线
	_T* pSource_End, * pSource_Cur;
	_T* pDest_Cur;
	int x;
	//先搞Ti数据
	pSource_Cur = JJt;
	pSource_End = pSource_Cur + 6 * 12;
	pDest_Cur = oSigma_H.m_pLine[iCamera_i].m_pCamera_Data[0].m_Data_6x6;
	while (pSource_Cur < pSource_End)
	{
		for (x = 0; x < 6; x++)
			pDest_Cur[x] += pSource_Cur[x];
		pSource_Cur += 12;
		pDest_Cur += 6;
	}

	//再搞Tj数据
	pSource_Cur = &JJt[6 * 12 + 6];
	pSource_End = pSource_Cur + 6 * 12;
	pDest_Cur = oSigma_H.m_pLine[iCamera_j].m_pCamera_Data[0].m_Data_6x6;
	while (pSource_Cur < pSource_End)
	{
		for (x = 0; x < 6; x++)
			pDest_Cur[x] += pSource_Cur[x];
		pSource_Cur += 12;
		pDest_Cur += 6;
	}

	//再搞非对角线上的Item
	pSource_Cur = &JJt[6];
	pSource_End = pSource_Cur + 6 * 12;
	typename Pose_Graph_Sigma_H<_T>::Camera_Line oLine = oSigma_H.m_pLine[iCamera_i];
	for (pDest_Cur = NULL, x = 1; x < oLine.m_iCount; x++)
	{
		if (oLine.m_pCamera_Data[x].m_iIndex == iCamera_j)
		{//找到
			pDest_Cur = oLine.m_pCamera_Data[x].m_Data_6x6;
			break;
		}
	}
	if (pDest_Cur)
	{
		while (pSource_Cur < pSource_End)
		{
			for (x = 0; x < 6; x++)
				pDest_Cur[x] = pSource_Cur[x];
			pSource_Cur += 12;
			pDest_Cur += 6;
		}
	}
	else
	{
		printf("Errror in Distribute_Data\n");
		return;
	}
	return;
}
template<typename _T>void Copy_Data_2_Sparse(Pose_Graph_Sigma_H<_T> oPose_Graph, Sparse_Matrix<_T>* poA)
{//讲oPose_Graph里的数据用稀疏矩阵表示，以便后续解方程
	int i, j, k;
	Sparse_Matrix<_T> oA = *poA;
	_T* pSource_Cur;
	typename Pose_Graph_Sigma_H<_T>::Camera_Line* poLine;
	for (i = 0; i < oPose_Graph.m_iCamera_Count; i++)
	{//最外层逐个相机
		poLine = &oPose_Graph.m_pLine[i];
		if (!poLine->m_iCount)
			continue;
		//if (i == 9)
			//printf("here");
		int iRow = i * 6;
		for (j = 0; j < 6; j++, iRow++)
		{//逐行
			oA.m_pRow[iRow] = oA.m_iCur_Item;
			//逐个Camera
			for (k = 0; k < poLine->m_iCount; k++)
			{
				typename Pose_Graph_Sigma_H<_T>::Camera_Data* poCamera_Data = &poLine->m_pCamera_Data[k];
				pSource_Cur = &poLine->m_pCamera_Data[k].m_Data_6x6[j * 6];
				typename Sparse_Matrix<_T>::Item* poItem = &oA.m_pBuffer[oA.m_iCur_Item];
				unsigned int iItem_x = poCamera_Data->m_iIndex * 6,
					iItem_y = i * 6 + j;
				if (oA.m_iCur_Item + 6 >= oA.m_iMax_Item_Count)
				{
					printf("Insufficient space in Copy_Data_2_Sparse\n");
					return;
				}
				for (int i = 0; i < 6; i++, iItem_x++)
				{
					if (pSource_Cur[i] != 0)
					{
						*poItem = { iItem_x,iItem_y,pSource_Cur[i],++oA.m_iCur_Item };
						poItem++;
					}
				}
			}
			if (oA.m_pRow[iRow] != oA.m_iCur_Item)
				oA.m_pBuffer[oA.m_iCur_Item - 1].m_iRow_Next = 0;
			else
				oA.m_pRow[iRow] = 0;
		}
	}
	Build_Link_Col_1(oA);
	//Disp_Fillness(oA);

	//以下脱离oPose_Graph，直接根据对成型抄过来
	typename Sparse_Matrix<_T>::Item* poRow_Item = NULL, * poCol_Item;
	for (i = 0; i < oA.m_iCol_Count; i++)
	{
		if (!oA.m_pCol[i])
			continue;
		poCol_Item = &oA.m_pBuffer[oA.m_pCol[i]];
		if (oA.m_pRow[i])
		{
			poRow_Item = &oA.m_pBuffer[oA.m_pRow[i]];
			if (poCol_Item->y == poRow_Item->x)
				continue;
		}
		int iOrg_Index = oA.m_iCur_Item;
		while (poCol_Item->y < (unsigned int)i && poCol_Item->y < oA.m_pBuffer[oA.m_pRow[i]].x)
		{
			//if (i == 7 && poCol_Item->y == 5)
			  // printf("here");            
			if (poCol_Item->m_fValue)
			{
				oA.m_pBuffer[oA.m_iCur_Item] = { poCol_Item->y, poCol_Item->x,poCol_Item->m_fValue,oA.m_iCur_Item + 1 };
				poRow_Item = &oA.m_pBuffer[oA.m_iCur_Item];
				oA.m_iCur_Item++;
			}
			if (poCol_Item->m_iCol_Next)
				poCol_Item = &oA.m_pBuffer[poCol_Item->m_iCol_Next];
			else
				break;
		}
		if (iOrg_Index != oA.m_iCur_Item)
		{
			oA.m_pBuffer[oA.m_iCur_Item - 1].m_iRow_Next = oA.m_pRow[i];
			oA.m_pRow[i] = iOrg_Index;
		}
	}
	Build_Link_Col(oA);
	*poA = oA;
}
template<typename _T>void TQ_2_Rt(_T TQ[7], _T Rt[4 * 4])
{//将一个se3上的数据转换为Rt. se3数据前三项为位移，后4项为4元数
	//TQ中的T表示Translation Q表示为Quaternion
	//此函数跟sophus已经完全一致。需要留意的是四元数中实部与虚部各自的位置
	_T R[3 * 3];
	Quaternion_2_Rotation_Matrix(&TQ[3], R);
	Gen_Homo_Matrix(R, TQ, Rt);
	return;
}
template<typename _T>void Gen_Cube(_T(**ppCube)[3], int* piCount, float fScale, _T x_Center, _T y_Center, _T z_Center)
{//不搞齐次坐标，只搞x,y,z坐标
	_T Cube_1[8][4],(*pCube)[3];
	Gen_Cube(Cube_1, fScale, x_Center, y_Center, z_Center);
	pCube = (_T(*)[3])pMalloc(&oMatrix_Mem, 8 * 3 * sizeof(_T));
	for (int i = 0; i < 8; i++)
		memcpy(pCube[i], Cube_1[i], 3 * sizeof(_T));
	if (ppCube)
		*ppCube = pCube;
	if (piCount)
		*piCount = 8;
	return;
}
template<typename _T> void Gen_Cube(_T Cube[][4], float fScale, _T x_Center, _T y_Center, _T z_Center)
{//先下后上，先后再前，从现在开始，面向屏幕方向为正，和以前不一样
	_T Cube_1[8][4] = { {-1,-1,1,1},	//左下后
						{1,-1,1,1},	//右下后
						{1,-1,-1,1},		//右下前
						{-1,-1,-1,1},	//左下前
						{-1,1,1,1},	//左上后
						{1,1,1,1},		//右上后
						{1,1,-1,1},		//右上前
						{-1,1,-1,1} };	//左上前
	int i;
	for (i = 0; i < 8; i++)
	{
		Cube[i][0] = Cube_1[i][0] * fScale + x_Center;
		Cube[i][1] = Cube_1[i][1] * fScale + y_Center;
		Cube[i][2] = Cube_1[i][2] * fScale + z_Center;
		Cube[i][3] = 1;
	}
	return;
}

template<typename _T>void Gen_Sphere(_T(**ppPoint_3D)[3], int* piCount, _T r ,int iStep_Count)
{//生成一个球，半径为1
// x = r*sin(beta)*cos(alpha)
// y = r*sin(beta)*sin(alpha)
// z = r*cos(beta)
	_T alpha, beta, (*pPoint_3D)[3], x, y, z, fDelta_2, r_sin_beta, fDelta_2_Start;
	int i, j, iCur_Point = 0, iStep_Count_1;
	const _T fDelta_1 = PI / iStep_Count;
	Line_1 oLine;
	pPoint_3D = (_T(*)[3])pMalloc(&oMatrix_Mem, (iStep_Count * iStep_Count/2) * 3 * sizeof(_T));

	Cal_Line(&oLine, 0.f, 1.f, (float)iStep_Count, (float)iStep_Count);
	for (i = 0; i <= (iStep_Count >> 1); i++)
	{//外层是z
		beta = i * fDelta_1;
		z = r* (_T)cos(beta);
		iStep_Count_1 = (int)fGet_Line_y(&oLine, (float)i);
		fDelta_2 = PI * 2.f / iStep_Count_1;
		r_sin_beta = r* (_T)sin(beta);
		fDelta_2_Start = (iRandom() % 100) * (3.14f / 100.f);
		for (j = 0; j < iStep_Count_1; j++)
		{
			alpha = j * fDelta_2 + fDelta_2_Start;
			x = r_sin_beta * (_T)cos(alpha);
			y = r_sin_beta * (_T)sin(alpha);
			pPoint_3D[iCur_Point][0] = x;
			pPoint_3D[iCur_Point][1] = y;
			pPoint_3D[iCur_Point][2] = z;
			iCur_Point++;
			if (z > 0)
			{
				pPoint_3D[iCur_Point][0] = x;
				pPoint_3D[iCur_Point][1] = y;
				pPoint_3D[iCur_Point][2] = -z;
				iCur_Point++;
			}
		}
		//printf("%d\n", iStep_Count_1);
	}
	Shrink(&oMatrix_Mem, pPoint_3D, iCur_Point * 3 * sizeof(_T));
	if (ppPoint_3D)
		*ppPoint_3D = pPoint_3D;
	if (piCount)
		*piCount = iCur_Point;
	return;
}
void Cal_Plane(Plane* poPlane, float a, float b, float d)
{//构造 z = ax + bx +c
	poPlane->a = a;
	poPlane->b = b;
	poPlane->d = d;
	return;
}

void Cal_Plane(Plane* poPlane, float a, float b, float c, float d)
{
	poPlane->a = a / c;
	poPlane->b = b / c;
	poPlane->d = d / c;
	return;
}
template<typename _T>_T fGet_Plane_z(Plane oPlane, _T x, _T y)
{
	return -(_T)(oPlane.a * x + oPlane.b * y + oPlane.d);
}
template<typename _T>void Cal_Plane(Plane* poPlane, _T Point[3][3])
{//用空间中三点来确定一个平面
	//列三条式的齐次方程方程，求4个未知数，
	_T A[] = { Point[0][0],Point[0][1],Point[0][2],1.f,
					Point[1][0],Point[1][1],Point[1][2],1.f,
					Point[2][0],Point[2][1],Point[2][2],1.f };
	_T B[3] = { 0 }, X[4 * 2];
	int iResult, iBasic_Solution_Count;
	Solve_Linear_Solution_Construction(A, 3, 4, B, &iResult, X, &iBasic_Solution_Count);
	poPlane->a = (float)(X[0] / X[2]);
	poPlane->b = (float)(X[1] / X[2]);
	poPlane->d = (float)(X[3] / X[2]);
	printf("%f\n", fGet_Plane_z(*poPlane, 0.f, 0.f));
	return;
}

template<typename _T>void Gen_Plane(_T(**ppPoint)[3], int* piCount, _T K[3 * 3], _T T[4 * 4])
{//这个不好，太难调
	Plane oPlane;   //构造一个面
	//Cal_Plane(&oPlane, 10, 10, 10,10);  //a,b,c,d形式
	_T(*pPoint)[3] = (_T(*)[3])pMalloc(100 * 3 * sizeof(_T));

	_T Point[3][3] = { {1,0,0},
		{0,1,0},
		{0,0,10} };
	_T Temp[4];
	Cal_Plane(&oPlane, Point);

	float x, y, z;
	int i = 0;
	for (y = -50; y < 50; y += 10)
	{
		for (x = -50; x < 50; x += 10, i++)
		{
			z = fGet_Plane_z(oPlane, x, y);
			pPoint[i][0] = x;
			pPoint[i][1] = y;
			pPoint[i][2] = z + 1000;
			if (K)
			{
				if (pPoint[i][2] < 0)
					printf("Error");
				memcpy(Temp, pPoint[i], 3 * sizeof(_T));
				Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
				Temp[0] /= Temp[2], Temp[1] /= Temp[2];
				if (Temp[0] > K[2] * 2 || Temp[1] > K[5] * 2)
					printf("Error");
			}

			if (T)
			{
				memcpy(Temp, pPoint[i], 3 * sizeof(_T));
				Temp[3] = 1.f;
				Matrix_Multiply(T, 4, 4, Temp, 1, Temp);
				Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
				Temp[0] /= Temp[2], Temp[1] /= Temp[2];
				if (Temp[0] > K[2] * 2 || Temp[1] > K[5] * 2)
					printf("Error");
			}
		}
	}
	//bSave_PLY("c:\\tmp\\1.ply", pPoint, 100);
	*ppPoint = pPoint;
	*piCount = i;

	return;
}

template<typename _T>void Gen_Plane_z0(_T(**ppPoint_3D)[3], int* piCount)
{//做一组样本，z=0
	int x, y, i;
	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(100 * 3 * sizeof(_T));
	for (i = y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++, i++)
		{
			pPoint_3D[i][0] = (_T)x;
			pPoint_3D[i][1] = (_T)y;
			pPoint_3D[i][2] = 0.f;
		}
	}
	if (ppPoint_3D)
		*ppPoint_3D = pPoint_3D;
	if (piCount)
		*piCount = i;
	return;
}
template<typename _T>void Get_K_Inv(_T K[3 * 3], _T K_Inv[3 * 3])
{
	K_Inv[2] = -K[2] / K[0];
	K_Inv[5] = -K[5] / K[4];
	K_Inv[0] = 1.f / K[0];
	K_Inv[4] = 1.f / K[4];
	K_Inv[8] = 1;
	K_Inv[1] = K_Inv[3] = K_Inv[6] = K_Inv[7] = 0;
}
template<typename _T>void Gen_Pose(_T T[4 * 4], _T v0, _T v1, _T v2, _T theta, _T t0, _T t1, _T t2, int* piResult)
{//一步生成相机位姿了事, v0,v1,v2,theta 表示一个旋转向量，t0,t1,t2表示位移
//相机首先绕(v0,v1,v2)轴绕theta弧度，然后移动到(t0,t1,t2)

	_T R[3 * 3], Rotation_Vector[] = { v0,v1,v2,theta };
	_T t[] = { t0,t1,t2 };
	int iResult;
	Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
	Gen_Homo_Matrix(R, t, T);
	Get_Inv_Matrix_Row_Op_2(T, T, 4, &iResult); //w2c
	//Disp(T, 4, 4, "T");
	if (piResult)
		*piResult = iResult;
}
template<typename _T>void Free_2_Image_Match(Image_Match_Param<_T> *poParam)
{
	if (!poParam)
		return;
	Image_Match_Param<_T> oParam = *poParam;

	//至此，三种位置全在
	Free_Image(&oParam.m_oImage_0);
	Free_Image(&oParam.m_oImage_1);
	Free_Image(&oParam.m_oImage_2);
	free(oParam.m_pImage_Point_0);
	Free(oParam.m_pNorm_Point_0);
	Free(oParam.m_pNorm_Point_1);
	Free(oParam.m_pPoint_3D_0);
	Free(oParam.m_pPoint_3D_1);
	Free_Report(oParam.m_oReport);

}
template<typename _T>void Match_2_Image(const char* pcFile_0, const char* pDepth_File_0,
	const char* pcFile_1, const char* pDepth_File_1,
	_T fDepth_Factor, //深度量化银子
	_T K[],    //内参，可以是NULL
	int* piCount,  //点对数量
	_T(**ppImage_Point_0)[2], _T(**ppImage_Point_1)[2],   //像素平面点对
	_T(**ppNorm_Point_0)[2], _T(**ppNorm_Point_1)[2],     //归一化平面点对
	_T(**ppPoint_3D_0)[3], _T(**ppPoint_3D_1)[3],          //空间点对
	Image* poImage_0, Image* poImage_1,	Image* poImage_2)
{//尝试搞个大一统，以后转入两张图就直接来

	Image oImage_0, oImage_1, oImage_2;
	unsigned short* pDepth_0=NULL, * pDepth_1=NULL;
	_T(*pPoint_0)[2], (*pPoint_1)[2];
	_T(*pNorm_Point_0)[2], (*pNorm_Point_1)[2]; //归一化平面坐标
	_T(*pPoint_3D_0)[3], (*pPoint_3D_1)[3];

	int i, j, iCount;

	//装入两个图像
	bLoad_Image(pcFile_0, &oImage_0);
	bLoad_Image(pcFile_1, &oImage_1);
	Init_Image(&oImage_2, oImage_0.m_iWidth * 2, oImage_0.m_iHeight, Image::IMAGE_TYPE_BMP, 24);
	Concat_Image(oImage_0, oImage_1, &oImage_2, 0);

	//装入图像1的深度
	if(pDepth_File_0)bLoad_Raw_Data(pDepth_File_0, (unsigned char**)&pDepth_0, NULL);
	if(pDepth_File_1)bLoad_Raw_Data(pDepth_File_1, (unsigned char**)&pDepth_1, NULL);
	Sift_Match_2_Image(pcFile_0,
		pcFile_1, &pPoint_0, &pPoint_1, &iCount, -1);

	pPoint_3D_0 = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
	pPoint_3D_1 = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));

	if(pDepth_0)Image_Pos_2_3D(pPoint_0, pDepth_0, iCount, oImage_0.m_iWidth, K, fDepth_Factor, pPoint_3D_0);
	if(pDepth_1)Image_Pos_2_3D(pPoint_1, pDepth_1, iCount, oImage_1.m_iWidth, K, fDepth_Factor, pPoint_3D_1);

	//Disp((_T*)pPoint_0_3D, iCount, 3, "Point_3D");
	//Disp((_T*)pPoint_1_3D, iCount, 3, "Point_3D");

	if (pDepth_0 && pDepth_1)
	{
		for (i = j = 0; i < iCount; i++)
		{
			if (pPoint_3D_0[i][2] != 0 && pPoint_3D_1[i][2] != 0)
			{
				pPoint_0[j][0] = pPoint_0[i][0];
				pPoint_0[j][1] = pPoint_0[i][1];
				pPoint_3D_0[j][0] = pPoint_3D_0[i][0];
				pPoint_3D_0[j][1] = pPoint_3D_0[i][1];
				pPoint_3D_0[j][2] = pPoint_3D_0[i][2];

				pPoint_1[j][0] = pPoint_1[i][0];
				pPoint_1[j][1] = pPoint_1[i][1];
				pPoint_3D_1[j][0] = pPoint_3D_1[i][0];
				pPoint_3D_1[j][1] = pPoint_3D_1[i][1];
				pPoint_3D_1[j][2] = pPoint_3D_1[i][2];
				j++;
			}
		}
		iCount = j;
	}	

	pNorm_Point_0 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	pNorm_Point_1 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	Image_Pos_2_Norm(pPoint_0, iCount, K, pNorm_Point_0);
	Image_Pos_2_Norm(pPoint_1, iCount, K, pNorm_Point_1);


	////验算一下
	//for (i = 0; i < iCount; i++)
	//{
	//    _T Temp[3];
	//    _T eps = 1e-7;
	//    Matrix_Multiply(pPoint_3D_0[i], 1, 2, 1.f / pPoint_3D_0[i][2], Temp);
	//    if (abs(Temp[0] -pNorm_Point_0[i][0])>eps || abs(Temp[1] - pNorm_Point_0[i][1])>eps)
	//        printf("err");
	//    
	//    Matrix_Multiply(pPoint_3D_1[i], 1, 2, 1.f / pPoint_3D_1[i][2], Temp);
	//    if (abs(Temp[0] - pNorm_Point_1[i][0]) > eps || abs(Temp[1] - pNorm_Point_1[i][1]) > eps)
	//        printf("err");

	//    Matrix_Multiply(K, 3, 3, pPoint_3D_0[i],1, Temp);
	//    Matrix_Multiply(Temp, 1, 2, 1.f/Temp[2], Temp);
	//    eps = 1e-4;
	//    if (abs(Temp[0] - pPoint_0[i][0]) > eps )
	//        printf("err");

	//    Matrix_Multiply(K, 3, 3, pPoint_3D_1[i], 1, Temp);
	//    Matrix_Multiply(Temp, 1, 2, 1.f / Temp[2], Temp);
	//    eps = 1e-4;
	//    if (abs(Temp[0] - pPoint_1[i][0]) > eps)
	//        printf("err");
	//}

	if (ppImage_Point_0)
		*ppImage_Point_0 = pPoint_0;
	if (ppImage_Point_1)
		*ppImage_Point_1 = pPoint_1;

	if (ppNorm_Point_0)
		*ppNorm_Point_0 = pNorm_Point_0;
	if (ppNorm_Point_1)
		*ppNorm_Point_1 = pNorm_Point_1;

	if (ppPoint_3D_0)
		*ppPoint_3D_0 = pPoint_3D_0;
	if (ppPoint_3D_1)
		*ppPoint_3D_1 = pPoint_3D_1;
	if (piCount)
		*piCount = iCount;
	if (poImage_0) *poImage_0 = oImage_0; else Free_Image(&oImage_0);
	if (poImage_1)*poImage_1 = oImage_1;else Free_Image(&oImage_1);
	if (poImage_2)*poImage_2 = oImage_2;else Free_Image(&oImage_2);
	free(pDepth_0);
	free(pDepth_1);
	return;
}

template<typename _T>void DLT(_T Point_3D_0[][3], _T Point_3D_1[][3], int iCount, _T T[])
{//估计出一个位姿，其实这个方法毫无意义，因为根本没有ICP快，也不见得有ICP准

	//_T A[12 * 12] = {};
	_T* A = (_T*)pMalloc(iCount * 3 * 12 * sizeof(_T)),
		* B = (_T*)pMalloc(iCount * 3 * sizeof(_T));
	_T H[12];
	int i, j, k, iPos_2 = 0, iResult;
	memset(A, 0, iCount * 3 * 12 * sizeof(_T));
	memset(B, 0, iCount * 3 * sizeof(_T));
	for (i = 0; i < iCount; i++, iPos_2 += 3)
	{
		int iPos_0 = i * 3 * 12;
		for (j = 0; j < 3; j++, iPos_0 += 12)
		{
			int iPos_1 = iPos_0 + j * 4;
			for (k = 0; k < 3; k++)
				A[iPos_1 + k] = Point_3D_0[i][k];
			A[iPos_1 + k] = 1;
		}
		B[iPos_2 + 0] = Point_3D_1[i][0];
		B[iPos_2 + 1] = Point_3D_1[i][1];
		B[iPos_2 + 2] = Point_3D_1[i][2];
	}
	//Disp(A, iCount, 12, "A");
	if (iCount == 4)
		Solve_Linear_Gause(A, 12, B, H, &iResult);
	else//注意，解矛盾方程组基本上数据一致，但有些许误差
		Solve_Linear_Contradictory(A, iCount * 3, 12, B, H, &iResult);	//慢就慢在这

	//Disp(H, 3, 4, "H");
	//Matrix_Multiply(A, 12, 12, H, 1, Temp);

	//还得盐酸一把
	_T Temp[4 * 4];
	_T eps = 1e-7f;
	Normalize(H, 12, H);

	//以下估计R,t
	union {
		_T Hr[3 * 3];
		_T R[3 * 3];
	};
	_T t[3] = { 0 };
	Crop_Matrix(H, 3, 4, 0, 0, 3, 3, Hr);
	//Disp(H, 3, 4, "H");
	//Disp(Hr, 3, 3, "Hr");
	Matrix_2_R(Hr, R);
	//Disp(R, 3, 3, "R");

	//剩下搞t,两种方法，方法1，自己推导
	for (i = 0; i < iCount; i++)
	{
		Matrix_Multiply(R, 3, 3, Point_3D_0[i], 1, Temp);
		Vector_Minus(Point_3D_1[i], Temp, 3, Temp);
		//Disp(Temp, 1, 3, "Diff");
		Vector_Add(t, Temp, 3, t);
	}
	t[0] /= iCount, t[1] /= iCount, t[2] /= iCount;

	////方法2，R与H有个Scale,数据差很远，但更快
	//Temp[0] = H[0], Temp[1] = H[4], Temp[2] = H[8];
	////printf("%f\n", fGet_Mod(Temp, 3));
	//_T fScale = 1.f / fGet_Mod(Temp, 3);
	////printf("H Mod %f\n", fGet_Mod(H, 12));
	//t[0] = H[3] * fScale, t[1] = H[7] * fScale, t[2] = H[11] * fScale;

	//Disp(t, 1, 3, "t");
	/*for (i = 0; i < 4; i++)
	{
		Matrix_Multiply(R, 3, 3, Point_3D_0[i], 1, Temp);
		Vector_Add(Temp, t, 3, Temp);
		Disp(Point_3D_1[i], 1, 3, "p1");
		Disp(Temp, 1, 3, "Rp0 + t");
	}*/

	Gen_Homo_Matrix(R, t, T);
	Free(A);
	Free(B);
	return;
}
template<typename _T>_T fGet_Error(_T A[], int m, int n, _T X[])
{//返回齐次矛盾方程组的误差
	_T fTotal = 0;
	int i, iPos = 0;
	for (i = 0; i < m; i++, iPos += n)
	{
		_T fValue = fDot(&A[iPos], X, n);
		fTotal += fValue*fValue;
	}
	return fTotal / m;
}
template<typename _T>void DLT_svd(_T Point_3D_0[][3], _T Point_3D_1[][3], _T Norm_Point_1[][2], int iCount, _T T[])
{//6点组成一个A矩阵
	_T* A = (_T*)pMalloc(iCount * 2 * 12 * sizeof(_T));
	int i, k, iPos_0,iResult;
	_T H[12];
	memset(A, 0, iCount * 2 * 12 * sizeof(_T));
	for (iPos_0 = i = 0; i < iCount; i++, iPos_0 += 2 * 12)
	{
		int iPos_1 = iPos_0;
		for (k = 0; k < 3; k++)
			A[iPos_1 + k] = Point_3D_0[i][k];
		A[iPos_1 + 3] = 1;

		//以下用符号
		iPos_1 = iPos_0 + 8;
		for (k = 0; k < 3; k++)
			A[iPos_1 + k] = -Point_3D_0[i][k] * Norm_Point_1[i][0];
		A[iPos_1 + 3] = -Norm_Point_1[i][0];
		//Disp(A, 1, 12, "A");

		iPos_1 = iPos_0 + 12 + 4;
		for (k = 0; k < 3; k++)
			A[iPos_1 + k] = Point_3D_0[i][k];
		A[iPos_1 + 3] = 1;

		iPos_1 = iPos_0 + 12 + 8;
		for (k = 0; k < 3; k++)
			A[iPos_1 + k] = -Point_3D_0[i][k] * Norm_Point_1[i][1];
		A[iPos_1 + 3] = -Norm_Point_1[i][1];
	}

	//求齐次矛盾方程的两种方法
	////方法1，SVD分解
	//SVD_Info oSVD;
	//SVD_Alloc(iCount * 2, 12, &oSVD, A);
	//svd_3(A, oSVD, &iResult);
	//for (i = 0; i < 12; i++)
	//	H[i] = ((_T*)oSVD.Vt)[11 * 12 + i];
	//Free_SVD(&oSVD);

	//方法2,求A'A的最小特征值对应的特征向量，用反幂法，快无数
	//这其实是一个解矛盾方程组问题，只是矛盾方程组乃齐次方程
	//基于一个理论，对A的svd分解得到的的解等价于A'A最小特征值对应的特征向量，用反幂法
	Solve_Linear_Contradictory(A, iCount * 2, 12, (_T*)NULL, H,&iResult);
			
	//此处不知需不需要给H矩阵做一次验算，如果Hp的z<0,要对整个H取反
	//然而如果只有部分z<0，怎么办？
	//还得验算一把
	_T Temp[12] = { 0 };
	//_T eps = 1e-7f;
	////Disp(H, 12, 1, "H");
	//for (i = 0; i < 6; i++)
	//{
	//	//memcpy(&Temp[i * 4], Point_3D_1[i], 3 * sizeof(_T));
	//	//Temp[i * 4 + 3] = 1;
	//	memcpy(Temp, Point_3D_0[i], 3 * sizeof(_T));
	//	Temp[3] = 1;
	//	Matrix_Multiply(H, 3, 4, Temp, 1, Temp);
	//	//实际上，H与变换阵差了一个wcale
	//	Temp[0] /= Temp[2], Temp[1] /= Temp[2];
	//	if (abs(Temp[0] - Norm_Point_1[i][0]) > eps || abs(Temp[1] - Norm_Point_1[i][1]) > eps)
	//	{
	//		/*Disp(Temp, 1, 2, "err: HP/z");
	//		Disp(Norm_Point_1[i], 1, 2, "Norm Point");*/
	//	}
	//}

	//以下估计R,t
	union {
		_T Hr[3 * 3];
		_T R[3 * 3];
	};
	_T t[3] = { 0 };
	Crop_Matrix(H, 3, 4, 0, 0, 3, 3, Hr);
	//Disp(H, 3, 4, "H");
	//Disp(Hr, 3, 3, "Hr");
	Matrix_2_R(Hr, R);
	//Disp(R, 3, 3, "R");

	//出来以后，R与H有个Scale
	Temp[0] = H[0], Temp[1] = H[4], Temp[2] = H[8];
	//printf("%f\n", fGet_Mod(Temp, 3));
	_T fScale = 1.f / fGet_Mod(Temp, 3);

	//printf("H Mod %f\n", fGet_Mod(H, 12));
	t[0] = H[3] * fScale, t[1] = H[7] * fScale, t[2] = H[11] * fScale;
	//Disp(t, 3, 1, "t");

	/*for (i = 0; i < 6; i++)
	{
		Matrix_Multiply(R, 3, 3, Point_3D_0[i], 1, Temp);
		Vector_Add(Temp, t, 3, Temp);
		Disp(Temp, 1, 3, "Temp");
		Disp(Point_3D_1[i], 1, 3, "p1");
	}*/
	/*memset(T, 0, 4 * 4 * sizeof(_T));
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
			T[i * 4 + j] = R[i * 3 + j];
		T[i * 4 + 3] = t[i];
	}
	T[15] = 1;*/
	Gen_Homo_Matrix(R, t, T);
	/* Disp(R, 3, 3, "R");
	 Disp(t, 1, 3, "t");
	 Disp(T, 4, 4, "T");*/

	Free(A);
	return;
}
template<typename _T>void Get_R_Inv(_T R[3 * 3], _T R_Inv[3 * 3])
{//利用R的正交性，转置等于求逆
 // r0  r1  r2      r0  r3  r6
//  r3  r4  r5  =>  r1  r4  r7
//  r6  r7  r8      r2  r5  r8
	if (R == R_Inv)
	{
		std::swap(R[1], R[3]);
		std::swap(R[2], R[6]);
		std::swap(R[5], R[7]);
	}else
	{
		R_Inv[0] = R[0];
		R_Inv[1] = R[3];
		R_Inv[2] = R[6];
		R_Inv[3] = R[1];
		R_Inv[4] = R[4];
		R_Inv[5] = R[7];
		R_Inv[6] = R[2];
		R_Inv[7] = R[5];
		R_Inv[8] = R[8];
	}
}
template<typename _T>void Get_T_Inv(_T T[4 * 4], _T T_Inv[4 * 4])
{//对一个位姿进行快速求逆
	union {
		_T R[3 * 3];
		_T R_Inv[3 * 3];
	};
	_T t[3];
	Get_R_t(T, R, t);
	Get_R_Inv(R, R_Inv);
	//Matrix_Multiply(R_Inv, 3, 3, t, 1, t);
	Matrix_Multiply_3x1(R_Inv, t, t);

	t[0] = -t[0], t[1] = -t[1], t[2] = -t[2];
	Gen_Homo_Matrix(R_Inv, t, T_Inv);
}
template<typename _T>void Test_Triangulate(_T Point_3D[][3], _T Norm_Point_0[][2], _T Norm_Point_1[][2], int iCount, _T T[4 * 4], _T* pfError)
{//计算两个相机经过三角化后的平均误差
	_T fError = 0, * pPoint_3D;
	int i;
	for (i = 0; i < iCount; i++)
	{
		_T Temp[4];
		pPoint_3D = Point_3D[i];
		Temp[0] = pPoint_3D[0] / pPoint_3D[2];
		Temp[1] = pPoint_3D[1] / pPoint_3D[2];

		fError += fGet_Distance(Temp, Norm_Point_0[i], 2);
	}
	//printf("fError:%f\n", fError);

	for (i = 0; i < iCount; i++)
	{
		pPoint_3D = Point_3D[i];
		_T Temp[4] = { pPoint_3D[0], pPoint_3D[1], pPoint_3D[2], 1 };
		Matrix_Multiply(T, 4, 4, Temp, 1, Temp);
		Temp[0] = Temp[0] / Temp[2];
		Temp[1] = Temp[1] / Temp[2];
		fError += fGet_Distance(Temp, Norm_Point_1[i], 2);
	}
	fError /= iCount;
	if(pfError)
		*pfError = fError;
	//printf("fError:%f\n", fError);
}
template<typename _T>void Estimate_2_Image_T(const char* pcFile_0, const char* pcFile_1, _T K[3 * 3], Image_Match_Param<_T>* poParam)
{//给定连个文件的路径，直接估计出个位姿
	int iCount;
	Image_Match_Param<_T> oParam = *poParam;

	Match_2_Image(pcFile_0, (const char*)NULL,
		pcFile_1, (const char*)NULL,
		(_T)5000.f, K, &iCount,
		&oParam.m_pImage_Point_0, &oParam.m_pImage_Point_1,
		&oParam.m_pNorm_Point_0, &oParam.m_pNorm_Point_1,
		&oParam.m_pPoint_3D_0, &oParam.m_pPoint_3D_1, &oParam.m_oImage_0, &oParam.m_oImage_1, &oParam.m_oImage_2);

	Ransac_Report oReport;

	Ransac_Estimate_E(oParam.m_pImage_Point_0, oParam.m_pImage_Point_1, iCount,(float)((abs(K[0]) + abs(K[4])) / 2.f), (float)K[2], (float)K[5], &oReport);
	Shrink_Match_Point(&oParam.m_pImage_Point_0, &oParam.m_pImage_Point_1, oReport.m_pInlier_Mask, iCount);
	iCount = oReport.m_oSupport.m_iInlier_Count;

	//Ransac_Estimate_H(oParam.m_pImage_Point_0, oParam.m_pImage_Point_1, iCount, &oReport,&oMatrix_Mem);

	//Recalculate Normpoint
	Image_Pos_2_Norm(oParam.m_pImage_Point_0, iCount, K, oParam.m_pNorm_Point_0);
	Image_Pos_2_Norm(oParam.m_pImage_Point_1, iCount, K, oParam.m_pNorm_Point_1);

	/*_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
	if (!pPoint_3D)
		return;*/

	_T T[4 * 4], R[3 * 3], t[3];
	unsigned char* pInlier_Mask = oReport.m_pInlier_Mask;
	
	E_2_R_t((_T*)oReport.m_Modal, oParam.m_pNorm_Point_0, oParam.m_pNorm_Point_1, iCount, R, t, oParam.m_pPoint_3D, &oParam.m_iMatch_Count,pInlier_Mask);
	Gen_Homo_Matrix(R, t, T);
	if (iCount != oParam.m_iMatch_Count && oParam.m_iMatch_Count)
	{//此处还得搞搞，必须重新查一遍，看看那些点符合条件
		int i, j;
		for (i = 0, j = 0; i < iCount; i++)
		{
			if (pInlier_Mask[i])
			{
				memcpy(oParam.m_pNorm_Point_0[j], oParam.m_pNorm_Point_0[i], 2 * sizeof(_T));
				memcpy(oParam.m_pNorm_Point_1[j], oParam.m_pNorm_Point_1[i], 2 * sizeof(_T));
				j++;
			}	
		}
		if (j != oParam.m_iMatch_Count)
			printf("err");
		iCount = oParam.m_iMatch_Count;
	}

	//Disp(T, 4, 4, "T");
	//oParam.m_pPoint_3D = pPoint_3D;
	oParam.m_oReport = oReport;
	oParam.m_iMatch_Count = oParam.m_iMatch_Count;
	memcpy(oParam.K, K, 9 * sizeof(_T));
	memcpy(oParam.T, T, 4 * 4 * sizeof(_T));
	*poParam = oParam;
}
template<typename _T>void Disp_T(_T T[4 * 4])
{
	_T R[3 * 3], t[3];
	Get_R_t(T, R, t);
	Disp(T, 4, 4, "T");
	printf("Is rotation Matrix:%d\n", bIs_R(R));
	_T V[4];
	Rotation_Matrix_2_Vector(R, V);
	Disp(V, 1, 4, "Rotation_Vector");
	return;
}



template<typename _T>void Init_LM_Param(LM_Param_g2o<_T>* poParam)
{
	LM_Param_g2o<_T> oParam;
	oParam.m_iIter = 0;
	oParam.m_bStop = 0;
	oParam.Lamda = -1.f;
	oParam.m_fLoss = 0;
	*poParam = oParam;
	
	return;
}
template<typename _T>void Solve_Linear_LM(_T A[],int iOrder, _T b[],  _T x[],int *pbResult,LM_Param_g2o<_T> *poParam)
{//用Levenberg Marquart解方程，此处要迭代
	int i, bResult;
	LM_Param_g2o<_T> oParam = *poParam;
	const int iMax_Size = 256;
	_T Buffer[iMax_Size],
		*f = Buffer;

	//Disp(b, 1, 3, "b");
	if (oParam.m_iIter == 0)
	{//第一次，特殊对待,取对角线最大值		
		_T tau = (_T)1e-5, fMax = (_T)0.f;
		for (i = 0; i < iOrder; i++)
			if (Abs(A[i * iOrder + i]) > fMax)
				fMax = Abs(A[i * iOrder + i]);
		//第一次迭代的时候，Lamda定位 0.00001*对角线最大元
		oParam.Lamda = tau * fMax;
		oParam._ni = 2;
	}

	_T rho = 0;
	int	qmax = 0;
	/*if (oParam.m_iIter == 13)
		printf("Here");*/

	do {
		//解分方程看看
		Add_I_Matrix(A, iOrder, oParam.Lamda);
		Solve_Linear_Gause(A, iOrder, b, x, &bResult);
		//Disp(A, iOrder, iOrder, "H");

		//恢复方程
		Add_I_Matrix(A, iOrder, -oParam.Lamda);

		//将Delta_x加到原来的X上
		Vector_Add(oParam.m_pOrg_X, x, iOrder, oParam.m_pOrg_X);
		
		//重新算一次误差
		_T fSum_e = 0;
		for (i = 0; i < oParam.m_iSample_Count; i++)
		{
			int iPos_Sample_In = i * oParam.m_iIn_Dim,
				iPos_Y = i * oParam.m_iOut_Dim;
			oParam.Get_J(&oParam.m_pSample_In[iPos_Sample_In], &oParam.Y[iPos_Y], NULL, oParam.m_pOrg_X, f);
			for (int j = 0; j < oParam.m_iOut_Dim; j++)
			{
				f[j] -= oParam.Y[iPos_Y + j];
				fSum_e += f[j] * f[j];
			}				
		}
		
		if (!bResult)
			fSum_e = std::numeric_limits<_T>::max();
		//rho不过是个表示发散/收敛程度的参数
		rho = (oParam.m_fLoss - fSum_e);

		//搞个scale
		_T fScale;
		if (bResult)
		{
			fScale = (_T)1e-3;
			for (int i = 0; i < iOrder; i++)
				fScale += x[i] * (oParam.Lamda * x[i] + b[i]);
		}else
			fScale = 1;
		rho /= fScale;

		if (rho > 0 && std::_Is_finite(fSum_e) && bResult)
		{//本次迭代的最后一次
			_T alpha = (_T)(1. - pow((2 * rho - 1), 3));
			alpha = Min(alpha, 2.f/3.f);
			_T scaleFactor = Max(1.f/3.f, alpha);
			//可见，scaleFactor在[1/3,2/3]之间，Lamda必然缩小
			oParam.Lamda *= scaleFactor;
			oParam._ni = 2;
			oParam.m_fLoss = fSum_e;
		}else
		{//如果方向不对发散了，则增大Lamda
			oParam.Lamda*=oParam._ni;
			oParam._ni *= 2;
			if (!std::_Is_finite(oParam.Lamda)) 
				break;
		}
		//此处要恢复一下方程
		Vector_Minus(oParam.m_pOrg_X, x, iOrder, oParam.m_pOrg_X);

		qmax++;
	} while (rho < 0  && qmax<10 && !poParam->m_bStop);	//此处本来还有个外部干涉信号可控制停止
	*poParam = oParam;
	//printf("QMax:%d\n", qmax);
	return;
}
template<typename _T>void Bundle_Adjust(_T Sample_In[], const int iIn_Dim, _T Y[], const int iOut_Dim, int iSample_Count, _T X[], int iParam_Count, _T eps,int iMax_Iter_Count,
	void(*Get_J)(_T*, _T*, _T*, _T*, _T*,...),...)
{//试图做一个统一的格式，避免每次都调一顿
	//雅可比函数原型：template<typename _T>void Get_J_1(_T Sample_In[], _T Y[], _T J[],_T X[],_T *g)
	//其中X[]指的是待优化参数，g为 f(x) = g(x) - yi中的g(x)
	//约定 f(x) = g(x) - yi,  否则会影响后面的-J'f
	//限制，该方法只适用于显式求解，不适用于扰动方程这类曲里拐弯的解法
	int i, iIter, iResult,
		iJ_Size = iOut_Dim * iParam_Count,
		iH_Size = iParam_Count * iParam_Count;

	const int iMax_Size = 256;
	_T Buffer[iMax_Size];
	_T* J = Buffer, * Jt = J + iJ_Size,
		* H = Jt + iJ_Size,
		* Temp = H + iH_Size,
		* Jtf = Temp + iH_Size,
		* Delta_X = Jtf + iParam_Count,
		* f = Delta_X + iParam_Count;
	//Jt = Temp;
	if (f + iOut_Dim - Buffer >= iMax_Size)
	{
		printf("Insufficient buffer size:%lld", f + iOut_Dim - Buffer);
		return;
	}
	_T fSum_e, fSum_e_Pre = 0;	
	LM_Param_g2o<_T> oLM_Param;
	Init_LM_Param(&oLM_Param);
	oLM_Param.Get_J = Get_J;
	oLM_Param.m_pSample_In = Sample_In;
	oLM_Param.Y = Y;
	oLM_Param.m_iSample_Count = iSample_Count;
	oLM_Param.m_iIn_Dim = iIn_Dim;
	oLM_Param.m_iOut_Dim = iOut_Dim;

	for (iIter = 0;iIter<iMax_Iter_Count; iIter++)
	{
		fSum_e = 0;
		memset(H, 0, iH_Size * sizeof(_T));
		memset(Jtf, 0, iJ_Size * sizeof(_T));
		//_T fSum_Grad = 0;
		for (i = 0; i < iSample_Count; i++)
		{
			int iPos_Sample_In = i * iIn_Dim,
				iPos_Y = i * iOut_Dim;
			/*if (iIter == 1 && i == iSample_Count - 1)
				printf("Here");*/
			Get_J(&Sample_In[iPos_Sample_In], &Y[iPos_Y], J, X, f);
			//Disp(J, iOut_Dim, iParam_Count, "J");
			//计算J'J
			/*for (int k = 0; k < 4; k++)
				fSum_Grad += fGet_Mod(&J[k * 4], 4);*/
			Transpose_Multiply(J, iOut_Dim, iParam_Count, Temp, 0);
			//Disp(Temp, 4, 4, "H");

			//累加到H
			Matrix_Add(H, Temp, iParam_Count, H);
			for (int j = 0; j < iOut_Dim; j++)
			{
				f[j] -= Y[iPos_Y + j];
				fSum_e += f[j] * f[j];
				/*if(iIter==1)
					printf("%f\n", f[j]);*/
			}
			Matrix_Transpose(J, iOut_Dim, iParam_Count, Jt);
			Matrix_Multiply(Jt, iParam_Count, iOut_Dim, f, 1, Temp);
			Vector_Add(Jtf, Temp, iParam_Count, Jtf);
			//Disp(Jft, iParam_Count, 1, "Jft");
		}
		//Disp(H, iParam_Count, iParam_Count, "H");
		//Disp(Jtf, iParam_Count, 1, "Jft");

		Matrix_Multiply(Jtf, iParam_Count, 1, (_T)-1.f, Jtf);
		//Disp(H, iParam_Count, iParam_Count, "H");

		////高斯法，不靠谱，经常发散
		//Solve_Linear_Gause(H, iParam_Count, Jtf, Delta_X, &iResult);

		//LM法，更靠谱
		oLM_Param.m_iIter = iIter, oLM_Param.m_fLoss = fSum_e, oLM_Param.m_pOrg_X = X;
		Solve_Linear_LM(H, iParam_Count, Jtf, Delta_X, &iResult,&oLM_Param);
		fSum_e = oLM_Param.m_fLoss;
		printf("Lamda:%f\n", oLM_Param.Lamda);

		//Disp(Delta_X, 1, 3, "Delta");
		Vector_Add(X, Delta_X, iParam_Count, X);
		printf("Iter:%d loss: %e\n",iIter, fSum_e);
		if (Abs(fSum_e_Pre - fSum_e) < eps || fSum_e<eps)
			break;
		fSum_e_Pre = fSum_e;
	}
	return;
}
void Disp_Ransac_Report(Ransac_Report oReport)
{
	if (oReport.m_bSuccess)
		printf("Ransac succeed\n");
	else
	{
		printf("Ransac fail\n");
		return;
	}
	printf("Sample Count:%d Inlier:%d\n",oReport.m_iSample_Count,oReport.m_oSupport.m_iInlier_Count);
	printf("Error Sum:%f\n", oReport.m_oSupport.m_fResidual_Sum);
	return;
}
template<typename _T>void Test_T(_T Point_3D[][3], _T Norm_Point_0[][2], _T T0[4*4], int iCount)
{
	int i;
	_T fTotal_0 = 0, Temp[4];
	for (i = 0; i < iCount; i++)
	{
		memcpy(Temp, Point_3D[i], 3 * sizeof(_T));
		Temp[3] = 1;
		Matrix_Multiply(T0, 4, 4, Temp, 1, Temp);
		Temp[0] /= Temp[2];
		Temp[1] /= Temp[2];
		//Vector_Minus(Temp, Norm_Point_0[i], 2, Temp);
		//fTotal_0 += fGet_Mod(Temp, 2);
		fTotal_0 += fGet_Distance(Temp, Norm_Point_0[i],2);
	}
	printf("Norm Point 0 Error Sum:%f Avg Error:%f\n", fTotal_0,fTotal_0/iCount);
	return;
}

template<typename _T>void Get_Deriv_E_P(_T K[3 * 3], _T Camera[4*4], _T TP[3], _T J[2 * 6])
{//求de/dP = de/dP' * dP'/dP, 其中P'=TP

	_T R[3 * 3];
	_T J_UV_TP[2 * 3];  //dE/dP      

	Get_R_t(Camera, R);
	Get_Drive_UV_P(K[0], K[4], TP, J_UV_TP);
	Matrix_Multiply(J_UV_TP, 2, 3, R, 3, J);
}

template<typename _T>void bSave_PLY(const char* pcFile, Point_Cloud<_T> oPC)
{
	bSave_PLY(pcFile, oPC.m_pPoint, oPC.m_iCount, oPC.m_pColor);
}
template<typename _T>void Init_Point_Cloud(Point_Cloud<_T>* poPC,int iMax_Count,int bHas_Color)
{
	int iSize;
	Point_Cloud<_T> oPC = {};
	oPC.m_iMax_Count = iMax_Count;
	oPC.m_bHas_Color = bHas_Color;
	iSize = iMax_Count * 3 * sizeof(_T);
	if (bHas_Color)
		iSize += ALIGN_SIZE_128(iMax_Count) + iMax_Count * 3;

	oPC.m_pBuffer = (unsigned char*)pMalloc(iSize);
	iSize = iMax_Count;
	oPC.m_pPoint = (_T(*)[3])oPC.m_pBuffer;
	oPC.m_pColor = (unsigned char(*)[3])(oPC.m_pPoint + iMax_Count);
	oPC.m_pColor = (unsigned char(*)[3])ALIGN_ADDR_128(oPC.m_pColor);
	memset(oPC.m_pPoint, 0, iMax_Count * 3 * sizeof(_T));
	memset(oPC.m_pColor, 255, iMax_Count * 3);

	*poPC = oPC;
	return;
}
template<typename _T>void Free_Point_Cloud(Point_Cloud<_T>* poPC)
{
	if (poPC->m_pBuffer)
		Free(poPC->m_pBuffer);
	*poPC = {};
}
template<typename _T>void Draw_Point(Point_Cloud<_T>* poPC, _T x, _T y, _T z, int R, int G, int B)
{
	Point_Cloud<_T> oPC = *poPC;
	if (oPC.m_iCount + 1 > oPC.m_iMax_Count)
	{
		printf("Insufficient memroy");
		return;
	}
	_T* pPoint = oPC.m_pPoint[oPC.m_iCount];
	pPoint[0] = x;
	pPoint[1] = y;
	pPoint[2] = z;

	if (oPC.m_pColor)
	{
		unsigned char* pColor = oPC.m_pColor[oPC.m_iCount];
		pColor[0] = R, pColor[1] = G, pColor[2] = B;
	}

	oPC.m_iCount++;
	*poPC = oPC;
	return;
}
template<typename _T>void Draw_Sphere(Point_Cloud<_T>* poPC, _T x, _T y, _T z,_T r, int iStep_Count,int R,int G,int B)
{
	Point_Cloud<_T> oPC = *poPC;
	_T (*pPoint_3D)[3];
	int i,iCount,iPos;
	Gen_Sphere(&pPoint_3D, &iCount, r,iStep_Count);
	if (oPC.m_iCount + iCount > oPC.m_iMax_Count)
	{
		printf("Insufficient memroy");
		return;
	}
	iPos = oPC.m_iCount;
	for (i = 0; i < iCount; i++,iPos++)
	{
		_T* pPoint_3D_1 = oPC.m_pPoint[iPos];
		unsigned char* pColor = oPC.m_pColor[iPos];
		memcpy(pPoint_3D_1, pPoint_3D[i], 3 * sizeof(_T));
		pPoint_3D_1[0] += x;
		pPoint_3D_1[1] += y;
		pPoint_3D_1[2] += z;
		pColor[0] = R, pColor[1] = G, pColor[2] = B;
	}
	//memcpy(oPC.m_pPoint + oPC.m_iCount, pPoint_3D, iCount * 3*sizeof(_T));
	oPC.m_iCount += iCount;
	*poPC = oPC;
	Free(pPoint_3D);
	return;
}
template<typename _T>void Draw_Line(Point_Cloud<_T>* poPC, _T x0, _T y0, _T z0, _T x1, _T y1, _T z1, int iCount, int R, int G, int B)
{
	int i;
	for (i = 0; i < iCount; i++)
	{
		_T Pos[3] = {x0+ (x1 - x0) * i / iCount,
			y0+ (y1 - y0) * i / iCount,
			z0+(z1 - z0) * i / iCount };
		Draw_Point(poPC, Pos[0], Pos[1], Pos[2], R, G, B);
	}
	return;
}
template<typename _T>void Draw_Camera(Point_Cloud<_T>* poPC, _T T[4 * 4],int R,int G,int B)
{//看看能否画出个简陋的相机位姿
	_T View_Point[4] = { T[3],T[7],T[11],1};    //此处应该算是视点
	_T Norm_Plane[4][4] = { {-1,1,-1,1 },
		{1,1,-1,1},
		{1,-1,-1,1},
		{-1,-1,-1,1} };    //归一化平面上的4个点
	_T Norm_Center[4] = { 0,0,-1,1 };      //归一化平面上的中心
	//_T Temp[4];
	int i;
	//剩下的一律从原地搬过去
	//Draw_Sphere(poPC, View_Point[0], View_Point[1], View_Point[2],(_T)0.2f,40,255,0,0);
	Draw_Point(poPC, View_Point[0], View_Point[1], View_Point[2],R,G,B);

	Matrix_Multiply(T, 4, 4, Norm_Center, 1, Norm_Center);
	Draw_Line(poPC, View_Point[0], View_Point[1], View_Point[2], Norm_Center[0], Norm_Center[1], Norm_Center[2],50,R,G,B);

	for (i = 0; i < 4; i++)
	{
		Matrix_Multiply(T, 4, 4, Norm_Plane[i], 1, Norm_Plane[i]);
		Draw_Line(poPC, View_Point[0], View_Point[1], View_Point[2], Norm_Plane[i][0], Norm_Plane[i][1], Norm_Plane[i][2],50,R,G,B);
	}
	for (i = 0; i < 4; i++)
		Draw_Line(poPC, Norm_Plane[i][0], Norm_Plane[i][1], Norm_Plane[i][2], Norm_Plane[(i + 1) & 3][0], Norm_Plane[(i + 1) & 3][1], Norm_Plane[(i + 1) & 3][2], 50, R, G, B);

	Draw_Line(poPC, Norm_Plane[0][0], Norm_Plane[0][1], Norm_Plane[0][2], Norm_Plane[2][0], Norm_Plane[2][1], Norm_Plane[2][2], 50, R, G, B);
	Draw_Line(poPC, Norm_Plane[1][0], Norm_Plane[1][1], Norm_Plane[1][2], Norm_Plane[3][0], Norm_Plane[3][1], Norm_Plane[3][2], 50, R, G, B);

	return;
}
template<typename _T>void Draw_Image(Point_Cloud<_T>* poPC, Image oImage, _T T[4 * 4],_T K[3*3],int iCount)
{//将图片画在像素平面上，中心对准(0,0,-1)
 //iCount, 一共画多少点
	Point_Cloud<_T> oPC = *poPC;
	if (oPC.m_iCount + iCount > oPC.m_iMax_Count)
	{
		printf("Insufficient space in Draw_Image\n");
		return;
	}
	int iSize = oImage.m_iWidth * oImage.m_iHeight;
	unsigned char* pMap = (unsigned char*)pMalloc(iSize);
	memset(pMap, 0, iSize);
	int i;
	for(i=0;i<iCount;)
	{
		int iPos = iGet_Random_No() % iSize;
		if (!pMap[iPos])
			pMap[iPos] = 1, i++;
		else
			printf("*");
	}
	int x, y;
	for (i = y = 0; y < oImage.m_iHeight; y++)
	{
		for (x = 0; x < oImage.m_iWidth; x++, i++)
		{
			if (pMap[i])
			{
				unsigned char RGB[3] = { oImage.m_pChannel[0][i], oImage.m_pChannel[1][i], oImage.m_pChannel[2][i] };
				_T Norm_Pos[] = { (x - K[2]) / K[0],(y - K[5]) / K[4],-1,1 };
				if (T)
					Matrix_Multiply(T, 4, 4, Norm_Pos, 1, Norm_Pos);

				Draw_Point(&oPC, Norm_Pos[0], Norm_Pos[1], Norm_Pos[2], RGB[0], RGB[1], RGB[2]);
			}
		}
	}
	//bSave_PLY("c:\\tmp\\1.ply", oPC);
	Free(pMap);
	*poPC = oPC;
	return;
}

template<typename _T>void Get_K_Inv_With_gama(_T K[], _T K_Inv[])
{//对内参快速求逆
	_T fx = K[0], fy = K[4],
		cx = K[2], cy = K[5],
		gama = K[1];
	K_Inv[0] = 1.f / fx;
	K_Inv[4] = 1.f / fy;
	K_Inv[1] = -gama / (fx * fy);
	K_Inv[2] = gama * cy / (fx * fy) -cx / fx  ;
	K_Inv[5] = -cy / fy;
	K_Inv[3] = K_Inv[6] =K_Inv[7] = 0;
	K_Inv[8] = 1;
	return;
}

template<typename _T>void Undistort_uv(_T TP[3], _T K[3*3], _T Dist_Coeff[], int iCoeff_Count, _T uv[2])
{//对TP进行带畸变投影
 //第一步，投影到归一化平面
	_T TP_1[2] = { TP[0] / TP[2], TP[1] / TP[2] };

	_T r2 = TP_1[0] * TP_1[0] + TP_1[1] * TP_1[1],
		r4 = r2 * r2,
		r6 = r4 * r2;
	//前三个为径向畸变参数
	_T d1 = 1 + r2 * Dist_Coeff[0] + r4 * Dist_Coeff[1] + r6*Dist_Coeff[2];
	//后两个为切向畸变
	_T d2_x = 2 * Dist_Coeff[3] * TP_1[0] * TP_1[1] + Dist_Coeff[4] * (r2 + 2 * TP_1[0] * TP_1[0]),
		d2_y = Dist_Coeff[3] * (r2 + 2 * TP_1[1] * TP_1[1]) + 2*Dist_Coeff[4] * TP_1[0] * TP_1[1];

	//u = fx * d * x + cx,
	uv[0] = K[0] * (d1 * TP_1[0] + d2_x) + K[2];
	//v = fy * d * y + cy;
	uv[1] = K[4] * (d1 * TP_1[1] + d2_y) + K[5];

	return;
}