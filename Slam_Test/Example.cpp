//搞个临时文件，想搞明白Essential Matrix
#include "stdio.h"
#include "stdarg.h"
#include "Image.h"
#include "Matrix.h"
#include "Reconstruct.h"
#include "iostream"
using namespace std;

template<typename _T> int bTemp_Load_Data(const char* pcFile, _T(**ppT)[7], int* piPoint_Count,
	Measurement<_T>** ppMeasurement, int* piMeasure_Count);

void SB_Sample()
{
	NCC(Image{}, Image{}, 0, 0, (double)0.f, (double)0.f);
	NCC(Image{}, Image{}, 0, 0, (float)0.f, (float)0.f);

	Normalize((double*)NULL, 0, (double*)NULL);
	Normalize((float*)NULL, 0, (float*)NULL);

	bTemp_Load_Data(NULL, (double(**)[16])NULL, NULL, (double**)NULL);
	bTemp_Load_Data(NULL, (float(**)[16])NULL, NULL, (float**)NULL);

	bTemp_Load_Data(NULL, (double(**)[7])NULL, NULL, (Measurement<double>**)NULL, NULL);
	bTemp_Load_Data(NULL, (float(**)[7])NULL, NULL, (Measurement<float>**)NULL, NULL);

	Temp_Load_File(NULL, (double(**)[3])NULL, (double(**)[3])NULL, NULL);
	Temp_Load_File(NULL, (float(**)[3])NULL, (float(**)[3])NULL, NULL);

	Temp_Load_File_1(NULL, (double(**)[3])NULL, (double(**)[3])NULL, NULL);
	Temp_Load_File_1(NULL, (float(**)[3])NULL, (float(**)[3])NULL, NULL);
	
}

//static void Test_2()
//{//位姿图优化，最简形式
//	typedef float _T;
//	_T(*pKsi)[7];
//	Measurement<_T>* pMeasurement;
//	int i, j, iIter, iResult, iPoint_Count, iMeasurement_Count;
//	union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
//	union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
//	union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
//
//	_T E_6[6], E_4x4[4 * 4], //E可以是一个矩阵
//		fSum_e, fSum_e_Pre = (_T)1e10;
//	_T J_Inv[6 * 6], Adj[6 * 6], Temp[6 * 6], Delta_Pose[4 * 4];
//	_T* Jt, * J, * H, * JEt, * Sigma_H, * Sigma_JEt, * Delta_X;
//
//	_T(*Camera)[4 * 4];
//	iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iPoint_Count, &pMeasurement, &iMeasurement_Count);
//	if (!iResult)return;
//	int iAdjust_Count = 10;
//
//	Jt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
//	J = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
//	H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
//	JEt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
//	Sigma_JEt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
//	Delta_X = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
//	Sigma_H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
//	Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iAdjust_Count * 4 * 4 * sizeof(_T));
//	for (i = 0; i < iAdjust_Count; i++)
//		TQ_2_Rt(pKsi[i], Camera[i]);
//
//	for (iIter = 0;; iIter++)
//	{
//		fSum_e = 0;
//		memset(Sigma_H, 0, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
//		memset(Sigma_JEt, 0, iAdjust_Count * 6 * sizeof(_T));
//		for (i = 0; i < iAdjust_Count - 1; i++)
//		{//没一个测量都带来一个调整权重
//			Measurement<_T> oM = pMeasurement[i];
//			//_T *ksi_i = pKsi[oM.m_Pose_Index[0]],
//				//*ksi_j = pKsi[oM.m_Pose_Index[1]];
//			//TQ_2_Rt(ksi_i, Ti);
//			//TQ_2_Rt(ksi_j, Tj);
//			memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
//			memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
//			TQ_2_Rt(oM.Delta_ksi, M_4x4);
//			Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
//			if (!iResult)break;
//			Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
//			if (!iResult)break;
//
//			Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);
//			Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);
//			SE3_2_se3(E_4x4, E_6);
//
//			//Disp(E_6, 6, 1, "E_6");
//			for (j = 0; j < 6; j++)
//				fSum_e += E_6[j] * E_6[j];
//
//			memset(Jt, 0, iAdjust_Count * 36 * sizeof(_T));
//
//			//接着求雅可比
//			Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
//			Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
//			if (!iResult)break;
//			Get_Adj(Tj_Inv, Adj);
//
//			//此时形成两个雅可比
//			//第一个雅可比Jt
//			Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
//			//∂eij/∂ξj
//			Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[1] * 6, 0);
//
//			//∂eij/∂ξi
//			Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
//			//Transpose_Multiply(Temp, 6, 6, J1t,0);
//			Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[0] * 6, 0);
//
//			//printf("%d %d\n", oM.m_Pose_Index[0], oM.m_Pose_Index[1]);
//			Matrix_Transpose(Jt, 6, iAdjust_Count * 6, J);
//
//			Matrix_Multiply(J, iAdjust_Count * 6, 6, Jt, iAdjust_Count * 6, H);
//			Matrix_Add(Sigma_H, H, iAdjust_Count * 6, Sigma_H);         //∑H, JJ'到位
//			Matrix_Multiply(J, iAdjust_Count * 6, 6, E_6, 1, JEt);      //JE'到位
//
//			//Disp(JEt, iAdjust_Count * 6, 1);
//			Vector_Add(Sigma_JEt, JEt, iAdjust_Count * 6, Sigma_JEt);
//		}
//		printf("%f\n", fSum_e);
//
//		Add_I_Matrix(Sigma_H, iAdjust_Count * 6);
//		Solve_Linear_Gause(Sigma_H, iAdjust_Count * 6, Sigma_JEt, Delta_X, &iResult);
//		Matrix_Multiply(Delta_X, 1, iAdjust_Count * 6, (_T)-1, Delta_X);
//		if (fSum_e_Pre <= fSum_e || !iResult)
//			break;
//
//		for (i = 0; i < iAdjust_Count; i++)
//		{
//			se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
//			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
//		}
//		fSum_e_Pre = fSum_e;
//		printf("iIter:%d %.10f\n", iIter, fSum_e);
//	}
//	return;
//}
//
//static void Test_4()
//{//位姿图优化，试一下信息矩阵，无卵用
//	typedef float _T;
//	_T(*pKsi)[7];
//	Measurement<_T>* pMeasurement;
//	int i, j, iIter, iResult, iPoint_Count, iMeasurement_Count;
//	union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
//	union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
//	union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
//
//	_T E_6[6], E_4x4[4 * 4], //E可以是一个矩阵
//		fSum_e, fSum_e_Pre = (_T)1e10;
//	_T J_Inv[6 * 6], Adj[6 * 6], Temp[6 * 6], Delta_Pose[4 * 4];
//	_T* Jt, * J, * Jt_Sigma, * Jt_Sigma_E, * H, * Sigma_H, * Sigma_Jt_Sigma_E, * Delta_X;
//
//	_T(*Camera)[4 * 4];
//	iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iPoint_Count, &pMeasurement, &iMeasurement_Count);
//	if (!iResult)return;
//	int iAdjust_Count = 200;
//
//	Jt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
//	J = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
//	H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
//	Jt_Sigma = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
//	Jt_Sigma_E = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
//	Sigma_Jt_Sigma_E = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
//
//	Delta_X = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
//	Sigma_H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
//	Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iAdjust_Count * 4 * 4 * sizeof(_T));
//	for (i = 0; i < iAdjust_Count; i++)
//		TQ_2_Rt(pKsi[i], Camera[i]);
//	for (iIter = 0;; iIter++)
//	{
//		fSum_e = 0;
//		memset(Sigma_H, 0, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
//		memset(Sigma_Jt_Sigma_E, 0, iAdjust_Count * 6 * sizeof(_T));
//
//		for (i = 0; i < iAdjust_Count - 1; i++)
//		{//没一个测量都带来一个调整权重
//			Measurement<_T> oM = pMeasurement[i];
//			memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
//			memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
//			TQ_2_Rt(oM.Delta_ksi, M_4x4);
//			Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
//			if (!iResult)break;
//			Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
//			if (!iResult)break;
//			//Disp(M_Inv_4x4, 4, 4, "M(-1)");
//			//Disp(Ti_Inv, 4, 4, "Ti_Inv");
//			Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);     //=Tij(-1) * Ti(-1)
//			Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);             //=Tij(-1) * Ti(-1) * Tj
//			SE3_2_se3(E_4x4, E_6);  //注意，书上是错的，正确的是eij = ln(Tij(-1) * Ti(-1) * Tj)
//			for (j = 0; j < 6; j++)
//				fSum_e += E_6[j] * E_6[j];
//
//			memset(Jt, 0, iAdjust_Count * 36 * sizeof(_T));
//
//			//接着求雅可比
//			Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
//			Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
//			if (!iResult)break;
//			Get_Adj(Tj_Inv, Adj);
//
//
//			//此时形成两个雅可比
//			//第一个雅可比Jt
//			Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
//			//∂eij/∂ξj
//			Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[1] * 6, 0);
//
//			//∂eij/∂ξi
//			Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
//			//Transpose_Multiply(Temp, 6, 6, J1t,0);
//			Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[0] * 6, 0);
//
//			//printf("%d %d\n", oM.m_Pose_Index[0], oM.m_Pose_Index[1]);
//			Matrix_Transpose(Jt, 6, iAdjust_Count * 6, J);
//
//			//求 J * Z(-1)* Jt    
//			_T JtJ[6 * 6];
//			Matrix_Multiply(Jt, 6, iAdjust_Count * 6, J, 6, JtJ);
//			Matrix_Multiply(J, iAdjust_Count * 6, 6, JtJ, 6, Jt_Sigma);
//
//			Matrix_Multiply(Jt_Sigma, iAdjust_Count * 6, 6, Jt, iAdjust_Count * 6, H);
//			Matrix_Add(Sigma_H, H, iAdjust_Count * 6, Sigma_H);     //J * Z(-1)* Jt                       
//
//			//求 J*Z(-1)*e
//			Matrix_Multiply(Jt_Sigma, iAdjust_Count * 6, 6, E_6, 1, Jt_Sigma_E);
//			Vector_Add(Sigma_Jt_Sigma_E, Jt_Sigma_E, iAdjust_Count * 6, Sigma_Jt_Sigma_E);
//			//Disp_Fillness(Sigma_H, iAdjust_Count*6, iAdjust_Count* 6);
//		}
//
//		//解方程J1*J1' Δx = -J1 * f(x)'
//		Add_I_Matrix(Sigma_H, iAdjust_Count * 6);
//		Solve_Linear_Gause(Sigma_H, iAdjust_Count * 6, Sigma_Jt_Sigma_E, Delta_X, &iResult);
//		Matrix_Multiply(Delta_X, 1, iAdjust_Count * 6, (_T)-1, Delta_X);
//		//Disp(Delta_X, 1, iAdjust_Count * 6);
//
//		if (fSum_e_Pre <= fSum_e || !iResult)
//			break;
//
//		for (i = 0; i < iAdjust_Count; i++)
//		{
//			se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
//			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
//		}
//		fSum_e_Pre = fSum_e;
//		printf("iIter:%d %.10f\n", iIter, fSum_e);
//	}
//}
//
//static void Test_5()
//{//位姿图优化，引入信息矩阵，看看够不够丝滑
//	typedef float _T;
//	int i, j, iResult, iMeasurement_Count, iCamera_Count;
//	_T(*pKsi)[7], (*Camera)[4 * 4];
//	Measurement<_T>* pMeasurement;
//	Pose_Graph_Sigma_H<_T> oPose_Graph;
//	iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iCamera_Count, &pMeasurement, &iMeasurement_Count);
//	int iAdjust_Count = 200;
//	iCamera_Count = 2500;
//	Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iCamera_Count * 4 * 4 * sizeof(_T));
//	//int iAdjust_Count = iMeasurement_Count;
//	for (i = 0; i < iCamera_Count; i++)
//		TQ_2_Rt(pKsi[i], Camera[i]);
//	Free(&oMatrix_Mem, pKsi);
//	Init_Pose_Graph(pMeasurement, iAdjust_Count - 1, iAdjust_Count, &oPose_Graph);
//
//	int iIter;
//	union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
//	union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
//	union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
//	_T E_4x4[4 * 4], E_6[6], J_Inv[6 * 6], Adj[6 * 6],
//		Jt[12 * 6], J[6 * 12], H[12 * 12], Delta_Pose[4 * 4], Temp[6 * 6],
//		* Sigma_J_Z_Inv_E, * Delta_X;
//	_T fSum_e, e, fSum_e_Pre = (_T)1e10;
//	Sparse_Matrix<_T> oSigma_H;
//	Sigma_J_Z_Inv_E = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
//	Delta_X = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
//	//Disp_Mem(&oMatrix_Mem, 0);
//	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(&oMatrix_Mem, iCamera_Count * 3 * sizeof(_T));
//
//	for (iIter = 0;; iIter++)
//	{
//		fSum_e = 0;
//		Reset_Pose_Graph(oPose_Graph);
//		Init_Sparse_Matrix(&oSigma_H, iAdjust_Count * 3 * 6 * 6, iAdjust_Count * 6, iAdjust_Count * 6);
//		memset(Sigma_J_Z_Inv_E, 0, iCamera_Count * 6 * sizeof(_T));
//
//		for (i = 0; i < iAdjust_Count - 1; i++)
//		{//没一个测量都带来一个调整权重
//			Measurement<_T> oM = pMeasurement[i];
//			if (oM.m_Camera_Index[0] >= iCamera_Count || oM.m_Camera_Index[1] >= iCamera_Count)
//				continue;
//			memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
//			memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
//			TQ_2_Rt(oM.Delta_ksi, M_4x4);
//			Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
//			if (!iResult)break;
//			Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
//			if (!iResult)break;
//
//			//注意，书上是错的
//			Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);
//			Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);			//=Tij(-1) * Ti(-1) * Tj
//
//			SE3_2_se3(E_4x4, E_6);
//
//			for (e = 0, j = 0; j < 3; j++)
//				e += E_6[j] * E_6[j];
//			fSum_e += e;
//
//			//printf("Sum_e:%.10f\n", fSum_e);
//			//接着求雅可比
//			Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
//			Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
//			if (!iResult)break;
//
//			Get_Adj(Tj_Inv, Adj);
//			//此时形成两个雅可比, Ti的雅可比放在Jt的0-5列，Tj的雅可比放在Jt的6-11列
//			//第一个雅可比
//			Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
//			//∂eij/∂ξj
//			Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 6, 0);
//
//			//∂eij/∂ξi  算第二个雅可比
//			Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
//			Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 0, 0);
//			Matrix_Transpose(Jt, 6, 12, J);
//			//到此， J, Jt已经就绪，第一个第二个雅可比都紧凑放在Jt中
//
//			//注意了，这里求的Z^-1 不是 H矩阵，H=JJt, 然而，Z^-1 = JtJ，矩阵是不可交换的！
//			//所有的网上结论都是错的
//			union {
//				_T Z_Inv[6 * 6];
//				_T J_Z_Inv[12 * 6];
//				_T J_Z_Inv_E[12 * 1];
//			};
//
//			//先求H=J * Z^-1 * Jt
//			Matrix_Multiply(Jt, 6, 12, J, 6, Z_Inv);        //=Z^-1 = JtJ
//			Matrix_Multiply(J, 12, 6, Z_Inv, 6, J_Z_Inv);   //=J*Z^-1
//
//			//H=J * Z^-1 * Jt   相当于取平方
//			Matrix_Multiply(J_Z_Inv, 12, 6, Jt, 12, H);
//			//此处要把H矩阵散发到稀疏矩阵Sigma_H中去
//			Distribute_Data(oPose_Graph, H, oM.m_Camera_Index[0], oM.m_Camera_Index[1]);
//
//			//再求 J * Z^1 * E
//			Matrix_Multiply(J_Z_Inv, 12, 6, E_6, 1, J_Z_Inv_E);
//			Vector_Add(&Sigma_J_Z_Inv_E[oM.m_Camera_Index[0] * 6], J_Z_Inv_E, 6, &Sigma_J_Z_Inv_E[oM.m_Camera_Index[0] * 6]);
//			Vector_Add(&Sigma_J_Z_Inv_E[oM.m_Camera_Index[1] * 6], &J_Z_Inv_E[6], 6, &Sigma_J_Z_Inv_E[oM.m_Camera_Index[1] * 6]);
//		}
//		printf("iIter:%d %.10f\n", iIter, fSum_e);
//		if (fSum_e_Pre <= fSum_e)
//			break;
//
//		Copy_Data_2_Sparse(oPose_Graph, &oSigma_H);
//		//Add_I_Matrix(&oSigma_H, &iResult, (_T)100.f);   //此处终于需要改变ramda值了！
//		Add_I_Matrix(&oSigma_H, &iResult, (_T)10.f);   //此处终于需要改变ramda值了！
//		Compact_Sparse_Matrix(&oSigma_H);
//		unsigned long long tStart = iGet_Tick_Count();
//		Solve_Linear_Gause_1(oSigma_H, Sigma_J_Z_Inv_E, Delta_X, &iResult);
//		printf("%lld\n", iGet_Tick_Count() - tStart);
//
//		Free_Sparse_Matrix(&oSigma_H);
//		Matrix_Multiply(Delta_X, 1, iCamera_Count * 6, (_T)-1, Delta_X);
//		if (!iResult)
//			break;
//
//		if (!iResult)
//			break;
//		for (i = 0; i < iCamera_Count; i++)
//		{
//			se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
//			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
//		}
//		fSum_e_Pre = fSum_e;
//	}
//}

static void E_Test_3()
{//自己生成一个球，用这个球的各点坐标测试E矩阵的特性
	int i, iCount;
	typedef float _T;
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	_T T[4 * 4], R[3 * 3];    //相机外参
	_T t[] = { 100,0,0 };

	{//生成一个相机外参，表示从点集1 -> 点集2
		_T Rotation_Vector[] = { 0,1,0,PI / 6 };
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, t, T);
		Disp(Rotation_Vector, 4, 1);
	}

	Gen_Sphere(&pPoint_3D_1, &iCount, 1000.f);
	//将点集向z方向位移1000
	for (i = 0; i < iCount; i++)
		pPoint_3D_1[i][2] += 2000.f;

	//Gen_Cube(&pPoint_3D_1, &iCount,1,0.f,0.f,1000.f);
	pPoint_3D_2 = (_T(*)[3])pMalloc(&oMatrix_Mem, (iCount + 1) * 3 * sizeof(_T));
	for (i = 0; i < iCount; i++)
	{
		_T Point_3D[4];
		memcpy(Point_3D, pPoint_3D_1[i], 3 * sizeof(_T));
		Point_3D[3] = 1.f;
		Matrix_Multiply(T, 4, 4, Point_3D, 1, pPoint_3D_2[i]);
	}

	//投影到归一化平面上
	_T(*pNP_1)[2], (*pNP_2)[2];
	pNP_1 = (_T(*)[2])pMalloc(&oMatrix_Mem, iCount * 3 * sizeof(_T));
	pNP_2 = (_T(*)[2])pMalloc(&oMatrix_Mem, iCount * 3 * sizeof(_T));
	for (i = 0; i < iCount; i++)
	{//
		pNP_1[i][0] = pPoint_3D_1[i][0] / pPoint_3D_1[i][2];
		pNP_1[i][1] = pPoint_3D_1[i][1] / pPoint_3D_1[i][2];
		//pNP_1[i][2] = 1.f;

		pNP_2[i][0] = pPoint_3D_2[i][0] / pPoint_3D_2[i][2];
		pNP_2[i][1] = pPoint_3D_2[i][1] / pPoint_3D_2[i][2];
		//pNP_2[i][2] = 1.f;
	}

	Ransac_Report oReport;
	Ransac_Estimate_E(pNP_1, pNP_2, iCount, 1.f, 0.f, 0.f, &oReport);
	E_2_R_t(oReport.m_Modal_f, pNP_1, pNP_2, iCount, R, t);

	//测试办法，用三角化推导出每一个空间点的坐标P, 再求 P' = Rt * P
	//再将P' 投影到归一化平面上得到NP'，比较NP_2 与 NP'
	Test_E(oReport.m_Modal_f, pNP_1, pNP_2, iCount);

	//然而，这个实验并不能还原空间点的真正位置，关键就在Rt中的t可以是at,具有某种尺度不变性
	//故此，单靠归一化平面上的点集不能重建。
	//直觉靠像素平面与归一化平面之间的关系能逆推出焦距

	//bSave_PLY("c:\\tmp\\1.ply", pNP_2, iCount);
	return;
}

static void E_Test_1()
{//找一组数据验算E矩阵。虽然代码有点散，但咬死了推导过程
#define SAMPLE_COUNT 8
	typedef double _T;

	_T Sample[SAMPLE_COUNT][4] = { {48.00000000, 76.00000000, 300.00000000, 1.00000000},
									{18.00000000, 303.00000000, 300.00000000, 1.00000000},
									{135.00000000, 182.00000000, 300.00000000, 1.00000000},
									{123.00000000, 182.00000000, 300.00000000, 1.00000000},
									{379.00000000, 12.00000000, 300.00000000, 1.00000000},
									{204.00000000, 7.00000000, 300.00000000, 1.00000000},
									{829.00000000, 3.00000000, 300.00000000, 1.00000000},
									{252.00000000, 39.00000000, 300.00000000, 1.00000000} }	
	, K[3 * 3], Pos[4];

	_T Screen_Pos_1[SAMPLE_COUNT][2] = {},Screen_Pos_2[SAMPLE_COUNT][2];
	Image oImage;
	int i;

	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);

	//for (i = 0; i < 80; i++)
	//{
	//	float fAngle = (2.f * PI * i) / 80.f;
	//	
	//	Sample[i][0] = cos(fAngle) * (i*5+400);
	//	Sample[i][1] = sin(fAngle) * (i*5+400);
	//	Sample[i][2] = 300;
	//	Sample[i][3] = 1;
	//	//printf("%f %f %f\n", fAngle, cos(fAngle), Sample[i][0]);
	//}
//Start:	//本来此处想来个循环寻找一组合适的点
	//Disp((_T*)Sample, 8, 4,"Sample");

	//先搞搞相机内参，先把这个搞利索了
	Gen_Camera_Intrinsic(K,1,100,-100,960,540);
	//Disp(K, 3, 3, "K");
	
	for (i = 0; i < SAMPLE_COUNT; i++)
	{
		Matrix_Multiply(K, 3, 3, Sample[i], 1, Pos);
		Matrix_Multiply(Pos, 1, 2, Pos[2]!=0?1.f / Pos[2]:0, Screen_Pos_1[i]);
		//Disp(Screen_Pos_1[i], 1, 3);
		Draw_Point(oImage, (int)Screen_Pos_1[i][0], (int)Screen_Pos_1[i][1],1);
		//sprintf(File, "c:\\tmp\\temp\\%d.bmp", i);
		//bSave_Image(File, oImage);
	}
	bSave_Image("c:\\tmp\\temp\\1.bmp", oImage);
	//再构造一个相机外参 Rt, 绕y轴旋转30度, 再平移 30,0
	_T Rotation_Vector[4] = { 0,1,0, PI * 5 / 180 }, t[3] = { -10,0,0 },
		R[3 * 3], Rt[4 * 4];
	_T Temp_1[4*4];

	Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
	Gen_Homo_Matrix(R, t, Rt);
	Get_Inv_Matrix_Row_Op(Rt, Rt, 4);
	
	//Disp(Rt, 4, 4, "Rt");
	Set_Color(oImage);
	
	for (i = 0; i < SAMPLE_COUNT; i++)
	{
		//先外参， P1 = (1/z) * K*T*P0
		Matrix_Multiply(Rt, 4, 4, Sample[i],1, Pos);
		Matrix_Multiply(K, 3, 3, Pos, 1, Pos);
		Matrix_Multiply(Pos, 1, 2,Pos[2]!=0? 1.f/Pos[2]:0,Screen_Pos_2[i]);
		//Disp(Screen_Pos_2[i], 1, 3);
		Draw_Point(oImage, (int)Screen_Pos_2[i][0], (int)Screen_Pos_2[i][1],1);
		//sprintf(File, "c:\\tmp\\temp\\%d.bmp", i);
		//bSave_Image(File, oImage);
	}
	bSave_Image("c:\\tmp\\temp\\2.bmp", oImage);
	//Disp((_T*)Screen_Pos_2, 8,2);

	//八点有了，看看能否搞个E
	_T Norm_Point_1[8][2], Norm_Point_2[8][2], E[3 * 3];
	_T Residual[8];
	Normalize_Point(Screen_Pos_1, Screen_Pos_2, 8, Norm_Point_1, Norm_Point_2,(float) K[0], (float)K[2], (float)K[5]);
	Estimate_E(Norm_Point_1, Norm_Point_2, 8,E);
	Compute_Squared_Sampson_Error(Norm_Point_1, Norm_Point_2, 8, E, Residual);
	
	Ransac_Report oReport_E;
	Ransac_Estimate_E(Screen_Pos_1, Screen_Pos_2, SAMPLE_COUNT, (float)K[0], (float)K[2], (float)K[5],&oReport_E);
	/*if (oReport_E.m_bSuccess)
	{
		Disp((_T*)Sample, 8, 4, "Sample");
	}*/
	Disp((_T*)Norm_Point_1, 8, 2, "Norm_Point_1");
	Disp((_T*)Norm_Point_2, 8, 2, "Norm_Point_2");

	Disp(E, 3, 3, "E");
	
	//Disp((_T*)Norm_Point_1, 8, 2);	
	E_2_R_t(E, Norm_Point_1, Norm_Point_2,8, R, t);
	
	//R,t再验算一下
	_T I[4*4],New_Point[8][4];
	Gen_Homo_Matrix(R, t, Rt);
	Gen_I_Matrix(I, 4, 4);
	for (i = 0; i < 8; i++)
	{//以下验算在三角化的范围内成功，根据求解路径可知 相机内参为I。 相机1的外参为I 相机2的外参为Rt
		_T z1, z2;
		Triangulate_Point(Norm_Point_1[i], Norm_Point_2[i], I, Rt, New_Point[i]);
		Disp(New_Point[i], 1, 4,"Point_3D");

		//验算  (1/z) * KP * P1
		Matrix_Multiply(I, 4, 4, New_Point[i], 1, Temp_1);
		z1 = Temp_1[2];
		Matrix_Multiply(Temp_1, 1, 4, 1.f/z1,Temp_1);
		Disp(Norm_Point_1[i], 1, 2, "Norm_Point_1");
		Disp(Temp_1, 1, 3, "相机1的投影");

		//验算 (1/z) * KP * P2
		Matrix_Multiply(Rt, 4, 4, New_Point[i], 1, Temp_1);
		z2 = Temp_1[2];
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z1, Temp_1);
		Disp(Norm_Point_2[i], 1, 2, "Norm_Point_2");
		Disp(Temp_1, 1, 3, "相机2的投影");

		//先看 z * NP1
		Matrix_Multiply(Norm_Point_1[i], 1, 2, z1, Temp_1);
		Disp(Temp_1, 1, 2, "z1 * NP1");

		//再验算 z * Rt(-1) * NP2 = P 
		int iResult;
		_T Rt_1[4 * 4],Temp_2[4];

		Get_Inv_Matrix_Row_Op(Rt, Rt_1, 4, &iResult);
		Matrix_Multiply(Rt, 4, 4, New_Point[i], 1, Temp_1);
		Disp(Temp_1, 1, 4, "Rt * P");
		
		memcpy(Temp_2, Norm_Point_2[i], 2 * sizeof(_T));
		Temp_2[2] = 1.f;
		Matrix_Multiply(Temp_2, 1, 3, z2, Temp_1);
		Disp(Temp_1, 1, 3, "z2 * NP2");

		Temp_1[3] = 1;
		Matrix_Multiply(Rt_1, 4, 4, Temp_1, 1, Temp_1);
		Disp(Temp_1, 1, 4, " z2 * Rt(-1) * NP2");
//
		memcpy(Temp_2, Norm_Point_1[i], 2 * sizeof(_T));
		Temp_2[2] = 1.f;
		Matrix_Multiply(Temp_2, 1, 3, z1, Temp_1);
		Temp_1[3] = 1.f;
		Disp(Temp_1, 1, 4, "z1 * NP1");

		//此处已经有了一个非常非常棒的结果， NP2 的齐次坐标 = z1/z2 * Rt * NP1
		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2,Temp_1);
		Disp(Temp_1, 1, 4,"z1/z2 * Rt * NP1");

		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Temp_1[2] = 1;
		Matrix_Multiply(Temp_1, 1, 3, z1, Temp_1);
		Disp(Temp_1, 1, 4, "z1 * NP1");
		
		//此时，既然已经恢复了深度，那么z1 * NP1 已经是3维坐标，为了与Rt相乘，必须再化为齐次坐标
		Temp_1[3] = 1;
		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2, Temp_1);
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");


		//实践证明， 首先 x'= R(x-t) 的运算顺序不适合这个模型，显然这是先位移后旋转。而我们
		//整个相机模型都是先旋转后位移
		//齐次，R是三维，x,x'是二维，也不能直接计算

		//正确的关系是 x' = (z1/z2) * Rt * NP1, 此时，z1,z2的参与必不可少，因为要恢复齐次
		//而z1,z2只有三角化以后才有，故此要验算这个结果，必须逐步恢复齐次坐标
		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Matrix_Multiply(Temp_1, 1, 2, z1, Temp_1);	//第一次化齐次坐标 (x,y,1) * z1
		Temp_1[2] = z1, Temp_1[3] = 1;				//第二次化齐次坐标 (z1*x, z1*y, z1, 1)

		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);		//再算 Rt * x
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2, Temp_1);	//最后再除以 z2, 此时就符合三角化公式了
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");
	}

	_T V[4];
	Rotation_Matrix_2_Vector(R, V);

	//开始再次搞那堆E的推导，目标是验证 x' = R(x-t)
	_T Norm_Point_Homo_1[8][3], Norm_Point_Homo_2[8][3];
	for (i = 0; i < 8; i++)
	{
		Norm_Point_Homo_1[i][0] = Norm_Point_1[i][0];
		Norm_Point_Homo_1[i][1] = Norm_Point_1[i][1];
		Norm_Point_Homo_1[i][2] = 1;

		Norm_Point_Homo_2[i][0] = Norm_Point_2[i][0];
		Norm_Point_Homo_2[i][1] = Norm_Point_2[i][1];
		Norm_Point_Homo_2[i][2] = 1;
	}

	//先验证 x'*E*x=0，从此处看出，数值的最终结果在0.00x 级别上	
	for (i = 0; i < 8; i++)
	{
		Matrix_Multiply(Norm_Point_Homo_2[i], 1, 3, E, 3, Temp_1);
		Matrix_Multiply(Temp_1, 1, 3, Norm_Point_Homo_1[i], 1, Temp_1);
		printf("x\'*E*x= %f\n", Temp_1[0]);
	}

	//再验证 x'= R(x-t)
	for (i = 0; i < 8; i++)
	{
		Vector_Minus(Norm_Point_Homo_1[i], t, 3,Temp_1);
		Matrix_Multiply(R, 3, 3, Temp_1, 1, Temp_1);
	}
	return;
}
void Camera_Extrinsic_Test_2()
{
	float P_0[4] = { 0,0,1000,1 } , P_1[4];
	{//第一个实验，不考虑相机，将点先绕原点转30度，再向右移动100
		//左手系
		float Rotation_Vector[4] = { 0,1,0,PI / 6.f }, t[3] = { 100,0,0 };
		float R[3 * 3], T[4 * 4],P_1[4];
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, t, T);
		
		//再算 P1=T*P0
		Matrix_Multiply(T, 4, 4, P_0, 1, P_1);
		Disp(P_1, 1, 4, "P1");
	}

	{//第二个实验，以相机得观点看待问题，先将相机与第一个实验反方向旋转30度，然后还是
		//按照世界坐标向左移动100，看其位置
		float Rotation_Vector[4] = { 0,1,0,-PI / 6.f }, t[3] = { -100,0,0};
		float R[3 * 3], T[4 * 4], P_1[4],w2c[4*4];
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, t, T);
				
		Get_Inv_Matrix_Row_Op(T, w2c, 4);
		Matrix_Multiply(w2c, 4, 4, P_0, 1, P_1);
		Disp(P_1, 1, 4, "P_1");
		//可见，实验二与实验一的结果不一样，只能按照各自的观点进行计算
	}

	{//第二个实验，用李代数		
		float Ksi[6];
		float Rotation_Vector[4] = { 0,1,0,-PI / 6.f }, t[3] = { -100,0,0 };
		float c2w[4 * 4],w2c[4*4];

		//必须用这个函数根据t,φ求i出ξ
		Gen_Ksi_by_Rotation_Vector_t(Rotation_Vector, t, Ksi, 4);

		se3_2_SE3(Ksi, c2w);	//此方法也没啥营养，只是将ksi转换为c2w

		Matrix_Multiply(c2w, 4, 4, P_0, 1, P_1);
		Get_Inv_Matrix_Row_Op(c2w, w2c, 4);
		Matrix_Multiply(w2c, 4, 4, P_0, 1, P_1);
		Disp(P_1, 1, 4, "P_1");
		//可见，实验三与实验二等价，特别注意ρ和t的区别，不能直接用t
	}

	return;
}
void Camera_Extrinsic_Test_1()
{//相机参数实验
	typedef float _T;

	//设有一点，在世界坐标(0,0,100)
	_T Point_0[4] = { 0,0,100,1 };
	//原相机在(0,0,0)处
	_T Camera_0[4] = { 0, 0, 0, 1 };

	//先看点的移动,先将相机绕y旋转60度, 先左乘R ,再右乘T，即先旋转再位移
	//在z正对屏幕方向为正，绕y轴旋转的情况下，旋转角度为正相当于顺时针
	_T Rotation_Vector[4] = { 0,1,0, -60 * PI / 180 },
		t[3] = { (_T)(Point_0[2] * sqrt(3.f)),0,0 };
	_T R[3 * 3];	
	_T Point_1[4];
	Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
	Matrix_Multiply(R, 3, 3, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 3, "P0 move to");
	Vector_Add(Point_1, t, 3, Point_1);
	//Disp(Point_1, 1, 3, "Point_0 move to");
		
	//再试一下相机参数，必须画图，从以下实验可以看出，w2c正是相机1的外参
	//用一个特例证明的w2c就是相机外参
	_T c2w[4 * 4], w2c[4 * 4];
	Gen_Homo_Matrix(R, t, c2w);
	Get_Inv_Matrix_Row_Op(c2w, w2c, 4);
	Matrix_Multiply(w2c, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 4, "Point_1");

	//用两个过程看相机的外参
	_T Temp_1[4 * 4] = {}, Temp_2[4 * 4],Temp_3[4*4];
	Gen_Homo_Matrix(R, (_T*)NULL, Temp_1);
	Gen_Homo_Matrix((_T*)NULL, t, Temp_2);	
	Matrix_Multiply(Temp_2, 4, 4, Temp_1, 4, Temp_3);
	//Disp(Temp_3, 4, 4, "Rt");	//可见先R后t才是c2w

	Matrix_Multiply(Temp_1, 4, 4, Temp_2, 4, Temp_3);
	Disp(c2w, 4, 4, "c2w");
	//Disp(Temp_3, 4, 4, "tR");	//先t后R不是c2w

	Get_Inv_Matrix_Row_Op(Temp_3, Temp_3,4);
	Matrix_Multiply(Temp_3, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 4, "Point_1");
	
	Matrix_Multiply(w2c, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_0, 1, 4, "x0");
	//Disp(Point_1, 1, 4, "x1");
	
	{//先试一下最简单的平行投影
		_T x0[2] = { Point_0[0],Point_0[1] },
			x1[2] = { Point_1[0],Point_1[1] };
		_T I[4 * 4],Point_3D[4];
		Gen_I_Matrix(I, 4, 4);
		
		Triangulate_Point(x0, x1, I, w2c, Point_3D);
		//Disp(Point_3D, 1, 3, "Point_3D");	//神奇的一幕发生了，确实能恢复到世界坐标
	}
	
	{//第二个实验，给像素平面的坐标是否能三角化出原坐标
		_T I[4*4],K[3 * 3], KP_0[4 * 4],KP_1[4*4];
		_T x0[4], x1[4], Point_3D[4];

		//此处K是到像素平面
		Gen_Camera_Intrinsic(K, 200, 1, 1, 960, 540);
		Gen_Homo_Matrix(K, (_T*)NULL, KP_0);	//可以用相机内参构造一个齐次矩阵
		Gen_I_Matrix(I, 4, 4);
		Disp(I, 4, 4, "I");
		Matrix_Multiply(KP_0, 4, 4, I, 4, KP_0);
		Disp(KP_0, 4, 4, "KP");

		Matrix_Multiply(KP_0, 4, 4, Point_0,1, x0);
		//记得要乘以 1/Z，否则不是投影
		Matrix_Multiply(x0, 1, 2, 1.f / x0[2],x0);
		Disp(Point_0, 1, 3, "Point_0");
		Disp(x0, 1, 2,"x0");

		Gen_Homo_Matrix(K, (_T*)NULL, KP_1);
		Matrix_Multiply(KP_1, 4, 4, w2c, 4, KP_1);

		Matrix_Multiply(KP_1, 4, 4, Point_0, 1, x1);
		//记得要乘以 1/Z，否则不是投影
		Matrix_Multiply(x1, 1, 4, 1.f / x1[2], x1);
		Disp(x1, 1, 4,"x1");
		//至此， x0,x1都已经是 (1/z)* KP * P 的结果

		Triangulate_Point(x0, x1, KP_0, KP_1, Point_3D);
		Disp(Point_3D, 1, 3, "Point_3D");	//实在太神奇了，到像素屏幕坐标恢复成功
	}
	{//用归一化平面来做相机参数，看看能否三角化
		_T focal = 1, cx = 0.5, cy = 0.5;
		_T K[3 * 3],KP_0[4*4],KP_1[4*4],I[4*4];
		_T x0[4], x1[4], Point_3D[4];
		Gen_Camera_Intrinsic(K, focal, 1, 1, cx, cy);
		Disp(K, 3, 3, "K");

		////看看相机内参对不对，是对的
		//Matrix_Multiply(K, 3, 3, Point_0, 1, x0);
		//Matrix_Multiply(x0, 1, 3, 1.f / x0[2], x0);
		//Disp(x0, 1, 2, "x0");

		Gen_I_Matrix(I, 4, 4);
		Gen_Homo_Matrix(K, (_T*)NULL, KP_0);
		Matrix_Multiply(KP_0, 4, 4, I, 4, KP_0);

		Gen_Homo_Matrix(K, (_T*)NULL, KP_1);
		Matrix_Multiply(KP_1, 4, 4, w2c,4, KP_1);

		Matrix_Multiply(KP_0, 4, 4, Point_0, 1, x0);
		Matrix_Multiply(x0, 1, 4, 1.f / x0[2], x0);
		Disp(x0, 1, 4, "x0");

		Matrix_Multiply(KP_1, 4, 4, Point_0, 1, x1);
		Matrix_Multiply(x1, 1, 4, 1.f / x1[2], x1);
		Disp(x1, 1, 4, "x1");

		Triangulate_Point(x0, x1, KP_0, KP_1, Point_3D);
		Disp(Point_3D, 1, 3, "Point_3D");	//完美！投到归一化平面也行
	}
	return;
}
void Camera_Intrinsic_Test()
{//相机内参实验，假定位针孔相机，假定焦距与物体坐标用同一距离单位，成正像
	{//第一个实验，搞明白相机的焦距变化对成像平面上的坐标（u,v)的影响
		float X = 100, Y = 200, Z = 1000;    //点在1000之外
		float f = 10;      //假定焦距为10
		float u, v;
		//x/x' = z/f 求 x' => x'=x*f/z
		u = X * f / Z;
		v = Y * f / Z;
		//z' =z*f/z=f
		printf("投影平面上的坐标为：u:%f v:%f z:%f\n", u, v, f);

		//换个焦距，看看成像平面上的坐标变化
		f = 20;             //假定焦距为20
		u = X * f / Z;
		v = Y * f / Z;
		printf("投影平面上的坐标为：u:%f v:%f z:%f\n", u, v, f);
		//结论，焦距越大，投影坐标越大

		//还有个要点。对于(u,v,f)= (x,y,z) * (f/z) 中，x,y,z可以取任意正值，世界无限大
		//(u,v)作为值域也可以取任意值，即成像平面可以无限大
	}

	{//第二个实验，不同的焦距投影到归一化平面上的坐标
		float X = 100, Y = 200, Z = 1000;    //点在1000之外
		float f = 10;      //假定焦距为10
		float u, v;

		//第一次变化，到成像平面
		u = X * f / Z;
		v = Y * f / Z;
		//第二次变化，再投影到归一化平面，即投影到光心距离为1的平面上
		u /= f;
		v /= f;
		printf("focal:%f u:%f v:%f\n", f, u, v);

		//改变焦距，看看归一化平面上的投影变化
		f = 20;      //假定焦距为10
		//第一次变化，到成像平面
		u = X * f / Z;
		v = Y * f / Z;
		//第二次变化，再投影到归一化平面，即投影到光心距离为1的平面上
		u /= f;
		v /= f;
		printf("focal:%f u:%f v:%f\n", f, u, v);
		//结论1，可见，无论焦距多少，投影到归一化平面上的坐标不变


		//假定一步到位，相机的焦距为1
		f = 1;      //假定焦距为10
		u = X * f / Z;
		v = Y * f / Z;
		printf("focal:%f u:%f v:%f\n", f, u, v);
		//结论2，相机在归一化平面上的投影可以归结为不必考虑焦距，即焦距可以省略或者
		//焦距假定为1
	}

	{//第三个实验，到像素平面的投影。可以理解为成像平面与像素平面之间存在一个缩放。
		//我的理解是成像平面每单位对应多少像素
		float X = 100, Y = 200, Z = 1000;    //点在1000之外
		float f = 10;      //假定焦距为10
		float u, v;
		float s = 20;            //成像平面每单位s个像素

		u = X * f / Z;
		v = Y * f / Z;
		u *= s;
		v *= s;
		printf("像素平面上u:%f v:%f\n", u, v);

		f = 20;      //假定焦距为10
		u = X * f / Z;
		v = Y * f / Z;
		u *= s;
		v *= s;
		printf("像素平面上u:%f v:%f\n", u, v);
		//结论，从成像平面到像素平面的投影，会随焦距的变化而变化
	}

	{//第4个实验，归一化坐标与像素平面坐标的关系
		float X = 100, Y = 200, Z = 1000;    //点在1000之外
		float f = 20;      //假定焦距为10
		float u, v;
		float s = 20;            //成像平面每单位s个像素

		u = X * f / Z;
		v = Y * f / Z;
		printf("归一化坐标：u:%f v:%f\n", u / f, v / f);
		u *= s;
		v *= s;
		printf("像素平面上u:%f v:%f\n", u, v);
		printf("从像素平面直接推归一化坐标：u:%f v:%f\n", u / (f * s), v / (f * s));
		//此时，忽略位移，f*x就是最简相机内参
		//结论，通过相机内参可以一步换算为归一化坐标
	}

	{//第5个实验，已知像素平面上的坐标，还原点的空间位置
		float u = 40, v = 80;
		float Z = 1000;
		float fxy = 400;    //相机参数， fx=fy=400

		float u1, v1;   //归一化平面坐标
		u1 = u / fxy;
		v1 = v / fxy;
		printf("归一化平面坐标：u:%f v:%f\n", u1, v1);
		printf("空间坐标：X:%f Y:%f Z:%f\n", u1 * Z, v1 * Z, Z);
		//此处已经提供了相机内参还原空间坐标的线索
	}
}
void E_Test_2()
{//最简E 验算代码，给定一个E, 还有两个归一化匹配点NP1与NP2。验算一把 NP2=(z1/z2) * Rt * NP1
	typedef double _T;
	_T Norm_Point_1[8][2] = {	{0.16000024, -0.25333321},
								{0.06000023, -1.00999989},
								{0.45000024, -0.60666655},
								{0.41000024, -0.60666655},
								{1.26333360, -0.03999987},
								{0.68000025, -0.02333320},
								{2.76333363, -0.00999987},
								{0.84000025, -0.12999987} },
		Norm_Point_2[8][2] = {	{0.10408432, -0.25007111},
								{0.00579728, -1.00564635},
								{0.37978469, -0.58427734},
								{0.34255785, -0.58624562},
								{1.08598052, -0.03606178},
								{0.58908081, -0.02204666},
								{2.17661246, -0.00806469},
								{0.73006282, -0.12123358} },
		E[3 * 3] = { 0.04694487, 0.10917079, 0.56591215,
					-0.18813973, -0.00669381, -0.99870273,
					-0.56528370, 1.01189271, 0.03976162 };

	Test_E(E, Norm_Point_1, Norm_Point_2, 8);
	return;
}

static void Sift_Test_1()
{//Sift实验，给出一张黑白图，求所有的特征点
	float(*pPoint)[2];
	int iCount;
	unsigned long long tStart = iGet_Tick_Count();
	Get_Sift_Feature("c:\\tmp\\Scene_Cut_A.bmp", &pPoint, &iCount, 0);
	printf("%lld\n", iGet_Tick_Count() - tStart);
	free(pPoint);
	return;
}
static void Sift_Test_2()
{//两张图的匹配，返回两组匹配点位置
	float(*pPoint_1)[2] = NULL, (*pPoint_2)[2];
	int iMatch_Count;

	Sift_Match_2_Image("C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A16.bmp",
		"C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A17.bmp", &pPoint_1, &pPoint_2, &iMatch_Count);

	//for (int i = 0; i < iMatch_Count; i++)
		//printf("%f %f %f %f\n", pPoint_1[i][0], pPoint_1[i][1], pPoint_2[i][0], pPoint_2[i][1]);
	free(pPoint_1);
	return;
}
static void Sift_Test_3()
{//整个目录遍历，此处用Mem_Mgr
	Sift_Match_Map oMatch_Map;
	Sift_Simple_Match_Item oMatch;
	Mem_Mgr oMem_Mgr;
	Init_Mem_Mgr(&oMem_Mgr, 128000000, 1024, 997);
	Sift_Match_Path_1("C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp", &oMatch_Map, &oMem_Mgr);
	int y, x, iIndex;
	for (y = 0; y < oMatch_Map.m_iImage_Count; y++)
	{
		for (x = y + 1; x < oMatch_Map.m_iImage_Count; x++)
		{
			iIndex = iUpper_Triangle_Cord_2_Index(x, y, oMatch_Map.m_iImage_Count);
			oMatch = oMatch_Map.m_pMatch[iIndex];
			printf("Image Pair:%d %d Match Count:%d\n", oMatch.m_iImage_A, oMatch.m_iImage_B, oMatch.m_iMatch_Count);
			/* for (int k = 0; k < oMatch.m_iMatch_Count; k++)
				 printf("%f %f %f %f\n", oMatch.m_pPoint_1[k][0], oMatch.m_pPoint_1[k][1],
					 oMatch.m_pPoint_2[k][0], oMatch.m_pPoint_2[k][1]);*/
		}
	}

	Free(&oMem_Mgr, oMatch_Map.m_pBuffer);
	Free_Mem_Mgr(&oMem_Mgr);
	return;
}
static void Sift_Test_4()
{//整个目录遍历，最简接口
	Sift_Match_Map oMatch_Map;
	Sift_Simple_Match_Item oMatch;

	Sift_Match_Path("C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\ET\\bmp", &oMatch_Map);
	int y, x, iIndex;
	for (y = 0; y < oMatch_Map.m_iImage_Count; y++)
	{
		for (x = y + 1; x < oMatch_Map.m_iImage_Count; x++)
		{
			iIndex = iUpper_Triangle_Cord_2_Index(x, y, oMatch_Map.m_iImage_Count);
			oMatch = oMatch_Map.m_pMatch[iIndex];
			printf("Image Pair:%d %d Match Count:%d\n", oMatch.m_iImage_A, oMatch.m_iImage_B, oMatch.m_iMatch_Count);
			//for (int k = 0; k < oMatch.m_iMatch_Count; k++)
				//printf("%f %f %f %f\n", oMatch.m_pPoint_1[k][0], oMatch.m_pPoint_1[k][1], oMatch.m_pPoint_2[k][0], oMatch.m_pPoint_2[k][1]);
		}
	}
	if(oMatch_Map.m_pBuffer)
		free(oMatch_Map.m_pBuffer);
	return;
}

void Ransac_Test()
{
	typedef double _T;
	_T(*pPoint_1)[2], (*pPoint_2)[2], (*pNew_Point_1)[2], (*pNew_Point_2)[2];
	int iCount;
	float Camera[3] = { 768,320,240 };
	Ransac_Report oReport_H, oReport_E, oReport_F;
	Mem_Mgr oMem_Mgr;
	Init_Mem_Mgr(&oMem_Mgr, 100000000, 1024, 997);
	Temp_Load_Match_Point(&pPoint_1, &pPoint_2, &iCount);
	//Sift_Match_2_Image("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp\\et000.bmp",
	  //  "C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp\\et003.bmp", &pPoint_1, &pPoint_2, &iCount);

	Ransac_Estimate_H(pPoint_1, pPoint_2, iCount, &oReport_H, &oMem_Mgr);
	Ransac_Estimate_E(pPoint_1, pPoint_2, iCount, Camera[0], Camera[1], Camera[2], &oReport_E, &oMem_Mgr);
	Ransac_Estimate_F(pPoint_1, pPoint_2, iCount, &oReport_F, &oMem_Mgr);
	//Disp_Report(oReport_E);
	
	//根据三份Report决定用哪个矩阵进行位姿估计
	Two_View_Geometry oTwo_View_Geo = { oReport_E, oReport_F,oReport_H };
	Determine_Confg(&oTwo_View_Geo, pPoint_1, pPoint_2, iCount, &pNew_Point_1, &pNew_Point_2);
	Estimate_Relative_Pose(oTwo_View_Geo, Camera, Camera, pNew_Point_1, pNew_Point_2, oTwo_View_Geo.num_inliers, &oMem_Mgr);
	
	Free_Report(oReport_H, &oMem_Mgr);
	Free_Report(oReport_E, &oMem_Mgr);
	Free_Report(oReport_F, &oMem_Mgr);
	free(pPoint_1);
	//如果是读入的点，此处要释放
	free(pPoint_2);
	//Free_Report(oReport_H, &oMem_Mgr);
	Disp_Mem(&oMatrix_Mem, 0);
	Free_Mem_Mgr(&oMem_Mgr);
}

void SVD_Test_1()
{//搞一个2000x9的矩阵
#define _T double
	const int w = 9, h = 2000;
	_T M[w * h];
	/* = { -1.11092001, 1.09082501, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.34592309, -0.33966582, 0.31138433,
		 0.44694880, -1.33252455, -1.00000000, 0.00000000, 0.00000000, 0.00000000, -0.07166018, 0.21364628, 0.16033197,
		 1.61034985, 0.32395583, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 3.06228775, 0.61604377, -1.90162886,
		 -0.94637863, -0.08225629, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 1.35323869, 0.11761930, 1.42991256,
		 0.00000000, 0.00000000, 0.00000000, -1.11092001, 1.09082501, -1.00000000, -1.16227131, 1.14124742, -1.04622411,
		 0.00000000, 0.00000000, 0.00000000, 0.44694880, -1.33252455, -1.00000000, -0.45111539, 1.34494674, 1.00932230,
		 0.00000000, 0.00000000, 0.00000000, 1.61034985, 0.32395583, -1.00000000, 0.33483522, 0.06735917, -0.20792701,
		 0.00000000, 0.00000000, 0.00000000, -0.94637863, -0.08225629, -1.00000000, 0.23170076, 0.02013871, 0.24482882 };*/

		 //可以自动赋值，这是个不满秩的矩阵，能很好的测出问题
	for (int i = 0; i < w * h; i++)
		M[i] = i;

	SVD_Info oSVD;

	/*_T* U, * S, * Vt;
	int U_h, U_w, S_w, Vt_h, Vt_w;
	SVD_Allocate(h, w, &U, &S, &Vt, &U_h, &U_w, &S_w, &Vt_h, &Vt_w);*/
	//Disp_Mem(&oMatrix_Mem, 0);
	SVD_Alloc<_T>(h, w, &oSVD);

	//Disp(M, 8, 9, "M");
	int iResult;

	svd_3(M, oSVD, &iResult, 0.000001);
	//Disp((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U, "U");
	//Disp((_T*)oSVD.S, 1, oSVD.w_Min_S, "S");
	//Disp((_T*)oSVD.Vt, oSVD.h_Min_Vt, oSVD.w_Min_Vt,"Vt");
	

	//验算SVD结果
	Test_SVD(M, oSVD, &iResult, 0.000001);
	
	//基础行变换，可以看秩
	Elementary_Row_Operation_1(M, h, w, M, &iResult);
	//Disp((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U);
	Free_SVD(&oSVD);	//用完记得释放
	
	return;
#undef _T
}

void Gradient_Method_Test_1()
{//多元函数求极值最速下降法（梯度法）求解 min y = 2*x1^2 + x2 ^2 -2x1x2 - 4x1 +4
	//此乃二元二次多项式构成的函数，试用多项式结构看看
	Polynormial oPoly;
	Init_Polynormial(&oPoly, 2, 20);
	//不断加单项式
	Add_Poly_Term(&oPoly, 2, 2, 0);
	Add_Poly_Term(&oPoly, 1, 0, 2);
	Add_Poly_Term(&oPoly, -2, 1, 1);
	Add_Poly_Term(&oPoly, -4, 1, 0);
	Add_Poly_Term(&oPoly, 4, 0, 0);
	//Disp(oPoly);

	//所谓雅可比就是一阶导组成的向量
	Polynormial Jacob[2];	//因为二元函数，所以偏导数有两个：fx,fy
	int i, j;
	//求出各变量的一阶偏导
	for (i = 0; i < oPoly.m_iElem_Count; i++)
	{
		Init_Polynormial(&Jacob[i], 2, 20);
		Get_Derivation(&oPoly, i, &Jacob[i]);
		//Disp(Jacob[i]);
	}

	//再求各个变量的二阶导，即Hess矩阵
	Polynormial Hess[2 * 2];
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			Init_Polynormial(&Hess[i * 2 + j], 2, 20);
			Get_Derivation(&Jacob[i], j, &Hess[i * 2 + j]);
			//Disp(Hess[i * 2 + j]);
		}
	}

	//梯度法两大要素，1，求一阶导，（fx0,fx1,...,fxn)就是梯度，此处增长最快，也叫Jacob
	//要求min,就是反过来，即 -Jacob为最速下降方向
	//此时，要有个初值(x0,x1,...,xn), 此处没啥经验，用(0,0)作为初值
	// xk_1 = xk - t*xk		此处xk 与 xk_1为向量，形成一个迭代格

	//梯度法要素二，如何确定 t值？不能拍脑袋，一格娘炮两格扯蛋，要恰到好处
	//求 t也是个优化问题， 问 t取何值，使 min( f(xk - t* Delta_xk) ) 其中 Delta_xk 就是Jacob

	//df/dt 是一个数，一个函数值
	//根据泰勒展开，此处有一个通用公式
	// t = (J'J) /(J'* H *J)

	float xk_1[2], xk[2] = { 0,0 };	//初值
	float J[2], H[2 * 2], Temp_1[2], t;
	const float eps = (float)1e-5;
	int iIter;
	for (iIter = 0; iIter < 300; iIter++)
	{
		//代入xk, 求出J向量
		for (i = 0; i < 2; i++)
		{
			//Disp(Jacob[i]);
			J[i] = fGet_Polynormial_Value(Jacob[i], xk);
		}

		for (i = 0; i < 2; i++)
			for (j = 0; j < 2; j++)
				H[i * 2 + j] = fGet_Polynormial_Value(Hess[i * 2 + j], xk);

		Matrix_Multiply(J, 1, 2, J, 1, &t);
		Matrix_Multiply(J, 1, 2, H, 2, Temp_1);
		Matrix_Multiply(Temp_1, 1, 2, J, 1, Temp_1);
		t /= Temp_1[0];

		Matrix_Multiply(J, 1, 2, t, Temp_1);	//t*J
		Vector_Minus(xk, Temp_1, 2, xk_1);		//xk_1 = xk-t*J;
		Disp(xk, 1, 2, "xk");
		Disp(xk_1, 1, 2, "xk_1");

		//if (abs(xk[0]-xk_1[0])< eps && abs(xk[1] -xk_1[1])<eps)
			//break;
		//另一种收敛判断，如果梯度的模<eps
		if (fGet_Mod(J, 2) < eps)
			break;

		memcpy(xk, xk_1, 2 * sizeof(float));	//xk=xk_1
	}

	//释放
	Free_Polynormial(&oPoly);
	for (i = 0; i < 2; i++)
		Free_Polynormial(&Jacob[i]);
	for (i = 0; i < 2 * 2; i++)
		Free_Polynormial(&Hess[i]);
	return;
}
void Optimize_Newton_Test_2()
{//多元函数优化问题，牛顿法。目前还欠个求解的充分条件。否则发散了也不知哪里出了问题
 //此处求解的问题是 min(f(x)= x0^4 - 2*x0^2*x1 + x0^2 + x1^2 - 4x0 + 4
 //感觉此处已经基本上找到套路，但是还有个头疼的问题，初值如何确定？
 //先构造多项式
	Polynormial oPoly;
	const int iElem_Count = 2;  //变量数

	Init_Polynormial(&oPoly, iElem_Count, 20);
	Add_Poly_Term(&oPoly, 1, 4, 0);
	Add_Poly_Term(&oPoly, -2, 2, 1);
	Add_Poly_Term(&oPoly, 1, 2, 0);
	Add_Poly_Term(&oPoly, 1, 0, 2);
	Add_Poly_Term(&oPoly, -4, 1, 0);
	Add_Poly_Term(&oPoly, 4, 0, 0);
	Disp(oPoly);

	//牛顿法要素1，求一阶导Jacob
	Polynormial Jacob[iElem_Count]; //二元函数，有两个偏导数 fx0 fx1
	int i;
	for (i = 0; i < iElem_Count; i++)
	{
		Init_Polynormial(&Jacob[i], iElem_Count, 20);
		Get_Derivation(&oPoly, i, &Jacob[i]);
		Disp(Jacob[i]);
	}

	//牛顿法要素2，求二阶导Hess
	int j;
	Polynormial Hess[iElem_Count * iElem_Count];
	for (i = 0; i < iElem_Count; i++)
	{
		for (int j = 0; j < iElem_Count; j++)
		{
			Init_Polynormial(&Hess[i * iElem_Count + j], iElem_Count, 20);
			Get_Derivation(&Jacob[i], j, &Hess[i * iElem_Count + j]);
			Disp(Hess[i * iElem_Count + j]);
		}
	}

	//要素三，迭代格为 xk_1 = xk - H(-1)(xk) * J(xk), 与一元方程的迭代格何其相似 xk_1 = xk - f'(xk)/f''(xk)
	float H[iElem_Count * iElem_Count], J[iElem_Count], Temp_1[4];
	float xk[iElem_Count] = { 0,0 };
	int iIter, iResult;
	const float eps = (float)1e-5;
	for (iIter = 0; iIter < 300; iIter++)
	{
		for (i = 0; i < iElem_Count; i++)
			for (j = 0; j < iElem_Count; j++)
				H[i * iElem_Count + j] = fGet_Polynormial_Value(Hess[i * iElem_Count + j], xk);
		for (i = 0; i < iElem_Count; i++)
			J[i] = fGet_Polynormial_Value(Jacob[i], xk);

		//Disp(H, 2, 2);
		Get_Inv_Matrix_Row_Op(H, H, 2, &iResult);
		if (!iResult)
		{//不满秩
			printf("err");
			break;
		}
		Matrix_Multiply(H, 2, 2, J, 1, Temp_1);     //H(-1)(xk)* J(xk)
		Vector_Minus(xk, Temp_1, 2, Temp_1);
		if (fGet_Distance(xk, Temp_1, 2) < eps)
			break;
		memcpy(xk, Temp_1, iElem_Count * sizeof(float));
	}

	Free_Polynormial(&oPoly);
	for (i = 0; i < iElem_Count; i++)
		Free_Polynormial(&Jacob[i]);
	for (i = 0; i < iElem_Count * iElem_Count; i++)
		Free_Polynormial(&Hess[i]);
	return;
}
void Optimize_Newton_Test_1()
{//再次玩最优化问题，先搞一次函数最优化，看看否建立对一般形式的f(x)的求解形式
	//设函数为x^5 + x^3 -7x
	//牛顿法要素1，一阶导，二阶导要好求
	//第一步，要求出 f'(x)和 f''(x)的解析式: 
	//          f'(x) = 5x^4 + 3x^2 -7
	//          f''(x)= 20x^3 + 6x

	//牛顿法要素2，要大致确定驻点范围，利用 f'(x)=0 这个线索，正因为有这个条件使得求解x点成为可能
	//对于 f'(x)= 5x^4 + 3x^2 -7, 当 x=0 时 f'(x)=-7,
	//当 x=1时， f'(x) = 1. 又当  0<x<1时，f'(x)连续， 所以必有驻点
	//所以，可以挑[0,1]之间一点作为初点 x0, 就选 x0=0

	//牛顿法要素3, 迭代格为  x(k+1)= xk - f'(xk)/f''(xk) 故此 f''(x0)不能=0
	//x0=0 时， f'(x0)=f'(0)= -7 f'(x0)=f''(0)=0，显然不行，所以要改用x0=1
	//x1=1 时， f'(x0)=5+3-7=1 f''(x0)=20+6= 26, 这个初值可以
	typedef double _T;
#define eps 1e-10

	_T xk = 1.f - 1.f / 26;
	_T f1, f2;
	int iIter;
	for (iIter = 0; iIter < 30; iIter++)
	{
		//f'(x)=5x ^ 4 + 3x ^ 2 - 7
		f1 = 5 * pow(xk, 4) + 3 * xk * xk - 7;
		//f''(x)= 20x^3 + 6x
		f2 = 20 * pow(xk, 3) + 6 * xk;
		xk = xk - f1 / f2;
		if (abs(f1) < eps)
			break;
	}
	//还要检验一下 是否 f''(x)>0
	if (f1 <= 0)
		printf("Fail\n");
	else if (iIter < 30 || abs(f1) < eps)//x^5 + x^3 -7x
		printf("O了\n最优点在:%f, 最优解:%f\n", xk, pow(xk, 5) + pow(xk, 3) - 7 * xk);
	//总结，牛顿法快，但是要求多多，比如一阶二阶导是否好求，一阶导是否容易推出极值点范围
	//然而，这个优化方法并不是元问题，不断追问，它的基本套路事求解 f'(x)=0, 而这个本质上
	//是解方程问题，所以解方程才是优化问题的关键所在
#undef eps
}

void Sec_Method_Test()
{//两点截弦法求 f(x) = x^2 -2 =0
	//先确定x0,x1， x0=1时，f(x0)=-1; x1=1.5时，f(x1)=0.25。故此，初值有了
	float fPre_x = 1e10, x,	//与x轴得交点
		yk, xk = 0,		// 1,
		yk_1, xk_1 = 2;			// 1.5;	//可以视为x(k+1),y(k+1)
	int iIter;
	for (iIter = 0; iIter < 30; iIter++)
	{
		yk = xk * xk - 2;			//yk=f(xk)
		yk_1 = xk_1 * xk_1 - 2;		//yk_1 = f(xk_1)

		//过(xk,yk), (xk_1,yk_1)两点有一条直线，求其交点(x,y)
		//(y-yk)/(x-xk) = (yk_1-yk)/(xk_1-xk) , 倒数也一样成立
		//(x-xk)/(y-yk) = (xk_1-xk)/(yk_1-yk)	=>
		//x-xk = [(xk_1-xk)/(yk_1-yk)] * (y-yk) =>
		//x = xk + [(xk_1-xk)/(yk_1-yk)] * (y-yk)
		//再求这条直线与x轴的交点, 代入y=0 得到x
		x = xk + ((xk_1 - xk) / (yk_1 - yk)) * (0 - yk);

		//判断是否收敛, 若 |f(x)|<eps 则结束迭代 
		if (fPre_x == x)
			break;
		printf("iIter:%d f(x)=%f\n", iIter, x * x - 2);

		//然后淘汰 xk, 剩下xk_1, x作为下一截弦的两端点
		xk = xk_1;
		fPre_x = xk_1 = x;
	}
	printf("iIter:%d f(x)=%f\n", iIter, x * x - 2);
	return;
	//小结：条件：1，估出定义域(a,b)；2，f(x)在 (a,b)上连续
	//不需求导
}

void Non_Linear_Test()
{//一元非线性方程求解， 以 f(x)= x.e^0.5x -1 =0 在 [0,1]中的根
#define eps 0.00001f
	//f(0)= 0-1=-1	f(1)=1* e^0.5x-1>0 所以可以用对分法
	//先来个对半分法
	float x, y;
	float a = 0, b = 1, x_pre = -1;
	int i;
	for (i = 0;; i++)
	{
		x = (a + b) * 0.5f;
		y = x * (float)exp(x * 0.5f) - 1;
		if (x == x_pre)
			break;
		if (y < 0)
			a = x;
		else
			b = x;
		printf("i:%d x=%f\n", i, x);
		x_pre = x;
	}

	//一般迭代法，诸多限制。 要化成 x= phi(x) 且 phi(x)的一阶导<1
	//变形 x= 1/ e^(x/2) 
	x = x_pre = 0;
	for (i = 0;; i++)
	{
		x = 1.f / (float)exp(x / 2.f);
		if (x == x_pre)
			break;
		printf("i:%d x=%f\n", i, x);
		x_pre = x;
	}

	//尝试Aitken法加速
	float z, w;
	x = x_pre = 0;
	for (i = 0; i < 4; i++)
	{
		y = 1.f / (float)exp(x / 2.f);
		z = 1.f / (float)exp(y / 2.f);
		if (x == y || y == z)
			break;
		//这段神奇的代码尚欠推导，此处决定了迭代的加速
		w = z - (z - y) * (z - y) / ((z - y) - (y - x));
		x = w;
		printf("i:%d x=%f\n", i, x);
	}

	//牛顿法， 构造数列 xn_1= xn + f(xn)/f'(xn)， 要求f(x)在[a,b]上可导且不能有 f'(x)=0
	//牛顿法的根基是泰勒展开，这个推导过程要吃透
	x = 1;
	float fx, f1x;	//分别为f(x)与 f'(x)
	for (i = 0;; i++)
	{
		fx = x * (float)exp(x / 2.f) - 1;
		f1x = (float)exp(x / 2.f) + (x / 2.f) * (float)exp(x / 2.f);
		y = x - fx / f1x;
		if (y == x)
			break;
		x = y;
		printf("i:%d %f:\n", i, x);
	}

	float fy;	//以下 fx= f(xn) fy= f(y)
	//这个方法非常有效，只要求f(a)与f(b)异号，但是收敛阶只到1.618，也已经足够快
	x = 0; y = 1;
	for (i = 0;; i++)
	{
		fx = x *(float)exp(x / 2.f) - 1;
		fy = y * (float)exp(y / 2.f) - 1;
		if (fx == fy)
			break;	//此处可以加一个停机条件，否则下面可能出现分母为0
		z = (x * fy - y * fx) / (fy - fx);
		if (z == y)
			break;
		printf("i:%d x:%f\n", i, z);
		x = y;
		y = z;
	}
	return;
#undef eps
}

static void Least_Square_Test_1()
{//先搞个线性实验，拟合一条直线 f(x)= ax+b
	Image oImage;
	Line_1 oLine;
	int x, y, x1, y1, bResult;
	float A[1024][2], B[1024], ab[2], a, b;
	int m;	//一共有多少行数据

	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);

	//构造一条曲线，从(100, 100)到(300, 400)，搞定后kx+m 相当于 ax+b
	Cal_Line(&oLine, 100, 200, 300, 400);
	for (m = 0, x = 100; x < 400; x += 5)
	{
		y = (int)(oLine.k * x + oLine.m);
		//还要加上(-3,3)之间的随机数
		x1 = x + iGet_Random_No() % 11 - 5;
		y1 = y + iGet_Random_No() % 11 - 5;

		Draw_Point(oImage, x1, y1, 2);
		A[m][0] = (float)x1;
		A[m][1] = 1;
		B[m] = (float)y1;
		m++;	//行数加1
	}
	Solve_Linear_Contradictory((float*)A, m, 2, B, ab, &bResult);
	a = ab[0], b = ab[1];
	x = 100, y = (int)(a * x + b);
	x1 = 400, y1 = (int)(a * (float)x1 + b);
	Mid_Point_Line(oImage, x, y, x1, y1, 255, 0, 0);
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	return;
}
static void Least_Square_Test_2()
{//搞条复杂一点的  y= exp(ax^2 + bx +c)
	//先画一下 y= exp(2x^2+ 3x+4)
	//通过以上例子可以总结，某些曲线拟合关键在于能否通过一定的转换化成线性形式
	float x, y, x1, y1;
	int m, bResult;
	float A[1024][3], B[1024], abc[3], a, b, c;

	Image oImage;

	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 24);
	Set_Color(oImage);

	for (m = 0, x = 0; x < 0.5f; x += 0.01f)
	{
		y = 2.f * x * x + 3.f * x + 4.f;
		y = (float)exp(y);

		x1 = x + (iGet_Random_No() % 100) / 1000.f;
		y1 = y + iGet_Random_No() % 11 - 5;

		A[m][0] = x1 * x1;
		A[m][1] = x1;
		A[m][2] = 1;	//注意：常数项对应的系数恒为1
		B[m] = y1;

		//printf("y:%f\n", y);
		x1 *= 1800;
		if (x1 >= 0 && x1 < oImage.m_iWidth - 1 && y1 >= 0 && y1 <= oImage.m_iHeight - 1)
			Draw_Point(oImage, (int)x1, (int)y1, 2);
		m++;
	}

	//由于此模型为非线性。要用线性的方法来搞，必须变形为： ln(y)= ax^2 + bx +c
	//然后怎么解，还得搞搞
	for (y = 0; y < m; y++)
		B[(int)y] = (float)log(B[(int)y]);
	Solve_Linear_Contradictory((float*)A, m, 3, B, abc, &bResult);

	for (m = 0, x = 0; x < 0.5f; x += 0.01f)
	{
		a = abc[0], b = abc[1], c = abc[2];
		y = 1 * x * x + b * x + c;
		y = (float)exp(y);
		x1 = x * 1800.f;
		if (x1 >= 0 && x1 < oImage.m_iWidth - 1 && y >= 0 && y <= oImage.m_iHeight - 1)
			Draw_Point(oImage, (int)x1, (int)y, 2, 255, 0, 0);
		//printf("%f %f\n",y,B[m]);
		m++;
	}
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	return;
}

void Least_Square_Test_3()
{//搞个二维曲面拟合实验， z= a (x/w)^2 + b(y/h)^2 搞一组添加了随机噪声的样本，用这些样本来做实验
//至此，由样本拟合参数的牛顿法OK了，但是梯度法还没行
	const int w = 10, h = 10;
	const int iSample_Count = w * h;
	const float eps = (float)1e-6;
	float* pCur_Point, (*pPoint_3D)[3] = (float(*)[3])malloc(iSample_Count * 3 * sizeof(float));
	float a0 = 4, b0 = 5,
		xa, xb;  //待求参数

	//造样本集
	int y, x;
	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			pCur_Point = pPoint_3D[y * w + x];
			pCur_Point[0] = (float)x;
			pCur_Point[1] = (float)y;
			pCur_Point[2] = a0 * (float)pow((float)x / w, 2) + b0 * (float)pow((float)y / h, 2);
			//printf("x:%f y:%f z:%f\n", pCur_Point[0], pCur_Point[1], pCur_Point[2]);
			pCur_Point[2] += (float)pow(-1, y * w + x) * iGet_Random_No_cv(0, 10) / 50.f;
			//printf("x:%f y:%f z1:%f\n", pCur_Point[0], pCur_Point[1], pCur_Point[2]);
		}
	}
	//存个盘看图像
	//bSave_PLY("c:\\tmp\\1.ply", pPoint_3D, iSample_Count, 1);

	//目标是求 min(yi - fi(x))^2 这个是最小二乘的一般形式，yi好办，每个样本的函数值，问题是， xk在此处对应的是什么？
	//显然我们要求的是a,b，所以， f(x)不再视为关于向量x的函数，而是视为 关于a,b的函数，所以
	//此处，我们讲a,b改为 xa,xb，这样可能好看些。故此， xk就是(xa,xb)在迭代过程中的一个取值，
	//目标是建立迭代格 x=(xa,xb)= (J'J)(-1) * J'*e + xk
	//先对fi(x)在xk处进行一阶泰勒展开 = fi(xk) + J(xk)(x-xk) 二元为 fi(x,y)= fi(xk,yk) + f'xk(xk,yk)*(x-xk) + f'yk(xk,yk)(y-yk)
	//于是，原来求解问题变成 min Sigma[ (yi-fi(x))^2 ] , x=(xa,xb)
	//泰勒展开 = min Sigma [ yi- fi(xk) - J(xk)(x-xk)]^2 , 等于只将yi-fi(x)展开
	//将前面两项合并为一项 ei = yi - fi(xk) 则上式=
	// min Sigma [ (ei - J(xk)(x-xk))^2] , 驻点在一阶导等于0处
	//即求解 ei - J(xk)(x-xk) = 0 矛盾方程组，求x, 此时，将J视为fi(x)关于x的一阶偏导矩阵，
	// J (x-xk)=ei => J'J (x-xk) = J'ei 
	// (J'J)(-1)(J'J) (x-xk) = (J'J)(-1) * J' ei
	// x-xk=  (J'J)(-1) * J' ei
	//x = (J'J)(-1) * J' ei + xk

	xa = 1, xb = 1;               //xk初值
	float J[iSample_Count][2];  //Jacob
	float Jt[2][iSample_Count]; //J'
	float e[iSample_Count];
	float Temp[iSample_Count * 2];  //搞个足够大的临时空间
	int i, iResult;
	while (1)
	{
		for (i = 0; i < iSample_Count; i++)
		{
			pCur_Point = pPoint_3D[i];
			//逐个求一阶偏导 f(a,b)= a (x/w)^2 + b(y/h)^2
			//fi'a (对a求偏导)  = (x/w)^2
			J[i][0] = (float)pow(pCur_Point[0] / w, 2);
			//fi'b (对b求偏导) = (y/h)^2
			J[i][1] = (float)pow(pCur_Point[1] / h, 2);

			//ei = yi - fi(xk) 要特别留意此处，fi(xk)作为一个整体，要加括号，否则发散
			e[i] = pCur_Point[2] - (xa * (float)pow(pCur_Point[0] / w, 2) + xb * (float)pow(pCur_Point[1] / h, 2));
		}

		Matrix_Transpose((float*)J, iSample_Count, 2, (float*)Jt);          //= J' 2*n
		Matrix_Multiply((float*)Jt, 2, iSample_Count, (float*)J, 2, Temp);  //=J'J 2x2
		Get_Inv_Matrix_Row_Op(Temp, Temp, 2, &iResult);                     //(J'J)(-1) 逆矩阵 2x2
		Matrix_Multiply(Temp, 2, 2, (float*)Jt, iSample_Count, Temp);        //=(J'J)(-1)*J' 2xn
		Matrix_Multiply(Temp, 2, iSample_Count, e, 1, Temp);                //=(J'J)(-1)*J'*e 2x1

		xa = Temp[0] + xa;
		xb = Temp[1] + xb;
		if (abs(Temp[0]) < eps && abs(Temp[1]) < eps)
			break;
		printf("%f %f\n", Temp[0], Temp[1]);
	}
	return;
}
void Least_Square_Test_4()
{//重做一组最小二乘法拟合曲面 z=3*x^2 + 4*y^2, 增加一点噪声，一共拟合2个参数
//第一个实验，一阶梯度法， 迭代格为 x(k+1) = xk + Δx 而 Δx= -tJ
	//第一步，先搞样本
	float xyzs[100][4]; //x, y, f(x), sample
	const float a = 3.f, b = 4.f;
	int y, x, i;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (float)x;
			xyzs[i][1] = (float)y;
			xyzs[i][2] = a * x * x + b * y * y;
			xyzs[i][3] = xyzs[i][2] + ((iGet_Random_No() % 100) - 50.f) / 100.f;
		}
	}

	float C[2] = { 1,1 }, Temp[2 * 2];
	float t, fPart_1, fPart_2; //-2(yi - f(xi))
	int iIter = 0;

	//F(x) = [yi - f(xi)]^2，从参数的观点看，F(x)= [yi - fi(C)]^2
	for (iIter = 0;; iIter++)
	{//注意，此处求解的2可以省略
		float J[2] = { 0 }, H[2][2] = { 0 };
		for (i = 0; i < 100; i++)
		{   //F(x) = ∑ [yi - (c1 * x ^ 2 + c2 * y ^ 2)]^2
			//先求J向量 ∂F/∂cj = -2∑[yi - fi(C)]*∂fi/∂cj
			fPart_1 = -2 * (xyzs[i][3] - (C[0] * xyzs[i][0] * xyzs[i][0] + C[1] * xyzs[i][1] * xyzs[i][1]));
			J[0] += fPart_1 * xyzs[i][0] * xyzs[i][0];  //∂fi/∂c=x^2
			J[1] += fPart_1 * xyzs[i][1] * xyzs[i][1];  //∂fi/∂c=y^2

			//∂F/∂c1 = -2∑[yi - fi(C)]* x^2
			//∂^2F/∂c1c1= -2∑ x^2 * (-∂fi/∂c0)= 2∑x^2* ∂fi/∂c1 = 2∑x^2* x^2 = 2∑x^4
			H[0][0] += 2.f * (float)pow(xyzs[i][0], 4);
			//∂^2F/∂c1c2=2∑x^2* ∂fi/∂c2= 2∑x^2*y^2;
			H[0][1] += 2 * xyzs[i][0] * xyzs[i][0] * xyzs[i][1] * xyzs[i][1];
			//∂F/∂c2=-2∑[yi - fi(C)]*y^2
			//∂F/∂c2c1=-2∑y^2*(-∂fi/∂c1) = -2∑y^2*(-x^2)=-2∑y^2*x^2;
			H[1][0] += 2 * xyzs[i][1] * xyzs[i][1] * xyzs[i][0] * xyzs[i][0];
			//∂F/∂c2c2= -2∑y^2*(-∂fi/∂c2)=2∑y^2 * y^2
			H[1][1] += 2.f * (float)pow(xyzs[i][1], 4);
		}

		////再求个t= (J'J) /(J'* H *J)
		//t = fDot(J, J, 2);
		//Matrix_Multiply(J, 1, 2, (float*)H, 2, Temp);
		//Matrix_Multiply(Temp, 1, 2, J, 1, Temp);
		//t /= Temp[0];

		//以上方法要求二阶导，感觉不是一阶梯度法，以下用另外一种方式求解步长t
		//t 满足 F(C + ΔC)达到最小 F(x) =∑[yi - fi(C-tJ)]^2 
		//dF/dt = 2∑[yi - fi(C-tJ)]*df/dt= 2∑[yi - ((c1-J1t) *x^2 + (c2-J2t)*y^2))]*df/dt
		//f =(c1-J1t) *x^2 + (c2-J2t)*y^2 = (c1*x^2 + c2*y2^2) - (J1*x^2+J2*y^2)*t
		//df/dt = -(J1*x^2+J2*y^2)
		//dF/dt = -2∑[yi - ((c1*x^2 + c2*y2^2) - (J1*x^2+J2*y^2)*t)]*(J1*x^2+J2*y^2) =0
		// dF/dt= -2∑ (yi - ((c1*x^2 + c2*y2^2))*(J1*x^2+J2*y^2) - (J1*x^2+J2*y^2)*(J1*x^2+J2*y^2)t=0
		//t= ∑ (yi - ((c1*x^2 + c2*y2^2))*(J1*x^2+J2*y^2)/∑(J1*x^2+J2*y^2)*(J1*x^2+J2*y^2)

		fPart_1 = fPart_2 = 0;
		for (i = 0; i < 100; i++)
		{
			float fTemp = J[0] * xyzs[i][0] * xyzs[i][0] + J[1] * xyzs[i][1] * xyzs[i][1];;
			fPart_1 += fTemp * (C[0] * xyzs[i][0] * xyzs[i][0] + C[1] * xyzs[i][1] * xyzs[i][1] - xyzs[i][3]);
			fPart_2 += fTemp * fTemp;
		}
		t = fPart_1 / fPart_2;
		//以上两种方法的收敛速度一样，都是6次，故此可以看作同一级别的计算。所以，以上
		//计算方法有可能是同一算法，要展开才知道

		//ΔC = -tJ
		Matrix_Multiply(J, 1, 2, -t, Temp);
		//C(k + 1) = Ck + ΔC
		Vector_Add(C, Temp, 2, C);

		if (fGet_Mod(Temp, 2) < 0.00001f)
			break;
	}

	Disp(Temp, 1, 2);
	return;
}
void Least_Square_Test_5()
{//二阶梯度法，看看能否收敛快点
	//第一步，先搞样本
	float xyzs[100][4]; //x, y, f(x), sample
	const float a = 3.f, b = 4.f;
	int y, x, i;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (float)x;
			xyzs[i][1] = (float)y;
			xyzs[i][2] = a * x * x + b * y * y;
			xyzs[i][3] = xyzs[i][2] + ((iGet_Random_No() % 100) - 50.f) / 100.f;
		}
	}

	float C[2] = { 1,1 }, Temp[2 * 2];
	float fPart_1; //-2(yi - f(xi))
	int iIter = 0, iResult;

	//F(x) = [yi - f(xi)]^2，从参数的观点看，F(x)= [yi - fi(C)]^2
	for (iIter = 0;; iIter++)
	{//注意，此处求解的2可以省略
		float J[2] = { 0 }, H[2][2] = { 0 };
		for (i = 0; i < 100; i++)
		{   //F(x) = ∑ [yi - (c1 * x ^ 2 + c2 * y ^ 2)]^2
			//先求J向量 ∂F/∂cj = -2∑[yi - fi(C)]*∂fi/∂cj
			fPart_1 = -2 * (xyzs[i][3] - (C[0] * xyzs[i][0] * xyzs[i][0] + C[1] * xyzs[i][1] * xyzs[i][1]));
			J[0] += fPart_1 * xyzs[i][0] * xyzs[i][0];  //∂fi/∂c=x^2
			J[1] += fPart_1 * xyzs[i][1] * xyzs[i][1];  //∂fi/∂c=y^2

			//∂F/∂c1 = -2∑[yi - fi(C)]* x^2
			//∂^2F/∂c1c1= -2∑ x^2 * (-∂fi/∂c0)= 2∑x^2* ∂fi/∂c1 = 2∑x^2* x^2 = 2∑x^4
			H[0][0] += 2.f * (float)pow(xyzs[i][0], 4);
			//∂^2F/∂c1c2=2∑x^2* ∂fi/∂c2= 2∑x^2*y^2;
			H[0][1] += 2 * xyzs[i][0] * xyzs[i][0] * xyzs[i][1] * xyzs[i][1];
			//∂F/∂c2=-2∑[yi - fi(C)]*y^2
			//∂F/∂c2c1=-2∑y^2*(-∂fi/∂c1) = -2∑y^2*(-x^2)=-2∑y^2*x^2;
			H[1][0] += 2 * xyzs[i][1] * xyzs[i][1] * xyzs[i][0] * xyzs[i][0];
			//∂F/∂c2c2= -2∑y^2*(-∂fi/∂c2)=2∑y^2 * y^2
			H[1][1] += 2.f * (float)pow(xyzs[i][1], 4);
		}

		//最主要的差别在于ΔC = -H(-1) * J
		Get_Inv_Matrix_Row_Op((float*)H, (float*)H, 2, &iResult);
		if (!iResult)
			printf("Fail");
		Matrix_Multiply((float*)H, 2, 2, J, 1, Temp);
		Matrix_Multiply(Temp, 1, 2, -1.f, Temp);

		//C(k + 1) = Ck + ΔC
		Vector_Add(C, Temp, 2, C);
		if (fGet_Mod(Temp, 2) < 0.00001f)
			break;
	}
	//结论：收敛快
	Disp(Temp, 1, 2);
	return;
}

void Least_Square_Test_6()
{//第三种方法，拟合曲面 z=3*x^2 + 4*y^2，希望不必求解二阶导，所谓高斯牛顿法
//此处已经不是求F(x)的负梯度再迈一个步长这种观念了，而是直接干ΔC，走一个ΔC，使得下降最快
	float xyzs[100][4]; //x, y, f(x), sample
	const float a = 3.f, b = 4.f;
	int y, x, i;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (float)x;
			xyzs[i][1] = (float)y;
			xyzs[i][2] = a * x * x + b * y * y;
			xyzs[i][3] = xyzs[i][2] + ((iGet_Random_No() % 100) - 50.f) / 100.f;
		}
	}

	float C[2] = { 1,1 }, Delta_C[2];
	const float eps = 0.000001f;
	int iIter = 0, iResult;
	for (iIter = 0;; iIter++)
	{//注意，此处求解的2可以省略
		float J[2] = { 0 }, H[2][2] = { 0 }, H_Inv[2][2],
			Temp[2 * 2];

		float f, Jf[2] = { 0 };

		for (i = 0; i < 100; i++)
		{   // f(C)= ∑yi - gi(C) =∑yi -(c1*x^2 + c2*y^2)   , 求 f(C)的梯度
			//∂f/∂c1 = -∑x^2
			J[0] = -xyzs[i][0] * xyzs[i][0];
			//∂f/∂c2 = -∑y^2
			J[1] = -xyzs[i][1] * xyzs[i][1];

			f = xyzs[i][3] - (C[0] * xyzs[i][0] * xyzs[i][0] + C[1] * xyzs[i][1] * xyzs[i][1]);

			Jf[0] += f * J[0];  //求∑Jf
			Jf[1] += f * J[1];

			Matrix_Multiply(J, 2, 1, J, 2, Temp);
			Matrix_Add((float*)H, Temp, 2, (float*)H);  //∑H = ∑JJ'
		}

		//求解Jf = JJ'ΔC，解出ΔC
		Get_Inv_Matrix_Row_Op((float*)H, (float*)H_Inv, 2, &iResult);

		//ΔC = H(-1) * Jf
		Matrix_Multiply((float*)H_Inv, 2, 2, Jf, 1, Delta_C);
		Matrix_Multiply(Delta_C, 1, 2, -1.f, Delta_C);

		if (fGet_Mod(Delta_C, 2) < eps)
			break;
		Vector_Add(Delta_C, C, 2, C);
	}
	printf("Iteration times:%d\n", iIter);
	Disp(C, 1, 2, "Solution C");
	return;
}
static void Gradient_Method_Test_2()
{//搞搞求非线性极值
//先试一下 f(x,y)= x^2 + y^2
	//看看二阶导是否容易求
	// fx= 2*x,		fy=2*y	于是梯度为 -(2x,2y)
	float y, x, z;
	float fx, fy;	//一阶偏导
	float t, delta_x, delta_y;

	x = -800; y = 100;
	z = x * x + y * y;
	while (1)
	{
		fx = 2 * x;
		fy = 2 * y;
		//此时， (fx,fy)指明了前进的方向，但是没有指明前进的步长，步子迈得太大会扯到蛋

		//t就是表征顺着梯度方向走的步长 delta_x = t * fx, delta_y=t*fy
		//然而t 怎么算呢？就是满足 f(x+delta_x,y+delta_x)- f(x,y)达到最大
		//即 f(x+ t*fx, y + t*fy)-f(x,y) 达到最大
		//即可求 df/dt=0, 解此方程即可。所以有解的条件为 对 r的导数方程容易解，否则麻烦
		t = 0.5f;	//居然好解

		delta_x = -t * fx;
		delta_y = -t * fy;
		x += delta_x;
		y += delta_y;

		z = x * x + y * y;
	}
	//这个例子不好，一步到位
}
static void Optimize_Newton_Test_3()
{//牛顿法优化问题， 求解 min f(x1,x2)= x1^3 + x2^3 -3(x1+x2)
	float x1 = 6, x2 = 4;	//这个初值有一定限制，必须在一定范围内？怎么估？
	float fx1, fx2;	//一阶导
	float fx1x1, fx1x2, fx2x1, fx2x2;	//二阶导
	//迭代格式为 xk_1= xk - grad^2 f(xk)^(-1) * grad f(xk)

	while (1)
	{
		//首先求一阶导
		//fx1= 3x1^2-3 fx2=3x2^2-3 -> grad f(x)= 3(x1^2-1, x2^2-1)
		fx1 = 3 * x1 * x1 - 3, fx2 = 3 * x2 * x2 - 3;

		//再求二阶导
		//fx1x1= 3*2*x1=6x1
		fx1x1 = 6 * x1;
		//fx1x2=0	没有x2项
		fx1x2 = 0;
		//fx2x1 = 0;
		fx2x1 = fx1x2;		//求导顺序可换，Hesse矩阵必然堆成
		//fx2x2= 3*2*x2=6x2
		fx2x2 = 6 * x2;

		//直接用向量与矩阵表示
		float xk_1[2], xk[] = { x1,x2 };
		float grad_f[] = { fx1,fx2 };
		float grad2_f[] = { fx1x1,fx1x2,
							fx2x1,fx2x2 };

		//求 xk+1
		float Inv_grad2_f[4], Temp[2];
		int bResult;
		Get_Inv_Matrix_Row_Op(grad2_f, Inv_grad2_f, 2, &bResult);
		Matrix_Multiply(Inv_grad2_f, 2, 2, grad_f, 2, Temp);
		Vector_Minus(xk, Temp, 2, xk_1);

		Vector_Minus(xk_1, xk, 2, Temp);
		if (fGet_Mod(Temp, 2) == 0.f)
			break;

		x1 = xk_1[0], x2 = xk_1[1];	//xk=xk_1
		Disp(xk, 1, 2, "xk");	//可以看出，迅速收敛，牛逼！
	}

	return;
}

void Polynormial_Test()
{//构造多项式例子
	Polynormial oPoly;
	Init_Polynormial(&oPoly, 3, 20);
	//加入各项。如果缺变量，即xi^0=1，即其幂为0
	Add_Poly_Term(&oPoly, 0.5f, 1, 0, 3);
	Add_Poly_Term(&oPoly, 0.5f, 1, 0, 3);
	Add_Poly_Term(&oPoly, 0.5f, 1, 2, 3);
	Disp(oPoly);
	Get_Derivation(&oPoly, 0);	//多项式求导
	Disp(oPoly);
	Free_Polynormial(&oPoly);	//记得释放内存
	return;
}

void RGBD_Test()
{//此处为一个RGBD图根据相机参数及 深度量化因子恢复空间点坐标的例子
	//必须要有两大参数：1，相机内参；2，深度因子
	typedef double _T;
	//已知相机二所观察到的点
	_T(*pPoint_3D)[3];
	_T K[3 * 3] = { 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 };    //已知条件相机内参
	unsigned char* pDepth_1;
	int i, iSize, iPoint_Count;
	Image oImage_1;
	bLoad_Raw_Data("D:\\Sample\\Depth\\Slam_Test\\1.depth", &pDepth_1, &iSize);
	bLoad_Image("D:\\Sample\\Depth\\Slam_Test\\1.bmp", &oImage_1);
	pPoint_3D = (_T(*)[3])malloc(iSize * 3 * sizeof(_T));

	//此处做了一个恢复函数，不看细节犹可
	unsigned char(*pColor)[3] = (unsigned char(*)[3])malloc(iSize * 3);
	RGBD_2_Point_3D(oImage_1, (unsigned short*)pDepth_1, (_T(*)[3])K, (_T)5000.f, pPoint_3D, &iPoint_Count, pColor);
	//bSave_PLY("c:\\tmp\\2.ply", pPoint_3D, iPoint_Count,pColor);

	//此处正经开始做恢复的演示
	//恢复要素1，深度量化因子，从量化为16位的深度图中恢复真实的深度，米为单位
	_T fDepth_Factor = 5000; //这个数字怀疑和相机参数一样，来自上游。不知道可能不行
	iSize = oImage_1.m_iWidth * oImage_1.m_iHeight;
	int y, x;

	//首先简单取数，把现有的x,y,z取出
	for (iPoint_Count = y = i = 0; y < oImage_1.m_iHeight; y++)
	{
		for (x = 0; x < oImage_1.m_iWidth; x++, i++)
		{
			if (*(unsigned short*)&pDepth_1[i << 1])
			{
				pPoint_3D[iPoint_Count][0] = x;
				pPoint_3D[iPoint_Count][1] = y;
				//取个深度，顺序有点变态
				pPoint_3D[iPoint_Count][2] = (unsigned short)((pDepth_1[i << 1] << 8) + pDepth_1[(i << 1) + 1]);
				iPoint_Count++;
			}
		}
	}

	for (i = 0; i < iPoint_Count; i++)
	{
		//第一步，先恢复真实深度。除以深度因子方可
		pPoint_3D[i][2] /= fDepth_Factor;
		//第二步，恢复真实x,y
		//已知屏幕坐标u,y 首先得其归一化坐标
		//x = (u-tx)/fx y=(v-ty)/fy 然后再乘上深度z即为真实的点坐标
		pPoint_3D[i][0] = (pPoint_3D[i][0] - K[2]) * pPoint_3D[i][2] / K[0];
		pPoint_3D[i][1] = (pPoint_3D[i][1] - K[5]) * pPoint_3D[i][2] / K[4];
	}
	free(pPoint_3D);
	free(pColor);
	free(pDepth_1);
	Free_Image(&oImage_1);
	return;
}

template<typename _T>void Temp_Load_File_1(const char* pcFile, _T(**ppPoint_3D_1)[3], _T(**ppPoint_3D_2)[3], int* piCount)
{
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	int i, iCount = (int)(iGet_File_Length((char*)pcFile) / (6 * sizeof(float)));
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
	{
		printf("Failt to load file\n");
		return;
	}
	pPoint_3D_1 = (_T(*)[3])malloc(iCount * 3 * sizeof(_T) * 2);
	pPoint_3D_2 = pPoint_3D_1 + iCount;
	for (i = 0; i < iCount; i++)
	{
		float Data[6];
		fread(Data, 1, 6 * sizeof(float), pFile);
		pPoint_3D_1[i][0] = Data[0];
		pPoint_3D_1[i][1] = Data[1];
		pPoint_3D_1[i][2] = Data[2];
		pPoint_3D_2[i][0] = Data[3];
		pPoint_3D_2[i][1] = Data[4];
		pPoint_3D_2[i][2] = Data[5];
	}
	fclose(pFile);
	if (ppPoint_3D_1)
		*ppPoint_3D_1 = pPoint_3D_1;
	if (ppPoint_3D_2)
		*ppPoint_3D_2 = pPoint_3D_2;
	if (piCount)
		*piCount = iCount;
	return;
}

template<typename _T>void Temp_Load_File(const char* pcFile, _T(**ppPoint_3D_1)[3], _T(**ppPoint_3D_2)[3], int* piCount)
{
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	int i, iCount = (int)(iGet_File_Length((char*)pcFile) / (5 * sizeof(float)));
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
	{
		printf("Failt to load file\n");
		return;
	}
	pPoint_3D_1 = (_T(*)[3])malloc(iCount * 3 * sizeof(_T) * 2);
	pPoint_3D_2 = pPoint_3D_1 + iCount;
	for (i = 0; i < iCount; i++)
	{
		float Data[5];
		fread(Data, 1, 5 * sizeof(float), pFile);
		pPoint_3D_1[i][0] = Data[0];
		pPoint_3D_1[i][1] = Data[1];
		pPoint_3D_1[i][2] = Data[2];
		pPoint_3D_2[i][0] = Data[3];
		pPoint_3D_2[i][1] = Data[4];
		pPoint_3D_2[i][2] = 0;
	}
	fclose(pFile);
	if (ppPoint_3D_1)
		*ppPoint_3D_1 = pPoint_3D_1;
	if (ppPoint_3D_2)
		*ppPoint_3D_2 = pPoint_3D_2;
	if (piCount)
		*piCount = iCount;
	return;
}


template<typename _T>void Normalize(_T Point_3D[][3], Point_2D<_T> Point_2D[], int iPoint_Count, _T Camera[][9], int iCamera_Count)
{
	_T fScale, Mid[3], * pPoint_1D = (_T*)pMalloc(&oMatrix_Mem, iPoint_Count * sizeof(_T));
	_T Temp[3];
	int i, j;

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < iPoint_Count; j++)
			pPoint_1D[j] = Point_3D[j][i];
		Mid[i] = oGet_Nth_Elem(pPoint_1D, iPoint_Count, iPoint_Count >> 1);
	}
	//Temp Code
	//Mid[0] = 1.65443f;
	//Disp(Mid, 1, 3, "Mid");

	for (j = 0; j < iPoint_Count; j++)
	{
		Vector_Minus(Point_3D[j], Mid, 3, Temp);
		pPoint_1D[j] = Abs(Temp[0]) + Abs(Temp[1]) + Abs(Temp[2]);
	}

	//搞了个Bounding box, 边长为 3个点到中心距离的中位数之和，也许足够大
	//然后将点装在这个Box, 然后再投影到边长为100的Box
	fScale = oGet_Nth_Elem(pPoint_1D, iPoint_Count, iPoint_Count >> 1);
	//Temp code
	//fScale = 22.608061669690585f;
	//FILE* pFile = fopen("c:\\tmp\\2.txt","wb");
	fScale = 100.0f / fScale;
	//Point_3D = scale * (Point_3D - Mid)
	for (i = 0; i < iPoint_Count; i++)
	{
		Vector_Minus(Point_3D[i], Mid, 3, Temp);
		Matrix_Multiply(Temp, 1, 3, fScale, Point_3D[i]);
		//fprintf(pFile, "%f %f %f\n", Point_3D[i][0], Point_3D[i][1], Point_3D[i][2]);
	}
	//fclose(pFile);
	//[0 - 2] : angle - axis rotation
	//[3, 4, 5] 位移 
	//[6] focal 焦距
	//[7 - 8] second and forth order radial distortion
	_T* pCur_Camera;
	_T Rotation_Vector[4], R[3 * 3];

	for (i = 0; i < iCamera_Count; i++)
	{
		pCur_Camera = Camera[i];
		Matrix_Multiply(pCur_Camera, 1, 3, (_T)-1.f, Rotation_Vector);
		Rotation_Vector_3_2_4(Rotation_Vector, Rotation_Vector);
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Matrix_Multiply(R, 3, 3, &pCur_Camera[3], 1, Temp);
		Matrix_Multiply(Temp, 1, 3, (_T)-1.f, Temp);

		//Temp就是center 将center也规格化
		Vector_Minus(Temp, Mid, 3, Temp);
		Matrix_Multiply(Temp, 1, 3, fScale, Temp);

		//还要调整一次
		Rotation_Vector[3] = -Rotation_Vector[3];
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Matrix_Multiply(R, 3, 3, Temp, 1, Temp);
		Matrix_Multiply(Temp, 1, 3, (_T)-1.f, &pCur_Camera[3]);
	}

	//Temp code
	//Camera[10][3] = 0.073109f;
	return;
}

void BA_Test_Schur_Ref()
{//Schur_Test的一个参考，只有数据对照意义
	typedef double _T;
	int iCamera_Count, iPoint_Count, iObservation_Count, i, iIter, iResult;
	_T(*pPoint_3D)[3], eps = (_T)1.0e-14,
		(*pCamera_Data)[3 * 3];    //只是个内参
	Point_2D<_T>* pPoint_2D, oCur_Point;
	
	//[0 - 2] : angle - axis rotation
	//[3, 4, 5] 位移 
	//[6] focal 焦距
	//[7 - 8] 畸变模型中的k1,k2
	_T Camera[16][4 * 4], Rotation_Vector[4], R[16][3 * 3];

	Temp_Load_File_2(&iCamera_Count, &iPoint_Count, &iObservation_Count, &pPoint_2D, &pPoint_3D, &pCamera_Data);
	//Normalize(pPoint_3D, pPoint_2D, iPoint_Count, pCamera_Data, iCamera_Count);
	for (i = 0; i < iCamera_Count; i++)
	{
		Rotation_Vector_3_2_4(pCamera_Data[i], Rotation_Vector);
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R[i]);
		Gen_Homo_Matrix(R[i], &pCamera_Data[i][3], Camera[i]);
	}

	_T* Jt, * J, * JJt, * JE, * Sigma_JE, * Sigma_H, * H_Inv, * Delta_Ksi;
	_T E[2], Delta_Pose[4 * 4], fSum_e, fSum_e_Pre = (_T)1e20;
	const int iAdjust_Count = 100;
	int iJt_w = 6 * iCamera_Count + iAdjust_Count * 3;
	unsigned char* pPoint_Adjust_Flag;  //1:已经调整，0：未
	//在此开内存
	Jt = (_T*)pMalloc(&oMatrix_Mem, 2 * iJt_w * sizeof(_T));
	J = (_T*)pMalloc(&oMatrix_Mem, iJt_w * 2 * sizeof(_T));
	JE = (_T*)pMalloc(&oMatrix_Mem, iJt_w * sizeof(_T));
	Delta_Ksi = (_T*)pMalloc(&oMatrix_Mem, iJt_w * sizeof(_T));
	Sigma_JE = (_T*)pMalloc(&oMatrix_Mem, iJt_w * sizeof(_T));
	Sigma_H = (_T*)pMalloc(&oMatrix_Mem, iJt_w * iJt_w * sizeof(_T));
	H_Inv = (_T*)pMalloc(&oMatrix_Mem, iJt_w * iJt_w * sizeof(_T));
	JJt = (_T*)pMalloc(&oMatrix_Mem, iJt_w * iJt_w * sizeof(_T));
	pPoint_Adjust_Flag = (unsigned char*)pMalloc(&oMatrix_Mem, (iPoint_Count + 7) >> 3);
	
	//以下只估计相机组，小试牛刀，已经O了
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, iJt_w * iJt_w * sizeof(_T));
		memset(Sigma_JE, 0, iJt_w * sizeof(_T));
		memset(pPoint_Adjust_Flag, 0, (iPoint_Count + 7) >> 3);

		for (i = 0; i < iObservation_Count; i++)
		{
			oCur_Point = pPoint_2D[i];
			_T* pCur_Point_3D = pPoint_3D[oCur_Point.m_iPoint_Index];
			_T Point_3D_1[4], Point_4D[4] = { pCur_Point_3D[0], pCur_Point_3D[1], pCur_Point_3D[2], 1 };
			_T* pCur_Camera = Camera[oCur_Point.m_iCamera_Index];
			_T Temp[4];
			_T focal = pCamera_Data[oCur_Point.m_iCamera_Index][6];
			//Disp(pCamera_Data[0], 1, 9, "Camera");
			//Disp(pCur_Point_3D, 1, 3, "Point_3D");
			//Disp(oCur_Point.m_Pos, 1, 2, "Point_2D");

			//此处存在z<0的情况，所以从 I 矩阵出发得不到最优解
			if (Point_4D[2] == 0.0)
				continue;
			if (i >= iAdjust_Count)
				continue;
			Matrix_Multiply(pCur_Camera, 4, 4, Point_4D, 1, Point_3D_1);   //得TP
			//Disp(pCamera_Data[oCur_Point.m_iCamera_Index], 1, 6, "Camera");
			//Disp(pCur_Camera, 4, 4, "Camera");
			//Disp(Point_4D, 1, 3, "Point");
			//Disp(Point_3D_1, 1, 4, "TP");
			Temp[0] = -Point_3D_1[0] / Point_3D_1[2], Temp[1] = -Point_3D_1[1] / Point_3D_1[2];     //投影到归一化平面上
			Temp[0] *= focal, Temp[1] *= focal;           //投影到成像平面上
			//Disp(Temp, 1, 2, "uv");
			
			E[0] = Temp[0] - oCur_Point.m_Pos[0];	//对应点i的数值差
			E[1] = Temp[1] - oCur_Point.m_Pos[1];
			/*if (iIter == 1 && i==6)
			{
				Disp(pCur_Camera, 4, 4, "Camera");
				Disp(pCur_Point_3D, 1, 3, "Point_3D");
				Disp(E, 1, 2, "E");
				printf("i:%d %f\n", i, fSum_e);
			}*/
				
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差

			//构造雅可比
			union {
				_T Jt_TP_Ksi[3 * 6];
				_T Jt_E_Ksi[2 * 6];
			};
			_T Jt_UV_P[2 * 3], Jt_UV_Porg[2 * 3];
			Get_Drive_UV_P(focal, focal, Point_3D_1, Jt_UV_P);
			Get_Deriv_TP_Ksi_1(pCur_Camera, Point_4D, Jt_TP_Ksi);
			//Disp(Jt_TP_Ksi, 2, 6);
			Matrix_Multiply(Jt_UV_P, 2, 3, Jt_TP_Ksi, 6, Jt_E_Ksi);
			Matrix_Multiply(Jt_E_Ksi, 2, 6, (_T)-1.f, Jt_E_Ksi);	
			//Disp(Jt_E_Ksi, 2, 6, "Jt");

			memset(Jt, 0, 2 * iJt_w * sizeof(_T));
			Copy_Matrix_Partial(Jt_E_Ksi, 2, 6, Jt, iJt_w, oCur_Point.m_iCamera_Index * 6, 0);
			//还要求原来的Porg微弱扰动对uv的影响,
			//uv = KTP => ∂uv/∂p = ∂uv/∂p' * ∂p'/∂p
			if (i < iAdjust_Count)
			{
				Matrix_Multiply(Jt_UV_P, 2, 3, R[oCur_Point.m_iCamera_Index], 3, Jt_UV_Porg);
				//为什么此处乘以-1反而错误？
				//Matrix_Multiply(Jt_UV_Porg, 2, 3, (_T)-1.f, Jt_UV_Porg);
				Copy_Matrix_Partial(Jt_UV_Porg, 2, 3, Jt, iJt_w, iCamera_Count * 6 + oCur_Point.m_iPoint_Index * 3, 0);
			}
			
			Matrix_Transpose(Jt, 2, iJt_w, J);
			Matrix_Multiply(J, iJt_w, 2, Jt, iJt_w, JJt);           //JJ'
			Matrix_Add(Sigma_H, JJt, iJt_w, Sigma_H);
			Matrix_Multiply(J, iJt_w, 2, E, 1, JE);             //JE
			//Disp(JE, iJt_w,1 );

			Vector_Add(Sigma_JE, JE, iJt_w, Sigma_JE);
			//Disp_Fillness(Sigma_JE, 1, iJt_w);
		}

		Add_I_Matrix(Sigma_H, iJt_w,(_T)1);
		//方法1，解方程方法
		Solve_Linear_Gause(Sigma_H, iJt_w, Sigma_JE, Delta_Ksi, &iResult);
		//Disp(Delta_Ksi, 1, iCamera_Count * 6, "Delta_Ksi");

		////方法2，schur消元Crop Camera Data，此处做个schur参考
		//_T* B, * C_Inv, * v, * w, * pTemp;
		//union {
		//	_T* E;
		//	_T* Et;
		//};
		//_T* E_Cinv, * v_E_Cinv_w, * B_E_Cinv_Et;
		//v = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
		//w = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 3 * sizeof(_T));
		//B = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * iCamera_Count * 6 * sizeof(_T));
		//E = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * iAdjust_Count * 3 * sizeof(_T));
		//C_Inv = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 3 * iAdjust_Count * 3 * sizeof(_T));
		//E_Cinv = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * iAdjust_Count * 3 * sizeof(_T));
		//v_E_Cinv_w = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
		//B_E_Cinv_Et = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * iCamera_Count * 6 * sizeof(_T));

		//memcpy(v, Sigma_JE, iCamera_Count * 6 * sizeof(_T));
		//memcpy(w, Sigma_JE + iCamera_Count * 6, iAdjust_Count * 3 * sizeof(_T));
		//Crop_Matrix(Sigma_H, iJt_w, iJt_w, 0, 0, iCamera_Count * 6, iCamera_Count * 6, B);
		//Crop_Matrix(Sigma_H, iJt_w, iJt_w, iCamera_Count * 6, 0, iAdjust_Count * 3, iCamera_Count * 6, E);
		//Crop_Matrix(Sigma_H, iJt_w, iJt_w, iCamera_Count * 6, iCamera_Count * 6, iAdjust_Count * 3, iAdjust_Count * 3, C_Inv);
		//Get_Inv_Matrix_Row_Op_2(C_Inv, C_Inv, iAdjust_Count * 3, &iResult);

		////算出EC(-t)
		//pTemp = H_Inv;
		//Matrix_Multiply(E, iCamera_Count * 6, iAdjust_Count * 3, C_Inv, iAdjust_Count * 3, E_Cinv);
		//Matrix_Multiply(E_Cinv, iCamera_Count * 6, iAdjust_Count * 3, w, 1, pTemp); //EC(-1)w
		//Vector_Minus(v, pTemp, iCamera_Count * 6, v_E_Cinv_w);    //v-EC(-1)w

		////再算B-EC(-1)*E'
		//Matrix_Transpose(E, iCamera_Count * 6, iAdjust_Count * 3, Et);
		//Matrix_Multiply(E_Cinv, iCamera_Count * 6, iAdjust_Count * 3, Et, iCamera_Count * 6, pTemp);    //EC(-1)E'
		//Vector_Minus(B, pTemp, iCamera_Count * 6 * iCamera_Count * 6, B_E_Cinv_Et);
		////printf("%d\n", bIs_Symmetric_Matrix(B_E_Cinv_Et, iCamera_Count * 6));
		//Solve_Linear_Gause(B_E_Cinv_Et, iCamera_Count * 6, v_E_Cinv_w, Delta_Ksi, &iResult);
		////Disp(Delta_Ksi, 1, iCamera_Count * 6, "Delta_Ksi");

		//Matrix_Multiply(Et, iAdjust_Count * 3, iCamera_Count * 6, Delta_Ksi, 1, pTemp); //Et * Delta Xc
		//Vector_Minus(w, pTemp, iAdjust_Count * 3, pTemp);
		//Matrix_Multiply(C_Inv, iAdjust_Count * 3, iAdjust_Count * 3, pTemp, 1, Delta_Ksi + iCamera_Count * 6);
		////Disp(Delta_Ksi + iCamera_Count * 6, 1, iAdjust_Count * 3, "Delta_Ksi");

		Matrix_Multiply(Delta_Ksi, 1, iJt_w, (_T)-1, Delta_Ksi);
		if (fSum_e_Pre <= fSum_e || !iResult || fSum_e<eps)
			break;

		//每6个一组delta ksi
		for (i = 0; i < iCamera_Count; i++)
		{
			se3_2_SE3(&Delta_Ksi[6 * i], Delta_Pose);	//此处有异议
			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
			//Disp(Camera[i], 4, 4, "Camera");
		}

		//然后轮到调整点的位置
		for (i = 0; i < iAdjust_Count; i++)
		{//
			int iBit = bGet_Bit(pPoint_Adjust_Flag, pPoint_2D[i].m_iPoint_Index);
			//Disp(Camera[pPoint_2D[i].m_iPoint_Index], 4, 4, "Camera");
			if (!iBit)
			{
				//pPoint_3D[i][0] += Delta_Ksi[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 0];
				//pPoint_3D[i][1] += Delta_Ksi[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 1];
				//pPoint_3D[i][2] += Delta_Ksi[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 2];
				
				//好像以下才对
				pPoint_3D[pPoint_2D[i].m_iPoint_Index][0] += Delta_Ksi[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 0];
				pPoint_3D[pPoint_2D[i].m_iPoint_Index][1] += Delta_Ksi[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 1];
				pPoint_3D[pPoint_2D[i].m_iPoint_Index][2] += Delta_Ksi[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 2];

				Set_Bit(pPoint_Adjust_Flag, pPoint_2D[i].m_iPoint_Index);
			}
		}
		fSum_e_Pre = fSum_e;
		//Disp(Camera[0], 4, 4, "Camera");
		printf("iIter:%d %e\n", iIter, 0.5f*fSum_e);
	}
	return;
}

void BA_Test_3_3()
{//不完整后端调整，连点与矩阵一起调。但是应对不了大规模线性方程求解。通用线性方程求解已经走到头
	typedef float _T;
	int iCamera_Count, iPoint_Count, iObservation_Count, i, iIter, iResult;
	float(*pPoint_3D)[3],
		(*pCamera_Data)[3 * 3];    //只是个内参
	Point_2D<_T>* pPoint_2D, oCur_Point;
	//[0 - 2] : angle - axis rotation
	//[3, 4, 5] 位移 
	//[6] focal 焦距
	//[7 - 8] 畸变模型中的k1,k2
	_T Camera[16][4 * 4], Rotation_Vector[4], R[16][3 * 3];
	Temp_Load_File_2(&iCamera_Count, &iPoint_Count, &iObservation_Count, &pPoint_2D, &pPoint_3D, &pCamera_Data);
	const int iAdjust_Count = 3000;   //iObservation_Count;
	Normalize(pPoint_3D, pPoint_2D, iPoint_Count, pCamera_Data, iCamera_Count);

	for (i = 0; i < iCamera_Count; i++)
	{
		Rotation_Vector_3_2_4(pCamera_Data[i], Rotation_Vector);
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R[i]);
		Gen_Homo_Matrix(R[i], &pCamera_Data[i][3], Camera[i]);
	}

	//分析点与相机之间的对应关系
	int Point_Count_Each_Camera[16] = {};
	Schur_Camera_Data<_T> oAll_Camera_Data;
	for (i = 0; i < iAdjust_Count; i++)
		Point_Count_Each_Camera[pPoint_2D[i].m_iCamera_Index]++;

	int w_Jt = 6 * iCamera_Count + iAdjust_Count * 3;   //Sigma_H矩阵的阶
	Sparse_Matrix<_T> oSigma_H, oSigma_JE, oH_Inv, oDelta_X;
	_T* Sigma_JE, fSum_e, Delta_Pose[4 * 4], fSum_e_Pre = (_T)1e20, E[2], Jt[2 * 9], J[9 * 2], JJt[9 * 9], * Delta_X;
	unsigned char* pPoint_Adjust_Flag;

	Init_Sparse_Matrix(&oSigma_H, iCamera_Count * 6 * 6 + iAdjust_Count * (6 * 3 * 2 + 3 * 3), w_Jt, w_Jt);
	Init_Sparse_Matrix(&oH_Inv, oSigma_H.m_iMax_Item_Count * 10, w_Jt, w_Jt);
	Init_Sparse_Matrix(&oSigma_JE, oSigma_H.m_iRow_Count, 1, oSigma_H.m_iRow_Count);
	Init_Sparse_Matrix(&oDelta_X, oSigma_H.m_iRow_Count, 1, oSigma_H.m_iRow_Count);

	Sigma_JE = (_T*)pMalloc(&oMatrix_Mem, w_Jt * sizeof(_T));
	Delta_X = (_T*)pMalloc(&oMatrix_Mem, w_Jt * sizeof(_T));
	pPoint_Adjust_Flag = (unsigned char*)pMalloc(&oMatrix_Mem, (iPoint_Count + 7) >> 3);

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Reset_Sparse_Matrix(&oSigma_H);
		Reset_Sparse_Matrix(&oH_Inv);
		Reset_Sparse_Matrix(&oSigma_JE);
		Init_All_Camera_Data(&oAll_Camera_Data, Point_Count_Each_Camera, iCamera_Count, iAdjust_Count);
		memset(Sigma_JE, 0, w_Jt * sizeof(_T));
		memset(pPoint_Adjust_Flag, 0, (iPoint_Count + 7) >> 3);

		for (i = 0; i < iObservation_Count; i++)
		{
			oCur_Point = pPoint_2D[i];
			_T* pCur_Point_3D = pPoint_3D[oCur_Point.m_iPoint_Index];
			_T Point_3D_1[4], Point_4D[4] = { pCur_Point_3D[0], pCur_Point_3D[1], pCur_Point_3D[2], 1 };
			_T* pCur_Camera = Camera[oCur_Point.m_iCamera_Index];
			_T Temp[4];
			_T focal = pCamera_Data[oCur_Point.m_iCamera_Index][6];
			//此处存在z<0的情况，所以从 I 矩阵出发得不到最优解
			if (Point_4D[2] == 0.0)
				continue;
			if (i >= iAdjust_Count)
				continue;
			Matrix_Multiply(pCur_Camera, 4, 4, Point_4D, 1, Point_3D_1);   //得TP

			Temp[0] = -Point_3D_1[0] / Point_3D_1[2], Temp[1] = -Point_3D_1[1] / Point_3D_1[2];     //投影到归一化平面上
			Temp[0] *= focal, Temp[1] *= focal;           //投影到成像平面上

			E[0] = Temp[0] - oCur_Point.m_Pos[0];	//对应点i的数值差
			E[1] = Temp[1] - oCur_Point.m_Pos[1];
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差

			//构造雅可比
			union {
				_T Jt_TP_Ksi[3 * 6];
				_T Jt_E_Ksi[2 * 6];
			};
			memset(Jt, 0, 2 * 9 * sizeof(_T));
			_T Jt_UV_P[2 * 3], Jt_UV_Porg[2 * 3];
			Get_Drive_UV_P(focal, focal, Point_3D_1, Jt_UV_P);
			Get_Deriv_TP_Ksi_1(pCur_Camera, Point_4D, Jt_TP_Ksi);
			Matrix_Multiply(Jt_UV_P, 2, 3, Jt_TP_Ksi, 6, Jt_E_Ksi);
			Copy_Matrix_Partial(Jt_E_Ksi, 2, 6, Jt, 9, 0, 0);
			//还要求原来的Porg微弱扰动对uv的影响,
			//uv = KTP => ∂uv/∂p = ∂uv/∂p' * ∂p'/∂p
			if (i < iAdjust_Count)
			{
				Matrix_Multiply(Jt_UV_P, 2, 3, R[oCur_Point.m_iCamera_Index], 3, Jt_UV_Porg);
				//为什么此处乘以-1反而误差更大？局部性？
				Copy_Matrix_Partial(Jt_UV_Porg, 2, 3, Jt, 9, 6, 0);
			}

			//Disp(Jt, 2, 9, "Jt");
			Matrix_Multiply(Jt, 2, 9, (_T)-1.f, Jt);
			Matrix_Transpose(Jt, 2, 9, J);
			Transpose_Multiply(J, 9, 2, JJt);

			//将JJt 分陪到相关的矩阵分块中
			Distribute_Data(oAll_Camera_Data, JJt, oCur_Point.m_iCamera_Index, oCur_Point.m_iPoint_Index);

			_T JE[9];
			Matrix_Multiply(J, 9, 2, E, 1, JE);             //JE
			Vector_Add(&Sigma_JE[oCur_Point.m_iCamera_Index * 6], JE, 6, &Sigma_JE[oCur_Point.m_iCamera_Index * 6]);
			Vector_Add(&Sigma_JE[iCamera_Count * 6 + oCur_Point.m_iPoint_Index * 3], &JE[6], 3, &Sigma_JE[iCamera_Count * 6 + oCur_Point.m_iPoint_Index * 3]);
		}

		////将Camera_Data抄到稀疏矩阵
		Copy_Data_2_Sparse(oAll_Camera_Data, &oSigma_H);
		Dense_2_Sparse(Sigma_JE, oSigma_H.m_iRow_Count, 1, &oSigma_JE);

		//方法1，解方程
		Add_I_Matrix(&oSigma_H);
		Solve_Linear_Gause(oSigma_H, Sigma_JE, Delta_X, &iResult);

		////方法2，求逆
		//Get_Inv_Matrix_Row_Op(oSigma_H, &oH_Inv, &iResult);
		//if (!iResult)
		//{
		//    Add_I_Matrix(&oSigma_H);
		//    Get_Inv_Matrix_Row_Op(oSigma_H, &oH_Inv, &iResult);
		//}
		//Matrix_Multiply(oH_Inv, oSigma_JE, &oDelta_X);
		//Sparse_2_Dense(oDelta_X, Delta_X);

		Matrix_Multiply(Delta_X, 1, w_Jt, (_T)-1, Delta_X);
		if (fSum_e_Pre <= fSum_e || !iResult)
			break;

		//每6个一组delta ksi
		for (i = 0; i < iCamera_Count; i++)
		{
			se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
		}

		//然后轮到调整点的位置
		for (i = 0; i < iAdjust_Count; i++)
		{//
			int iBit = bGet_Bit(pPoint_Adjust_Flag, pPoint_2D[i].m_iPoint_Index);
			if (!iBit)
			{
				pPoint_3D[i][0] += Delta_X[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 0];
				pPoint_3D[i][1] += Delta_X[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 1];
				pPoint_3D[i][2] += Delta_X[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 2];
				Set_Bit(pPoint_Adjust_Flag, pPoint_2D[i].m_iPoint_Index);
			}
		}
		fSum_e_Pre = fSum_e;
		printf("iIter:%d %f\n", iIter, fSum_e);
		Free(&oAll_Camera_Data);
	}
	printf("done\n");
	return;
}
void BA_Test_2()
{//尝试自建点云替代样本，检验位姿估计的有效性
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
			//xyzs[i][3] = xyzs[i][2];	// +((iGet_Random_No() % 100) - 50.f) / 100.f;
		}
	}

	_T Ksi[6], Pose_Org[4 * 4],
		K[3 * 3] = { 520.9f,  0.f,    325.1f,     //内参必须有
					0.f,      521.f,  249.7f,
					0.f,      0.f,      1.f };
	_T Rotation_Vector[4] = { 0,1,0,-PI / 6.f }, t[3] = { -100,0,0 };

	//c2w
	Gen_Ksi_by_Rotation_Vector_t(Rotation_Vector, t, Ksi);
	se3_2_SE3(Ksi, Pose_Org);
	Get_Inv_Matrix_Row_Op(Pose_Org, Pose_Org, 4);
	Disp(Pose_Org, 4, 4, "Pose_Org");

	//生成一组点，
	_T Point_3D_1[101][3], Point_2D_2[101][2];
	_T Point_3D_2[4];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(Point_3D_1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(Point_3D_2, Point_3D_1[i], 3 * sizeof(_T));
		Point_3D_2[3] = 1;
		Matrix_Multiply(Pose_Org, 4, 4, Point_3D_2, 1, Point_3D_2);

		////加点噪声，留心观察对位姿的影响，把一下这句注释则可得到精确解
		//for(int j=0;j<3;j++)
		//	Point_3D_2[j] += ((iGet_Random_No() % 100) - 50.f) / 100.f;

		Matrix_Multiply(K, 3, 3, Point_3D_2, 1, Point_3D_2);
		Point_3D_2[0] /= Point_3D_2[2], Point_3D_2[1] /= Point_3D_2[2], Point_3D_2[2] = 1;
		memcpy(Point_2D_2[i], Point_3D_2, 2 * sizeof(_T));
	}

	_T Pose[4 * 4];
	int iResult;
	Bundle_Adjust_3D2D_1(Point_3D_1, Point_2D_2, 100, K, Pose, &iResult);
	Disp(Pose, 4, 4, "胜利来的如此突然");
	return;
}

void Ceres_Test_2()
{//设有RGBD数据图1， RGBD数据图2，求相机2的位姿Rt
	typedef double _T;
	//第一步，装入图1，图2的信息
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	_T(*pPoint_2D_2)[2];
	_T K[3 * 3] = { 520.9f,  0.f,    325.1f,     //内参必须有
					0.f,      521.f,  249.7f,
					0.f,      0.f,      1.f },
		fDepth_Factor = 5000.f;                 //深度图量化参数

	int i, iCount, iResult;
	Temp_Load_File("sample\\7.8.2.bin", &pPoint_3D_1, &pPoint_3D_2, &iCount);
	pPoint_2D_2 = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
	for (i = 0; i < iCount; i++)
		memcpy(pPoint_2D_2[i], pPoint_3D_2[i], 2 * sizeof(_T));
	_T Pose[4 * 4];

	Bundle_Adjust_3D2D_1(pPoint_3D_1, pPoint_2D_2, iCount,K, Pose, &iResult);
	Disp(Pose, 4, 4, "Pose");
	return;
}

void ICP_Test_1()
{//BA解 ICP，假定已经通过某种方法找到匹配点对
	typedef float _T;
	_T(*P_1)[3], (*P_2)[3];
	int iCount;

	Temp_Load_File_1("Sample\\7.10.bin", &P_2, &P_1, &iCount);
	_T Pose[4 * 4];
	int iResult;
	ICP_BA_2_Image_1((_T(*)[3])P_1, (_T(*)[3])P_2, iCount, Pose, &iResult);
	Disp(Pose, 4, 4, "秃然胜利");
	free(P_2);
	return;
}

void ICP_Test_2()
{
	typedef double _T;
	_T(*P_1)[3], P_1_Centroid[3] = { 0 },
		(*P_2)[3], P_2_Centroid[3] = { 0 };
	_T Pose[4 * 4];
	int iCount, iResult;
	Temp_Load_File_1("Sample\\7.10.bin", &P_2, &P_1, &iCount);
	ICP_SVD(P_1, P_2, iCount, Pose, &iResult);
	Disp(Pose, 4, 4, "再下一城");
	free(P_2);
	return;
}

template<typename _T>void Temp_Load_Data(const char* pcFile, _T** ppBuffer)
{
	int i, iCount = (int)iGet_File_Length((char*)pcFile) / 4;
	FILE* pFile = fopen(pcFile, "rb");
	_T* pBuffer = (_T*)malloc(iCount * sizeof(_T));
	for (i = 0; i < iCount; i++)
	{
		float fData;
		fread(&fData, 1, sizeof(float), pFile);
		pBuffer[i] = fData;
	}
	*ppBuffer = pBuffer;
	return;
}

void Optical_Flow_Test_1()
{//目测不靠谱，这玩意不结合 ransac就是一坨屎
	typedef double _T;
	_T(*kp1)[2], (*kp2)[2];
	int iCount, iMatch_Count;
	iCount = (int)iGet_File_Length((char*)"sample\\8.3.bin") / (4 * 2);
	Temp_Load_Data("sample\\8.3.bin", (_T**)&kp1);
	//Temp_Load_Data("sample\\8.3_2.bin", (_T**)&kp2);
	kp2 = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
	Image oImage_1, oImage_2;
	bLoad_Image("D:\\Software\\3rdparty\\slambook2\\ch8\\LK1.bmp", &oImage_1);
	bLoad_Image("D:\\Software\\3rdparty\\slambook2\\ch8\\LK2.bmp", &oImage_2);

	Optical_Flow_1(oImage_1, oImage_2, kp1, kp2, iCount, &iMatch_Count);
	for (int i = 0; i < iCount; i++)
	{
		Draw_Point(oImage_1, (int)kp1[i][0], (int)kp1[i][1]);
		Draw_Point(oImage_2, (int)kp2[i][0], (int)kp2[i][1]);
	}
	bSave_Image("c:\\tmp\\1.bmp", oImage_1);
	bSave_Image("c:\\tmp\\2.bmp", oImage_2);

	free(kp1);
	free(kp2);
	Free_Image(&oImage_1);
	Free_Image(&oImage_2);
}

void ICP_Test_3()
{//还是用BA的方法解ICP问题，这次自建数据，不借助外部数据
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i, iResult;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
		}
	}

	_T Ksi[6], Pose_Org[4 * 4];
	_T Rotation_Vector[4] = { 0,1,0,-PI / 6.f }, t[3] = { -100,0,0 };

	//c2w
	Gen_Ksi_by_Rotation_Vector_t(Rotation_Vector, t, Ksi);
	se3_2_SE3(Ksi, Pose_Org);

	_T P1[101][3], P11[4],
		P2[101][3];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(P1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(P11, P1[i], 3 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose_Org, 4, 4, P11, 1, P2[i]);

		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P2[i][j] += (iGet_Random_No() % 100) / 100.f;
	}

	{//方法1，简单BA，只调整位姿不调整点
		_T Pose[4 * 4];
		ICP_BA_2_Image_1(P1, P2, 100, Pose, &iResult);
		Disp(Pose_Org, 4, 4, "Pose_Org");
		Disp(Pose, 4, 4, "Estimated");
		Disp_Error(P1, P2, 100, Pose);
	}

	{//方法2，简单svd
		_T Pose[4 * 4];
		ICP_SVD(P1, P2, 100, Pose, &iResult);
		Disp(Pose_Org, 4, 4, "Pose_Org");
		Disp(Pose, 4, 4, "Estimated");
		Disp_Error(P1, P2, 100, Pose);
	}

	{//调整原点集位置，一种破坏性的方法
		_T Pose[4 * 4];
		ICP_BA_2_Image_2(P1, P2, 100, Pose, &iResult);
		Disp_Error(P1, P2, 100, Pose);
	}
}

void ICP_Test_4()
{//试一下三个点集的ICP，该实验只是估计位姿，不调整点集
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i, iResult;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
		}
	}
	//造两个相机位姿给点集2，点集3
	_T Pose_Org[2][4 * 4];
	_T Rotation_Vector[2][4] = { { 0,1,0,-PI / 6.f },
									{0,0,1,-PI / 3.f } };
	_T t[2][3] = { { -100,0,0 },
					{ -200,0,0 } };

	//_T Q[4],V[4],R[3*3];
	//Rotation_Vector_2_Matrix(Rotation_Vector[0], R);
	//Rotation_Matrix_2_Vector(R, V);
	//Disp(R, 3, 3, "R");
	//Disp(V, 1, 4, "V");

	Gen_Homo_Matrix_1(Rotation_Vector[0], t[0], Pose_Org[0]);
	Gen_Homo_Matrix_1(Rotation_Vector[1], t[1], Pose_Org[1]);

	//Disp(Pose_Org[0], 4, 4, "Pose_Org_1");
	//Disp(Pose_Org[1], 4, 4, "Pose_Org_2");

	//造三个点集
	_T P1[101][3], P2[101][3], P3[101][3], P11[4];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(P1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(P11, P1[i], 3 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose_Org[0], 4, 4, P11, 1, P2[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P2[i][j] += (iGet_Random_No() % 100) / 100.f;

		Matrix_Multiply(Pose_Org[1], 4, 4, P11, 1, P3[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P3[i][j] += (iGet_Random_No() % 100) / 100.f;
	}

	/*_T Delta_1[4 * 4];
	Get_Delta_Pose(Pose_Org[0], Pose_Org[1], Delta_1);
	Disp_Error(P1, P2, 100, Pose_Org[0]);
	Disp_Error(P1, P3, 100, Pose_Org[1]);
	Disp_Error(P2, P3, 100, Delta_1);*/


	//T12: 相机2相对相机1的位姿  T13: 相机3相对于相机1的位姿，这两个是待求位姿
	//T23: 相机3相对于相机2的位姿，这个是中间位姿，用于求Δx
	_T T12[4 * 4], T13[4 * 4], T23[4 * 4],
		Delta_Pose[2][4 * 4], Pose_Pre[2][4 * 4],
		Jct[4 * 6], Jt[3 * 12], J[12 * 3], JEt[12];
	_T  Sigma_H[12 * 12], fSum_e, fSum_e_Pre = 1e10,
		E[3], H_Inv[12 * 12];

	_T Sigma_JEt[12], H[12 * 12], X[12];
	_T P_Temp[4]; //P1'
	int j, k, iIter;
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T13, 4, 4);

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Get_Delta_Pose(T12, T13, T23);
		/*if (iIter == 1)
		{
			Disp(T12, 4, 4, "T12");
			Disp(T13, 4, 4, "T13");
			Disp(T23, 4, 4, "T23");
		}*/
		memset(Sigma_H, 0, 12 * 12 * sizeof(_T));
		memset(Sigma_JEt, 0, 12 * sizeof(_T));

		for (i = 0; i < 100; i++)
		{
			//先搞搞P1,P2匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T12, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P2[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi_1(T12, P1[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * 12 * sizeof(_T));
			//∂T12P1/∂ 放在前6列
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
				for (k = 0; k < 6; k++)
					Jt[j * 12 + k] = Jct[j * 6 + k];
			Matrix_Multiply(Jt, 3, 12, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, 12, J);
			Matrix_Multiply(J, 12, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, 12, 3, Jt, 12, H);
			Matrix_Add(Sigma_H, H, 12, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, 12, Sigma_JEt);   //∑JE'

			//再搞P1,P3匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T13, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi_1(T13, P1[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * 12 * sizeof(_T));
			//∂T13P1/∂ 放在后6列
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
				for (k = 0; k < 6; k++)
					Jt[j * 12 + 6 + k] = Jct[j * 6 + k];
			Matrix_Multiply(Jt, 3, 12, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, 12, J);
			Matrix_Multiply(J, 12, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, 12, 3, Jt, 12, H);
			Matrix_Add(Sigma_H, H, 12, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, 12, Sigma_JEt);   //∑JE'

			//搞T23，假如还是调整T13，窃以为这一步很重要，否则解决不了累积误差。
			//加了这个调整以后，能形成相机路径闭环
			memcpy(P_Temp, P2[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T23, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];

			Get_Deriv_TP_Ksi_1(T13, P1[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * 12 * sizeof(_T));
			//∂T13P1/∂ 放在后6列
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
				for (k = 0; k < 6; k++)
					Jt[j * 12 + 6 + k] = Jct[j * 6 + k];
			Matrix_Multiply(Jt, 3, 12, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, 12, J);
			Matrix_Multiply(J, 12, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, 12, 3, Jt, 12, H);
			Matrix_Add(Sigma_H, H, 12, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, 12, Sigma_JEt);   //∑JE'

			////以下这步注释掉完全没有影响，收敛还快
			////假如连T12一起也调整了。目前看到这个可有可无，加了对T12的调整以后，收敛慢了，数据基本一致
			////然而这个实验相机太少，无法评价这一步是否需要。
			//Get_Deriv_TP_Ksi(T12, P1[i], Jct);    //∂TP/∂ξ
			//memset(Jt, 0, 3 * 12 * sizeof(_T));
			////∂T12P1/∂ 放在前6列
			//for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
			//    for (k = 0; k < 6; k++)
			//        Jt[j * 12 + k] = Jct[j * 6 + k];
			//Matrix_Multiply(Jt, 3, 12, (_T)-1.f, Jt);    //J'已经到位
			//Matrix_Transpose(Jt, 3, 12, J);
			//Matrix_Multiply(J, 12, 3, E, 1, JEt);    //JE'到位
			//Matrix_Multiply(J, 12, 3, Jt, 12, H);
			//Matrix_Add(Sigma_H, H, 12, Sigma_H);         //∑H, JJ'到位
			//Vector_Add(Sigma_JEt, JEt, 12, Sigma_JEt);   //∑JE'
		}

		Get_Inv_Matrix_Row_Op_2(Sigma_H, H_Inv, 12, &iResult);
		if (!iResult)
		{//此处对∑H的调整是关键，否则很有可能不满秩，注意，此处调整量应该是
			// H + λI，只不过H 没有呈现比较大的数值，故此λ=1已经够用
			_T I[12 * 12];
			Gen_I_Matrix(I, 12, 12);
			Matrix_Add(Sigma_H, I, 12, Sigma_H);
			Get_Inv_Matrix_Row_Op_2(Sigma_H, H_Inv, 12, &iResult);
		}

		//Δx= -(∑H)(-1) * ∑JE'	 (E'为3x1)
		Matrix_Multiply(H_Inv, 12, 12, Sigma_JEt, 1, X);
		Matrix_Multiply(X, 1, 12, (_T)-1, X);

		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;

		//先更新一下Pose_Estimate
		se3_2_SE3(X, Delta_Pose[0]);
		se3_2_SE3(X + 6, Delta_Pose[1]);

		memcpy(Pose_Pre[0], T12, 4 * 4 * sizeof(_T));
		memcpy(Pose_Pre[1], T13, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose[0], 4, 4, T12, 4, T12);
		Matrix_Multiply(Delta_Pose[1], 4, 4, T13, 4, T13);

		printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}

	Get_Delta_Pose(Pose_Pre[0], Pose_Pre[1], T23);
	Disp_Error(P1, P2, 100, Pose_Pre[0]);
	Disp_Error(P1, P3, 100, Pose_Pre[1]);
	Disp_Error(P2, P3, 100, T23);
	return;
}

void ICP_Test_5()
{//最简闭环实验，三个点集的ICP，该实验要做两大尝试，第一，对于闭环的匹配点集尝试一种一般的位姿估计，
//第二，连点集一起调整
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i, iResult;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
		}
	}

	//造两个相机位姿给点集2，点集3
	_T Pose_Org[2][4 * 4];
	_T Rotation_Vector[2][4] = { { 0,1,0,-PI / 6.f },
									{0,1,0,-PI / 3.f } };
	_T t[2][3] = { { -100,0,0 },
					{ -200,0,0 } };

	Gen_Homo_Matrix_1(Rotation_Vector[0], t[0], Pose_Org[0]);
	Gen_Homo_Matrix_1(Rotation_Vector[1], t[1], Pose_Org[1]);

	//造三个点集
	_T P1[101][3], P2[101][3], P3[101][3], P11[4];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(P1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(P11, P1[i], 3 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose_Org[0], 4, 4, P11, 1, P2[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P2[i][j] += (iGet_Random_No() % 100) / 100.f;

		Matrix_Multiply(Pose_Org[1], 4, 4, P11, 1, P3[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P3[i][j] += (iGet_Random_No() % 100) / 100.f;
	}

	//T12: 相机2相对相机1的位姿  T13: 相机3相对于相机1的位姿，这两个是待求位姿
	//T23: 相机3相对于相机2的位姿，这个是中间位姿，用于求Δx
	_T T12[4 * 4], T13[4 * 4], T23[4 * 4], R12[3 * 3], R23[3 * 3],
		Delta_Pose[2][4 * 4], Pose_Pre[2][4 * 4],
		Jct[4 * 6];
	const int w = 18;
	_T Jt[3 * w], J[w * 3], JEt[w];
	_T  Sigma_H[w * w], fSum_e, fSum_e_Pre = 1e10,
		E[3], H_Inv[w * w];

	_T Sigma_JEt[w], H[w * w], X[w];
	_T P_Temp[4]; //P1'
	int j, k, iIter;
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T23, 4, 4);
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, w * w * sizeof(_T));
		memset(Sigma_JEt, 0, w * sizeof(_T));
		//根据T12,T23推出T13，T13= T23 * T12
		Matrix_Multiply(T23, 4, 4, T12, 4, T13);
		Get_R_t(T12, R12);
		Get_R_t(T23, R23);
		for (i = 0; i < 100; i++)
		{
			//先搞搞P1,P2匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T12, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P2[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi_1(T12, P1[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * w * sizeof(_T));
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P
			{//∂T12P1/∂ξ 放在前6列
				for (k = 0; k < 6; k++)
					Jt[j * w + k] = Jct[j * 6 + k];
				//将∂P2/∂P1 放在12-15列
				for (k = 0; k < 3; k++)
					Jt[j * w + 12 + k] = R12[j * 3 + k];
			}
			Matrix_Multiply(Jt, 3, w, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, w, J);
			Matrix_Multiply(J, w, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, w, 3, Jt, w, H);
			Matrix_Add(Sigma_H, H, w, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, w, Sigma_JEt);   //∑JE'

			//再搞P2,P3匹配
			memcpy(P_Temp, P2[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T23, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi_1(T23, P2[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * w * sizeof(_T));
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P
			{//∂T23P2/∂ξ 放在6-11列
				for (k = 0; k < 6; k++)
					Jt[j * w + 6 + k] = Jct[j * 6 + k];
				for (k = 0; k < 3; k++)
					Jt[j * w + 15 + k] = R23[j * 3 + k];
			}
			Matrix_Multiply(Jt, 3, w, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, w, J);
			Matrix_Multiply(J, w, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, w, 3, Jt, w, H);
			Matrix_Add(Sigma_H, H, w, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, w, Sigma_JEt);   //∑JE'

			//第三步，闭环一步，用P1,P3的误差来修正T23
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T13, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi_1(T23, P2[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * w * sizeof(_T));
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P
			{//∂T23P2/∂ξ 放在6-11列
				for (k = 0; k < 6; k++)
					Jt[j * w + 6 + k] = Jct[j * 6 + k];
				//此处也不能少，最后一步闭环不但影响位姿，还要调整点集
				for (k = 0; k < 3; k++)
					Jt[j * w + 15 + k] = R23[j * 3 + k];
			}
			Matrix_Multiply(Jt, 3, w, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, w, J);
			Matrix_Multiply(J, w, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, w, 3, Jt, w, H);
			Matrix_Add(Sigma_H, H, w, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, w, Sigma_JEt);   //∑JE'
		}

		iResult = 0;
		if (!iResult)
		{//此处对∑H的调整是关键，否则很有可能不满秩，注意，此处调整量应该是
			// H + λI，只不过H 没有呈现比较大的数值，故此λ=1已经够用
			_T I[w * w];
			Gen_I_Matrix(I, w, w);
			Matrix_Add(Sigma_H, I, w, Sigma_H);
			Get_Inv_Matrix_Row_Op_2(Sigma_H, H_Inv, w, &iResult);
		}
		//Δx= -(∑H)(-1) * ∑JE'	 (E'为3x1)
		Matrix_Multiply(H_Inv, w, w, Sigma_JEt, 1, X);
		Matrix_Multiply(X, 1, w, (_T)-1, X);
		//if (iIter == 1)
			//Disp(X, 1, w, "X");
		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;

		//先更新一下Pose_Estimate
		se3_2_SE3(X, Delta_Pose[0]);
		se3_2_SE3(X + 6, Delta_Pose[1]);

		memcpy(Pose_Pre[0], T12, 4 * 4 * sizeof(_T));
		memcpy(Pose_Pre[1], T23, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose[0], 4, 4, T12, 4, T12);
		Matrix_Multiply(Delta_Pose[1], 4, 4, T23, 4, T23);

		//继续调整P1,P2
		for (i = 0; i < 100; i++)
		{
			Vector_Add(P1[i], &X[12], 3, P1[i]);
			Vector_Add(P2[i], &X[15], 3, P2[i]);
		}
		printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}

	memcpy(T12, Pose_Pre[0], 4 * 4 * sizeof(_T));
	memcpy(T23, Pose_Pre[1], 4 * 4 * sizeof(_T));
	Matrix_Multiply(T23, 4, 4, T12, 4, T13);

	printf("成功闭环！%f\n",fSum_e_Pre);
	/*Disp_Error(P1, P2, 100, T12);
	Disp_Error(P1, P3, 100, T13);
	Disp_Error(P2, P3, 100, T23);
	Disp(P1[0], 1, 3, "P1");
	Disp(P2[0], 1, 3, "P2");*/
	return;
}
void Transform_Example_2D()
{//用正路方法，二维下的变换

	{//第一个实验，点(0,100)绕原点逆时针旋转30度
		float R[2 * 2];	//齐次变换矩阵，先生成这个矩阵，跟着理论走
		float P1[2], P0[2] = { 100,0 };
		Gen_Rotation_Matrix_2D(R, PI / 6.f);
		Disp(R, 2, 2, "R");
		Matrix_Multiply(R, 2, 2, P0, 1, P1);
		Disp(P1, 2, 1, "旋转后");
	}

	{//第二个实验，位移必须用齐次坐标，就是第三维为1，点(0,0,1)位移(10,10)
		float P1[3], P0[3] = { 0,0,1 };
		float T[3 * 3] = { 0,0, 10,
							0,0, 10,
							0,0, 1 };
		Matrix_Multiply(T, 3, 3, P0, 1, P1);
		Disp(P1, 3, 1, "位移后");
	}
	{//第三个实验，生成一个旋转加位移齐次矩阵
		float R[2 * 2], t[2] = { 10,10 };	//齐次变换矩阵，先生成这个矩阵，跟着理论走
		float P1[3], P0[3] = { 100,0,1 };
		float T[3 * 3];
		Gen_Rotation_Matrix_2D(R, PI / 6.f);
		Gen_Homo_Matrix_2D(R, t, T);
		Disp(T, 3, 3, "齐次变换矩阵");
		Matrix_Multiply(T, 3, 3, P0, 1, P1);
		Disp(P1, 3, 1, "变换后坐标");

		//此处解释T矩阵的动作费解
		Matrix_Multiply(R, 2, 2, P0, 1, P1);
		Vector_Add(P1, t, 2, P1);
		Disp(P1, 2, 1, "P1");	//可见，先旋转后位移。反之不然。因为矩阵乘法不可交换
	}
}
void Sparse_Matrix_Test()
{//稀疏矩阵小试牛刀，试一下三个点集的ICP，该实验只是估计位姿，不调整点集
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i, iResult;
	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
		}
	}

	//造两个相机位姿给点集2，点集3
	_T Pose_Org[2][4 * 4];
	_T Rotation_Vector[2][4] = { { 0,1,0,-PI / 6.f },
									{0,0,1,-PI / 3.f } };
	_T t[2][3] = { { -100,0,0 },
					{ -200,0,0 } };

	Gen_Homo_Matrix_1(Rotation_Vector[0], t[0], Pose_Org[0]);
	Gen_Homo_Matrix_1(Rotation_Vector[1], t[1], Pose_Org[1]);

	//Disp(Pose_Org[0], 4, 4, "Pose_Org_1");
	//Disp(Pose_Org[1], 4, 4, "Pose_Org_2");

	//造三个点集
	_T P1[101][3], P2[101][3], P3[101][3], P11[4];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(P1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(P11, P1[i], 3 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose_Org[0], 4, 4, P11, 1, P2[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P2[i][j] += (iGet_Random_No() % 100) / 100.f;

		Matrix_Multiply(Pose_Org[1], 4, 4, P11, 1, P3[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P3[i][j] += (iGet_Random_No() % 100) / 100.f;
	}

	//T12: 相机2相对相机1的位姿  T13: 相机3相对于相机1的位姿，这两个是待求位姿
	//T23: 相机3相对于相机2的位姿，这个是中间位姿，用于求Δx
	_T T12[4 * 4], T13[4 * 4], T23[4 * 4],
		Delta_Pose[2][4 * 4], Pose_Pre[2][4 * 4],
		Jct[4 * 6];
	_T fSum_e, fSum_e_Pre = 1e10,
		E[3];

	_T/* Sigma_JEt[12],*/ X[12];
	_T P_Temp[4]; //P1'
	Sparse_Matrix<_T> oSigma_JEt, oSigma_H;
	Sparse_Matrix<_T> oJ, oJt, oE, oJEt, oH, oH_Inv, oDelta_X;

	int j, k, iIter;
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T13, 4, 4);
	//Init_Sparse_Matrix(&oSigma_JEt, 12,1,12);
	Init_Sparse_Matrix(&oSigma_H, 12 * 12, 12, 12);
	Init_Sparse_Matrix(&oJt, 3 * 12, 12, 3);
	Init_Sparse_Matrix(&oJ, 12 * 3, 3, 12);
	Init_Sparse_Matrix(&oE, 3 * 1, 1, 3);
	Init_Sparse_Matrix(&oJEt, 12 * 1, 1, 12);
	Init_Sparse_Matrix(&oSigma_JEt, 12, 1, 12);
	Init_Sparse_Matrix(&oH, 12 * 12, 12, 12);
	Init_Sparse_Matrix(&oH_Inv, 12 * 12, 12, 12);
	Init_Sparse_Matrix(&oDelta_X, 1 * 12, 1, 12);
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T13, 4, 4);

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Get_Delta_Pose(T12, T13, T23);
		Reset_Sparse_Matrix(&oSigma_H);
		Reset_Sparse_Matrix(&oSigma_JEt);
		Reset_Sparse_Matrix(&oH_Inv);
		Reset_Sparse_Matrix(&oDelta_X);

		for (i = 0; i < 100; i++)
		{
			//先搞搞P1,P2匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T12, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P2[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi_1(T12, P1[i], Jct);    //?TP/?ξ
			Reset_Sparse_Matrix(&oJt);
			Reset_Sparse_Matrix(&oE);
			//?T12P1/? 放在前6列
			for (j = 0; j < 3; j++)         //将?TP/?ξ与 ?P'/?P 
			{
				for (k = 0; k < 6; k++)
					Set_Value(&oJt, k, j, Jct[j * 6 + k]);
				Set_Value(&oE, 0, j, E[j]);
			}
			Matrix_Multiply(oJt, (_T)-1.f);				//J'已经到位
			Matrix_Transpose_1(oJt, &oJ);
			Matrix_Multiply(oJ, oE, &oJEt);             //JE
			//printf("i:%d %f\n",i, fGet_Value(&oJEt, 0, 4));
			Matrix_Multiply(oJ, oJt, &oH);				//H=JJt			
			Matrix_Add(oSigma_H, oH, &oSigma_H);		//
			Matrix_Add(oSigma_JEt, oJEt, &oSigma_JEt);

			//再搞P1, P3匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T13, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi_1(T13, P1[i], Jct);    //∂TP/∂ξ
			Reset_Sparse_Matrix(&oJt);
			Reset_Sparse_Matrix(&oE);
			//?T12P1/? 放在前6列
			for (j = 0; j < 3; j++)         //将?TP/?ξ与 ?P'/?P 
			{
				for (k = 0; k < 6; k++)
					Set_Value(&oJt, 6 + k, j, Jct[j * 6 + k]);
				Set_Value(&oE, 0, j, E[j]);
			}
			Matrix_Multiply(oJt, (_T)-1.f);				//J'已经到位
			Matrix_Transpose_1(oJt, &oJ);
			Matrix_Multiply(oJ, oE, &oJEt);             //JE
			Matrix_Multiply(oJ, oJt, &oH);				//H=JJt
			Matrix_Add(oSigma_H, oH, &oSigma_H);		//
			Matrix_Add(oSigma_JEt, oJEt, &oSigma_JEt);

			//搞T23，假如还是调整T13，窃以为这一步很重要，否则解决不了累积误差。
			//加了这个调整以后，能形成相机路径闭环
			memcpy(P_Temp, P2[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T23, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];

			Get_Deriv_TP_Ksi_1(T13, P1[i], Jct);    //∂TP/∂ξ
			Reset_Sparse_Matrix(&oJt);
			Reset_Sparse_Matrix(&oE);
			//∂T13P1/∂ 放在后6列
			for (j = 0; j < 3; j++)         //将?TP/?ξ与 ?P'/?P 
			{
				for (k = 0; k < 6; k++)
					Set_Value(&oJt, 6 + k, j, Jct[j * 6 + k]);
				Set_Value(&oE, 0, j, E[j]);
			}
			Matrix_Multiply(oJt, (_T)-1.f);				//J'已经到位
			Matrix_Transpose_1(oJt, &oJ);
			Matrix_Multiply(oJ, oE, &oJEt);             //JE
			Matrix_Multiply(oJ, oJt, &oH);				//H=JJt
			Matrix_Add(oSigma_H, oH, &oSigma_H);		//
			Matrix_Add(oSigma_JEt, oJEt, &oSigma_JEt);
		}
		
		////方法1，解方程
		//Sparse_2_Dense(oSigma_JEt, Sigma_JEt);		
		//Solve_Linear_Gause(oSigma_H, Sigma_JEt, X, &iResult);
		//if (!iResult)
		//{
		//	Add_I_Matrix(&oSigma_H);
		//	Solve_Linear_Gause(oSigma_H, Sigma_JEt, X, &iResult);
		//}
		
		//方法2，Sigma_H求逆
		//Disp(oSigma_H, "Sigma_H");
		Get_Inv_Matrix_Row_Op(oSigma_H, &oH_Inv, &iResult);
		//Disp(oH_Inv, "Inv");
		if (!iResult)
		{
			Add_I_Matrix(&oSigma_H);
			Get_Inv_Matrix_Row_Op(oSigma_H, &oH_Inv, &iResult);
		}
		Matrix_Multiply(oH_Inv, oSigma_JEt, &oDelta_X);
		Sparse_2_Dense(oDelta_X, X);
		
		Matrix_Multiply(X, 1, 12, (_T)-1, X);
		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;

		//先更新一下Pose_Estimate
		se3_2_SE3(X, Delta_Pose[0]);
		se3_2_SE3(X + 6, Delta_Pose[1]);

		memcpy(Pose_Pre[0], T12, 4 * 4 * sizeof(_T));
		memcpy(Pose_Pre[1], T13, 4 * 4 * sizeof(_T));

		Matrix_Multiply(Delta_Pose[0], 4, 4, T12, 4, T12);
		Matrix_Multiply(Delta_Pose[1], 4, 4, T13, 4, T13);

		printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}

	Get_Delta_Pose(Pose_Pre[0], Pose_Pre[1], T23);
	Disp_Error(P1, P2, 100, Pose_Pre[0]);
	Disp_Error(P1, P3, 100, Pose_Pre[1]);
	Disp_Error(P2, P3, 100, T23);

	Free_Sparse_Matrix(&oSigma_JEt);
	Free_Sparse_Matrix(&oH);
	Free_Sparse_Matrix(&oH_Inv);
	Free_Sparse_Matrix(&oSigma_H);
	Free_Sparse_Matrix(&oJt);
	Free_Sparse_Matrix(&oJ);
	Free_Sparse_Matrix(&oE);
	Free_Sparse_Matrix(&oJEt);
	Free_Sparse_Matrix(&oDelta_X);
	Disp_Mem(&oMatrix_Mem,0);
	return;
}

void Sphere_Test_2()
{//4基站定位实验，高斯牛顿法寻找4个球的交点最优解
	typedef float _T;
	/*_T Sphere_Center[4][3] = { { 200,300,400 },
								{ 300,400,410 } ,
								{ 350,300,420 } ,
								{ 200,400,430 } };*/
								//4个无交集的球
	_T Sphere_Center[4][3] = { { 200,300,400 },
								{ 500,300,410 } ,
								{ 550,600,420 } ,
								{ 200,600,430 } };

	_T d[4] = { 100,110,120,130 };	//可视为基站到物体距离
	int i;
	Image oImage;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (i = 0; i < 4; i++)
		Draw_Arc(oImage, (int)d[i], (int)Sphere_Center[i][0], (int)Sphere_Center[i][1]);

	_T P[3] = { 0,0,0 }, Pre_P[3];	//最后的解
	int iIter, iResult;

	_T Jt[1 * 3], J[3 * 1], JJt[3 * 3], Je[3 * 1], H_Inv[3 * 3], Delta_X[3],
		fSum_e, e, fSum_e_Pre = 1e10;
	_T Sigma_H[3 * 3], Sigma_Je[3 * 1];
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		//置零
		memset(Sigma_H, 0, 3 * 3 * sizeof(_T));
		memset(Sigma_Je, 0, 3 * sizeof(_T));

		//到4张皮的距离和最小感觉最好，因为这才是题意
		for (i = 0; i < 4; i++)
		{	//e = [(x-a)^2 + (x-b)^2 + (z-c)^2]^1/2
			_T fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
				(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]) +
				(P[2] - Sphere_Center[i][2]) * (P[2] - Sphere_Center[i][2]);
			e = d[i] - (_T)sqrt(fDist_Sqr);
			fSum_e += abs(e);	// e* e;	//这里就必须用e的平方了，或者用绝对值，因为有正有负。要琢磨一下误差的度量对结果的影响
			//?e/?x = - [(x-a)^2 + (x-b)^2 + (z-a)^2] *(x-a)
			Jt[0] = -(_T)pow(fDist_Sqr, -0.5f) * (P[0] - Sphere_Center[i][0]);
			Jt[1] = -(_T)pow(fDist_Sqr, -0.5f) * (P[1] - Sphere_Center[i][1]);
			Jt[2] = -(_T)pow(fDist_Sqr, -0.5f) * (P[2] - Sphere_Center[i][2]);

			memcpy(J, Jt, 3 * sizeof(_T));
			Matrix_Multiply(J, 3, 1, Jt, 3, JJt);
			Matrix_Multiply(J, 3, 1, e, Je);
			//	//累加
			Matrix_Add(Sigma_H, JJt, 3, Sigma_H);
			Vector_Add(Sigma_Je, Je, 3, Sigma_Je);
		}

		////试一下,到圆心距离和最小
		//for (i = 0; i < 4; i++)
		//{	//e = [(x-a)^2 + (x-b)^2 + (z-c)^2]^1/2
		//	float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]) +
		//		(P[2] - Sphere_Center[i][2]) * (P[2] - Sphere_Center[i][2]);
		//	if (fDist_Sqr == 0)
		//		continue;
		//	e = sqrt(fDist_Sqr);
		//	fSum_e += e;	//此处到底用e还是e的平方？用e表示到点的距离之和，用e*e表示距离的平方和，微妙差别

		//	//?e/?x = [(x-a)^2 + (x-b)^2 + (z-c)^2] *(x-a) 
		//	Jt[0] = pow(fDist_Sqr, -0.5) * (P[0] - Sphere_Center[i][0]);
		//	Jt[1] = pow(fDist_Sqr, -0.5) * (P[1] - Sphere_Center[i][1]);
		//	Jt[2] = pow(fDist_Sqr, -0.5) * (P[2] - Sphere_Center[i][2]);

		//	memcpy(J, Jt, 3 * sizeof(_T));
		//	Matrix_Multiply(J, 3, 1, Jt, 3, JJt);
		//	Matrix_Multiply(J, 3, 1, e, Je);
		//	//	//累加
		//	Matrix_Add(Sigma_H, JJt, 3, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 3, Sigma_Je);
		//}

		//后面全一样，毫无差别毫无营养，搞利索以后改成解方程，
		//会比求个逆可能快一点
		Sigma_H[0] += 1, Sigma_H[4] += 1, Sigma_H[8] += 1;
		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 3, &iResult);
		Matrix_Multiply(H_Inv, 3, 3, Sigma_Je, 1, Delta_X);
		Matrix_Multiply(Delta_X, 3, 1, (_T)-1.f, Delta_X);

		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;
		printf("Iter:%d Error:%f\n", iIter, fSum_e);

		Pre_P[0] = (P[0] += Delta_X[0]);
		Pre_P[1] = (P[1] += Delta_X[1]);
		Pre_P[2] = (P[2] += Delta_X[2]);
		fSum_e_Pre = fSum_e;
	}

	Disp(Pre_P, 1, 3, "Point");
	Draw_Point(oImage, (int)Pre_P[0], (int)Pre_P[1]);
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	Free_Image(&oImage);
}

void Sphere_Test_1()
{//4基站定位实验，高斯牛顿法寻找4个圆的交点最优解
	typedef float _T;
	_T Sphere_Center[4][3] = { { 200,300,400 },
								{ 300,400,410 } ,
								{ 350,300,420 } ,
								{ 200,400,430 } };

	_T R[4] = { 100,110,120,130 };
	Image oImage;
	int i;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (i = 0; i < 4; i++)
		Draw_Arc(oImage, (int)R[i], (int)Sphere_Center[i][0], (int)Sphere_Center[i][1]);

	_T P[3] = { 0,0,0 }, Pre_P[3];	//最后的解
	int iIter, iResult;

	_T Jt[1 * 2], J[2 * 1], JJt[2 * 2], Je[2 * 1], H_Inv[2 * 2], Delta_X[2],
		fSum_e, e, fSum_e_Pre = 1e10;
	_T Sigma_H[2 * 2], Sigma_Je[2 * 1];
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, 2 * 2 * sizeof(_T));
		memset(Sigma_Je, 0, 2 * sizeof(_T));

		//这个条件不好
		//for (i = 0; i < 2; i++)
		//{
		//	e = R[i] * R[i] - (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) -
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
		//	fSum_e += e * e;
		//	//?e/?x= (-2x, -2y)	,第一步顺着这个梯度走，能走到目的地
		//	Jt[0] = -2 * (P[0]-Sphere_Center[i][0]), Jt[1] = -2 * (P[1]-Sphere_Center[i][1]);
		//	//Disp(Jt, 1, 2, "Jt");
		//	memcpy(J, Jt, 2 * sizeof(_T));
		//	//第二步，要求Δx
		//	Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
		//	//Disp(JJt, 2, 2, "JJt");
		//	Matrix_Multiply(J, 2, 1, e, Je);

		//	//累加
		//	Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		//}

		////以下为到4个圆边距离和最近
		//for (i = 0; i < 4; i++)
		//{//e(x) = Ri - [(x-a)^2 + (x-b)^2]^1/2
		//	float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
		//	e= R[i] - sqrt(fDist_Sqr);
		//	fSum_e += e * e;
		//	//?e/?x = - [(x-a)^2 + (x-b)^2] *(x-a)
		//	Jt[0] = -pow(fDist_Sqr,-0.5) * (P[0] - Sphere_Center[i][0]);
		//	Jt[1] = -pow(fDist_Sqr,-0.5) * (P[1] - Sphere_Center[i][1]);

		//	memcpy(J, Jt, 2 * sizeof(_T));
		//	Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
		//	Matrix_Multiply(J, 2, 1, e, Je);
		//	//	//累加
		//	Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		//}

		//试一下,到圆心距离和最小
		for (i = 0; i < 4; i++)
		{
			_T fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
				(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
			if (fDist_Sqr == 0)
				continue;
			e = (_T)sqrt(fDist_Sqr);
			fSum_e += e;
			//?e/?x = [(x-a)^2 + (x-b)^2] *(x-a)
			Jt[0] = (_T)pow(fDist_Sqr, -0.5f) * (P[0] - Sphere_Center[i][0]);
			Jt[1] = (_T)pow(fDist_Sqr, -0.5f) * (P[1] - Sphere_Center[i][1]);

			memcpy(J, Jt, 2 * sizeof(_T));
			Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
			Matrix_Multiply(J, 2, 1, e, Je);
			//	//累加
			Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
			Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		}

		Sigma_H[0] += 1, Sigma_H[3] += 1;
		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 2, &iResult);
		Matrix_Multiply(H_Inv, 2, 2, Sigma_Je, 1, Delta_X);
		Matrix_Multiply(Delta_X, 2, 1, (_T)-1.f, Delta_X);

		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;
		printf("Iter:%d Error:%f\n", iIter, fSum_e);
		//Draw_Point(oImage, P[0], P[1],2);

		Pre_P[0] = (P[0] += Delta_X[0]);
		Pre_P[1] = (P[1] += Delta_X[1]);
		fSum_e_Pre = fSum_e;
	}
	Draw_Point(oImage, (int)Pre_P[0], (int)Pre_P[1], 2);
	Disp(Pre_P, 1, 3, "Point");
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	Free_Image(&oImage);
	return;
}
void Sphere_Test_3()
{//尝试用线性法解
	typedef float _T;
	////4个有交集的球
	//_T Sphere_Center[4][3] = { { 200,300,400 },
	//							{ 300,400,410 } ,
	//							{ 350,300,420 } ,
	//							{ 200,400,430 } };

	//4个无交集的球
	_T Sphere_Center[4][3] = { { 200,300,400 },
								{ 500,300,410 } ,
								{ 550,600,420 } ,
								{ 200,600,430 } };

	_T d[4] = { 100,110,120,130 };	//可视为基站到物体距离
	int i, iResult;
	Image oImage;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (i = 0; i < 4; i++)
		Draw_Arc(oImage, (int)d[i], (int)Sphere_Center[i][0], (int)Sphere_Center[i][1]);
	//bSave_Image("c:\\tmp\\1.bmp", oImage);

	_T A[3 * 3], B[3], X[3];
	for (i = 0; i < 3; i++)
	{
		//(2a1 - 2ai)x + (2b1 - 2bi)y + (2c1 - 2ci)z
		A[i * 3 + 0] = 2 * (Sphere_Center[0][0] - Sphere_Center[1 + i][0]);
		A[i * 3 + 1] = 2 * (Sphere_Center[0][1] - Sphere_Center[1 + i][1]);
		A[i * 3 + 2] = 2 * (Sphere_Center[0][2] - Sphere_Center[1 + i][2]);
		//r2^2 - r1^2 + a1^2 - a2^2 + b1^2 - b2^2 + c1^2 - c2^2
		B[i] = d[i] * d[i] - d[0] * d[0] +
			Sphere_Center[0][0] * Sphere_Center[0][0] - Sphere_Center[1 + i][0] * Sphere_Center[1 + i][0] +
			Sphere_Center[0][1] * Sphere_Center[0][1] - Sphere_Center[1 + i][1] * Sphere_Center[1 + i][1] +
			Sphere_Center[0][2] * Sphere_Center[0][2] - Sphere_Center[1 + i][2] * Sphere_Center[1 + i][2];
	}

	//printf("%d\n", iGet_Rank(A, 3, 3));
	//Solve_Linear_Gause(A, 3, B, X, &iResult);
	Solve_Linear_Contradictory(A, 3, 3, B, X, &iResult);
	Disp(X, 1, 3, "X");
	Draw_Point(oImage, (int)X[0], (int)X[1]);
	bSave_Image("c:\\tmp\\1.bmp", oImage);
		
	return;
}
template<typename _T>_T fGet_Loss(_T Point[2], _T Point_Ref[2],_T focal, _T distort_param_1, _T distort_param_2,_T e[2])
{
	union {
		_T r2;
		_T focal_distortion;
		_T fValue;
	};
	//_T Temp[2];
	r2 = Point[0] * Point[0] + Point[1] * Point[1];
	focal_distortion =focal*( 1.f + r2 * (distort_param_1 + distort_param_2 * r2));

	//Point[0] *= focal_distortion;
	//Point[1] *= focal_distortion;
	e[0] = Point[0] * focal_distortion - Point_Ref[0];
	e[1] = Point[1] * focal_distortion - Point_Ref[1];

	fValue = sqrt(e[0] * e[0] + e[1] * e[1]);
	fValue = 2.0 * fValue - 1.f;
	return fValue*0.5f;
}
void Schur_Test()
{//做一个Schur消元实验，这个实验依然有缺陷，迭代的结果太粗糙，怀疑没有用到鲁棒和函数
	typedef double _T;
	//先把数据装入，还是用第九讲数据
	int iCamera_Count, iPoint_Count, iObservation_Count, i, iIter, iResult;
	_T(*pPoint_3D)[3],
		(*pCamera_Data)[3 * 3];		//相机参数
									//0,1,2参数为旋转向量
									//3,4,5为位移向量
									//6为焦距
									//7,8为即便参数

	Point_2D<_T>* pPoint_2D, oCur_Point;
	_T Camera[16][4 * 4], Rotation_Vector[4], R[16][3 * 3];
	Temp_Load_File_2(&iCamera_Count, &iPoint_Count, &iObservation_Count, &pPoint_2D, &pPoint_3D, &pCamera_Data);
	const int iAdjust_Count = iObservation_Count;
	Normalize(pPoint_3D, pPoint_2D, iPoint_Count, pCamera_Data, iCamera_Count);
	for (i = 0; i < iCamera_Count; i++)
	{
		Rotation_Vector_3_2_4(pCamera_Data[i], Rotation_Vector);
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R[i]);
		Gen_Homo_Matrix(R[i], &pCamera_Data[i][3], Camera[i]);
	}
	//分析点与相机之间的对应关系
	int Point_Count_Each_Camera[16] = {}, w_Jt = 6 * iCamera_Count + iAdjust_Count * 3;   //Sigma_H矩阵的阶
	_T* Sigma_JE, fSum_e, Delta_Pose[4 * 4], fSum_e_Pre = (_T)1e20,
		E[2], Jt[2 * 9], J[9 * 2], JJt[9 * 9], Temp[4], * Delta_X;
	unsigned char* pPoint_Adjust_Flag;
	Schur_Camera_Data<_T> oAll_Camera_Data;
	for (i = 0; i < iAdjust_Count; i++)
		Point_Count_Each_Camera[pPoint_2D[i].m_iCamera_Index]++;

	Sigma_JE = (_T*)pMalloc(&oMatrix_Mem, w_Jt * sizeof(_T));
	pPoint_Adjust_Flag = (unsigned char*)pMalloc(&oMatrix_Mem, (iPoint_Count + 7) >> 3);
	Delta_X = (_T*)pMalloc(&oMatrix_Mem, w_Jt * sizeof(_T));
	//Disp_Mem(&oMatrix_Mem, 0);
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Init_All_Camera_Data(&oAll_Camera_Data, Point_Count_Each_Camera, iCamera_Count, Min(iPoint_Count, iAdjust_Count));
		memset(Sigma_JE, 0, w_Jt * sizeof(_T));
		memset(pPoint_Adjust_Flag, 0, (iPoint_Count + 7) >> 3);
		for (i = 0; i < iObservation_Count; i++)
		{
			oCur_Point = pPoint_2D[i];
			_T* pCur_Point_3D = pPoint_3D[oCur_Point.m_iPoint_Index];
			_T Point_3D_1[4], Point_4D[4] = { pCur_Point_3D[0], pCur_Point_3D[1], pCur_Point_3D[2], 1 };
			_T* pCur_Camera = Camera[oCur_Point.m_iCamera_Index];
			_T focal = pCamera_Data[oCur_Point.m_iCamera_Index][6];
			//此处存在z<0的情况，所以从 I 矩阵出发得不到最优解
			if (Point_4D[2] == 0.0)
				continue;
			if (i >= iAdjust_Count)
				continue;
			
				
			/*Disp(Point_4D, 1, 4);
			Disp(pCamera_Data[oCur_Point.m_iCamera_Index], 9, 1);*/
			Matrix_Multiply(pCur_Camera, 4, 4, Point_4D, 1, Point_3D_1);   //得TP
			
			Temp[0] = -Point_3D_1[0] / Point_3D_1[2], Temp[1] = -Point_3D_1[1] / Point_3D_1[2];     //投影到归一化平面上
			//if (i == iObservation_Count - 1)
			//{
			//	fGet_Loss(Temp,oCur_Point.m_Pos, pCamera_Data[oCur_Point.m_iCamera_Index][6],
			//		pCamera_Data[oCur_Point.m_iCamera_Index][7], pCamera_Data[oCur_Point.m_iCamera_Index][8],E);

			//	/*Disp(pCamera_Data[oCur_Point.m_iCamera_Index], 6, 1, "Camera");
			//	Disp(Point_4D, 1, 4, "Point");
			//	Disp(Temp, 1, 2, "Dest");*/
			//}

			Temp[0] *= focal, Temp[1] *= focal;           //投影到成像平面上的uv
			E[0] = Temp[0] - oCur_Point.m_Pos[0];	//对应点i的数值差
			E[1] = Temp[1] - oCur_Point.m_Pos[1];
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差

			/*fSum_e +=fGet_Loss(Temp,oCur_Point.m_Pos, pCamera_Data[oCur_Point.m_iCamera_Index][6],
				pCamera_Data[oCur_Point.m_iCamera_Index][7], pCamera_Data[oCur_Point.m_iCamera_Index][8],E);*/

			//printf("Point:%d err:%f\n", oCur_Point.m_iPoint_Index, E[0] * E[0] + E[1] * E[1]);

			//构造雅可比
			union {
				_T Jt_TP_Ksi[3 * 6];
				_T Jt_E_Ksi[2 * 6];
			};
			memset(Jt, 0, 2 * 9 * sizeof(_T));
			_T Jt_UV_P[2 * 3], Jt_UV_Porg[2 * 3];
			//感觉以下两个倒数要加上即便影响
			Get_Drive_UV_P(focal, focal, Point_3D_1, Jt_UV_P);
			Get_Deriv_TP_Ksi_1(pCur_Camera, Point_4D, Jt_TP_Ksi);
			Matrix_Multiply(Jt_UV_P, 2, 3, Jt_TP_Ksi, 6, Jt_E_Ksi);
			Copy_Matrix_Partial(Jt_E_Ksi, 2, 6, Jt, 9, 0, 0);

			//还要求原来的Porg微弱扰动对uv的影响,
			//uv = KTP => ∂uv/∂p = ∂uv/∂p' * ∂p'/∂p
			if (i < iAdjust_Count)
			{
				Matrix_Multiply(Jt_UV_P, 2, 3, R[oCur_Point.m_iCamera_Index], 3, Jt_UV_Porg);
				//为什么此处乘以-1反而误差更大？局部性？
				Copy_Matrix_Partial(Jt_UV_Porg, 2, 3, Jt, 9, 6, 0);
			}

			//Disp(Jt, 2, 9, "Jt");
			Matrix_Multiply(Jt, 2, 9, (_T)-1.f, Jt);
			Matrix_Transpose(Jt, 2, 9, J);
			Transpose_Multiply(J, 9, 2, JJt);

			//将JJt 分陪到相关的矩阵分块中
			Distribute_Data(oAll_Camera_Data, JJt, oCur_Point.m_iCamera_Index, oCur_Point.m_iPoint_Index);
			_T JE[9];
			Matrix_Multiply(J, 9, 2, E, 1, JE);             //JE
			Vector_Add(&Sigma_JE[oCur_Point.m_iCamera_Index * 6], JE, 6, &Sigma_JE[oCur_Point.m_iCamera_Index * 6]);
			Vector_Add(&Sigma_JE[iCamera_Count * 6 + oCur_Point.m_iPoint_Index * 3], &JE[6], 3, &Sigma_JE[iCamera_Count * 6 + oCur_Point.m_iPoint_Index * 3]);
		}
		//Disp(Sigma_JE, 1, w_Jt, "Sigma_JE");
		if (fSum_e_Pre <= fSum_e)
			break;

		//出来以后数据全准备好了，可以schur消元了
		Solve_Linear_Schur(oAll_Camera_Data, Sigma_JE, Delta_X, &iResult);

		Free(&oAll_Camera_Data);
		if (!iResult)
			break;
		//Disp_Mem(&oMatrix_Mem, 0);
		Matrix_Multiply(Delta_X, 1, w_Jt, (_T)-1, Delta_X);

		//每6个一组delta ksi
		for (i = 0; i < iCamera_Count; i++)
		{
			se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
		}

		//然后轮到调整点的位置
		for (i = 0; i < Min(iAdjust_Count, iPoint_Count); i++)
		{//
			int iBit = bGet_Bit(pPoint_Adjust_Flag, pPoint_2D[i].m_iPoint_Index);
			if (!iBit)
			{
				pPoint_3D[i][0] += Delta_X[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 0];
				pPoint_3D[i][1] += Delta_X[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 1];
				pPoint_3D[i][2] += Delta_X[iCamera_Count * 6 + pPoint_2D[i].m_iPoint_Index * 3 + 2];
				Set_Bit(pPoint_Adjust_Flag, pPoint_2D[i].m_iPoint_Index);
			}
		}
		fSum_e_Pre = fSum_e;
		printf("iIter:%d %f\n", iIter, fSum_e);
	}
	return;
}

template<typename _T> int bTemp_Load_Data(const char* pcFile, _T(**ppT)[7], int* piPoint_Count,
	Measurement<_T>** ppMeasurement, int* piMeasure_Count)
{//装入2500个点
	_T(*pPose_7)[7];
	Measurement<_T>* pMeasurement;
	int i, iResult, iPoint_Count = 2500;
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
		return 0;

	pPose_7 = (_T(*)[7])pMalloc(&oMatrix_Mem, 4000 * 7 * sizeof(_T));
	//for (i = 0; i < iPoint_Count; i++)
	for (i = 0; ; i++)
	{
		float Pose[7];
		int iCur;
		if ((iResult = fscanf(pFile, "VERTEX_SE3:QUAT %d %f %f %f %f %f %f %f ", &iCur, &Pose[0], &Pose[1], &Pose[2], &Pose[4], &Pose[5], &Pose[6], &Pose[3])) < 8)
			break;
		for (int j = 0; j < 7; j++)
			pPose_7[i][j] = Pose[j];
	}
	iPoint_Count = i;
	Shrink(&oMatrix_Mem, pPose_7, iPoint_Count * 7 * sizeof(_T));

	//再装入观察数据
	pMeasurement = (Measurement<_T>*)pMalloc(&oMatrix_Mem, 11000 * sizeof(Measurement<_T>));
	for (i = 0;; i++)
	{
		int j, iCur;
		Measurement<_T>* poM = &pMeasurement[i];
		float Pose[7];
		if ((iResult = fscanf(pFile, "EDGE_SE3:QUAT %d %d %f %f %f %f %f %f %f ", &poM->m_Camera_Index[0], &poM->m_Camera_Index[1],
			&Pose[0], &Pose[1], &Pose[2],
			&Pose[4], &Pose[5], &Pose[6], &Pose[3])) < 9)
			break;
		for (j = 0; j < 7; j++)
			poM->Delta_ksi[j] = Pose[j];
		//Disp(&poM->Delta_ksi[3], 4, 1);
		int Information[] = { 10000,0, 0, 0, 0, 0, 10000, 0, 0, 0, 0, 10000, 0, 0, 0, 40000, 0, 0, 40000, 0, 40000 };
		for (j = 0; j < 21; j++)
		{
			fscanf(pFile, "%d ", &iCur);
			if (iCur != Information[j])
				printf("error");
		}
	}
	Shrink(&oMatrix_Mem, pMeasurement, i * sizeof(Measurement<_T>));
	fclose(pFile);

	*piPoint_Count = iPoint_Count;
	*ppT = pPose_7;
	*ppMeasurement = pMeasurement;
	*piMeasure_Count = i;
	return 1;
}

void Pose_Graph_Test_1()
{//位姿图优化，两大问题尚未解决，1，鲁棒性函数（删点）;2,性能
//但是已经可以收敛，目测比原模型光滑。但是，这只是一个粗暴的方法，只做全局最优化，没有引入信息矩阵
//故此还没完全收敛成一个光滑的球

	typedef float _T;
	int i, j, iResult, iMeasurement_Count, iCamera_Count;
	_T(*pKsi)[7], (*Camera)[4 * 4];
	Measurement<_T>* pMeasurement;
	Pose_Graph_Sigma_H<_T> oPose_Graph;
	iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iCamera_Count, &pMeasurement, &iMeasurement_Count);
	iMeasurement_Count = iMeasurement_Count;
	iCamera_Count = 2500;
	Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iCamera_Count * 4 * 4 * sizeof(_T));
	//int iAdjust_Count = iMeasurement_Count;
	for (i = 0; i < iCamera_Count; i++)
		TQ_2_Rt(pKsi[i], Camera[i]);
	Free(&oMatrix_Mem, pKsi);
	Init_Pose_Graph(pMeasurement, iMeasurement_Count, iCamera_Count, &oPose_Graph);

	int iIter;
	union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
	union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
	union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
	_T E_4x4[4 * 4], E_6[6], J_Inv[6 * 6], Adj[6 * 6],
		Jt[12 * 6], J[6 * 12], H[12 * 12], JEt[12], Delta_Pose[4 * 4], Temp[6 * 6],
		* Sigma_JEt, * Delta_X;
	_T fSum_e, e, fSum_e_Pre = (_T)1e10;
	Sparse_Matrix<_T> oSigma_H;
	Sigma_JEt = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
	Delta_X = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
	
	//Disp_Mem(&oMatrix_Mem, 0);
	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(&oMatrix_Mem, iCamera_Count * 3 * sizeof(_T));

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Reset_Pose_Graph(oPose_Graph);
		Init_Sparse_Matrix(&oSigma_H, oPose_Graph.m_iCamera_Data_Count * 3 * 6 * 6, iCamera_Count * 6, iCamera_Count * 6);
		memset(Sigma_JEt, 0, iCamera_Count * 6 * sizeof(_T));
		for (i = 0; i < iMeasurement_Count; i++)
		{//没一个测量都带来一个调整权重
			Measurement<_T> oM = pMeasurement[i];
			if (oM.m_Camera_Index[0] >= iCamera_Count || oM.m_Camera_Index[1] >= iCamera_Count)
				continue;
			memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
			memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
			TQ_2_Rt(oM.Delta_ksi, M_4x4);
			Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
			if (!iResult)break;
			Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
			if (!iResult)break;

			//注意，书上是错的
			Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);	
			Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);			//=Tij(-1) * Ti(-1) * Tj
			SE3_2_se3(E_4x4, E_6);

			for (e = 0, j = 0; j < 3; j++)
				e += E_6[j] * E_6[j];
			fSum_e += e;

			//接着求雅可比
			Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
			Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
			if (!iResult)break;

			Get_Adj(Tj_Inv, Adj);
			//此时形成两个雅可比, Ti的雅可比放在Jt的0-5列，Tj的雅可比放在Jt的6-11列
			//第一个雅可比
			Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
			//∂eij/∂ξj
			Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 6, 0);

			//∂eij/∂ξi
			Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
			Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 0, 0);

			Matrix_Transpose(Jt, 6, 12, J);
			Matrix_Multiply(J, 12, 6, Jt, 12, H);

			//此处要把H矩阵散发到稀疏矩阵Sigma_H中去
			Distribute_Data(oPose_Graph, H, oM.m_Camera_Index[0], oM.m_Camera_Index[1]);
			//Disp(H, 12, 12, "H");
			//轮到搞JEt
			Matrix_Multiply(J, 12, 6, E_6, 1, JEt);      //JE'到位
			Vector_Add(&Sigma_JEt[oM.m_Camera_Index[0] * 6], JEt, 6, &Sigma_JEt[oM.m_Camera_Index[0] * 6]);
			Vector_Add(&Sigma_JEt[oM.m_Camera_Index[1] * 6], &JEt[6], 6, &Sigma_JEt[oM.m_Camera_Index[1] * 6]);
		}
		printf("iIter:%d %f\n", iIter, fSum_e);
		if (fSum_e_Pre <= fSum_e)
			break;

		Copy_Data_2_Sparse(oPose_Graph, &oSigma_H);

		Add_I_Matrix(&oSigma_H, &iResult, 1.f);
		Compact_Sparse_Matrix(&oSigma_H);
		unsigned long long tStart = iGet_Tick_Count();
		Solve_Linear_Gause_1(oSigma_H, Sigma_JEt, Delta_X, &iResult);
		printf("%lld\n", iGet_Tick_Count() - tStart);

		Free_Sparse_Matrix(&oSigma_H);
		Matrix_Multiply(Delta_X, 1, iCamera_Count * 6, (_T)-1, Delta_X);
		if (!iResult)
			break;
		for (i = 0; i < iCamera_Count; i++)
		{
			//Disp(Delta_X, 6,1);
			se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
			pPoint_3D[i][0] = Camera[i][3];
			pPoint_3D[i][1] = Camera[i][7];
			pPoint_3D[i][2] = Camera[i][11];
		}
		char File[256];
		sprintf(File, "c:\\tmp\\%d.ply", iIter);
		bSave_PLY(File, pPoint_3D, iCamera_Count);
		fSum_e_Pre = fSum_e;
	}
	printf("Last error:%f\n", fSum_e);
	return;
}

void Pose_Graph_Test_2()
{//引入信息矩阵，已经按照书本搞，但是还是无卵用。算法值得保留，因为首次要修改ramda
	typedef double _T;
	int i, iResult, iMeasurement_Count, iCamera_Count;
	_T(*pKsi)[7], (*Camera)[4 * 4];
	Measurement<_T>* pMeasurement;
	Pose_Graph_Sigma_H<_T> oPose_Graph;
	iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iCamera_Count, &pMeasurement, &iMeasurement_Count);
	iMeasurement_Count = iMeasurement_Count;
	iCamera_Count = 2500;
	Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iCamera_Count * 4 * 4 * sizeof(_T));
	for (i = 0; i < iCamera_Count; i++)
		TQ_2_Rt(pKsi[i], Camera[i]);

	Free(&oMatrix_Mem, pKsi);
	Init_Pose_Graph(pMeasurement, iMeasurement_Count, iCamera_Count, &oPose_Graph);

	int iIter;
	union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
	union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
	union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
	_T E_4x4[4 * 4], E_6[6], J_Inv[6 * 6], Adj[6 * 6],
		Jt[12 * 6], J[6 * 12], H[12 * 12], Delta_Pose[4 * 4], Temp[6 * 6],
		* Sigma_J_Z_Inv_E, * Delta_X;
	_T fSum_e, e, fSum_e_Pre = (_T)1e20;
	Sparse_Matrix<_T> oSigma_H;
	Sigma_J_Z_Inv_E = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
	Delta_X = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));

	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(&oMatrix_Mem, iCamera_Count * 3 * sizeof(_T));
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Reset_Pose_Graph(oPose_Graph);
		Init_Sparse_Matrix(&oSigma_H, oPose_Graph.m_iCamera_Data_Count * 3 * 6 * 6, iCamera_Count * 6, iCamera_Count * 6);
		memset(Sigma_J_Z_Inv_E, 0, iCamera_Count * 6 * sizeof(_T));
		for (i = 0; i < iMeasurement_Count; i++)
		{//没一个测量都带来一个调整权重
			Measurement<_T> oM = pMeasurement[i];
			if (oM.m_Camera_Index[0] >= iCamera_Count || oM.m_Camera_Index[1] >= iCamera_Count)
				continue;
			memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
			memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
			TQ_2_Rt(oM.Delta_ksi, M_4x4);
			Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
			if (!iResult)break;
			Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
			if (!iResult)break;

			//注意，书上是错的
			Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);
			Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);			//=Tij(-1) * Ti(-1) * Tj

			SE3_2_se3(E_4x4, E_6);
			//Disp(E_6, 6, 1);
			int j;
			for (e = 0, j = 0; j < 3; j++)
				e += E_6[j] * E_6[j];
			fSum_e += e;

			//printf("Sum_e:%.10f\n", fSum_e);
			//接着求雅可比
			Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
			Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
			if (!iResult)break;

			Get_Adj(Tj_Inv, Adj);
			//此时形成两个雅可比, Ti的雅可比放在Jt的0-5列，Tj的雅可比放在Jt的6-11列
			//第一个雅可比
			Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
			//∂eij/∂ξj
			Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 6, 0);

			//∂eij/∂ξi  算第二个雅可比
			Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
			Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 0, 0);
			Matrix_Transpose(Jt, 6, 12, J);
			//到此， J, Jt已经就绪，第一个第二个雅可比都紧凑放在Jt中

			//注意了，这里求的Z^-1 不是 H矩阵，H=JJt, 然而，Z^-1 = JtJ，矩阵是不可交换的！
			//所有的网上结论都是错的
			union {
				_T Z_Inv[6 * 6];
				_T J_Z_Inv[12 * 6];
				_T J_Z_Inv_E[12 * 1];
			};
			
			//先求H=J * Z^-1 * Jt
			Matrix_Multiply(Jt, 6, 12, J, 6, Z_Inv);        //=Z^-1 = JtJ

			//_T Temp[1 * 6];
			/*Matrix_Multiply(E_6, 1, 6, Z_Inv, 6, Temp);
			Matrix_Multiply(Temp, 1, 6, E_6, 1, &e);
			fSum_e += e;*/

			Matrix_Multiply(J, 12, 6, Z_Inv, 6, J_Z_Inv);   //=J*Z^-1

			//H=J * Z^-1 * Jt   相当于取平方
			Matrix_Multiply(J_Z_Inv, 12, 6, Jt, 12, H);
			//此处要把H矩阵散发到稀疏矩阵Sigma_H中去
			Distribute_Data(oPose_Graph, H, oM.m_Camera_Index[0], oM.m_Camera_Index[1]);

			//再求 J * Z^1 * E
			Matrix_Multiply(J_Z_Inv, 12, 6, E_6, 1, J_Z_Inv_E);
			Vector_Add(&Sigma_J_Z_Inv_E[oM.m_Camera_Index[0] * 6], J_Z_Inv_E, 6, &Sigma_J_Z_Inv_E[oM.m_Camera_Index[0] * 6]);
			Vector_Add(&Sigma_J_Z_Inv_E[oM.m_Camera_Index[1] * 6], &J_Z_Inv_E[6], 6, &Sigma_J_Z_Inv_E[oM.m_Camera_Index[1] * 6]);

		}
		printf("iIter:%d %.10f\n", iIter, fSum_e);
		if (fSum_e_Pre <= fSum_e)
			break;

		Copy_Data_2_Sparse(oPose_Graph, &oSigma_H);
		Add_I_Matrix(&oSigma_H, &iResult, (_T)500.f);   //此处终于需要改变ramda值了！
		Compact_Sparse_Matrix(&oSigma_H);
		unsigned long long tStart = iGet_Tick_Count();
		Solve_Linear_Gause_1(oSigma_H, Sigma_J_Z_Inv_E, Delta_X, &iResult);
		printf("%lld\n", iGet_Tick_Count() - tStart);

		Free_Sparse_Matrix(&oSigma_H);
		Matrix_Multiply(Delta_X, 1, iCamera_Count * 6, (_T)-1, Delta_X);
		if (!iResult)
			break;

		if (!iResult)
			break;
		for (i = 0; i < iCamera_Count; i++)
		{
			se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
			pPoint_3D[i][0] = Camera[i][3];
			pPoint_3D[i][1] = Camera[i][7];
			pPoint_3D[i][2] = Camera[i][11];
		}

		char File[256];
		sprintf(File, "c:\\tmp\\%d.ply", iIter);
		bSave_PLY(File, pPoint_3D, iCamera_Count);
		fSum_e_Pre = fSum_e;
		fSum_e_Pre = fSum_e;
	}
}
void Pose_Graph_Test_3()
{//引入信息矩阵，已经按照书本搞，但是还是无卵用。算法值得保留，因为首次要修改ramda
	typedef double _T;
	int i, iResult, iMeasurement_Count, iCamera_Count;
	_T(*pKsi)[7], (*Camera)[4 * 4];
	Measurement<_T>* pMeasurement;
	Pose_Graph_Sigma_H<_T> oPose_Graph;
	iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iCamera_Count, &pMeasurement, &iMeasurement_Count);
	iMeasurement_Count = iMeasurement_Count;
	iCamera_Count = 2500;
	Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iCamera_Count * 4 * 4 * sizeof(_T));
	for (i = 0; i < iCamera_Count; i++)
		TQ_2_Rt(pKsi[i], Camera[i]);

	Free(&oMatrix_Mem, pKsi);
	Init_Pose_Graph(pMeasurement, iMeasurement_Count, iCamera_Count, &oPose_Graph);

	int iIter;
	union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
	union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
	union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
	_T E_4x4[4 * 4], E_6[6], J_Inv[6 * 6], Adj[6 * 6],
		Jt[12 * 6], J[6 * 12], H[12 * 12], Delta_Pose[4 * 4], Temp[6 * 6],
		* Sigma_J_Z_Inv_E, * Delta_X;
	_T fSum_e, e, fSum_e_Pre = (_T)1e20;
	Sparse_Matrix<_T> oSigma_H;
	Sigma_J_Z_Inv_E = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
	Delta_X = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));

	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(&oMatrix_Mem, iCamera_Count * 3 * sizeof(_T));
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Reset_Pose_Graph(oPose_Graph);
		Init_Sparse_Matrix(&oSigma_H, oPose_Graph.m_iCamera_Data_Count * 3 * 6 * 6, iCamera_Count * 6, iCamera_Count * 6);
		memset(Sigma_J_Z_Inv_E, 0, iCamera_Count * 6 * sizeof(_T));
		for (i = 0; i < iMeasurement_Count; i++)
		{//没一个测量都带来一个调整权重
			Measurement<_T> oM = pMeasurement[i];
			if (oM.m_Camera_Index[0] >= iCamera_Count || oM.m_Camera_Index[1] >= iCamera_Count)
				continue;
			memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
			memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
			TQ_2_Rt(oM.Delta_ksi, M_4x4);
			Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
			if (!iResult)break;
			Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
			if (!iResult)break;

			//注意，书上是错的
			Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);
			Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);			//=Tij(-1) * Ti(-1) * Tj

			SE3_2_se3(E_4x4, E_6);
			//Disp(E_6, 6, 1);
			int j;
			for (e = 0, j = 0; j < 3; j++)
				e += E_6[j] * E_6[j];
			/*if (e > 10)
				printf("%f\n", e);*/

			//fSum_e += e;

			//printf("Sum_e:%.10f\n", fSum_e);
			//接着求雅可比
			Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
			Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
			if (!iResult)break;

			Get_Adj(Tj_Inv, Adj);
			//此时形成两个雅可比, Ti的雅可比放在Jt的0-5列，Tj的雅可比放在Jt的6-11列
			//第一个雅可比
			Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
			//∂eij/∂ξj
			Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 6, 0);

			//∂eij/∂ξi  算第二个雅可比
			Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
			Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 0, 0);
			Matrix_Transpose(Jt, 6, 12, J);
			//到此， J, Jt已经就绪，第一个第二个雅可比都紧凑放在Jt中

			//注意了，这里求的Z^-1 不是 H矩阵，H=JJt, 然而，Z^-1 = JtJ，矩阵是不可交换的！
			//所有的网上结论都是错的
			union {
				_T Z_Inv[6 * 6];
				_T J_Z_Inv[12 * 6];
				_T J_Z_Inv_E[12 * 1];
			};
			_T Infomation[] = { 10000,0, 0, 0, 0, 0,
				0, 10000, 0, 0, 0, 0,
				0, 0, 10000, 0, 0, 0,
				0, 0, 0, 40000, 0, 0,
				0, 0, 0, 0, 40000, 0,
				0, 0, 0, 0, 0, 40000 };
			_T Temp[1 * 6];
			//先求H=J * Z^-1 * Jt
			//Matrix_Multiply(Jt, 6, 12, J, 6, Z_Inv);        //=Z^-1 = JtJ
			memcpy(Z_Inv, Infomation, 6 * 6 * sizeof(_T));
			//Get_Inv_Matrix_Row_Op_2(Infomation, Z_Inv, 6, &iResult);

			//Matrix_Multiply(E_6, 1, 6, Z_Inv, 6, Temp);
			//Matrix_Multiply(Temp, 1, 6, E_6, 1, &e);
			Matrix_Multiply(Z_Inv, 6, 6, E_6, 1, Temp);
			Matrix_Multiply(E_6, 1, 6, Temp, 1, &e);
			fSum_e += e;
			/*int j;
			for (e = 0, j = 0; j < 6; j++)
				e += Temp[j] * Temp[j];
			fSum_e += e;*/

			Matrix_Multiply(J, 12, 6, Z_Inv, 6, J_Z_Inv);   //=J*Z^-1

			//H=J * Z^-1 * Jt   相当于取平方
			Matrix_Multiply(J_Z_Inv, 12, 6, Jt, 12, H);
			//此处要把H矩阵散发到稀疏矩阵Sigma_H中去
			Distribute_Data(oPose_Graph, H, oM.m_Camera_Index[0], oM.m_Camera_Index[1]);

			//再求 J * Z^1 * E
			Matrix_Multiply(J_Z_Inv, 12, 6, E_6, 1, J_Z_Inv_E);
			
			Vector_Add(&Sigma_J_Z_Inv_E[oM.m_Camera_Index[0] * 6], J_Z_Inv_E, 6, &Sigma_J_Z_Inv_E[oM.m_Camera_Index[0] * 6]);
			Vector_Add(&Sigma_J_Z_Inv_E[oM.m_Camera_Index[1] * 6], &J_Z_Inv_E[6], 6, &Sigma_J_Z_Inv_E[oM.m_Camera_Index[1] * 6]);
		}
		printf("iIter:%d %.10f\n", iIter, fSum_e);
		if (fSum_e_Pre <= fSum_e)
			break;

		Copy_Data_2_Sparse(oPose_Graph, &oSigma_H);
		Add_I_Matrix(&oSigma_H, &iResult, (_T)1000);   //此处终于需要改变ramda值了！
		Compact_Sparse_Matrix(&oSigma_H);
		unsigned long long tStart = iGet_Tick_Count();
		Solve_Linear_Gause_1(oSigma_H, Sigma_J_Z_Inv_E, Delta_X, &iResult);
		//Disp(Delta_X, 10, 1);
		printf("%lld\n", iGet_Tick_Count() - tStart);

//		Free_Sparse_Matrix(&oSigma_H);
		Matrix_Multiply(Delta_X, 1, iCamera_Count * 6, (_T)-1, Delta_X);
		if (!iResult)
			break;

		if (!iResult)
			break;
		for (i = 0; i < iCamera_Count; i++)
		{
			se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
			Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
			pPoint_3D[i][0] = Camera[i][3];
			pPoint_3D[i][1] = Camera[i][7];
			pPoint_3D[i][2] = Camera[i][11];
		}

		char File[256];
		sprintf(File, "c:\\tmp\\%d.ply", iIter);
		bSave_PLY(File, pPoint_3D, iCamera_Count);
		fSum_e_Pre = fSum_e;
		fSum_e_Pre = fSum_e;
	}
}

template<typename _T>int bTemp_Load_Data(const char* pcFile, _T Pose[][4 * 4])
{
	_T Temp[7];
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
		return 0;
	int iResult;
	for (int i = 0; i < 5; i++)
	{
		iResult = fscanf(pFile, "%f %f %f %f %f %f %f", &Temp[0], &Temp[1], &Temp[2], &Temp[4], &Temp[5], &Temp[6], &Temp[3]);
		TQ_2_Rt(Temp, Pose[i]);
	}
	fclose(pFile);
	return 1;
}
void Point_Cloud_Test()
{//一个最没营养的例子，已知深度图，内参，相机位姿，求一团点云
	typedef float _T;
	_T Pose[5][4 * 4];
	int x, y, i, iPos, iResult, iWidth = 640, iHeight = 480;
	//读入位姿
	if (!bTemp_Load_Data("D:\\Software\\3rdparty\\slambook2\\ch12\\dense_RGBD\\data\\pose.txt", Pose))
		return;

	Image oImage;
	char File[256];
	_T(*Point[5])[3], (*pPoint)[3];
	unsigned char(*Color[5])[3], (*pColor)[3];

	//转入数据
	for (i = 1; i <= 5; i++)
	{
		sprintf(File, "c:\\tmp\\temp\\%d.bmp", i);
		bLoad_Image(File, &oImage);
		iWidth = oImage.m_iWidth, iHeight = oImage.m_iHeight;
		pColor = Color[i - 1] = (unsigned char(*)[3])pMalloc(&oMatrix_Mem, iWidth * iHeight * 3);
		for (y = 0; y < oImage.m_iHeight; y++)
		{
			for (x = 0; x < oImage.m_iWidth; x++)
			{
				iPos = y * oImage.m_iWidth + x;
				pColor[iPos][0] = oImage.m_pChannel[0][iPos];
				pColor[iPos][1] = oImage.m_pChannel[1][iPos];
				pColor[iPos][2] = oImage.m_pChannel[2][iPos];
			}
		}
		Free_Image(&oImage);

		pPoint = Point[i - 1] = (_T(*)[3])pMalloc(&oMatrix_Mem, iWidth * iHeight * 3 * sizeof(_T));
		sprintf(File, "c:\\tmp\\temp\\%d.dat", i);
		FILE* pFile = fopen(File, "rb");
		unsigned short iValue;
		for (iPos = 0; iPos < iWidth * iHeight; iPos++)
		{
			iResult = (int)fread(&iValue, 2, 1, pFile);
			pPoint[iPos][2] =(_T)((unsigned short)(iValue << 8) + (unsigned short)(iValue >> 8));
			/*if (pPoint[iPos][2] > (1 << 15))
				printf("Here");*/
		}
		fclose(pFile);
		printf("%d\n", i);
	}

	_T cx = 319.5f;
	_T cy = 239.5f;
	_T fx = 481.2f;
	_T fy = -480.0f;
	_T depthScale = 5000.0f;

	_T(*pPoint_Cloud_Geo)[3] = (_T(*)[3])pMalloc(&oMatrix_Mem, iWidth * iHeight * 5 * 3 * sizeof(_T));
	unsigned char(*pPoint_Cloud_Color)[3] = (unsigned char(*)[3])pMalloc(&oMatrix_Mem, iWidth * iHeight * 5 * 3);
	int iPoint_Count = 0;
	for (i = 0; i < 5; i++)
	{
		pPoint = Point[i];
		_T* pPose = Pose[i];
		_T Point_4[4];
		Point_4[3] = 1;
		for (iPos = y = 0; y < iHeight; y++)
		{
			for (x = 0; x < iWidth; x++, iPos++)
			{
				if (pPoint[iPos][2] == 0)
					continue;
				Point_4[2] = pPoint[iPos][2] / depthScale;
				Point_4[0] = (x - cx) * Point_4[2] / fx;
				Point_4[1] = (y - cy) * Point_4[2] / fy;

				Matrix_Multiply(pPose, 4, 4, Point_4, 1, pPoint_Cloud_Geo[iPoint_Count]);
				memcpy(pPoint_Cloud_Color[iPoint_Count], Color[i][iPos], 3);
				iPoint_Count++;
			}
		}
	}
	//那些下采样，滤波什么的没啥营养，暂时不要，单看下面就可以
	bSave_PLY("c:\\tmp\\1.ply", pPoint_Cloud_Geo, iPoint_Count, pPoint_Cloud_Color);

	return;
}
void Cholosky_Test_1()
{//以简单堆成性质构造正定矩阵失败，还得老老实实先搞个对角矩阵A，再搞一堆特征向量X，再用相似矩阵原理来搞
	typedef float _T;
	//用逆推的方法构造正定矩阵看看
	//_T LLt[4 * 4], Temp[4 * 4], L[] = { 1,0,0,0,  //非正定
	//            2,3,0,0,
	//            0,4,5,0,
	//            0,2,8,10 };
	const int iDim = 10;
	_T* pLLt, * pTemp, * pL;
	int y, x, iResult;
	pL = (_T*)pMalloc(iDim * iDim);
	pLLt = (_T*)pMalloc(iDim * iDim);
	pTemp = (_T*)pMalloc(iDim * iDim);
	for (y = 0; y < iDim; y++)
		for (x = 0; x < iDim; x++)
			pL[y * iDim + x] = (_T)((x > y) ? 0 : iRandom() % 100);

	//Disp(pL, iDim, iDim, "L");
	Transpose_Multiply(pL, iDim, iDim, pLLt);
	Cholosky_Decompose(pLLt, iDim, pTemp, &iResult);

	Sparse_Matrix<_T> oLLt, oB;
	Init_Sparse_Matrix(&oLLt, iDim * iDim, iDim, iDim);
	Dense_2_Sparse(pLLt, iDim, iDim, &oLLt);
	Cholosky_Decompose(oLLt, &oB, &iResult);

	Free(pL);
	Free(pLLt);
	Free(pTemp);
	Free_Sparse_Matrix(&oLLt);
	Free_Sparse_Matrix(&oB);
	//Disp(oB, "B");
}

void H_Test_4()
{//原版想搞个张定友标定实验，发觉条件不够
	typedef double _T;
	//验证以下K矩阵的逆
	_T K_Inv[3 * 3], K[3 * 3] = { 1000,0,960, //搞一个内参
				 0, 1000,540,
				0,  0,  1 };
	int i, iCount, iResult;

	Get_K_Inv(K, K_Inv);
	_T(*pPoint_3D_0)[3], (*pPoint_3D_1)[3], (*pPoint_3D_2)[3], (*pPoint_3D_3)[3];
	_T(*pUV_0)[2], (*pUV_1)[2], (*pUV_2)[2], (*pUV_3)[2];
	_T T1[4 * 4], T2[4 * 4], T3[4 * 4];
	_T A[6 * 5], X[5];    //试一下列6条式子，求5个变量，用svd搞

	Ransac_Report Report[3];

	Gen_Plane_z0(&pPoint_3D_0, &iCount);
	pPoint_3D_1 = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
	pPoint_3D_2 = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
	pPoint_3D_3 = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
	pUV_0 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	pUV_1 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	pUV_2 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	pUV_3 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));


	Gen_Pose(T1, (_T)0.f, (_T)1.f, (_T)0.f, (_T)PI / 6, (_T)0.f, (_T)0.f, (_T)-1000.f);
	Gen_Pose(T2, (_T)0.f, (_T)1.f, (_T)0.f, (_T)PI / 7, (_T)0.f, (_T)0.f, (_T)-1000.f);
	Gen_Pose(T3, (_T)0.f, (_T)1.f, (_T)0.f, (_T)PI / 8, (_T)0.f, (_T)0.f, (_T)-1000.f);

	for (i = 0; i < iCount; i++)
	{
		pUV_0[i][0] = pPoint_3D_0[i][0];
		pUV_0[i][1] = pPoint_3D_0[i][1];

		_T Temp[4] = { pPoint_3D_0[i][0], pPoint_3D_0[i][1], pPoint_3D_0[i][2], 1.f };
		Matrix_Multiply(T1, 4, 4, Temp, 1, Temp);
		memcpy(pPoint_3D_1[i], Temp, 3 * sizeof(_T));
		Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
		pUV_1[i][0] = Temp[0] / Temp[2], pUV_1[i][1] = Temp[1] / Temp[2];

		memcpy(Temp, pPoint_3D_0[i], 3 * sizeof(_T));
		Temp[3] = 1.f;
		Matrix_Multiply(T2, 4, 4, Temp, 1, Temp);
		memcpy(pPoint_3D_2[i], Temp, 3 * sizeof(_T));
		Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
		pUV_2[i][0] = Temp[0] / Temp[2], pUV_2[i][1] = Temp[1] / Temp[2];

		memcpy(Temp, pPoint_3D_0[i], 3 * sizeof(_T));
		Temp[3] = 1.f;
		Matrix_Multiply(T3, 4, 4, Temp, 1, Temp);
		memcpy(pPoint_3D_3[i], Temp, 3 * sizeof(_T));
		Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
		pUV_3[i][0] = Temp[0] / Temp[2], pUV_3[i][1] = Temp[1] / Temp[2];
	}

	Ransac_Estimate_H(pUV_0, pUV_1, iCount, &Report[0], &oMatrix_Mem);
	Ransac_Estimate_H(pUV_0, pUV_2, iCount, &Report[1], &oMatrix_Mem);
	Ransac_Estimate_H(pUV_0, pUV_3, iCount, &Report[2], &oMatrix_Mem);
	if (!Report[0].m_bSuccess || !Report[1].m_bSuccess || !Report[2].m_bSuccess)
		return;

	_T* H, B[3 * 3];
	Transpose_Multiply(K_Inv, 3, 3, B, 0);
	Disp(B, 3, 3, "B");
	B[1] = B[2], B[2] = B[4], B[3] = B[5], B[4] = B[8];
	B[5] = B[6] = B[7] = B[8] = 0;
	Disp(B, 9, 1, "B");

	for (i = 0; i < 3; i++)
	{
		//开始列式，先根据1,2列
	//    hi1 hj1 		'	* 	b11	
	//    hi1 hj3 + hi3 hj1		b13
	//    hi2 hj2				b22
	//    hi2 hj3 + hi3 hj2		b23
	//    hi3 hj3				b33
		H = (_T*)Report[i].m_Modal;
		//Disp(H, 3, 3, "H");
		//第一条v12 * b =0
		A[(i * 2) * 5 + 0] = H[0] * H[1];                 //h11 * h21
		A[(i * 2) * 5 + 1] = H[0] * H[7] + H[6] * H[1];   //h11 * h23 + h13*h21
		A[(i * 2) * 5 + 2] = H[3] * H[4];                 //h12*h22
		A[(i * 2) * 5 + 3] = H[3] * H[7] + H[6] * H[4];   //h12 * h23 + h13*h22
		A[(i * 2) * 5 + 4] = H[6] * H[7];                 //h13*h23
		//第二条(v11 - v22) *b =0
		A[((i * 2) + 1) * 5 + 0] = H[0] * H[0] - H[1] * H[1];//h11*h11 - h21*h21
		A[((i * 2) + 1) * 5 + 1] = H[0] * H[6] + H[6] * H[0] - H[1] * H[7] - H[7] * H[1];//h11*h13+h13*h11   -(h21*h23 + h23*h21)
		A[((i * 2) + 1) * 5 + 2] = H[3] * H[3] - H[2] * H[2];//h12*h12   - h22*h22
		A[((i * 2) + 1) * 5 + 3] = H[3] * H[6] + H[6] * H[3] - H[4] * H[7] - H[7] * H[4];//h12*h13 + h13*h12 - (h22*h23+h23*h22)
		A[((i * 2) + 1) * 5 + 4] = H[6] * H[6] - H[7] * H[7];//h13*h13 - h23*h23

		printf("Dot:%f\n", fDot(&A[(i * 2) * 5 + 0], B, 5));
		printf("Dot:%f\n", fDot(&A[((i * 2) + 1) * 5 + 0], B, 5));
	}

	SVD_Info oSVD;
	SVD_Alloc<_T>(6, 5, &oSVD);
	svd_3((_T*)A, oSVD, &iResult);

	//Vt的最后一行就是解
	memcpy(X, &((_T*)oSVD.Vt)[4 * 5], 5 * sizeof(_T));
	Disp(X, 5, 1, "X");

	/*printf("Dot:%f\n", fDot(&A[0], X, 5));
	printf("Dot:%f\n", fDot(&A[5], X, 5));
	printf("Dot:%f\n", fDot(&A[10], X, 5));
	printf("Dot:%f\n", fDot(&A[15], X, 5));
	printf("Dot:%f\n", fDot(&A[20], X, 5));
	printf("Dot:%f\n", fDot(&A[25], X, 5));*/


	//再算K
	_T K_Estimate[3 * 3] = { 0 };
	K_Estimate[0] = sqrt(1.f / X[0]);
	K_Estimate[4] = sqrt(1.f / X[2]);
	K_Estimate[2] = -X[1] / X[0];
	K_Estimate[5] = -X[3] / X[2];
	//这里做不下去了，显然搞不定scale

	Disp(A, 6, 5, "A");

	Free(pPoint_3D_0), Free(pPoint_3D_1);
	Free(pUV_0), Free(pUV_1);
	Free_Report(Report[0], &oMatrix_Mem);

	return;
}

static void H_Test_1()
{//试一下从像素平面的角度观察
	int i, iCount,iResult;
	typedef float _T;
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3], Point_4[4];
	_T T[4 * 4], R[3 * 3];      //相机外参
	_T K[3 * 3] = { 1000,0,960, //搞一个内参
				 0, 1000,540,
				0,  0,  1 };
	_T t[] = { 10,0,0 };

	{//生成一个相机外参，表示从点集1 -> 点集2
		_T Rotation_Vector[] = { 0,1,0,PI / 6 };
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, t, T);
		Get_Inv_Matrix_Row_Op_2(T, T, 4, &iResult); //w2c
		Disp(T, 4, 4, "T");
	}

	//生成两组空间平面上的点集
	Gen_Plane(&pPoint_3D_1, &iCount, K, T);
	pPoint_3D_2 = (_T(*)[3])pMalloc((iCount * 3 + 1) * sizeof(_T));
	Point_4[3] = 1;
	for (i = 0; i < iCount; i++)
	{
		memcpy(Point_4, pPoint_3D_1[i], 3 * sizeof(_T));
		Matrix_Multiply(T, 4, 4, Point_4, 1, pPoint_3D_2[i]);
	}

	_T  uv[3];
	//以下一段单纯为了投影到像素平面上看看效果
	{
		Image oImage;
		Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 8);
		Set_Color(oImage);

		for (i = 0; i < iCount; i++)
		{
			if (pPoint_3D_1[i][2] < 0)
				continue;
			Matrix_Multiply(K, 3, 3, pPoint_3D_1[i], 1, uv);
			uv[0] /= pPoint_3D_1[i][2], uv[1] /= pPoint_3D_1[i][2], uv[2] = 1;
			//printf("i:%d %f %f\n",i, uv[0], uv[1]);
			if (uv[0] >= 0 && uv[0] < oImage.m_iWidth &&
				uv[1] >= 0 && uv[1] < oImage.m_iHeight)
				Draw_Point(oImage, (int)uv[0], (int)uv[1]);
		}
		bSave_Image("c:\\tmp\\1.bmp", oImage);
	}

	{//试一下两个点集投影到归一化平面上进行匹配
		_T(*pNorm_Point_1)[2] = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
		_T(*pNorm_Point_2)[2] = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
		Ransac_Report oReport;
		for (i = 0; i < iCount; i++)
		{
			//搞归一化平面1上的点集1
			pNorm_Point_1[i][0] = pPoint_3D_1[i][0] / pPoint_3D_1[i][2];
			pNorm_Point_1[i][1] = pPoint_3D_1[i][1] / pPoint_3D_1[i][2];

			//搞归一化平面2上的点集
			pNorm_Point_2[i][0] = pPoint_3D_2[i][0] / pPoint_3D_2[i][2];
			pNorm_Point_2[i][1] = pPoint_3D_2[i][1] / pPoint_3D_2[i][2];
		}

		Ransac_Estimate_H(pNorm_Point_1, pNorm_Point_2, iCount, &oReport, &oMatrix_Mem);
		//以下可以验算一下两个归一化平面是否有： p2 ≌ H p1，注意，此处是尺度意义的相等，
		//数据非常漂亮
		for (i = 0; i < oReport.m_oSupport.m_iInlier_Count; i++)
		{
			if (oReport.m_pInlier_Mask[i])
			{
				_T Temp[3] = { pNorm_Point_1[i][0],pNorm_Point_1[i][1],1 };
				Matrix_Multiply((_T*)oReport.m_Modal, 3, 3, Temp, 1, Temp);
				printf("p2:%f %f\t", pNorm_Point_2[i][0], pNorm_Point_2[i][1]);
				//注意，此处要除以z，这正是尺度等价的意义
				printf("%f %f\n", Temp[0] / Temp[2], Temp[1] / Temp[2]);
			}
		}
		Free_Report(oReport, &oMatrix_Mem);
		Free(pNorm_Point_1), Free(pNorm_Point_2);
	}

	{
		//试一下在像素平面上搞一下，p1,p2为像素平面I1,I2的匹配点对
		int iPixel_Count = 0;
		Ransac_Report oReport;
		_T(*pUV_1)[2] = (_T(*)[2])pMalloc((iCount * 2 + 1) * sizeof(_T)),
			(*pUV_2)[2] = (_T(*)[2])pMalloc((iCount * 2 + 1) * sizeof(_T));
		for (i = 0; i < iCount; i++)
		{
			_T Temp[4];
			Matrix_Multiply(K, 3, 3, pPoint_3D_1[i], 1, Temp);
			if (Temp[2] < 0)
				continue;
			Temp[0] /= Temp[2], Temp[1] /= Temp[2];
			if (Temp[0] < 0 || Temp[0] >= 1920 ||
				Temp[1] < 0 || Temp[1] >= 1080)
				continue;
			pUV_1[iPixel_Count][0] = Temp[0];
			pUV_1[iPixel_Count][1] = Temp[1];

			Matrix_Multiply(K, 3, 3, pPoint_3D_2[i], 1, Temp);
			if (Temp[2] < 0)
				continue;
			Temp[0] /= Temp[2], Temp[1] /= Temp[2];
			if (Temp[0] < 0 || Temp[0] >= 1920 ||
				Temp[1] < 0 || Temp[1] >= 1080)
				continue;
			pUV_2[iPixel_Count][0] = Temp[0];
			pUV_2[iPixel_Count][1] = Temp[1];
			iPixel_Count++;
		}

		//实验结果，在像素平面上，有p2 ≌ H p1，
		Ransac_Estimate_H(pUV_1, pUV_2, iPixel_Count, &oReport, &oMatrix_Mem);
		for (i = 0; i < oReport.m_oSupport.m_iInlier_Count; i++)
		{
			if (oReport.m_pInlier_Mask[i])
			{
				_T Temp[3] = { pUV_1[i][0],pUV_1[i][1],1 };
				Matrix_Multiply((_T*)oReport.m_Modal, 3, 3, Temp, 1, Temp);
				printf("p2:%f %f\t", pUV_2[i][0], pUV_2[i][1]);
				//注意，此处要除以z，这正是尺度等价的意义
				printf("%f %f\n", Temp[0] / Temp[2], Temp[1] / Temp[2]);
			}
		}

		//再实验验证 p2≌ H*KP，数据漂亮！
		for (i = 0; i < oReport.m_oSupport.m_iInlier_Count; i++)
		{
			_T Temp[3];
			Matrix_Multiply(K, 3, 3, pPoint_3D_1[i], 1, Temp);
			Matrix_Multiply((_T*)oReport.m_Modal, 3, 3, Temp, 1, Temp);
			printf("p2:%f %f\t", pUV_2[i][0], pUV_2[i][1]);
			printf("%f %f\n", Temp[0] / Temp[2], Temp[1] / Temp[2]);
		}
		Free_Report(oReport, &oMatrix_Mem);
		Free(pUV_1), Free(pUV_2);
	}
	Free(pPoint_3D_1), Free(pPoint_3D_2);
	return;
}

void H_Test_3()
{//验证像素平面上的H矩阵
	typedef float _T;
	//第一步，另建一个空间平面 z=0
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	int i, iCount, iResult;
	Gen_Plane_z0(&pPoint_3D_1, &iCount);
	pPoint_3D_2 = (_T(*)[3])pMalloc((iCount * 3 + 1) * sizeof(_T));
	_T T[4 * 4], R[3 * 3], t[] = { 10,0,-1000 };      //相机外参
	{//生成一个相机外参，表示从点集1 -> 点集2
		_T Rotation_Vector[] = { 0,1,0,PI / 6 };
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, t, T);                   //c2w
		Get_Inv_Matrix_Row_Op_2(T, T, 4, &iResult); //w2c
		Disp(T, 4, 4, "T");
	}
	for (i = 0; i < iCount; i++)
	{
		_T Temp[4] = { pPoint_3D_1[i][0], pPoint_3D_1[i][1], pPoint_3D_1[i][2], 1.f };
		Matrix_Multiply(T, 4, 4, Temp, 1, pPoint_3D_2[i]);;
	}
	Disp((_T*)pPoint_3D_2, iCount, 3);

	_T K[3 * 3] = { 1000,0,960, //搞一个内参
				 0, 1000,540,
				0,  0,  1 };
	_T(*pUV_1)[2] = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	_T(*pUV_2)[2] = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	for (i = 0; i < iCount; i++)
	{
		_T Temp[3];
		pUV_1[i][0] = pPoint_3D_1[i][0];
		pUV_1[i][1] = pPoint_3D_1[i][1];

		Matrix_Multiply(K, 3, 3, pPoint_3D_2[i], 1, Temp);
		pUV_2[i][0] = Temp[0] / Temp[2], pUV_2[i][1] = Temp[1] / Temp[2];
	}
	Ransac_Report oReport;
	//完美估计
	Ransac_Estimate_H(pUV_1, pUV_2, iCount, &oReport, &oMatrix_Mem);

	//此处关键，想验证张定友标定法中的H矩阵
	_T H[3 * 3];    //再从张正友标定法推导一个H出来，两相比较
	_T T1[3 * 3] = { T[0],T[1],T[3],
					T[4],T[5],T[7],
					T[8],T[9],T[11] };
	Matrix_Multiply(K, 3, 3, T1, 3, H);
	//从以下可以看出，两者数值不一样，但是两者可以只差一个scale
	Disp((_T*)oReport.m_Modal, 3, 3, "Modal");
	Disp(H, 3, 3, "H");
	for (i = 0; i < 9; i++) //从以下看出，两者非常接近了，符合猜想
		printf("H%d: %f %f\n", i, H[i], ((_T*)oReport.m_Modal)[i] * H[0] / ((_T*)oReport.m_Modal)[0]);

	//验证 p2 ≌ HP, 其中为z=0平面上的点
	for (i = 0; i < iCount; i++)
	{
		//此处一定要注意，虽然在z=0平面上，此处用以计算的坐标必须是 (x,y,1)
		_T Temp[3] = { pPoint_3D_1[i][0],pPoint_3D_1[i][1],1.f };
		Matrix_Multiply(H, 3, 3, Temp, 1, Temp);
		printf("p2:%f %f\t", pUV_2[i][0], pUV_2[i][1]);
		printf("%f %f\n", Temp[0] / Temp[2], Temp[1] / Temp[2]);
	}

	printf("数据完全一致。故此张定友标定法中的H矩阵是一般H矩阵的一个特例\n");
	//bSave_PLY("c:\\tmp\\1.ply", pPoint_3D_1, iCount);

	return;
}

void H_Test_2()
{//搞个球，此时点不在同一个平面上，看看H矩阵还存在不存在
//反面例子，这个例子揭示了非共面空间点估计不出一个H矩阵
	typedef float _T;
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	int i, iCount,iResult;
	_T T[4 * 4], R[3 * 3], t[] = { 10,0,0 };      //相机外参
	_T K[3 * 3] = { 1000,0,960, //搞一个内参
				 0, 1000,540,
				0,  0,  1 };

	Gen_Sphere(&pPoint_3D_1, &iCount, 100.f);
	for (i = 0; i < iCount; i++)
		pPoint_3D_1[i][2] += 500;

	{//生成一个相机外参，表示从点集1 -> 点集2
		_T Rotation_Vector[] = { 0,1,0,PI / 6 };
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, t, T);
		Get_Inv_Matrix_Row_Op_2(T, T, 4, &iResult); //w2c
		Disp(T, 4, 4, "T");
	}
	pPoint_3D_2 = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
	for (i = 0; i < iCount; i++)
	{
		_T Temp[4] = { pPoint_3D_1[i][0], pPoint_3D_1[i][1], pPoint_3D_1[i][2], 1.f };
		Matrix_Multiply(T, 4, 4, Temp, 1, pPoint_3D_2[i]);
	}
	_T(*pUV_1)[2] = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	_T(*pUV_2)[2] = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
	Ransac_Report oReport;

	{//首先在归一化平面上搞搞看行不行，最纯粹
		//数据显示，貌似等价，然而误差大了不少
		for (i = 0; i < iCount; i++)
		{
			pUV_1[i][0] = pPoint_3D_1[i][0] / pPoint_3D_1[i][2];
			pUV_1[i][1] = pPoint_3D_1[i][1] / pPoint_3D_1[i][2];

			pUV_2[i][0] = pPoint_3D_2[i][0] / pPoint_3D_2[i][2];
			pUV_2[i][1] = pPoint_3D_2[i][1] / pPoint_3D_2[i][2];
		}
		Ransac_Estimate_H(pUV_1, pUV_2, iCount, &oReport, &oMatrix_Mem);
		for (i = 0; i < iCount; i++)
		{
			if (oReport.m_pInlier_Mask[i])
			{
				_T Temp[3] = { pUV_1[i][0],pUV_1[i][1],1.f };
				Matrix_Multiply((_T*)oReport.m_Modal, 3, 3, Temp, 1, Temp);
				printf("p2:%f %f\t", pUV_2[i][0], pUV_2[i][1]);
				printf("%f %f\n", Temp[0] / Temp[2], Temp[1] / Temp[2]);
			}
		}
		Free_Report(oReport, &oMatrix_Mem);
	}

	//再轮到像素平面上搞
	{
		for (i = 0; i < iCount; i++)
		{
			_T Temp[4] = { pPoint_3D_2[i][0],pPoint_3D_2[i][1],pPoint_3D_2[i][2],1.f };
			Matrix_Multiply(T, 4, 4, Temp, 1, Temp);
			Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
			pUV_2[i][0] = Temp[0] / Temp[2], pUV_2[i][1] = Temp[1] / Temp[2];

			Matrix_Multiply(K, 3, 3, pPoint_3D_1[i], 1, Temp);
			pUV_1[i][0] = Temp[0] / Temp[2], pUV_1[i][1] = Temp[1] / Temp[2];
		}
		//秘密就在这里，显然，估计H矩阵的效果很差，只有1/4的点符合变换阵
		//可见H矩阵选哟共面点
		Ransac_Estimate_H(pUV_1, pUV_2, iCount, &oReport, &oMatrix_Mem);
		printf("Only %d/%d points matched\n", oReport.m_oSupport.m_iInlier_Count, iCount);
		//而E矩阵则丝滑估计，100%命中
		Ransac_Estimate_E(pUV_1, pUV_2, iCount, K[0], K[2], K[5], &oReport, &oMatrix_Mem);
	}

	bSave_PLY("c:\\tmp\\1.ply", pPoint_3D_2, iCount);
	return;
}

const float fx = 481.2f;       // 相机内参
const float fy = -480.0f;
const float cx = 319.5f;
const float cy = 239.5f;

#define Project(x,y,Point) \
{ Point[0] = ((x) - cx) / fx;Point[1] = ((y) - cy) / fy; Point[2] = 1; }
#define Project_Inv(Point,x,y) \
{ \
    x = Point[0] * fx / Point[2] + cx; \
    y = Point[1] * fy / Point[2] + cy; \
}

#define Interpolate(oImage, x, y,fValue)    \
{   \
    int iPos = ((int)y) * oImage.m_iWidth + (int)x; \
    unsigned char* pd = &oImage.m_pChannel[0][iPos]; \
    _T xx = x - floor(x); \
    _T yy = y - floor(y); \
    fValue= _T(((1 - xx) * (1 - yy) * _T(pd[0]) + \
        xx * (1 - yy) * _T(pd[1]) + \
        (1 - xx) * yy * _T(pd[oImage.m_iWidth]) + \
        xx * yy * _T(pd[oImage.m_iWidth + 1])) / 255.f); \
}

template<typename _T>_T NCC(Image oRef, Image oCur, int x1, int y1, _T x2, _T y2)
{//未拆完，怀疑就是个块比较
	const int ncc_window_size = 3;
	const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);
	_T /*values_ref[ncc_area],*/ values_curr[ncc_area];
	int i, x, y, iPos;
	_T /*value_ref,*/ value_curr,/*mean_ref = 0,*/ mean_curr = 0;
	unsigned char values_ref[ncc_area], value_ref;
	union {
		unsigned int mean_ref_int = 0;
		_T mean_ref;
	};
	for (i = 0, y = -ncc_window_size; y <= ncc_window_size; y++)
	{
		iPos = ((int)(y + y1)) * oRef.m_iWidth - ncc_window_size + (int)x1;
		for (x = -ncc_window_size; x <= ncc_window_size; x++, i++, iPos++)
		{
			//value_curr = fInterpolate(oCur, x2 + x, y2 + y);
			Interpolate(oCur, x2 + x, y2 + y, value_curr);

			value_ref = oRef.m_pChannel[0][iPos];
			mean_ref_int += value_ref;
			mean_curr += value_curr;

			values_ref[i] = value_ref;
			values_curr[i] = value_curr;
		}
	}

	mean_ref = (_T)mean_ref_int / (ncc_area * 255.f);
	mean_curr /= ncc_area;

	_T numerator = 0, demoniator1 = 0, demoniator2 = 0;
	for (i = 0; i < ncc_area; i++)
	{
		_T diff_1 = values_ref[i] * (1.f / 255.f) - mean_ref, diff_2 = values_curr[i] - mean_curr;
		_T n = diff_1 * diff_2;
		numerator += n;
		demoniator1 += diff_1 * diff_1;
		demoniator2 += diff_2 * diff_2;
	}
	return _T(numerator / sqrt(demoniator1 * demoniator2 + 1e-10));   // 防止分母出现零
}

template<typename _T>int epipolarSearch(Image oRef, Image oCur,
	int x, int y, _T fDepth, _T fDepth_Cov, _T Delta_T[4 * 4], _T pt_curr[2], _T epipolar_direction[2])
{
	_T f_ref[3], P_ref[4], px_mean_curr[4];
	Project(x, y, f_ref);
	Normalize(f_ref, 3, f_ref); //不知什么意思
	Matrix_Multiply(f_ref, 3, 1, fDepth, P_ref);
	//Disp(P_ref, 4, 1, "P_ref");
	P_ref[3] = 1;

	Matrix_Multiply(Delta_T, 4, 4, P_ref, 1, px_mean_curr);
	Project_Inv(px_mean_curr, px_mean_curr[0], px_mean_curr[1]);
	//Disp(px_mean_curr, 2, 1, "px_mean_curr");
	_T d_min = fDepth - 3 * fDepth_Cov, d_max = fDepth + 3 * fDepth_Cov;
	if (d_min < 0.1f)
		d_min = 0.1f;
	_T px_min_curr[4], px_max_curr[4];
	Matrix_Multiply(f_ref, 3, 1, d_min, px_min_curr);
	px_min_curr[3] = 1;
	Matrix_Multiply(Delta_T, 4, 4, px_min_curr, 1, px_min_curr);
	Project_Inv(px_min_curr, px_min_curr[0], px_min_curr[1]);

	Matrix_Multiply(f_ref, 3, 1, d_max, px_max_curr);
	px_max_curr[3] = 1;
	Matrix_Multiply(Delta_T, 4, 4, px_max_curr, 1, px_max_curr);
	Project_Inv(px_max_curr, px_max_curr[0], px_max_curr[1]);

	_T epipolar_line[2], half_length;
	Vector_Minus(px_max_curr, px_min_curr, 2, epipolar_line);
	Normalize(epipolar_line, 2, epipolar_direction);
	half_length = 0.5f * fGet_Mod(epipolar_line, 2);
	if (half_length > 100)
		half_length = 100;
	//Disp(epipolar_line, 2, 1, "epipolar_line");

	_T best_ncc = -1.f, best_px_curr[2], px_curr[2], Temp[2], l;
	int i;
	for (i = 0, l = -half_length; l <= half_length; l += 0.7f, i++)
	{
		Matrix_Multiply(epipolar_direction, 2, 1, l, Temp);
		Vector_Add(px_mean_curr, Temp, 2, px_curr);
		//判断Temp点是否还在图的内框内
		if (!(px_curr[0] >= 20 && px_curr[0] < oRef.m_iWidth - 20 &&
			px_curr[1] >= 20 && px_curr[1] < oRef.m_iHeight - 20))
			continue;
		_T ncc = 0;
		ncc = NCC(oRef, oCur, x, y, px_curr[0], px_curr[1]);

		if (ncc > best_ncc)
		{
			best_ncc = ncc;
			best_px_curr[0] = px_curr[0];
			best_px_curr[1] = px_curr[1];
		}
	}
	if (best_ncc < 0.85f)      // 只相信 NCC 很高的匹配
		return 0;
	pt_curr[0] = best_px_curr[0];
	pt_curr[1] = best_px_curr[1];
	return 1;
}
template<typename _T>int updateDepthFilter(_T x1, _T y1, _T x2, _T y2, _T Delta_T[4 * 4], _T epipolar_direction[2], _T Depth[], _T Depth_Cov[], int iWidth, int iHeight)
{
	_T T_Inv[4 * 4], f_ref[4], f_curr[4];
	int iResult;
	Get_Inv_Matrix_Row_Op_2(Delta_T, T_Inv, 4, &iResult);
	Project(x1, y1, f_ref);
	Normalize(f_ref, 3, f_ref);

	Project(x2, y2, f_curr);
	Normalize(f_curr, 3, f_curr);

	_T R[3 * 3], t[3], f2[3], b[2];
	union {
		_T A[2 * 2];
		_T A_Inv[2 * 2];
	};

	Get_R_t(T_Inv, R, t);
	Matrix_Multiply(R, 3, 3, f_curr, 1, f2);
	b[0] = fDot(t, f_ref, 3), b[1] = fDot(t, f2, 3);
	A[0 * 2 + 0] = fDot(f_ref, f_ref, 3);
	A[0 * 2 + 1] = -fDot(f_ref, f2, 3);
	A[1 * 2 + 0] = -A[0 * 2 + 1];
	A[1 * 2 + 1] = -fDot(f2, f2, 3);

	_T ans[2], xm[3], xn[3], p_esti[3], depth_estimation, p[3], a[3];
	Get_Inv_Matrix_Row_Op_2(A, A_Inv, 2, &iResult);
	Matrix_Multiply(A_Inv, 2, 2, b, 1, ans);
	Matrix_Multiply(f_ref, 3, 1, ans[0], xm);

	Matrix_Multiply(f2, 3, 1, ans[1], xn);
	Vector_Add(xn, t, 3, xn);

	Vector_Add(xm, xn, 3, p_esti);
	Matrix_Multiply(p_esti, 3, 1, (_T)0.5f, p_esti);

	depth_estimation = fGet_Mod(p_esti, 3);
	Matrix_Multiply(f_ref, 3, 1, depth_estimation, p);
	Vector_Minus(p, t, 3, a);

	_T t_norm, a_norm, alpha, beta;
	t_norm = fGet_Mod(t, 3);
	a_norm = fGet_Mod(a, 3);
	alpha = acos(fDot(f_ref, t, 3) / t_norm);
	beta = acos(-fDot(a, t, 3) / (a_norm * t_norm));

	_T f_curr_prime[3];
	Project(x2 + epipolar_direction[0], y2 + epipolar_direction[1], f_curr_prime);
	Normalize(f_curr_prime, 3, f_curr_prime);

	_T beta_prime, gamma, p_prime, d_cov, d_cov2;
	beta_prime = acos(-fDot(f_curr_prime, t, 3) / t_norm);
	gamma = PI - alpha - beta_prime;
	p_prime = t_norm * sin(beta_prime) / sin(gamma);
	d_cov = p_prime - depth_estimation;
	d_cov2 = d_cov * d_cov;

	_T mu, sigma2, mu_fuse, sigma_fuse2;
	int iPos = ((int)y1) * iWidth + (int)x1;
	mu = Depth[iPos];
	sigma2 = Depth_Cov[iPos];
	mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
	sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);
	Depth[iPos] = mu_fuse;
	Depth_Cov[iPos] = sigma_fuse2;

	return 0;
}
template<typename _T>void Update(Image oRef, Image oCur, _T Depth[], _T Depth_Cov[], _T Delta_T[4 * 4])
{
	int x, y;
	_T pt_cur[2], epipolar_direction[2];
	for (x = 20; x < oRef.m_iWidth - 20; x++)
	{
		for (y = 20; y < oRef.m_iHeight - 20; y++)
		{
			if (!epipolarSearch(oRef, oCur, x, y, Depth[y * oRef.m_iWidth + x], sqrt(Depth_Cov[y * oRef.m_iWidth + x]), Delta_T, pt_cur, epipolar_direction))
				continue;

			updateDepthFilter((_T)x, (_T)y, pt_cur[0], pt_cur[1], Delta_T, epipolar_direction, Depth, Depth_Cov, oRef.m_iWidth, oRef.m_iHeight);
			//printf("here");
		}
	}
}

template<typename _T>void evaludateDepth(_T depth_truth[], _T depth_estimate[], int iWidth, int iHeight)
{
	int y, x, iPos, cnt_depth_data = 0;
	_T ave_depth_error = 0, ave_depth_error_sq = 0, error;
	for (y = 20; y <= iHeight - 20; y++)
	{
		for (x = 20; x <= iWidth - 20; x++)
		{
			iPos = y * iWidth + x;
			error = depth_truth[iPos] - depth_estimate[iPos];
			ave_depth_error += error;
			ave_depth_error_sq += error * error;
			cnt_depth_data++;
		}
	}
	ave_depth_error /= cnt_depth_data;
	ave_depth_error_sq /= cnt_depth_data;

	cout << "Average squared error = " << ave_depth_error_sq << ", average error: " << ave_depth_error << endl;
}
template<typename _T>int bTemp_Load_Data(char* pcFile, _T(**ppCamera)[4 * 4], int* piCamera_Count, _T** ppDepth)
{
	_T(*pCamera)[4 * 4] = NULL, * pDepth;
	int bRet = 1;
	char str[16];

	//char Value[256];
	FILE* pFile = fopen(pcFile, "rb");
	int iResult, iTemp, iCamera_Count = 0;
	int i;
	if (!pFile)
	{
		bRet = 0;
		goto END;
	}
	pCamera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, 1000 * 16 * sizeof(_T));
	if (!pCamera)
	{
		bRet = 0;
		goto END;
	}
	union {
		_T Temp[7];
		double Temp_1[7];
		float Temp_2[7];
	};

	while (1)
	{
		iResult = fscanf(pFile, "scene_%03d.png ", &iTemp);
		if (iResult <= 0)
			break;

		char str[16];
		if (typeid(_T) == typeid(double))
			strcpy(str, "%lf ");
		else
			strcpy(str, "%f ");
		iResult = fscanf(pFile, str, &Temp[0]);
		iResult = fscanf(pFile, str, &Temp[1]);
		iResult = fscanf(pFile, str, &Temp[2]);
		iResult = fscanf(pFile, str, &Temp[4]);
		iResult = fscanf(pFile, str, &Temp[5]);
		iResult = fscanf(pFile, str, &Temp[6]);
		iResult = fscanf(pFile, str, &Temp[3]);

		TQ_2_Rt(Temp, pCamera[iCamera_Count++]);
		//Disp(pCamera[0], 4, 4);
	}
	fclose(pFile);

	Shrink(&oMatrix_Mem, pCamera, iCamera_Count * 16 * sizeof(_T));
	pDepth = (_T*)pMalloc(&oMatrix_Mem, 640 * 480 * sizeof(_T));
	if (typeid(_T) == typeid(double))
		strcpy(str, "%lf ");
	else
		strcpy(str, "%f ");

	pFile = fopen("D:\\Software\\3rdparty\\slambook2\\ch12\\test_data\\depthmaps\\scene_000.depth", "rb");
	i = 0;
	for (int y = 0; y < 480; y++)
		for (int x = 0; x < 640; x++, i++)
		{
			_T fValue;
			fscanf(pFile, str, &fValue);
			pDepth[i] = fValue / 100.f;
		}


	*ppCamera = pCamera;
	*piCamera_Count = iCamera_Count;
	*ppDepth = pDepth;
END:

	return bRet;
}


static void Epipolar_Search_Test()
{//尝试搞单目稠密重建，感觉不靠谱，速度很慢
	typedef double _T;

	int i, iResult, iCamera_Count;
	Image oImage_Ref, oImage_Cur;
	char File[256];
	_T(*pCamera)[4 * 4], * pRef_Depth;
	if (!bTemp_Load_Data((char*)"D:\\Software\\3rdparty\\slambook2\\ch12\\test_data\\first_200_frames_traj_over_table_input_sequence.txt", &pCamera, &iCamera_Count, &pRef_Depth))
		return;
	_T* pT_Ref, * pTi, Delta_T[4 * 4], Temp[4 * 4];
	_T* pDepth, * pDepth_Cov;
	pT_Ref = pCamera[0];
	bLoad_Image("c:\\tmp\\temp\\000.bmp", &oImage_Ref);
	pDepth = (_T*)pMalloc(&oMatrix_Mem, oImage_Ref.m_iWidth * oImage_Ref.m_iHeight * sizeof(_T));
	pDepth_Cov = (_T*)pMalloc(&oMatrix_Mem, oImage_Ref.m_iWidth * oImage_Ref.m_iHeight * sizeof(_T));
	for (i = 0; i < oImage_Ref.m_iWidth * oImage_Ref.m_iHeight; i++)
		pDepth[i] = pDepth_Cov[i] = 3.f;
	//Disp(pCamera[0], 4, 4, "T");
	Disp_T(pCamera[0]);
	Disp_T(pCamera[1]);

	for (i = 1; i < 100; i++)
	{
		pTi = pCamera[i];
		//Disp(pTi, 4, 4, "T");
		//ΔT = Ti(-1) * Tref
		Get_Inv_Matrix_Row_Op_2(pTi, Temp, 4, &iResult);
		Matrix_Multiply(Temp, 4, 4, pT_Ref, 4, Delta_T);

		//Disp(pT_Ref, 4, 4, "Ref");
		sprintf(File, "c:\\tmp\\temp\\%03d.bmp", 0);
		if (!bLoad_Image(File, &oImage_Cur))
			continue;
		//Disp(Delta_T, 4, 4);
		Update<_T>(oImage_Ref, oImage_Cur, pDepth, pDepth_Cov, Delta_T);
		evaludateDepth(pRef_Depth, pDepth, oImage_Ref.m_iWidth, oImage_Ref.m_iHeight);

		Free_Image(&oImage_Cur);
		//printf("%d\n", i);
	}
	return;
}

void Sift_And_Estimate_E_Test_1()
{//本例子为最简例子，通过两图匹配估计出一个E矩阵
	typedef float _T;
	_T(*pPoint_1)[2], (*pPoint_2)[2], R[9], t[3];
	int iCount;
	Ransac_Report oReport;
	char File_1[] = "C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Ajay\\A01.bmp",
		File_2[] = "C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Ajay\\A02.bmp";
	Image oImage_1, oImage_2, oImage = {};
	bLoad_Image(File_1, &oImage_1), bLoad_Image(File_2, &oImage_2);
	Concat_Image(oImage_1, oImage_2, &oImage);
	Free_Image(&oImage_1), Free_Image(&oImage_2);

	Sift_Match_2_Image(File_1, File_2, &pPoint_1, &pPoint_2, &iCount);
	//此处把查出来的点对存起来，下次避免再次匹配
	//bSave_Raw_Data("c:\\tmp\\Point.bin", (unsigned char*)pPoint_1, iCount * 2 *2* sizeof(_T));

	//此处可以替代上面的匹配，直接从文件装入
	//bLoad_Raw_Data("c:\\tmp\\Point.bin", (unsigned char**)&pPoint_1, &iCount);
	//iCount /= 2 * 2 * sizeof(float);
	//pPoint_2 = pPoint_1 + iCount;

	//E矩阵估计
	Ransac_Estimate_E(pPoint_1, pPoint_2, iCount, 640, 240, 320, &oReport);
	//Disp(oReport.m_Modal_f, 3, 3);
	//printf("Residual Sum: %f\n", oReport.m_oSupport.m_fResidual_Sum);

	//估计完了释放多余内存
	Shrink_Match_Point(&pPoint_1, &pPoint_2, oReport.m_pInlier_Mask, iCount);
	iCount = oReport.m_oSupport.m_iInlier_Count;
	//画出点对
	Draw_Match_Point(pPoint_1, pPoint_2, iCount, oImage);

	//可以看一下匹配结果
	//Draw_Match_Point(pPoint_1, pPoint_2,oReport.m_pInlier_Mask, iCount, oImage);
	//bSave_Image("c:\\tmp\\1.bmp", oImage);

	//归一化
	_T(*pNorm_Point_1)[2] = (_T(*)[2])pMalloc(oReport.m_oSupport.m_iInlier_Count * 2 * sizeof(_T) * 2),
		(*pNorm_Point_2)[2] = pNorm_Point_1 + oReport.m_oSupport.m_iInlier_Count;
	Normalize_Point(pPoint_1, pPoint_2, iCount, pNorm_Point_1, pNorm_Point_2, 320, 240, 320);

	//unsigned long long tStart = iGet_Tick_Count();
	//for(int i=0;i<1000;i++)
	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
	E_2_R_t((_T*)oReport.m_Modal, pNorm_Point_1, pNorm_Point_2, iCount, R, t, pPoint_3D, &iCount);
	Disp(R, 3, 3, "R_E");
	Disp(t, 1, 3, "t_E");
	//Disp((_T*)pPoint_3D, iCount, 3, "Point_3D");
	//printf("%lld\n", iGet_Tick_Count() - tStart);
	//Test_E((_T*)oReport.m_Modal, pNorm_Point_1, pNorm_Point_2, iCount);

	//释放内存
	if (pPoint_1)free(pPoint_1);
	if (pNorm_Point_1)Free(pNorm_Point_1);
	Free_Report(oReport);
	Free_Image(&oImage);
	Free(pPoint_3D);
	return;
}
void Sift_And_Estimate_H_Test_1()
{//本例子为最简例子，通过两图匹配估计出一个H矩阵
	typedef float _T;
	_T(*pPoint_1)[2], (*pPoint_2)[2];

	int iCount;
	Ransac_Report oReport;
	char File_1[] = "C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Ajay\\A01.bmp",
		File_2[] = "C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Ajay\\A02.bmp";
	Image oImage_1, oImage_2, oImage = {};
	bLoad_Image(File_1, &oImage_1), bLoad_Image(File_2, &oImage_2);
	Concat_Image(oImage_1, oImage_2, &oImage);
	Free_Image(&oImage_1), Free_Image(&oImage_2);

	//Sift_Match_2_Image(File_1, File_2, &pPoint_1, &pPoint_2, &iCount);
	//此处把查出来的点对存起来，下次避免再次匹配
	//bSave_Raw_Data("c:\\tmp\\Point.bin", (unsigned char*)pPoint_1, iCount * 2 *2* sizeof(_T));

	//此处可以替代上面的匹配，直接从文件装入
	bLoad_Raw_Data("c:\\tmp\\Point.bin", (unsigned char**)&pPoint_1, &iCount);
	iCount /= 2 * 2 * sizeof(float);
	pPoint_2 = pPoint_1 + iCount;

	//E矩阵估计
	Ransac_Estimate_H(pPoint_1, pPoint_2, iCount, &oReport);
	//估计完了释放多余内存
	Shrink_Match_Point(&pPoint_1, &pPoint_2, oReport.m_pInlier_Mask, iCount);
	iCount = oReport.m_oSupport.m_iInlier_Count;

	_T K[] = { 640,   0, 240,  0, 640, 320 ,  0 ,  0,   1 };
	_T R1[9], R2[9], t1[3], t2[3];

	Decompose_H((_T*)oReport.m_Modal, R1, R2, t1, t2, K, K);
	Disp(R1, 3, 3, "R1_H");
	Disp(R2, 3, 3, "R2_H");
	Disp(t1, 1, 3, "t1_H");
	Disp(t2, 1, 3, "t2_H");

	/*Disp(R1, 3, 3, "R1");
	Disp(t1, 1, 3, "t1");
	Disp(R2, 3, 3, "R2");
	Disp(t2, 1, 3, "t2");*/

	//画出点对
	//Draw_Match_Point(pPoint_1, pPoint_2, iCount, oImage);
	if (pPoint_1)free(pPoint_1);
	Free_Report(oReport);
	Free_Image(&oImage);
	return;
}

void DLT_Test_1()
{//此处用RGBD数据搞一个位位姿估计
	typedef float _T;
	_T K[] = { 520.9f, 0, 325.1f, 0, 521.0f, 249.7f, 0, 0, 1 };
	int iCount,iResult;
	Image_Match_Param<_T> oParam = {};

	Match_2_Image("D:\\Software\\3rdparty\\slambook2\\ch7\\1.bmp", "D:\\Software\\3rdparty\\slambook2\\ch7\\1.Depth",
		"D:\\Software\\3rdparty\\slambook2\\ch7\\2.bmp", "D:\\Software\\3rdparty\\slambook2\\ch7\\2.Depth",
		5000.f, K, &iCount,
		&oParam.m_pImage_Point_0, &oParam.m_pImage_Point_1,
		&oParam.m_pNorm_Point_0, &oParam.m_pNorm_Point_1,
		&oParam.m_pPoint_3D_0, &oParam.m_pPoint_3D_1, &oParam.m_oImage_0, &oParam.m_oImage_1, &oParam.m_oImage_2);

	//用图像A的归一化平面坐标与图像B的空间坐标来推导出一个位姿
	_T T[16];

	//bSave_Raw_Data("c:\\tmp\\temp\\Point_3D.bin", (unsigned char*)oParam.m_pPoint_3D_0, iCount * 3 * sizeof(_T));
	//bSave_Raw_Data("c:\\tmp\\temp\\Point_2D.bin", (unsigned char*)oParam.m_pImage_Point_1, iCount * 2 * sizeof(_T));
	//bSave_Raw_Data("c:\\tmp\\temp\\K.bin", (unsigned char*)K, 3 * 3 * sizeof(_T));

	//Disp((_T*)oParam.m_pNorm_Point_1, 6, 2);
	//iCount = 100;
	unsigned long long tStart;

	tStart = iGet_Tick_Count();
	for (int i = 0; i < 1000; i++)
	DLT_svd(oParam.m_pPoint_3D_0, (_T(*)[3])NULL, oParam.m_pNorm_Point_1, iCount, T);
	printf("svd DLT Loss:%f Time Span:%lldms\n", Test_T(oParam.m_pPoint_3D_0, oParam.m_pPoint_3D_1, T, iCount), iGet_Tick_Count() - tStart);

	tStart = iGet_Tick_Count();
	for (int i = 0; i < 1000; i++)
	DLT(oParam.m_pPoint_3D_0, oParam.m_pPoint_3D_1, iCount, T);
	printf("Ours Loss:%f Time Span:%lldms\n", Test_T(oParam.m_pPoint_3D_0, oParam.m_pPoint_3D_1, T, iCount), iGet_Tick_Count() - tStart);
	
	tStart = iGet_Tick_Count();
	for (int i = 0; i < 1000; i++)
	ICP_SVD(oParam.m_pPoint_3D_0, oParam.m_pPoint_3D_1, iCount, T, &iResult);
	printf("ICP Loss:%f Time Span:%lldms\n", Test_T(oParam.m_pPoint_3D_0, oParam.m_pPoint_3D_1, T, iCount), iGet_Tick_Count() - tStart);
	
	ICP_BA_2_Image_1(oParam.m_pPoint_3D_0, oParam.m_pPoint_3D_1, iCount, T, &iResult);
	printf("ICP BA Loss:%f\n", Test_T(oParam.m_pPoint_3D_0, oParam.m_pPoint_3D_1, T, iCount));

	//printf("%lld\n", iGet_Tick_Count() - tStart);

	//至此，三种位置全在
	Free_Image(&oParam.m_oImage_2);
	free(oParam.m_pImage_Point_0);
	Free(oParam.m_pNorm_Point_0);
	Free(oParam.m_pNorm_Point_1);
	Free(oParam.m_pPoint_3D_0);
	Free(oParam.m_pPoint_3D_1);
	return;
}

template<typename _T>static void Get_J_1(_T Sample_In[], _T Y[], _T J[], _T X[], _T* g)
{//获得一个雅可比矩阵,每一个Get_J一个问题
	_T x = Sample_In[0], y = Y[0],
		a = X[0], b = X[1], c = X[2];
	//本问题是 f(x) = g(x)- yi = exp(a*x*x + b*x + c) - yi
	_T fTemp = exp(a * x * x + b * x + c);
	*g = fTemp;
	//df/da =exp(a*x*x + b*x + c) * x*x;
	J[0] = fTemp * x * x;
	//df/db =exp(a*x*x + b*x + c) * x
	J[1] = fTemp * x;
	//df/dc =exp(a*x*x + b*x + c) * 1
	J[2] = fTemp;
}

static void General_BA_Test_1()
{//试一下更统一的最小二乘法解法，一维数据
	typedef float _T;
	const int N = 100, iIn_Dim = 1, iOut_Dim = 1, iParam_Count = 3;
	int i;
	_T Sample[100][iIn_Dim],    //输入变量，相当于x
		Y[N][iOut_Dim],         //输出期望值，相当于yi
		X[iParam_Count] = { 2,-1,5 };        //待估计参数

	//造数据，这部分应该是采集而来
	for (i = 0; i < N; i++)
	{
		const _T ar = 1, br = 2, cr = 1;
		Sample[i][0] = (_T)i / N;
		Y[i][0] = exp(ar * Sample[i][0] * Sample[i][0] + br * Sample[i][0] + cr);
	}

	_T eps = (_T)1e-2;

	////造数据结束
	//Bundle_Adjust((_T*)Sample, iIn_Dim, (_T*)Y, iOut_Dim, N, X, iParam_Count, eps,100,
		//Get_J_1);
	return;
}

template<typename _T>static void Get_J_2(_T Sample_In[], _T Y[], _T J[], _T X[], _T* g)
{//获得一个雅可比矩阵,每一个Get_J一个问题
	//_T Y1[2];
	Matrix_Multiply(X, 2, 2, Sample_In, 1, g);
	memset(J, 0, 2 * 4 * sizeof(_T));
	J[0] = Sample_In[0];
	J[1] = Sample_In[1];
	J[6] = Sample_In[0];
	J[7] = Sample_In[1];
}

static void Generic_BA_Test_2()
{//手搓一个P' = T*P,二维数据
	//求解 min sigma||f(x)||^2 = min sigma||g(x) =yi||^2
	typedef float _T;
	const int N = 100, iIn_Dim = 2, iOut_Dim = 2, iParam_Count = 4;
	int i;
	_T Sample[N][iIn_Dim],    //输入变量，相当于x
		Y[N][iOut_Dim];         //输出期望值，相当于yi
	_T T[4] = { 1,2,3,4 },      //期望解
		T1[2 * 2];             //优化解

	Gen_I_Matrix(T1, 2, 2);
	for (i = 0; i < N; i++)
	{
		Sample[i][0] = (_T)(iGet_Random_No() % 100);
		Sample[i][1] = (_T)(iGet_Random_No() % 100);
		Matrix_Multiply(T, 2, 2, Sample[i], 1, Y[i]);
		Y[i][0] += (iGet_Random_No() % 100) / 100.f;
		Y[i][1] += (iGet_Random_No() % 100) / 100.f;
	}

	_T eps = (_T)1e-20;
	//Bundle_Adjust((_T*)Sample, iIn_Dim, (_T*)Y, iOut_Dim, N, T1, iParam_Count, eps, 100,Get_J_2);
	//Disp(T1, 2, 2, "T1");
	return;
}

template<typename _T>static void Get_J_3(_T Sample_In[], _T Y[], _T J[], _T X[], _T* g)
{//J =	x0	x1	x2	0...			0..
//      0...        x0	x1	x2	0...
//      0...		0..             x0	x1	x2
	Matrix_Multiply(X, 3, 3, Sample_In, 1, g);
	memset(J, 0, 3 * 9 * sizeof(_T));
	J[0] = J[1 * 9 + 3] = J[2 * 9 + 6] = Sample_In[0];
	J[1] = J[1 * 9 + 4] = J[2 * 9 + 7] = Sample_In[1];
	J[2] = J[1 * 9 + 5] = J[2 * 9 + 8] = Sample_In[2];
	return;
}

static void Generic_BA_Test_3()
{//最小二乘法，三维数据，求解 P' = TP 的优化
	//f(x) = g(x) - yi 
	typedef float _T;
	const int N = 100, iIn_Dim = 3, iOut_Dim = 3, iParam_Count = 9;
	int i;
	_T Sample[N][iIn_Dim],    //输入变量，相当于x
		Y[N][iOut_Dim];         //输出期望值，相当于yi
	_T T[iIn_Dim * iIn_Dim] = { 1,2,3,4,5,6,7,8,10 },      //期望解
		T1[iIn_Dim * iIn_Dim];             //优化解
	//printf("%d\n", iGet_Rank(T, 3, 3));
	Gen_I_Matrix(T1, iIn_Dim, iIn_Dim);
	for (i = 0; i < N; i++)
	{
		Sample[i][0] = (_T)(iGet_Random_No() % 100);
		Sample[i][1] = (_T)(iGet_Random_No() % 100);
		Sample[i][2] = (_T)(iGet_Random_No() % 100);
		Matrix_Multiply(T, 3, 3, Sample[i], 1, Y[i]);
	}
	_T eps =(_T)1e-20;
	//Bundle_Adjust((_T*)Sample, iIn_Dim, (_T*)Y, iOut_Dim, N, T1, iParam_Count, eps,100, Get_J_3);
	//Disp(T1, 3, 3, "X");
}

template<typename _T>void BA_PnP_3D_2D_Pose_Get_J(_T Point_3D[3],_T Camera[4*4],_T K[3*3],_T Point_2D[2],_T J[2*6],_T E[2])
{//根据相机位置，观察点，观察数据算个雅可比与误差
	_T Point_4D[4] = {Point_3D[0],Point_3D[1],Point_3D[2],1};
	_T Point_3D_1[4];
	union {
		_T Point_2D_1[3];
		_T J_UV_TP[2 * 3];  //duv/dP'
	};    
	Matrix_Multiply(Camera, 4, 4, Point_4D, 1, Point_3D_1);
	if (E)
	{
		Matrix_Multiply(K, 3, 3, Point_3D_1, 1, Point_2D_1);
		Point_2D_1[0] /= Point_2D_1[2];
		Point_2D_1[1] /= Point_2D_1[2];
		E[0] = Point_2D_1[0] - Point_2D[0];
		E[1] = Point_2D_1[1] - Point_2D[1];
	}

	if (J)
	{//再算位姿的雅可比
	 //上下两种方法皆可
	 // 方法1，按照数学推导
	 //_T J_TP_Ksi[3 * 6]; //dP'/dksi
	 //Get_Drive_UV_P(K[0], K[0], Point_3D_1, J_UV_TP);
	 ////Disp(J_UV_TP, 2, 3, "duv/dP'");
	 //Get_Deriv_TP_Ksi(Camera, Point_4D, J_TP_Ksi);
	 ////Disp(J_TP_Ksi, 3, 6, "dP'/dksi");
	 //Matrix_Multiply(J_UV_TP, 2, 3, J_TP_Ksi, 6, J);
	 ////Disp(J, 2, 6, "J");

	 //方法2，一步到位， = de/dP' * dP'/dksi = duv/dP' * dP'/dksi
		Get_Deriv_E_Ksi(K, Point_3D_1, J);
		//Disp(J, 2, 6, "J");
	}
	return;
}

template<typename _T>void BA_PnP_Zhang_LM_g2o(
	_T A[],int iOrder, _T b[],  _T x[],  LM_Param_g2o<_T> *poParam,int *pbResult,  //第一部分参数，解方程必备
	//第二部分参数，该问题的额外需要数据
	_T Camera_4x4[][16],int iCamera_Count, _T K[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[],int iPoint_2D_Count)
{//为BA_PnP_3D_2D_Pose提供LM解线性方程
 //先用g2o验证一下收敛与精度
	int i, bResult = 1;
	LM_Param_g2o<_T> oParam = *poParam;
	const int iMax_Size = 256;
	_T Buffer[iMax_Size],
		*f = Buffer;

	//Disp(A, iOrder, iOrder, "A");
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
	_T(*pCamera_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));

	do {
		//oParam.Lamda = 1;     //临时设置一下

		//解分方程看看        
		Add_I_Matrix(A, iOrder, oParam.Lamda);
		Solve_Linear_Gause(A, iOrder, b, x, &bResult);
		//Disp(x, 1, iOrder, "Delta_X");
		// 
		//恢复方程
		Add_I_Matrix(A, iOrder, -oParam.Lamda);

		//先更新一下Camera,即把x加上去
		memcpy(pCamera_1, Camera_4x4, iCamera_Count*16 * sizeof(_T));
		for (i = 0; i < iCamera_Count; i++)
		{
			_T* pCamera_Delta_6 = &x[i * 6];
			_T Camera_Delta_4x4[4 * 4]; 
			se3_2_SE3(pCamera_Delta_6, Camera_Delta_4x4);
			Matrix_Multiply(Camera_Delta_4x4, 4, 4, pCamera_1[i], 4, pCamera_1[i]);
		}
		//再将所有的点算一遍，求误差
		_T fSum_e = 0;
		for (i = 0; i < iPoint_2D_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2];
			BA_PnP_3D_2D_Pose_Get_J(Point_3D[oPoint_2D.m_iPoint_Index],
				pCamera_1[oPoint_2D.m_iCamera_Index], K, oPoint_2D.m_Pos, (_T*)NULL, E);
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
		}
		//printf("%f\n",fSum_e);

		if (!bResult)   //没啥营养，就是为了跳出
			fSum_e = std::numeric_limits<_T>::max();

		//rho不过是个表示发散/收敛程度的参数
		rho = (oParam.m_fLoss - fSum_e);

		//搞个scale
		_T fScale;
		if (bResult)
		{
			fScale = (_T)1e-3;
			//从此处可以看出，Scale在解方程成功的时候起作用
			//可以想象，x就是增长步长，不断收敛，故此，fScale也不断减少
			for (int i = 0; i < iOrder; i++)
				fScale += x[i] * (oParam.Lamda * x[i] + b[i]);
		}else
			fScale = 1;

		rho /= fScale;  //显然, rho>0时表示解收敛

		if (rho > 0 && std::_Is_finite(fSum_e) && bResult)
		{//本次迭代的最后一次
		 //fScale递减，rho增大，pow增大，alpha减少，scaleFactor减少
			_T alpha = (_T)(1. - pow((2 * rho - 1), 3));
			alpha = Min(alpha, 2.f/3.f);
			_T scaleFactor = Max(1.f/3.f, alpha);

			//可见，scaleFactor在[1/3,2/3]之间，Lamda必然缩小
			oParam.Lamda *= scaleFactor;
			oParam._ni = 2;
			oParam.m_fLoss = fSum_e;
		}else
		{//加入对角线调整失败，则继续改下去
			oParam.Lamda*=oParam._ni;     //等于将原来的Lamda增大_ni倍，_ni则成倍增大  
			oParam._ni *= 2;            //理论上，对_ni加倍增大
			if (!std::_Is_finite(oParam.Lamda)) 
				break;
		}
		qmax++;
	}while (rho < 0  && qmax<10 && !poParam->m_bStop);	//此处本来还有个外部干涉信号可控制停止
	*pbResult = bResult;

	//若成功，则直接更新Camera了事，省了后续的更新
	memcpy((_T*)Camera_4x4, pCamera_1, iCamera_Count * 16 * sizeof(_T));
	*poParam = oParam;
	Free(pCamera_1);
	return;
}

template<typename _T>void BA_PnP_3D_2D_Pose(_T Camera[][16], int iCamera_Count,_T K[],_T Point_3D[][3],int iPoint_3D_Count, Point_2D<_T> Observation_2D[],int iObservation_Count, _T fLoss_eps = (_T)1e-10)
{//一个BA的标准形式，专门解决位姿估计
 //Camera: 所有的相机位姿，以4x4矩阵表示 
 //Point_3D： 可能有噪声的空间点位置
 //Point_2D, 各个相机对应的2D点
	const int iJ_w = iCamera_Count * 6;
	_T* J = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* Jt = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Delta_X = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_H = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T)),
		* JtJ = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T));

	_T fSum_e = 0, fSum_e_Pre = (_T)1e20;
	_T focal = K[0];
	int i,iIter,iResult;
	LM_Param_g2o<_T> oLM_Param;
	Init_LM_Param(&oLM_Param);

	_T fLoss_Diff_eps = (_T)1e-2;
	fLoss_Diff_eps*= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大

	//注意，以下的整个求误差与求LM配合得并不好，结构不好，是有计算上得冗余
	//这种冗余不仅仅是计算得重复，最怕得是数据不一致
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, iJ_w * iJ_w * sizeof(_T));
		memset(Sigma_JtE, 0, iJ_w * sizeof(_T));
		for (i = 0; i < iObservation_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2],J_E_Ksi[2*6];
			BA_PnP_3D_2D_Pose_Get_J(Point_3D[oPoint_2D.m_iPoint_Index],
				Camera[oPoint_2D.m_iCamera_Index],K, oPoint_2D.m_Pos,J_E_Ksi, E);
			//Disp(J_E_Ksi, 2, 6, "de/dksi");
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差

			memset(J, 0, 2 * iJ_w * sizeof(_T));
			Copy_Matrix_Partial(J_E_Ksi, 2, 6, J, iJ_w, oPoint_2D.m_iCamera_Index * 6, 0);

			//累加Sigma_H，可以证明，步步求和和最后将全部J进行A'A计算相等
			Transpose_Multiply(J, 2, iJ_w, JtJ, 0);
			Matrix_Add(Sigma_H, JtJ, iJ_w, Sigma_H);

			//累加Sigma_JtE;
			Matrix_Transpose(J, 2, iJ_w, Jt);
			Matrix_Multiply(Jt, iJ_w, 2, E, 1, JtE);
			Vector_Add(Sigma_JtE, JtE, iJ_w, Sigma_JtE);
		}
		printf("iIter:%d %e\n", iIter, 0.5f*fSum_e);
		Matrix_Multiply(Sigma_JtE, 1, iJ_w, (_T)-1, Sigma_JtE);

		//Add_I_Matrix(Sigma_H, iJ_w, (_T)1);
		//方法1，解方程方法
		//Solve_Linear_Gause(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &iResult);
		//Disp(Delta_X, 1, iJ_w, "Delta_X");

		//方法2，LM方法
		oLM_Param.m_fLoss = fSum_e, oLM_Param.m_iIter = iIter;
		BA_PnP_Zhang_LM_g2o(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &oLM_Param, &iResult,
			Camera,iCamera_Count, K, Point_3D, iPoint_3D_Count, Observation_2D, iObservation_Count);

		if (!iResult)
		{
			printf("Bundle Adjust LM解失败\n");
			break;
		}
		if (fSum_e<=oLM_Param.m_fLoss  || oLM_Param.m_fLoss < fLoss_eps || Abs(oLM_Param.m_fLoss-fSum_e)<fLoss_Diff_eps)
			break;
		fSum_e_Pre = oLM_Param.m_fLoss;        
	}

	Free(J);
	Free(Jt);
	Free(JtE);
	Free(Delta_X);
	Free(Sigma_JtE);
	Free(Sigma_H);
	Free(JtJ);
	return;
}

void Pose_Estimate_Test_1()
{//只优化位姿，不优化点
	typedef float _T;
	//第一部分，造数据
	const int iCamera_Count = 16,    //iCamera_Count=1, iPoint_Count=100时收敛奇怪，需要重搞LM
		iPoint_Count = 100,
		iObservation_Count = iPoint_Count*iCamera_Count;
	int i;

	_T(*pTrue_Point_3D)[3] = //真实点
		(_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));
	_T(*pNoisy_Point_3D)[3]=//真实点加上噪声
		(_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));
	for (i = 0; i < iPoint_Count; i++)
	{
		const int Recip = 1234567;
		_T Point_3D[] = { ((_T)((int)iGet_Random_No() % Recip) / Recip - 0.5f) * 3,
			(_T)((int)iGet_Random_No() % Recip) / Recip - (_T)0.5,
			(_T)((int)iGet_Random_No() % Recip) / Recip + (_T)3 };
		memcpy(pTrue_Point_3D[i], Point_3D, 3 * sizeof(_T));
	}

	_T(*pCamera_4x4)[4 * 4] =   //相机外参的T矩阵
		(_T(*)[4 * 4])pMalloc(iCamera_Count * 4*4 * sizeof(_T));
	_T K[3 * 3] = { 1000,0,320, 0,1000,240,0,0,1 };  //相机内参
	for (i = 0; i < iCamera_Count; i++)
	{//造位姿
		_T Camera_6[] = { 0,0,0,(_T)(i * 0.04 - 1.f),0,0 };
		_T Rotation_Vector[6], R[3 * 3];
		Rotation_Vector_3_2_4(Camera_6, Rotation_Vector);
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, &Camera_6[3], pCamera_4x4[i]);
		//注意，此处不能用se3_2_SE3，因为此Ksi不是彼ksi
	}

	//_T(*pPoint_2D_Ref)[iCamera_Count][2] = //空间点在每个相机像素平面上的投影
	//    (_T(*)[iCamera_Count][2])pMalloc(iPoint_Count * iCamera_Count * 2 * sizeof(_T));
	Point_2D<_T>* pObservation_2D = (Point_2D<_T>*)pMalloc(iPoint_Count * iCamera_Count * sizeof(Point_2D<_T>));
	for (i = 0; i < iPoint_Count; i++)
	{
		const int Recip = 1234567;
		pNoisy_Point_3D[i][0] = pTrue_Point_3D[i][0]+ (_T)((int)iGet_Random_No() % Recip) / Recip;
		pNoisy_Point_3D[i][1] = pTrue_Point_3D[i][1]+(_T)((int)iGet_Random_No() % Recip) / Recip;
		pNoisy_Point_3D[i][2] = pTrue_Point_3D[i][2]+(_T)((int)iGet_Random_No() % Recip) / Recip;
		//Disp(pNoisy_Point_3D[i], 1, 3, "Point_3D");
		for (int j = 0; j < iCamera_Count; j++)
		{           
			_T Point_4D[] = { pTrue_Point_3D[i][0],pTrue_Point_3D[i][1],pTrue_Point_3D[i][2],1 };  
			_T Point_3D[4], Point_2D[2];

			Matrix_Multiply(pCamera_4x4[j], 4, 4, Point_4D, 1, Point_3D);
			Matrix_Multiply(K, 3, 3, Point_3D, 1, Point_3D);
			Point_2D[0] = Point_3D[0] / Point_3D[2];
			Point_2D[1] = Point_3D[1] / Point_3D[2];

			const int Recip = 1234567;
			Point_2D[0] += (_T)((int)iGet_Random_No() % Recip) / Recip,
				Point_2D[1] += (_T)((int)iGet_Random_No() % Recip) / Recip;
			//Disp(Point_2D, 1, 2, "Point_2D");
			pObservation_2D[i*iCamera_Count+j].m_iCamera_Index = j;
			pObservation_2D[i*iCamera_Count+j].m_iPoint_Index = i;
			memcpy(pObservation_2D[i*iCamera_Count+j].m_Pos, Point_2D, 2 * sizeof(_T));
		}
	}
	//至此，数据已经做完

	//调用BA
	BA_PnP_3D_2D_Pose(pCamera_4x4, iCamera_Count,K, pNoisy_Point_3D, iPoint_Count,
		pObservation_2D, iObservation_Count);


	Free(pCamera_4x4);
	Free(pTrue_Point_3D);
	Free(pNoisy_Point_3D);
	Free(pObservation_2D);
	return;
}

template<typename _T>void BA_PnP_3D_2D_Pose_N_Point_Get_J(_T Point_3D[3],_T Camera[4*4],_T K[3*3],_T Point_2D[2],_T J_E_Ksi[2*6], _T J_E_P[2*3], _T E[2])
{//根据相机位置，观察点，观察数据算个雅可比与误差
	_T Point_4D[4] = {Point_3D[0],Point_3D[1],Point_3D[2],1};
	_T Point_3D_1[4];
	union {
		_T Point_2D_1[3];
		_T J_UV_TP[2 * 3];  //duv/dP'
	};    
	Matrix_Multiply(Camera, 4, 4, Point_4D, 1, Point_3D_1);
	if (E)
	{
		Matrix_Multiply(K, 3, 3, Point_3D_1, 1, Point_2D_1);
		Point_2D_1[0] /= Point_2D_1[2];
		Point_2D_1[1] /= Point_2D_1[2];
		E[0] = Point_2D_1[0] - Point_2D[0];
		E[1] = Point_2D_1[1] - Point_2D[1];
	}

	if (J_E_Ksi && J_E_P)
	{//再算位姿的雅可比
	 //方法2，一步到位， = de/dP' * dP'/dksi = duv/dP' * dP'/dksi
		Get_Deriv_E_Ksi(K, Point_3D_1, J_E_Ksi);
		Get_Deriv_E_P(K, Camera, Point_3D_1, J_E_P);        
		//留心看两个雅可比，可以看出有大量重合数据，还可以对求导进一步优化计算
	}
	return;
}

template<typename _T>void BA_PnP_3D_2D_Pose_N_Point_LM(
	_T A[], int iOrder, _T b[], _T x[], LM_Param_g2o<_T>* poParam, int* pbResult,  //第一部分参数，解方程必备
	//第二部分参数，该问题的额外需要数据
	_T Camera_4x4[][16], int iCamera_Count, _T K[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iPoint_2D_Count)
{//对位姿与点联合优化解LM

	int i, bResult = 1;
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
	_T(*pCamera_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));
	_T(*pPoint_3D_1)[3] = (_T(*)[3])pMalloc(iPoint_3D_Count * 3 * sizeof(_T));

	do {
		//oParam.Lamda = 1;     //临时设置一下

		//解分方程看看        
		Add_I_Matrix(A, iOrder, oParam.Lamda);
		Solve_Linear_Gause(A, iOrder, b, x, &bResult);
		//Disp(A, 1, iOrder, "A");
		//Disp(b, 1, iOrder, "b");
		//Disp(x, 1, iOrder, "x");
		// 
		//恢复方程
		Add_I_Matrix(A, iOrder, -oParam.Lamda);

		memcpy(pCamera_1, Camera_4x4, iCamera_Count*16 * sizeof(_T));
		memcpy(pPoint_3D_1, Point_3D, iPoint_3D_Count * 3 * sizeof(_T));

		//先更新一下Camera,即把x加上去
		for (i = 0; i < iCamera_Count; i++)
		{
			_T* pCamera_Delta_6 = &x[i * 6];
			_T Camera_Delta_4x4[4 * 4]; 
			se3_2_SE3(pCamera_Delta_6, Camera_Delta_4x4);
			Matrix_Multiply(Camera_Delta_4x4, 4, 4, pCamera_1[i], 4, pCamera_1[i]);
		}

		//再更新一下点位置，把x加上去
		_T* x1 = x + iCamera_Count * 6;
		for (i = 0; i < iPoint_3D_Count; i++)
			Vector_Add(pPoint_3D_1[i], &x1[i * 3], 3,pPoint_3D_1[i] );

		//再将所有的点算一遍，求误差
		_T fSum_e = 0;
		for (i = 0; i < iPoint_2D_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2];
			BA_PnP_3D_2D_Pose_N_Point_Get_J(pPoint_3D_1[oPoint_2D.m_iPoint_Index],
				pCamera_1[oPoint_2D.m_iCamera_Index],K, oPoint_2D.m_Pos,(_T*)NULL,(_T*)NULL, E);
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
		}
		//printf("%f\n",fSum_e);

		if (!bResult)   //没啥营养，就是为了跳出
			fSum_e = std::numeric_limits<_T>::max();

		//rho不过是个表示发散/收敛程度的参数
		rho = (oParam.m_fLoss - fSum_e);

		//搞个scale
		_T fScale;
		if (bResult)
		{
			fScale = (_T)1e-3;
			//从此处可以看出，Scale在解方程成功的时候起作用
			//可以想象，x就是增长步长，不断收敛，故此，fScale也不断减少
			for (int i = 0; i < iOrder; i++)
				fScale += x[i] * (oParam.Lamda * x[i] + b[i]);
		}else
			fScale = 1;

		rho /= fScale;  //显然, rho>0时表示解收敛
		if (rho > 0 && std::_Is_finite(fSum_e) && bResult)
		{//本次迭代的最后一次
		 //fScale递减，rho增大，pow增大，alpha减少，scaleFactor减少
			_T alpha = (_T)(1. - pow((2 * rho - 1), 3));
			alpha = Min(alpha, 2.f/3.f);
			_T scaleFactor = Max(1.f/3.f, alpha);

			//可见，scaleFactor在[1/3,2/3]之间，Lamda必然缩小
			oParam.Lamda *= scaleFactor;
			oParam._ni = 2;
			oParam.m_fLoss = fSum_e;
		} else
		{
			//加入对角线调整失败，则继续改下去
			oParam.Lamda*=oParam._ni;     //等于将原来的Lamda增大_ni倍，_ni则成倍增大  
			oParam._ni *= 2;            //理论上，对_ni加倍增大
			if (!std::_Is_finite(oParam.Lamda)) 
				break;
		}
		qmax++;
	}while (rho < 0  && qmax<10 && !poParam->m_bStop);	//此处本来还有个外部干涉信号可控制停止
	*pbResult = bResult;
	//若成功，则直接更新Camera,Point_3D了事，省了后续的更新
	memcpy((_T*)Camera_4x4, pCamera_1, iCamera_Count * 16 * sizeof(_T));
	memcpy((_T*)Point_3D, pPoint_3D_1, iPoint_3D_Count * 3 * sizeof(_T));
	*poParam = oParam;
	Free(pCamera_1);
	Free(pPoint_3D_1);
	return;
}

template<typename _T>void BA_PnP_3D_2D_Pose_N_Point(_T Camera[][16], int iCamera_Count, _T K[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
{//一个BA的标准形式，专门解决位姿估计
 //Camera: 所有的相机位姿，以4x4矩阵表示 
 //Point_3D： 可能有噪声的空间点位置
 //Point_2D, 各个相机对应的2D点
	const int iJ_w = iCamera_Count * 6 + iPoint_3D_Count * 3;
	_T* J = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* Jt = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Delta_X = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_H = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T)),
		* JtJ = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T));


	_T fSum_e = 0, fSum_e_Pre = (_T)1e20;
	_T focal = K[0];
	int i,iIter,iResult;
	LM_Param_g2o<_T> oLM_Param;
	Init_LM_Param(&oLM_Param);

	_T fLoss_Diff_eps = 1e-20;
	fLoss_Diff_eps*= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, iJ_w * iJ_w * sizeof(_T));
		memset(Sigma_JtE, 0, iJ_w * sizeof(_T));
		for (i = 0; i < iObservation_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2],J_E_Ksi[2*6], J_E_P[2*3];
			BA_PnP_3D_2D_Pose_N_Point_Get_J(Point_3D[oPoint_2D.m_iPoint_Index],
				Camera[oPoint_2D.m_iCamera_Index],K, oPoint_2D.m_Pos,J_E_Ksi,J_E_P, E);
			/*Disp(J_E_Ksi, 2, 6, "de/dksi");
			Disp(J_E_P, 2, 3, "de/dP");*/
			//Disp(J, 2, 6, "J");
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差

			memset(J, 0, 2 * iJ_w * sizeof(_T));
			Copy_Matrix_Partial(J_E_Ksi, 2, 6, J, iJ_w, oPoint_2D.m_iCamera_Index * 6, 0);
			Copy_Matrix_Partial(J_E_P, 2, 3, J, iJ_w, iCamera_Count * 6 + oPoint_2D.m_iPoint_Index * 3, 0);
			/*if(i==2)
			Disp(J, 2, iJ_w, "J");*/

			//累加Sigma_H
			Transpose_Multiply(J, 2, iJ_w, JtJ, 0);
			Matrix_Add(Sigma_H, JtJ, iJ_w, Sigma_H);
			//Disp(Sigma_H, iJ_w, iJ_w, "Sigma_H");

			//累加Sigma_JtE;
			Matrix_Transpose(J, 2, iJ_w, Jt);
			Matrix_Multiply(Jt, iJ_w, 2, E, 1, JtE);
			Vector_Add(Sigma_JtE, JtE, iJ_w, Sigma_JtE);
		}
		printf("iIter:%d %e\n", iIter, 0.5f*fSum_e);

		//加上负号，右边才是 -J'E
		Matrix_Multiply(Sigma_JtE, 1, iJ_w, (_T)-1, Sigma_JtE);

		//Add_I_Matrix(Sigma_H, iJ_w, (_T)1);
		////方法1，解方程方法
		//Solve_Linear_Gause(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &iResult);
		//Disp(Delta_X, 1, iJ_w, "Delta_X");
		//Disp(Sigma_H, iJ_w, iJ_w, "Sigma_H");
		//Disp(Sigma_JtE, 1, iJ_w, "Sigma_JtE");

		//方法2，LM方法
		oLM_Param.m_fLoss = fSum_e, oLM_Param.m_iIter = iIter;
		BA_PnP_3D_2D_Pose_N_Point_LM(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &oLM_Param, &iResult,
			Camera,iCamera_Count, K, Point_3D, iPoint_3D_Count, Observation_2D, iObservation_Count);

		if (!iResult)
		{
			printf("Bundle Adjust LM解失败\n");
			break;
		}
		if (fSum_e<=oLM_Param.m_fLoss  || oLM_Param.m_fLoss < fLoss_eps || Abs(oLM_Param.m_fLoss-fSum_e)<fLoss_Diff_eps)
			break;
		fSum_e_Pre = oLM_Param.m_fLoss;
	}

	Free(J);
	Free(Jt);
	Free(JtE);
	Free(Delta_X);
	Free(Sigma_JtE);
	Free(Sigma_H);
	Free(JtJ);
}

void Pose_Estimate_Test_2()
{//连位姿带空间点一起优化
	typedef double _T;
	//第一部分，造数据
	const int iCamera_Count = 16,    //iCamera_Count=1, iPoint_Count=100时收敛奇怪，需要重搞LM
		iPoint_Count = 200,
		iObservation_Count = iPoint_Count*iCamera_Count;
	int i;

	_T(*pTrue_Point_3D)[3] = //真实点
		(_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));
	_T(*pNoisy_Point_3D)[3]=//真实点加上噪声
		(_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));
	for (i = 0; i < iPoint_Count; i++)
	{
		const int Recip = 1234567;
		_T Point_3D[] = { ((_T)((int)iGet_Random_No() % Recip) / Recip - 0.5f) * 3,
			(_T)((int)iGet_Random_No() % Recip) / Recip - 0.5,
			(_T)((int)iGet_Random_No() % Recip) / Recip + 3 };
		memcpy(pTrue_Point_3D[i], Point_3D, 3 * sizeof(_T));
	}

	_T(*pCamera_4x4)[4 * 4] =   //相机外参的T矩阵
		(_T(*)[4 * 4])pMalloc(iCamera_Count * 4*4 * sizeof(_T));
	_T K[3 * 3] = { 1000,0,320, 0,1000,240,0,0,1 };  //相机内参
	for (i = 0; i < iCamera_Count; i++)
	{//造位姿
		_T Camera_6[] = { 0,0,0,i * 0.04 - 1.f,0,0 };
		_T Rotation_Vector[6], R[3 * 3];
		Rotation_Vector_3_2_4(Camera_6, Rotation_Vector);
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, &Camera_6[3], pCamera_4x4[i]);
		//注意，此处不能用se3_2_SE3，因为此Ksi不是彼ksi
	}

	//_T(*pPoint_2D_Ref)[iCamera_Count][2] = //空间点在每个相机像素平面上的投影
	//    (_T(*)[iCamera_Count][2])pMalloc(iPoint_Count * iCamera_Count * 2 * sizeof(_T));
	Point_2D<_T>* pObservation_2D = (Point_2D<_T>*)pMalloc(iPoint_Count * iCamera_Count * sizeof(Point_2D<_T>));
	for (i = 0; i < iPoint_Count; i++)
	{
		const int Recip = 1234567;
		pNoisy_Point_3D[i][0] = pTrue_Point_3D[i][0]+ (double)((int)iGet_Random_No() % Recip) / Recip;
		pNoisy_Point_3D[i][1] = pTrue_Point_3D[i][1]+(double)((int)iGet_Random_No() % Recip) / Recip;
		pNoisy_Point_3D[i][2] = pTrue_Point_3D[i][2]+(double)((int)iGet_Random_No() % Recip) / Recip;
		//Disp(pNoisy_Point_3D[i], 1, 3, "Point_3D");
		for (int j = 0; j < iCamera_Count; j++)
		{           
			_T Point_4D[] = { pTrue_Point_3D[i][0],pTrue_Point_3D[i][1],pTrue_Point_3D[i][2],1 };  
			_T Point_3D[4], Point_2D[2];

			Matrix_Multiply(pCamera_4x4[j], 4, 4, Point_4D, 1, Point_3D);
			Matrix_Multiply(K, 3, 3, Point_3D, 1, Point_3D);
			Point_2D[0] = Point_3D[0] / Point_3D[2];
			Point_2D[1] = Point_3D[1] / Point_3D[2];

			const int Recip = 1234567;
			Point_2D[0] += (double)((int)iGet_Random_No() % Recip) / Recip,
				Point_2D[1] += (double)((int)iGet_Random_No() % Recip) / Recip;
			//Disp(Point_2D, 1, 2, "Point_2D");
			pObservation_2D[i*iCamera_Count+j].m_iCamera_Index = j;
			pObservation_2D[i*iCamera_Count+j].m_iPoint_Index = i;
			memcpy(pObservation_2D[i*iCamera_Count+j].m_Pos, Point_2D, 2 * sizeof(_T));
		}
	}
	//至此，数据已经做完，造数据部分和纯位姿估计完全一致

	//调用BA
	BA_PnP_3D_2D_Pose_N_Point(pCamera_4x4, iCamera_Count,K, pNoisy_Point_3D, iPoint_Count,
		pObservation_2D, iObservation_Count);

	Free(pCamera_4x4);
	Free(pTrue_Point_3D);
	Free(pNoisy_Point_3D);
	Free(pObservation_2D);
}

//template<typename _T>static void BA_PnP_3D_2D_Pose_N_Point_1_Get_J(_T Point_3D[3],_T Camera[4*4],_T K[3*3],_T Point_2D[2],_T J_E_Ksi[2*6], _T J_E_P[2*3], _T E[2])
//{//根据相机位置，观察点，观察数据算个雅可比与误差
//	_T Point_4D[4] = {Point_3D[0],Point_3D[1],Point_3D[2],1};
//	_T Point_3D_1[4];
//	union {
//		_T Point_2D_1[3];
//		_T J_UV_TP[2 * 3];  //duv/dP'
//	};    
//	Matrix_Multiply(Camera, 4, 4, Point_4D, 1, Point_3D_1);
//	if (E)
//	{
//		Matrix_Multiply(K, 3, 3, Point_3D_1, 1, Point_2D_1);
//		Point_2D_1[0] /= Point_2D_1[2];
//		Point_2D_1[1] /= Point_2D_1[2];
//		E[0] = Point_2D_1[0] - Point_2D[0];
//		E[1] = Point_2D_1[1] - Point_2D[1];
//	}
//
//	if (J_E_Ksi && J_E_P)
//	{//再算位姿的雅可比
//	 //方法2，一步到位， = de/dP' * dP'/dksi = duv/dP' * dP'/dksi
//		Get_Deriv_E_Ksi(K, Point_3D_1, J_E_Ksi);
//		Get_Deriv_E_P(K, Camera, Point_3D_1, J_E_P);        
//		//留心看两个雅可比，可以看出有大量重合数据，还可以对求导进一步优化计算
//	}
//	/*Disp(J_E_Ksi, 2, 6, "de/dksi");
//	Disp(J_E_P, 2, 3, "de/dP");*/
//	return;
//}
//
//template<typename _T>static void BA_PnP_3D_2D_Pose_N_Point_1_LM(
//	_T A[], int iOrder, _T b[], _T x[], LM_Param<_T>* poParam, int* pbResult,  //第一部分参数，解方程必备
//	//第二部分参数，该问题的额外需要数据
//	_T Camera_4x4[][16], int iCamera_Count, _T K[][3*3], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iPoint_2D_Count)
//{//对位姿与点联合优化解LM
//
//	int i, bResult = 1;
//	LM_Param<_T> oParam = *poParam;
//	const int iMax_Size = 256;
//	_T Buffer[iMax_Size],
//		*f = Buffer;
//
//	//Disp(b, 1, 3, "b");
//	if (oParam.m_iIter == 0)
//	{//第一次，特殊对待,取对角线最大值		
//		_T tau = (_T)1e-5, fMax = (_T)0.f;
//		for (i = 0; i < iOrder; i++)
//			if (Abs(A[i * iOrder + i]) > fMax)
//				fMax = Abs(A[i * iOrder + i]);
//		//第一次迭代的时候，Lamda定位 0.00001*对角线最大元
//		oParam.Lamda = tau * fMax;
//		oParam._ni = 2;
//	}
//
//	_T rho = 0;
//	int	qmax = 0;
//	_T(*pCamera_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));
//	_T(*pPoint_3D_1)[3] = (_T(*)[3])pMalloc(iPoint_3D_Count * 3 * sizeof(_T));
//
//	do {
//		//oParam.Lamda = 1;     //临时设置一下
//
//		//解分方程看看        
//		Add_I_Matrix(A, iOrder, oParam.Lamda);
//		//printf("Lamda:%f\n", oParam.Lamda);
//		Solve_Linear_Gause(A, iOrder, b, x, &bResult);
//		//Disp(A, iOrder, iOrder, "A");
//		//Disp(b, 1, iOrder, "b");
//		//Disp(x, 1, iOrder, "Delta_X");
//		// 
//		//恢复方程
//		Add_I_Matrix(A, iOrder, -oParam.Lamda);
//
//		memcpy(pCamera_1, Camera_4x4, iCamera_Count*16 * sizeof(_T));
//		memcpy(pPoint_3D_1, Point_3D, iPoint_3D_Count * 3 * sizeof(_T));
//
//		//先更新一下Camera,即把x加上去
//		for (i = 0; i < iCamera_Count; i++)
//		{
//			_T* pCamera_Delta_6 = &x[i * 6];
//			_T Camera_Delta_4x4[4 * 4]; 
//			se3_2_SE3(pCamera_Delta_6, Camera_Delta_4x4);
//			Matrix_Multiply(Camera_Delta_4x4, 4, 4, pCamera_1[i], 4, pCamera_1[i]);
//		}
//
//		//再更新一下点位置，把x加上去
//		_T* x1 = x + iCamera_Count * 6;
//		for (i = 0; i < iPoint_3D_Count; i++)
//			Vector_Add(pPoint_3D_1[i], &x1[i * 3], 3,pPoint_3D_1[i] );
//
//		//再将所有的点算一遍，求误差
//		_T fSum_e = 0;
//		for (i = 0; i < iPoint_2D_Count; i++)
//		{
//			Point_2D<_T> oPoint_2D = Observation_2D[i];
//			_T E[2];
//			BA_PnP_3D_2D_Pose_N_Point_1_Get_J(pPoint_3D_1[oPoint_2D.m_iPoint_Index],
//				pCamera_1[oPoint_2D.m_iCamera_Index],K[oPoint_2D.m_iCamera_Index], oPoint_2D.m_Pos, (_T*)NULL, (_T*)NULL, E);
//			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
//		}
//		//printf("%f\n",fSum_e);
//
//		if (!bResult)   //没啥营养，就是为了跳出
//			fSum_e = std::numeric_limits<_T>::max();
//
//		//rho不过是个表示发散/收敛程度的参数
//		rho = (oParam.m_fLoss - fSum_e);
//
//		//搞个scale
//		_T fScale;
//		if (bResult)
//		{
//			fScale = (_T)1e-3;
//			//从此处可以看出，Scale在解方程成功的时候起作用
//			//可以想象，x就是增长步长，不断收敛，故此，fScale也不断减少
//			for (int i = 0; i < iOrder; i++)
//				fScale += x[i] * (oParam.Lamda * x[i] + b[i]);
//		}else
//			fScale = 1;
//
//		rho /= fScale;  //显然, rho>0时表示解收敛
//		if (rho > 0 && std::_Is_finite(fSum_e) && bResult)
//		{//本次迭代的最后一次
//		 //fScale递减，rho增大，pow增大，alpha减少，scaleFactor减少
//			_T alpha = (_T)(1. - pow((2 * rho - 1), 3));
//			alpha = Min(alpha, 2.f/3.f);
//			_T scaleFactor = Max(1.f/3.f, alpha);
//
//			//可见，scaleFactor在[1/3,2/3]之间，Lamda必然缩小
//			oParam.Lamda *= scaleFactor;
//			oParam._ni = 2;
//			oParam.m_fLoss = fSum_e;
//		} else
//		{
//			//加入对角线调整失败，则继续改下去
//			oParam.Lamda*=oParam._ni;     //等于将原来的Lamda增大_ni倍，_ni则成倍增大  
//			oParam._ni *= 2;            //理论上，对_ni加倍增大
//			if (!std::_Is_finite(oParam.Lamda)) 
//				break;
//		}
//		qmax++;
//	}while (rho < 0  && qmax<10 && !poParam->m_bStop);	//此处本来还有个外部干涉信号可控制停止
//	*pbResult = bResult;
//	//若成功，则直接更新Camera,Point_3D了事，省了后续的更新
//	memcpy((_T*)Camera_4x4, pCamera_1, iCamera_Count * 16 * sizeof(_T));
//	memcpy((_T*)Point_3D, pPoint_3D_1, iPoint_3D_Count * 3 * sizeof(_T));
//	*poParam = oParam;
//	Free(pCamera_1);
//	Free(pPoint_3D_1);
//	return;
//}
//
//template<typename _T>static void BA_PnP_3D_2D_Pose_N_Point_1(_T Camera[][16], int iCamera_Count, _T K[][9], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-20)
//{//一个BA的标准形式，专门解决位姿估计
// //Camera: 所有的相机位姿，以4x4矩阵表示 
// //Point_3D： 可能有噪声的空间点位置
// //Point_2D, 各个相机对应的2D点
//	int i,iIter,iResult;
//	//for (i = 0; i < iObservation_Count; i++)
//	//    printf("Camera:%d Point_3D:%d\n", Observation_2D[i].m_iCamera_Index, Observation_2D[i].m_iPoint_Index);
//
//	const int iJ_w = iCamera_Count * 6 + iPoint_3D_Count * 3;
//	_T* J = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
//		* Jt = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
//		* JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
//		* Delta_X = (_T*)pMalloc(iJ_w * sizeof(_T)),
//		* Sigma_JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
//		* Sigma_H = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T)),
//		* JtJ = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T));
//
//	_T fSum_e = 0, fSum_e_Pre = (_T)1e20;
//	LM_Param<_T> oLM_Param;
//	Init_LM_Param(&oLM_Param);
//
//	_T fLoss_Diff_eps = 1e-2;
//	fLoss_Diff_eps*= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大
//
//	for (iIter = 0;; iIter++)
//	{
//		fSum_e = 0;
//		memset(Sigma_H, 0, iJ_w * iJ_w * sizeof(_T));
//		memset(Sigma_JtE, 0, iJ_w * sizeof(_T));
//		for (i = 0; i < iObservation_Count; i++)
//		{
//			Point_2D<_T> oPoint_2D = Observation_2D[i];
//			_T E[2],J_E_Ksi[2*6], J_E_P[2*3];
//			/*if (oPoint_2D.m_iPoint_Index > iPoint_3D_Count)
//			printf("error");*/
//			BA_PnP_3D_2D_Pose_N_Point_1_Get_J(Point_3D[oPoint_2D.m_iPoint_Index],
//				Camera[oPoint_2D.m_iCamera_Index],K[oPoint_2D.m_iCamera_Index], oPoint_2D.m_Pos, J_E_Ksi, J_E_P, E);
//			/*Disp(J_E_Ksi, 2, 6, "de/dksi");
//			Disp(J_E_P, 2, 3, "de/dP");*/
//			//Disp(J, 2, 6, "J");
//			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
//
//			memset(J, 0, 2 * iJ_w * sizeof(_T));
//			Copy_Matrix_Partial(J_E_Ksi, 2, 6, J, iJ_w, oPoint_2D.m_iCamera_Index * 6, 0);
//			Copy_Matrix_Partial(J_E_P, 2, 3, J, iJ_w, iCamera_Count * 6 + oPoint_2D.m_iPoint_Index * 3, 0);
//			/*if(i==2)
//			Disp(J, 2, iJ_w, "J");*/
//
//			//累加Sigma_H
//			Transpose_Multiply(J, 2, iJ_w, JtJ, 0);
//			Matrix_Add(Sigma_H, JtJ, iJ_w, Sigma_H);
//			//Disp(Sigma_H, iJ_w, iJ_w, "Sigma_H");
//
//			//累加Sigma_JtE;
//			Matrix_Transpose(J, 2, iJ_w, Jt);
//			Matrix_Multiply(Jt, iJ_w, 2, E, 1, JtE);
//			Vector_Add(Sigma_JtE, JtE, iJ_w, Sigma_JtE);
//		}
//		printf("iIter:%d %e\n", iIter, 0.5f*fSum_e);
//
//		//加上负号，右边才是 -J'E
//		Matrix_Multiply(Sigma_JtE, 1, iJ_w, (_T)-1, Sigma_JtE);
//
//		//Add_I_Matrix(Sigma_H, iJ_w, (_T)1);
//		////方法1，解方程方法
//		//Solve_Linear_Gause(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &iResult);
//		//Disp(Delta_X, 1, iJ_w, "Delta_X");
//		//Disp(Sigma_H, iJ_w, iJ_w, "Sigma_H");
//		//Disp(Sigma_JtE, 1, iJ_w, "Sigma_JtE");
//
//		//方法2，LM方法
//		oLM_Param.m_fLoss = fSum_e, oLM_Param.m_iIter = iIter;
//		BA_PnP_3D_2D_Pose_N_Point_1_LM(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &oLM_Param, &iResult,
//			Camera,iCamera_Count, K, Point_3D, iPoint_3D_Count, Observation_2D, iObservation_Count);
//
//		if (!iResult)
//		{
//			printf("Bundle Adjust LM解失败\n");
//			break;
//		}
//		if (fSum_e<=oLM_Param.m_fLoss  || oLM_Param.m_fLoss < fLoss_eps || Abs(oLM_Param.m_fLoss-fSum_e)<fLoss_Diff_eps)
//			break;
//		fSum_e_Pre = oLM_Param.m_fLoss;
//	}
//
//	Free(J);
//	Free(Jt);
//	Free(JtE);
//	Free(Delta_X);
//	Free(Sigma_JtE);
//	Free(Sigma_H);
//	Free(JtJ);
//}

template<typename _T>static void BA_PnP_3D_2D_Pose_N_Point_1_Get_J(_T Point_3D[3], _T Camera[4 * 4], _T Intrinsic[3], _T Point_2D[2], _T J_E_Ksi[2 * 6], _T J_E_Intrin[2 * 3], _T J_E_P[2 * 3], _T E[2])
{//根据相机位置，观察点，观察数据算个雅可比与误差
 //这个映射有些奇怪，用反像
 //P'= TP
 //u = -f * d * P'x/P'z
 //v = -f * d * P'y/P'z
	_T Point_4D[4] = {Point_3D[0],Point_3D[1],Point_3D[2],1};
	_T TP[4];

	Matrix_Multiply(Camera, 4, 4, Point_4D, 1, TP);	//得P'

	_T TPz_Sqr = TP[2] * TP[2];
	_T TPx_div_TPz = TP[0] / TP[2],
		TPy_div_TPz = TP[1] / TP[2];

	//r2 = (P'x^2 + P'y^2)/P'z^2
	_T r2 = (TP[0] * TP[0] + TP[1] * TP[1]) / TPz_Sqr;
	_T f=Intrinsic[0], l1 = Intrinsic[1], l2 = Intrinsic[2];
	//d=1.0 + r2 * (l1 + l2 * r2);
	_T d = 1.f + r2 * (l1 + l2 * r2);

	_T f_div_TPz = -f / TP[2];


	//各种导数
	//r2 = (P'x/P'z)^2 + (P'y/P'z)^2
	//dr2/dTPx = 2*(P'x/P'z)* (1/P'z) = 2* P'x/P'z^2
	_T dr2_TPx = 2 * TP[0] / TPz_Sqr;
	//dr2/dTPy = 2*P'y/P'z^2
	_T dr2_TPy = 2 * TP[1] / TPz_Sqr;
	//dr2/dTPz = (P'x^2 + P'y^2) * (-2) * P'z^(-3) = -2 * r2 /P'z
	_T dr2_TPz = -2 * r2 / TP[2];

	//d = 1 + r2 * l1 + l2 * r2*r2;  dd/dr2= l1 + 2*l2*r2 
	_T l2_x_2r2 = 2 * l2 * r2;
	_T dd_TPx = l1 * dr2_TPx + l2_x_2r2 * dr2_TPx; 
	_T dd_TPy = l1 * dr2_TPy + l2_x_2r2 * dr2_TPy; 
	_T dd_TPz = l1 * dr2_TPz + l2_x_2r2 * dr2_TPz; 

	//u = -f * d * P'x/P'z   du/dd = -(TPx*f)/TPz
	//du/dTPx = -f/P'z * (du/dd * TPx + d)
	_T du_TPx = f_div_TPz * (dd_TPx * TP[0] + d);
	//du/dTPy = -f * P'x/P'z  * dd_TPy 
	_T du_TPy = f_div_TPz * TP[0] * dd_TPy;
	//dd/dTPz = (dd/dPz* TPz - d)/ TPz^2
	_T du_TPz = -f * TP[0] * (dd_TPz * TP[2] - d) / TPz_Sqr;

	//v = f * d * -P'y/P'z
	_T dv_TPx = f_div_TPz * dd_TPx*TP[1];  
	_T dv_TPy = f_div_TPz * (dd_TPy * TP[1] + d);
	//dd/dTPz = (dd/dPz* TPz - d)/ TPz^2
	_T dv_TPz = -f * TP[1] * (dd_TPz * TP[2] - d) / TPz_Sqr;
	_T J_E_TP[2 * 3] = { du_TPx,du_TPy,du_TPz,
		dv_TPx,dv_TPy,dv_TPz };

	if (J_E_Intrin)
	{
		//u = f * d * (-1)P'x/P'z
		//v = f * d * (-1)P'y/P'z
		J_E_Intrin[0] = -d*TPx_div_TPz;
		J_E_Intrin[3] = -d*TPy_div_TPz;

		//d = 1 + r2 * l1 + l2 * r2*r2
		J_E_Intrin[1] = -f * TPx_div_TPz * r2;
		//dv/dl1 = -f*(P'y/P'z)*r2
		J_E_Intrin[4]= -f * TPy_div_TPz * r2;

		//du/dl2 = f*(P'y/P'z)*r2*r2
		J_E_Intrin[2] = J_E_Intrin[1] * r2;
		//dv/dl2 = -f*(P'y/P'z)*r2*r2
		J_E_Intrin[5] =  J_E_Intrin[4] * r2;
	}
	//Disp(J_E_Intrin, 2, 3, "de/dintrinsic");
	if (E)
	{
		//u = f * d * (-1)P'x/P'z
		//v = f * d * (-1)P'y/P'z
		_T u = f_div_TPz * d * TP[0];
		_T v = f_div_TPz * d * TP[1];

		E[0] = u- Point_2D[0];
		E[1] = v - Point_2D[1];
	}
	//Disp(E, 1, 2, "E");
	if (J_E_Ksi && J_E_P)
	{//再算位姿的雅可比
		_T J_TP_Ksi[3 * 6];
		Get_Deriv_TP_Ksi_1(Camera, Point_4D, J_TP_Ksi);
		Matrix_Multiply(J_E_TP, 2, 3, J_TP_Ksi, 6, J_E_Ksi);

		//de/dP
		_T R[3 * 3];  //dE/dP
		Get_R_t(Camera, R);
		Matrix_Multiply(J_E_TP, 2, 3, R, 3, J_E_P);
		//Disp(J_E_P, 2, 3, "de/dTP");
	}

	/*Disp(J_E_Ksi, 2, 6, "de/dksi");
	Disp(J_E_TP, 2, 3, "de/dTP");
	Disp(J_E_Intrin, 2, 3, "de/dintrinsic");*/
	return;
}

template<typename _T>static void BA_PnP_3D_2D_Pose_N_Point_1_LM(
	_T A[], int iOrder, _T b[], _T x[], LM_Param_g2o<_T>* poParam, int* pbResult,  //第一部分参数，解方程必备
	//第二部分参数，该问题的额外需要数据
	_T Camera_4x4[][16], int iCamera_Count, _T Intrinsic[][3], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iPoint_2D_Count)
{//对位姿与点联合优化解LM

	int i, bResult = 1;
	LM_Param_g2o<_T> oParam = *poParam;
	const int iMax_Size = 256;
	_T Buffer[iMax_Size],
		* f = Buffer;

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

	_T(*pCamera_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));
	_T(*pIntrinsic_1)[3] = (_T(*)[3])pMalloc(iCamera_Count * 3 * sizeof(_T));
	_T(*pPoint_3D_1)[3] = (_T(*)[3])pMalloc(iPoint_3D_Count * 3 * sizeof(_T));

	do {
		//oParam.Lamda = 1;     //临时设置一下

		//解分方程看看        
		Add_I_Matrix(A, iOrder, oParam.Lamda);
		//printf("Lamda:%f\n", oParam.Lamda);
		//bSave_Raw_Data("c:\\tmp\\temp\\A.bin", (unsigned char*)A, iOrder * iOrder * sizeof(_T));
		//bSave_Raw_Data("c:\\tmp\\temp\\b.bin", (unsigned char*)b, iOrder * sizeof(_T));
		Solve_Linear_Gause(A, iOrder, b, x, &bResult);
		//Disp(A, iOrder, iOrder, "A");
		//Disp(b, 1, iOrder, "b");
		//Disp(x, 1, iOrder, "Delta_X");
		// 
		//恢复方程
		Add_I_Matrix(A, iOrder, -oParam.Lamda);

		memcpy(pCamera_1, Camera_4x4, iCamera_Count * 16 * sizeof(_T));
		memcpy(pIntrinsic_1, Intrinsic, iCamera_Count * 3 * sizeof(_T));
		memcpy(pPoint_3D_1, Point_3D, iPoint_3D_Count * 3 * sizeof(_T));

		//先更新一下Camera,即把x加上去
		for (i = 0; i < iCamera_Count; i++)
		{
			_T* pCamera_Delta_6 = &x[i * 9];
			_T Camera_Delta_4x4[4 * 4]; 
			se3_2_SE3(pCamera_Delta_6, Camera_Delta_4x4);
			Matrix_Multiply(Camera_Delta_4x4, 4, 4, pCamera_1[i], 4, pCamera_1[i]);

			_T* pIntrinsic_3 = &x[i * 9+6];
			Vector_Add(pIntrinsic_1[i], pIntrinsic_3, 3, pIntrinsic_1[i]);
		}

		//再更新一下点位置，把x加上去
		_T* x1 = x + iCamera_Count * 9;
		for (i = 0; i < iPoint_3D_Count; i++)
			Vector_Add(pPoint_3D_1[i], &x1[i * 3], 3,pPoint_3D_1[i] );

		//再将所有的点算一遍，求误差
		_T fSum_e = 0;
		for (i = 0; i < iPoint_2D_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2];            
			BA_PnP_3D_2D_Pose_N_Point_1_Get_J(pPoint_3D_1[oPoint_2D.m_iPoint_Index],
				pCamera_1[oPoint_2D.m_iCamera_Index],pIntrinsic_1[oPoint_2D.m_iCamera_Index], oPoint_2D.m_Pos, (_T*)NULL, (_T*)NULL,(_T*)NULL, E);

			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
		}

		if (!bResult)   //没啥营养，就是为了跳出
			fSum_e = std::numeric_limits<_T>::max();

		//rho不过是个表示发散/收敛程度的参数
		rho = (oParam.m_fLoss - fSum_e);

		//搞个scale
		_T fScale;
		if (bResult)
		{
			fScale = (_T)1e-3;
			//从此处可以看出，Scale在解方程成功的时候起作用
			//可以想象，x就是增长步长，不断收敛，故此，fScale也不断减少
			for (int i = 0; i < iOrder; i++)
				fScale += x[i] * (oParam.Lamda * x[i] + b[i]);
		}else
			fScale = 1;

		rho /= fScale;  //显然, rho>0时表示解收敛
		if (rho > 0 && std::_Is_finite(fSum_e) && bResult)
		{//本次迭代的最后一次
		 //fScale递减，rho增大，pow增大，alpha减少，scaleFactor减少
			_T alpha = (_T)(1. - pow((2 * rho - 1), 3));
			alpha = Min(alpha, 2.f/3.f);
			_T scaleFactor = Max(1.f/3.f, alpha);

			//可见，scaleFactor在[1/3,2/3]之间，Lamda必然缩小
			oParam.Lamda *= scaleFactor;
			oParam._ni = 2;
			oParam.m_fLoss = fSum_e;
			printf("Lamda:%f\n", oParam.Lamda);
		} else
		{
			//加入对角线调整失败，则继续改下去
			oParam.Lamda*=oParam._ni;     //等于将原来的Lamda增大_ni倍，_ni则成倍增大  
			oParam._ni *= 2;            //理论上，对_ni加倍增大
			if (!std::_Is_finite(oParam.Lamda)) 
				break;
		}
		qmax++;
	}while (rho < 0  && qmax<10 && !poParam->m_bStop);	//此处本来还有个外部干涉信号可控制停止

	*pbResult = bResult;
	//若成功，则直接更新Camera,Intrinsic,Point_3D了事，省了后续的更新
	memcpy((_T*)Camera_4x4, pCamera_1, iCamera_Count * 16 * sizeof(_T));
	memcpy((_T*)Intrinsic, pIntrinsic_1, iCamera_Count * 3 * sizeof(_T));
	memcpy((_T*)Point_3D, pPoint_3D_1, iPoint_3D_Count * 3 * sizeof(_T));
	*poParam = oParam;
	Free(pCamera_1);
	Free(pIntrinsic_1);
	Free(pPoint_3D_1);
}


template<typename _T>static void BA_PnP_3D_2D_Pose_N_Point_1(_T Camera[][16], int iCamera_Count, _T Intrinsic[][3], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-20)
{//优化位姿，点，内参f和两个畸变参数
 //Camera: 所有的相机位姿，以4x4矩阵表示 
 //Point_3D： 可能有噪声的空间点位置
 //Point_2D, 各个相机对应的2D点
	int i, iIter, iResult=0;
	//for (i = 0; i < iObservation_Count; i++)
	//    printf("Camera:%d Point_3D:%d\n", Observation_2D[i].m_iCamera_Index, Observation_2D[i].m_iPoint_Index);
	const int iJ_w = iCamera_Count * 9 + iPoint_3D_Count * 3;
	_T* J = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* Jt = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Delta_X = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_H = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T)),
		* JtJ = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T));


	_T fSum_e = 0, fSum_e_Pre = (_T)1e20;
	LM_Param_g2o<_T> oLM_Param;
	Init_LM_Param(&oLM_Param);

	_T fLoss_Diff_eps = 1e-10;
	fLoss_Diff_eps*= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, iJ_w * iJ_w * sizeof(_T));
		memset(Sigma_JtE, 0, iJ_w * sizeof(_T));
		for (i = 0; i < iObservation_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2], J_E_Ksi[2 * 6], J_E_Intrin[2*3], J_E_P[2 * 3];
			BA_PnP_3D_2D_Pose_N_Point_1_Get_J(Point_3D[oPoint_2D.m_iPoint_Index],
				Camera[oPoint_2D.m_iCamera_Index],Intrinsic[oPoint_2D.m_iCamera_Index], oPoint_2D.m_Pos, J_E_Ksi,J_E_Intrin, J_E_P, E);
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差

			//将三组雅可比拷到目标位置
			memset(J, 0, 2 * iJ_w * sizeof(_T));
			Copy_Matrix_Partial(J_E_Ksi, 2, 6, J, iJ_w, oPoint_2D.m_iCamera_Index * 9, 0);
			Copy_Matrix_Partial(J_E_Intrin, 2, 3, J, iJ_w, oPoint_2D.m_iCamera_Index * 9+6, 0);
			Copy_Matrix_Partial(J_E_P, 2, 3, J, iJ_w, iCamera_Count * 9 + oPoint_2D.m_iPoint_Index * 3, 0);
			//Disp(J, 2, iJ_w, "J");

			//累加Sigma_H
			Transpose_Multiply(J, 2, iJ_w, JtJ, 0);
			Matrix_Add(Sigma_H, JtJ, iJ_w, Sigma_H);
			//Disp(Sigma_H, iJ_w, iJ_w, "Sigma_H");

			//累加Sigma_JtE;
			Matrix_Transpose(J, 2, iJ_w, Jt);
			Matrix_Multiply(Jt, iJ_w, 2, E, 1, JtE);
			Vector_Add(Sigma_JtE, JtE, iJ_w, Sigma_JtE);
		}
		printf("iIter:%d %e\n", iIter, 0.5f*fSum_e);

		//加上负号，右边才是 -J'E
		Matrix_Multiply(Sigma_JtE, 1, iJ_w, (_T)-1, Sigma_JtE);

		//方法2，LM方法
		oLM_Param.m_fLoss = fSum_e, oLM_Param.m_iIter = iIter;
		BA_PnP_3D_2D_Pose_N_Point_1_LM(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &oLM_Param, &iResult,
			Camera,iCamera_Count, Intrinsic, Point_3D, iPoint_3D_Count, Observation_2D, iObservation_Count);

		if (!iResult)
		{
			printf("Bundle Adjust LM解失败\n");
			break;
		}
		if (fSum_e<=oLM_Param.m_fLoss  || oLM_Param.m_fLoss < fLoss_eps || Abs(oLM_Param.m_fLoss-fSum_e)<fLoss_Diff_eps)
			break;
		fSum_e_Pre = oLM_Param.m_fLoss;
	}

	Free(J);
	Free(Jt);
	Free(JtE);
	Free(Delta_X);
	Free(Sigma_JtE);    
	Free(Sigma_H);
	Free(JtJ);

	return;
}

void Pose_Estimate_Test_3()
{//Simple_Bundle_Adjust Test，连f, l1, l2一起优化，用g2o方法进行LM解方程
	typedef double _T;
	//先数据
	_T(*pPoint_3D)[3], (*pCamera_Data)[3 * 3];    //只是个内参
	_T Camera[16][4 * 4], Rotation_Vector[4], R[16][3 * 3];
	_T Intrinsic[16][3];    //三个内参，一个焦距f, 两个畸变参数
	Point_2D<_T>* pPoint_2D;

	//*******************先造数据**************************************************
	int iCamera_Count, iPoint_Count, iObservation_Count;
	Temp_Load_File_2(&iCamera_Count, &iPoint_Count, &iObservation_Count, &pPoint_2D, &pPoint_3D, &pCamera_Data);
	int i;

	for (i = 0; i < iCamera_Count; i++)
	{    //把相机数据从6元组转换为4x4矩阵
		Rotation_Vector_3_2_4(pCamera_Data[i], Rotation_Vector);
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R[i]);
		Gen_Homo_Matrix(R[i], &pCamera_Data[i][3], Camera[i]);
		Intrinsic[i][0] = pCamera_Data[i][6];
		Intrinsic[i][1] = pCamera_Data[i][7];
		Intrinsic[i][2] = pCamera_Data[i][8];
	}
	//*******************先造数据**************************************************

	iCamera_Count = 16;
	iPoint_Count = 200;
	iObservation_Count = 200;

	unsigned long long tStart = iGet_Tick_Count();
	BA_PnP_3D_2D_Pose_N_Point_1(Camera, iCamera_Count,Intrinsic, pPoint_3D, iPoint_Count,
		pPoint_2D, iObservation_Count);
	printf("%lld\n", iGet_Tick_Count() - tStart);

	free(pPoint_2D);
	free(pPoint_3D);
	free(pCamera_Data);
	return;
}

template<typename _T>static void BA_PnP_3D_2D_Pose_N_Point_2_LM(
	_T A[], int iOrder, _T b[], _T x[], LM_Param_g2o<_T>* poParam, int* pbResult,  //第一部分参数，解方程必备
	//第二部分参数，该问题的额外需要数据
	_T Camera_4x4[][16], int iCamera_Count, _T Intrinsic[][3], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iPoint_2D_Count)
{//对位姿与点联合优化解LM，用近似ceres方法
	int i, bResult=0;
	LM_Param_g2o<_T> oParam = *poParam;
	if (oParam.m_iIter == 0)    //第一次，特殊对待,取对角线最大值
	{
		oParam.radius_ = 10000.f;
		oParam.decrease_factor_ = 2.f;
	}

	//Disp(b, 1, 3, "b");
	_T tau = 1.f / oParam.radius_;    // (_T)1e-4,  //ceres用1e-4    //g2o用1e-5
	//fMax = (_T)0.f;
	_T* Diag = (_T*)pMalloc(iOrder * sizeof(_T));

	for (i = 0; i < iOrder; i++)
	{
		//Diag[i] = sqrt(tau*Abs(A[i * iOrder + i]));
		Diag[i] = tau*Abs(A[i * iOrder + i]);
		//printf("%f\n", sqrt(Diag[i]));
		//限个幅
		Diag[i] = Clip3(1e-6, 1e32, Diag[i]);       
	}
	
	int	qmax = 0;
	_T(*pCamera_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));
	_T(*pIntrinsic_1)[3] = (_T(*)[3])pMalloc(iCamera_Count * 3 * sizeof(_T));
	_T(*pPoint_3D_1)[3] = (_T(*)[3])pMalloc(iPoint_3D_Count * 3 * sizeof(_T));
	do {
		//解分方程看看        
		for (i = 0; i < iOrder; i++)
			A[i * iOrder + i] += Diag[i];   // *Diag[i];

		Solve_Linear_Gause(A, iOrder, b, x, &bResult);
		//Disp(A, iOrder, iOrder, "A");
		//Disp(x, 1, iOrder, "x");

		//恢复方程
		for (i = 0; i < iOrder; i++)
			A[i * iOrder + i] -= Diag[i];   // *Diag[i];

		memcpy(pCamera_1, Camera_4x4, iCamera_Count * 16 * sizeof(_T));
		memcpy(pIntrinsic_1, Intrinsic, iCamera_Count * 3 * sizeof(_T));
		memcpy(pPoint_3D_1, Point_3D, iPoint_3D_Count * 3 * sizeof(_T));

		//先更新一下Camera,即把x加上去
		for (i = 0; i < iCamera_Count; i++)
		{
			_T* pCamera_Delta_6 = &x[i * 9];
			_T Camera_Delta_4x4[4 * 4]; 
			se3_2_SE3(pCamera_Delta_6, Camera_Delta_4x4);
			Matrix_Multiply(Camera_Delta_4x4, 4, 4, pCamera_1[i], 4, pCamera_1[i]);

			_T* pIntrinsic_3 = &x[i * 9+6];
			Vector_Add(pIntrinsic_1[i], pIntrinsic_3, 3, pIntrinsic_1[i]);
		}

		//再更新一下点位置，把x加上去
		_T* x1 = x + iCamera_Count * 9;
		for (i = 0; i < iPoint_3D_Count; i++)
			Vector_Add(pPoint_3D_1[i], &x1[i * 3], 3,pPoint_3D_1[i] );

		//再将所有的点算一遍，求误差
		_T fSum_e = 0;
		for (i = 0; i < iPoint_2D_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2];            
			BA_PnP_3D_2D_Pose_N_Point_1_Get_J(pPoint_3D_1[oPoint_2D.m_iPoint_Index],
				pCamera_1[oPoint_2D.m_iCamera_Index],pIntrinsic_1[oPoint_2D.m_iCamera_Index], oPoint_2D.m_Pos, (_T*)NULL, (_T*)NULL,(_T*)NULL, E);

			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
		}

		if (!bResult)   //没啥营养，就是为了跳出
			fSum_e = std::numeric_limits<_T>::max();
		/*else
		fSum_e *= 0.5f;*/

		if (fSum_e < oParam.m_fLoss)
		{//本次迭代的最后一次
			/*_T relative_decrease = (oParam.m_fLoss - fSum_e) / oParam.m_fLoss;
			_T historical_relative_decrease =
				(oParam.m_fLoss - fSum_e) /
				(oParam.accumulated_reference_model_cost_change_ + oParam.m_fLoss);

			relative_decrease = Max(relative_decrease, historical_relative_decrease);*/
			//printf("%f\n", 1.0 - pow(2.0 * relative_decrease - 1.0, 3));
			//oParam.radius_ = oParam.radius_ / max(1.0 / 3.0, 1.0 - pow(2.0 * relative_decrease - 1.0, 3));
			oParam.radius_ *= 2;
			oParam.accumulated_reference_model_cost_change_ += oParam.m_fLoss;
			oParam.m_fLoss = fSum_e;
			oParam.decrease_factor_ = 2.0;  //将放大倍数恢复为2.0
			break;
		}else
		{
			oParam.radius_ /= 2.f;
			oParam.decrease_factor_ *= 2;
		}
		qmax++;
	}while (qmax<10 && !poParam->m_bStop);	//此处本来还有个外部干涉信号可控制停止
	*pbResult = bResult;

	//若成功，则直接更新Camera,Intrinsic,Point_3D了事，省了后续的更新
	memcpy((_T*)Camera_4x4, pCamera_1, iCamera_Count * 16 * sizeof(_T));
	memcpy((_T*)Intrinsic, pIntrinsic_1, iCamera_Count * 3 * sizeof(_T));
	memcpy((_T*)Point_3D, pPoint_3D_1, iPoint_3D_Count * 3 * sizeof(_T));
	*poParam = oParam;
	Free(pCamera_1);
	Free(pIntrinsic_1);
	Free(pPoint_3D_1);
	Free(Diag);
	return;
}
template<typename _T>static void BA_PnP_3D_2D_Pose_N_Point_2(_T Camera[][16], int iCamera_Count, _T Intrinsic[][3], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-20)
{//优化位姿，点，内参f和两个畸变参数
 //Camera: 所有的相机位姿，以4x4矩阵表示 
 //Point_3D： 可能有噪声的空间点位置
 //Point_2D, 各个相机对应的2D点
	int i, iIter, iResult=0;
	//for (i = 0; i < iObservation_Count; i++)
	//    printf("Camera:%d Point_3D:%d\n", Observation_2D[i].m_iCamera_Index, Observation_2D[i].m_iPoint_Index);
	const int iJ_w = iCamera_Count * 9 + iPoint_3D_Count * 3;
	_T* J = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* Jt = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Delta_X = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_H = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T)),
		* JtJ = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T));


	_T fSum_e = 0, fSum_e_Pre = (_T)1e20;
	LM_Param_g2o<_T> oLM_Param;
	Init_LM_Param(&oLM_Param);

	_T fLoss_Diff_eps = 1e-10;
	fLoss_Diff_eps*= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, iJ_w * iJ_w * sizeof(_T));
		memset(Sigma_JtE, 0, iJ_w * sizeof(_T));
		for (i = 0; i < iObservation_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2], J_E_Ksi[2 * 6], J_E_Intrin[2*3], J_E_P[2 * 3];
			BA_PnP_3D_2D_Pose_N_Point_1_Get_J(Point_3D[oPoint_2D.m_iPoint_Index],
				Camera[oPoint_2D.m_iCamera_Index],Intrinsic[oPoint_2D.m_iCamera_Index], oPoint_2D.m_Pos, J_E_Ksi,J_E_Intrin, J_E_P, E);
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差

			//将三组雅可比拷到目标位置
			memset(J, 0, 2 * iJ_w * sizeof(_T));
			Copy_Matrix_Partial(J_E_Ksi, 2, 6, J, iJ_w, oPoint_2D.m_iCamera_Index * 9, 0);
			Copy_Matrix_Partial(J_E_Intrin, 2, 3, J, iJ_w, oPoint_2D.m_iCamera_Index * 9+6, 0);
			Copy_Matrix_Partial(J_E_P, 2, 3, J, iJ_w, iCamera_Count * 9 + oPoint_2D.m_iPoint_Index * 3, 0);
			//Disp(J, 2, iJ_w, "J");

			//累加Sigma_H
			Transpose_Multiply(J, 2, iJ_w, JtJ, 0);
			Matrix_Add(Sigma_H, JtJ, iJ_w, Sigma_H);
			//Disp(Sigma_H, iJ_w, iJ_w, "Sigma_H");

			//累加Sigma_JtE;
			Matrix_Transpose(J, 2, iJ_w, Jt);
			Matrix_Multiply(Jt, iJ_w, 2, E, 1, JtE);
			Vector_Add(Sigma_JtE, JtE, iJ_w, Sigma_JtE);
		}
		printf("iIter:%d %e\n", iIter, 0.5f*fSum_e);

		//加上负号，右边才是 -J'E
		Matrix_Multiply(Sigma_JtE, 1, iJ_w, (_T)-1, Sigma_JtE);

		//方法2，LM方法
		oLM_Param.m_fLoss = fSum_e, oLM_Param.m_iIter = iIter;
		BA_PnP_3D_2D_Pose_N_Point_2_LM(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &oLM_Param, &iResult,
			Camera,iCamera_Count, Intrinsic, Point_3D, iPoint_3D_Count, Observation_2D, iObservation_Count);

		if (!iResult)
		{
			printf("Bundle Adjust LM解失败\n");
			break;
		}
		if (fSum_e<=oLM_Param.m_fLoss  || oLM_Param.m_fLoss < fLoss_eps || Abs(oLM_Param.m_fLoss-fSum_e)<fLoss_Diff_eps)
			break;
		fSum_e_Pre = oLM_Param.m_fLoss;
	}

	Free(J);
	Free(Jt);
	Free(JtE);
	Free(Delta_X);
	Free(Sigma_JtE);    
	Free(Sigma_H);
	Free(JtJ);

	return;
}
void Pose_Estimate_Test_4()
{//Simple_Bundle_Adjust Test，连f, l1, l2一起优化
	typedef double _T;
	//先数据
	_T(*pPoint_3D)[3], (*pCamera_Data)[3 * 3];    //只是个内参
	_T Camera[16][4 * 4], Rotation_Vector[4], R[16][3 * 3];
	_T Intrinsic[16][3];    //三个内参，一个焦距f, 两个畸变参数
	Point_2D<_T>* pPoint_2D;

	//*******************先造数据**************************************************
	int iCamera_Count, iPoint_Count, iObservation_Count;
	Temp_Load_File_2(&iCamera_Count, &iPoint_Count, &iObservation_Count, &pPoint_2D, &pPoint_3D, &pCamera_Data);
	int i;

	for (i = 0; i < iCamera_Count; i++)
	{    //把相机数据从6元组转换为4x4矩阵
		Rotation_Vector_3_2_4(pCamera_Data[i], Rotation_Vector);
		Rotation_Vector_4_2_Matrix(Rotation_Vector, R[i]);
		Gen_Homo_Matrix(R[i], &pCamera_Data[i][3], Camera[i]);
		Intrinsic[i][0] = pCamera_Data[i][6];
		Intrinsic[i][1] = pCamera_Data[i][7];
		Intrinsic[i][2] = pCamera_Data[i][8];
	}
	//*******************先造数据**************************************************

	iCamera_Count = 16;
	iPoint_Count = 100;
	iObservation_Count = 100;

	unsigned long long tStart = iGet_Tick_Count();
	BA_PnP_3D_2D_Pose_N_Point_2(Camera, iCamera_Count,Intrinsic, pPoint_3D, iPoint_Count,
		pPoint_2D, iObservation_Count);
	printf("%lld\n", iGet_Tick_Count() - tStart);

	return;
}
void Cholosky_Test_2()
{//尝试一下用cholosky解方程
	//时间证明，稠密矩阵用cholosky分解乃越优越慢，毫无意义
	typedef double _T;
	const int iOrder = 200 * 3 + 16 * 9;
	_T* A, * b, * x = (_T*)pMalloc(iOrder * sizeof(_T));
	int i,iResult;
	unsigned long long tStart;
	bLoad_Raw_Data("c:\\tmp\\temp\\A.bin", (unsigned char**)&A,NULL);
	bLoad_Raw_Data("c:\\tmp\\temp\\b.bin", (unsigned char**)&b,NULL);
	tStart = iGet_Tick_Count();
	//for(i=0;i<1000;i++)
		Solve_Linear_Gause(A, iOrder, b, x, &iResult);
	//Test_Linear(A, iOrder, b, x);
	//printf("%lld\n", iGet_Tick_Count() - tStart);

	//Cholosky解法
	//Ax = BB'x = b, 设B'x = y
	//设By = b，先解y
	_T* B = (_T*)pMalloc(iOrder * iOrder * sizeof(_T));
	tStart = iGet_Tick_Count();
	for (i = 0; i < 100; i++)
	{
		Cholosky_Decompose(A, iOrder, B, &iResult);
		Solve_Linear_Gause(B, iOrder, b, x, &iResult);	
		Matrix_Transpose(B, iOrder, iOrder, B);
		Solve_Linear_Gause(B, iOrder, x,x, &iResult);
	}
	printf("%lld\n", iGet_Tick_Count() - tStart);
	Test_Linear(A, iOrder, b, x);

	free(A);
	free(b);
	Free(x);
	Free(B);
	return;
}

void Solve_Linear_AAt_Test()
{
	typedef double _T;
	const int iOrder = 200;
	_T A[iOrder * iOrder];
	int i, j, iResult;
	_T x[iOrder], B[iOrder];

	for (i = 0; i < iOrder; i++)
		B[i] = iGet_Random_No();
	_T fTotal = 0;
	int iCount = 0;
	unsigned long long tStart = iGet_Tick_Count();
	for (i = 0; i < 5000; i++)
	{
		for (j = 0; j < 2 * iOrder; j++)
			A[j] = iGet_Random_No() % 100;
		Transpose_Multiply(A, 2, iOrder, A, 0);
		Add_I_Matrix(A, iOrder, (_T)(iGet_Random_No() % 5000));
		//Disp(A, iOrder, iOrder, "A");
		Solve_Linear_Gause_AAt(A, iOrder, B, x, &iResult);
		//Solve_Linear_Gause(A, iOrder, B, x, &iResult);

		////算误差和
		//if (iResult)
		//{
		//    _T fError=fLinear_Equation_Check(A, iOrder, B, x);
		//    /*if (fError > 1)
		//        printf("here");*/
		//    fTotal += fError;            
		//}		
	}
	printf("%lld Error:%f\n", iGet_Tick_Count() - tStart,fTotal);

	//Disp(A, iOrder, iOrder, "A");
	return;
}

void Test_Main()
{
	//Point_Cloud_Test();				//点云重建最简例子，没营养

	//Pose_Graph_Test_1();			//位姿图优化，暴力解
	//Pose_Graph_Test_2();			//位姿图优化，引入信息矩阵，无卵用
	//Pose_Graph_Test_3();			//位姿图优化，用源信息矩阵，无卵用

	//以下两个例子已经基本等价于ceres的精度，只欠cholosky优化
	//Pose_Estimate_Test_1();			//相当于BA_Demo，只估计位姿
	//Pose_Estimate_Test_2();			//相当于BA_Demo，位姿与点一起调整
	//Pose_Estimate_Test_3();			//用了problem-16-22106-pre中100个样本，但是没有优化到所有参数
										//这个例子理论上连focal, 两个畸变参数一起优化，远比只优化位姿与点好
										//但是，优化focal，畸变参数有没有用？这不应该是内参吗
										//100点已经等价ceres效果
	//Pose_Estimate_Test_4();			//同Pose_Estimate_Test_3，LM用了Ceres类似方法修改对角线
										//有些许的加速收敛，有了这个原型可以轻易写出schur法
									
	//BA_Test_Schur_Ref();
	//Schur_Test();				//Schur消元法解大样本例
	//BA_Test_1();
	//BA_Test_2();				//Ba方法求解PnP问题
	//BA_Test_3_3();				//3000点BA后端
	
	//General_BA_Test_1();		//一种更简洁的BA方法，单独搞J矩阵
	//Generic_BA_Test_2();		//估计二维矩阵
	//Generic_BA_Test_3();		//估计三维矩阵

	//Transform_Example_2D();	//二维下的旋转，位移变换

	//Sparse_Matrix_Test();	//稀疏矩阵实验，等于ICP_Test_4
	//ICP_Test_1();	//两图ICP BA方法
	//ICP_Test_2();	//两图ICP SVD方法
	//ICP_Test_3();	//两图ICP 实验，自己造数据
	//ICP_Test_4();	//闭环实验，仅估计位姿
	//ICP_Test_5();	//最简三图闭环实验
		
	//Least_Square_Test_4();	//一阶梯度法
	//Least_Square_Test_5();	//二阶梯度法
	//Least_Square_Test_6();	//高斯牛顿法

	//E_Test_1();	
	//E_Test_2();	//对E矩阵进行验算实验
	//E_Test_3();		//自己生成一个球数据测试E矩阵

	//H_Test_1();		//单应矩阵实验，验证归一化平面与像素平面下H矩阵的有效性
	//H_Test_2();		//反面例子，球面点估计不出一个H矩阵
	//H_Test_3();			//验证张定友标定法中的特殊H矩阵，与一般H矩阵只差一个scale，即等价

	DLT_Test_1();		//此处DLT做了优化，目前小范围内优化版本精度与速度更好
	
	////4个Sift实验，各种接口场合
	//Sift_Test_1();
	//Sift_Test_2();
	//Sift_Test_3();
	//Sift_Test_4();

	//SVD_Test_1();		//作为SVD分解实验还是不错的
	//E_Test_2();		//这个例子非常简洁
	//Ransac_Test();	//Ransac实验

	//Camera_Intrinsic_Test();	//内参实验
	//Camera_Extrinsic_Test_1();	//相机参数实验
	//Camera_Extrinsic_Test_2();	//相机参数实验
	
	//Sphere_Test_1();	//4基站定位实验，二维方法
	//Sphere_Test_2();	//4基站定位实验，三维方法
	//Sphere_Test_3();	//4基站定位实验，三维方法解线性方程法

	//Epipolar_Search_Test();	//极线搜索实验

	//Cholosky_Test_1();	//Cholosky分解实验
	//Solve_Linear_AAt_Test();	//尝试解AAt x = b 的快速方法，有精度问题，不知有没有现实意义

	//从以下两个试验看出，同样的数据不同的方法做出来的误差很大
	//Sift_And_Estimate_E_Test_1();	//通过两图匹配估计出一个E矩阵
	//Sift_And_Estimate_H_Test_1();	//通过两图匹配估计出一个H矩阵
}