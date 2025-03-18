//本文件打算放所有的李代数实验
#include "Reconstruct.h"
//void Test_1()
//{//试一下自己搞极限搜索
//	typedef float _T;
//	//_T K[] = { 520.9f, 0, 325.1f, 0, 521.0f, 249.7f, 0, 0, 1 };
//	_T K[] = { 521.f, 0, 325.1f, 0, 521.f, 249.7f, 0, 0, 1 };
//	Image_Match_Param<_T> oParam = { 0 };
//	int iCount;
//
//	Match_2_Image("D:\\Software\\3rdparty\\slambook2\\ch7\\1.bmp", "D:\\Software\\3rdparty\\slambook2\\ch7\\1.Depth",
//		"D:\\Software\\3rdparty\\slambook2\\ch7\\2.bmp", "D:\\Software\\3rdparty\\slambook2\\ch7\\2.Depth",
//		5000.f, K, &iCount,
//		&oParam.m_pImage_Point_0, &oParam.m_pImage_Point_1,
//		&oParam.m_pNorm_Point_0, &oParam.m_pNorm_Point_1,
//		&oParam.m_pPoint_3D_0, &oParam.m_pPoint_3D_1,NULL,NULL, &oParam.m_oImage_2);
//
//	//E矩阵估计
//	Ransac_Report oReport;
//	Ransac_Estimate_E(oParam.m_pImage_Point_0, oParam.m_pImage_Point_1, iCount, 640, 240, 320, &oReport);
//	//Disp_Ransac_Report(oReport);
//
//	//估计完了释放多余内存
//	Shrink_Match_Point(&oParam.m_pImage_Point_0, &oParam.m_pImage_Point_1, oReport.m_pInlier_Mask, iCount);
//	iCount = oReport.m_oSupport.m_iInlier_Count;
//
//	Normalize_Point(oParam.m_pImage_Point_0, oParam.m_pImage_Point_1, iCount, oParam.m_pNorm_Point_0, oParam.m_pNorm_Point_1, K[0], K[2], K[5]);
//
//	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T)),
//		R[3 * 3], t[3],T0[4*4],T1[4*4];
//	E_2_R_t((_T*)oReport.m_Modal, oParam.m_pNorm_Point_0, oParam.m_pNorm_Point_1, iCount, R, t, pPoint_3D, &iCount);
//	Gen_I_Matrix(T0, 4, 4);
//	Gen_Homo_Matrix(R, t, T1);
//
//	Test_T(pPoint_3D, oParam.m_pNorm_Point_0, T0, iCount);
//	Test_T(pPoint_3D, oParam.m_pNorm_Point_1, T1, iCount);
//
//	Free_2_Image_Match(&oParam);
//	Free_Report(oReport);
//	Free(pPoint_3D);
//	return;
//}

void Rotation_Matrix_Test_1()
{
	typedef double _T;	//随便改数据精度

	{//绕一轴，旋转某个弧度，得到一个旋转矩阵，就是李群上的一个元素
		_T R[3 * 3];
		_T Rotation_Vector[4] = { 1,2,3 };	//旋转向量的三维形式
		Disp(Rotation_Vector, 1, 3, "3维旋转向量形式");

		//将三维形式转换为4维形式，看函数的代码即可知道，很简单，只是个归一化，模即为theta
		Rotation_Vector_3_2_4(Rotation_Vector, Rotation_Vector);
		Disp(Rotation_Vector, 1, 4, "4维旋转向量形式");

		//旋转向量的4维形式转换维旋转矩阵,看这个函数的代码即可知道变换过程
		Rotation_Vector_2_Matrix(Rotation_Vector, R);
		Disp(R, 3, 3, "旋转向量");

		//以下函数测试一个矩阵是否维旋转矩阵，从代码可以看出
		//判断的依据：目前只从标准正交判断，所以留下一个疑问
		//是否只要是标准正交的3阶方阵就是旋转矩阵
		if(bIs_R(R))
			printf("这是个旋转矩阵\n");
		else
			printf("这不是旋转矩阵\n");
	}
	
	{//试一下旋转向量的封，结，幺，逆性质
		//先做两个旋转矩阵
		_T R1[3 * 3], R2[3 * 3];
		_T Rotation_Vector_1[4] = { 1,2,3 },	//旋转向量的三维形式
			Rotation_Vector_2[4] = { 7,8,9 };
		Rotation_Vector_3_2_4(Rotation_Vector_1, Rotation_Vector_1);
		Rotation_Vector_3_2_4(Rotation_Vector_2, Rotation_Vector_2);
		Rotation_Vector_2_Matrix(Rotation_Vector_1, R1);
		Rotation_Vector_2_Matrix(Rotation_Vector_2, R2);
		Disp(R1, 3, 3, "R1");Disp(R2, 3, 3, "R2");
		//数据做好了，两个矩阵

		//封闭性，旋转矩阵的运算为乘法。两个矩阵相乘还是一个旋转矩阵
		_T R3[3 * 3];
		Matrix_Multiply(R1, 3, 3, R2, 3, R3);
		if(bIs_R(R3))
			printf("这是个旋转矩阵\n");
		else
			printf("这不是旋转矩阵\n");

		//结合律
		_T R4[3 * 3];
		Matrix_Multiply(R1, 3, 3, R2, 3, R4);
		Matrix_Multiply(R3, 3, 3, R4, 3, R4);
		Disp(R4, 3, 3, "(R1xR2)xR3");

		Matrix_Multiply(R2, 3, 3, R3, 3, R4);
		Matrix_Multiply(R1, 3, 3, R4, 3, R4);
		Disp(R4, 3, 3, "R1x(R2xR3)");

		//幺元
		Gen_I_Matrix(R4, 3, 3);	//单位矩阵就是幺元
		Matrix_Multiply(R1, 3, 3, R4, 3, R4);;
		Disp(R4, 3, 3, "R1xI");
		Disp(R1, 3, 3, "R1");

		//求逆
		Get_Inv_Matrix_Row_Op_2(R1, R4, 3);
		Disp(R4, 3, 3, "R1的逆矩阵");
		if(bIs_R(R4))	//可见，其逆还是旋转矩阵
			printf("这是个旋转矩阵\n");
		else
			printf("这不是旋转矩阵\n");
		Matrix_Multiply(R1, 3, 3, R4, 3, R4);
		//以下可见，旋转矩阵有逆
		Disp(R4, 3, 3, "R1 x R1_inv");
	}
}
void Rotation_Matrix_Test_2()
{//第二个实验，四元数，旋转矩阵，旋转向量之间的转换
	typedef double _T;	//随便改数据精度
	{//绕一轴，旋转某个弧度，得到一个旋转矩阵，就是李群上的一个元素
		_T R[3 * 3];
		_T Rotation_Vector_1[4] = { 1,2,3 };	//旋转向量的三维形式
		_T Q[4];							//四元数

		//先将旋转矩阵化为4维向量形式
		Rotation_Vector_3_2_4(Rotation_Vector_1, Rotation_Vector_1);

		//旋转向量到4元数
		Disp(Rotation_Vector_1, 1, 4, "4D Rotation_Vector");
		Rotation_Vector_2_Quaternion(Rotation_Vector_1, Q);
		Disp(Q, 1, 4, "Quaternion");

		//四元数到旋转向量
		_T Rotation_Vector_2[4];
		Quaternion_2_Rotation_Vector(Q, Rotation_Vector_2);
		Disp(Rotation_Vector_2, 1, 4, "4D Rotation_Vector");	//可以前后对照一下误差

		//四元数转换维旋转矩阵
		Quaternion_2_Rotation_Matrix(Q, R);
		Disp(R, 3, 3, "R");

		//旋转矩阵到四元数
		_T Q1[4];
		Rotation_Matrix_2_Quaternion(R, Q1);
		//留心此处的符号，在视觉Slam中，经常符号会有取反的时候，大部分不影响结果
		Disp(Q1, 1, 4, "Quaternion");	
	}

	{
		_T Rotation_Vector[4] = { -0.96,0,	0 };
		_T Q[4];
		Rotation_Vector_3_2_4(Rotation_Vector,Rotation_Vector);
		Rotation_Vector_2_Quaternion(Rotation_Vector, Q);
		Disp(Q, 1, 4, "Q");
	}
}
static void Test_1()
{
	int i;
#define BUFFER_SIZE 1000
	int Buffer[BUFFER_SIZE];

	while (1)
	{
		for (i = 0; i < BUFFER_SIZE; i++)
			Buffer[i] = iGet_Random_No() % 100;
		Quick_Sort(Buffer, 0, BUFFER_SIZE - 1);
		for (i = 1; i < BUFFER_SIZE; i++)
			if (Buffer[i - 1] > Buffer[i])
				printf("err");
	}
	return;
}
void Lie_Test_Main()
{
	//旋转矩阵实验
	//Rotation_Matrix_Test_1();	
	//Rotation_Matrix_Test_2();
	Test_1();
}