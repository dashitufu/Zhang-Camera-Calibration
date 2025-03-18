//���ļ���������е������ʵ��
#include "Reconstruct.h"
//void Test_1()
//{//��һ���Լ��㼫������
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
//	//E�������
//	Ransac_Report oReport;
//	Ransac_Estimate_E(oParam.m_pImage_Point_0, oParam.m_pImage_Point_1, iCount, 640, 240, 320, &oReport);
//	//Disp_Ransac_Report(oReport);
//
//	//���������ͷŶ����ڴ�
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
	typedef double _T;	//�������ݾ���

	{//��һ�ᣬ��תĳ�����ȣ��õ�һ����ת���󣬾�����Ⱥ�ϵ�һ��Ԫ��
		_T R[3 * 3];
		_T Rotation_Vector[4] = { 1,2,3 };	//��ת��������ά��ʽ
		Disp(Rotation_Vector, 1, 3, "3ά��ת������ʽ");

		//����ά��ʽת��Ϊ4ά��ʽ���������Ĵ��뼴��֪�����ܼ򵥣�ֻ�Ǹ���һ����ģ��Ϊtheta
		Rotation_Vector_3_2_4(Rotation_Vector, Rotation_Vector);
		Disp(Rotation_Vector, 1, 4, "4ά��ת������ʽ");

		//��ת������4ά��ʽת��ά��ת����,����������Ĵ��뼴��֪���任����
		Rotation_Vector_2_Matrix(Rotation_Vector, R);
		Disp(R, 3, 3, "��ת����");

		//���º�������һ�������Ƿ�ά��ת���󣬴Ӵ�����Կ���
		//�жϵ����ݣ�Ŀǰֻ�ӱ�׼�����жϣ���������һ������
		//�Ƿ�ֻҪ�Ǳ�׼������3�׷��������ת����
		if(bIs_R(R))
			printf("���Ǹ���ת����\n");
		else
			printf("�ⲻ����ת����\n");
	}
	
	{//��һ����ת�����ķ⣬�ᣬ�ۣ�������
		//����������ת����
		_T R1[3 * 3], R2[3 * 3];
		_T Rotation_Vector_1[4] = { 1,2,3 },	//��ת��������ά��ʽ
			Rotation_Vector_2[4] = { 7,8,9 };
		Rotation_Vector_3_2_4(Rotation_Vector_1, Rotation_Vector_1);
		Rotation_Vector_3_2_4(Rotation_Vector_2, Rotation_Vector_2);
		Rotation_Vector_2_Matrix(Rotation_Vector_1, R1);
		Rotation_Vector_2_Matrix(Rotation_Vector_2, R2);
		Disp(R1, 3, 3, "R1");Disp(R2, 3, 3, "R2");
		//���������ˣ���������

		//����ԣ���ת���������Ϊ�˷�������������˻���һ����ת����
		_T R3[3 * 3];
		Matrix_Multiply(R1, 3, 3, R2, 3, R3);
		if(bIs_R(R3))
			printf("���Ǹ���ת����\n");
		else
			printf("�ⲻ����ת����\n");

		//�����
		_T R4[3 * 3];
		Matrix_Multiply(R1, 3, 3, R2, 3, R4);
		Matrix_Multiply(R3, 3, 3, R4, 3, R4);
		Disp(R4, 3, 3, "(R1xR2)xR3");

		Matrix_Multiply(R2, 3, 3, R3, 3, R4);
		Matrix_Multiply(R1, 3, 3, R4, 3, R4);
		Disp(R4, 3, 3, "R1x(R2xR3)");

		//��Ԫ
		Gen_I_Matrix(R4, 3, 3);	//��λ���������Ԫ
		Matrix_Multiply(R1, 3, 3, R4, 3, R4);;
		Disp(R4, 3, 3, "R1xI");
		Disp(R1, 3, 3, "R1");

		//����
		Get_Inv_Matrix_Row_Op_2(R1, R4, 3);
		Disp(R4, 3, 3, "R1�������");
		if(bIs_R(R4))	//�ɼ������滹����ת����
			printf("���Ǹ���ת����\n");
		else
			printf("�ⲻ����ת����\n");
		Matrix_Multiply(R1, 3, 3, R4, 3, R4);
		//���¿ɼ�����ת��������
		Disp(R4, 3, 3, "R1 x R1_inv");
	}
}
void Rotation_Matrix_Test_2()
{//�ڶ���ʵ�飬��Ԫ������ת������ת����֮���ת��
	typedef double _T;	//�������ݾ���
	{//��һ�ᣬ��תĳ�����ȣ��õ�һ����ת���󣬾�����Ⱥ�ϵ�һ��Ԫ��
		_T R[3 * 3];
		_T Rotation_Vector_1[4] = { 1,2,3 };	//��ת��������ά��ʽ
		_T Q[4];							//��Ԫ��

		//�Ƚ���ת����Ϊ4ά������ʽ
		Rotation_Vector_3_2_4(Rotation_Vector_1, Rotation_Vector_1);

		//��ת������4Ԫ��
		Disp(Rotation_Vector_1, 1, 4, "4D Rotation_Vector");
		Rotation_Vector_2_Quaternion(Rotation_Vector_1, Q);
		Disp(Q, 1, 4, "Quaternion");

		//��Ԫ������ת����
		_T Rotation_Vector_2[4];
		Quaternion_2_Rotation_Vector(Q, Rotation_Vector_2);
		Disp(Rotation_Vector_2, 1, 4, "4D Rotation_Vector");	//����ǰ�����һ�����

		//��Ԫ��ת��ά��ת����
		Quaternion_2_Rotation_Matrix(Q, R);
		Disp(R, 3, 3, "R");

		//��ת������Ԫ��
		_T Q1[4];
		Rotation_Matrix_2_Quaternion(R, Q1);
		//���Ĵ˴��ķ��ţ����Ӿ�Slam�У��������Ż���ȡ����ʱ�򣬴󲿷ֲ�Ӱ����
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
	//��ת����ʵ��
	//Rotation_Matrix_Test_1();	
	//Rotation_Matrix_Test_2();
	Test_1();
}