#include "Reconstruct.h"
//һ��imu����
//IMU 1624426287.22854877 0.000680678408277777028 -0.000532325421858261482 0.000656243798749856877 -0.605724081666666581 0.0254972899999999988 9.80681344416666789
template<typename _T> struct IMU {
	_T a[3];	//���ٶȣ�x,y,z���������϶��м��ٶ�
	_T w[3];	//���ٶȣ������������н��ٶ�
};
template<typename _T>int bRead_IMU_Line(char* pcLine, IMU<_T>*poIMU)
{
	IMU<_T> oIMU;
	int iResult=sscanf(pcLine, "IMU %lf %lf %lf %lf %lf %lf", &oIMU.a[0], &oIMU.a[1], &oIMU.a[2], &oIMU.w[0], &oIMU.w[1], &oIMU.w[2]);
	*poIMU = oIMU;
	return iResult == 6;
}
template<typename _T>void Disp_IMU(IMU<_T> oIMU)
{
	printf("a: %f\t%f\t%f\n", oIMU.a[0], oIMU.a[1], oIMU.a[2]);
	printf("w: %f\t%f\t%f\n", oIMU.w[0], oIMU.w[1], oIMU.w[2]);
}

template<typename _T>void Exp(_T w[3], _T theta, _T B[3 * 3])
{
	//e^(theta*w^) = cos(theta)* I + (1-cos(theta)) �ئ�' + sin(theta)��^
	_T fCos_theta = cos(theta),
		fSin_theta = sin(theta);
	_T w_Hat[3 * 3], wwt[3 * 3];
	Hat(w,w_Hat);
	Matrix_Multiply(w_Hat,3,3, fSin_theta, w_Hat);

	Transpose_Multiply(w, 3, 1, wwt);
	Matrix_Multiply(wwt, 3, 3, 1.f - fCos_theta, wwt);

	Gen_I_Matrix(B, 3, 3);
	Matrix_Multiply(B, 3,3,fCos_theta, B);

	Matrix_Add(B, wwt,3, B);
	Matrix_Add(B, w_Hat, 3, B);

	return;
}
template<typename _T>void Get_R_t1(_T R_t0[3 * 3], _T w[3],_T R_t1[3*3])
{//�뵱Ȼ����IMU������wֵ��t0ʱ�̵�R������R_t1
//����delta tΪʱ��Ƭ=1�� w�Ѿ����� delta t��һ��֮��ı������е���
	_T w_Exp[3 * 3];
	//����exp(w%);
	_T w1[4];
	//�˴��ؼ����ȵý�w�ֽ�Ϊ 4Ԫ��ת����
	Rotation_Vector_3_2_4(w, w1);
	Exp(w1, (_T)w1[3], w_Exp);
	//Disp(w_Exp, 3, 3, "w_Exp");

	////�����òο��㷨���㣬�ɼ�����ֵ�Ѿ��ܽӽ�
	//_T w_Hat[3 * 3], w_Exp_1[3*3];
	//Hat(w, w_Hat);
	//Exp_Ref(w_Hat, 3, w_Exp_1,1e-10);
	//Disp(w_Exp_1, 3, 3, "w_Exp_1");
	//Vector_Minus(w_Exp, w_Exp_1, 3 * 3, w_Exp_1);
	//printf("%e\n", fGet_Mod(w_Exp_1, 9));

	//R(t1) = R(to)*exp(w^delta_t)	
	Matrix_Multiply(R_t0, 3, 3, w_Exp,3, R_t1);

	return;
}

template<typename _T>void Rotation_Vector_4_2_w(_T Rotation_Vector_4[3], _T w[3])
{//δ�㶨
	_T J[3*3];
	Get_J_by_Rotation_Vector(Rotation_Vector_4, J);
	Matrix_Multiply(J, 3, 3, Rotation_Vector_4, 1, w);
	Matrix_Multiply(w, 1, 3, Rotation_Vector_4[3], w);
	return;
}
template<typename _T>void Rotation_Vector_3_2_w(_T Rotation_Vector_3[3], _T w[3])
{//δ�㶨
	_T Rotation_Vector_4[4];
	Rotation_Vector_3_2_4(Rotation_Vector_3, Rotation_Vector_4);

	//Ҫ����ת������
	Rotation_Vector_4_2_w(Rotation_Vector_4, w);
	return;
}
template<typename _T>void w_2_Rotation_Vector_3(_T w[3], _T Rotation_Vector_3[3])
{//δ�㶨
	_T w_4[4];
	union {
		_T J[3 * 3];
		_T J_Inv[3 * 3];
	};
	Rotation_Vector_3_2_4(w, w_4);
	Get_J_by_Rotation_Vector(w_4, J);
	int iResult;
	Get_Inv_Matrix_Row_Op_2(J, J_Inv, 3, &iResult);
	Matrix_Multiply(J_Inv, 3, 3, w, 1, Rotation_Vector_3);

	return;
}
template<typename _T>void Get_Derive_R_t(_T R[3*3], _T w[3], _T R_Deriv[3*3])
{//����ת��ʱ��t���� dR/dt = R * w^
	_T w_Hat[3 * 3];
	Hat(w, w_Hat);
	Matrix_Multiply(R, 3, 3, w_Hat, 3, R_Deriv);
	return;
}

template<typename _T>void Get_Deriv_T_t(_T T[4 * 4], _T w[3], _T v[3],_T T_Deriv[4*4])
{//�÷���1�� dT/dt =	Rw^	v
//						0	0
	union {
		_T R[3 * 3];
		_T R_Deriv[3 * 3];
	};
	Get_R_t(T, R);
	_T w_Hat[3 * 3];
	Hat(w, w_Hat);
	Matrix_Multiply(R, 3, 3, w_Hat, 3, R_Deriv);
	Copy_Matrix_Partial(R, 3, 3, T_Deriv, 4, 0, 0);
	T_Deriv[3] = v[0];
	T_Deriv[7] = v[1];
	T_Deriv[11] = v[2];
	T_Deriv[12] = T_Deriv[13] = T_Deriv[14] = T_Deriv[15]=0;
	return;
}
template<typename _T>void Exp_Phi_Delta_Phi_3(_T Phi[3], _T Delta_Phi[3], _T R[3*3])
{///���� R = exp( (��+delta_��)^), ���� ��+delta_�ն�Ӧ����ת����
	_T Sum[3];
	Vector_Add(Phi, Delta_Phi, 3, Sum);
	Rotation_Vector_3_2_Matrix(Sum, R);
	return;
}

template<typename _T>void Get_Delta_R(_T Phi[3], _T Delta_Phi[3], _T Delta_R[3 * 3])
{//���� Delta_R = exp( (Jl(��)* delta_��)^)
	_T Phi_4[4];
	_T Jl[3 * 3];
	_T V[3];

	Rotation_Vector_3_2_4(Phi, Phi_4);
	Get_J_by_Rotation_Vector(Phi_4, Jl);
	Matrix_Multiply(Jl, 3, 3, Delta_Phi, 1, V);
	Rotation_Vector_3_2_Matrix(V, Delta_R);
	return;
}

void IMU_Test_Main()
{
	typedef double _T;
	IMU<_T> oIMU;
	int iResult=bRead_IMU_Line("IMU 1624426287.22854877 0.000680678408277777028 -0.000532325421858261482 0.000656243798749856877 -0.605724081666666581 0.0254972899999999988 9.80681344416666789",  &oIMU);
	//Disp_IMU(oIMU);

	{	//��һ�����飬����t0ʱ�̵���ת��������ٶ�w����t1ʱ�̵���ת����
		_T I[3 * 3],R_t1[3*3],R_Deriv[3*3];
		Gen_I_Matrix(I, 3, 3);
		Get_R_t1(I, oIMU.w, R_t1);

		Get_Derive_R_t(I, oIMU.w, R_Deriv);
		//Disp(R_Deriv, 3, 3, "R_Deriv");
	}
	
	{//�����޵����˹��ʽ
		_T V[4] = { 1,2,3 }, V_Hat[3 * 3];
		_T R[3 * 3];
		//����1��������չ��
		Hat(V, V_Hat);
		Exp_Ref(V_Hat, 3, R);
		//Disp(R, 3, 3, "R");
		
		//�޵����˹��ʽ
		Rotation_Vector_3_2_4(V, V);
		Rotation_Vector_4_2_Matrix(V, R);
		//Disp(R, 3, 3, "R");
	}

	{//��dT/dt, λ�˶�ʱ��ĵ���
		_T V[4] = { 1,2,3 }, R[3 * 3], t[3] = { 10,20,30 },T[4*4];
		Rotation_Vector_3_2_4(V, V);
		Rotation_Vector_4_2_Matrix(V, R);
		Gen_Homo_Matrix(R, t, T);
		_T w[] = { 2,3,4 }, v[] = {100,200,300}, T_Deriv[4 * 4];
		Get_Deriv_T_t(T, w, v, T_Deriv);
		//Disp(T_Deriv, 4, 4, "T_Deriv");
	}

	{//��֤BCH������BCH��ʽֻ�ǽ���
		_T RV_3[3], Delta_RV_3[3], RV1_3[3], R1[3*3];
		Get_Random_Norm_Vec(RV_3, 3);
		Get_Random_Norm_Vec(Delta_RV_3, 3);
		Vector_Multiply(RV_3, 3, 1.5, RV_3);
		Vector_Multiply(Delta_RV_3, 3, 0.1, Delta_RV_3);

		//����exp( (A+B)^)
		Vector_Add(RV_3, Delta_RV_3, 3, RV1_3);
		Rotation_Vector_3_2_Matrix(RV1_3, R1);
		Disp(R1, 3, 3, "R_1");
		Disp(RV1_3, 1, 3, "RV1_3");

		////һ����λ����
		//Exp_Phi_Delta_Phi_3(RV_3, Delta_RV_3, R1);
		//Disp(R1, 3, 3, "R_1");

		//�Ƚ�delta_R_V���� exp( (Jl(R_V)*delta_R_V)^)
		_T RV_4[4], Delta_RV_4[4];
		Rotation_Vector_3_2_4(RV_3, RV_4);
		Rotation_Vector_3_2_4(Delta_RV_3, Delta_RV_4);

		//exp( (Jl(��)*����)^)
		_T J[3 * 3],Temp[4*4];
		Get_J_by_Rotation_Vector(RV_4, J);
		Matrix_Multiply(J, 3, 3, Delta_RV_3,1, Temp);
		Rotation_Vector_3_2_Matrix(Temp, Temp);	//Delta_R
		//Disp(Temp, 3, 3, "Delta_R");
		
		Rotation_Vector_3_2_Matrix(RV_3, R1);
		Matrix_Multiply(Temp, 3, 3, R1, 3, R1);
		Disp(R1, 3, 3, "R_1");
		Rotation_Matrix_2_Vector(R1, Temp);
		Rotation_Vector_4_2_3(Temp, Temp);
		Disp(Temp, 1, 3, "RV1_3");

		////��Get_Delta_R
		//Rotation_Vector_3_2_Matrix(RV_3, R1);
		//Get_Delta_R(RV_3, Delta_RV_3, Temp);
		//Disp(Temp, 3, 3, "Delta_R");
		//Matrix_Multiply(Temp, 3, 3, R1, 3, R1);
		//Disp(R1, 3, 3, "R_1");
	}

	{//��BCH�������Ը��� T = Delta_T * T
		_T RV_3[3], Delta_RV_3[3];
		_T t[3] = { 100,200,300 }, Delta_t[3] = { 0.1,0.2,0.3 };

		Get_Random_Norm_Vec(RV_3, 3);
		Get_Random_Norm_Vec(Delta_RV_3, 3);	//�䵱ksi
		Vector_Multiply(RV_3, 3, 1.5, RV_3);
		Vector_Multiply(Delta_RV_3, 3, 0.1, Delta_RV_3);

		//����RV + Delta_RV��Ӧʲô�µ�λ�ˣ���Ȼse3��ֱ�����������������̫Զ
		_T Ksi[6], T[4 * 4];
		Vector_Add(RV_3, Delta_RV_3, 3, &Ksi[3]);
		Vector_Add(t, Delta_t, 3, Ksi);
		Gen_Homo_Matrix_1(&Ksi[3], Ksi, T);
		Disp(T, 4, 4, "T");

		//��Delta_Ksi, ��ӦDelta_T, �ټӵ�ԭ����T
		Gen_Homo_Matrix_1(RV_3, t, T);
		_T Delta_T[4 * 4], T1[4 * 4];
		Gen_Homo_Matrix_1(Delta_RV_3, Delta_t, Delta_T);
		Matrix_Multiply(Delta_T, 4, 4, T, 4, T1);
		Disp(T1, 4, 4, "T");

		//��se3_2_SE3�ķ���ת��Delta_Ksi
		memcpy(Ksi, Delta_t, 3 * sizeof(_T));
		memcpy(&Ksi[3], Delta_RV_3, 3 * sizeof(_T));
		se3_2_SE3(Ksi, Delta_T);
		Matrix_Multiply(Delta_T, 4, 4, T, 4, T1);
		Disp(T1, 4, 4, "T");
	}

	return;
}