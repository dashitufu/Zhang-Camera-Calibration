#include "Chess_Board_Detect.h"
#include "Slam.h"

void K_Test_1()
{//内参试验
	typedef double _T;
	_T K3[3], K9[3 * 3];

	{//等效35mm 内参
		//第一个试验，用等效35mm 求一个只有 fxy, cx,cy 的内参
		Get_K3_by_eq_focal<_T>(35, 4000, 1808, K3);
		Get_K9_by_eq_focal<_T>(35, 4000, 1808, K9);

		_T p1[3] = { 10, -10,100 },
			p2[3] = { 20, -20,100 };
		_T p3[3], p4[3];

		K3_Proj<_T>(K3, p1, p3);
		K3_Proj<_T>(K3, p2, p4);
		Disp(K3, 1, 3, "K3");
		Disp(p3, 1, 2, "p3");
		//Disp(p4, 1, 2, "p4");

		Matrix_Multiply(K9, 3, 3, p1, 1, p3);
		Vector_Multiply(p3, 3, 1.f / p3[2], p3);
		Disp(p3, 1, 3, "p3");
	}
		
	{
		_T uv[3], P[3] = { 1,1,10 };
		Get_K3_by_eq_focal<_T>(35, 1920, 1080, K3);
		Get_K9_by_eq_focal<_T>(35, 1920, 1080, K9);
		K3_Proj(K3, P, uv,1);
		Disp(uv, 1, 3, "uv");

		Matrix_Multiply_3x1(K9, P, uv);
		Vector_Multiply(uv, 3, 1.f / uv[2], uv);
		Disp(uv, 1, 3, "uv");

		//K 的尺度不变性，无论乘以多少scale, 投影不变
		Vector_Multiply<_T>(K9, 3*3, 100, K9);
		Matrix_Multiply_3x1(K9, P, uv);
		Vector_Multiply(uv, 3, 1.f / uv[2], uv);
		Disp(uv, 1, 3, "uv");

		//K3 没有平移不变性
		Vector_Multiply<_T>(K3, 3, 100, K3);
		K3_Proj(K3, P, uv, 1);
		Disp(uv, 1, 3, "uv");
	}
	return;
}

void Normalize_Test_1()
{//归一化试验，来个boudnign box，再构造一个内参形式的K矩阵
	typedef float _T;
	//先造平面点，没有z
	const int w = 11, h = 8,
		n = w * h;
	_T Corner_Ref[n][2], Norm_Ref[n][2];
	Gen_Corner_Ref<_T>(11, 8, 0.02f, Corner_Ref);

	_T K[3], K_Inv[3];
	Normalize_by_B_Box_2d<_T>(Corner_Ref, n, 1, Norm_Ref,K,K_Inv);
	Disp((_T*)Corner_Ref, w * h, 2, "Corner Ref");
	return;
}

void Normalize_Test_2()
{//将点用齐次坐标表示，显示齐次坐标在这个条件下如何完美等价z=0 的操作
	typedef float _T;
	//先造平面点，没有z
	const int w = 11, h = 8,
		n = w * h;
	_T Corner_Ref[n][2], Norm_Ref[n][2];
	Gen_Corner_Ref<_T>(11, 8, 0.02f, Corner_Ref);

	_T K[3], K_Inv[3];
	Normalize_by_B_Box_2d<_T>(Corner_Ref, n, 1, Norm_Ref, K, K_Inv);

	//将点改成齐次坐标，(x,y,1)
	_T Corner_Ref_3d[n][3], Norm_Ref_3d[n][3];
	for (int i = 0; i < n; i++)
	{
		Corner_Ref_3d[i][0] = Corner_Ref[i][0];
		Corner_Ref_3d[i][1] = Corner_Ref[i][1];
		Corner_Ref_3d[i][2] = 1;
		Norm_Ref_3d[i][0] = Norm_Ref[i][0];
		Norm_Ref_3d[i][1] = Norm_Ref[i][1];
		Norm_Ref_3d[i][2] = 1;
	}

	//正向验算 uv = 1/s * K * P
	for (int i = 0; i < n; i++)
	{//数据完美，经过投影后，全部成为齐次坐标
		_T uv[3];
		_T K1[9] = { K[0],0,K[1],
					0,K[0],K[2],
					0,	0,	1 };
		Matrix_Multiply(K1, 3, 3, Corner_Ref_3d[i], 1, uv);
		//K3_Proj<_T>(K, Corner_Ref_3d[i], uv);
		printf("(%f,%f)->(%f,%f,%f) %f\n", Norm_Ref[i][0], Norm_Ref[i][1],
			uv[0], uv[1], uv[2],
			fGet_Distance(Norm_Ref[i], uv, 2));
	}

	//逆向投影 p = s * K(-1)*uv
	for (int i = 0; i < n; i++)
	{//完美恢复，而且目标数据是齐次坐标
		_T p[3];
		_T K_Inv_1[9] = {	K_Inv[0],0,K_Inv[1],
							0,K_Inv[0],K_Inv[2],
							0,	0,	1 };
		Matrix_Multiply(K_Inv_1, 3, 3, Norm_Ref_3d[i], 1, p);
		printf("(%f,%f)->(%f,%f,%f) %f\n", Corner_Ref[i][0], Corner_Ref[i][1],
			p[0], p[1], p[2],
			fGet_Distance(Corner_Ref[i], p, 2));
	}
	return;
}


void Normalize_Test_3()
{//装入真实数据，头到图像上看看
	typedef float _T;
	//先造平面点，没有z
	const int w = 11, h = 8,
		n = w * h;
	_T Corner_Ref[n][2], Norm_Ref[n][2];
	Gen_Corner_Ref<_T>(11, 8, 0.02f, Corner_Ref);

	Image oRef;
	Init_Image(&oRef, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oRef);

	_T K[3];
	Normalize_by_B_Box_2d<_T>(Corner_Ref, n, 1000, Norm_Ref, K);
	//for (int i = 0; i < n; i++)
		//Draw_Arc(oRef, 10, (int)Norm_Ref[i][0], (int)Norm_Ref[i][1]);
	//bSave_Image("c:\\tmp\\1.bmp", oRef);

	//装入真实数据
	_T(*pUV)[2], Norm_uv[n][2];
	int iImage_Count = 1;
	Image oImage;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 24);
	Set_Color(oImage);
	Load_Poine_2D("c:\\tmp\\temp\\corner.bin", &iImage_Count, n, w, h, &pUV);
	Normalize_by_B_Box_2d<_T>(pUV, n, 1000, Norm_uv, K);
	
	
	{
		//用归一化平面上的数据求H 矩阵
		_T H[3 * 3], fError = 0;
		_T A[n * 2 * 9], fCond_Num;
		Gen_H_Coeff_z_0(Norm_Ref, Norm_uv, n, A);
		fCond_Num = fGet_Cond_Num(A, n);
		Estimate_H_Ref(Norm_Ref, Norm_uv, n, H, 0);
		printf("条件数：%f 拍脑袋Bouding box 归一化，误差：%f\n", fCond_Num,Test_H_2d<_T>(Norm_Ref, Norm_uv, n, H));

		//原来数据做估计
		Estimate_H_Ref(Corner_Ref, pUV, n, H);
		printf("原数据交由opencv自行归一化，%f\n", Test_H_2d<_T>(Corner_Ref, pUV, n, H));

		////无归一化原数据
		Gen_H_Coeff_z_0(Corner_Ref, pUV, n, A);
		fCond_Num = fGet_Cond_Num(A, n);
		Estimate_H_Ref(Corner_Ref, pUV, n, H,0);
		printf("条件数:%f 无归一化，%f\n", fCond_Num, Test_H_2d<_T>(Corner_Ref, pUV, n, H));
	}

	{//看看不同尺寸的归一化能否改善条件数
		Normalize_by_B_Box_2d<_T>(Corner_Ref, n, 1, Norm_Ref, K);
		Normalize_by_B_Box_2d<_T>(pUV, n, 1, Norm_uv, K);
		_T A[n * 2 * 9], fCond_Num;
		Gen_H_Coeff_z_0(Norm_Ref, Norm_uv, n, A);
		fCond_Num = fGet_Cond_Num(A, n);

		_T H[3 * 3];
		Estimate_H_Ref(Norm_Ref, Norm_uv, n, H, 0);
		printf("条件数: %f Error:%f\n", fCond_Num, Test_H_2d<_T>(Norm_Ref, Norm_uv, n, H));
	}

	{//试一下Hartley 方法
		Normalize_Hartley_2d<_T>(Corner_Ref, n, Norm_Ref);
		Normalize_Hartley_2d<_T>(pUV, n, Norm_uv);
		_T A[n * 2 * 9], fCond_Num;
		Gen_H_Coeff_z_0(Norm_Ref, Norm_uv, n, A);
		fCond_Num = fGet_Cond_Num(A, n);

		_T H[3 * 3];
		Estimate_H_Ref(Norm_Ref, Norm_uv, n, H, 0);
		printf("条件数: %f Error:%f\n", fCond_Num, Test_H_2d<_T>(Norm_Ref, Norm_uv, n, H));
	}
	
	Free_Image(&oRef);
	Free_Image(&oImage);
	Free(pUV);
	return;
}

int main()
{
	bInit_Env_CPU(15000000);
	//K_Test_1();
	//Normalize_Test_3();
	Chess_Board_Detect_Main();
		
#ifdef WIN32
	_CrtDumpMemoryLeaks();
#endif
	return 0;
}