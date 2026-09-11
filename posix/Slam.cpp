#include "Image.h"
#include "Slam.h"
template void K9_2_K4(float K9[9], float K4[4]);
template void K9_2_K4(double K9[9], double K4[4]);
template<typename _T>void K9_2_K4(_T K9[9],_T K4[4])
{
	K4[0] = K9[0];
	K4[1] = K9[4];
	K4[2] = K9[2];
	K4[3] = K9[5];
}
template void K4_2_K9(float K4[9], float K9[4]);
template void K4_2_K9(double K4[9], double K9[4]);
template<typename _T>void K4_2_K9(_T K4[9], _T K9[4])
{
	memset(K9, 0, 9 * sizeof(_T));
	K9[0] = K4[0];
	K9[4] = K4[1];
	K9[2] = K4[2];
	K9[5] = K4[3];
	K9[8] = 1;
}
template void Get_K9_by_eq_focal(float eq_f, float w, float h, float K[3 * 3]);
template void Get_K9_by_eq_focal(double eq_f, double w, double h, double K[3 * 3]);
template<typename _T>void Get_K9_by_eq_focal(_T eq_f, _T w, _T h, _T K[3 * 3])
{//通过等效焦距求相机内参矩阵K
	//eq_f： equivalent focal length 等效焦距，单位mm，对应长宽36mm * 24mm 成像平面
	//w, h 相机图片的长宽
	//例：Get_K<_T>(35, 4000, 2256, K);	35mm等效小孔成像焦距，4000x2256希昂宿平面
	memset(K, 0, 9 * sizeof(_T));
	K[0] = K[4] = (_T)w * eq_f / 36;
	K[2] = (_T)w / 2;
	K[5] = (_T)h / 2;

	//此处就是变为(u,v,1)齐次坐标的分别所在
	//_T z = w;
	//K[8] = (_T)z * 35 / 36;	//非齐次
	K[8] = 1;				//(u,v)必齐次
	return;
}

//简化版
template void Get_K4_by_eq_focal(double eq_f, double w, double h, double K[4]);
template<typename _T>void Get_K4_by_eq_focal(_T eq_f, _T w, _T h, _T K[4])
{//通过等效焦距求相机内参矩阵K
	//eq_f： equivalent focal length 等效焦距，单位mm，对应长宽36mm * 24mm 成像平面
	//w, h 相机图片的长宽
	//例：Get_K<_T>(35, 4000, 2256, K);	35mm等效小孔成像焦距，4000x2256希昂宿平面
	K[0] = K[1] = (_T)w * eq_f / 36;
	K[2] = (_T)w / 2;
	K[3] = (_T)h / 2;
	return;
}

template void Get_K3_by_eq_focal(double eq_f, double w, double h, double K[3]);
template<typename _T>void Get_K3_by_eq_focal(_T eq_f, _T w, _T h, _T K[3])
{//通过等效焦距求相机内参矩阵K
	//eq_f： equivalent focal length 等效焦距，单位mm，对应长宽36mm * 24mm 成像平面
	//w, h 相机图片的长宽
	//例：Get_K<_T>(35, 4000, 2256, K);	35mm等效小孔成像焦距，4000x2256希昂宿平面
	K[0] = (_T)w * eq_f / 36;
	K[1] = (_T)w / 2;
	K[2] = (_T)h / 2;
	return;
}

template void K3_Proj(float K[3], float P[3], float uv[2], int bHas_z);
template void K3_Proj(double K[3], double P[3], double uv[2], int bHas_z);
template<typename _T>void K3_Proj(_T K[3], _T P[3], _T uv[],int bHas_z)
{//求空间一点P经过相机内参K投影到成像平面的值
//注意：K3 做不到尺度不变性，只是用来简化计算，一乘scale 就打乱
	if (bHas_z)
	{//注意，本来K 是个正经的三维投影到二维的矩阵
		uv[0] = (K[0] * P[0]) / P[2] + K[1];
		uv[1] = (K[0] * P[1]) / P[2] + K[2];
		//= (0 * px + 0 * py + 1*pz)/(0 * px + 0 * py + 1*pz) =1;
		uv[2] = 1;
	}else
	{//干二维的缩放平移只是副业，此时假想P为齐次坐标(x,y,1)
		uv[0] = K[0] * P[0] + K[1];
		uv[1] = K[0] * P[1] + K[2];
	}
	return;
}

template void K3_Inv(float K[3], float K_Inv[3]);
template void K3_Inv(double K[3], double K_Inv[3]);
template<typename _T>void K3_Inv(_T K[3], _T K_Inv[3])
{//求K3投影矩阵的逆
//将 K的最简形式拆开，u = (f * px)/pz + cx
//	px = pz * (u - cx) / f = pz * (u * 1 / f - cx / f)
//	py = pz * (v - cy) / f = pz * (v * 1 / f - cy / f)
//	= >
//	px = pz *	1/f		0		-cx/f	*	u
//	py			0		1/f		-cy/f		v
//	1			0		0		1			1
	K_Inv[0] = 1.f / K[0];
	K_Inv[1] = -K[1] / K[0];
	K_Inv[2] = -K[2] / K[0];
	return;
}

template<typename _T>void K4_Inv(_T K[4], _T K_Inv[4])
{//求K3投影矩阵的逆
//将 K的最简形式拆开，	u = (fx * px)/pz + cx
//						v = (fy * py)/pz + cy
//	px = pz * (u - cx) / fx = pz * (u * 1 / f - cx / f)
//	py = pz * (v - cy) / fy = pz * (v * 1 / f - cy / f)
//	= >
//	px = pz *	1/fx	0		-cx/fx *	u
//	py			0		1/fx	-cy/fy		v
//	1			0		0		1			1
	K_Inv[0] = 1.f / K[0];
	K_Inv[1] = 1.f / K[1];
	K_Inv[2] = -K[2] / K[0];
	K_Inv[3] = -K[3] / K[1];
}

template void Normalize_by_B_Box_2d(float P[][2], int n, float fBox_Size, float Norm[][2], float K[3], float K_Inv[]);
template void Normalize_by_B_Box_2d(double P[][2], int n, double fBox_Size, double Norm[][2], double K[3], double K_Inv[]);
template<typename _T>void Normalize_by_B_Box_2d(_T P[][2], int n,
	_T fBox_Size, _T Norm[][2], _T K[3], _T K_Inv[])
{//给所有点求个Bouding Box，将中心对准中心，相当于投影到一个平面上
//实践证明，这个方法虽然能降低条件数，但是远比Hartlay 方法差，本身
//bounding box 就是把最差的点算进来，这些烂点扭曲了数据的分布形态
	_T B_Box[2][2];
	Get_Bounding_Box(P, n, B_Box);

	_T Box_Size[2] = { B_Box[1][0] - B_Box[0][0], B_Box[1][1] - B_Box[0][1] };
	_T fMax = Max(Box_Size[0], Box_Size[1]);
	_T Center[2] = { B_Box[0][0] + Box_Size[0] / 2,
					B_Box[0][1] + Box_Size[1] / 2 };

	_T fScale = fBox_Size / fMax;
	// org * s + delta = new => delta = new - org*s 
	_T Offset[2] = { fBox_Size / 2 - Center[0] * fScale, fBox_Size / 2 - Center[1] * fScale };
	_T K1[3] = { fScale,Offset[0],Offset[1] };

	for (int i = 0; i < n; i++)
		K3_Proj<_T>(K1, P[i], Norm[i], 0);

	if (K)
		memcpy(K, K1, 3 * sizeof(_T));
	if (K_Inv)
		K3_Inv(K1, K_Inv);
	return;
}

template void Normalize_Hartley_2d(float P[][2], int n, float Norm[][2], float K[3], float K_Inv[]);
template void Normalize_Hartley_2d(double P[][2], int n, double Norm[][2], double K[3], double K_Inv[]);
template<typename _T>void Normalize_Hartley_2d(_T P[][2], int n,
	_T Norm[][2], _T K[3], _T K_Inv[])
{//用Hartley 归一化方法
	const _T sqrt_2 = (_T)sqrt(2);

	_T E[2];
	Get_E_2d(P, n, E);

	_T fDev = 0;
	for (int i = 0; i < n; i++)
		fDev += (_T)sqrt(fGet_Distance(P[i], E, 2));
	fDev /= n;

	_T K1[3] = { sqrt_2 / fDev,-K1[0] * E[0], -K1[0] * E[1] };

	for (int i = 0; i < n; i++)
		K3_Proj(K1, P[i], Norm[i], 0);

	if (K)
		memcpy(K, K1, 3 * sizeof(_T));

	if (K_Inv)
		K3_Inv(K1, K_Inv);
	return;
}
template void Normalize_Dev_2d(float P[][2], int n, float Norm[][2], float K[4], float K_Inv[4]);
template void Normalize_Dev_2d(double P[][2], int n, double Norm[][2], double K[4], double K_Inv[4]);
template<typename _T>void Normalize_Dev_2d(_T P[][2], int n, _T Norm[][2], _T K[4], _T K_Inv[4])
{//用偏离绝对值归一化，此乃安全函数，源可以等于目的
	_T E[2];
	Get_E_2d(P, n, E);

	_T Dev[2] = { 0 };
	Get_Dev_2d<_T>(P, n, E, Dev);

	_T s[2] = { 1.f / Dev[0],  1.f / Dev[1] };
	for (int i = 0; i < n; i++)
	{
		Norm[i][0] = s[0] * (P[i][0] - E[0]);
		Norm[i][1] = s[1] * (P[i][1] - E[1]);
	}

	if(K || K_Inv)
	{
		_T K1[4] = { s[0],s[1], -E[0] * s[0], -E[1] * s[1] };
		if(K)
			memcpy(K, K1, 4 * sizeof(_T));
		if (K_Inv)
			K4_Inv(K1, K_Inv);
	}
	return;
}

template<typename _T>static void Normalize_Zhang(_T Point_2D[][2], int iPoint_Count,
	_T Norm_Point[][2], _T Scale[2], _T Offset[2])
{//对一组点归一化，这个和Colmap又不一样，不求Max,Min
	//毫无营养
	_T Mean[2] = { 0 };
	int i;
	//对该点集中求中心
	for (i = 0; i < iPoint_Count; i++)
	{
		//printf("%f %f\n", Point_2D[i][0], Point_2D[i][1]);
		Mean[0] += Point_2D[i][0], Mean[1] += Point_2D[i][1];
	}

	Mean[0] /= iPoint_Count;
	Mean[1] /= iPoint_Count;

	//注意，这不是标准差，叫1：平均绝对偏差；2，L1 Loss；3，Mean Absolute Erro
	_T Dev[2] = {};	//再求个偏离度
	for (i = 0; i < iPoint_Count; i++)
		Dev[0] += abs(Point_2D[i][0] - Mean[0]), Dev[1] += abs(Point_2D[i][1] - Mean[1]);
	Dev[0] /= iPoint_Count;
	Dev[1] /= iPoint_Count;

	//将绝对偏差的导数作为scale，对每个样本求一个偏离度，再乘以这个scale
	_T s[2] = { 1.f / Dev[0],  1.f / Dev[1] };
	for (i = 0; i < iPoint_Count; i++)
	{
		Norm_Point[i][0] = s[0] * (Point_2D[i][0] - Mean[0]);
		Norm_Point[i][1] = s[1] * (Point_2D[i][1] - Mean[1]);
	}
	Scale[0] = s[0], Scale[1] = s[1];
	Offset[0] = -Mean[0] * s[0], Offset[1] = -Mean[1] * s[1];
	return;
}

template void Gen_H_Coeff_z_0(float P[][2], float uv[][2], int n, float A[]);
template void Gen_H_Coeff_z_0(double P[][2], double uv[][2], int n, double A[]);
template<typename _T>void Gen_H_Coeff_z_0(_T P[][2], _T uv[][2], int n, _T A[])
{//很简单，就是给 z=0 的平面点集构造系数矩阵A
	_T* A1 = A, * P_Cur = P[0], * uv_Cur = uv[0];
	for (int i = 0; i < n; i++)
	{
		_T px = P_Cur[0], py = P_Cur[1], u = uv_Cur[0], v = uv_Cur[1];
		//x	y	1	0	0	0	-ux	-uy	-u
		A1[0] = px, A1[1] = py, A1[2] = 1;
		A1[3] = A1[4] = A1[5] = 0;
		A1[6] = -px * u, A1[7] = -py * u, A1[8] = -u;
		A1 += 9;

		//0	0	0	x	y	1	-vx	-vy	-v
		A1[0] = A1[1] = A1[2] = 0;
		A1[3] = px, A1[4] = py, A1[5] = 1;
		A1[6] = -px * v, A1[7] = -py * v, A1[8] = -v;
		A1 += 9;
		P_Cur += 2, uv_Cur += 2;
	}
}
template int Estimate_H_Zhang(float Norm_P[][2], float uv[][2], int n, float H[3 * 3],	float K_Norm[], int bNormalize, Normalize_Method iMethod);
template int Estimate_H_Zhang(double Norm_P[][2], double uv[][2], int n, double H[3 * 3],	double K_Norm[], int bNormalize, Normalize_Method iMethod);
template<typename _T>int Estimate_H_Zhang(_T Norm_P[][2], _T uv[][2], int n, _T H[3 * 3], 
	_T K_Norm[], int bNormalize, Normalize_Method iMethod)
{//尽可能快
	//**********先分配**************************************/
	int iSize = ALIGN_SIZE_8(n * 2 * 9 * sizeof(_T)) +
		ALIGN_SIZE_8(n * 2 * sizeof(_T)) * 2, bRet = 0;
	_T* A = NULL, (*pNorm_uv)[2];

	unsigned char* p = (unsigned char*)pMalloc(iSize);
	if (!p)
		goto END;
	A = (_T*)p, p += ALIGN_SIZE_8(n * 2 * 9 * sizeof(_T));
	pNorm_uv = (_T(*)[2])p;
	//**********先分配**************************************/

	_T K_uv_Inv[4];
	if (bNormalize)
	{
		Normalize_2d<_T>(uv, n, pNorm_uv, iMethod, NULL, K_uv_Inv);
		Gen_H_Coeff_z_0(Norm_P, pNorm_uv, n, A);
	}else
		Gen_H_Coeff_z_0(Norm_P, uv, n, A);
	//Disp((_T*)pNorm_uv, n, 2, "Norm");

	//******************接着解齐次矛盾方程组 Ax =0*****************/
	_T AtA[9 * 9];
	Transpose_Multiply(A, n * 2, 9, AtA, 0);
	int iResult;
	iResult = bInverse_Power<_T>(AtA, 9, (_T*)NULL, H, (_T)1e-8);
	if (!iResult)
		Solve_Homo_Linear_SVD(A, n * 2, 9, H, &iResult);

	if (!iResult)
		goto END;
	//******************接着解齐次矛盾方程组 Ax =0*****************/

	if (bNormalize)
	{//此时需要恢复Scale
		if (iMethod == Normalize_Method::Dev)
		{
			//Normalize by Dev
			_T K_uv_Inv_1[3 * 3] = { K_uv_Inv[0],0,K_uv_Inv[2],
								0,K_uv_Inv[1],K_uv_Inv[3],
								0,0,1 };
			_T K_P_1[3 * 3] = { K_Norm[0],0,K_Norm[2],
								0,K_Norm[1],K_Norm[3],
								0,0,1 };
			Matrix_Multiply_3x3<_T>(K_uv_Inv_1, H, H);
			Matrix_Multiply_3x3<_T>(H, K_P_1, H);
		}else if (iMethod == Normalize_Method::Hartley ||
			iMethod == Normalize_Method::Bounding_Box)
		{
			//Hartley 方法
			_T K_uv_Inv_1[3 * 3] = { K_uv_Inv[0],0,K_uv_Inv[1],
								0,K_uv_Inv[0],K_uv_Inv[2],
								0,0,1 };
			_T K_P_1[3 * 3] = { K_Norm[0],0,K_Norm[1],
								0,K_Norm[0],K_Norm[2],
								0,0,1 };
			Matrix_Multiply_3x3<_T>(K_uv_Inv_1, H, H);
			Matrix_Multiply_3x3<_T>(H, K_P_1, H);
		}else
			printf("Not implemented\n");
	}
	//Disp(H, 3, 3, "H");
	//Normalize(H, 9, H);
	bRet = 1;
END:
	Free(A);
	return bRet;
}

template void Normalize_2d(float P[][2], int n, float Norm[][2], Normalize_Method iMethod, float K[4], float K_Inv[4]);
template void Normalize_2d(double P[][2], int n, double Norm[][2], Normalize_Method iMethod, double K[4], double K_Inv[4]);
template<typename _T>void Normalize_2d(_T P[][2], int n, _T Norm[][2],
	Normalize_Method iMethod, _T K[4], _T K_Inv[4])
{
	switch (iMethod)
	{
	case Dev:
		Normalize_Dev_2d<_T>(P, n, Norm, K, K_Inv);
		break;
	case Hartley:
		Normalize_Hartley_2d(P, n, Norm, K, K_Inv);
		break;
	case Bounding_Box:
		Normalize_by_B_Box_2d<_T>(P, n, 1, Norm, K, K_Inv);
		break;
	case None:
		memcpy(Norm, P, n * 2 * sizeof(_T));
		break;
	default:
		printf("Not implemented in Normalize_2d\n");
	}
	return;
}

template int Estimate_H_Ref(float P[][2], float uv[][2], int n, float H[3 * 3], int bNormalize, int bUse_SVD);
template int Estimate_H_Ref(double P[][2], double uv[][2], int n, double H[3 * 3], int bNormalize, int bUse_SVD);
template<typename _T>int Estimate_H_Ref(_T P[][2], _T uv[][2], int n, _T H[3 * 3], int bNormalize, int bUse_SVD)
{//估计一个H矩阵，满足(u,v,1)' = 1/z * HP
//这是有限制的H矩阵估计，要求参考点Point_Ref 同一定死在z=0 平面上
	const Normalize_Method Norm_Method = Normalize_Method::Dev;

	//**********先分配**************************************/
	int iSize = ALIGN_SIZE_8(n * 2 * 9 * sizeof(_T)) +
		ALIGN_SIZE_8(n * 2 * sizeof(_T)) * 2, bRet = 0;
	_T* A = NULL, (*pNorm_P)[2], (*pNorm_uv)[2];
	unsigned char* p = (unsigned char*)pMalloc(iSize);
	if (!p)
		goto END;

	A = (_T*)p, p += ALIGN_SIZE_8(n * 2 * 9 * sizeof(_T));
	pNorm_P = (_T(*)[2])p;	p += ALIGN_SIZE_8(n * 2 * sizeof(_T));
	pNorm_uv = (_T(*)[2])p;
	//**********先分配**************************************/

	_T K_P[4], K_uv_Inv[4];
	if (bNormalize)
	{
		if(Norm_Method == Normalize_Method::Dev)
		{
			Normalize_Dev_2d<_T>(P, n, pNorm_P, K_P);
			Normalize_Dev_2d<_T>(uv, n, pNorm_uv, NULL, K_uv_Inv);
		}else if(Norm_Method == Normalize_Method::Hartley)
		{
			Normalize_Hartley_2d<_T>(P, n, pNorm_P, K_P);
			Normalize_Hartley_2d<_T>(uv, n, pNorm_uv, NULL, K_uv_Inv);
		}else if(Norm_Method == Normalize_Method::Bounding_Box)
		{
			Normalize_by_B_Box_2d<_T>(P, n,1, pNorm_P, K_P);
			Normalize_by_B_Box_2d<_T>(uv, n, 1,pNorm_uv, NULL, K_uv_Inv);
		}
		Gen_H_Coeff_z_0(pNorm_P, pNorm_uv, n, A);
	}else
		Gen_H_Coeff_z_0(P, uv, n, A);

	//******************接着解齐次矛盾方程组 Ax =0*****************/
	int iResult;
	if (bUse_SVD)//用SVD方法算
		Solve_Homo_Linear_SVD(A, n*2, 9, H, &iResult);
	else
	{
		_T AtA[9 * 9];
		Transpose_Multiply(A, n * 2, 9, AtA, 0);
		iResult = bInverse_Power<_T>(AtA, 9, (_T*)NULL, H, (_T)1e-8);
	}
	if (!iResult)
		goto END;
	//******************接着解齐次矛盾方程组 Ax =0*****************/
	
	if (bNormalize)
	{//此时需要恢复Scale
		if(Norm_Method==Normalize_Method::Dev)
		{
			//Normalize by Dev
			_T K_uv_Inv_1[3 * 3] = { K_uv_Inv[0],0,K_uv_Inv[2],
								0,K_uv_Inv[1],K_uv_Inv[3],
								0,0,1 };
			_T K_P_1[3 * 3] = { K_P[0],0,K_P[2],
								0,K_P[1],K_P[3],
								0,0,1 };
			Matrix_Multiply_3x3<_T>(K_uv_Inv_1, H, H);
			Matrix_Multiply_3x3<_T>(H, K_P_1, H);
		}else if(Norm_Method == Normalize_Method::Hartley)
		{
			//Hartley 方法
			_T K_uv_Inv_1[3 * 3] = { K_uv_Inv[0],0,K_uv_Inv[1],
								0,K_uv_Inv[0],K_uv_Inv[2],
								0,0,1 };
			_T K_P_1[3 * 3] = { K_P[0],0,K_P[1],
								0,K_P[0],K_P[2],
								0,0,1 };
			Matrix_Multiply_3x3<_T>(K_uv_Inv_1, H, H);
			Matrix_Multiply_3x3<_T>(H, K_P_1, H);
		}else if (Norm_Method == Normalize_Method::Bounding_Box)
		{
			//Bounding Box 方法
			_T K_uv_Inv_1[3 * 3] = { K_uv_Inv[0],0,K_uv_Inv[1],
								0,K_uv_Inv[0],K_uv_Inv[2],
								0,0,1 };
			_T K_P_1[3 * 3] = { K_P[0],0,K_P[1],
								0,K_P[0],K_P[2],
								0,0,1 };
			Matrix_Multiply_3x3<_T>(K_uv_Inv_1, H, H);
			Matrix_Multiply_3x3<_T>(H, K_P_1, H);
		}
	}

	bRet = 1;
END:
	Free(A);
	return bRet;
}
template<typename _T>_T Test_H(_T P[][3], _T uv[][2], int n, _T H[3 * 3])
{
	_T fError = 0;
	for (int i = 0; i < n; i++)
	{
		_T uv_1[3];
		Matrix_Multiply(H, 3, 3, P[i], 1, uv_1);
		uv_1[0] /= uv_1[2], uv_1[1] /= uv_1[2];
		fError += (uv_1[0] - uv[i][0]) * (uv_1[0] - uv[i][0]) +
			(uv_1[1] - uv[i][1]) * (uv_1[1] - uv[i][1]);
	}
	fError = (_T)sqrt(fError) / n;
	return fError;
}

template float Test_H_2d(float P[][2], float uv[][2], int n, float H[3 * 3]);
template double Test_H_2d(double P[][2], double uv[][2], int n, double H[3 * 3]);
template<typename _T>_T Test_H_2d(_T P[][2], _T uv[][2], int n, _T H[3 * 3])
{//测试 uv = 1/s * H * P 的误差
	_T(*pP1)[3] = (_T(*)[3])pMalloc(n * 3 * sizeof(_T));
	for (int i = 0; i < n; i++)
		pP1[i][0] = P[i][0], pP1[i][1] = P[i][1], pP1[i][2] = 1;
	_T fError = Test_H(pP1, uv, n, H);
	Free(pP1);
	return fError;
	//return 0;
}

//**********************一组旋转转换************************/
template<typename _T>void Rotation_Vector_3_2_Matrix(_T V[3], _T R[3 * 3])
{
	_T V1[4];
	Rotation_Vector_3_2_4(V, V1);
	Rotation_Vector_4_2_Matrix(V1, R);
}

#define Scale_Matrix_1(A1,fMax) \
{ \
	A1[0][0] *= fMax; \
	A1[0][1] *= fMax; \
	A1[0][2] *= fMax; \
	A1[1][0] *= fMax; \
	A1[1][1] *= fMax; \
	A1[1][2] *= fMax; \
	A1[2][0] *= fMax; \
	A1[2][1] *= fMax; \
	A1[2][2] *= fMax; \
}

template void Rotation_Vector_4_2_Matrix(float V[4], float R[3 * 3]);
template void Rotation_Vector_4_2_Matrix(double V[4], double R[3 * 3]);
template<typename _T>void Rotation_Vector_4_2_Matrix(_T V[4], _T R[3 * 3])
{//旋转向量到旋转矩阵，试一把看看准不准, 3阶已经一摸一样
//向量的前三个分量为标准化旋转轴，最后一个分量为旋转角
//严格右手系，xyz，x向左，y向远,z向上
	//第一条公式，R = exp(V^)
	//第二条公式，罗德里格斯公式，避开算无限项，此处用第二种
	_T fCos_Theta, fSin_Theta;
	_T fTheta = V[3];
	_T V_1[3];
	//fTheta需要来个求模？
	fCos_Theta = (_T)cos(fTheta);
	fSin_Theta = (_T)sin(fTheta);
	_T nnt[3][3], I[3][3] = { {1,0,0},{0,1,0},{0,0,1} };
	_T Skew_Sym[3][3];

	//保险起见，规格化一下
	Normalize(V, 3, V_1);

	Matrix_Multiply(V_1, 3, 1, V_1, 3, (_T*)nnt);
	Scale_Matrix_1(I, fCos_Theta);
	Scale_Matrix_1(nnt, (1 - fCos_Theta));
	Hat(V_1, (_T*)Skew_Sym);
	Scale_Matrix_1(Skew_Sym, fSin_Theta);

	Matrix_Add((_T*)I, (_T*)nnt, 3, (_T*)R);
	Matrix_Add((_T*)R, (_T*)Skew_Sym, 3, (_T*)R);
	return;
}
template<typename _T>void Rotation_Vector_4_2_3(_T V[], _T V1[])
{//将4维旋转向量转换维3维旋转向量

	for (int i = 0; i < 3; i++)
		V1[i] = V[i] * V[3];
}
template<typename _T>void Rotation_Vector_3_2_4(_T V[], _T V1[])
{//将3维旋转向量转化为4维旋转向量
	_T V2[4];
	Normalize(V, 3, V2);
	V2[3] = fGet_Mod(V, 3);
	memcpy(V1, V2, 4 * sizeof(_T));
}

template<typename _T>void Rotation_Matrix_2_Vector(_T R[3 * 3], _T V[4])
{//从旋转矩阵到旋转向量就是解 Rn=n，其中n就是待求的转轴，显然特征值为1， 求解特征方程
//这个函数有问题
	//由于特征值=1， 代入(A-rI)x=0, 求得x便是特征向量。而r=1,所以解(A-I)x=0即可
	//_T I[3][3] = { 1,0,0,0,1,0,0,0,1 };
	printf("tended to be obsolete\n");

	_T R_1[3][3];// B[3] = { 0 }, V_1[3 * 3]
	_T B[3] = { 0 };
	int iResult;
	if (R == V)
	{
		printf("R connot be V\n");
		return;
	}

	memcpy(R_1, R, 9 * sizeof(_T));
	R_1[0][0] -= 1.f, R_1[1][1] -= 1.f, R_1[2][2] -= 1.f;

	//所以应该用svd分解
	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3((_T*)R_1, oSVD, &iResult);

	//Vt的最后一行就是解
	memcpy(V, &((_T*)oSVD.Vt)[6], 3 * sizeof(_T));
	Free_SVD(&oSVD);

	//再求旋转角度，感觉来个负数才行，待考，具体还得验证
	_T fTr = fGet_Tr(R, 3);
	const _T eps = (_T)1e-5;

	_T fTemp = (fTr - 1.f) / 2.f;
	fTemp = Clip3(-1.f, 1.f, fTemp);
	fTemp = -acos(fTemp);
	V[3] = fTemp;

	//还有个问题尚未弄利索，旋转向量的正负号问题。这是因为特征向量可正可符，因为
	//特征向量乘以任意常数依旧是原矩阵的特征向量。故此这个向量的正负号是否影响后续
	//的求解，尚待深化
	return;
}
template<typename _T>void Rotation_Vector_2_Quaternion(_T V[4], _T Q[4])
{//旋转向量转换为四元组，本来按照定义，一个旋转向量由一个标准化向量作为旋转轴与一个旋转角度构成
	_T fSin_Theta_Div_2;
	_T V_1[3];
	_T fTheta = V[3];
	Normalize(V, 3, V_1);
	Q[0] = cos(fTheta / 2.f);
	fSin_Theta_Div_2 = sin(fTheta / 2.f);
	Q[1] = V_1[0] * fSin_Theta_Div_2;
	Q[2] = V_1[1] * fSin_Theta_Div_2;
	Q[3] = V_1[2] * fSin_Theta_Div_2;
	return;
}
template<typename _T>void Rotation_Matrix_2_Quaternion(_T R[], _T Q[])
{//旋转矩阵到四元组。此处不完全实现，由于缺乏直接算法，故此靠一个旋转向量作为中间商倒腾过去
	_T V[4];
	Rotation_Matrix_2_Vector(R, V);
	Rotation_Vector_2_Quaternion(V, Q);
	return;
}
void Quaternion_Add(float Q_1[], float Q_2[], float Q_3[])
{
	for (int i = 0; i < 4; i++)
		Q_3[i] = Q_1[i] + Q_2[i];
	return;
}
void Quaternion_Minus(float Q_1[], float Q_2[], float Q_3[])
{
	for (int i = 0; i < 4; i++)
		Q_3[i] = Q_1[i] - Q_2[i];
	return;
}
void Quaternion_Conj(float Q_1[], float Q_2[])
{//简单求个共轭
	Q_2[0] = Q_1[0];
	Q_2[1] = -Q_1[1];
	Q_2[2] = -Q_1[2];
	Q_2[3] = -Q_1[3];
}
void Quaternion_Multiply(float Q_1[], float Q_2[], float Q_3[])
{//乘法既不是点积也不是外积，有其定义
	Q_3[0] = Q_1[0] * Q_2[0] - Q_1[1] * Q_2[1] - Q_1[2] * Q_2[2] - Q_1[3] * Q_2[3];
	Q_3[1] = Q_1[0] * Q_2[1] + Q_1[1] * Q_2[0] + Q_1[2] * Q_2[3] - Q_1[3] * Q_2[2];
	Q_3[2] = Q_1[0] * Q_2[2] - Q_1[1] * Q_2[3] + Q_1[2] * Q_2[0] + Q_1[3] * Q_2[1];
	Q_3[3] = Q_1[0] * Q_2[3] + Q_1[1] * Q_2[2] - Q_1[2] * Q_2[1] + Q_1[3] * Q_2[0];
	return;
}
void Quaternion_Inv(float Q_1[], float Q_2[])
{//对四元数求逆
	float fMod = fGet_Mod(Q_1, 4);
	int i;
	Quaternion_Conj(Q_1, Q_2);
	fMod *= fMod;
	for (i = 0; i < 4; i++)
		Q_2[i] /= fMod;
	return;
}
template<typename _T>void Quaternion_2_Rotation_Matrix(_T Q[4], _T R[])
{//四元数转换为旋转矩阵， R= vv' + s^2*I + 2sv^ + (v^)^2
	_T fValue, M_2[3][3], M_1[3][3] = { {1,0,0},{0,1,0},{0,0,1} };	//临时矩阵	
	//先算个vv' 直接放R即可
	Matrix_Multiply(&Q[1], 3, 1, &Q[1], 3, R);

	//再算 s^2*I
	fValue = Q[0] * Q[0];
	Scale_Matrix_1(M_1, fValue);
	Matrix_Add(R, (_T*)M_1, 3, R);

	//再算2sv
	Hat(&Q[1], (_T*)M_1);
	fValue = 2.f * Q[0];
	memcpy(M_2, M_1, 3 * 3 * sizeof(_T));
	Scale_Matrix_1(M_2, fValue);
	Matrix_Add(R, (_T*)M_2, 3, R);

	//再算(v^) ^ 2，上面已经搞定了M_1= v^
	Matrix_Multiply((_T*)M_1, 3, 3, (_T*)M_1, 3, (_T*)M_2);
	Matrix_Add(R, (_T*)M_2, 3, R);

	//Disp((float*)R, 3, 3);
	return;
}
template<typename _T>void Quaternion_2_Rotation_Vector(_T Q[4], _T V[4])
{//由四元数转换为旋转向量,注意了，四元数必须是标准化，即|v|=1，否则旋转角度就不对
	_T fSin_Theta_Div_2;
	V[3] = 2.f * acos(Q[0]);
	fSin_Theta_Div_2 = sin(V[3] / 2.f);
	V[0] = Q[1] / fSin_Theta_Div_2;
	V[1] = Q[2] / fSin_Theta_Div_2;
	V[2] = Q[3] / fSin_Theta_Div_2;
	return;
}
//**********************一组旋转转换************************/

//************************一组投影函数************************/
template void Get_Distort_Coeff(float x, float y, float D[5], float dc[2 * 5]);
template void Get_Distort_Coeff(double x, double y, double D[5], double dc[2 * 5]);
template<typename _T>void Get_Distort_Coeff(_T x, _T y, _T D[5], _T dc[2 * 5])
{//得畸变参数系数矩阵 2x5
	//畸变系数矩阵, Distort Coefficients
	//x1 * r² 	x1 * r^4		x1 * r^6		2*x1 * y1		(r² + 2x1²) 
	//y1 * r²	y1 * r^4 		y1 * r^6		(r² + 2y²) 		2 * x1 * y1
	//Pd = Md * D		2x5 * 5x1 => 2x1 
	_T r2 = x * x + y * y, r4 = r2 * r2, r6 = r2 * r4;
	_T dc_1[] = { x * r2, x * r4,	x * r6,	2 * x * y,	r2 + 2 * x * x,
		y * r2,	y * r4, y * r6,	r2 + 2 * y * y, 2 * x * y };
	memcpy(dc, dc_1, 2 * 5 * sizeof(_T));
	return;
}

template<typename _T>void Get_Distort_Value(_T x, _T y, _T D[5], _T d[2])
{//对于归一化平面上得坐标(x,y) 及给定得畸变参数，求畸变具体数值
	_T dc[2 * 5];
	Get_Distort_Coeff(x, y, D, dc);
	Matrix_Multiply(dc, 2, 5, D, 1, d);
	return;
}

template void Get_dTP_dKsi(float Pt[3], float Deriv[4 * 6]);
template void Get_dTP_dKsi(double Pt[3], double Deriv[4 * 6]);
template<typename _T>void Get_dTP_dKsi(_T Pt[3], _T Deriv[4 * 6])
{//对于Pt = TP, 求T 上的扰动对Pt的影响
// = dTP/dKsi = dPt/dKsi
	_T P1_M[3 * 3];

	//dTP/dksi = dP'/dksi= I -P'^
	Hat(Pt, P1_M);

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

template void Get_dTP_dKsi(float T[3 * 4], float P[2], float Deriv[4 * 6]);
template void Get_dTP_dKsi(double T[3 * 4], double P[2], double Deriv[4 * 6]);
template<typename _T>void Get_dTP_dKsi(_T T[3*4], _T P[2], _T Deriv[4 * 6])
{//这个是符合函数，先求P' = TP, 再求 dTP/dKsi 然后求导
	_T P1[4] = { P[0],P[1],P[2],1 }, TP[4];
	Matrix_Multiply(T, 3, 4, P1, 1, TP);
	Get_dTP_dKsi(TP, Deriv);
	return;
}
template<typename _T>void Get_dPd_dPn(_T x, _T y, _T k1, _T k2, _T k3, _T p1, _T p2, _T dPd_dPn[4])
{//(x,y) 为归一化平面未畸变前的坐标，畸变是求导中的难点，特别容易出错
	_T r2 = x * x + y * y, r4 = r2 * r2, r6 = r2 * r4;
	//1 + k1 * r² + k2 * r^4 + k3 * r^6
	_T d = 1 + k1 * r2 + k2 * r4 + k3 * r6;
	//dd / dr2 = k1 + 2 * k2 * r2 + 3 * K3 * r2 ^ 2
	_T dd_dr2 = k1 + 2 * k2 * r2 + 3 * k3 * r2 * r2;

	//d + 2x^2  * dd/dr2 + 2*p1*y + 6*p2*x
	dPd_dPn[0] = d + 2 * x * x * dd_dr2 + 2 * p1 * y + 6 * p2 * x;
	//2*x*y * dd/dr2 + 2*p1*x + 2*p2*y
	dPd_dPn[2] = dPd_dPn[1] = 2 * x * y * dd_dr2 + 2 * p1 * x + 2 * p2 * y;
	//d + 2y^2 * dyd/dr2 + 6 * p1 * y + 2*p2*x 
	dPd_dPn[3] = d + 2 * y * y * dd_dr2 + 6 * p1 * y + 2 * p2 * x;
}

template void Get_PnP_Deriv(float P[4], float uv_Ref[2], float T[4 * 4], float K[3 * 3], float D[5], float dE_dK[], float dE_dKsi[], float dE_dD[], float dE_dP[], float E[2]);
template void Get_PnP_Deriv(double P[4], double uv_Ref[2], double T[4 * 4], double K[3 * 3], double D[5], double dE_dK[], double dE_dKsi[], double dE_dD[], double dE_dP[], double E[2]);
template<typename _T>void Get_PnP_Deriv(_T P[4], _T uv_Ref[2],_T T[4 * 4], _T K[3 * 3], _T D[5],
	_T dE_dK[],_T dE_dKsi[], _T dE_dD[], _T dE_dP[],_T E[2])
{//搞个满血版的求导，只作为基准程序对数据用
	//第一步，求TP
	_T Pt[3];
	Matrix_Multiply(T, 3, 4, P, 1, Pt);
	//Disp(Pt, 1, 3, "Pt");

	//投影到归一化平面
	_T x = Pt[0] / Pt[2], y = Pt[1] / Pt[2];
	_T Pn[2] = { x,y };

	//根据畸变参数与归一化坐标算出具体畸变得距离
	_T d[2], dc[2 * 5];
	Get_Distort_Coeff(x, y, D, dc);
	Matrix_Multiply(dc, 2, 5, D, 1, d);

	_T Pd[2];
	Vector_Add(Pn, d, 2, Pd);

	//再将归一化平面投影到像素平面
	_T UV[2];
	UV[0] = Pd[0] * K[0] + K[2];
	UV[1] = Pd[1] * K[4] + K[5];
	/*Disp(K, 3, 3, "K");
	Disp(D, 1, 5, "D");
	Disp(T, 4, 4, "T");
	Disp(P, 1, 3, "P");*/
	//Disp(UV, 1, 2, "uv");

	_T E1[2];
	E1[0] = uv_Ref[0] - UV[0];
	E1[1] = uv_Ref[1] - UV[1];
	//Disp(uv_Ref, 1, 2, "uv'");
	//Disp(E1, 1, 2, "E");

	//轮到求导
	_T dLoss_dE[2] = { -1,-1 };

	_T dLoss_dUV[2] = { -E1[0],-E1[1] };	//dLoss/dE = (-eu,	-ev)

	_T dE_dK1[2 * 4] = { -Pd[0],	0, -1,	0,
						0,		-Pd[1],		0, -1 };
	//Disp(dE_dK1, 2, 4, "dE/dK");

	//对畸变后的坐标进行求导
	_T fx = K[0], fy = K[4];
	_T dE_dPd[2 * 2] = { -fx,0,
						0,-fy };
	//Disp(dE_dPd, 2, 2, "dE/dPd");

	_T dE_dD1[2 * 5];	//= dE/dPd * dt
	Matrix_Multiply(dE_dPd, 2, 2, dc, 5, dE_dD1);
	//Disp(dE_dPd, 2, 2, "dE/dPd");
	//Disp(dE_dD1, 2, 5, "dE/dD");

	_T dPd_dPn[4];
	Get_dPd_dPn(x, y, D[0], D[1], D[2], D[3], D[4], dPd_dPn);
	//Disp(dPd_dPn, 2, 2, "dPd/dPn");

	_T dE_dPn[2*2];	//dE/dPn = dE/dPd * dPd/dPn 	2x2 * 2x2 => 2x2
	Matrix_Multiply(dE_dPd, 2, 2, dPd_dPn, 2, dE_dPn);
	//Disp(dE_dPn, 2, 2, "dE/dPn");

	_T Ptz_Recip = 1 / Pt[2], Ptz_Recip_Sqr = Ptz_Recip * Ptz_Recip;
	_T dPn_dPt[2 * 3] = { Ptz_Recip,0,-Pt[0] * Ptz_Recip_Sqr,
					0,Ptz_Recip ,-Pt[1] * Ptz_Recip_Sqr };
	//Disp(dPn_dPt, 2, 3, "dPn/dPt");

	_T dE_dPt[2 * 3];
	Matrix_Multiply(dE_dPn, 2, 2, dPn_dPt, 3, dE_dPt);
	//Disp(dE_dPt, 2, 3, "dE/dPt");

	_T dTP_dKsi[4 * 6];
	Get_dTP_dKsi(Pt, dTP_dKsi);
	//Disp(dTP_dKsi, 3, 6, "dTP/dKsi");

	_T dE_dKsi1[2 * 6];
	Matrix_Multiply(dE_dPt, 2, 3, dTP_dKsi, 6, dE_dKsi1);
	//Disp(dE_dKsi1, 2, 6, "dE/dKsi");

	_T dE_dP1[2 * 3];	//= dE/dPt * R
	_T R[3 * 3];
	Get_R_t<_T>(T, R, NULL);
	Matrix_Multiply(dE_dPt, 2, 3, R, 3, dE_dP1);
	//Disp(dE_dP1, 2, 3, "dE/dP");

	if (dE_dK)
		memcpy(dE_dK, dE_dK1, 2 * 4 * sizeof(_T));
	if (dE_dKsi)
		memcpy(dE_dKsi, dE_dKsi1, 2 * 6 * sizeof(_T));
	if (dE_dD)
		memcpy(dE_dD, dE_dD1, 2 * 5 * sizeof(_T));
	if (dE_dP)
		memcpy(dE_dP, dE_dP1, 2 * 3 * sizeof(_T));
	if (E)
		memcpy(E, E1, 2 * sizeof(_T));

	return;
}
template void Get_uv_Ref(float P[4], float T[4 * 4], float K[3 * 3], float D[5], float uv[2]);
template void Get_uv_Ref(double P[4], double T[4 * 4], double K[3 * 3], double D[5], double uv[2]);
template<typename _T>void Get_uv_Ref(_T P[4], _T T[4 * 4], _T K[3 * 3], _T D[5], _T uv[2])
{//搞一个满血版的从空间点开始，经过相机投影，畸变，投影到像素平面
	//第一步，求TP
	_T Pt[3];
	Matrix_Multiply(T, 3, 4, P, 1, Pt);
	//Disp(Pt, 1, 3, "Pt");
	//投影到归一化平面
	_T x = Pt[0] / Pt[2], y = Pt[1] / Pt[2];
	_T Pn[2] = { x,y };
	//Disp(Pn, 1, 2, "Pn");
	//根据畸变参数与归一化坐标算出具体畸变得距离
	_T d[2];
	Get_Distort_Value(x, y, D, d);

	_T Pd[2];
	Vector_Add(Pn, d, 2, Pd);
	//Disp(Pd, 1, 2, "Pd");

	//再将归一化平面投影到像素平面
	uv[0] = Pd[0] * K[0] + K[2];
	uv[1] = Pd[1] * K[4] + K[5];
	//printf("u= %f*%f+%f= %f", Pd[0], K[0], K[2], uv[0]);

	return;
}
//************************一组投影函数************************/

//***********************李群李代数**********************************/
template void Get_J_E_uv(float P[4], float uv[2], float T[3 * 4], float K[4], float D[5], float J[2][15], float E[2]);
template void Get_J_E_uv(double P[4], double uv[2], double T[3 * 4], double K[4], double D[5], double J[2][15], double E[2]);
template<typename _T>void Get_J_E_uv(_T P[4], _T uv[2], _T T[3 * 4], _T K[4], _T D[5], _T J[2][15], _T E[2])
{//在像素平面上求雅可比，残差
///给定一个空间点，给定相机位姿T, 内参K,D，一个对应的uv，算个雅可比，误差E
	_T dE_dKsi[2 * 6], dE_dK[2 * 4], dE_dD[2 * 5];
	Get_PnP_Deriv<_T>(P, uv, T, K, D, dE_dK, dE_dKsi, dE_dD, NULL, E);
	//Disp(dE_dK, 2, 4, "dE/dK");
	//Disp(dE_dD, 2, 5, "dE/dD");
	//Disp(dE_dKsi, 2, 6, "dE/dKsi");
	Copy_Matrix_Partial<_T>(dE_dKsi, 2, 6, (_T*)J, 15, 0, 0);
	Copy_Matrix_Partial<_T>(dE_dK, 2, 4, (_T*)J, 15, 6, 0);
	Copy_Matrix_Partial<_T>(dE_dD, 2, 5, (_T*)J, 15, 10, 0);
	return;
}

template<typename _T>void TP(_T T[3 * 4], _T P[3], _T Pt[3])
{//Pt: 表示P经过相机位姿T 变换后的相机位置 Pt = TP
	Pt[0] = T[0] * P[0] + T[1] * P[1] + T[2] * P[2] + T[3];
	Pt[1] = T[4] * P[0] + T[5] * P[1] + T[6] * P[2] + T[7];
	Pt[2] = T[8] * P[0] + T[9] * P[1] + T[10] * P[2] + T[11];
}

template<typename _T>void Get_PnP_Norm_Deriv(_T P[4], _T uv_Ref[2], _T T[4 * 4], _T K[3 * 3], _T D[5],
	_T dE_dK[2 * 4] = NULL, _T dE_dKsi[2 * 6] = NULL, _T dE_dD[2 * 5] = NULL, _T dE_dP[2 * 3] = NULL, _T E[2] = NULL)
{
	_T Pt[3];
	TP(T, P, Pt);

	//投影到归一化平面
	_T x = Pt[0] / Pt[2], y = Pt[1] / Pt[2];
	_T Pn[2] = { x,y };
	//Disp(Pn, 1, 2, "Pn");

	_T d[2], dc[2 * 5];
	Get_Distort_Coeff<_T>(x, y, D, dc);
	Matrix_Multiply(dc, 2, 5, D, 1, d);

	_T Pd[2];
	Vector_Add(Pn, d, 2, Pd);
	//Disp(Pd, 1, 2, "Pd");

	_T E1[2], UVn[2] = { (uv_Ref[0] - K[2]) / K[0], (uv_Ref[1] - K[3]) / K[1] };
	Vector_Minus(UVn, Pd, 2, E1);
	//Disp(E1, 1, 2, "E");

	_T dE_dK1[] = { -(uv_Ref[0] - K[2]) / (K[0] * K[0]),0, -1 / K[0],0,
					0,-(uv_Ref[1] - K[3]) / (K[1] * K[1]),0,-1 / K[1] };
	//Disp(dE_dK1, 2, 4, "dE/dK");

	//对归一化平面坐标进行求导
	_T r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
	_T dn = 1 + D[0] * r2 + D[1] * r4 + D[2] * r6;
	_T dPd_dPn[4] = { dn + 2 * D[3] * Pn[1] + 4 * D[4] * Pn[0] ,	2 * D[3] * Pn[1],
				2 * D[4] * Pn[1],	dn + 4.f * D[3] * Pn[1] + 2 * D[4] * Pn[0] };


	_T dE_dPd[2 * 2] = { -1,0,
				0,-1 };
	//Disp(dE_dPd, 2, 2, "dE/dPd");

	_T dE_dD1[2 * 5];	//= dE/dPd * dt
	Matrix_Multiply(dE_dPd, 2, 2, dc, 5, dE_dD1);
	//Disp(dE_dD1, 2, 5, "dE/dD");
	_T dE_dPn[2 * 2];
	Matrix_Multiply(dE_dPd, 2, 2, dPd_dPn, 2, dE_dPn);

	_T Ptz_Recip = 1 / Pt[2], Ptz_Recip_Sqr = Ptz_Recip * Ptz_Recip;
	_T dPn_dPt[2 * 3] = { Ptz_Recip,0,-Pt[0] * Ptz_Recip_Sqr,
					0,Ptz_Recip ,-Pt[1] * Ptz_Recip_Sqr };

	_T dE_dPt[2 * 3];
	Matrix_Multiply(dE_dPn, 2, 2, dPn_dPt, 3, dE_dPt);
	//Disp(dE_dPt, 2, 3, "dE/dPt");

	_T dTP_dKsi[4 * 6];
	Get_dTP_dKsi(Pt, dTP_dKsi);
	Disp(dTP_dKsi, 3, 6, "dTP/dKsi");

	_T dE_dKsi1[2 * 6];
	Matrix_Multiply(dE_dPt, 2, 3, dTP_dKsi, 6, dE_dKsi1);
	//Disp(dE_dKsi1, 2, 6, "dE/dKsi");

	_T dE_dP1[2 * 3];	//= dE/dPt * R
	_T R[3 * 3];
	Get_R_t<_T>(T, R, NULL);
	Matrix_Multiply(dE_dPt, 2, 3, R, 3, dE_dP1);
	//Disp(dE_dP1, 2, 3, "dE/dP");

	if (dE_dK)
		memcpy(dE_dK, dE_dK1, 2 * 4 * sizeof(_T));
	if (dE_dKsi)
		memcpy(dE_dKsi, dE_dKsi1, 2 * 6 * sizeof(_T));
	if (dE_dD)
		memcpy(dE_dD, dE_dD1, 2 * 5 * sizeof(_T));
	if (dE_dP)
		memcpy(dE_dP, dE_dP1, 2 * 3 * sizeof(_T));
	if (E)
		memcpy(E, E1, 2 * sizeof(_T));

	return;
}

template void Get_J_E_Norm(float P[4], float uv[2], float T[3 * 4], float K[4], float D[5], float J[2][15], float E[2]);
template void Get_J_E_Norm(double P[4], double uv[2], double T[3 * 4], double K[4], double D[5], double J[2][15], double E[2]);
template<typename _T>void Get_J_E_Norm(_T P[4], _T uv[2], _T T[3 * 4], _T K[4], _T D[5], _T J[2][15], _T E[2])
{//在归一化平面上求雅可比，残差
///给定一个空间点，给定相机位姿T, 内参K,D，一个对应的uv，算个雅可比，误差E
	_T dE_dKsi[2 * 6], dE_dK[2 * 4], dE_dD[2 * 5];
	Get_PnP_Norm_Deriv<_T>(P, uv, T, K, D, dE_dK, dE_dKsi, dE_dD, NULL, E);
	Copy_Matrix_Partial<_T>(dE_dKsi, 2, 6, (_T*)J, 15, 0, 0);
	Copy_Matrix_Partial<_T>(dE_dK, 2, 4, (_T*)J, 15, 6, 0);
	Copy_Matrix_Partial<_T>(dE_dD, 2, 5, (_T*)J, 15, 10, 0);
	return;
}

template<typename _T>void Get_J_by_Rotation_Vector(_T Rotation_Vector_4[4], _T J[])
{//给定的旋转向量，求出J矩阵。旋转向量为4元组
	//再求位移t,先求出个J，a为旋转向量的转轴
	_T fValue, Temp_1[3][3], I[3][3] = { {1,0,0},{0,1,0},{0,0,1} };
	int i;
	memset(J, 0, 3 * 3 * sizeof(_T));

	//先求J第一部分 (sin(theta)/theta)*I
	if (Rotation_Vector_4[3] != 0)
		fValue = (_T)sin(Rotation_Vector_4[3]) / Rotation_Vector_4[3];
	else
		fValue = 0;
	for (i = 0; i < 9; i++)
		((_T*)J)[i] = fValue * ((_T*)I)[i];

	//再求J第二部分 (1-sin(theta)/theta) * axa'
	fValue = 1.f - fValue;
	Matrix_Multiply(Rotation_Vector_4, 3, 1, Rotation_Vector_4, 3, (_T*)Temp_1);
	//Disp((float*)Temp_1, 3, 3);
	for (i = 0; i < 9; i++)
		((_T*)J)[i] += fValue * ((_T*)Temp_1)[i];

	//再求第三部分 (1-cos(theta))/theta * a^
	if (Rotation_Vector_4[3] != 0)
		fValue = (_T)(1 - cos(Rotation_Vector_4[3])) / Rotation_Vector_4[3];
	else
		fValue = 0;
	Hat(Rotation_Vector_4, (_T*)Temp_1);
	//Disp((float*)Temp_1, 3, 3);
	for (i = 0; i < 9; i++)
		((_T*)J)[i] += fValue * ((_T*)Temp_1)[i];
	//Disp((float*)J, 3, 3);
}

template void se3_2_SE3(float Ksi[6], float T[]);
template void se3_2_SE3(double Ksi[6], double T[]);
template<typename _T>void se3_2_SE3(_T Ksi[6], _T T[])
{//T是6维 se3向量Ksi对应的4x4矩阵, Ksi前rho后phi
//注意：感觉这个转坏是错误的，问题在J上
//转换完毕后，T完全与图形学的三维转换矩阵一致
//总结， SE3中的旋转矩阵，se3中的六维向量，都是先旋转后位移
	//首先求R
	_T R[3][3];
	_T Rotation_Vector[4];

	Disp(&Ksi[3], 1, 3, "Ksi");
	Normalize(&Ksi[3], 3, Rotation_Vector);
	Rotation_Vector[3] = fGet_Mod(&Ksi[3], 3);	//此处已经将旋转向量化为4维表示

	Rotation_Vector_4_2_Matrix(Rotation_Vector, (_T*)R);
	Disp((_T*)R, 3, 3,"R");

	_T J[3][3], J_Rho[3];

	//问题可能就在下面
	//显然，J与ρ无关，只从φ推导出来
	Get_J_by_Rotation_Vector(Rotation_Vector, (_T*)J);
	Matrix_Multiply((_T*)J, 3, 3, Ksi, 1, J_Rho);

	//然后将 R,J_Rho, 0', 1组合成T
	T[0] = R[0][0], T[1] = R[0][1], T[2] = R[0][2], T[3] = J_Rho[0];
	T[4] = R[1][0], T[5] = R[1][1], T[6] = R[1][2], T[7] = J_Rho[1];
	T[8] = R[2][0], T[9] = R[2][1], T[10] = R[2][2], T[11] = J_Rho[2];
	T[12] = T[13] = T[14] = 0, T[15] = 1;

	return;
}

template void Gen_Pose_By_V3_t(float R[], float t[], float T[]);
template void Gen_Pose_By_V3_t(double R[], double t[], double T[]);
template<typename _T>void Gen_Pose_By_V3_t(_T V3[], _T t[], _T T[])
{
	_T R[3 * 3];
	Rotation_Vector_3_2_Matrix(V3, R);
	Gen_Pose_By_R_t(R, t, T);
}
template void Gen_Pose_By_R_t(float R[], float t[], float T[]);
template void Gen_Pose_By_R_t(double R[], double t[], double T[]);
template<typename _T>void Gen_Pose_By_R_t(_T R[], _T t[], _T T[])
{//安全函数，目标可以等于源
//用旋转坐标与位移坐标构成一个4x4 齐次变换矩阵，此处由旋转与平移构成
//这个矩阵的物理意义应该是先旋转后平移
	_T T1[4 * 4];
	if (R)
	{
		T1[0] = R[0], T1[1] = R[1], T1[2] = R[2];
		T1[4] = R[3], T1[5] = R[4], T1[6] = R[5];
		T1[8] = R[6], T1[9] = R[7], T1[10] = R[8];
	}
	else
	{//此处代考
		memset(T1, 0, 4 * 4 * sizeof(_T));
		T1[0] = T[1 * 4 + 1] = T[2 * 4 + 2] = 0;
	}
	T1[15] = 1;
	if (t)
	{
		T1[3] = t[0];
		T1[7] = t[1];
		T1[11] = t[2];
	}
	else
		T1[3] = T1[7] = T1[11] = 0;

	T1[12] = T1[13] = T1[14] = 0;
	memcpy(T, T1, 4 * 4 * sizeof(_T));
	return;
}

template void T_2_R9_t(float T[3 * 4], float R[3 * 3], float t[3]);
template void T_2_R9_t(double T[3 * 4], double R[3 * 3], double t[3]);
template<typename _T>void T_2_R9_t(_T T[3 * 4], _T R[3 * 3], _T t[3])
{
	Copy_Matrix_Partial(T, 4, 4, R, 3, 0, 0);
	t[0] = T[3], t[1] = T[7], t[2] = T[11];
}
template void Vee(float M[], float V[3]);
template void Vee(double M[], double V[3]);
template<typename _T>void Vee(_T M[], _T V[3])
{//反对称矩阵到向量
	V[0] = M[7];
	V[1] = M[2];
	V[2] = M[3];
	return;
}

template void Hat(float V[], float M[]);
template void Hat(double V[], double M[]);
template<typename _T>void Hat(_T V[], _T M[])
{//根据给定的向量构造反对称矩阵，改回与书中一致
	if (V == M)
	{
		_T M1[3 * 3];
		M1[0] = M1[4] = M1[8] = 0;
		M1[1] = -V[2], M1[3] = V[2];
		M1[2] = V[1], M1[6] = -V[1];
		M1[5] = -V[0], M1[7] = V[0];
		memcpy(M, M1, 3 * 3 * sizeof(_T));
	}
	else
	{
		M[0] = M[4] = M[8] = 0;
		M[1] = -V[2], M[3] = V[2];
		M[2] = V[1], M[6] = -V[1];
		M[5] = -V[0], M[7] = V[0];
	}
	return;
}
template void Get_R_t(float T[4 * 4], float R[3 * 3], float t[3]);
template void Get_R_t(double T[4 * 4], double R[3 * 3], double t[3]);
template<typename _T>void Get_R_t(_T T[4 * 4], _T R[3 * 3], _T t[3])
{//从4x4 齐次矩阵中抽取R，t
	if (R)
	{
		R[0] = T[0], R[1] = T[1], R[2] = T[2];
		R[3] = T[4], R[4] = T[5], R[5] = T[6];
		R[6] = T[8], R[7] = T[9], R[8] = T[10];
	}
	if (t)
		t[0] = T[3], t[1] = T[7], t[2] = T[11];
	return;
}
//***********************李群李代数**********************************/

//******************************画出位姿*****************************************************/
template void Draw_Camera(Point_Cloud<float>* poPC, float T[4 * 4], int R, int G, int B);
template void Draw_Camera(Point_Cloud<double>* poPC, double T[4 * 4], int R, int G, int B);
template<typename _T>void Draw_Camera(Point_Cloud<_T>* poPC, _T T[4 * 4], int R, int G, int B)
{//看看能否画出个简陋的相机位姿

	//先对T求逆
	_T T_Inv[4 * 4];
	int iResult;
	Get_Inv_Matrix(T, T_Inv, 4, &iResult);

	_T View_Point[4] = { T_Inv[3],T_Inv[7],T_Inv[11],1 };    //此处应该算是视点
	_T Norm_Plane[4][4] = { {-1,1,-1,1 },
		{1,1,-1,1},
		{1,-1,-1,1},
		{-1,-1,-1,1} };    //归一化平面上的4个点
	_T Norm_Center[4] = { 0,0,-1,1 };      //归一化平面上的中心
	//_T Temp[4];
	int i;
	//剩下的一律从原地搬过去
	//Draw_Sphere(poPC, View_Point[0], View_Point[1], View_Point[2],(_T)0.2f,40,255,0,0);
	Draw_Point(poPC, View_Point[0], View_Point[1], View_Point[2], R, G, B);

	Matrix_Multiply(T_Inv, 4, 4, Norm_Center, 1, Norm_Center);
	Draw_Line(poPC, View_Point[0], View_Point[1], View_Point[2], Norm_Center[0], Norm_Center[1], Norm_Center[2], 50, R, G, B);

	for (i = 0; i < 4; i++)
	{
		Matrix_Multiply(T_Inv, 4, 4, Norm_Plane[i], 1, Norm_Plane[i]);
		Draw_Line(poPC, View_Point[0], View_Point[1], View_Point[2], Norm_Plane[i][0], Norm_Plane[i][1], Norm_Plane[i][2], 50, R, G, B);
	}
	for (i = 0; i < 4; i++)
		Draw_Line(poPC, Norm_Plane[i][0], Norm_Plane[i][1], Norm_Plane[i][2], Norm_Plane[(i + 1) & 3][0], Norm_Plane[(i + 1) & 3][1], Norm_Plane[(i + 1) & 3][2], 50, R, G, B);

	Draw_Line(poPC, Norm_Plane[0][0], Norm_Plane[0][1], Norm_Plane[0][2], Norm_Plane[2][0], Norm_Plane[2][1], Norm_Plane[2][2], 50, R, G, B);
	Draw_Line(poPC, Norm_Plane[1][0], Norm_Plane[1][1], Norm_Plane[1][2], Norm_Plane[3][0], Norm_Plane[3][1], Norm_Plane[3][2], 50, R, G, B);
	return;
}
//******************************画出位姿*****************************************************/

template<typename _T>void Get_H_Block(_T J[2][15], _T E[2], _T Camera[6 * 6], _T Cam_Corner[6 * 9], _T Corner[9 * 9], _T JtE[6 + 9])
{//单独拎出来搞块
	//Camera	属于位姿块，6x6	左上角
	//Corner	属于内参块，9x9	右下角
	//Cam_Corner 属于位姿-内参块,右上，左下角，堆成，所以只寸一个
	//JtE:		也要累加进去
	int i, j, iDest_Pos;

	////造一些数据来看看
	//for (int i = 0; i < 2; i++)
	//	for (int j = 0; j < 15; j++)
	//		J[i][j] = (i + 1) + j;

	//_T Temp[15 * 15] = {};
	//Transpose_Multiply<_T>((_T*)J, 2, 15, Temp, 0);

	//Disp((_T*)J, 2, 15, "J");
	//Disp(Temp, 15, 15, "Temp");

	for (i = 0; i < 6; i++)
	{
		iDest_Pos = i * 6 + i;
		for (j = i; j < 6; j++, iDest_Pos++)
		{
			//iDest_Pos = i * 6 + j;
			Camera[iDest_Pos] += J[0][i] * J[0][j] + J[1][i] * J[1][j];
			//Corner[iDest_Pos] += J[0][i + 6] * J[0][j + 6] + J[1][i + 6] * J[1][j + 6];
		}
	}
	//Disp(Camera, 6, 6, "Camera");
	for (i = 0; i < 9; i++)
	{
		iDest_Pos = i * 9 + i;
		for (j = i; j < 9; j++, iDest_Pos++)
			Corner[iDest_Pos] += J[0][i + 6] * J[0][j + 6] + J[1][i + 6] * J[1][j + 6];
	}
	//Disp(Corner, 9, 9, "Corner");
	for (i = 0; i < 6; i++)
		for (j = 0; j < 9; j++)
			Cam_Corner[i * 9 + j] += J[0][i] * J[0][j + 6] + J[1][i] * J[1][j + 6];

	for (int i = 0; i < 15; i++)
		JtE[i] += J[0][i] * E[0] + J[1][i] * E[1];

	//Disp(Cam_Corner, 6, 9, "Cam_Corner");
	return;
}

template void Get_H_JtE(float T[][3 * 4], float K[4], float D[5], float P[][2], Point_2D<float> uv[], int iObservation_Count, int iOrder, float H[], float JtE[]);
template void Get_H_JtE(double T[][3 * 4], double K[4], double D[5], double P[][2], Point_2D<double> uv[], int iObservation_Count, int iOrder, double H[], double JtE[]);
template<typename _T>void Get_H_JtE(_T T[][3 * 4], _T K[4], _T D[5], _T P[][2], Point_2D<_T> uv[],	int iObservation_Count, int iOrder, _T H[], _T JtE[])
{//给定所有的位姿，K,D，生成一个H = J'J, 预计J'E
	_T Camera[6 * 6] = {}, Cam_KD[6 * 9] = {}, K_D[9 * 9] = {},
		JtE_1[6 + 9] = {};
	int iPre_Cam_Index = uv[0].m_iCamera_Index,
		iCount_Minus_1 = iObservation_Count - 1;
	memset(JtE, 0, iOrder * sizeof(_T));
	memset(H, 0, iOrder * iOrder * sizeof(_T));

	_T K9[3 * 3];
	K4_2_K9(K, K9);
	for (int i = 0; i < iObservation_Count;)
	{
		//每一点都能算出一个雅可比，E
		Point_2D<_T> oUV = uv[i];
		if (oUV.m_iCamera_Index != iPre_Cam_Index)
		{
			//补下三角
			for (int y = 1; y < 6; y++)
			{
				for (int x = 0; x < y; x++)
				{
					int iDest_Pos = y * 6 + x,
						iSource_Pos = x * 6 + y;
					Camera[iDest_Pos] = Camera[iSource_Pos];
				}
			}for (int y = 1; y < 9; y++)
			{
				for (int x = 0; x < y; x++)
				{
					int iDest_Pos = y * 9 + x,
						iSource_Pos = x * 9 + y;
					K_D[iDest_Pos] = K_D[iSource_Pos];
				}
			}

			int iCam_Index = iPre_Cam_Index != -1 ? iPre_Cam_Index : oUV.m_iCamera_Index;

			//将块拷到sigma_H
			Copy_Matrix_Partial(Camera, 6, 6, H, iOrder, iCam_Index * 6, iCam_Index * 6);
			Copy_Matrix_Partial(Cam_KD, 6, 9, H, iOrder, iOrder - 9, iCam_Index * 6);

			memcpy(JtE + iCam_Index * 6, JtE_1, 6 * sizeof(_T));
			Vector_Add(JtE + iOrder - 9, JtE_1 + 6, 9, JtE + iOrder - 9);

			_T* pDest_1 = &H[(iOrder - 9) * iOrder + iCam_Index * 6];
			for (int y = 0; y < 9; y++, pDest_1 += iOrder)
				for (int x = 0; x < 6; x++)
					pDest_1[x] = Cam_KD[x * 9 + y];

			if (iPre_Cam_Index == -1)
				break;
			memset(Camera, 0, 6 * 6 * sizeof(_T));
			memset(Cam_KD, 0, 6 * 9 * sizeof(_T));
			memset(JtE_1, 0, 15 * sizeof(_T));
			iPre_Cam_Index = oUV.m_iCamera_Index;
			continue;
		}

		_T J[2][6 + 9], E[2];
		_T* P1 = P[oUV.m_iPoint_Index];
		_T P2[4] = { P1[0],P1[1],0,1 };

		Get_J_E_uv(P2, oUV.m_Pos, T[oUV.m_iCamera_Index], K9, D, J, E);

		//然后用J 算一个 H = J'J
		Get_H_Block<_T>(J, E, Camera, Cam_KD, K_D, JtE_1);

		if (i == iCount_Minus_1)
			iPre_Cam_Index = -1;
		else
			i++;
	}
	//Disp_Fillness(H, iWidth_H, iWidth_H, "H");
	Copy_Matrix_Partial(K_D, 9, 9, H, iOrder, iOrder - 9, iOrder - 9);

	////验算
	//memset(H, 0, 15 * 15 * sizeof(_T));
	//memset(JtE, 0, 15 * sizeof(_T));
	//_T fTotal = 0;
	//for (int i = 0; i < iObservation_Count;i++)
	//{
	//	Point_2D<_T> oUV = uv[i];
	//	_T J[2][6 + 9], E[2], JtJ[15 * 15];
	//	_T* P1 = P[oUV.m_iPoint_Index];
	//	_T P2[3] = { P1[0],P1[1],0 };
	//	Get_J_E_Norm(P2, oUV.m_Pos, T[oUV.m_iCamera_Index], K, D, J, E);
	//	//Disp((_T*)J, 2, 15, "J");
	//	//Disp((_T*)E, 2, 1, "E");

	//	Transpose_Multiply((_T*)J, 2, 15, JtJ, 0);
	//	//printf("%f\n", JtJ[0]);
	//	Vector_Add(H, JtJ, 15 * 15, H);
	//	//fTotal += JtJ[0];
	//	//printf("fTotal:%f H[0]:%f\n", fTotal, H[0]);

	//	At_x_B((_T*)J, 2, 15, E, 1, JtE_1);
	//	printf("%f\n", JtE_1[0]);
	//	fTotal += JtE_1[0];
	//	Vector_Add(JtE, JtE_1, 15, JtE);
	//}
	////printf("%f\n", fGet_Cond_Num(H, 15));
	////Disp(H, 15, 15, "H");
	////Disp(JtE, 15, 1, "JtE");
	return;
}