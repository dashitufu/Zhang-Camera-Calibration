#include "Common.h"
#include "Matrix.h"
#include "../esp_dsp_pc/esp_dsp.h"

static void Test_1()
{
	typedef float _T;
	const int  ma = 1920,na = 1080,
		mb = na, nb = 1920,
		mc = ma,nc = nb;

	_T* A = (_T*)pMalloc(na * ma * sizeof(_T)),
		* B = (_T*)pMalloc(nb * mb * sizeof(_T)),
		* C = (_T*)pMalloc(nc * mc * sizeof(_T));
	if (!A || !B || !C)
		return;

	//дьЪ§Он
	for (int y = 0; y < ma; y++)
		for (int x = 0; x < na; x++)
			A[y * na + x] =(float)(y * 10 + x);	//y + x / 100.f;
	for (int y = 0; y < mb; y++)
		for (int x = 0; x < nb; x++)
			B[y * nb + x] =(float)(y * 10 + x);	// y + x / 10.f;

	unsigned long long tStart = iGet_Tick_Count();
	//for(int i=0;i<1000;i++)
	Matrix_Multiply(A, ma, na, B, nb, C);
	//Matrix_Multiply_float_1((float*)A, ma, na, (float*)B, nb, (float*)C);
	printf("%lld %f\n", iGet_Tick_Count() - tStart,fGet_Mod(C,mc*nc));

	/*Disp(A, ma, na, "A");
	Disp(B, mb, nb, "B");
	Disp(C, mc, nc, "C");*/
	Free(A);	Free(B);	Free(C);
	return;
}
static void Test_2()
{
	float A[] = { 1,2,3,4 },
		B[] = { 2,3,4,5 },
		C[4];

	dsps_mul_f32(A, B, C,4,1,1,1);
	Disp(C, 1, 4, "C");
	return;
}
int main_2()
{
	bInit_Env_CPU(800000000, 128, 97);
	Test_1();
	//Test_2();

	Free_Env_CPU();
#ifdef WIN32
	_CrtDumpMemoryLeaks();
#endif
	return 0;
}