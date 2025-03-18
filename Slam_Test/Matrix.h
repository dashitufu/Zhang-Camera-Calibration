//由于数学库同时应对float/double两种类型，故此该版开始，全变为模板函数
#pragma once 
#include "Common.h"
//extern "C"
//{
//	#include "Buddy_System.h"
//}

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

#define PI 3.1415926f
#define Abs(A) ((A)>=0?(A):(-(A)))
#define MAX_FLOAT ((float)0xFFFFFFFFFFFFFFFF)	//仅仅给一个足够大的数字，并不是iEEE的浮点数最大值
#define ZERO_APPROCIATE	0.00001f

#define Get_Homo_Pos(Pos, Homo_Pos) Homo_Pos[0]=Pos[0], Homo_Pos[1]=Pos[1], Homo_Pos[2]=Pos[2],Homo_Pos[3]=1;

typedef struct Complex_d {//双精度复数
	double real;	//实部
	double im;		//虚部 imagary part	
}Complex_d;
typedef struct Complex_f {//双精度复数
	float real;	//实部
	float im;		//虚部 imagary part	
}Complex_f;

typedef struct Polynormial {	//多元多项式，次数必须为正整数(>=0, =0表示没有该项
	unsigned char* m_pTerm;		//多项式中的项
	float* m_pCoeff;			//每项的系数
	int m_iElem_Count;			//变量（元）数
	int m_iTerm_Count;			//项数
	int m_iMax_Term_Count;		//目前最大项数，为日后扩展用
}Polynormial;

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

template<typename _T> class Sparse_Matrix {
public:
	typedef class Item {
	public:
		unsigned int x, y;	//0xFFFFFFFF未非法值
		_T m_fValue;
		int m_iRow_Next;	//行方向下一个，向右
		int m_iCol_Next;	//列方向下一个，向下
	}Item;

	unsigned int* m_pRow=NULL;	//行桶
	unsigned int* m_pCol;	//列桶
	Item* m_pBuffer;		//内容从[1]开始 [0]表示NULL
	int m_iCur_Item=1;		//当前在哪个Item
	int m_iMax_Item_Count=0;		//整个matrix共右多少个item
	int m_iRow_Count;
	int m_iCol_Count;
};

unsigned long long iGet_Random_No_cv(unsigned long long* piState);
#define Malloc_1(oPtr, iSize,pBuffer) \
{ \
	(pBuffer)= (_T*)((oPtr).m_pBuffer+(oPtr).m_iCur); \
	(oPtr).m_iCur +=ALIGN_SIZE_128((iSize)); \
	if ( (oPtr).m_iCur > (oPtr).m_iMax_Buffer_Size) \
	{ \
		(oPtr).m_iCur -= ALIGN_SIZE_128((iSize));\
		(pBuffer)=NULL; \
		printf("Fail to allocate memory in Malloc, Total:%u Remain:%u Need:%d\n",(oPtr).m_iMax_Buffer_Size,(oPtr).m_iMax_Buffer_Size-(oPtr).m_iCur,(int)(iSize)); \
	} \
}
template<typename _T>void Disp(Sparse_Matrix<_T> oMatrix, const char Caption[] = NULL);
template<typename _T>void Disp_Fillness(Sparse_Matrix<_T> oMatrix);

template<typename _T>void Disp(_T* M, int iHeight, int iWidth,const char* pcCaption=NULL)
{
	int i, j;
	if (pcCaption)
		printf("%s\n", (char*)pcCaption);

	for (i = 0; i < iHeight; i++)
	{
		for (j = 0; j < iWidth; j++)
		{
			if (std::is_same_v<_T, float>)
				printf("%.8f, ", (float)M[i * iWidth + j]);
			else if(std::is_same_v<_T,double>)
				printf("%.7f, ", (double)M[i * iWidth + j]);
			else if (std::is_same_v<_T, int> || std::is_same_v<_T, unsigned int>)
				printf("%d,", (int)M[i * iWidth + j]);
			else if(std::is_same_v<_T,short> || std::is_same_v<_T, unsigned short>)
				printf("%d,", (short)M[i * iWidth + j]);
		}
		printf("\n");
	}
	return;
}
template<typename _T>void Disp_Fillness(_T A[], int m, int n);

template<typename _T>void Matrix_Transpose(_T* A, int ma, int na, _T* At)
{//矩阵转置
	int y, x;
	_T* At_1;
	if (abs((int)(A -At))<ma*na )
		At_1 = (_T*)pMalloc(&oMatrix_Mem, ma * na * sizeof(_T));
	else
		At_1 = At;
	for (y = 0; y < ma; y++)
		for (x = 0; x < na; x++)
			At_1[x * ma + y] = A[y * na + x];
	if (abs((int)(A - At)) < ma * na)
	{
		memcpy(At, At_1, ma * na * sizeof(_T));
		Free(&oMatrix_Mem, At_1);
	}
	return;
}

template<typename _T>void Matrix_Multiply_3x3(_T A[3*3], _T B[3*3], _T C[3*3])
{//计算C=AxB
	//Light_Ptr oPtr = oMatrix_Mem;
	_T C_1[9];
	//Malloc_1(oPtr, 3 * 3 * sizeof(_T),C_1);
	C_1[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	C_1[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	C_1[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

	C_1[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	C_1[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	C_1[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

	C_1[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	C_1[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	C_1[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
	memcpy(C, C_1, 9 * sizeof(_T));
	return;
}
template<typename _T>void Add_I_Matrix(_T A[], int iOrder, _T ramda=1);

void Init_Env();
void Free_Env();
//排列组合函数
unsigned long long iFactorial(int n);
unsigned long long iFactorial(unsigned long long iPre_Value, int n);

unsigned int iGet_Random_No();
template<typename _T>void Get_Random_Norm_Vec(_T V[], int n);;
unsigned long long iGet_Random_No_cv();
unsigned long long iGet_Random_No_cv(unsigned long long* piState);
int iGet_Random_No_cv(int a, int b);
template<typename _T> _T fGet_Random_No(_T mean, _T sigma);

//void Schmidt_Orthogon(float* A, int m, int n, float* B);	//施密特正交化
template<typename _T>void Schmidt_Orthogon(_T* A, int m, int n, _T* B);

void Polor_2_Rect_Coordinate(float rho, float theta, float phi, float* px, float* py, float* pz);	//极坐标转直角坐标系
void Polor_2_Rect_Coordinate(float rho, float theta, float* px, float* py);							//二维
void Rect_2_Polor_Coordinate(float x, float y, float z, float* prho, float* ptheta, float* pphi);	//直角坐标系转极坐标
void Rect_2_Screen_Coordinate(float x, float y, int* px_Screen, int* py_Screen, int iWidth = 1920, int iHeight = 1080);	//直角坐标系转屏幕坐标
void Screen_2_Coordinate(int x_Screen, int y_Screen, float* px, float* py, int iWidth = 1920, int iHeight = 1080);			//屏幕坐标转直角坐标系

//一组与线性方程有关的函数
template<typename _T>void LLt_Decompose(_T A[], int iOrder, _T B[], int* pbSuccess=NULL);
template<typename _T>void Cholosky_Decompose(_T A[], int iOrder, _T B[]=NULL,int* pbSuccess=NULL);
void Conjugate_Gradient(float* A, const int n, float B[], float X[]);
void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess);
void Solve_Linear_Cramer(float* A, int iOrder, float* B, float* X, int* pbSuccess); //克莱姆法则解线性方程组
template<typename _T>void Test_Linear(_T A[], int iOrder, _T B[], _T X[]);

template<typename _T>void Solve_Linear_Gause_AAt(_T* A, int iOrder, _T* B, _T* X, int* pbSuccess);
template<typename _T>void Solve_Linear_Gause(_T* A, int iOrder, _T* B, _T* X, int* pbSuccess);	//高斯法求解线性方程组
//void Solve_Linear_Gause(float* A, int iOrder, float* B, float* X, int* pbSuccess=NULL);
template<typename _T>void Solve_Linear_LLt(_T A[], int iOrder, _T* B, _T* X, int* pbSuccess=NULL);

template<typename _T>void Solve_Linear_Gauss_Seidel(_T A[], _T B[], int iOrder, _T X[], int* pbResult);
template<typename _T>void Solve_Linear_Jocabi(_T A[], _T B[], int iOrder, _T X[], int* pbResult);				//雅可比迭代法求解线性方程组
void Solve_Linear_Gauss_Seidel(float A[], float B[], int iOrder, float X[], int* pbResult);
template<typename _T> void Solve_Linear_Contradictory(_T* A, int m, int n, _T* B, _T* X, int* pbSuccess);
//void Solve_Linear_Contradictory(float* A, int m, int n, float* B, float* X, int* pbSuccess);	//解矛盾方程组，求最小二乘解
//void Solve_Linear_Solution_Construction(float* A, int m, int n, float B[], int* pbSuccess, float* pBasic_Solution = NULL, int* piBasic_Solution_Count = NULL, float* pSpecial_Solution = NULL);
template<typename _T> void Solve_Linear_Solution_Construction(_T* A, int m, int n, _T B[], int* pbSuccess, _T* pBasic_Solution=NULL, int* piBasic_Solution_Count=NULL, _T* pSpecial_Solution=NULL);
template<typename _T> void Solve_Linear_Solution_Construction_1(_T* A, int m, int n, _T B[], int* pbSuccess, _T* pBasic_Solution = NULL, int* piBasic_Solution_Count = NULL, _T* pSpecial_Solution = NULL);

void Get_Linear_Solution_Construction(float A[], int m, int n, float B[]);//看解的结构

//验算线性方程解
template<typename _T>_T fLinear_Equation_Check(_T* A, int iOrder, _T* B, _T* X, _T eps=1e-9);

template<typename _T>void Elementary_Row_Operation(_T A[], int m, int n, _T A_1[], int* piRank=NULL, _T** ppBasic_Solution=NULL, _T** ppSpecial_Solution=NULL);
template<typename _T>void Elementary_Row_Operation_1(_T A[], int m, int n, _T A_1[], int* piRank=NULL, _T** ppBasic_Solution=NULL, _T** ppSpecial_Solution=NULL);

int bIs_Linearly_Dependent(float A[], int m, int n);	//判断向量组A是否线性相关
template<typename _T>int bIs_Orthogonal(_T* A, int h, int w=0);
template<typename _T>int bIs_R(_T R[9], _T eps = 1e-6);
template<typename _T>void Gen_I_Matrix(_T M[], int h, int w);	//生成对角阵

void Gen_Iterate_Matrix(float A[], int n, float B[], float B_1[], float C[]);	//生成迭代中的B
int bIs_Contrative_Mapping(float B[], int n);					//判断一个矩阵是否为压缩矩阵，这是一个线性方程组是否能用迭代法求解的必要条件
int bIs_Diagonal_Dominant(float A[], int iOrder);			//判断一个方阵是否为对角线占优而且是行占优矩阵
template<typename _T>int bIs_Symmetric_Matrix(_T A[], int iOrder, const _T eps = (_T)1e-4);
template<typename _T>int bIs_Unit_Vector(_T* V, int na, _T eps = ZERO_APPROCIATE);//测试向量是否为单位向量
void Diagonalize(float A[], int iOrder, float Diag[], float P[], int* pbSuccess);	//对称矩阵对角化

void Solve_Cubic(double A[3][3], Complex_d Root[3], int* piRoot_Count);
void Solve_Eigen(double A[3][3], Complex_d Root[3], int* piRoot_Count);
void Solve_Eigen_3x3(float A[], Complex_f Root[3]);
void Solve_Eigen_Vector(float A[], int iOrder, float fEigen_Value, float Q[], int* piCount);	//根据给定的特征值代入A， 求解特征方程得到特征向量

void Solve_Poly(float Coeff[], int iCoeff_Count, Complex_f Root[], int* piRoot_Count = NULL);

//**************************稀疏矩阵*******************************
#define Get_Item(oMatrix, x1, y1, poItem) \
{ \
	int iCur; \
	iCur = (oMatrix).m_pRow[y1]; \
	if (!iCur) \
		poItem = NULL; \
	else \
	{\
		poItem = &(oMatrix).m_pBuffer[iCur];\
		while (poItem)\
		{\
			if (poItem->x == (unsigned int)x1)\
				break;\
			if (poItem->m_iRow_Next && poItem->x < (unsigned int)x1)\
				poItem = &(oMatrix).m_pBuffer[poItem->m_iRow_Next];\
			else\
			{\
				poItem = NULL;\
				break;\
			}\
		}\
	}\
}

template<typename _T> void Build_Link_Col_1(Sparse_Matrix<_T> oC)
{
	/*从最下一行向上建立Col Link*/
	int iItem_Index;
	typename Sparse_Matrix<_T>::Item* poItem;
	memset(oC.m_pCol, 0, oC.m_iCol_Count * sizeof(int));
	for (int i = oC.m_iRow_Count - 1; i >= 0; i--)
	{
		//if (i == 238)
			//printf("here");
		if (oC.m_pRow[i])
		{
			iItem_Index = oC.m_pRow[i];
			poItem = &oC.m_pBuffer[iItem_Index];
			while (1)
			{
				if (oC.m_pCol[poItem->x])/*此column下一行有东西*/
					poItem->m_iCol_Next = oC.m_pCol[poItem->x];
				oC.m_pCol[poItem->x] = iItem_Index;
				if (poItem->m_iRow_Next)
				{
					iItem_Index = poItem->m_iRow_Next;
					poItem = &oC.m_pBuffer[poItem->m_iRow_Next];
				}
				else
					break;
			}
		}
	}
}

#define Build_Link_Col(oC) \
{ \
	/*从最下一行向上建立Col Link*/ \
	int iItem_Index; \
	typename Sparse_Matrix<_T>::Item* poItem; \
	memset(oC.m_pCol,0,oC.m_iCol_Count*sizeof(int)); \
	for (int i = oC.m_iRow_Count - 1; i >= 0; i--) \
	{ \
		if (oC.m_pRow[i]) \
		{ \
			iItem_Index = oC.m_pRow[i]; \
			poItem = &oC.m_pBuffer[iItem_Index]; \
			while (1) \
			{ \
				if (oC.m_pCol[poItem->x])/*此column下一行有东西*/ \
					poItem->m_iCol_Next = oC.m_pCol[poItem->x]; \
				oC.m_pCol[poItem->x] = iItem_Index; \
				if (poItem->m_iRow_Next) \
				{ \
					iItem_Index = poItem->m_iRow_Next; \
					poItem = &oC.m_pBuffer[poItem->m_iRow_Next]; \
				}else \
					break; \
			} \
		} \
	} \
}
template<typename _T>int bIs_Symmetric_Matrix(Sparse_Matrix<_T> oA);
template<typename _T> _T fGet_Value(Sparse_Matrix<_T>* poMatrix, int x, int y);
template<typename _T>void Init_Sparse_Matrix(Sparse_Matrix<_T>* poMatrix, int iItem_Count, int w, int h);
//template<typename _T>void Init_Sparse_Matrix(Sparse_Matrix<_T>* poMatrix, int iItem_Count, int iMax_Order);
template<typename _T>void Compact_Sparse_Matrix(Sparse_Matrix<_T>* poMatrix);
template<typename _T>void Free_Sparse_Matrix(Sparse_Matrix<_T>* poMatrix);
template<typename _T>void Reset_Sparse_Matrix(Sparse_Matrix<_T>* poA);
template<typename _T>void Set_Value(Sparse_Matrix<_T>* poMatrix, int x, int y, _T fValue);
template<typename _T>void Matrix_Multiply(Sparse_Matrix<_T> A, Sparse_Matrix<_T> B, Sparse_Matrix<_T>* poC, int bCompact=1);
template<typename _T>void Matrix_Multiply(Sparse_Matrix<_T> A, _T a);
template<typename _T>void Resize_Matrix(Sparse_Matrix<_T>* poA, int iNew_Item_Count);
template<typename _T>void Matrix_Transpose_1(Sparse_Matrix<_T> A, Sparse_Matrix<_T>* poAt);
template<typename _T>void Matrix_Minus(Sparse_Matrix<_T> oA, Sparse_Matrix<_T> oB, Sparse_Matrix<_T>* poC, int bCompact=1);
template<typename _T>void Matrix_Add(Sparse_Matrix<_T> oA, Sparse_Matrix<_T> oB, Sparse_Matrix<_T>* poC, int bCompact=1);
template<typename _T>void Sparse_2_Dense(Sparse_Matrix<_T> oA, _T B[]);
template<typename _T>void Dense_2_Sparse(_T A[], int m, int n, Sparse_Matrix<_T>* poA, int bCompact=1);
template<typename _T>void Add_I_Matrix(Sparse_Matrix<_T>* poA, int* pbSuccess = NULL, _T ramda=1.f);
template<typename _T>void Solve_Linear_Gause(Sparse_Matrix<_T> oA, _T B[], _T X[], int* pbSuccess = NULL);
template<typename _T>void Solve_Linear_Gause_1(Sparse_Matrix<_T>oA1, _T B[], _T X[], int* pbSuccess = NULL);
template<typename _T>void Solve_Linear_Gause_2(Sparse_Matrix<_T>oA1, _T B[], _T X[], int* pbSuccess = NULL);
template<typename _T>void Cholosky_Decompose(Sparse_Matrix<_T> oA, Sparse_Matrix<_T> *poB,int *pbSuccess=NULL);

template<typename _T>void Re_Arrange_Sparse_Matrix(Sparse_Matrix<_T>* poA);
template<typename _T>void Get_Inv_Matrix_Row_Op(Sparse_Matrix<_T> oA, Sparse_Matrix<_T>* poA_Inv, int* pbSuccess=NULL);
//**************************稀疏矩阵*******************************

template<typename _T>void Vector_Add(_T A[], _T B[], int n, _T C[]);
template<typename _T>void Vector_Minus(_T A[], _T B[], int n, _T C[]);
template<typename _T>void Vector_Multiply(_T A[], int n,_T a,  _T B[]);
template<typename _T> void Matrix_Multiply(_T* A, int ma, int na, _T a, _T* C);
template<typename _T>void Matrix_Multiply_3x1(_T A[3 * 3], _T B[3], _T C[3]);

template<typename _T> void Matrix_Multiply(_T* A, int ma, int na, _T* B, int nb, _T* C);
template<typename _T> void Transpose_Multiply(_T A[], int m, int n, _T B[], int bAAt=1);
template<typename _T>void Matrix_Add(_T A[], _T B[], int iOrder, _T C[]);
template<typename _T>void Crop_Matrix(_T Source[], int m, int n, int x, int y, int w, int h, _T Dest[], int iDest_Stride=0);

void Matrix_x_Vector(double A[3][3], double X[3], double Y[3]);
void QR_Decompose(double A1[3][3], double Q[3][3], double R[3][3]);
void QR_Decompose(float* A, int ma, int na, float* Q, float* R);
void QR_Decompose(float* A, int na, float* R, float* Q, int* pbSuccess = NULL, int* pbDup_Root = NULL);

template<typename _T>int bInverse_Power(_T A[], int n, _T* pfEigen_Value, _T Eigen_Vector[], _T eps = 1e-9f);
template<typename _T>int bInverse_Power(_T A[3][3], _T* pfEigen_Value, _T Eigen_Vector[3], _T eps = 1e-9f);

template<typename _T>void LU_Decompose(_T A[], _T L[], _T U[], int n, int* pbSuccess=NULL);
template<typename _T>int bTest_LU(_T* A, _T* L, _T* U, int n);
template<typename _T>void Power_Method(_T A[], int n, _T Eigen_Vector[], _T* pfEigen_Value);
template<typename _T>int bTest_Eigen(_T A[], int n, _T fEigen_Value, _T Eigen_Vector[], _T eps = 1e-7);

template<typename _T> void Normalize(_T V[], int n, _T V_1[]);
template<typename _T>void Softmax(_T A[], int n, _T B[]);
template<typename _T>void Softmax(_T A[], int m, int n, int iPadding, _T B[]);
template<typename _T>void Homo_Normalize(_T V0[], int n, _T V1[]);	//这就是个傻逼方法，用来欺骗template

template<typename _T> void Get_Inv_Matrix_Row_Op(_T* pM, _T* pInv, int iOrder, int* pbSuccess=NULL);	//矩阵求逆
//template<typename _T>void Get_Inv_Matrix_Row_Op_1(_T A[], _T A_Inv[], int iOrder, int* pbSuccess=NULL);	//列主元法矩阵求逆
template<typename _T>void Get_Inv_Matrix_Row_Op_2(_T A[], _T A_Inv[], int iOrder, int* pbSuccess=NULL);

void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess);	//用伴随矩阵求逆
template<typename _T>_T fGet_Determinant(_T* A, int iOrder);			//求行列式

template<typename _T> _T fGet_Mod(_T V[], int n);//求向量的模

template<typename _T> _T fGet_Distance(_T V_1[], _T V_2[], int n);	//求两向量距离
float fGet_Theta(float v0[], float v1[], float Axis[], int n);		//求两个向量之间的夹角
int iRank(double A[3][3]);								//求3x3矩阵的秩
template<typename _T>int iGet_Rank(_T* A, int m, int n);				//求一般矩阵的秩，用初等行变换法
template<typename _T>void Cross_Product(_T V0[], _T V1[], _T V2[]);	//求外积
template<typename _T>_T fDot(_T V0[], _T V1[], int iDim);		//求内积，点积

//以下为一组仿射变换矩阵的生成
template<typename _T> void Gen_Roation_Matrix_2D(_T Rotation_Center[2], _T theta, _T R[3 * 3]);
template<typename _T>void Gen_Rotation_Matrix_2D(_T R[2 * 2], float fTheta);

template<typename _T>void Gen_Rotation_Matrix(_T Axis[3], _T fTheta, _T R[]);//根据旋转轴与旋转角度生成一个旋转矩阵，此处用列向量，往后一路左乘
template<typename _T>void Gen_Translation_Matrix(_T Offset[3], _T T[]);	//平移变换
void Gen_Scale_Matrix(float Scale[3], float T[]);			//比例变换

//四元数,旋转矩阵，旋转向量互换函数
template<typename _T>void Quaternion_2_Rotation_Matrix(_T Q[4], _T R[]);	//四元数转旋转矩阵
template<typename _T>void Quaternion_2_Rotation_Vector(_T Q[4], _T V[4]);	//四元数转旋转向量
void Quaternion_Add(float Q_1[], float Q_2[], float Q_3[]);	//四元数加
void Quaternion_Minus(float Q_1[], float Q_2[], float Q_3[]);	//四元数减，其实这两个函数可以用普通向量加减
void Quaternion_Conj(float Q_1[], float Q_2[]);				//简单求个共轭
void Quaternion_Multiply(float Q_1[], float Q_2[], float Q_3[]);	//四元数乘法
void Quaternion_Inv(float Q_1[], float Q_2[]);	//四元数求逆
template<typename _T>void Rotation_Matrix_2_Quaternion(_T R[], _T Q[]);		//旋转矩阵转换四元数
template<typename _T>void Rotation_Matrix_2_Vector(_T R[3*3], _T V[4]);		//旋转矩阵转旋转向量

template<typename _T>void Rotation_Vector_4_2_Matrix(_T V[4], _T R[3 * 3]);		//旋转向量转换旋转矩阵
template<typename _T>void Rotation_Vector_3_2_Matrix(_T V[3], _T R[3 * 3]);		//旋转向量转换旋转矩阵
template<typename _T>void Rotation_Vector_2_Quaternion(_T V[4], _T Q[4]);	//旋转向量转换四元数
template<typename _T>void Rotation_Vector_3_2_4(_T V[], _T V1[]);	
template<typename _T>void Rotation_Vector_4_2_3(_T V[], _T V1[]);
template<typename _T>void Hat(_T V[], _T M[]);				//构造反对称矩阵

void Vee(float M[], float V[3]);			//反对称矩阵到旋转向量
template<typename _T>void se3_2_SE3(_T Ksi[6], _T T[]);		//从se3向量到SE3矩阵的转换
template<typename _T>void SE3_2_se3(_T T[4 * 4], _T ksi[6]);
template<typename _T>void SE3_2_se3(_T Rotation_Vector[4], _T t[3], _T Ksi[6]);	//一个旋转向量加一个位移向量构造出一个Ksi

void SIM3_2_sim3(float Rotation_Vector[], float t[], float s, float zeta[7]);
void sim3_2_SIM3(float zeta[7], float Rotation_Vector[4], float t[], float* ps);
template<typename _T>void Get_J_by_Rotation_Vector(_T Rotation_Vector_4[4], _T J[]);
template<typename _T> void Gen_Ksi_by_Rotation_Vector_t(_T Rotation_Vector[4], _T t[3], _T Ksi[6], int n=4);

template<typename _T> void Exp_Ref(_T A[], int n, _T B[],_T eps = (_T)1e-10);
template<typename _T>void Gen_Homo_Matrix_2D(_T R[2 * 2], _T t[2], _T T[3 * 3]);
template<typename _T>void Gen_Homo_Matrix(_T R[3*3], _T t[3], _T c2w[3*3]);			//用旋转坐标与位移坐标构成一个SE3变换矩阵
template<typename _T>void Gen_Homo_Matrix_1(_T Rotation_Vector[3], _T t[3], _T c2w[]);

template<typename _T>void Get_R_t(_T T[4 * 4], _T R[3 * 3]=NULL, _T t[3]=NULL);
template<typename _T>void Matrix_2_R(_T Hr[3 * 3], _T R[3 * 3]);

void Gen_Homo_Matrix(float R[], float t[], float s, float M[]);	//用旋转,位移，缩放构成一个变换矩阵

//取代透视变换，一步计算
void Perspective(float Pos_Source[3], float h[3], float Pos_Dest[3]);
void Perspective_Camera(float Pos_Source[3], float h[3], float Pos_Dest[3]);

template<typename _T> void svd_3(_T* A,SVD_Info oInfo, int* pbSuccess=NULL, double eps= 2.2204460492503131e-15);
template<typename _T>void Test_SVD(_T A[], SVD_Info oSVD, int* piResult=NULL, double eps= 2.2204460492503131e-15);
void Free_SVD(SVD_Info* poInfo);

//注意，用法为 SVD_Alloc<_T>(...)
template<typename _T>void SVD_Alloc(int h, int w, SVD_Info* poInfo, _T* A=NULL);

//一组多元多项式函数
void Disp(Polynormial oPoly, int iElem_No=-1);
void Free_Polynormial(Polynormial* poPoly);				//释放多项式所占内存
void Init_Polynormial(Polynormial* poPoly, int iElem_Count, int iMax_Term_Count);	//初始化
void Add_Poly_Term(Polynormial* poPoly, float fCoeff, int first, ...);
void Get_Derivation(Polynormial* poSource, int iElem, Polynormial* poDest=NULL);	//多项式对xi求导
float fGet_Polynormial_Value(Polynormial oPoly, float x[]);	//代入x求多项式值

template<typename _T>void Copy_Matrix_Partial(_T Source[], int iSource_Stride, int x, int y, _T Dest[], int m, int n);
template<typename _T>void Copy_Matrix_Partial(_T Source[], int m, int n, _T Dest[], int iDest_Stride, int x, int y);
template<typename _T>void Matrix_Add_Partial(_T Source[], int iSource_Stride, int x1, int y1, int w, int h, _T Dest[], int iDest_Stride, int x2, int y2);

float fGet_e();

template<typename _T>_T fGet_Tr(_T M[], int iOrder);
