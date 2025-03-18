//这个文件管所有独立于特殊平台的通用函数
#pragma once
#ifndef WIN32
	#define WIN32
#endif

#include "stdio.h"
#include "stdlib.h"
#include "memory.h"
#include "sys/timeb.h"
#include "math.h"
#include <mutex>
//using namespace std;

extern "C"
{
	#include "Buddy_System.h"
}

#ifndef WIN32
#include <unistd.h>	//给sleep用
#endif

#define Abs(A) ((A)>=0?(A):(-(A)))
#ifndef Min 
#define Min(A,B)( A<=B?A:B)
#endif
#ifndef Max
#define Max(A,B)(A>=B?A:B)
#endif
#ifndef Clip3
#define Clip3(x,y,z)  ( (z)<(x)?(x): (z)>(y)?(y):(z))
#endif

#define ALIGN_SIZE_8(iSize) ((( (unsigned long long)(iSize)+7)>>3)<<3) 
#define ALIGN_SIZE_128(iSize) ((( (unsigned long long)(iSize)+127)>>7)<<7) 
#define ALIGN_SIZE_1024(iSize) ((( (unsigned long long)(iSize)+1023)>>10)<<10) 
#define ALIGN_ADDR_128(pAddr) (((((unsigned long long)(pAddr))+127)>>7)<<7)

#define bGet_Bit(pBuffer, iBit_Pos) (pBuffer)[(iBit_Pos) >> 3] & (1 << ((iBit_Pos) & 0x7))
//int bGet_Bit(unsigned char* pBuffer, int iBit_Pos)
//{	return pBuffer[iBit_Pos >> 3] & (1 << (iBit_Pos & 0x7));}
#define Set_Bit(pBuffer, iBit_Pos) \
{\
	(pBuffer)[(iBit_Pos) >> 3] |= (1 << ((iBit_Pos) & 0x7)); \
}

template<typename _T> struct Point_2D {
	unsigned int m_iCamera_Index;
	unsigned int m_iPoint_Index;
	_T m_Pos[2];
};

typedef struct BitPtr {
	unsigned char* m_pBuffer;
	int m_iCur;		//当前字节,位于m_oBuffer[]中的第m_iCur Byte.
	int m_iBitPtr;	//当前字节中的当前位
	int m_iEnd;		//m_Buffer的合法数据是有范围的, 合法数据的最后一个字节位于m_iEnd-1, 如果m_iCur>=m_iEnd则为越界
}BitPtr;

extern Mem_Mgr oMatrix_Mem;;

//void Set_Bit(unsigned char* pBuffer, int iBit_Pos)
//{	pBuffer[iBit_Pos >> 3] |= (1 << (iBit_Pos & 0x7)); }
const int WriteBitsMask[] = { 0,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFF };
const int WriteBitsMask2[] = { 0xFF,0x7F,0x3F,0x1F,0x0F,0x07,0x03,0x01 };

#define WriteBits2(oBitPtr,iLen,iValue1)	\
{\
	unsigned int iValue=(iValue1)<<(32- (oBitPtr).m_iBitPtr-(iLen));	\
	unsigned char *pCur=(oBitPtr).m_pBuffer+ (oBitPtr).m_iCur;	\
	unsigned int iTemp1=(oBitPtr).m_iBitPtr+iLen;\
	*pCur= (*pCur &WriteBitsMask[(oBitPtr).m_iBitPtr]) | ((iValue>>24)&WriteBitsMask2[(oBitPtr).m_iBitPtr]);	\
	if(iTemp1>=8)									\
	{											\
		pCur[1]=(iValue & 0x00FF0000)>>16;		\
		pCur[2]=(iValue & 0x0000FF00)>>8;		\
		pCur[3]=(iValue & 0x000000FF);			\
		(oBitPtr).m_iCur+=iTemp1>>3;			\
		(oBitPtr).m_iBitPtr=iTemp1 & 0x7;		\
	}else										\
		(oBitPtr).m_iBitPtr=iTemp1;			\
}

unsigned long long iGet_File_Length(char* pcFile);
unsigned long long iGet_Tick_Count();
int iGet_File_Count(const char* pcPath);	//获取一个目录所有文件
void Get_All_File(const char* pcPath, char* pBuffer);
int bSave_Bin(const char* pcFile, float* pData, int iSize);
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer, int* piSize=NULL);
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer, int iSize = 0, int bNeed_Malloc = 1, int iFrame_No = 0);
int bLoad_Text_File(const char* pcFile, char** ppBuffer, int* piSize=NULL);
int bSave_Raw_Data(const char* pcFile, unsigned char* pBuffer, int iSize);

//上三角坐标转换为索引值
int iUpper_Triangle_Cord_2_Index(int x, int y, int w);

//上三角有效元数个数
int iGet_Upper_Triangle_Size(int w);

//一组有的没的随机数生成
int iRandom(int iStart, int iEnd);
int iRandom();

//早晚得废
//以后废弃的临时函数
template<typename _T>
void Temp_Load_Match_Point(_T(**ppPoint_1)[2], _T(**ppPoint_2)[2], int* piCount)
{
	_T(*pPoint_1)[2], (*pPoint_2)[2];
	int i, iCount = (int)iGet_File_Length((char*)"c:\\tmp\\2.bin") / (4 * sizeof(float));
	pPoint_1 = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
	pPoint_2 = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
	FILE* pFile = fopen("c:\\tmp\\1.bin", "rb");
	for (i = 0; i < iCount; i++)
	{
		float Data[2];
		fread(Data, 1, 2 * sizeof(float), pFile);
		pPoint_1[i][0] = (_T)Data[0];
		pPoint_1[i][1] = (_T)Data[1];
		fread(Data, 1, 2 * sizeof(float), pFile);
		pPoint_2[i][0] = (_T)Data[0];
		pPoint_2[i][1] = (_T)Data[1];		
	}
	fclose(pFile);
	*ppPoint_1 = pPoint_1, * ppPoint_2 = pPoint_2;
	*piCount = iCount;
}

void Init_BitPtr(BitPtr* poBitPtr, unsigned char* pBuffer, int iSize);
int iGetBits(BitPtr* poBitPtr, int iLen);

//解决两组基本数据类型的第n大与及快速排序，待优化
template<typename _T> _T oGet_Nth_Elem(_T Seq[], int iCount, int iNth);
template<typename _T> void Quick_Sort(_T Seq[], int iStart, int iEnd);
template<typename _T>int bSave_PLY(const char* pcFile, _T Point[][3], int iPoint_Count,unsigned char Color[][3]=NULL, int bText=1);

//Temp code 
template<typename _T>void Temp_Load_File(const char* pcFile, _T(**ppPoint_3D_1)[3], _T(**ppPoint_3D_2)[3], int* piCount);
template<typename _T>void Temp_Load_File_1(const char* pcFile, _T(**ppPoint_3D_1)[3], _T(**ppPoint_3D_2)[3], int* piCount);
template<typename _T>void Temp_Load_File_2(int* piCameta_Count, int* piPoint_Count, int* piObservation_Count, Point_2D<_T>** ppPoint_2D, _T(**ppPoint_3D)[3], _T(**ppCamera)[3 * 3]);
//以下函数也没啥用，第九讲用
template<typename _T>void Normalize(_T Point_3D[][3], Point_2D<_T> Point_2D[], int iPoint_Count, _T Camera[][9], int iCamera_Count);

//分配内存
//Input： iSize: 要分配内存的字节数
//return: void 指针，用户自己转换为自己的类型
//void* pMalloc(Mem_Mgr* poMem_Mgr, unsigned int iSize);
//void* pMalloc_1(Mem_Mgr* poMem_Mgr, unsigned int iSize);

void* pMalloc(unsigned int iSize);

void Shrink(void* p, unsigned int iSize);
void Free(void* p);
void Disp_Mem();

////释放一项
////Input:	p: 原来分配的指针
//void Free(Mem_Mgr* poMem_Mgr, void* p);
//
////初始化内存池
////Input:	iSize：			整个内存池管理的内存大小，小于等于最后能分配的内存
////			iBlock_Size:	用户观点的用户块块大小
////			iMax_Piece_Count: 最多能分配多少片
////return:	若成功poMem_Mgr->m_pBuffer有数据，否则为NULL，以此判断是否成功即可
//void Init_Mem_Mgr(Mem_Mgr* poMem_Mgr, unsigned long long iSize, int iBlock_Size, int iMax_Piece_Count);
//
////释放整个内存池
//void Free_Mem_Mgr(Mem_Mgr* poMem_Mgr);
//
////将原来分配的大小缩小到iSize，原来分配大小必须大于等于iSize，否则啥也不干
////Input: iSize: 新的大小
//void Shrink(Mem_Mgr* poMem_Mgr, void* p, unsigned int iSize);
//
////将原来p分配到的内存扩大到iSize，入宫iSize<=原来大小，则啥也不干
////Input:	p: 原来内存指针
////			iSize: 新的大小
//int bExpand(Mem_Mgr* poMem_Mgr, void* p, unsigned int iSize);
//
////显示内存的使用情况，对所有的Item按position排个序
//void Disp_Mem(Mem_Mgr* poMem_Mgr, int iLayer_Count);