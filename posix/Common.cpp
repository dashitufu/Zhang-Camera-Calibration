/*****************************网络函数*************************************/
#ifdef WIN32
	#define Sleep	Sleep
#elif ESP32
	#include "arduino.h"
	#define Sleep delay
#else
	#define Sleep(ms) nanosleep((ms)*1000))
#endif

#ifdef WIN32
	#include <direct.h>
	#include "WinSock2.h"
	#include <conio.h> // Windows 特有的控制台输入输出头文件 [2]
	#include <ws2tcpip.h>
	#define Close_Socket closesocket
#else
	#include <sys/socket.h>
	#include <unistd.h>
	#include <dirent.h>
	#define Close_Socket close
#endif

#pragma comment(lib, "ws2_32.lib")
/*****************************网络函数*************************************/

#include "stdio.h"
#include "Common.h"
#include <mutex>

#ifdef WIN32
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#include "io.h"
#include "windows.h"
#include <sys/timeb.h>  // 必须引入此标准 C 库头文件
#endif
#ifndef WIN32
#include "sys/time.h"
#endif

extern "C"
{	
#include "Buddy_System.h"
}

Mem_Mgr_Ex oMem_Mgr;
using namespace std;

void SB_Common()
{//template实例化，只对vc有效
	fGet_Distance((double*)NULL, (double*)NULL, 0);
	fGet_Distance((float*)NULL, (float*)NULL, 0);

	bRead_PLY_File<float>(NULL, NULL, NULL, NULL, NULL);
	bRead_PLY_File<double>(NULL, NULL, NULL, NULL, NULL);
	bRead_PLY_File<int>(NULL, NULL, NULL, NULL, NULL);

	bSave_PLY(NULL, (double(*)[3])NULL, 0);
	bSave_PLY(NULL, (float(*)[3])NULL, 0);

	oGet_Nth_Elem((double*)NULL, 0, 0);
	oGet_Nth_Elem((float*)NULL, 0, 0);
	oGet_Nth_Elem((int*)NULL, 0, 0);

	Quick_Sort((double*)NULL, 0, 0);
	Quick_Sort((float*)NULL, 0, 0);
	Quick_Sort((int*)NULL, 0, 0);

	Get_Random_Norm_Vec((double*)NULL, 0);
	Get_Random_Norm_Vec((float*)NULL, 0);

	Init_Point_Cloud((Point_Cloud<float>*)NULL, 0, 0);
	Init_Point_Cloud((Point_Cloud<double>*)NULL, 0, 0);

	Free_Point_Cloud((Point_Cloud<float>*)NULL);
	Free_Point_Cloud((Point_Cloud<double>*)NULL);

	bSave_PLY(NULL, Point_Cloud < double>{});
	bSave_PLY(NULL, Point_Cloud < float>{});

	Draw_Line((Point_Cloud<double>*)NULL, (double)0, (double)0, (double)0, (double)0, (double)0, (double)0);
	Draw_Line((Point_Cloud<float>*)NULL, (float)0, (float)0, (float)0, (float)0, (float)0, (float)0);

	Draw_Sphere((Point_Cloud<double>*)NULL, (double)0, (double)0, (double)0);
	Draw_Sphere((Point_Cloud<float>*)NULL, (float)0, (float)0, (float)0);

	Draw_Rect<float>(NULL, 0, 0, 0, 0, 0);

	fAngle_2_Radian<float>(0);
	fAngle_2_Radian<double>(0);

	fRadian_2_Angle<float>(0);
	fRadian_2_Angle<double>(0);

	Get_Nearest_Point_Ref<float>(NULL, NULL, NULL);
	Get_Nearest_Point_Ref<double>(NULL, NULL, NULL);

	Get_Nearest_Point<float>(NULL, NULL, NULL);
	Get_Nearest_Point<double>(NULL, NULL, NULL);
	Get_Nearest_Point<int>(NULL, NULL, NULL);

	Build_Tree<float>((float(*)[3])NULL, 0, NULL);
	Build_Tree<double>((double(*)[3])NULL, 0, NULL);
	Build_Tree<int>((int(*)[3])NULL, 0, NULL);

	Build_Tree<float>((KD_Point_3D<float>*)NULL, 0, NULL);
	Build_Tree<double>((KD_Point_3D<double>*)NULL, 0, NULL);

	Free_KD_Tree<float>(NULL);
	Free_KD_Tree<double>(NULL);
	Free_KD_Tree<int>(NULL);

	Init_Queue<int>(NULL, 0);
	In_Queue<int>(NULL, 0);
	Out_Queue<int>(NULL);
	Free_Queue<int>(NULL);
}

void Lock_Semaphore_For_Thread(Semaphore_For_Thread* ps, int iThreadID)
{//简单地加个排斥锁了事，注意：这个函数在单线程中不会阻塞，例：
	if (ps && (mutex*)ps->m_poLock)
		((mutex*)ps->m_poLock)->lock();
	return;
}
void Unlock_Semaphore_For_Thread(Semaphore_For_Thread* ps, int iThreadID)
{
	if (ps && (mutex*)ps->m_poLock)
		((mutex*)ps->m_poLock)->unlock();
	return;
}

unsigned long long iGet_Tick_Count()
{//求当前的毫秒级Tick Count。之所以不搞微秒级是因为存在时间片问题，即使毫秒级也不准，很有可能Round到16毫秒一个时间片，聊胜于无
#ifndef WIN32
	timeval oTime;
	gettimeofday(&oTime, NULL);
	unsigned long long tTime = oTime.tv_sec * 1000000 + oTime.tv_usec;
	return tTime / 1000;
#else
	timeb tp;
	ftime(&tp);
	return (unsigned long long) ((unsigned long long)tp.time * 1000 + tp.millitm);
#endif
}

template void Disp_Fillness(double A[], int m, int n, const char Caption[]);
template void Disp_Fillness(float A[], int m, int n, const char Caption[]);
template<typename _T>void Disp_Fillness(_T A[], int m, int n, const char Caption[])
{//显示一个矩阵的非零情况
	if (Caption)
		printf("%s\n", Caption);

	for (int y = 0; y < m; y++)
	{
		for (int x = 0; x < n; x++)
		{
			if (A[y * n + x] != 0)
				printf("*");
			else
				printf(".");
		}
		printf("\n");
	}
}

/****************************位操作函数有******************************/
void Init_BitPtr(BitPtr* poBitPtr, unsigned char* pBuffer, int iSize)
{
	poBitPtr->m_iBitPtr = poBitPtr->m_iCur = 0;
	poBitPtr->m_iEnd = iSize;
	poBitPtr->m_pBuffer = pBuffer;
}

int iGetBits(BitPtr* poBitPtr, int iLen)
{
	unsigned int iValue, iCur = poBitPtr->m_iCur, iShiftBits;
	int iBitLeft = iLen;
	//先搞定第一字节
	iValue = ((poBitPtr->m_pBuffer[poBitPtr->m_iCur] << poBitPtr->m_iBitPtr) & 0xFF) << 24;
	iBitLeft -= (8 - poBitPtr->m_iBitPtr);
	iCur++;
	iShiftBits = 16 + poBitPtr->m_iBitPtr;
	while (iBitLeft > 0)
	{
		if (iShiftBits > 0)
			iValue |= (poBitPtr->m_pBuffer[iCur] & 0xFF) << iShiftBits;
		iBitLeft -= 8;
		iShiftBits -= 8;
		iCur++;
	}
	iValue = iLen ? iValue >> (32 - iLen) : 0;
	poBitPtr->m_iCur += (poBitPtr->m_iBitPtr + iLen) >> 3;
	poBitPtr->m_iBitPtr = (iLen + poBitPtr->m_iBitPtr) & 7;
	return iValue;
}
/****************************位操作函数有******************************/

/*****************************上三角函数*******************************/
int iGet_Upper_Triangle_Size(int w)
{
	int iSize;
	iSize = w * (w - 1) / 2;
	return iSize;
}

int iUpper_Triangle_Cord_2_Index(int x, int y, int w)
{//一个上三角矩阵，给定(x,y)坐标，转换为索引值, 注意，w为上三角矩形的列数, 不是上三角第一行有效元数个数
	int iPos;
	//比如 w=4, y=2 x=3 Index=7
	if (x <= y || y >= w - 1)
		return -1;
	iPos = ((w - 1) + (w - y)) * y / 2 + x - y - 1;
	return iPos;
}
/*****************************上三角函数*******************************/

/******************************一组随机数生成函数*****************************/
unsigned int iGet_Random_No()
{//伪随机	(789, 123)	(389,621) 已通过，可以自定义自己的种子
#define b 389
#define c 621
	static unsigned int iNum = 0xFFFFFFF;	//GetTickCount();	//种子
	return iNum = iNum * b + c;
#undef b
#undef c
}
template<typename _T>void Get_Random_Norm_Vec(_T V[], int n)
{//生成一个归一化向量
	double fTotal = 0;
	int i;
	for (i = 0; i < n; i++)
	{
		unsigned int iValue = iGet_Random_No();
		V[i] = (_T)iValue;
		fTotal += (unsigned long long)iValue * iValue;
	}
	_T fMod = (_T)sqrt((double)fTotal);
	for (i = 0; i < n; i++)
		V[i] /= fMod;
}

int iRandom(int iStart, int iEnd)
{//从iStart到iEnd之间随机出一个数字，由于RandomInteger太傻逼，没有必要把时间浪费在傻逼身上
	return iStart + iGet_Random_No() % (iEnd - iStart + 1);
}
int iRandom()
{//尝试搞一个无参数获得随机数的函数，计算办法为 iRandom_No= iRandom_No*a + c; 最后取模 0x7FFFFFFF
#define m 1999999973			//基于模为素数能令散列更均匀的理论
	static unsigned int a = (unsigned int)iGet_Tick_Count(); //1103515245;		//1103515245为很好的初始值，但选取静态值变成了确定问题
	static unsigned int c = 2347;	// 12345;
	static unsigned long long iRandom_No = 1;
	iRandom_No = (iRandom_No * a + c) % m;	//求和不溢出，再求模
	return (int)iRandom_No;
#undef m
}
unsigned long long iGet_Random_No_cv(unsigned long long* piState)
{
	*piState = (unsigned long long)(unsigned) * piState * 4164903690U + (unsigned)(*piState >> 32);
	return *piState;
}
/******************************一组随机数生成函数*****************************/
/******************************Quick Sort及相关*********************************/
template<typename _T>int iQuick_Sort_Partition(_T pBuffer[], int left, int right)
{//小到大的顺序
 //Real fRef;
	_T iValue, oTemp;
	int pos = right;

	//试一下将中间元素交换到头，多一步可能挽救了极端差形态
	oTemp = pBuffer[right];
	pBuffer[right] = pBuffer[(left + right) >> 1];
	pBuffer[(left + right) >> 1] = oTemp;

	right--;
	iValue = pBuffer[pos];
	while (left <= right)
	{
		while (left < pos && pBuffer[left] <= iValue)
			left++;
		while (right >= 0 && pBuffer[right] > iValue)
			right--;
		if (left >= right)
			break;
		oTemp = pBuffer[left];
		pBuffer[left] = pBuffer[right];
		pBuffer[right] = oTemp;
	}

	oTemp = pBuffer[left];
	pBuffer[left] = pBuffer[pos];
	pBuffer[pos] = oTemp;

	return left;
}
template<typename _T>int iAdjust_Left(_T* pStart, _T* pEnd)
{
	_T oTemp, * pCur_Left = pEnd - 1,
		* pCur_Right;
	_T oRef = *pEnd;

	//为了减少一次判断，此处先扫过去
	while (pCur_Left >= pStart && *pCur_Left == oRef)
		pCur_Left--;
	pCur_Right = pCur_Left;
	pCur_Left--;

	while (pCur_Left >= pStart)
	{
		if (*pCur_Left == oRef)
		{
			oTemp = *pCur_Left;
			*pCur_Left = *pCur_Right;
			*pCur_Right = oTemp;
			pCur_Right--;
		}
		pCur_Left--;
	}
	return (int)(pCur_Right - pStart);
}
template<typename _T> _T oGet_Nth_Elem(_T Seq[], int iCount, int iStart, int iEnd, int iNth, int bAdjust_Pos = 0)
{//取第n大元素，用Quick_Sort的Partition做
	int iPos;
	if (iStart < iEnd)
	{
		iPos = iQuick_Sort_Partition(Seq, iStart, iEnd);
		if (iPos > iNth)//那么第n大元素在左分区内
			return oGet_Nth_Elem(Seq, iCount, iStart, iPos - 1, iNth);
		else if (iPos < iNth)//第n大元素在右分区内
			return oGet_Nth_Elem(Seq, iCount, iPos + 1, iEnd, iNth);
		else //找到了，正好在iPos中
			return Seq[iPos];
	}
	else
	{//此时又分奇偶两种情况
		if (iCount & 1 || !bAdjust_Pos)
			return Seq[iStart];	//奇数好办，返回便是
		else //偶数的还要往前找最大值
		{
			_T fMax = Seq[iStart - 1];
			for (int i = iStart - 2; i >= 0; i--)
				if (Seq[i] > fMax)
					fMax = Seq[i];
			return (_T)((Seq[iStart] + fMax) / 2.f);
		}
	}
}

template<typename _T> _T oGet_Nth_Elem(_T Seq[], int iCount, int iNth)
{
	return oGet_Nth_Elem(Seq, iCount, 0, iCount - 1, iNth);
}
template<typename _T> void Quick_Sort(_T Seq[], int iStart, int iEnd)
{
	int iPos, iLeft;
	if (iStart < iEnd)
	{
		iPos = iQuick_Sort_Partition(Seq, iStart, iEnd);
		//如果iPos>=iEnd, 则遇上极差形态的组了，这时做一个推进，找出所有与Seq[iEnd]相等的项目，赶到右边
		if (iPos >= iEnd)
		{
			iLeft = iAdjust_Left(Seq + iStart, Seq + iPos);
			iLeft = iStart + iLeft;
		}
		else
			iLeft = iPos - 1;

		if (iStart < iLeft)
			Quick_Sort(Seq, iStart, iLeft);

		Quick_Sort(Seq, iPos + 1, iEnd);
	}
}
/******************************Quick Sort及相关*********************************/


template<typename _T>_T fAngle_2_Radian(_T fAngle)
{
	return fAngle * (_T)PI / (_T)180;
}
template<typename _T>_T fRadian_2_Angle(_T fRadian)
{
	return fRadian * (_T)180 / (_T)PI;
}
template<typename _T> _T fGet_Distance(_T V_1[], _T V_2[], int n)
{//求两向量的距离，用平方表示，不开方
	int i;
	_T fSum;
	for (i = 0, fSum = 0; i < n; i++)
		fSum += (V_1[i] - V_2[i]) * (V_1[i] - V_2[i]);
	return fSum;
}
int bGet_Line(FILE* pFile, char* pLine)
{
	char* pCur = pLine;
	do {
		*pCur = fgetc(pFile);
	} while (*pCur == '\n' || *pCur == '\r');
	if (*pCur == EOF)
		return 0;
	while (*pCur != '\n' && *pCur != '\r' && *pCur != EOF)
	{
		pCur++;
		*pCur = fgetc(pFile);
	}
	*pCur = 0;
	return 1;
}
int iRead_Line(FILE* pFile, char Line[], int iLine_Size)
{//从文件读入一行,返回一行大小
	int iResult;
	while (iResult = (int)fread((void*)Line, 1, 1, pFile))
		if (Line[0] != '\r' && Line[0] != '\n' && Line[0] != '\t' && Line[0] != ' ')
			break;
	if (!iResult)
		return 0;
	int i = 0;
	while (iResult = (int)fread((void*)&Line[++i], 1, 1, pFile))
	{
		if (Line[i] == '\r' || Line[i] == '\n')
			break;
		if (i > iLine_Size)
			return 0;
	}
	if (i < iLine_Size)
		Line[i] = 0;

	return iResult || i ? i : 0;
}
int bGet_Value(char* pText, int iSize, const char* pKey, char* pValue)
{//Purpose: 从pText中找到pKey, 然后得到其Value. pText的格式形如 Key=Value
 //Return: 0 if Fail, 1 if success
	char* pPos, * pEnd, * pValue1;
	char* pPos_Space, * pCur;
	int bError;
	while (1)
	{
		pPos = strstr(pText, pKey);	//寻找pKey串位置
		if (!pPos)
			return 0;
		pPos_Space = strstr(pPos, "=");
		if (!pPos_Space)
			return 0;
		for (bError = 0, pCur = pPos + strlen(pKey); pCur < pPos_Space; pCur++)
		{
			if (*pCur != ' ')
			{
				bError = 1;
				break;
			}
		}
		if (bError)
		{
			pText = pPos + 1;
			continue;
		}
		else
			pPos = pPos_Space + 1;
		//pPos++;
		while (*pPos == ' ' || *pPos == '\t')
			pPos++;
		for (pValue1 = pValue, pEnd = pPos; *pEnd != '\n' && *pEnd != '\r' && *pEnd != 0; pEnd++, pValue1++)
			*pValue1 = *pEnd;
		*pValue1 = 0;
		break;
	}
	return 1;
}

int bStricmp(char* pStr_0, char* pStr_1)
{//由于Linux与Windows不一致，被迫搞一个同一无大小写比较函数
#ifndef WIN32
#include "strings.h"
	int iResult = strcasecmp(pStr_0, pStr_1);
#else
	int iResult = _stricmp(pStr_0, pStr_1);
#endif
	if (iResult == 0)
		return 1;
	else
		return 0;
}

/******************************一组KD_Tree函数**************************/
template<typename _T>static void Plane_Split(KD_Point_3D<_T> Point[],/* int Index[],*/ int iCount, int iDim, float fMid_Value, int* piLim1, int* piLim2)
{//此处相当于Partition
	int iLeft = 0;
	int iRight = iCount - 1;
	while (1)
	{
		while (iLeft <= iRight && Point[iLeft].m_Pos[iDim] < fMid_Value)
			++iLeft;
		while (iLeft <= iRight && Point[iRight].m_Pos[iDim] >= fMid_Value)
			--iRight;

		if (iLeft > iRight)
			break;

		//Swap
		KD_Point_3D<_T> oTemp = Point[iLeft];
		Point[iLeft] = Point[iRight];
		Point[iRight] = oTemp;

		//std::swap(Point[iLeft], Point[iRight]);
		++iLeft; --iRight;
	}

	*piLim1 = iLeft;
	iRight = iCount - 1;
	//此处将等于中间值的赶到左边，其余感到右边
	for (;; ) {
		while (iLeft <= iRight && Point[iLeft].m_Pos[iDim] <= fMid_Value)
			++iLeft;
		while (iLeft <= iRight && Point[iRight].m_Pos[iDim] > fMid_Value)
			--iRight;
		if (iLeft > iRight)
			break;
		KD_Point_3D<_T> oTemp = Point[iLeft];
		Point[iLeft] = Point[iRight];
		Point[iRight] = oTemp;
	}
	//最后返回的是第一个大于fMid_Value的位置？
	*piLim2 = iLeft;
}

template<typename _T>static void Middle_Split_1(KD_Point_3D<_T> Point[], int* piSplit_Index, int iCount, float Bounding_Box[3][2], int* piDim, _T* pfMid_Value)
{//不靠Point_Index
	float fMin_Value, fMax_Value, fMax_Span, fTemp_Value, fMid_Value, fSpan;;
	int i, j, k;
	int iDim = 0;
	//寻找跨度最大的维度进行划分，这也有点道理
	fMax_Span = Bounding_Box[0][1] - Bounding_Box[0][0];

	for (i = 1; i < 3; i++)
	{
		if (Bounding_Box[i][1] - Bounding_Box[i][0] > fMax_Span)
		{
			fMax_Span = Bounding_Box[i][1] - Bounding_Box[i][0];
			iDim = i;
		}
	}

	fMin_Value = fMax_Value = (float)Point[0].m_Pos[iDim];
	for (i = 1; i < iCount; i++)
	{
		if ((fTemp_Value = (float)Point[i].m_Pos[iDim]) < fMin_Value)
			fMin_Value = fTemp_Value;
		else if (fTemp_Value > fMax_Value)
			fMax_Value = fTemp_Value;
	}

	fMid_Value = (float)((fMin_Value + fMax_Value) / 2.f);
	fMax_Span = fMax_Value - fMin_Value;
	k = iDim;

	for (i = 0; i < 3; i++)
	{
		if (i == k)
			continue;
		fSpan = Bounding_Box[i][1] - Bounding_Box[i][0];
		if (fSpan > fMax_Span)
		{
			fMin_Value = fMax_Value = (float)Point[0].m_Pos[i];
			for (j = 1; j < iCount; j++)
			{
				if ((fTemp_Value = (float)Point[j].m_Pos[i]) < fMin_Value)
					fMin_Value = fTemp_Value;
				else if (fTemp_Value > fMax_Value)
					fMax_Value = fTemp_Value;
			}
			fSpan = fMax_Value - fMin_Value;
			if (fSpan > fMax_Span)
			{
				fMax_Span = fSpan;
				iDim = i;
				fMid_Value = (float)((fMin_Value + fMax_Value) / 2.f);
			}
		}
	}
	//printf("Max_Span:%f Mid:%f Dim:%d\n", fMax_Span, fMid_Value,iDim);
	int iLim1, iLim2;
	Plane_Split<_T>(Point, iCount, iDim, (float)fMid_Value, &iLim1, &iLim2);
	//感觉是勉强保持一点点平衡，等于fMid_Value无论拨到那边都可以，放哪就是为了平衡
	if (iLim1 > iCount / 2)
		*piSplit_Index = iLim1;
	else if (iLim2 < iCount / 2)
		*piSplit_Index = iLim2;
	else
		*piSplit_Index = iCount / 2;

	*pfMid_Value = (_T)fMid_Value;
	*piDim = iDim;
}

template<typename _T>static void Divide_Tree_1(KD_Tree_Item<_T> Buffer[], KD_Point_3D<_T> Point[], int iLeft, int iRight, int* piCur_Node, float Bounding_Box[3][2], int iDim)
{//尝试不借助索引
#define LEAF_MAX_SIZE 16	//最后一层节点所含的叶（点）最大数量
							//修改并没有太大用处，可以进一步考察GPU的影响
							//并不改变到叶节点层数,但改变节点数量

	KD_Tree_Item<_T> oNode;
	int iSplit_Index, k;
	_T fMid_Value;

	if ((iRight - iLeft) <= LEAF_MAX_SIZE)
	{
		//printf("%d\n", iRight - iLeft);
		oNode.m_bIs_Point = 1;
		oNode.m_iLeft = iLeft;
		oNode.m_iRight = iRight;
		oNode.x_Dim = iDim;

		KD_Point_3D<_T> oPoint;
		//更新Bounding Box
		Bounding_Box[0][0] = Bounding_Box[0][1] = (float)Point[iLeft].m_Pos[0];
		Bounding_Box[1][0] = Bounding_Box[1][1] = (float)Point[iLeft].m_Pos[1];
		Bounding_Box[2][0] = Bounding_Box[2][1] = (float)Point[iLeft].m_Pos[2];
		for (k = iLeft + 1; k < iRight; ++k)
		{
			oPoint = Point[k];
			//fTemp_Value = Point[k].Pos_f[0];
			if (Bounding_Box[0][0] > oPoint.m_Pos[0])
				Bounding_Box[0][0] = (float)oPoint.m_Pos[0];
			else if (Bounding_Box[0][1] < oPoint.m_Pos[0])
				Bounding_Box[0][1] = (float)oPoint.m_Pos[0];

			//fTemp_Value = Point[k].Pos_f[1];
			if (Bounding_Box[1][0] > oPoint.m_Pos[1])
				Bounding_Box[1][0] = (float)oPoint.m_Pos[1];
			else if (Bounding_Box[1][1] < oPoint.m_Pos[1])
				Bounding_Box[1][1] = (float)oPoint.m_Pos[1];

			//fTemp_Value = Point[k].Pos_f[2];
			if (Bounding_Box[2][0] > oPoint.m_Pos[2])
				Bounding_Box[2][0] = (float)oPoint.m_Pos[2];
			else if (Bounding_Box[2][1] < oPoint.m_Pos[2])
				Bounding_Box[2][1] = (float)oPoint.m_Pos[2];
		}
	}
	else
	{
		Middle_Split_1<_T>(Point + iLeft, &iSplit_Index, iRight - iLeft, Bounding_Box, &iDim, &fMid_Value);
		oNode.m_bIs_Point = 0;
		oNode.x_Dim = iDim;

		float Left_Bounding_Box[3][2];
		memcpy(Left_Bounding_Box, Bounding_Box, 3 * 2 * sizeof(float));
		//Left_Bounding_Box[iDim][0] = Bounding_Box[iDim][0];
		Left_Bounding_Box[iDim][1] = (float)fMid_Value;

		Divide_Tree_1(Buffer, Point, iLeft, iLeft + iSplit_Index, piCur_Node, Left_Bounding_Box, iDim);
		oNode.m_iLeft = (*piCur_Node) - 1;
		float Right_Bounding_Box[3][2];
		memcpy(Right_Bounding_Box, Bounding_Box, 3 * 2 * sizeof(float));
		//Right_Bounding_Box[iDim][1] = Bounding_Box[iDim][1];
		Right_Bounding_Box[iDim][0] = (float)fMid_Value;
		Divide_Tree_1(Buffer, Point, iLeft + iSplit_Index, iRight, piCur_Node, Right_Bounding_Box, iDim);
		oNode.m_iRight = (*piCur_Node) - 1;
		oNode.m_fDiv_Low = (_T)Left_Bounding_Box[iDim][1];
		oNode.m_fDiv_High = (_T)Right_Bounding_Box[iDim][0];
		//if (oNode.m_fDiv_Low == oNode.m_fDiv_High)
			//printf("err");

		Bounding_Box[0][0] = Min(Left_Bounding_Box[0][0], Right_Bounding_Box[0][0]);
		Bounding_Box[0][1] = Max(Left_Bounding_Box[0][1], Right_Bounding_Box[0][1]);

		Bounding_Box[1][0] = Min(Left_Bounding_Box[1][0], Right_Bounding_Box[1][0]);
		Bounding_Box[1][1] = Max(Left_Bounding_Box[1][1], Right_Bounding_Box[1][1]);

		Bounding_Box[2][0] = Min(Left_Bounding_Box[2][0], Right_Bounding_Box[2][0]);
		Bounding_Box[2][1] = Max(Left_Bounding_Box[2][1], Right_Bounding_Box[2][1]);

		/*if (Bounding_Box[0][0] < -100000)
			printf("here");*/

	}
	Buffer[(*piCur_Node)++] = oNode;
	return;
#undef LEAF_MAX_SIZE
}

template<typename _T>static void Init_Tree(KD_Tree_3D<_T>* poTree, int iPoint_Count)
{//分配内存而已，不建树
	Light_Ptr oPtr;
	int iSize = iPoint_Count * sizeof(KD_Point_3D<_T>) + 128 +
		iPoint_Count * sizeof(KD_Tree_Item<_T>) + 128;

	KD_Tree_3D<_T> oTree = {};
	Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(iSize), iSize, 0);
	unsigned char* p;
	Malloc(oPtr, iPoint_Count * sizeof(KD_Point_3D<_T>), p);
	oTree.m_pPoint = (KD_Point_3D<_T>*)p;
	Malloc(oPtr, iPoint_Count * sizeof(KD_Tree_Item<_T>), p);
	oTree.m_pBuffer = (KD_Tree_Item<_T>*)p;
	*poTree = oTree;
}

template<typename _T>void Build_Tree(_T Point[][3], int iPoint_Count, KD_Tree_3D<_T>* poTree)
{//嘴贱数组作为输入
	KD_Tree_3D<_T> oTree;
	Init_Tree(&oTree, iPoint_Count);

	float Bounding_Box[3][2] = { {MAX_FLOAT,-MAX_FLOAT},
		{MAX_FLOAT,-MAX_FLOAT},
		{MAX_FLOAT,-MAX_FLOAT} };

	//_T Bounding_Box[3][2]  = { {0,(_T)oTree.m_Max_Size[0]},{0,(_T)oTree.m_Max_Size[1]},{0,(_T)oTree.m_Max_Size[2]} };
	//将点抄到树中，与原来Buffer脱离关系
	for (int i = 0; i < iPoint_Count; i++)
	{
		KD_Point_3D<_T> oPoint;
		oPoint = oTree.m_pPoint[i] = { Point[i][0], Point[i][1], Point[i][2], i };
		//顺便把Bounding Box也算了
		if (oPoint.x < Bounding_Box[0][0])
			Bounding_Box[0][0] = (float)oPoint.x;
		if (oPoint.x > Bounding_Box[0][1])
			Bounding_Box[0][1] = (float)oPoint.x;

		if (oPoint.y < Bounding_Box[1][0])
			Bounding_Box[1][0] = (float)oPoint.y;
		if (oPoint.y > Bounding_Box[1][1])
			Bounding_Box[1][1] = (float)oPoint.y;

		if (oPoint.z < Bounding_Box[2][0])
			Bounding_Box[2][0] = (float)oPoint.z;
		if (oPoint.z > Bounding_Box[2][1])
			Bounding_Box[2][1] = (float)oPoint.z;
	}

	oTree.m_iPoint_Count = iPoint_Count;

	int iCur_Node = 0;

	Divide_Tree_1(oTree.m_pBuffer, oTree.m_pPoint, 0, iPoint_Count, &iCur_Node, Bounding_Box, -1);
	oTree.m_iRoot = iCur_Node - 1;
	oTree.m_iNode_Count = iCur_Node;

	int iSize = iPoint_Count * sizeof(KD_Point_3D<_T>) + 128 +
		iCur_Node * sizeof(KD_Tree_Item<_T>) + 128;
	Shrink(oTree.m_pPoint, iSize);

	*poTree = oTree;
	return;
}

template<typename _T>void Build_Tree(KD_Point_3D<_T> Point[], int iPoint_Count, KD_Tree_3D<_T>* poTree)
{//对KD树建树，这个函数可以多个重载
	KD_Tree_3D<_T> oTree;
	Init_Tree(&oTree, iPoint_Count);

	//将点抄到树中，与原来Buffer脱离关系
	for (int i = 0; i < iPoint_Count; i++)
		oTree.m_pPoint[i] = { Point[i].x,Point[i].y,Point[i].z,i };


	int iCur_Node = 0;
	float Bounding_Box[3][2] = { {0,(float)oTree.m_Max_Size[0]},{0,(float)oTree.m_Max_Size[1]},{0,(float)oTree.m_Max_Size[2]} };

	Divide_Tree_1(oTree.m_pBuffer, oTree.m_pPoint, 0, iPoint_Count, &iCur_Node, Bounding_Box, -1);
	poTree->m_iRoot = iCur_Node - 1;

	int iSize = iPoint_Count * sizeof(KD_Point_3D<_T>) + 128 +
		iCur_Node * sizeof(KD_Tree_Item<_T>) + 128;
	Shrink(oTree.m_pPoint, iSize);

	*poTree = oTree;
	return;
}

template<typename _T>void Free_KD_Tree(KD_Tree_3D<_T>* poTree)
{
	Free(poTree->m_pPoint);
	*poTree = {};
}

template<typename _T>static void _Get_Nearest_Point(KD_Tree_3D<_T>* poTree, int iNode, _T Pos[3], Neighbour_Item<_T>* poNeighbour, float Dist[], float fMin_Distsq)
{
	KD_Tree_Item<_T> oNode = poTree->m_pBuffer[iNode];
	KD_Point_3D<_T>* pPoint = poTree->m_pPoint, oPoint;
	float fDistance;
	int i;
	if (oNode.m_bIs_Point)
	{//到达叶子节点，孩子是点
		Neighbour_Item<_T> oNeighbour = *poNeighbour;
		for (i = (int)oNode.m_iLeft; i < (int)oNode.m_iRight; ++i)
		{
			oPoint = pPoint[i];
			fDistance = (float)fGet_Distance(oPoint.m_Pos, Pos, 3);
			if (fDistance < oNeighbour.m_fDistance)
			{	//swap
				oNeighbour.m_fDistance = fDistance;
				oNeighbour.m_iIndex = i;
			}
		}
		*poNeighbour = oNeighbour;
		return;
	}
	//未到叶节点
	_T fValue = (_T)Pos[oNode.x_Dim];
	_T cut_dist;
	int bestChild, otherChild;

	if (fValue <= oNode.m_fDiv_Low)	//窃以为这个判断更直观
	{
		bestChild = oNode.m_iLeft;
		otherChild = oNode.m_iRight;
		cut_dist = fValue - oNode.m_fDiv_High;
	}
	else
	{
		bestChild = oNode.m_iRight;
		otherChild = oNode.m_iLeft;
		cut_dist = fValue - oNode.m_fDiv_Low;
	}

	cut_dist *= cut_dist;
	_Get_Nearest_Point(poTree, bestChild, Pos, poNeighbour, Dist, fMin_Distsq);
	float dst = Dist[oNode.x_Dim];
	fMin_Distsq = (float)(fMin_Distsq + cut_dist - dst);
	Dist[oNode.x_Dim] = (float)cut_dist;
	//经测试，以下判断确实最优,比单纯判断cut_dist<poNeighbour->m_fWorst_Distance 更优，但是原理尚未明
	if (fMin_Distsq <= poNeighbour->m_fDistance)
		_Get_Nearest_Point(poTree, otherChild, Pos, poNeighbour, Dist, fMin_Distsq);
	Dist[oNode.x_Dim] = (float)dst;

	return;
}

template<typename _T>void Get_Nearest_Point(KD_Tree_3D<_T>* poTree, _T Pos[3], _T Neighbour[3], float* pfDist, int* piOrg_Index)
{//仅找最近邻点, 要写若干个接口
	//float Dist[3] = { 0.f,0.f,0.f };
	//poNeighbour->m_fDistance = MAX_FLOAT;
	//Get_Nearest_Point(poTree, poTree->m_iRoot, *poPos, poNeighbour, Dist, 0);
	float Dist[3] = { 0.f,0.f,0.f };
	Neighbour_Item<_T> oNeighbour;
	oNeighbour.m_fDistance = MAX_FLOAT;
	_Get_Nearest_Point<_T>(poTree, poTree->m_iRoot, Pos, &oNeighbour, Dist, 0.f);

	KD_Point_3D<_T> oPoint = poTree->m_pPoint[oNeighbour.m_iIndex];
	Neighbour[0] = oPoint.m_Pos[0], Neighbour[1] = oPoint.m_Pos[1], Neighbour[2] = oPoint.m_Pos[2];
	if (piOrg_Index)
		*piOrg_Index = oPoint.m_iOrg_Index;
	if (pfDist)
		*pfDist = oNeighbour.m_fDistance;
	return;
}

template<typename _T>void Get_Nearest_Point_Ref(KD_Tree_3D<_T>* poTree, _T Pos[3], _T Neighbour[3], float* pfDist, int* piOrg_Index)
{
	Get_Nearest_Point_Ref(poTree->m_pPoint, poTree->m_iPoint_Count, Pos, Neighbour, pfDist, piOrg_Index);
}
template<typename _T>void Get_Nearest_Point_Ref(KD_Point_3D<_T> Point[], int iPoint_Count, _T Pos[3], _T Neighbour[3], float* pfDist, int* piOrg_Index, int* piNew_Index)
{//暴力计算，最慢，用于验算
	Neighbour_Item<_T> oNeighbour = { 0, MAX_FLOAT };
	int iNew_Index = 0;
	for (int i = 0; i < iPoint_Count; i++)
	{
		float fDist = (float)fGet_Distance(Point[i].m_Pos, Pos, 3);
		if (fDist < oNeighbour.m_fDistance)
		{
			oNeighbour = { Point[i].m_iOrg_Index,fDist };
			iNew_Index = i;
		}
		if (fDist == 0)
			break;
	}
	memcpy(Neighbour, Point[oNeighbour.m_iIndex].m_Pos, 3 * sizeof(_T));
	if (pfDist)
		*pfDist = oNeighbour.m_fDistance;
	if (piOrg_Index)
		*piOrg_Index = Point[oNeighbour.m_iIndex].m_iOrg_Index;
	if (piNew_Index)
		*piNew_Index = iNew_Index;
	return;
}/******************************一组KD_Tree函数**************************/

/*******************************Mem_Mgr***********************************/
void* pMalloc(Light_Ptr* poPtr, int iSize)
{//轻量级的内存分配
	unsigned char* pBuffer;
	Lock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	Malloc(*poPtr, iSize, pBuffer);
	Unlock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	return (void*)pBuffer;
}

void* pMalloc(unsigned int iSize)
{//原来的pMalloc多了个参数太麻烦，干脆包一层更少
	void* p;
	Lock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	p = pMalloc(&oMem_Mgr.oMem_Mgr, iSize);
	Unlock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	return p;
}
void Free(void* p)
{
	if (!p)return;
	Lock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	Free(&oMem_Mgr.oMem_Mgr, p);
	Unlock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
}
void* pMove_2_Front(void* p)
{
	Lock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	p = pMove_2_Front(&oMem_Mgr.oMem_Mgr,p);
	Unlock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	return p;
}
void Shrink(void* p, unsigned int iSize)
{
	Lock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	Shrink(&oMem_Mgr.oMem_Mgr, p, iSize);
	Unlock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	return;
}
int bExpand(void* p, unsigned int iSize)
{
	Lock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	int bRet;
	if (p)
		bRet = bExpand(&oMem_Mgr.oMem_Mgr, p, iSize);
	else
		bRet = 1;
	Unlock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	return bRet;
}
void Disp_Mem()
{
	Lock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	Disp_Mem(&oMem_Mgr.oMem_Mgr, 0);
	Unlock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
}

int bInit_Env_CPU(unsigned long long iSize, int iBlock_Size, int iMax_Piece_Count)
{//初始化个环境，分配些内存供一切函数临时使用
	//这个项目用统一内存
	oMem_Mgr.m_oS.m_poLock = new mutex();
	Init_Mem_Mgr(&oMem_Mgr.oMem_Mgr, iSize, iBlock_Size, iMax_Piece_Count);
	if (oMem_Mgr.oMem_Mgr.m_pBuffer)
	{
#ifdef ESP32
		printf("Memory Pool inited successfully\n");
#endif
		return 1;
	}
	//printf("Fail to init Memory Pool\n");
	return 0;
}

void Free_Env_CPU()
{
	Lock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	if (oMem_Mgr.oMem_Mgr.m_iPiece_Count)
		Disp_Mem(&oMem_Mgr.oMem_Mgr, 0);

	if (oMem_Mgr.oMem_Mgr.m_pBuffer)
		Free_Mem_Mgr(&oMem_Mgr.oMem_Mgr);
	Unlock_Semaphore_For_Thread(&oMem_Mgr.m_oS);
	if (oMem_Mgr.m_oS.m_poLock)
		delete (mutex*)oMem_Mgr.m_oS.m_poLock;
	oMem_Mgr = {};
}
/*******************************Mem_Mgr***********************************/

/*******************************直线函数*********************************/
void Cal_Line(Line_1* poLine, float x0, float y0, float x1, float y1)
{//过两点决定直线方程
	poLine->x0 = x0, poLine->y0 = y0, poLine->x1 = x1, poLine->y1 = y1;
	//求ax+bx+c=0
	poLine->a = (float)(y0 - y1);
	poLine->b = (float)(x1 - x0);
	poLine->c = (float)(x0 * y1 - x1 * y0);

	if (x0 != x1)
	{//此时才有y=kx+m的表示
		poLine->k = (float)(y0 - y1) / (x0 - x1);
		poLine->m = y0 - poLine->k * x0;
		poLine->theta = (float)atan(poLine->k);
		//还要决定直线的方向
		if (x1 < x0)
			poLine->theta += (float)PI;

		if (y0 == y1)
			poLine->m_iMode = Line_1::Mode::Degree_180;
		else
			poLine->m_iMode = Line_1::Mode::Degree_Other;
	}
	else
	{
		poLine->k = NAN;
		poLine->m = x0;	//x=x0
		if (y0 > y1)
			poLine->theta = (float)(3 * PI / 2);
		else
			poLine->theta = (float)(PI / 2);
		poLine->m_iMode = Line_1::Mode::Degree_90;
	}
	return;
}
float fGet_Line_y(Line_1* poLine, float x)
{//代入x，求y
	if (poLine->m_iMode == Line_1::Mode::Degree_90)
	{
		if (x != poLine->m)
			return NAN;	//此处不适用于cuda，得另想办法
		else
			return 0;	//随便返回一个值即可
	}
	else
		return poLine->k * x + poLine->m;
}
float fGet_Line_x(Line_1* poLine, float y)
{//代入y,求x= (y-m)/k
	if (poLine->k == 0)
	{//水平线，难求
		if (poLine->m == y)
			return 0;	//随便返回一个了事
		else
			return NAN;
	}
	else if (poLine->m_iMode == Line_1::Mode::Degree_90)
		return poLine->x0;
	else
		return (y - poLine->m) / poLine->k;
}
/*******************************直线函数*********************************/

/**********************一组点云函数****************************/
template<typename _T>void Init_Point_Cloud(Point_Cloud<_T>* poPC, int iMax_Count, int bHas_Color)
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
		printf("Insufficient memroy\n");
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

template<typename _T>void Gen_Sphere(_T(**ppPoint_3D)[3], int* piCount, _T r, int iStep_Count)
{//生成一个球，半径为1
 // x = r*sin(beta)*cos(alpha)
 // y = r*sin(beta)*sin(alpha)
 // z = r*cos(beta)
	_T alpha, beta, (*pPoint_3D)[3], x, y, z, fDelta_2, r_sin_beta, fDelta_2_Start;
	int i, j, iCur_Point = 0, iStep_Count_1;
	const _T fDelta_1 = (_T)(PI / iStep_Count);
	Line_1 oLine;
	pPoint_3D = (_T(*)[3])pMalloc((iStep_Count * iStep_Count / 2) * 3 * sizeof(_T));

	Cal_Line(&oLine, 0.f, 1.f, (float)iStep_Count, (float)iStep_Count);
	for (i = 0; i <= (iStep_Count >> 1); i++)
	{//外层是z
		beta = i * fDelta_1;
		z = r * (_T)cos(beta);
		iStep_Count_1 = (int)fGet_Line_y(&oLine, (float)i);
		fDelta_2 = (_T)(PI * 2.f / iStep_Count_1);
		r_sin_beta = r * (_T)sin(beta);
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
	Shrink(pPoint_3D, iCur_Point * 3 * sizeof(_T));
	if (ppPoint_3D)
		*ppPoint_3D = pPoint_3D;
	if (piCount)
		*piCount = iCur_Point;
	return;
}
template<typename _T>void Draw_Rect(Point_Cloud<_T>* poPC, _T x, _T y, _T z, _T w, _T h, int iStep, int R, int G, int B)
{
	Point_Cloud<_T> oPC = *poPC;
	Draw_Line(&oPC, x, y, z, x + w, y, z, (int)(w / iStep), R, G, B);
	Draw_Line(&oPC, x + w, y, z, x + w, y + h, z, (int)(h / iStep), R, G, B);
	Draw_Line(&oPC, x + w, y + h, z, x, y + h, z, (int)(w / iStep), R, G, B);
	Draw_Line(&oPC, x, y + h, z, x, y, z, (int)(h / iStep), R, G, B);
	*poPC = oPC;
	//bSave_PLY("c:\\tmp\\1.ply", oPC);
	return;
}
template<typename _T>void Draw_Sphere(Point_Cloud<_T>* poPC, _T x, _T y, _T z, _T r, int iStep_Count, int R, int G, int B)
{
	Point_Cloud<_T> oPC = *poPC;
	_T(*pPoint_3D)[3];
	int i, iCount, iPos;
	Gen_Sphere(&pPoint_3D, &iCount, r, iStep_Count);
	if (oPC.m_iCount + iCount > oPC.m_iMax_Count)
	{
		printf("Insufficient memroy");
		return;
	}
	iPos = oPC.m_iCount;
	for (i = 0; i < iCount; i++, iPos++)
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
		_T m_Pos[3] = { x0 + (x1 - x0) * i / iCount,
			y0 + (y1 - y0) * i / iCount,
			z0 + (z1 - z0) * i / iCount };
		Draw_Point(poPC, m_Pos[0], m_Pos[1], m_Pos[2], R, G, B);
	}
	return;
}
/**********************一组点云函数****************************/

/*****************************一组文件操作********************************/
template<typename _T>void bSave_PLY(const char* pcFile, Point_Cloud<_T> oPC)
{
	bSave_PLY(pcFile, oPC.m_pPoint, oPC.m_iCount, oPC.m_pColor);
}
template<typename _T>int bRead_PLY_File(const char* pcFile, PCC_Point<_T>** ppPoint, int* piPoint_Count, PCC_Face** ppFace, int* piFace_Count)
{//改一改，连点都放在 Oct_Tree_Item上
	int i, j, bRet = 1, iResult, bText = 1, iPoint_Count = 0, iFace_Count = 0,
		iColor_Count = 0, iCur_Color = 0, iSize, Color_Order[3];
	PCC_Point<_T>* pPoint = NULL;
	FILE* pFile = fopen(pcFile, "rb");
	float Min[3] = { MAX_FLOAT,MAX_FLOAT,MAX_FLOAT }, Max[3] = { -MAX_FLOAT,-MAX_FLOAT,-MAX_FLOAT, }, fMax = -MAX_FLOAT;
	char Line[256];
	//Pos_3<_T> oPos;
	PCC_Face* pFace = NULL;
	unsigned char* pCur = NULL;
	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		return 0;
	}

	while (bGet_Line(pFile, Line))
	{
		//printf("%s\n", Line);
		if (strstr(Line, "element vertex"))
		{//element vertex字段
			iResult = sscanf(Line + strlen("element vertex") + 1, "%d", &iPoint_Count);
			if (iResult < 1)
			{
				printf("No vertex\n");
				bRet = 0;
				goto END;
			}
		}
		else if (strstr(Line, "element face"))
		{
			iResult = sscanf(Line + strlen("element face") + 1, "%d", &iFace_Count);
			if (iResult < 1)
			{
				printf("No element face\n");
				bRet = 0;
				goto END;
			}
		}
		else if (bStricmp(Line, (char*)"format binary_little_endian 1.0") == 0)
			bText = 0;
		else if (bStricmp(Line, (char*)"property uchar red") == 0)
		{
			iColor_Count = 3;
			Color_Order[iCur_Color++] = 0;
		}
		else if (bStricmp(Line, (char*)"property uchar green") == 0)
		{
			iColor_Count = 3;
			Color_Order[iCur_Color++] = 1;
		}
		else if (bStricmp(Line, (char*)"property uchar blue") == 0)
		{
			iColor_Count = 3;
			Color_Order[iCur_Color++] = 2;
		}
		else if (bStricmp(Line, (char*)"property uchar alpha") == 0)
			iColor_Count = 4;
		else if (bStricmp(Line, (char*)"end_header") == 0)
		{//头读完了
			if (fgetc(pFile) != 0x0A)
				fseek(pFile, -1, SEEK_CUR);
			break;
		}
	}
	if (iFace_Count)
		pFace = (PCC_Face*)pMalloc(iFace_Count * sizeof(PCC_Face));
	else
		pFace = NULL;

	iSize = iPoint_Count;
	if (iPoint_Count)
		pPoint = (PCC_Point<_T>*)pMalloc(sizeof(PCC_Point<_T>) * iSize);
	else
		pPoint = NULL;

	if (bText)
	{
		float Temp[3];
		for (i = 0; i < iPoint_Count; i++)
		{
			bGet_Line(pFile, Line);
			iResult = sscanf(Line, "%f %f %f", &Temp[0], &Temp[1], &Temp[2]);
			for (j = 0; j < 3; j++)
			{
				if (Min[j] > Temp[j])
					Min[j] = Temp[j];
				if (Max[j] < Temp[j])
					Max[j] = Temp[j];
				//if (Min[0] < -1000)
					//printf("Here");
			}
			pPoint[i].m_oPos = { (_T)Temp[0],(_T)Temp[1],(_T)Temp[2] };
			pPoint[i].m_bAdd_To_Queue = pPoint[i].m_bHas_Normal = 0;
		}
	}
	else
	{//还得从Compute_Normal抄回来
		printf("Not implemented\n");
	}
	*piPoint_Count = iPoint_Count;
	*ppPoint = pPoint;
	if (piFace_Count)
		*piFace_Count = iFace_Count;
	if (ppFace)
		*ppFace = pFace;
	else if (pFace)
		Free(pFace);

END:
	if (pFile)
		fclose(pFile);

	return bRet;
}

template<typename _T>int bSave_PLY(const char* pcFile, _T Point[][3], int iPoint_Count, unsigned char Color[][3], int bText)
{//存点云，最简形式，用于实验，连结构都不要
	FILE* pFile = fopen(pcFile, "wb");
	char Header[512];
	int i;
	_T* pPos;

	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		return 0;
	}
	if (!iPoint_Count)
	{
		printf("No point to save\n");
		return 0;
	}

	//先写入Header
	sprintf(Header, "ply\r\n");
	if (bText)
		sprintf(Header + strlen(Header), "format ascii 1.0\r\n");
	else
		sprintf(Header + strlen(Header), "format binary_little_endian 1.0\r\n");
	sprintf(Header + strlen(Header), "comment HQYT generated\r\n");
	sprintf(Header + strlen(Header), "element vertex %d\r\n", iPoint_Count);
	sprintf(Header + strlen(Header), "property float x\r\n");
	sprintf(Header + strlen(Header), "property float y\r\n");
	sprintf(Header + strlen(Header), "property float z\r\n");

	if (Color)
	{
		sprintf(Header + strlen(Header), "property uchar red\r\n");
		sprintf(Header + strlen(Header), "property uchar green\r\n");
		sprintf(Header + strlen(Header), "property uchar blue\r\n");
	}

	sprintf(Header + strlen(Header), "end_header\r\n");
	fwrite(Header, 1, strlen(Header), pFile);

	for (i = 0; i < iPoint_Count; i++)
	{
		pPos = Point[i];
		if (bText)
		{
			fprintf(pFile, "%f %f %f ", pPos[0], pPos[1], pPos[2]);
			if (pPos[0] >= 1000000.f || isnan(pPos[0]))
				printf("here");
			if (Color)
				fprintf(pFile, "%d %d %d\r\n", Color[i][0], Color[i][1], Color[i][2]);
			else
				fprintf(pFile, "\r\n");
		}
	}
	fclose(pFile);
	return 1;
}
int iDelete_Dir(char Path[])
{
	int iResult;
#ifdef WIN32
	if (iResult = _rmdir(Path) == 0)
#else
	if (iResult = rmdir(Path) == 0)
#endif
		return 1;
	return 0;
}
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer, int iSize, int bNeed_Malloc, int iFrame_No)
{
	FILE* pFile = fopen(pcFile, "rb");
	unsigned long long iPos;
	int bRet = 0, iResult;
	iPos = (unsigned long long)iSize * iFrame_No;
	unsigned char* pBuffer;

	if (!iSize)
		iSize = (int)iGet_File_Length((char*)pcFile);

	if (bNeed_Malloc)
		pBuffer = (unsigned char*)pMalloc(iSize);
	else
		pBuffer = *ppBuffer;

	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		goto END;
	}
	if (!pBuffer)
	{
		printf("Fail to allocate memory in bLoad_Raw_Data, size:%d\n", iSize);
		goto END;
	}
#ifdef WIN32
	_fseeki64(pFile, iPos, SEEK_SET);
#else
	fseeko(pFile, (unsigned long long)iPos, SEEK_SET);
#endif

	iResult = (int)fread(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		if (pBuffer)
			free(pBuffer);
		*ppBuffer = NULL;
		printf("Fail to read data\n");
		bRet = 0;
		goto END;
	}
	*ppBuffer = pBuffer;
	bRet = 1;
END:
	if (pFile)
		fclose(pFile);
	if (!bRet)
	{
		if (pBuffer && bNeed_Malloc)
			free(pBuffer);
	}
	return bRet;
}
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer, int* piSize)
{
	FILE* pFile = fopen(pcFile, "rb");
	int bRet = 0, iResult, iSize;
	unsigned char* pBuffer=NULL;
	iSize = (int)iGet_File_Length((char*)pcFile);
	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		goto END;
	}
	pBuffer = (unsigned char*)pMalloc(iSize);
	if (!pBuffer)
	{
		printf("Fail to allocate memory in bLoad_Raw_Data, Size:%d\n",iSize);
		Disp_Mem();
		goto END;
	}

	iResult = (int)fread(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		bRet = 0;
		*ppBuffer = NULL;
		printf("Fail to read data\n");
		goto END;
	}else
		*ppBuffer = pBuffer;
	if (piSize)
		*piSize = iSize;
	bRet = 1;
END:
	if (pFile)
		fclose(pFile);
	if(!bRet)
		Free(pBuffer);
	return bRet;
}
int bLoad_Text_File(const char* pcFile, char** ppBuffer, int* piSize)
{//装入一个文本文件
	FILE* pFile = fopen(pcFile, "rb");
	int bRet = 0, iResult, iSize;
	char* pBuffer;
	iSize = (int)iGet_File_Length((char*)pcFile);
	pBuffer = (char*)pMalloc(iSize + 1);

	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		goto END;
	}
	if (!pBuffer)
	{
		printf("Fail to allocate memory in bLoad_Text_File\n");
		goto END;
	}

	iResult = (int)fread(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		if (pBuffer)
			free(pBuffer);
		*ppBuffer = NULL;
		printf("Fail to read data\n");
		goto END;
	}
	pBuffer[iSize] = '\0';
	*ppBuffer = pBuffer;
	if (piSize)
		*piSize = iSize;
	bRet = 1;
END:
	if (pFile)
		fclose(pFile);
	if (!bRet)
	{
		if (pBuffer)
			free(pBuffer);
	}
	return bRet;
}

int bSave_Raw_Data(const char* pcFile, void* pBuffer, int iSize)
{
	FILE* pFile = fopen(pcFile, "wb");
	if (!pFile)
	{
		printf("Fail to save file:%s\n", pcFile);
		return 0;
	}
	int iResult = (int)fwrite(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		//printf("Fail to save file:%s %d\n", pcFile,GetLastError());
		fclose(pFile);
		return 0;
	}
	fclose(pFile);
	return 1;
}

int bSave_Bin(const char* pcFile, float* pData, int iSize)
{//似乎并不需要，有Svae_Raw_Data就够了
	FILE* pFile = fopen(pcFile, "wb");
	if (!pFile)
	{
		printf("Fail to save:%s\n", pcFile);
		return 0;
	}
	int iResult = (int)fwrite(pData, 1, iSize, pFile);
	if (iResult != iSize)
	{
		printf("Fail to save:%s\n", pcFile);
		iResult = 0;
	}
	else
		iResult = 1;
	return iResult;
}

void Disp_File_List(char* pBuffer, int iSize, int bInclude_Dir)
{
	int iExtra_Space = bInclude_Dir ? 3 : 1;
	char* pCur = pBuffer;
	for (int i = 0; i < iSize;)
	{
		int iLen;
		if (bInclude_Dir)
		{
			printf("%c %s\n", pCur[0], &pCur[2]);
			iLen = (int)strlen(&pCur[2]);
		}
		else
		{
			printf("%s\n", pCur);
			iLen = (int)strlen(pCur);
		}
		pCur += iLen + iExtra_Space;
		i += iLen + iExtra_Space;
	}
}

#ifdef WIN32
Key iGet_Key()
{
	//一个箭头捡由两个字符组成
	int ch = _getch();
	if (ch == 0 || ch == 224)
	{//箭头键首字符为224，Fx键首字符为0
		ch = _getch();
		switch (ch)
		{
		case 72:
			return Arrow_Up;
		case 80:
			return Arrow_Down;
		case 75:
			return Arrow_Left;
		case 77:
			return Arrow_Right;
		default:
			return Invalid_Key;
		}
	}
	else if (ch == 27)
		return Esc;

	return Invalid_Key;
}

char* pGet_File_List(const char* pcPath, int *piFile_Count,int* piBuffer_Size,int bInclude_Dir)
{//取一个目录所有文件 例: c:\\tmp\\*.bin
	intptr_t handle;
	_finddata_t findData;
	int iCount = 0;
	handle = _findfirst(pcPath, &findData);    // 查找目录中的第一个文件
	if (handle == -1)
	{
		printf("File not found\n");
		return NULL;
	}

	//不一定够长
	const int iMax_Size = 100000;
	char* pBuffer = (char*)pMalloc(iMax_Size), * pCur = pBuffer;
	if (!pBuffer)
		return NULL;

	int bRet = 1;
	int iExtra_Space = bInclude_Dir ? 3 : 1;
	do
	{
		if(bStricmp(findData.name, (char*)".") || 
			bStricmp(findData.name, (char*)".."))
			continue;

		int iLen = (int)strlen(findData.name);
		int bRet = pCur + iLen + iExtra_Space - pBuffer <= iMax_Size ? 1 : 0;
		char cType = findData.attrib & _A_SUBDIR ? 'd' : 'f';
		if (!bRet)
			break;

		if (bInclude_Dir)
		{
			pCur[0] = cType;
			pCur[1] = 0;
			strcpy(&pCur[2], findData.name);
			pCur[iExtra_Space + iLen] = 0;
			iCount++;
		}else if(!(findData.attrib & _A_SUBDIR))
		{
			strcpy(pCur, findData.name);
			iCount++;
		}

		pCur[iLen + iExtra_Space] = 0;
		pCur += iLen + iExtra_Space;
	} while (_findnext(handle, &findData) == 0);    // 查找目录中的下一个文件
	_findclose(handle);    // 关闭搜索句柄

	if (!bRet)
	{
		Free(pBuffer);
		return NULL;
	}

	if (iCount && pCur - pBuffer)
	{
		Shrink(pBuffer, (unsigned int)(pCur - pBuffer));
		if(piBuffer_Size)
			*piBuffer_Size = (int)(pCur - pBuffer);
		if (piFile_Count)
			*piFile_Count = iCount;
	}
	else
	{
		Free(pBuffer), pBuffer = NULL;
		if (piBuffer_Size)
			*piBuffer_Size = 0;
		if (piFile_Count)
			*piFile_Count = 0;
	}

	return pBuffer;
}
int iCreate_Dir(char Path[])
{
	int iResult;
	if ( (iResult = _mkdir(Path)) == 0)
		return 1;
	return 0;
}
#else
int iCreate_Dir(char Path[])
{
	if (mkdir(Path, 0777) == 0)
		return 1;
	return 0;
}
char* pGet_File_List(const char* pcPath, int* piFile_Count, int* piBuffer_Size, int bInclude_Dir)
{//
	DIR* dir = opendir(pcPath);
	int iCount = 0;
	if (dir == NULL)
	{
		printf("无法打开 ESP32 存储分区\n");
		return NULL;
	}
	//不一定够长
	const int iMax_Size = 100000;
	char* pBuffer = (char*)pMalloc(iMax_Size), * pCur = pBuffer;
	if (!pBuffer)
		return NULL;

	int bRet = 1;
	dirent* entry;
	int iExtra_Space = bInclude_Dir ? 3 : 1;
	while (entry = readdir(dir))
	{//有文件/目录你
		int iLen = strlen(entry->d_name);
		int bRet = pCur + iLen + iExtra_Space - pBuffer <= iMax_Size ? 1 : 0;
		char cType = entry->d_type == DT_DIR ? 'd' : 'f';
		if (!bRet)
			break;

		if (bInclude_Dir)
		{
			pCur[0] = cType;
			pCur[1] = 0;
			strcpy(&pCur[2], entry->d_name);
			pCur[iExtra_Space + iLen] = 0;
			iCount++;
		}
		else if (cType = 'f')
		{
			strcpy(pCur, entry->d_name);
			iCount++;
		}

		pCur[iLen + iExtra_Space] = 0;
		pCur += iLen + iExtra_Space;
	}
	closedir(dir);

	if (!bRet)
	{
		Free(pBuffer);
		return NULL;
	}

	Shrink(pBuffer, pCur - pBuffer);
	if (piBuffer_Size)
		*piBuffer_Size = pCur - pBuffer;
	
	if (piFile_Count)
		*piFile_Count = iCount;
	return pBuffer;
}
#endif

unsigned long long iGet_File_Length(char* pcFile)
{//return: >-0 if success; -1 if fail
	FILE* pFile = fopen(pcFile, "rb");
	long long iLen;
#ifdef WIN32
	if (!pFile)
	{

		int iResult = GetLastError();
		printf("Fail to get file length, error:%d\n", iResult);

		return -1;
	}
	_fseeki64(pFile, 0, SEEK_END);
	iLen = _ftelli64(pFile);
#else
	if (!pFile)
	{
		printf("Fail to get file length\n");
		return -1;
	}

	fseeko(pFile, 0, SEEK_END);
	iLen = ftello(pFile);
#endif

	fclose(pFile);
	return iLen;
}

#ifdef WIN32
int iGet_File_Count(const char* pcPath)
{//给定文件路径，求此路径下得文件数量
	intptr_t handle;
	_finddata_t findData;
	int iCount = 0;
	handle = _findfirst(pcPath, &findData);    // 查找目录中的第一个文件
	if (handle == -1)
	{
		printf("File not found\n");
		return 0;
	}
	do
	{
		if (findData.attrib & _A_ARCH)
			iCount++;
	} while (_findnext(handle, &findData) == 0);    // 查找目录中的下一个文件
	_findclose(handle);    // 关闭搜索句柄
	return iCount;
}
#endif
/*****************************一组文件操作********************************/

/*****************************网络函数*************************************/
void Init_Socket_Env()
{
#ifdef WIN32
	SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (s != INVALID_SOCKET)
		return;

	// 1. 初始化 Winsock 库
	WSADATA wsaData;
	int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (result != 0)
	{
		printf("WSAStartup 失败，错误码:%d\n", result);
		return;
	}
#endif
}
void Free_Socket_Env()
{//纯属记不住，多此一举搞一个配对
#ifdef WIN32
	WSACleanup();
#endif
}

int iSendEx(int iSocket, void* buf, int iLen)
{
	int i, j, iRemain, iResult;
	i = iRemain = iLen;
	j = 0;
	if (!iSocket)
		return 0;
	char* pBuf_1 = (char*)buf;
	while (i > 0)
	{
		iResult = send(iSocket, (const char*)buf, iRemain, 0);
		if (iResult <= 0)
			return 0;
		j += iResult;
		i -= iResult;
		iRemain = i;
	}
	return iLen;
}

int iRecvEx(int iSocket, void* buf, int iLen)
{//purpose: pack the recv so as to improve the performance.
//Return: count of bytes if successful; SOCKET_ERROR if network error
	int i, j, iRemain, iResult;
	i = iRemain = iLen;
	j = 0;
	if (!iSocket)
		return 0;
	char* pBuf_1 = (char*)buf;
	while (i > 0)
	{
		iResult = recv(iSocket, pBuf_1 + j, iRemain, 0);
		if (iResult <= 0)
			return 0;
		j += iResult;
		i -= iResult;
		iRemain = i;
	}
	return iLen;
}
int iReply_Recv(int iSocket)
{
	int iReplay = REPLY_RECV;
	return iSendEx(iSocket, &iReplay, sizeof(int));
}
int iCmd_Shake_Hand_Client(int iSocket)
{
	int iCmd = CMD_SHAKE_HAND;
	int iResult = iSendEx(iSocket, &iCmd, sizeof(int));
	iResult = iRecvEx(iSocket, &iCmd, sizeof(int));
	if (iResult)
		printf("Shake hands sucessfully\n");
	else
		printf("Fail to shake hand\n");
	return iResult;
}
int iCmd_Shake_Hand_Server(int iSocket)
{//握手很简单那，前面在Distribute_Cmd 处已经获得握手Cmd
	//此处只需回到一个收到即可，类似三次握手的意思
	int iResult = iReply_Recv(iSocket);
	static int iCount = 0;
	if (!iResult)
		printf("Fail to shake hands\n");
	else
		printf("Shake hands successfully:%d \n", iCount++);
	return iResult;
}

void Destroy_Conn(int iSocket)
{//不建议使用，瞬间断开，不留任何体面的关闭
	//通常放在服务器端，用于保护不被恶意连接
	//netstat -ano | findstr port 不再右任何残留
	struct linger so_linger;
	so_linger.l_onoff = 1;   // 1 表示开启 Linger 选项
	so_linger.l_linger = 0;  // 0 表示超时时间为 0 秒
	// 设置 Socket 属性
	setsockopt(iSocket, SOL_SOCKET, SO_LINGER, (const char*)&so_linger, sizeof(so_linger));
}

void Close_Connection(int iSocket,int bDestroy)
{//没啥营养，就是让程序更好写
	if (bDestroy)
		Destroy_Conn(iSocket);

	Close_Socket(iSocket);
}
int iInit_Connection(char IP[], int iPort)
{
#ifdef WIN32
	Init_Socket_Env();
#endif

	// 1. 创建 IPv4 的 TCP 套接字
	int iSocket = (int)socket(AF_INET, SOCK_STREAM, 0);
	if (iSocket < 0) {
		perror("Fail to get sock_fd\n");
		return 0;
	}

	// 2. 配置目标服务器地址
	struct sockaddr_in server_addr = {};
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(iPort); // 主机字节序转网络字节序

	// 3. 解析 IP 地址
	if (inet_pton(AF_INET, IP, &server_addr.sin_addr) <= 0)
	{
		printf("无效的 IP 地址: %s\n", IP);
		Close_Socket(iSocket);
		return 0;
	}

	static int i = 0;
	printf("第%d趟，正在连接 %s:%d ...\n",i++, IP, iPort);
	if (connect(iSocket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
	{
		printf("Fail to connect remote server\n");
		Close_Socket(iSocket);
		return 0;
	}
	return iSocket;
}
int bSet_Non_Block(int iSocket)
{
	u_long mode = 1; // 1 代表开启非阻塞，0 代表恢复阻塞
	if (ioctlsocket(iSocket, FIONBIO, &mode) < 0)
	{
		printf("Fail to Set_Non_Block\n");
		return 0;
	}
	return 1;
}

int iListen(char IP[], int iPort)
{
#ifdef WIN32
	Init_Socket_Env();
#endif
	// 1. 创建流式套接字 (TCP)
	int iSocket = (int)socket(AF_INET, SOCK_STREAM, 0);
	if (iSocket < 0)
	{
		perror("Fail to get socket\n");
		return 0;
	}

	int opt = 1;
	// 2. 核心设置：开启端口复用或独占，防止程序重启时报端口被占用 (Address already in use)
#ifdef _WIN32
	//if (setsockopt(iSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt)) < 0) {
	if (setsockopt(iSocket, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, (char*)&opt, sizeof(opt)) < 0) {
#else
	if (setsockopt(iSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
#endif
		printf("Fail to setsockopt\n ");
		Close_Socket(iSocket);
		return 0;
	}

	// 3. 配置本地监听地址结构体
	struct sockaddr_in local_addr;
	memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_port = htons(iPort);                  // 主机字节序转网络字节序
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);      // 监听本地所有网卡/IP (0.0.0.0)

	// 4. 绑定端口
	if (bind(iSocket, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0)
	{
		printf("Fail to bind to port:%d\n",iPort);
		Close_Socket(iSocket);
		return 0;
	}

	// 5. 进入监听状态
	if (listen(iSocket, 800) < 0)
	{
		printf("Fail to listen");
		Close_Socket(iSocket);
		return 0;
	}

	printf("Listening to %d...\n", iPort);
	return iSocket;
}

//Distribute_Cmd 范例
static void Distribute_Cmd(int iSocket)
{
	int iCmd = 0;
	int iResult = iRecvEx(iSocket, &iCmd, 4);
	if (!iResult)
		return;	//出错了

	switch (iCmd)
	{
	case CMD_SHAKE_HAND:
		iResult = iCmd_Shake_Hand_Server(iSocket);
		break;
	case CMD_GET_FILE:
		iResult = iCmd_Get_File_Server(iSocket);
		break;
	default:
		break;
	}
	return;
}

void Listen(char IP[], int iPort, void* pCall_Back, int iTime_Out)
{//这个接口是简化操作，只留个Distribute_Cmd接口
	//iTime_Out： 延迟，毫秒
	//回调函数： void Distribute_Cmd(int iSocket)

	void (*pDistribute_Cmd)(int) = (void (*)(int))pCall_Back;
	int iSocket = iListen(IP, iPort);
	if (!iSocket)
		return;

	int iResult, iCounter = 0;
	if (iTime_Out)
		iResult = bSet_Non_Block(iSocket);	//设置非阻塞

	while (1)
	{
		int iSocket_Client = 0;
#ifdef WIN32
		int iLen = sizeof(sockaddr);
#else
		socklen_t  iLen = sizeof(sockaddr);
#endif

		sockaddr_in oClient;
		//注意，此处是有脾气的，Debug下反应很慢，大部分丢包
		//Release 下反应十分迅速
		iSocket_Client = (int)accept(iSocket, (sockaddr*)&oClient, &iLen);
		if (iSocket_Client < 0)
		{
			if(iCounter%100==0)
				printf("No request:%d\n",iCounter++);
			iSocket_Client = 0;
			Sleep(iTime_Out);
			continue;
		}
		//此处可以转到分发器分出去
		pDistribute_Cmd(iSocket_Client);
		Close_Connection(iSocket_Client, 1);
	}

	Close_Socket(iSocket);
#ifdef WIN32
	Free_Socket_Env();
#endif
}
int iSend_Buffer(int iSocket, unsigned char* pBuffer, int iSize)
{
	int iResult = iSendEx(iSocket, &iSize, 4);
	iResult = iSendEx(iSocket, pBuffer, iSize);
	return iResult;
}

int iRecv_Buffer(int iSocket, unsigned char** ppBuffer, int* piSize)
{
	int iSize, bRet = 1;
	int iResult = iRecvEx(iSocket, &iSize, sizeof(int));
	if (!iResult)
		return 0;

	//一旦分配了内存就不能短路退出，要goto END
	unsigned char* pBuffer = NULL;
	if (iSize)
	{
		pBuffer = (unsigned char*)pMalloc(iSize+1);
		if (!pBuffer)
			return 0;
		pBuffer[iSize] = 0;	//对于字符串，有用
	}else
		goto END;
		
	iResult = iRecvEx(iSocket, pBuffer, iSize);
	if (!iResult)
	{
		bRet = 0;
		goto END;
	}

END:
	if (!bRet)
		Free(pBuffer), pBuffer = NULL;
	*ppBuffer = pBuffer;
	*piSize = iSize;
	return bRet;
}

int iCmd_Capture_Client(int iSocket, unsigned char** ppBuffer, int* piSize, unsigned long long* ptTime_Stamp)
{//Time_Stamp: 微秒，不是毫秒，还得除以1000
	int iCmd = CMD_CAPTURE;
	int iResult = iSendEx(iSocket, &iCmd, sizeof(int));
	unsigned long long tTime_Stamp = 0;
	iResult = iRecvEx(iSocket, &tTime_Stamp, 8);
	iResult = iRecv_Buffer(iSocket, ppBuffer, piSize);
	if (!iResult)return 0;

	iResult = iReply_Recv(iSocket);
	if (!iResult && *ppBuffer)
		Free(*ppBuffer);
	if (ptTime_Stamp)
		*ptTime_Stamp = tTime_Stamp;
	return iResult;
}
int iCmd_Upload_File_Client(int iSocket, const char Local_File[], const char Remote_File[])
{//Upload 文件，并且顶文件名
	int iSize = 0;
	unsigned char* pBuffer = NULL;
	int iResult = bLoad_Raw_Data(Local_File, &pBuffer, &iSize);
	if (!iResult)
		return 0;

	int iCmd = CMD_UPLOAD_FILE;
	iResult = iSendEx(iSocket, &iCmd, 4);
	iResult = iSend_Buffer(iSocket, (unsigned char*)Remote_File, (int)strlen(Remote_File));
	iResult = iSend_Buffer(iSocket, pBuffer, iSize);
	if (pBuffer)
		Free(pBuffer);
	int bRet = 0;
	//上传会有个结果，从服务器回传
	iResult = iRecvEx(iSocket, &bRet, 4);
	iResult = iReply_Recv(iSocket);	//收到

	return bRet;
}

int iCmd_rd_Client(int iSocket, const char Path[])
{//返回：	0: 网络错误
//			-1: 服务器删除失败，可能文件不纯在等等
	int iSize = (int)strlen(Path);
	int iCmd = CMD_RD;
	int bRet, iResult = iSendEx(iSocket, &iCmd, 4);
	iResult = iSend_Buffer(iSocket, (unsigned char*)Path, iSize);
	iResult = iRecvEx(iSocket, &bRet, 4);
	if (!iResult)
		return 0;
	if (!bRet)
		return -1;
	iResult = iReply_Recv(iSocket);
	return 1;
}

int iCmd_md_Client(int iSocket, const char Path[])
{//返回：	0: 网络错误
//			-1: 服务器删除失败，可能文件不纯在等等
	int iSize = (int)strlen(Path);
	int iCmd = CMD_MD;
	int bRet, iResult = iSendEx(iSocket, &iCmd, 4);
	iResult = iSend_Buffer(iSocket,(unsigned char*)Path, iSize);
	iResult = iRecvEx(iSocket, &bRet, 4);
	if (!iResult)
		return 0;
	if (!bRet)
		return -1;
	iResult = iReply_Recv(iSocket);
	return 1;
}

int iCmd_Delete_File_Client(int iSocket, const char File[])
{//返回：	0: 网络错误
//			-1: 服务器删除失败，可能文件不纯在等等
	int iSize = (int)strlen(File);
	int iCmd = CMD_DELETE_FILE;
	int bRet, iResult = iSendEx(iSocket, &iCmd, 4);

	iResult = iSendEx(iSocket, &iSize, 4);
	iResult = iSendEx(iSocket, (char*)File, iSize);
	iResult = iRecvEx(iSocket, &bRet, 4);
	if (!iResult)
		return 0;
	if (!bRet)
		return -1;
	iResult = iReply_Recv(iSocket);
	return 1;
}

int iCmd_Get_File_List_Client(int iSocket, const char Path[], char **ppBuffer, int *piSize)
{//从服务器端拿一个目录的列表
	int iSize = (int)strlen(Path);
	int iCmd = CMD_GET_FILE_LIST, bRet = 1;
	int iResult = iSendEx(iSocket, &iCmd, sizeof(int));

	iResult = iSendEx(iSocket, (void*)&iSize, sizeof(int));
	iResult = iSendEx(iSocket, (void*)Path, iSize);

	unsigned char* pBuffer = NULL;
	iResult = iRecv_Buffer(iSocket, &pBuffer, &iSize);
	//iResult = iRecvEx(iSocket, &iSize, sizeof(int));
	//if (!iResult)
	//	return 0;

	////一旦分配了内存就不能短路退出，要goto END
	//unsigned char* pBuffer = (unsigned char*)pMalloc(iSize);
	//if (!pBuffer)
	//	return 0;

	//int bRet = 1;
	//iResult = iRecvEx(iSocket, pBuffer, iSize);
	if (!iResult)
	{
		bRet = 0;
		goto END;
	}
	iReply_Recv(iSocket);

	*ppBuffer = (char*)pBuffer;
	*piSize = iSize;
END:
	if (!bRet)
		Free(pBuffer);
	return bRet;
}
int iCmd_Rotate_Cam_Client(int iSocket, int iDir, float fDelta)
{//协议	
//Send:		iDir, fDelta	4字节
//Recv:		REPLY_RECV		三次握手，表示收完
	int iCmd = CMD_ROTATE_CAM;
	int iResult = iSendEx(iSocket, &iCmd, sizeof(int));
	iResult = iSendEx(iSocket, &iDir, sizeof(int));
	iResult = iSendEx(iSocket, &fDelta, sizeof(float));
	float fCur_Angle;
	iResult = iRecvEx(iSocket, &fCur_Angle, sizeof(int));
	if (iResult)
		printf("Cur Angle:%f\n", fCur_Angle);
	else
		printf("Fail to receive result\n");

	return iResult;
}
int iCmd_Set_Cam_Frame_Size_Client(int iSocket, int w, int h)
{//协议		
//Send		w,h	4字节
//Recv		Ret:1:0      4字节，成功与否
//Send		REPLY_RECV		三次握手，表示收完
	int iCmd = CMD_SET_CAM_FRAME_SIZE;
	int iResult = iSendEx(iSocket, &iCmd, sizeof(int));
	iResult = iSendEx(iSocket, &w, 4);
	iResult = iSendEx(iSocket, &h, 4);
	int iRet;
	iResult = iRecvEx(iSocket, &iRet, 4);
	if (!iResult)
		return 0;
	iResult = iReply_Recv(iSocket);
	return 1;
}

int iCmd_Get_File_Client(int iSocket, const char* pcRemove_File, const char* pcLocal_File)
{//从服务器端读取一个文件
	int iSize = (int)strlen(pcRemove_File);
	int iCmd = CMD_GET_FILE;
	int iResult = iSendEx(iSocket, &iCmd, sizeof(int));

	iResult = iSendEx(iSocket, (void*)&iSize, sizeof(int));
	iResult = iSendEx(iSocket, (void*)pcRemove_File, iSize);
	iResult = iRecvEx(iSocket, &iSize, sizeof(int));
	if (!iResult || iSize ==-1)
		return 0;
	//一旦分配了内存就不能短路退出，要goto END
	unsigned char* pBuffer = (unsigned char*)pMalloc(iSize);
	if (!pBuffer)
		return 0;

	int bRet = 1;
	iResult = iRecvEx(iSocket, pBuffer, iSize);
	if (!iResult)
	{
		bRet = 0;
		goto END;
	}
	iReply_Recv(iSocket);

	if (!bSave_Raw_Data(pcLocal_File, pBuffer, iSize))
	{
		printf("Fail to save File:%s\n", pcLocal_File);
		return 0;
	}
END:

	Free(pBuffer);
	return 1;
}
int iCmd_Get_File_List_Server(int iSocket)
{//获得文件目录
//协议：	iSize:		4字节
//			Path:		iSize 个字节	
// Response	iSize:		4字节	0 表示查无此文件
//			List_Content	iSize 个字节	
// Recv		REPLY_RECV		三次握手，表示收完
	int iMax_File_Len = 255;
	int iSize = 0;
#ifdef WIN32
	char Path[256] = "c:\\tmp";
#else
	char Path[256] = "/littlefs";
#endif
	char* pBuffer;
	int iResult = iRecv_Buffer(iSocket, (unsigned char**)&pBuffer, &iSize);
#ifdef WIN32
	sprintf(Path + strlen(Path), "%s\\*.*",pBuffer);
#else
	sprintf(Path + strlen(Path), "/%s", pBuffer);
#endif
	Free(pBuffer);
	pBuffer = pGet_File_List((const char*)Path,NULL, (int*)&iSize, 1);
	if (!pBuffer)
	{
		printf("Fail to Get File List\n");
		iSize = 0;
	}

	int bRet = 1;
	iResult = iSendEx(iSocket,&iSize, 4);
	if(iSize)
		iResult = iSendEx(iSocket,pBuffer, iSize);
	//三次握手，收确认
	iResult = iRecvEx(iSocket, &iSize, sizeof(int));
	if (!iResult)
		bRet = 0;

	/*static int iCounter = 0;
	if (bRet)
		printf("Get File List Successfully:%d\n", iCounter++);
	else
		printf("Fail to get File List\n");*/

	Free(pBuffer);
	return bRet;
}
int iCmd_Upload_File_Server(int iSocket)
{//从客户端上传一个文件到服务器
//协议：	iFile_Name_Size:	4字节
//			File_Name:			iFile_Name_Size个字节
//			iFile_Size			4字节
//			File_Content		iFile_Size个字节
//Response	Result:1,0			成功与否
//Recv		REPLY_RECV			三次握手，表示收完
#ifdef  ESP32
	char Path[256] = "/littlefs";
#else
	char Path[256] = "c:/tmp";
#endif
	int iMax_File_Name_Size = 255 - (int)strlen(Path) -2;
	int iSize,bRet=1;
	char* pFile_Name = NULL;
	unsigned char* pBuffer = NULL;
	int iResult = iRecv_Buffer(iSocket,(unsigned char**)&pFile_Name, &iSize);
	if (!iResult)
		return 0;
	if (iSize >= iMax_File_Name_Size)
	{
		bRet = 0;
		goto END;
	}
	iResult = iRecv_Buffer(iSocket, &pBuffer, &iSize);
	if (!iResult)
	{
		bRet = 0;
		goto END;
	}
	sprintf(Path, "%s/%s", (char*)Path, pFile_Name);
	iResult = bSave_Raw_Data(Path, pBuffer, iSize);
	if (iResult)
		printf("Upload file successfully\n");
	else
		printf("Fail to upload file\n");
	iResult = iSendEx(iSocket, &iResult, 4);
	iResult = iRecvEx(iSocket, &iResult, 4);
END:
	Free(pFile_Name);
	Free(pBuffer);
	return bRet;
}

int iCmd_rd_Server(int iSocket)
{//创建一个目录
//协议：	iSize: 4字节
//			Path_Name:		iSize 个字节	
//Response	iResult			4字节
//Recv		REPLY_RECV		三次握手，表示收完
#ifdef  ESP32
	char Path[256] = "/littlefs";
#else
	char Path[256] = "c:/tmp";
#endif
	int iSize, bRet = 1, iMax_Size = 256 - (int)strlen(Path) - 2;
	char* pPath_Name = NULL;
	int iResult = iRecv_Buffer(iSocket, (unsigned char**)&pPath_Name, &iSize);
	if (!iResult)
		return 0;
	if (iSize >= iMax_Size)
	{
		printf("File Name too long\n");
		bRet = 0;
		goto END;
	}
	sprintf(Path, "%s/%s", (char*)Path, pPath_Name);
	bRet = iDelete_Dir(Path);	//0为成功，  其他失败

	iResult = iSendEx(iSocket, &bRet, 4);
	iResult = iRecvEx(iSocket, &iResult, 4);
	iResult &= bRet;
END:
	if (pPath_Name)
		Free(pPath_Name);
	return iResult ? 1 : 0;
}

int iCmd_md_Server(int iSocket)
{//创建一个目录
//协议：	iSize: 4字节
//			Path_Name:		iSize 个字节	
//Response	iResult			4字节
//Recv		REPLY_RECV		三次握手，表示收完
#ifdef  ESP32
	char Path[256] = "/littlefs";
#else
	char Path[256] = "c:/tmp";
#endif
	int iSize, bRet = 1, iMax_Size = 256 - (int)strlen(Path) - 2;
	char* pPath_Name = NULL;
	int iResult = iRecv_Buffer(iSocket, (unsigned char**)&pPath_Name, &iSize);
	if (!iResult)
		return 0;
	if (iSize >= iMax_Size)
	{
		printf("File Name too long\n");
		bRet = 0;
		goto END;
	}
	sprintf(Path, "%s/%s", (char*)Path, pPath_Name);
	bRet = iCreate_Dir(Path);	//0为成功，  其他失败
	
	iResult = iSendEx(iSocket, &bRet, 4);
	iResult = iRecvEx(iSocket, &iResult, 4);
	iResult &= bRet;
END:
	if (pPath_Name)
		Free(pPath_Name);
	return iResult ? 1 : 0;
}
int iCmd_Delete_File_Server(int iSocket)
{//协议：	iSize: 4字节
//			File_Name:		iSize 个字节	
//Response	iResult			4字节
//Recv		REPLY_RECV		三次握手，表示收完
#ifdef  ESP32
	char Path[256] = "/littlefs";
#else
	char Path[256] = "c:/tmp";
#endif
	int iSize,bRet=1,iMax_Size = 256 - (int)strlen(Path) - 2;
	char* pFile_Name = NULL;
	int iResult = iRecv_Buffer(iSocket, (unsigned char**)&pFile_Name, &iSize);
	if (!iResult)
		return 0;
	if (iSize >= iMax_Size)
	{
		printf("File Name too long\n");
		bRet = 0;
		goto END;
	}
	sprintf(Path, "%s/%s", (char*)Path, pFile_Name);
	iResult = remove(Path);	//0为成功，  其他失败
	if (iResult)
		bRet = 0;
	iResult = iSendEx(iSocket, &bRet, 4);
	iResult = iRecvEx(iSocket, &iResult, 4);
	iResult &= bRet;
END:
	if (pFile_Name)
		Free(pFile_Name);
	return iResult ? 1 : 0;
}
int iCmd_Get_File_Server(int iSocket)
{//协议：	iSize: 4字节
//			File_Name:		iSize 个字节	
// Response	iSize:	4字节	0 表示查无此文件
//			File_Content	iSize 个字节	
// Recv		REPLY_RECV		三次握手，表示收完
	int iMax_File_Name_Len = 255;
	unsigned int iSize = 0;
	char* File_Name = NULL;
	int iResult = iRecv_Buffer(iSocket,(unsigned char**)&File_Name, (int*)&iSize);
	if (!iResult)
		return 0;

#ifdef WIN32
	char Path[256] = "c:\\tmp\\";
#else
	char Path[256] = "/littlefs/";
#endif
	int bRet = 1;
	unsigned char* pBuffer = NULL;
	if (strlen(File_Name) + strlen(Path) >= 256)
	{
		bRet = 0;
		goto END;
	}
	sprintf(Path, "%s%s", Path, File_Name);

	iResult = bLoad_Raw_Data(Path, &pBuffer, (int*)&iSize);
	if (!iResult)
	{
		iSize = 0xFFFFFFFF;
		iResult = iSendEx(iSocket, &iSize, sizeof(unsigned int));
		return 0;
	}

	iResult = iSendEx(iSocket, &iSize, sizeof(unsigned int));
	iResult = iSendEx(iSocket, pBuffer, iSize);
	if (!iResult)
	{
		bRet = 0;
		goto END;
	}

	//三次握手，收确认
	iResult = iRecvEx(iSocket, &iSize, sizeof(int));
	if (!iResult)
		bRet = 0;

	static int iCounter = 0;
	if (bRet)
		printf("Get File Successfully:%d\n", iCounter++);
	else
		printf("Fail to get File\n");

END:
	Free(File_Name);
	Free(pBuffer);
	return bRet;
}
/*****************************网络函数*************************************/

/*******************************简单队列****************************/
template<typename _T>void Init_Queue(Queue<_T>* poQueue, int iBuffer_Size)
{
	Queue<_T> oQueue;
	oQueue.m_iBuffer_Size = iBuffer_Size;
	oQueue.m_iCount = oQueue.m_iEnd = oQueue.m_iHead = 0;
	oQueue.m_pBuffer = (_T*)pMalloc(sizeof(_T) * iBuffer_Size);
	*poQueue = oQueue;
}
template<typename _T>void In_Queue(Queue<_T>* poQueue, _T oItem)
{
	if (poQueue->m_iEnd >= poQueue->m_iBuffer_Size)
	{
		printf("Insufficient Buffer in In_Queue\n");
		exit(0);
	}
	else
	{
		poQueue->m_pBuffer[poQueue->m_iEnd++] = oItem;
		poQueue->m_iEnd %= poQueue->m_iBuffer_Size;
		poQueue->m_iCount++;
	}
}
template<typename _T>_T Out_Queue(Queue<_T>* poQueue)
{
	if (!poQueue->m_iCount)
	{
		printf("No item in Queue\n");
		return 0;
	}
	else
	{
		int iQueue_Start = poQueue->m_iHead;
		poQueue->m_iHead = (iQueue_Start + 1) % poQueue->m_iBuffer_Size;
		poQueue->m_iCount--;
		return poQueue->m_pBuffer[iQueue_Start];
	}
}
template<typename _T>void Free_Queue(Queue<_T>* poQueue)
{
	if (poQueue->m_pBuffer)
		Free(poQueue->m_pBuffer);
	*poQueue = {};
}
/*******************************简单队列****************************/