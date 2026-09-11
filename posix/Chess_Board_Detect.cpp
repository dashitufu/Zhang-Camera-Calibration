#ifdef WIN32
	#include "Windows.h"
#endif
#include <algorithm>
#include "Chess_Board_Detect.h"
#include "Slam.h"

using namespace std;
typedef struct Contour_Tree_Node { //树形结构
	int m_iContour_ID;
	int m_iParent;
	int m_iFirst_Child;
	int m_iNext_Sibling;
}Contour_Tree_Node;

typedef struct Start_Count {
	int m_iContour_ID;
	int m_iStart;
	int m_iCount;
}Start_Count;

typedef struct Hash_Item {
	unsigned int A;
	unsigned int B;	//为减少数量，定义A<B
	//unsigned int m_iNeighbour_Count : 1;
	unsigned int m_iNext;
}Hash_Item;

//重做一堆结构，前面理解太肤浅
typedef struct Quad_Neighbour_Item {
	short m_iQuad_Index;
	short m_iCorner_Index;
	float m_fDist;
	void* ptr;				//该Neighbour的指针，可以直接访问过去
	//short m_iGroup_Index;	//缺省为-1，归入哪个组
}Quad_Neighbour_Item;

typedef struct Chess_Board_Corner {
	float Pos[2];
	unsigned int row;
	unsigned char m_iCount;
	Chess_Board_Corner* Neighbour[4];	//一个点与其他Quad的顶点形成的关系
}Chess_Board_Corner;

typedef struct Chess_Board_Quad {	//棋盘专用四元组
	union {
		unsigned short Corner_i[4][2];	//4个角点
		Chess_Board_Corner* Corner[4];
	};

	//int Neighbour[4];				//可以有4个邻居
	Quad_Neighbour_Item Neighbour[4];
	unsigned char m_iNeighbour_Count;			//邻居数
	unsigned char ordered;
	short m_iGroup_Index;	//缺省为-1，归入哪个组
	short row, col;

	float edge_sqr_len;				//边长平方
}Chess_Board_Quad;

typedef struct Term_Criteria {
	char m_iType;
	char m_iMax_Count;
	float eps;
}Term_Criteria;

typedef struct Line_Head {
	unsigned short* m_pStrip;
	unsigned char* m_pLine;
	unsigned short m_iCount;
	unsigned char m_iFirst_Color;
	//unsigned char m_iCur_Color;
}Line_Head;

template<typename _T> struct LM_Param_Ceres {//更复杂的LM方法，要存的数据可能更多
	unsigned char reuse_diagonal;	//初始化为什么，待考
	unsigned char bIsStepSuccessful;	//这一趟是否迭代成功
	unsigned char bStep_is_valid;
	unsigned short m_iIter;
	unsigned short num_consecutive_nonmonotonic_steps;
	_T radius;
	_T current_cost, reference_cost, candidate_cost, minimum_cost;

	_T decrease_factor;
	_T* m_pDiag;	//对角线元素
	_T accumulated_reference_model_cost_change;
	_T accumulated_candidate_model_cost_change;

	_T(*J)[2][15], (*Residual)[2];
	_T* H, * JtE;	//H矩阵
};

//*********************************第一部分，寻找连通域******************************************/
static void Init_Get_Contour_Info(Get_Contour_Info* poInfo, int iMax_Strip_Count, int iWidth, int iHeight)
{//初始化部分信息，Strip在这里开辟
	*poInfo = { NULL,NULL,NULL,(unsigned short)iWidth,(unsigned short)iHeight,iMax_Strip_Count };
	return;
}
static void Free_Contour_Info(Get_Contour_Info* poInfo)
{
	Get_Contour_Info oInfo = *poInfo;
	Free(oInfo.m_pContour);
	Free(oInfo.m_pLine);
	Free(oInfo.m_pStrip);
	*poInfo = {};
}

void Free_Contour_Result(Get_Contour_Result* poResult)
{
	Get_Contour_Result oResult = *poResult;
	if (oResult.m_pAll_Point)
		Free(oResult.m_pAll_Point);
	if (oResult.m_pContour)
		Free(oResult.m_pContour);
	*poResult = {};
}

void Line_2_Strip(unsigned char* pLine, int iWidth, unsigned short Strip[], int* piStrip_Count, unsigned char* piFirst_Color)
{
	//*piFirst_Color = !!pLine[0];
	unsigned char* pCur = pLine,
		* pEnd = pLine + iWidth;
	int iWidth_Minus_1 = iWidth - 1, iWidth_Plus_1 = iWidth + 1;

	int iCount = *piStrip_Count;
	Strip[iCount] = 0;		//覆盖上次哨兵
	
	if (pCur[0] != pCur[1])
	{
		if (!pCur[0] && pCur[-iWidth] && pCur[iWidth])	//黑色孤点
			pCur[0] = 0xFF;
		else
			Strip[++iCount] = 1;
		pCur++;
	}
	////这一步设置两个个哨兵，简化推进的难度
	unsigned char iSentinel_End = pLine[iWidth];	//iSentinel_Start = pLine[-1];
	pLine[iWidth] = !pLine[iWidth_Minus_1];
	*piFirst_Color = pLine[0];

	while (pCur < pEnd)
	{
		int iFlag = !!pCur[0];

		//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********
		unsigned long long  iFlag_1 = iFlag * 0xFFFFFFFFFFFFFFFF;
		while (*(unsigned long long*)pCur == iFlag_1)
			pCur += 8;
		//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********

		while (!!(*pCur) == iFlag)
			pCur++;

		//孤点形态				w
		//					  w	b w
		//						w
		if (!pCur[-1] &&			//当前点是黑点
			pCur[-2] &&				//前面点是白点
			pCur[-iWidth_Plus_1] &&		//上面为白点
			pCur[iWidth_Minus_1])			//下面点是白点
		{//有可能是孤点，看下一行的状况
			//是孤点了
			pCur[-1] = 0xFF;
			iCount--;
			continue;
		}
		//一个Strip OK了
		Strip[++iCount] = (int)(pCur - pLine);;
	}

	//恢复哨兵
	//pLine[-1] = iSentinel_Start;
	pLine[iWidth] = iSentinel_End;
	*piStrip_Count = iCount;
	return;
}

void Line_2_Strip_Bottom(unsigned char* pLine, int iWidth, unsigned short Strip[], int* piStrip_Count, unsigned char* piFirst_Color)
{//扫描顶行，看看有那些孤立点可以删除
//只删除黑色孤点，形如			1
//							  1	0 1
	int iWidth_Minus_1 = iWidth - 1;
	unsigned char* pCur = pLine,
		* pEnd = pLine + iWidth;

	//这一步设置一个哨兵，简化推进的难度
	unsigned char iSentinel_Org = pLine[iWidth];
	pLine[iWidth] = !pCur[iWidth_Minus_1];

	int iCount = *piStrip_Count;
	Strip[iCount] = 0;

	//第一点单独判断,单点先推进
	//						w
	//						b w
	if (pCur[0] != pCur[1])
	{
		if (!pCur[0] && pCur[-iWidth])
			pCur[0] = 0xFF;
		else
			Strip[++iCount] = 1;
		pCur++;
	}
	*piFirst_Color = !!pLine[0];

	while (pCur < pEnd)
	{
		int iFlag = !!pCur[0];

		//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********
		unsigned long long  iFlag_1 = iFlag * 0xFFFFFFFFFFFFFFFF;
		while (*(unsigned long long*)pCur == iFlag_1)
			pCur += 8;
		//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********

		while (!!(*pCur) == iFlag)
			pCur++;

		//孤点形态				w
		//					  w	b w
		if (!pCur[-1] &&				//当前点是黑点
			pCur[-2] &&				//前面点是白点
			pCur[-iWidth_Minus_1])			//上面为白点
		{//有可能是孤点，看下一行的状况
			//是孤点了
			*pCur = 0xFF;
			iCount--;
			continue;
		}
		//一个Strip OK了
		Strip[++iCount] = (int)(pCur - pLine);;
	}

	//恢复哨兵
	pLine[iWidth] = iSentinel_Org;
	*piStrip_Count = iCount;
	return;
}

void Line_2_Strip_Top(unsigned char* pLine, int iWidth, unsigned short Strip[], int* piStrip_Count, unsigned char* piFirst_Color)
{//扫描顶行，看看有那些孤立点可以删除
//只删除黑色孤点，形如		1 0 1
//							  1
	unsigned char* pCur = pLine,
		* pEnd = pLine + iWidth;
	int iWidth_Minus_1 = iWidth - 1;
	//这一步设置一个哨兵，简化推进的难度
	unsigned char iSentinel_Org = pLine[iWidth];
	pLine[iWidth] = !pCur[iWidth_Minus_1];
	int iCount = *piStrip_Count;
	Strip[0] = 0;

	////第一点单独判断,单点先推进
	if (pCur[0] != pCur[1])
	{
		if (!pCur[0] && pCur[iWidth])
			pCur[0] = 0xFF;
		else
			Strip[++iCount] = 1;
		pCur++;
	}
	*piFirst_Color = !!pLine[0];
	while (pCur < pEnd)
	{
		//unsigned char* pCur_1 = pCur;
		int iFlag = !!pCur[0];

		//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********
		unsigned long long  iFlag_1 = iFlag * 0xFFFFFFFFFFFFFFFF;
		while (*(unsigned long long*)pCur == iFlag_1)
			pCur += 8;
		//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********

		while (!!(*pCur) == iFlag)
			pCur++;

		//孤点形态				
		//					  w	b w
		//						w
		if (!pCur[-1] &&			//本店为黑
			pCur[-2] &&				//左边为白
			pCur[iWidth_Minus_1])	//下边为白
		{
			pCur[-1] = 0xFF;
			iCount--;
			continue;
		}
		//一个Strip OK了
		Strip[++iCount] = (unsigned short)(pCur - pLine);;
	}

	//恢复哨兵
	pLine[iWidth] = iSentinel_Org;
	*piStrip_Count = iCount;
	return;
}

static void Draw_Outline(Image oImage, Get_Contour_Result oResult, int iContour_ID)
{
	Get_Contour_Result::Contour oContour = oResult.m_pContour[iContour_ID];
	for (int i = 0; i < oContour.m_iOutline_Point_Count; i++)
	{
		unsigned short* pA = oContour.m_pPoint[i],
			* pB = oContour.m_pPoint[(i + 1) % oContour.m_iOutline_Point_Count];
		Draw_Line(oImage, pA[0], pA[1], pB[0], pB[1]);
	}
}
static void Draw_Outline(const char File[], Get_Contour_Result oResult, int iContour_ID)
{
	Image oImage;
	Init_Image(&oImage, oResult.m_iWidth, oResult.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	Draw_Outline(oImage, oResult, iContour_ID);
	bSave_Image(File, oImage);
	return;
}
static void Draw_All_Outline(const char File[], Get_Contour_Result oResult)
{
	Image oImage;
	Init_Image(&oImage, oResult.m_iWidth, oResult.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (int i = 0; i < oResult.m_iContour_Count; i++)
		Draw_Outline(oImage, oResult, i);
	bSave_Image(File, oImage);
	return;
}
static void Draw_Contour(const char* pcFile, Get_Contour_Info oInfo, int iContour_ID)
{//这是给第二阶段做了完整的索引，形成Contour, Line, Strip的完整指向才用
	Get_Contour_Info::Contour oContour = oInfo.m_pContour[iContour_ID];
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Strip oStrip;
	Image oImage;
	int i, j, k;
	Init_Image(&oImage, oInfo.m_iWidth, oInfo.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage, (!oContour.m_iBlack_or_White)*0xFF);

	//从以下代码可以看出，此时的strip已经不是按Line 分组，而是按contour 分组
	int iColor = oContour.m_iBlack_or_White * 0xFF;
	for (i = 0; i < oContour.m_iLine_Count; i++)
	{
		oLine = oInfo.m_pLine[oContour.m_iFirst_Line + i];
		for (j = 0; j < oLine.m_iStrip_Count; j++)
		{
			oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + j];
			for (k = oStrip.m_iStart; k <= oStrip.m_iEnd; k++)
				oImage.m_pChannel[0][oStrip.m_iY * oImage.m_iWidth + k] = iColor;
		}
	}
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
}

static int iGet_Max_Contour(Get_Contour_Result oResult, int iBlack_or_White = 1)
{
	int iMax = 0, iMax_Contour = -1;
	for (int i = 0; i < oResult.m_iContour_Count; i++)
	{
		if (iBlack_or_White == -1)
		{
			if (oResult.m_pContour[i].m_iOutline_Point_Count > iMax)
			{
				iMax = oResult.m_pContour[i].m_iArea;
				iMax_Contour = i;
			}
		}
		else
		{
			if (oResult.m_pContour[i].m_iOutline_Point_Count > iMax && oResult.m_pContour[i].m_iBlack_or_White == iBlack_or_White)
			{
				iMax = oResult.m_pContour[i].m_iArea;
				iMax_Contour = i;
			}
		}
	}
	//printf("Max Contour:%d Size:%d\n",iMax_Contour, iMax);
	return iMax_Contour;
}

int iGet_Max_Contour(Get_Contour_Info oInfo, int iBlack_or_White = 1)
{//iBlack_or_White： 白为1，黑为0
	int iMax = 0, iMax_Contour = -1;
	for (int i = 0; i < oInfo.m_iContour_Count; i++)
	{
		if (oInfo.m_pContour[i].m_iArea > (unsigned int)iMax && oInfo.m_pContour[i].m_iBlack_or_White == iBlack_or_White)
		{
			iMax = oInfo.m_pContour[i].m_iArea;
			iMax_Contour = i;
		}
	}
	printf("Max Contour:%d Size:%d\n", iMax_Contour, iMax);
	return iMax_Contour;
}

void Draw_Contour_Step_1(const char* pcFile, Get_Contour_Info oInfo, int iContour_ID)
{//第一阶段只抠出Contour，尚未形成Contour->Line->Strip索引可以用这 个函数
//此时，strip 按contour 分组，适用于最简形式
	Get_Contour_Info::Strip oStrip;
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Contour oContour = oInfo.m_pContour[iContour_ID];

	Image oImage;
	int y, i, j, iColor = oContour.m_iBlack_or_White * 0xFF;;
	Init_Image(&oImage, oInfo.m_iWidth, oInfo.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage,!iColor);
	for (y = 0; y < oInfo.m_iHeight; y++)
	{
		oLine = oInfo.m_pLine[y];
		for (i = 0; i < oLine.m_iStrip_Count; i++)
		{
			oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			if (oStrip.m_iContour_ID == iContour_ID)
			{
				for (j = oStrip.m_iStart; j <= oStrip.m_iEnd; j++)
					oImage.m_pChannel[0][y * oImage.m_iWidth + j] = iColor;
			}
		}
	}
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
}

static void Draw_Max_Contour(const char* pcFile, Get_Contour_Info oInfo, int iColor = 1, int iStep = 1)
{
	int iContour_ID = iGet_Max_Contour(oInfo, iColor);
	if (iStep == 1)
		Draw_Contour_Step_1(pcFile, oInfo, iContour_ID);
	else
		Draw_Contour(pcFile, oInfo, iContour_ID);
	return;
}

void Draw_Strip(Image oImage, Get_Contour_Info oInfo)
{
	for (int y = 0; y < oImage.m_iHeight; y++)
	{
		Get_Contour_Info::Line oLine = oInfo.m_pLine[y];
		for (int i = 0; i < oLine.m_iStrip_Count; i++)
		{
			Get_Contour_Info::Strip oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			int iStart = y * oImage.m_iWidth + oStrip.m_iStart;
			memset(&oImage.m_pChannel[0][iStart], oStrip.m_iBlack_or_White * 0xFF, oStrip.m_iEnd - oStrip.m_iStart + 1);
		}
	}
	return;
}

void Draw_Strip(const char File[],Get_Contour_Info oInfo)
{
	Image oImage;
	Init_Image(&oImage, oInfo.m_iWidth, oInfo.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Draw_Strip(oImage, oInfo);
	bSave_Image(File, oImage);
	Free_Image(&oImage);
	return;
}

static  int bGet_Strip_Org(Get_Contour_Info* poInfo, Image oAlpha)
{//扫描整个图片，将所有的Strip找出来
	Get_Contour_Info oInfo = *poInfo;
	Get_Contour_Info::Line oCur_Line;
	Get_Contour_Info::Strip oStrip;

	const unsigned long long iPattern_White = { 0xFFFFFFFFFFFFFFFF },
		iPattern_Black = 0;
	oInfo.m_pLine = (Get_Contour_Info::Line*)pMalloc(oInfo.m_iHeight * sizeof(Get_Contour_Info::Line));
	oInfo.m_pStrip = (Get_Contour_Info::Strip*)pMalloc(oInfo.m_iMax_Strip_Count * sizeof(Get_Contour_Info::Strip));

	if (!oInfo.m_pStrip || !oInfo.m_pLine)
	{
		//Disp_Mem();
		printf("bGet_Strip Error, Fail to allocated memeory\n");
		if (oInfo.m_pStrip)Free(oInfo.m_pStrip);
		if (oInfo.m_pLine)Free(oInfo.m_pLine);
		return 0;
	}

	unsigned char* pCur, * pCur_End;
	int y, x, iStrip_Count, iCur_Strip_Of_Previous_Line, bRet = 0;;
	pCur = oAlpha.m_pChannel[0];
	pCur_End = pCur + oAlpha.m_iWidth - 8;

	for (y = 0; y < oAlpha.m_iHeight; y++, pCur += oAlpha.m_iWidth, pCur_End += oAlpha.m_iWidth)
	{
		iStrip_Count = 0;
		oCur_Line.m_iFirst_Strip = oInfo.m_iStrip_Count;
		iCur_Strip_Of_Previous_Line = 0;

		for (x = 0; x < oAlpha.m_iWidth;)
		{
			unsigned char* pCur_1 = &pCur[x];
			int iFlag = !!pCur[x];

			//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********
			unsigned long long  iFlag_1 = iFlag * 0xFFFFFFFFFFFFFFFF;
			while (pCur_1 < pCur_End && iFlag_1 == *(unsigned long long*)pCur_1)
				pCur_1 += 8;
			//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********

			while (!!(*pCur_1) == iFlag && pCur_1 < pCur_End + 8)
				pCur_1++;

			oStrip.m_iStart = x;
			oStrip.m_iEnd = (unsigned short)(pCur_1 - pCur - 1);
			oStrip.m_iContour_ID = 0xFFFFFF;
			oStrip.m_iY = y;	//省不了，因为还有后面的勾边
			oStrip.m_iBlack_or_White = iFlag;
			oInfo.m_pStrip[oInfo.m_iStrip_Count++] = oStrip;

			if (oInfo.m_iStrip_Count >= oInfo.m_iMax_Strip_Count)
			{
				printf("Insufficient strip allocated\n");
				goto END;
			}
			x = oStrip.m_iEnd + 1;
		}
		oCur_Line.m_iStrip_Count = oInfo.m_iStrip_Count - oCur_Line.m_iFirst_Strip;
		oInfo.m_pLine[y] = oCur_Line;
	}

	Shrink(oInfo.m_pStrip, oInfo.m_iStrip_Count * sizeof(Get_Contour_Info::Strip));
	bRet = 1;
END:
	if (!bRet)
		Free_Contour_Info(&oInfo);
	*poInfo = oInfo;
	return bRet;
}

static int bGet_Strip(Get_Contour_Info* poInfo, Image oAlpha, int iOrphan_Len = 5, int iOrphan_Gap = 1)
{//实践证明，没有必要在strip级上做太多的清理，并没有清理出太多数据
//此处只清理孤立黑点
//返回： 本函数只有两种结果，	1：成功
//								0：内存不够
	Get_Contour_Info oInfo = *poInfo;
	Get_Contour_Info::Strip* pStrip_1 = NULL;
	int bRet = 0, iHeight_Minus_1 = oAlpha.m_iHeight - 1;
	oInfo.m_pLine = (Get_Contour_Info::Line*)pMalloc(oInfo.m_iHeight * sizeof(Get_Contour_Info::Line));
	unsigned short* pStrip = (unsigned short*)pMalloc((oInfo.m_iMax_Strip_Count + (oInfo.m_iWidth + 1) * 3) * sizeof(unsigned short));
	if (!oInfo.m_pLine || !pStrip)
	{
		printf("Insufficient stript in bGet_Strip\n");
		goto END;
	}

	int iStrip_Count, iPre_Strip_Count,y;
	unsigned char iFirst_Color;
	iStrip_Count = iPre_Strip_Count = 0;

	//先扫顶行，删除孤点
	Line_2_Strip_Top(oAlpha.m_pChannel[0], oAlpha.m_iWidth, pStrip, &iStrip_Count, &iFirst_Color);
	oInfo.m_pLine[0] = { 0,(unsigned short)(iStrip_Count - iPre_Strip_Count),iFirst_Color };
	iPre_Strip_Count = iStrip_Count;

	for (y = 1; y < iHeight_Minus_1; y++)
	{
		Line_2_Strip(&oAlpha.m_pChannel[0][y * oAlpha.m_iWidth], oAlpha.m_iWidth, pStrip, &iStrip_Count, &iFirst_Color);
		oInfo.m_pLine[y] = { (unsigned int)iPre_Strip_Count,(unsigned short)(iStrip_Count - iPre_Strip_Count),iFirst_Color };
		iPre_Strip_Count = iStrip_Count;
	}

	oAlpha.m_pChannel[0][y * oAlpha.m_iWidth +1] = 0xFF;
	Line_2_Strip_Bottom(&oAlpha.m_pChannel[0][y * oAlpha.m_iWidth], oAlpha.m_iWidth, pStrip, &iStrip_Count, &iFirst_Color);
	oInfo.m_pLine[y] = { (unsigned int)iPre_Strip_Count,(unsigned short)(iStrip_Count - iPre_Strip_Count),iFirst_Color };

	Shrink(pStrip, iStrip_Count * sizeof(unsigned short));
	//************************扫描全图生成短款strip*************************/

	//*************将短款strip 变成长款strip*********************************/
	if (!bExpand(pStrip, iStrip_Count * sizeof(Get_Contour_Info::Strip)))
		pStrip_1 = (Get_Contour_Info::Strip*)pMalloc(iStrip_Count * sizeof(Get_Contour_Info::Strip));
	else
		pStrip_1 = (Get_Contour_Info::Strip*)pStrip;

	int iTotal;
	iTotal = 0;
	for (y = oAlpha.m_iHeight - 1; y >= 0; y--)
	{
		Get_Contour_Info::Line oLine = oInfo.m_pLine[y];
		unsigned short iColor = oLine.m_iBlack_or_White ^ ((oLine.m_iCount - 1) & 1);
		int iLast_Strip_of_Line = oLine.m_iFirst_Strip + oLine.m_iCount - 1;
		for (int i = iLast_Strip_of_Line; i >= (int)oLine.m_iFirst_Strip; i--, iColor = !iColor)
		{
			Get_Contour_Info::Strip oStrip = { pStrip[i],
				(unsigned short)(i == iLast_Strip_of_Line ? oAlpha.m_iWidth - 1 : pStrip[i + 1] - 1),
				0xFFFFFF,(unsigned int)iColor,
				(unsigned short)y };
			pStrip_1[i] = oStrip;
		}
		oInfo.m_pLine[y].m_iStrip_Count = oLine.m_iCount;
		iTotal += oLine.m_iCount;
	}
	oInfo.m_pStrip = pStrip_1;
	oInfo.m_iStrip_Count = iStrip_Count;
	//*************将短款strip 变成长款strip*********************************/

	bRet = 1;	//最后所有多干完了，返回值为1
END:
	if ((void*)pStrip != (void*)pStrip_1)
		Free(pStrip);

	if (!bRet)
	{
		Free_Contour_Info(&oInfo);
		Free(pStrip);
	}

	*poInfo = oInfo;
	return bRet;
}

//这个已经毫无利用价值
//static int bGet_Strip_2(Get_Contour_Info* poInfo, Image oAlpha, int iOrphan_Len = 5, int iOrphan_Gap = 1)
//{//实践证明，没有必要在strip级上做太多的清理，并没有清理出太多数据
//	//扫描整个图片，将所有的Strip找出来
////返回： 本函数只有两种结果，	1：成功
////								0：内存不够
//	Get_Contour_Info oInfo = *poInfo;
//	Get_Contour_Info::Strip* pStrip_1 = NULL;
//
//	//Get_Contour_Info::Strip oStrip;
//	int bRet = 0;
//
//	//在行推进寻找下一个strip的快速判断全黑全白pattern
//	const unsigned long long iPattern_White = { 0xFFFFFFFFFFFFFFFF },
//		iPattern_Black = 0;
//	oInfo.m_pLine = (Get_Contour_Info::Line*)pMalloc(oInfo.m_iHeight * sizeof(Get_Contour_Info::Line));
//	unsigned short * pStrip = (unsigned short*)pMalloc((oInfo.m_iMax_Strip_Count + (oInfo.m_iWidth + 1) * 3) * sizeof(unsigned short));
//	if (!oInfo.m_pLine || !pStrip)
//	{
//		printf("Insufficient stript in bGet_Strip\n");
//		goto END;
//	}
//		
//	//************************扫描全图生成短款strip*************************/
//	Line_Head Head[3];
//	for (int i = 0; i < 3; i++)
//	{//先找做三行
//		Head[i] = { &pStrip[oInfo.m_iMax_Strip_Count + i * oInfo.m_iWidth],&oAlpha.m_pChannel[0][i * oAlpha.m_iWidth], 0 };
//		Line_2_Strip(&oAlpha.m_pChannel[0][i * oAlpha.m_iWidth], oAlpha.m_iWidth, Head[i].m_pStrip, &Head[i].m_iCount, &Head[i].m_iFirst_Color);
//	}
//
//	//Image oTemp;
//	//Init_Image(&oTemp, oAlpha.m_iWidth, 3, Image::IMAGE_TYPE_BMP, 8);
//	//memcpy(oTemp.m_pChannel[0], oAlpha.m_pChannel[0], oAlpha.m_iWidth * 2);
//
//	int iStrip_Count;
//	iStrip_Count= 0;
//	Remove_Orphan_1(Head[0].m_pStrip, Head[0].m_iFirst_Color, &Head[0].m_iCount,
//		Head[1].m_pStrip, Head[1].m_iFirst_Color, Head[0].m_pLine,
//		iOrphan_Len, iOrphan_Gap, pStrip, &iStrip_Count);
//		
//	oInfo.m_pLine[0] = { 0,Head[0].m_iCount,Head[0].m_iFirst_Color};
//	//printf("Line:%d First_Color:%d\n", 0, oInfo.m_pLine[0].m_iBlack_or_White);
//
//	for (int y = 2; y < oInfo.m_iHeight; y++)
//	{
//		Head[2].m_pLine = &oAlpha.m_pChannel[0][y * oAlpha.m_iWidth];
//		Line_2_Strip(Head[2].m_pLine, oAlpha.m_iWidth, Head[2].m_pStrip, &Head[2].m_iCount, &Head[2].m_iFirst_Color);
//		
//		if (iStrip_Count + Head[1].m_iCount >= oInfo.m_iMax_Strip_Count)
//			goto END;
//		Remove_Orphan_2(Head, iOrphan_Len, iOrphan_Gap, pStrip, &iStrip_Count);
//			
//		oInfo.m_pLine[y - 1] = { (unsigned int)(iStrip_Count - Head[1].m_iCount),Head[1].m_iCount, Head[1].m_iFirst_Color};
//		//printf("Start:%d Count:%d\n", oInfo.m_pLine[y - 1].m_iFirst_Strip, oInfo.m_pLine[y - 1].m_iStrip_Count);
//		Line_Head oHead = Head[0];
//		Head[0] = Head[1];
//		Head[1] = Head[2];
//		Head[2] = oHead;
//		//printf("Line:%d First_Color:%d\n", y-1, oInfo.m_pLine[y-1].m_iBlack_or_White);
//	}
//	Remove_Orphan_1(Head[1].m_pStrip, Head[1].m_iFirst_Color, &Head[1].m_iCount,
//		Head[0].m_pStrip, Head[0].m_iFirst_Color, Head[1].m_pLine,
//		iOrphan_Len, iOrphan_Gap, pStrip, &iStrip_Count);
//
//	oInfo.m_pLine[oAlpha.m_iHeight - 1] = {(unsigned int)(iStrip_Count - Head[1].m_iCount),Head[1].m_iCount,  Head[1].m_iFirst_Color};
//	Shrink(pStrip, iStrip_Count * sizeof(unsigned short));
//	//printf("Line:%d First_Color:%d\n", oAlpha.m_iHeight - 1, oInfo.m_pLine[oAlpha.m_iHeight - 1].m_iBlack_or_White);
//	//************************扫描全图生成短款strip*************************/
//
//	//*************将短款strip 变成长款strip*********************************/
//	if (!bExpand(pStrip, iStrip_Count * sizeof(Get_Contour_Info::Strip)))
//		pStrip_1 = (Get_Contour_Info::Strip*)pMalloc(iStrip_Count * sizeof(Get_Contour_Info::Strip));
//	else
//		pStrip_1 = (Get_Contour_Info::Strip*)pStrip;
//	
//	int iTotal, y;
//	iTotal = 0;
//	for (y = oAlpha.m_iHeight - 1; y >= 0; y--)
//	{
//		Get_Contour_Info::Line oLine = oInfo.m_pLine[y];
//		unsigned short iColor = oLine.m_iBlack_or_White ^ ((oLine.m_iCount - 1) & 1);
//		int iLast_Strip_of_Line = oLine.m_iFirst_Strip + oLine.m_iCount - 1;
//		for (int i = iLast_Strip_of_Line; i >= (int)oLine.m_iFirst_Strip; i--, iColor = !iColor)
//		{
//			//if (i == 65947)
//				//printf("Here");
//
//			Get_Contour_Info::Strip oStrip = { pStrip[i],
//				(unsigned short)(i == iLast_Strip_of_Line ? oAlpha.m_iWidth - 1 : pStrip[i + 1] - 1),
//				0xFFFFFF,(unsigned int)iColor,
//				(unsigned short)y };
//			pStrip_1[i] = oStrip;
//		}
//		oInfo.m_pLine[y].m_iStrip_Count = oLine.m_iCount;
//		iTotal += oLine.m_iCount;
//	}
//	oInfo.m_pStrip = pStrip_1;
//	oInfo.m_iStrip_Count = iStrip_Count;
//	//*************将短款strip 变成长款strip*********************************/
//
//	//Draw_Strip("c:\\tmp\\1.bmp", oInfo);
//	//bSave_Image("c:\\tmp\\2.bmp", oAlpha);
//	//Compare_Image("c:\\tmp\\1.bmp", "c:\\tmp\\2.bmp");
//
//	bRet = 1;	//最后所有多干完了，返回值为1
//END:
//	if ((void*)pStrip != (void*)pStrip_1)
//		Free(pStrip);
//
//	if (!bRet)
//	{
//		Free_Contour_Info(&oInfo);
//		Free(pStrip);
//	}
//
//	*poInfo = oInfo;
//	return bRet;
//}

static  int bConnect(Get_Contour_Info::Strip oA, Get_Contour_Info::Strip oB)
{//判断上下两个Strip是否链接
	if (oA.m_iBlack_or_White != oB.m_iBlack_or_White)
		return 0;
	if (oA.m_iBlack_or_White)
	{//白色用八连通域
		if (oA.m_iEnd + 1 < oB.m_iStart)
			return 0;
		else if (oB.m_iEnd + 1 < oA.m_iStart)
			return 0;
	}
	else
	{//黑色4连通域
		if (oA.m_iEnd < oB.m_iStart)
			return 0;
		else if (oB.m_iEnd < oA.m_iStart)
			return 0;
	}
	return 1;
}
static int bFind_Node(Get_Contour_Info::Contour* pContour, int iLink_Start, int iTo_Find)
{//顺Link而上，尝试找一个与iTo_Find相同ID的Contour
	int iCur = iLink_Start;
	while (1)
	{
		if (iCur == iTo_Find)
			return 1;
		if (pContour[iCur].m_iPart_Of == 0xFFFFFF)
			break;
		iCur = pContour[iCur].m_iPart_Of;
	}
	return 0;
}

static void Adjust_Link(Get_Contour_Info::Contour* pContour, int iLink_Start, int iRoot)
{//将整条链的所有节点指向iRoot，注意，这个动作并不能把所有的Contour Link都调整到
//两个，因为有些形态就是以很奇怪的轨迹找到父节点的。最明显的是Contour 0, 但是，这个
//动作能让凡是需要找的Link都不至于太长
	int iCur = iLink_Start;
	Get_Contour_Info::Contour oCur;
	while (1)
	{
		oCur = pContour[iCur];
		if (iCur != iRoot)
			pContour[iCur].m_iPart_Of = iRoot;
		if (oCur.m_iPart_Of != 0xFFFFFF)
			iCur = oCur.m_iPart_Of;
		else
			break;
	}
}

static int iFind_Match_Contour_1(Get_Contour_Info* poInfo, Get_Contour_Info::Line oPrevious_Line, Get_Contour_Info::Strip oStrip, int* piCur_Strip_Of_Previous_Line)
{//尝试简化搜索
 //先从上一条线推进到第一个与oStrip有交集的strip
	Get_Contour_Info oInfo = *poInfo;
	Get_Contour_Info::Strip oStrip_More, oPrevious_Strip;
	int i, iRoot = -1;	// , iSub_Root = -1;
	int iResult;
	for (iResult = 0, i = *piCur_Strip_Of_Previous_Line; i < (int)oPrevious_Line.m_iStrip_Count; i++)
	{//一路向右找到上面有线段与本线段有交集或者超出本线段最右为止
		oPrevious_Strip = oInfo.m_pStrip[oPrevious_Line.m_iFirst_Strip + i];
		if (bConnect(oStrip, oPrevious_Strip))
		{
			iRoot = oPrevious_Strip.m_iContour_ID;
			//int iNode_Count = 0;
			while (oInfo.m_pContour[iRoot].m_iPart_Of != 0xFFFFFF)
				iRoot = oInfo.m_pContour[iRoot].m_iPart_Of;	// , iNode_Count++;
			//这就恶心了，加了调整更慢
			//if (iNode_Count > 5)
				//Adjust_Link(oInfo.m_pContour, oPrevious_Strip.m_iContour_ID, iRoot);
			iResult = 1;
			break;
		}
		else
		{//分开黑白两种情况
			if (oStrip.m_iBlack_or_White)
			{
				if (oPrevious_Strip.m_iEnd >= oStrip.m_iEnd + 1)
					break;
			}
			else if (oPrevious_Strip.m_iEnd >= oStrip.m_iEnd)
				break;
		}
	}

	if (iResult && oPrevious_Strip.m_iEnd < oStrip.m_iEnd)
	{
		//第二步，横扫过去
		for (i++; i < (int)oPrevious_Line.m_iStrip_Count; i++)
		{
			oStrip_More = oInfo.m_pStrip[oPrevious_Line.m_iFirst_Strip + i];
			if (iResult = bConnect(oStrip, oStrip_More))
			{//可以向上修改
				int iCur = oStrip_More.m_iContour_ID;
				if (!bFind_Node(oInfo.m_pContour, iCur, iRoot))
					Adjust_Link(oInfo.m_pContour, iCur, iRoot);
			}

			//分黑白两种情况判断跳出条件
			if (oStrip.m_iBlack_or_White)
			{
				if (oStrip_More.m_iEnd >= oStrip.m_iEnd + 1)
					break;
			}
			else if (oStrip_More.m_iEnd >= oStrip.m_iEnd)
				break;

			//原来的方法不再适用，因为黑白的连通数不一样
			//if (oStrip_More.m_iEnd >= oStrip.m_iEnd)
			//break;
		}
	}
	*piCur_Strip_Of_Previous_Line = i;
	return iRoot;
}
static void Adjust_Contour(Get_Contour_Info* poInfo)
{//1,将所有子Contour的计数加到主Contour中去
	Get_Contour_Info oInfo = *poInfo;
	int i, iRoot;
	Get_Contour_Info::Contour oCur, oRoot, * poParent, * poCur, * poEnd, * poRoot;

	poCur = oInfo.m_pContour;
	poEnd = poCur + oInfo.m_iContour_Count;

	for (; poCur < poEnd; poCur++)
	{
		oCur = *poCur;
		if (oCur.m_iPart_Of != 0xFFFFFF /*&& oCur.m_iCount*/)
		{
			poRoot = &oInfo.m_pContour[oCur.m_iPart_Of];
			while (poRoot->m_iPart_Of != 0xFFFFFF)
				poRoot = &oInfo.m_pContour[poRoot->m_iPart_Of];
			iRoot = (int)(poRoot - oInfo.m_pContour);

			poParent = &oInfo.m_pContour[oCur.m_iPart_Of];
			oRoot = *poRoot;
			while (poParent != poRoot)
			{
				oRoot.m_iArea += poParent->m_iArea;
				poParent->m_iArea = 0;
				int iPart_of = poParent->m_iPart_Of;
				poParent->m_iPart_Of = iRoot;
				//poParent = &oInfo.m_pContour[poParent->m_iPart_Of];
				poParent = &oInfo.m_pContour[iPart_of];
			}

			//**********此处Fix了一个小Bug,算得更准*********
			oRoot.m_iArea += oCur.m_iArea;
			*poRoot = oRoot;

			oCur.m_iPart_Of = iRoot;
			oCur.m_iArea = 0;
			*poCur = oCur;
		}
	}

	//修改所有的strip
	for (i = 0; i < oInfo.m_iStrip_Count; i++)
	{
		if (oInfo.m_pContour[oInfo.m_pStrip[i].m_iContour_ID].m_iArea == 0)
			oInfo.m_pStrip[i].m_iContour_ID = oInfo.m_pContour[oInfo.m_pStrip[i].m_iContour_ID].m_iPart_Of;
	}

	//搞一个紧凑的Controu，需要映射pMap[iOrg_Pos]就是原来iPos的位置的新位置
	//剩下的全部都是Part_Of=0xFFFFFF，即全为根
	int* pMap = (int*)pMalloc(oInfo.m_iContour_Count * sizeof(int));
	int j;
	for (i = j = 0; i < oInfo.m_iContour_Count; i++)
	{
		if (oInfo.m_pContour[i].m_iArea)
		{
			oInfo.m_pContour[j] = oInfo.m_pContour[i];
			pMap[i] = j++;
		}
		else
			pMap[i] = -1;
	}
	oInfo.m_iContour_Count = j;

	//再修改Strip
	for (i = 0; i < oInfo.m_iStrip_Count; i++)
		oInfo.m_pStrip[i].m_iContour_ID = pMap[oInfo.m_pStrip[i].m_iContour_ID];

	Free(pMap);
	Shrink(oInfo.m_pContour, oInfo.m_iContour_Count * sizeof(Get_Contour_Info::Contour));

	*poInfo = oInfo;
	return;
}
static int bGet_Contour(Get_Contour_Info* poInfo, int iHeight)
{//此处根据strip生成所有的连通域
	Get_Contour_Info oInfo = *poInfo;
	Get_Contour_Info::Line oLine, oPrevious_Line = { 0 };
	Get_Contour_Info::Strip oStrip;
	Get_Contour_Info::Contour oContour;  //= { 0,-1 };;
	int y, i, iCur_Strip_Of_Previous_Line;
	oContour.m_iArea = 0;
	oContour.m_iPart_Of = 0xFFFFFF;
	oInfo.m_iMax_Domain_Count = oInfo.m_iStrip_Count/2;
	oInfo.m_pContour = (Get_Contour_Info::Contour*)pMalloc(oInfo.m_iMax_Domain_Count * sizeof(Get_Contour_Info::Contour));
	//Disp_Mem();
	if (!oInfo.m_pContour)
	{
		printf("Fail to allocate memory in Get_Contour\n");
		return 0;
	}
	memset(oInfo.m_pContour, 0, oInfo.m_iMax_Domain_Count * sizeof(Get_Contour_Info::Contour));
	int bRet = 0;
	for (y = 0; y < iHeight; y++)
	{
		if (y > 0)
			oPrevious_Line = oInfo.m_pLine[y - 1];
		oLine = oInfo.m_pLine[y];
		iCur_Strip_Of_Previous_Line = 0;
		for (i = 0; i < oLine.m_iStrip_Count; i++)
		{
			oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			oStrip.m_iContour_ID = iFind_Match_Contour_1(&oInfo, oPrevious_Line, oStrip, &iCur_Strip_Of_Previous_Line);
			if (oStrip.m_iContour_ID == 0xFFFFFF)
			{//查无此Domain,加
				if (oInfo.m_iContour_Count >= oInfo.m_iMax_Domain_Count)
				{
					printf("Too many domain\n");
					goto END;
				}
				oStrip.m_iContour_ID = oInfo.m_iContour_Count;
				oContour.m_iBlack_or_White = oStrip.m_iBlack_or_White;
				oInfo.m_pContour[oInfo.m_iContour_Count++] = oContour;
			}
			oInfo.m_pContour[oStrip.m_iContour_ID].m_iArea += oStrip.m_iEnd - oStrip.m_iStart + 1;
			oInfo.m_pStrip[oLine.m_iFirst_Strip + i] = oStrip;
		}
		//printf("y:%d %d\n",y, oInfo.m_iContour_Count);
	}

	Shrink(oInfo.m_pContour, oInfo.m_iContour_Count * sizeof(Get_Contour_Info::Contour));
	Adjust_Contour(&oInfo);
	bRet = 1;
END:
	//Disp_Mem();
	*poInfo = oInfo;
	return bRet;
}

static int bPrepare_Outline(Get_Contour_Info* poInfo)
{
	Get_Contour_Info oInfo = *poInfo;
	//对strip扫描一遍寻找每个contour的行开始与行结束，多少个Strip
	Get_Contour_Info::Strip oStrip;
	Get_Contour_Info::Contour oContour;
	Get_Contour_Info::Strip* pNew_Strip = NULL;
	Get_Contour_Info::Line* pLine_Strip_Map = NULL;

	//[0]: Top	[1]: Bottom		[2]: Strip Count
	unsigned int* pCur_Contour, (*pContour_Strip)[3] = (unsigned int(*)[3])pMalloc(oInfo.m_iContour_Count * 3 * sizeof(unsigned int));
	int i, bRet = 1;
	if (!pContour_Strip)
		goto END;

	for (i = 0; i < oInfo.m_iContour_Count; i++)
	{
		pContour_Strip[i][0] = 0xFFFFFFFF;
		pContour_Strip[i][1] = pContour_Strip[i][2] = 0;
	}

	//先找到每个连通域的高，低，strip数
	for (i = 0; i < oInfo.m_iStrip_Count; i++)
	{
		oStrip = oInfo.m_pStrip[i];
		pCur_Contour = pContour_Strip[oStrip.m_iContour_ID];

		//以下这样行不行，还得看最后结果，但上面真心很傻逼
		pCur_Contour[1] = oStrip.m_iY;
		if (pCur_Contour[0] == 0xFFFFFFFF)
			pCur_Contour[0] = oStrip.m_iY;

		pCur_Contour[2]++;
	}

	//确定所有Contour要多少行信息
	int iLine_Count;
	iLine_Count = 0;
	for (i = 0; i < oInfo.m_iContour_Count; i++)
	{
		Get_Contour_Info::Contour* poContour = &oInfo.m_pContour[i];
		pCur_Contour = pContour_Strip[i];
		poContour->m_iLine_Count = pCur_Contour[1] - pCur_Contour[0] + 1;
		poContour->m_iFirst_Line = iLine_Count;
		poContour->m_iY_Start = pCur_Contour[0];
		iLine_Count += poContour->m_iLine_Count;
	}

	////建立一个Contour->Line表, [Contour]可得到两个信息 [0]:行开始，[1]行数
	pLine_Strip_Map = (Get_Contour_Info::Line*)pMalloc(iLine_Count * sizeof(Get_Contour_Info::Line));
	memset(pLine_Strip_Map, 0, iLine_Count * sizeof(Get_Contour_Info::Line));
	for (i = 0; i < oInfo.m_iStrip_Count; i++)
	{
		oStrip = oInfo.m_pStrip[i];
		oContour = oInfo.m_pContour[oStrip.m_iContour_ID];
		//算从当前strip到当前contour的顶的相对位置
		int iOffset = oStrip.m_iY - pContour_Strip[oStrip.m_iContour_ID][0];
		pLine_Strip_Map[oContour.m_iFirst_Line + iOffset].m_iStrip_Count++;
	}

	int iStrip_Count;
	iStrip_Count = 0;
	for (i = 1; i < iLine_Count; i++)
	{
		iStrip_Count += pLine_Strip_Map[i - 1].m_iStrip_Count;
		pLine_Strip_Map[i].m_iFirst_Strip = iStrip_Count;
		pLine_Strip_Map[i - 1].m_iStrip_Count = 0;	//后i面有用，所以又重新置零，很傻但必须
	}

	pNew_Strip = (Get_Contour_Info::Strip*)pMalloc(oInfo.m_iStrip_Count * sizeof(Get_Contour_Info::Strip));
	if (!pNew_Strip)
		goto END;

	memset(pNew_Strip, 0, oInfo.m_iStrip_Count * sizeof(Get_Contour_Info::Strip));

	pLine_Strip_Map[i - 1].m_iStrip_Count = 0;
	for (i = 0; i < oInfo.m_iStrip_Count; i++)
	{
		oStrip = oInfo.m_pStrip[i];
		oContour = oInfo.m_pContour[oStrip.m_iContour_ID];
		int iOffset = oStrip.m_iY - pContour_Strip[oStrip.m_iContour_ID][0];
		Get_Contour_Info::Line* poLine = &pLine_Strip_Map[oContour.m_iFirst_Line + iOffset];
		pNew_Strip[poLine->m_iFirst_Strip + poLine->m_iStrip_Count++] = oStrip;
	}
	bRet = 1;
END:
	Free(pContour_Strip);
	Free(oInfo.m_pStrip);
	Free(oInfo.m_pLine);

	oInfo.m_pStrip = pNew_Strip;
	oInfo.m_pLine = pLine_Strip_Map;
	*poInfo = oInfo;
	return bRet;
}
int iFind_Contour(Get_Contour_Info oInfo,int x, int y)
{//根据一个像素坐标(x,y) 寻找一个包含它的Controu
//返回： Contour_ID
	for (int i = 0; i < oInfo.m_iStrip_Count; i++)
	{
		Get_Contour_Info::Strip oStrip = oInfo.m_pStrip[i];
		if (oStrip.m_iY == y && oStrip.m_iStart <= x && oStrip.m_iEnd >= x)
			return oStrip.m_iContour_ID;
	}
	return -1;
}
void Remove_Orphan_Contour(Image oImage, Get_Contour_Info *poInfo, int iArea)
{//面积小于等于iArea的contour一律删除
	int i, j;
	Get_Contour_Info oInfo = *poInfo;
	{//第一步，将太小的区域删除
		Get_Contour_Info::Contour* poCur = oInfo.m_pContour,
			* poEnd = oInfo.m_pContour + oInfo.m_iContour_Count;
		j = 0;
		while (poCur < poEnd)
		{
			if (poCur->m_iArea <= (unsigned int)iArea)
			{
				poCur->m_bMark_Deleted = 1;
				poCur->m_iNew_ID = 0xFFFFFF;
			}
			else
			{
				poCur->m_bMark_Deleted = 0;
				poCur->m_iNew_ID = j++;
			}
			poCur++;
		}
	}

	/*for (i = 0, j = 0; i < oInfo.m_iContour_Count; i++)
	{
		if (oInfo.m_pContour[i].m_iArea <= (unsigned int)iArea)
		{
			oInfo.m_pContour[i].m_bMark_Deleted = 1;
			oInfo.m_pContour[i].m_iNew_ID = 0xFFFFFF;
		}else
		{
			oInfo.m_pContour[i].m_bMark_Deleted = 0;
			oInfo.m_pContour[i].m_iNew_ID = j++;
		}
	}*/

	//第二部，合并Strip
	j = 0;	//有效Strip 的位置
	for (int y = 0; y < oInfo.m_iHeight; y++)
	{
		Get_Contour_Info::Line oLine = oInfo.m_pLine[y], oLine_Dup = oLine;
		Get_Contour_Info::Strip* poPre = NULL;
		Get_Contour_Info::Strip oStrip;
		oLine_Dup.m_iFirst_Strip = j;
		
		//以下Pre始终在新位置上，不是在原来序列位置
		for (i = 0; i < oLine.m_iStrip_Count - 1;)
		{
			oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			if (oInfo.m_pContour[oStrip.m_iContour_ID].m_bMark_Deleted)
			{//此Contour删除了
				Get_Contour_Info::Strip* poNext = &oInfo.m_pStrip[oLine.m_iFirst_Strip + i + 1];
				if (poPre)
				{
					poPre->m_iEnd = poNext->m_iEnd;
					i += 2;	//j不变
				}
				else//行头
				{
					poNext->m_iStart = oStrip.m_iStart;
					//poPre = &oInfo.m_pStrip[j];
					//oInfo.m_pStrip[j++] = *poNext;
					i++;
				}
				//i += 2;	//j不变
			}else
			{
				oStrip.m_iContour_ID = oInfo.m_pContour[oStrip.m_iContour_ID].m_iNew_ID;
				oInfo.m_pStrip[j] = oStrip;
				poPre = &oInfo.m_pStrip[j];
				i++,j++;
			}
		}
		/*if (j == 272)
			printf("here");*/
		//以下判断是有必要的，因为当次最后一个为deleted, i就会跳到最后，不判断则多算
		if (i < oLine.m_iStrip_Count)
		{
			//最后一个
			oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			if (oInfo.m_pContour[oStrip.m_iContour_ID].m_bMark_Deleted)
				poPre->m_iEnd = oStrip.m_iEnd;			//此Contour删除了
			else
			{
				oStrip.m_iContour_ID = oInfo.m_pContour[oStrip.m_iContour_ID].m_iNew_ID;
				oInfo.m_pStrip[j++] = oStrip;
			}
		}
		oLine_Dup.m_iStrip_Count = j - oLine_Dup.m_iFirst_Strip;
		oInfo.m_pLine[y] = oLine_Dup;
	}
	oInfo.m_iStrip_Count = j;
	Shrink(oInfo.m_pStrip, oInfo.m_iStrip_Count * sizeof(Get_Contour_Info::Strip));
	
	//第三部分，紧缩contour
	for (i = 0, j = 0; i < oInfo.m_iContour_Count; i++)
	{
		if (!oInfo.m_pContour[i].m_bMark_Deleted)
			oInfo.m_pContour[j++] = oInfo.m_pContour[i];
	}
	//重画Image
	oInfo.m_iContour_Count = j;
	Draw_Strip(oImage, oInfo);

	*poInfo = oInfo;
	////验算strip
	//for (int i = 0; i < oInfo.m_iStrip_Count ; i++)
	//{
	//	if ((int)oInfo.m_pStrip[i].m_iContour_ID >= oInfo.m_iContour_Count)
	//	{
	//		printf("here");
	//	}
	//}
	return;
}

static int bInit_Get_Contour_Result(Get_Contour_Info oInfo, Get_Contour_Result* poResult)
{
	Get_Contour_Result oResult = { 0 };
	oResult.m_pContour = (Get_Contour_Result::Contour*)pMalloc(oInfo.m_iContour_Count * sizeof(Get_Contour_Result::Contour));
	oResult.m_iContour_Count = oInfo.m_iContour_Count;
	oResult.m_iWidth = oInfo.m_iWidth, oResult.m_iHeight = oInfo.m_iHeight;

	*poResult = oResult;
	if (!oResult.m_pContour)
		return 0;
	else
		return 1;
}

static void Add_To_Hash_Table(int Hash_Table[], Hash_Item* Item, int iHash_Size, int* piCur_Item, unsigned int A, unsigned int B)
{//加入散列表
	unsigned long long iPos;
	int iHash_Pos, iCur_Item = *piCur_Item;
	Hash_Item oItem_Exist;

	if (A > B)
		std::swap(A, B);
	/*if (A == 8 && B == 32)
		printf("Here");*/
	iHash_Pos = ((A << 4) ^ B) % iHash_Size;
	if ((iPos = Hash_Table[iHash_Pos]) != 0)
	{//散列表有东西
		do {
			oItem_Exist = Item[iPos];
			if (oItem_Exist.A == A && oItem_Exist.B == B)
			{//重复了，此点不加入散列表
				*piCur_Item = iCur_Item;
				return;
			}
		} while (iPos = oItem_Exist.m_iNext);
	}

	if (iCur_Item < iHash_Size)
	{
		/*if (iCur_Item == 27)
			printf("here");*/
		Item[iCur_Item] = { A,B,(unsigned int)Hash_Table[iHash_Pos] };
		Hash_Table[iHash_Pos] = iCur_Item++;
	}
	else
		printf("Insufficient memory");

	*piCur_Item = iCur_Item;
}

static int bGet_Neighbour_Start(Get_Contour_Info oInfo, Start_Count** ppNeighbour_Start_Count, int** ppNeighbour_Block, int* piCur_Neighbour)
{
	int iHash_Size = oInfo.m_iContour_Count & 1 ? oInfo.m_iContour_Count + 2 : oInfo.m_iContour_Count + 1,
		* pHash_Table,
		iCur_Item = 1, bRet = 0;
	int* pNeighbour_Block = NULL, iCount, iAdd_Hash_Count, iHeight_Minus_1;
	Start_Count* pNeighbour_Start_Count = NULL;
	pHash_Table = (int*)pMalloc(iHash_Size * sizeof(int));
	Hash_Item* pHash_Item = (Hash_Item*)pMalloc(iHash_Size * sizeof(Hash_Item));
	if (!pHash_Table || !pHash_Item)
		goto END;

	Get_Contour_Info::Line oLine, oNext_Line;
	Get_Contour_Info::Strip oStrip_0, oStrip_1;

	int i, j, y;
	iHeight_Minus_1 = oInfo.m_iHeight - 1;
	memset(pHash_Table, 0, iHash_Size * sizeof(int));

	iAdd_Hash_Count = 0;
	for (y = 0; y < iHeight_Minus_1; y++)
	{
		oLine = oInfo.m_pLine[y];
		oNext_Line = oInfo.m_pLine[y + 1];

		i = j = 0;
		while (1)
		{
			oStrip_0 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			oStrip_1 = oInfo.m_pStrip[oNext_Line.m_iFirst_Strip + j];
			//这样写会不会高端点
			if (oStrip_0.m_iEnd < oStrip_1.m_iStart && i < oLine.m_iStrip_Count)
			{
				i++;
				continue;
			}
			else if (oStrip_1.m_iEnd < oStrip_0.m_iStart && j < oNext_Line.m_iStrip_Count)
			{
				i++;
				continue;
			}

			if (oStrip_0.m_iContour_ID != oStrip_1.m_iContour_ID)
			{
				iAdd_Hash_Count++;
				Add_To_Hash_Table(pHash_Table, pHash_Item, iHash_Size, &iCur_Item, oStrip_0.m_iContour_ID, oStrip_1.m_iContour_ID);
			}
			i++;
			if (i >= oLine.m_iStrip_Count || j >= oNext_Line.m_iStrip_Count)
				break;
			//iCounter++;
		}

		//再横向加关系
		for (i = 1; i < oLine.m_iStrip_Count; i++)
		{
			/*if (iCur_Item == 27)
				printf("Here");*/
			oStrip_0 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i - 1];
			oStrip_1 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			iAdd_Hash_Count++;
			Add_To_Hash_Table(pHash_Table, pHash_Item, iHash_Size, &iCur_Item, oStrip_0.m_iContour_ID, oStrip_1.m_iContour_ID);
		}
	}
	Free(pHash_Table), pHash_Table = NULL;

	pNeighbour_Start_Count = (Start_Count*)pMalloc(oInfo.m_iContour_Count * sizeof(Start_Count));
	memset(pNeighbour_Start_Count, 0, oInfo.m_iContour_Count * sizeof(Start_Count));
	pHash_Item++;	//调整到真正开始位置
	iCount = iCur_Item - 1;
	for (i = 0; i < iCount; i++)
	{
		pNeighbour_Start_Count[pHash_Item[i].A].m_iCount++;
		pNeighbour_Start_Count[pHash_Item[i].B].m_iCount++;
	}
	pNeighbour_Start_Count[0].m_iContour_ID = 0;
	pNeighbour_Start_Count[0].m_iStart = 0;
	for (i = 1; i < oInfo.m_iContour_Count; i++)
	{
		pNeighbour_Start_Count[i].m_iContour_ID = i;
		pNeighbour_Start_Count[i].m_iStart = pNeighbour_Start_Count[i - 1].m_iStart + pNeighbour_Start_Count[i - 1].m_iCount;
		pNeighbour_Start_Count[i - 1].m_iCount = 0;
	}
	pNeighbour_Start_Count[oInfo.m_iContour_Count - 1].m_iCount = 0;
	pNeighbour_Block = (int*)pMalloc(iCount * 2 * sizeof(int));

	for (i = 0; i < iCount; i++)
	{
		Hash_Item oItem = pHash_Item[i];
		/*if (i == 26)
			printf("Here");*/

		Start_Count oStart_Count = pNeighbour_Start_Count[oItem.A];
		pNeighbour_Block[oStart_Count.m_iStart + oStart_Count.m_iCount++] = oItem.B;
		pNeighbour_Start_Count[oItem.A] = oStart_Count;

		oStart_Count = pNeighbour_Start_Count[oItem.B];
		pNeighbour_Block[oStart_Count.m_iStart + oStart_Count.m_iCount++] = oItem.A;
		pNeighbour_Start_Count[oItem.B] = oStart_Count;
	}
	*ppNeighbour_Block = pNeighbour_Block;
	*ppNeighbour_Start_Count = pNeighbour_Start_Count;

	pHash_Item--;
	bRet = 1;
END:
	if (!bRet)
	{
		Free(pHash_Table);
		Free(pHash_Item);
		Free(pNeighbour_Block);
		Free(pNeighbour_Start_Count);
		*ppNeighbour_Block = NULL;
		*ppNeighbour_Start_Count = NULL;
	}
	else
		Free(pHash_Item);
	return bRet;
}

static int bStart_Count_2_Hierachy(Start_Count* pNeighbour_Start_Count, int* pNeighbour_Block, Get_Contour_Result oResult)
{
	//一趟趟扫描表，把点修正
	int i, j, iUndone_Count, iRound, iCounter, iCur_Item = 0, bRet = 0;
	int* pNode_Undone = (int*)pMalloc(oResult.m_iContour_Count * sizeof(int));
	int (*pHierachy)[4] = (int(*)[4])pMalloc(oResult.m_iContour_Count * 4 * sizeof(int));
	if (!pNode_Undone || !pHierachy)
		goto END;

	//先把树初始化为秃节点，无父无子无兄弟
	memset(pHierachy, -1, oResult.m_iContour_Count * 4 * sizeof(int));

	//pNode_Undone是一张表，记录所有还未干的节点，初始时为所有拥有两个邻居的才搞
	for (i = j = 0; i < oResult.m_iContour_Count; i++)
		if (pNeighbour_Start_Count[i].m_iCount >= 2)
		{
			pNode_Undone[j++] = i;
			//printf("Contour:%d Neighbour Count:%d\n",i, pNeighbour_Start_Count[i].m_iCount);
		}

	iUndone_Count = j;
	iRound = 0;
	iCounter = 0;
	while (iUndone_Count)
	{//此处不断调整，直到邻居数为0
		for (i = 0; i < iUndone_Count; i++)
		{
			Start_Count oParent_Start_Count = pNeighbour_Start_Count[pNode_Undone[i]];
			int* pParent = pHierachy[oParent_Start_Count.m_iContour_ID];	//oResult.m_pContour[oParent_Start_Count.m_iContour_ID].hierachy;
			//对Parent所有的孩子进行调整
			for (j = 0; j < oParent_Start_Count.m_iCount; j++)
			{
				Start_Count oChild_Start_Count = pNeighbour_Start_Count[pNeighbour_Block[oParent_Start_Count.m_iStart + j]];
				if (oChild_Start_Count.m_iCount == 1)
				{//该节点的邻居只有一个，以前的孩子都处理完了，那么表示它还有个父节点
					int* pNode = pHierachy[oChild_Start_Count.m_iContour_ID];	//oResult.m_pContour[oChild_Start_Count.m_iContour_ID].hierachy;
					//设置节点的父亲
					pNode[3] = oParent_Start_Count.m_iContour_ID;
					//节点的Next Sibling为父亲的第一个孩子
					pNode[1] = pParent[2];
					//节点的前一个兄弟与孩子都未知，暂设-1
					pNode[0] = -1;

					if (pParent[2] != -1)
					{//有Next Sibling才需要设
						//Next Sibling的前一个孩子是这个节点
						pHierachy[pParent[2]][0] = oChild_Start_Count.m_iContour_ID;
					}

					//父亲的第一个孩子设为这个节点
					pParent[2] = oChild_Start_Count.m_iContour_ID;

					//修改pNeighbour_2
					pNeighbour_Block[oParent_Start_Count.m_iStart + j] = -1;

					//更新Child_Start_Count
					pNeighbour_Start_Count[oChild_Start_Count.m_iContour_ID].m_iCount = 0;
				}
			}

			//更新oParent_Start_Count
			int k;
			for (j = k = 0; j < oParent_Start_Count.m_iCount; j++)
			{
				if (pNeighbour_Block[oParent_Start_Count.m_iStart + j] != -1)
				{
					pNeighbour_Block[oParent_Start_Count.m_iStart + k] = pNeighbour_Block[oParent_Start_Count.m_iStart + j];
					k++;
				}
			}
			pNeighbour_Start_Count[oParent_Start_Count.m_iContour_ID].m_iCount = k;
		}

		//再扫一次，抄到undone中
		for (i = j = 0; i < iUndone_Count; i++)
		{
			Start_Count oParent_Start_Count = pNeighbour_Start_Count[pNode_Undone[i]];
			if (oParent_Start_Count.m_iCount >= 2)
				pNode_Undone[j++] = pNode_Undone[i];
		}
		iUndone_Count = j;
		iCounter++;
		iRound++;
	}

	typedef struct Value_16 {
		char Value[16];
	}Value_16;
	for (i = 0; i < oResult.m_iContour_Count; i++)
	{
		*(Value_16*)oResult.m_pContour[i].hierachy =
			*(Value_16*)pHierachy[i];
	}
	//Disp_Mem();
	bRet = 1;
END:
	Free(pHierachy);
	Free(pNode_Undone);
	Free(pNeighbour_Block);
	Free(pNeighbour_Start_Count);
	return bRet;
}
static int bGen_Hierarchy(Get_Contour_Info oInfo, Contour_Tree_Node** ppTree, Get_Contour_Result oResult)
{//生成一棵树，谁和谁一级，谁和谁是父子关系
	//第一步，找到所有的两两联系
	int iCur_Neighbour;

	Start_Count* pNeighbour_Start_Count;
	int* pNeighbour_Block;
	if (!bGet_Neighbour_Start(oInfo, &pNeighbour_Start_Count, &pNeighbour_Block, &iCur_Neighbour))
		return 0;

	//一气呵成，还慢了
	if (!bStart_Count_2_Hierachy(pNeighbour_Start_Count, pNeighbour_Block, oResult))
		return 0;
	//Disp_Mem();
	return 1;
}

static void Outline_Start_White(Get_Contour_Info::Line* pLine, Get_Contour_Info::Strip* pStrip,
	int* pbRepeat_Visist, unsigned short Last_Point[2])
{//主要判断初点是否会多次访问
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Strip oStrip_0, oStrip_1;

	oLine = pLine[1];
	int i;

	oStrip_0 = pStrip[pLine[0].m_iFirst_Strip];
	oLine = pLine[1];
	for (i = 0; i < oLine.m_iStrip_Count; i++)
	{
		oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
		if (oStrip_1.m_iEnd + 1 == oStrip_0.m_iStart /*&& oStrip_0.m_iEnd>oStrip_0.m_iStart*/)
		{
			Last_Point[0] = oStrip_1.m_iEnd;
			Last_Point[1] = oStrip_1.m_iY;
			*pbRepeat_Visist = 1;
			return;
		}
	}
	*pbRepeat_Visist = 0;
}

static int iDir_of_Point_White(Get_Contour_Info::Strip oStrip, int iDir, int x, int* px, int iUp_Down)
{
	if (x - 1 > oStrip.m_iEnd || x + 1 < oStrip.m_iStart)
		return 0;
	int x1;
	if (iUp_Down == 1)
	{//判断下面的线，管1，2，3
		if (iDir == 1)
			x1 = x + 1;
		else if (iDir == 2)
			x1 = x;
		else
			x1 = x - 1;
	}
	else
	{
		if (iDir == 5)
			x1 = x - 1;
		else if (iDir == 6)
			x1 = x;
		else
			x1 = x + 1;
	}
	if (x1 >= oStrip.m_iStart && x1 <= oStrip.m_iEnd)
	{
		if (px)	*px = x1;
		return 1;
	}
	else
		return 0;
}

static int bFind_Next_Point_White(Get_Contour_Info::Contour oContour, Get_Contour_Info::Line* pLine,
	Get_Contour_Info::Strip* pStrip, int* piCur_Line, Get_Contour_Info::Strip* poStrip_To_Match, int* px,
	int* piDir, int iCounter)
{//简化一下
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Strip oStrip_0 = *poStrip_To_Match, oStrip_1;
	int i, k, iDir = *piDir, iDir_1, bFound = 0;
	int x = *px, x1;
	int iCur_Line = *piCur_Line;
	/*if (iCounter == 2)
	printf("here");*/
	for (k = 0; k < 8; k++)
	{
		if (iDir == 0)
		{//由于前进方向可以快速推进，故此要顾及推进时是否有上面的边
			if (x == oStrip_0.m_iEnd)
				iDir = (iDir + 1) % 8;	//本来就走到尽头了,继续转一格
			else
			{
				if (iCur_Line)
				{//有上边
					bFound = 0;
					oLine = pLine[iCur_Line - 1];
					for (i = 0; i < oLine.m_iStrip_Count; i++)
					{
						oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
						if (oStrip_1.m_iStart > x)
						{//有物阻挡
							if (oStrip_1.m_iStart - 1 <= oStrip_0.m_iEnd)
							{
								x1 = oStrip_1.m_iStart - 1;
								bFound = 1;
								iDir_1 = iDir;
								break;
							}
							else	//不必扫到最后，短路退出
								break;
						}
					}
					if (bFound)
					{
						*px = x1;
						*piCur_Line = iCur_Line;
						*poStrip_To_Match = oStrip_0;
						*piDir = (iDir_1 - 2 + 8) % 8;
						return 1;
					}
				}
				//向右到最后
				*px = oStrip_0.m_iEnd;
				*piDir = 6;
				return 1;
			}
		}
		else if (iDir == 4)
		{//同理，向左推进时要估计有没有下面的边
			if (x == oStrip_0.m_iStart)
				iDir = (iDir + 1) % 8;//本来就走到尽头了,继续转一格
			else
			{
				if (iCur_Line < oContour.m_iLine_Count - 1)
				{//向下寻找障碍
					bFound = 0;
					oLine = pLine[iCur_Line + 1];
					for (i = oLine.m_iStrip_Count - 1; i >= 0; i--)
					{
						oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
						if (oStrip_1.m_iEnd < x)
						{//有物阻挡
							if (oStrip_1.m_iEnd + 1 >= oStrip_0.m_iStart)
							{
								x1 = oStrip_1.m_iEnd + 1;
								bFound = 1;
								iDir_1 = iDir;
								break;
							}
							else//不必扫到最后，短路退出
								break;
						}
					}
					if (bFound)
					{
						*px = x1;
						*piCur_Line = iCur_Line;
						*poStrip_To_Match = oStrip_0;
						*piDir = (iDir_1 - 2 + 8) % 8;
						return 1;
					}
				}
				*px = oStrip_0.m_iStart;
				*piDir = 2;
				return 1;
			}
		}
		else
		{
			if (iDir == 5 || iDir == 6 || iDir == 7)
			{//上面扫一下
				if (iCur_Line)
				{//有上边
					bFound = 0;
					oLine = pLine[iCur_Line - 1];
					for (i = 0; i < oLine.m_iStrip_Count; i++)
					{
						oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
						iDir_1 = iDir_of_Point_White(oStrip_1, iDir, x, &x1, 0);
						if (iDir_1 == 0)
							continue;
						else
						{
							iDir_1 = iDir;
							bFound = 1;
							break;
						}
					}
					if (bFound)
					{
						*px = x1;
						*piCur_Line = iCur_Line - 1;
						*poStrip_To_Match = oStrip_1;
						*piDir = (iDir_1 - 2 + 8) % 8;
						return 1;
					}
				}
				//既然5,6,7都扫过了，就一口气踢进三格
				iDir = (iDir + 1) % 8;
			}
			else if (iDir == 1 || iDir == 2 || iDir == 3)
			{//三个都是向下找
				if (iCur_Line < oContour.m_iLine_Count - 1)
				{
					oLine = pLine[iCur_Line + 1];
					for (i = oLine.m_iStrip_Count - 1; i >= 0; i--)
					{
						oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
						iDir_1 = iDir_of_Point_White(oStrip_1, iDir, x, &x1, 1);
						if (iDir_1 == 0)
							continue;
						else
						{
							iDir_1 = iDir;
							bFound = 1;
							break;
						}
					}
					if (bFound)
					{
						*px = x1;
						*piCur_Line = iCur_Line + 1;
						*poStrip_To_Match = oStrip_1;
						*piDir = (iDir_1 - 2 + 8) % 8;;
						return 1;
					}
				}
				iDir = (iDir + 1) % 8;
			}
		}
	}
	printf("Error");
	return 0;
}

static void Gen_Outline_White(Get_Contour_Info::Contour oContour, Get_Contour_Info::Line* pLine,
	Get_Contour_Info::Strip* pStrip, unsigned short Point[][2], int iMax_Point_Count,
	int* piPoint_Count, int bBreak = 0)
{//没有好办法，只能还用旋转法
	Get_Contour_Info::Line oCur_Line = pLine[0];
	Get_Contour_Info::Strip oStrip = pStrip[oCur_Line.m_iFirst_Strip];

	//int iLeft_Right = 1;			//0向上，1向下；初始化为向下
	int iStart_x, iStart_y, x;	// , y;
	int iCur_Point = 0;
	int iCur_Line = 0;

	Point[0][0] = iStart_x = oStrip.m_iStart;
	Point[0][1] = iStart_y = oStrip.m_iY;
	iCur_Point = 1;
	x = oStrip.m_iStart;	// , y = oStrip_0.m_iY;
	if (oContour.m_iLine_Count == 1)
	{
		if (oStrip.m_iEnd == iStart_x)
			*piPoint_Count = 1;
		else
		{
			Point[1][0] = x, Point[1][1] = oStrip.m_iY;
			*piPoint_Count = 2;
		}
		return;
	}
	else if (oContour.m_iArea == 2)
	{
		oStrip = pStrip[pLine[1].m_iFirst_Strip];
		Point[1][0] = oStrip.m_iStart, Point[1][1] = oStrip.m_iY;
		*piPoint_Count = 2;
		return;
	}
	int iDir = 0;
	int iCounter = 0;
	int iPoint_Count = 1;

	unsigned short Last_Point[2], Previous_Point[2] = {};
	int bRe_Visit = 0, bSwap = 0;

	Outline_Start_White(pLine, pStrip, &bRe_Visit, Last_Point);
	int bIs_End = bRe_Visit ? 0 : 1;
	while (1)
	{
		bFind_Next_Point_White(oContour, pLine, pStrip, &iCur_Line, &oStrip, &x, &iDir, iCounter);

		if (bRe_Visit)
		{
			if (Previous_Point[0] == Last_Point[0] && Previous_Point[1] == Last_Point[1])
				bIs_End = 1;
			Previous_Point[0] = x;
			Previous_Point[1] = oStrip.m_iY;
		}

		if (iPoint_Count >= iMax_Point_Count)
		{
			printf("Gen_Outline_White Error: insufficient memory\n");
			break;
		}
		if (x == iStart_x && oStrip.m_iY == iStart_y && bIs_End)
			break;
		else
		{
			Point[iPoint_Count][0] = x;
			Point[iPoint_Count][1] = oStrip.m_iY;
			iPoint_Count++;
		}
		iCounter++;
	}
	*piPoint_Count = iPoint_Count;
	return;
}

static void Outline_Start_Black(Get_Contour_Info::Line* pLine, Get_Contour_Info::Strip* pStrip,
	int* pbRe_Visist, unsigned short Last_Point[2])
{//主要判断初点是否会多次访问
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Strip oStrip_0, oStrip_1;
	oLine = pLine[1];
	int i;

	//先取出初点所在的线
	oStrip_0 = pStrip[pLine[0].m_iFirst_Strip];
	if (oStrip_0.m_iStart == oStrip_0.m_iEnd)
	{//第一strip只有一点必然能绕场一周？代考
		*pbRe_Visist = 0;
		return;
	}

	//再对第二条线上所有的Strip进行判断
	oLine = pLine[1];
	for (i = 0; i < oLine.m_iStrip_Count; i++)
	{
		oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
		if (oStrip_1.m_iEnd == oStrip_0.m_iStart)
		{
			Last_Point[0] = oStrip_1.m_iEnd;
			Last_Point[1] = oStrip_1.m_iY;
			*pbRe_Visist = 1;
			return;
		}
	}
	*pbRe_Visist = 0;
}

static int bFind_Next_Point_Black(Get_Contour_Info::Contour oContour, Get_Contour_Info::Line* pLine,
	Get_Contour_Info::Strip* pStrip, int* piCur_Line, Get_Contour_Info::Strip* poStrip_To_Match, int* px,
	int* piDir, int iCounter)
{//用4连通方法
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Strip oStrip_0 = *poStrip_To_Match, oStrip_1;
	int i, k, iDir = *piDir, bFound = 0;
	int x = *px;
	int iCur_Line = *piCur_Line;
	for (k = 0; k < 4; k++)
	{//最多需按照一周4个方向
		if (iDir == 0)
		{//向右挺进
			if (x == oStrip_0.m_iEnd)
			{//本来就走到尽头了,继续转一格
				iDir = (iDir + 1) % 4;
			}
			else if (iCur_Line)
			{//有上边，找个能上去的点
				bFound = 0;
				oLine = pLine[iCur_Line - 1];
				for (i = 0; i < oLine.m_iStrip_Count; i++)
				{
					oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
					if (oStrip_1.m_iStart > x)
					{//有物阻挡
						if (oStrip_1.m_iStart <= oStrip_0.m_iEnd)
						{
							*px = oStrip_1.m_iStart;
							*piCur_Line = iCur_Line;
							*poStrip_To_Match = oStrip_0;
							*piDir = (iDir - 1 + 4) % 4;
							return 1;
						}
						else//无物阻挡也要推进到一定成都就得跳出，否则全线扫描了
							break;
					}
				}
				//移到最后
				*px = oStrip_0.m_iEnd;
				*piDir = 3;
				return 1;
			}
			else
			{//向右到最后
				*px = oStrip_0.m_iEnd;
				*piDir = 3;
				return 1;
			}
		}
		else if (iDir == 1)
		{//向下找
			if (iCur_Line < oContour.m_iLine_Count - 1)
			{
				oLine = pLine[iCur_Line + 1];
				for (i = oLine.m_iStrip_Count - 1; i >= 0; i--)
				{
					oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
					if (oStrip_1.m_iStart <= x)
					{//找到
						if (oStrip_1.m_iEnd >= x)
						{
							*px = x;
							*piCur_Line = iCur_Line + 1;
							*poStrip_To_Match = oStrip_1;
							*piDir = (iDir - 1 + 4) % 4;
							return 1;
						}
						else //不必全部Strip都扫完，可以短途退出
							break;
					}
				}
			}
			iDir = (iDir + 1) % 4;
		}
		else if (iDir == 2)
		{//向右
			if (x == oStrip_0.m_iStart)
			{//本来就走到尽头了,继续转一格
				iDir = (iDir + 1) % 4;
			}
			else
			{
				if (iCur_Line < oContour.m_iLine_Count - 1)
				{
					bFound = 0;
					oLine = pLine[iCur_Line + 1];
					for (i = oLine.m_iStrip_Count - 1; i >= 0; i--)
					{
						oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
						if (oStrip_1.m_iEnd < x)
						{
							if (oStrip_1.m_iEnd >= oStrip_0.m_iStart)
							{
								*px = oStrip_1.m_iEnd;
								*piCur_Line = iCur_Line;
								*poStrip_To_Match = oStrip_0;
								*piDir = (iDir - 1 + 4) % 4;
								return 1;
							}
							else//不必全部Strip都扫完，可以短途退出
								break;
						}
					}
				}
				//移到最后
				*px = oStrip_0.m_iStart;
				*piDir = 1;
				return 1;
			}
		}
		else if (iDir == 3)
		{//向上找
			if (iCur_Line)
			{//有上边
				bFound = 0;
				oLine = pLine[iCur_Line - 1];
				for (i = 0; i < oLine.m_iStrip_Count; i++)
				{
					oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
					if (oStrip_1.m_iEnd >= x)
					{//找到
						if (oStrip_1.m_iStart <= x)
						{
							*px = x;
							*piCur_Line = iCur_Line - 1;
							*poStrip_To_Match = oStrip_1;
							*piDir = (iDir - 1 + 4) % 4;
							return 1;
						}
						else
							break;
					}
				}
			}
			iDir = (iDir + 1) % 4;
		}
	}
	return 0;
}

static void Gen_Outline_Black(Get_Contour_Info::Contour oContour, Get_Contour_Info::Line* pLine,
	Get_Contour_Info::Strip* pStrip, unsigned short Point[][2], int iMax_Point_Count,
	int* piPoint_Count, int bBreak = 0)
{//求某个连通域的轮廓
	Get_Contour_Info::Line oCur_Line = pLine[0];
	Get_Contour_Info::Strip oStrip = pStrip[oCur_Line.m_iFirst_Strip];

	//先解决第一点
	int iStart_x, iStart_y, x;	// y;
	int iCur_Point = 0, iCur_Line = 0;
	Point[0][0] = iStart_x = oStrip.m_iStart;
	Point[0][1] = iStart_y = oStrip.m_iY;
	iCur_Point = 1;
	x = oStrip.m_iStart;
	if (oContour.m_iLine_Count == 1)
	{//只有一条线，必然最多只有两点
		if (oStrip.m_iEnd == iStart_x)
			*piPoint_Count = 1;
		else
		{
			Point[1][0] = oStrip.m_iEnd, Point[1][1] = oStrip.m_iY;
			*piPoint_Count = 2;
		}
		return;
	}
	else if (oContour.m_iArea == 2)
	{//只有轻微的提升
		oStrip = pStrip[pLine[1].m_iFirst_Strip];
		Point[1][0] = oStrip.m_iStart, Point[1][1] = oStrip.m_iY;
		*piPoint_Count = 2;
		return;
	}

	//再判断是否会多次回到初点
	int iDir = 0, iCounter = 0, iPoint_Count = 1;
	unsigned short Last_Point[2], Previous_Point[2] = {};
	int bRe_Visit = 0, //多次回到初点标志
		bSwap = 0;
	Outline_Start_Black(pLine, pStrip, &bRe_Visit, Last_Point);
	int bIs_End = bRe_Visit ? 0 : 1;

	while (1)
	{
		bFind_Next_Point_Black(oContour, pLine, pStrip, &iCur_Line, &oStrip, &x, &iDir, iCounter);
		if (bRe_Visit)
		{
			if (Previous_Point[0] == Last_Point[0] && Previous_Point[1] == Last_Point[1])
				bIs_End = 1;
			Previous_Point[0] = x;
			Previous_Point[1] = oStrip.m_iY;
			//printf("Revisit:%d bIs_End:%d\n", bRe_Visit,bIs_End);
		}

		//printf("%d %d\n", Point[iPoint_Count][0], Point[iPoint_Count][1]);
		if (iPoint_Count >= iMax_Point_Count)
		{
			printf("Gen_Outline_Black: insufficient memory\n");
			break;
		}
		if (x == iStart_x && oStrip.m_iY == iStart_y && bIs_End)
			break;
		else
		{
			Point[iPoint_Count][0] = x;
			Point[iPoint_Count][1] = oStrip.m_iY;
			iPoint_Count++;
		}

		iCounter++;
	}

	*piPoint_Count = iPoint_Count;
	return;
}

static void Shrink_Point(unsigned short Point[][2], int* piPoint_Count)
{
	int i, j, iPoint_Count = *piPoint_Count;
	if (iPoint_Count == 1)
		return;		//一点不调
	if (iPoint_Count == 2)
	{//重复点，不要。也许这部分代码也不要，后面一气呵成
		if (Point[1][0] == Point[0][0] && Point[1][1] == Point[0][1])
			*piPoint_Count = 1;
		return;
	}
	float fPre_Grad, fGrad;
#define MIN_FLOAT 1e-10
	//int iPre_Sign, iSign;	//1为正号，-0为符号，只有水平方向需要判断
	if (Point[0][0] == Point[1][0])
	{
		fPre_Grad = Point[1][1] > Point[0][1] ? MAX_FLOAT : -MAX_FLOAT;
		//垂直时要加上符号判别那段
		//iPre_Sign = Point[1][1] > Point[0][1] ? 1 : 0;
	}
	else
	{
		fPre_Grad = (float)(Point[1][1] - Point[0][1]) / (Point[1][0] - Point[0][0]);
		if (fPre_Grad == 0)
			fPre_Grad = (float)(Point[1][0] > Point[0][0] ? MIN_FLOAT : -MIN_FLOAT);
	}

	for (i = 2, j = 1; i < iPoint_Count; i++)
	{
		/*if (i == iPoint_Count-1)
			printf("here");*/
		if (Point[i][0] == Point[i - 1][0])
			fGrad = Point[i][1] > Point[i - 1][1] ? MAX_FLOAT : -MAX_FLOAT;
		else
		{
			fGrad = (float)(Point[i][1] - Point[i - 1][1]) / (Point[i][0] - Point[i - 1][0]);
			if (fGrad == 0)
				fGrad = (float)(Point[i][0] > Point[i - 1][0] ? MIN_FLOAT : -MIN_FLOAT);
		}

		if (fGrad != fPre_Grad)
			j++;
		else
		{//还得判断掉头情况
			if (Point[i][0] == Point[i - 2][0] && Point[i][1] == Point[i - 2][1])
				j++;
		}
		fPre_Grad = fGrad;
		Point[j][0] = Point[i][0], Point[j][1] = Point[i][1];
	}

	//最后一点还得继续判断
	if (j >= 2)
	{
		if (Point[j][0] == Point[0][0])
			fGrad = Point[0][1] > Point[j][1] ? MAX_FLOAT : -MAX_FLOAT;
		//fGrad = MAX_FLOAT;
		else
		{
			fGrad = (float)(Point[j][1] - Point[0][1]) / (Point[j][0] - Point[0][0]);
			if (fGrad == 0)
				fGrad = (float)(Point[0][0] > Point[j][0] ? MIN_FLOAT : -MIN_FLOAT);
		}
		if (fGrad == fPre_Grad)
			j--;
	}
	*piPoint_Count = j + 1;
	return;
}

static int bGen_Outline(Get_Contour_Info oInfo, Get_Contour_Result* poResult, const int iMax_Point_Count = 3000000)
{//尝试逐个生成轮廓
	int i;
	Get_Contour_Info::Contour oContour;
	Get_Contour_Result oResult = *poResult;
	//unsigned short (*pPoint)[2] = (unsigned short(*)[2])pMalloc(iMax_Point_Count * 2 * sizeof(unsigned short));
	oResult.m_pAll_Point = (unsigned short(*)[2])pMalloc(iMax_Point_Count * 2 * sizeof(unsigned short));
	if (!oResult.m_pAll_Point)
		return 0;

	unsigned short (*pPoint)[2] = oResult.m_pAll_Point;
	int iPoint_Count_Remain = iMax_Point_Count, iPoint_Count;
	int iMin_Strip_Count = 0xFFFFFFF,
		iMin_Line_Count = 0xFFFFFFF;

	//oResult.m_iNon_Root_Max_Area = 0;
	for (i = 0; i < oInfo.m_iContour_Count; i++)
	{
		int bBreak = 0;
		oContour = oInfo.m_pContour[i];

		//由于连通数不一样，分黑白两种方式进行勾边
		if (oContour.m_iBlack_or_White)
			Gen_Outline_White(oContour, &oInfo.m_pLine[oContour.m_iFirst_Line], oInfo.m_pStrip, pPoint, iPoint_Count_Remain, &iPoint_Count, bBreak);
		else
			Gen_Outline_Black(oContour, &oInfo.m_pLine[oContour.m_iFirst_Line], oInfo.m_pStrip, pPoint, iPoint_Count_Remain, &iPoint_Count, bBreak);

		Get_Contour_Info::Line oLine = oInfo.m_pLine[oContour.m_iFirst_Line];
		Get_Contour_Info::Strip oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip];

		if (iPoint_Count >= iPoint_Count_Remain)
		{//超出最大点数
			printf("Insufficient memory\n");
			oResult.m_iMax_Point_Count = 0;
			Free(oResult.m_pAll_Point);
			oResult.m_pAll_Point = NULL;	//调用程序可通过这个判断
			break;
		}
		else
		{
			//在此进行减点
			/*if (i == 1)
				printf("Here");*/
			Shrink_Point(pPoint, &iPoint_Count);
			//Disp((short*)pPoint, iPoint_Count, 2, "Point");
			oResult.m_pContour[i].m_pPoint = pPoint;
			oResult.m_pContour[i].m_iOutline_Point_Count = iPoint_Count;
			oResult.m_pContour[i].m_iArea = oContour.m_iArea;
			//if (oContour.m_iArea > oResult.m_iNon_Root_Max_Area && oResult.m_pContour[i].hierachy[3]!=-1)
				//oResult.m_iNon_Root_Max_Area = oContour.m_iArea;

			pPoint += iPoint_Count;
			iPoint_Count_Remain -= iPoint_Count;
		}
	}
	oResult.m_iMax_Point_Count = iMax_Point_Count - iPoint_Count_Remain + 1;
	Shrink(oResult.m_pAll_Point, oResult.m_iMax_Point_Count * 2 * sizeof(unsigned short));
	*poResult = oResult;
	return 1;
}

int bGet_Contour(Image oImage, Get_Contour_Result* poResult)
{//oImage: 图片数据		piContour_Count：返回一共找到多少个连通域
	Get_Contour_Info oInfo = {};
	unsigned long long tStart = iGet_Tick_Count();
	int bRet = 0;

	int iMax_Strip_Count = oImage.m_iWidth * oImage.m_iHeight / 2,
		iMax_Point_Count = iMax_Strip_Count;
	*poResult = {};
	////打算不搞预分配
	Init_Get_Contour_Info(&oInfo, iMax_Strip_Count, oImage.m_iWidth, oImage.m_iHeight);
	//bSave_Image("c:\\tmp\\1.bmp", oImage);

	//***********由于要应付大内存消耗，第一趟先扫出所有Strip，然后Compact内存********
	if (!bGet_Strip(&oInfo, oImage))
		goto END;
	//***********由于要应付大内存消耗，第一趟先扫出所有Strip，然后Compact内存********
	
	//******************根据Strip找到所有的连通域，但是尚未勾边**********************
	if (!bGet_Contour(&oInfo, oImage.m_iHeight))
		goto END;
	//******************根据Strip找到所有的连通域，但是尚未勾边**********************

	//************去除孤立小区域，令后续的迭代更快更良性****************************/
	//printf("%d %d\n", oInfo.m_iStrip_Count, oInfo.m_iContour_Count);
	Remove_Orphan_Contour(oImage, &oInfo, 4);
	//printf("%d %d\n", oInfo.m_iStrip_Count, oInfo.m_iContour_Count);
	//************去除孤立小区域，令后续的迭代更快更良性****************************/

	if (!bInit_Get_Contour_Result(oInfo, poResult))
		goto END;

	//*****************将所有的Contour组织成一棵树***********************************
	Contour_Tree_Node* pTree;
	if (!bGen_Hierarchy(oInfo, &pTree, *poResult))
		goto END;
	//*****************将所有的Contour组织成一棵树***********************************

	//******************为后面的轮廓勾勒建立拓扑，此处不能循环测试*******************
	if (!bPrepare_Outline(&oInfo))
		goto END;
	//******************为后面的轮廓勾勒建立拓扑，此处不能循环测试*******************

	//*********************************勾勒轮廓**************************************
	if (!bGen_Outline(oInfo, poResult, iMax_Point_Count))
		goto END;
	//*********************************勾勒轮廓**************************************
	bRet = 1;
END:
	Free_Contour_Info(&oInfo);
	if (!bRet)
		Free_Contour_Result(poResult);
	return bRet;
}

int bGet_Contour(Image oImage, Get_Contour_Info* poInfo)
{
	Get_Contour_Info oInfo = {};
	unsigned long long tStart = iGet_Tick_Count();
	int bRet = 0;

	int iMax_Strip_Count = oImage.m_iWidth * oImage.m_iHeight / 3,
		iMax_Point_Count = iMax_Strip_Count;

	//打算不搞预分配
	Init_Get_Contour_Info(&oInfo, iMax_Strip_Count, oImage.m_iWidth, oImage.m_iHeight);

	//***********由于要应付大内存消耗，第一趟先扫出所有Strip，然后Compact内存********
//#ifdef ESP32
	//if (!bGet_Strip(&oInfo, oImage,0))
//#else
	if (!bGet_Strip_Org(&oInfo, oImage))
//#endif
		goto END;
	//***********由于要应付大内存消耗，第一趟先扫出所有Strip，然后Compact内存********
	
	//******************根据Strip找到所有的连通域，但是尚未勾边**********************
	if (!bGet_Contour(&oInfo, oImage.m_iHeight))
		goto END;
	//******************根据Strip找到所有的连通域，但是尚未勾边**********************

	//************去除孤立小区域，令后续的迭代更快更良性****************************/
	//tStart = iGet_Tick_Count();
	Remove_Orphan_Contour(oImage, &oInfo, 4);
	//printf("%lld %d\n", iGet_Tick_Count() - tStart, oInfo.m_iContour_Count);
	//************去除孤立小区域，令后续的迭代更快更良性****************************/

	//Draw_Strip("c:\\tmp\\1.bmp", oInfo);
	*poInfo = oInfo;
	bRet = 1;
END:
	if (!bRet)
		Free_Contour_Info(&oInfo);

	return bRet;
}
//*********************************第一部分，寻找连通域******************************************/

//**************************第二部分，棋盘检测****************************************************/
//一组辅助函数
void Draw_Quads(const char* pcFile, Chess_Board_Quad* Quad, int iQuad_Count, int bi16 = 1, int w = 1920, int h = 1080)
{
	Image oImage;
	Init_Image(&oImage, w, h, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (int i = 0; i < iQuad_Count; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (bi16)
				Draw_Line(oImage, Quad[i].Corner_i[j][0], Quad[i].Corner_i[j][1],
					Quad[i].Corner_i[(j + 1) % 4][0], Quad[i].Corner_i[(j + 1) % 4][1]);
			else
				Draw_Line(oImage, (int)(Quad[i].Corner[j]->Pos[0]), (int)(Quad[i].Corner[j]->Pos[1]),
					(int)(Quad[i].Corner[(j + 1) % 4]->Pos[0]), (int)(Quad[i].Corner[(j + 1) % 4]->Pos[1]));
		}
	}
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
	return;
}

void Draw_Quad_Group(const char* pcFile, Chess_Board_Quad** Quad, int iQuad_Count, int w = 1920, int h = 1080)
{
	Image oImage;
	Init_Image(&oImage, w, h, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (int i = 0; i < iQuad_Count; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			Draw_Line(oImage, (int)(Quad[i]->Corner[j]->Pos[0]), (int)(Quad[i]->Corner[j]->Pos[1]),
				(int)(Quad[i]->Corner[(j + 1) % 4]->Pos[0]), (int)(Quad[i]->Corner[(j + 1) % 4]->Pos[1]));
		}
	}
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
	return;
}


//void Disp_Quad(Chess_Board_Quad oQuad, int bi16 = 1, int iIndex = -1)
//{
//	//printf("Corner:%d\n",iIndex);
//	if (bi16)
//	{
//		for (int i = 0; i < 4; i++)
//		{
//			unsigned short* pPos = oQuad.Corner_i[i];;
//			printf("%d %d\n", pPos[0], pPos[1]);
//		}
//	}
//	else
//	{
//		for (int i = 0; i < 4; i++)
//		{
//			float* pPos = oQuad.Corner[i]->Pos;
//			printf("%f %f\n", pPos[0], pPos[1]);
//		}
//	}
//	printf("Neighbour\n");
//	for (int i = 0; i < 4; i++)
//	{
//		if (oQuad.Neighbour[i].m_iQuad_Index == -1)
//			printf("%d\n", oQuad.Neighbour[i].m_iQuad_Index);
//		else
//		{
//			for (int j = 0; j < 4; j++)
//			{
//				Chess_Board_Quad* poNeighbour = (Chess_Board_Quad*)oQuad.Neighbour[i].ptr;
//				printf("\t%f %f\n", poNeighbour->Corner[j]->Pos[0],
//					poNeighbour->Corner[j]->Pos[1]);
//			}
//		}
//	}
//
//	//Disp(oQuad.Neighbour, 4, 1, "Neighbour");
//	printf("Count:%d edge_sqr_len:%f ordered:%d\n", oQuad.m_iNeighbour_Count, oQuad.edge_sqr_len, oQuad.ordered);
//	printf("\n");
//
//	return;
//}

void Disp_Quad(Chess_Board_Quad oQuad, int bi16 = 1)
{
	//printf("Corner:%d\n",iIndex);
	if (bi16)
	{
		for (int i = 0; i < 4; i++)
		{
			unsigned short* pPos = oQuad.Corner_i[i];;
			printf("%d %d\n", pPos[0], pPos[1]);
		}
	}
	else
	{
		for (int i = 0; i < 4; i++)
		{
			float* pPos = oQuad.Corner[i]->Pos;
			printf("%f %f\n", pPos[0], pPos[1]);
		}
	}
	printf("Neighbour\n");
	for (int i = 0; i < 4; i++)
	{
		if (oQuad.Neighbour[i].m_iQuad_Index == -1)
			printf("%d\n", oQuad.Neighbour[i].m_iQuad_Index);
		else
		{
			//for (int j = 0; j < 4; j++)
			{
				Chess_Board_Quad* poNeighbour = (Chess_Board_Quad*)oQuad.Neighbour[i].ptr;
				if (poNeighbour)
				{
					printf("Neighbour:%d \t%f %f\n", i, poNeighbour->Corner[0]->Pos[0],
						poNeighbour->Corner[0]->Pos[1]);
				}
			}
		}
	}

	//Disp(oQuad.Neighbour, 4, 1, "Neighbour");
	printf("Count:%d edge_sqr_len:%f ordered:%d\n", oQuad.m_iNeighbour_Count, oQuad.edge_sqr_len, oQuad.ordered);
	printf("\n");

	return;
}

void Disp_Quads(Chess_Board_Quad* pQuad, int iQuad_Count, int bi16 = 1)
{
	for (int i = 0; i < iQuad_Count; i++)
	{
		printf("Contour:%d\n", i);
		Disp_Quad(pQuad[i], bi16);
	}
	return;
}
void Disp_Quads(Chess_Board_Quad* Quad[], int iQuad_Count, int bi16 = 1)
{
	for (int i = 0; i < iQuad_Count; i++)
	{
		printf("Contour:%d\n", i);
		Disp_Quad(*Quad[i], bi16);
	}

	return;
}


static void Draw_Chess_Board(const char* pcFile, int iWidth_In_Grad, int iHeight_In_Grad, int iGrid_Size, int x_Start, int y_Start)
{//画一个棋盘，从(x_Start,y_Start)开始画
	int x, y, iColor;
	Image oImage;
	Init_Image(&oImage, 800, 800, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage, 255, 255, 255);

	for (y = 0; y < iHeight_In_Grad; y++)
	{
		iColor = (y & 1);
		int y_Start_1 = y_Start + y * iGrid_Size;
		for (x = 0; x < iWidth_In_Grad; x++)
		{
			int x1, y1, x_Start_1 = x_Start + x * iGrid_Size;
			for (y1 = y_Start_1; y1 < y_Start_1 + iGrid_Size; y1++)
				for (x1 = x_Start_1; x1 < x_Start_1 + iGrid_Size; x1++)
					oImage.m_pChannel[0][y1 * oImage.m_iWidth + x1] = iColor * 255;

			iColor = !iColor;
		}
	}
	bSave_Image(pcFile, oImage);
	return;
}

template<typename _T>int bIs_Contour_Convex(_T Point[][2], int n)
{//判断一个域是否为凸，其实原理很简单，通过叉乘，点乘推导角度，
//不需要求精确，只需分别(0,180) 还是(180, 360)即可，不用求反函数
	_T prev_pt[2], cur_pt[2] = { Point[n - 1][0],Point[n - 1][1] };
	//显然，prev_pt是倒数第2点
	memcpy(prev_pt, &Point[(n - 2 + n) % n], 2 * sizeof(_T));
	float dx0 = (float)(cur_pt[0] - prev_pt[0]);
	float dy0 = (float)(cur_pt[1] - prev_pt[1]);
	int orientation = 0;
	for (int i = 0; i < n; i++)
	{
		float dxdy0, dydx0;
		float dx, dy;

		prev_pt[0] = cur_pt[0];
		prev_pt[1] = cur_pt[1];

		cur_pt[0] = Point[i][0];
		cur_pt[1] = Point[i][1];

		dx = (float)(cur_pt[0] - prev_pt[0]);
		dy = (float)(cur_pt[1] - prev_pt[1]);
		dxdy0 = dx * dy0;
		dydx0 = dy * dx0;

		// find orientation
		// orient = -dy0 * dx + dx0 * dy;
		// orientation |= (orient > 0) ? 1 : 2;
		orientation |= (dydx0 > dxdy0) ? 1 : ((dydx0 < dxdy0) ? 2 : 3);
		if (orientation == 3)
			return 0;

		dx0 = dx;
		dy0 = dy;
	}

	return 1;
}


int bGenerate_Quad(Image oImage, Chess_Board_Quad** ppQuad, int* piQuad_Count, int* piMax_Quad_Count, int iDilation)
{
	const int Min_Level = 1, Max_Level = 7, Min_Area = 25;
	Get_Contour_Result oResult;
	Chess_Board_Quad* pAll_Quad = NULL;
	unsigned long long tStart;
	if (!bGet_Contour(oImage, &oResult))
		return 0;
	//Draw_Contour_Outline("c:\\tmp\\1.bmp", oResult,0);

	int i, iBoard_Index, iNew_Point_Count, iMax_Point_Count = 0, iQuad_Count = 0;
	int iQuad_Count_1 = 0;
	//先找到根节点
	Get_Contour_Result::Contour oContour = oResult.m_pContour[0];
	iBoard_Index = 0;
	while (oContour.hierachy[3] != -1)
	{
		iBoard_Index = oContour.hierachy[3];
		oContour = oResult.m_pContour[iBoard_Index];
	}

	for (i = 0; i < oResult.m_iContour_Count; i++)
	{
		oContour = oResult.m_pContour[i];
		if (oContour.m_iOutline_Point_Count > iMax_Point_Count)
			iMax_Point_Count = oContour.m_iOutline_Point_Count;
		if (oContour.m_iArea > 2)
			iQuad_Count++;
	}

	int bRet = 0;
	unsigned int* pContour_Quad = (unsigned int*)pMalloc(iQuad_Count * sizeof(int));
	unsigned short (*pNew_Point)[2] = (unsigned short(*)[2])pMalloc(iMax_Point_Count * 2 * sizeof(unsigned short));
	if (!pContour_Quad || !pNew_Point)
		goto END;

	//hierachy[0]：同级前一个轮廓 hierachy[1]：同级下一个轮廓
	//hierachy[2]:第一个子轮廓	hierachy[3]：父轮廓
	for (iQuad_Count = 0, i = oResult.m_iContour_Count - 1; i >= 0; i--)
	{
		oContour = oResult.m_pContour[i];
		//如果是根节点，或者有孔，都不能作为块
		if (oContour.hierachy[3] == -1 || oContour.hierachy[2] != -1)
			continue;

		unsigned short Contour_Rect[2][2];
		Get_Bounding_Box(oContour.m_pPoint, oContour.m_iOutline_Point_Count, Contour_Rect);
		if ((Contour_Rect[1][0] - Contour_Rect[0][0] + 1) * (Contour_Rect[1][1] - Contour_Rect[0][1] + 1) < Min_Area)
			continue;

		iNew_Point_Count = oContour.m_iOutline_Point_Count;

		int iLevel = Min_Level;
		for (; iLevel <= Max_Level && iNew_Point_Count > 4; iLevel++)
		{
			approxPolyDP(oContour.m_pPoint, oContour.m_iOutline_Point_Count, pNew_Point, &iNew_Point_Count, (float)iLevel);
			memcpy(oContour.m_pPoint, pNew_Point, iNew_Point_Count * 2 * sizeof(unsigned short));
			oContour.m_iOutline_Point_Count = iNew_Point_Count;
		}

		if (iNew_Point_Count != 4)
			continue;

		if (iLevel == Min_Level)
			memcpy(pNew_Point, oContour.m_pPoint, 4 * 2 * sizeof(unsigned short));

		//判断是否为凸
		if (!bIs_Contour_Convex(pNew_Point, iNew_Point_Count))
			continue;

		pContour_Quad[iQuad_Count++] = i;
		//printf("%d\n", i);
	}

	Free(pNew_Point), pNew_Point = NULL;
	if (iQuad_Count)
		Shrink(pContour_Quad, iQuad_Count * sizeof(int));
	else
		Free(pContour_Quad), pContour_Quad = NULL;

	int iMax_Quad_Count;
	iMax_Quad_Count = Max(2, iQuad_Count * 3);
	pAll_Quad = (Chess_Board_Quad*)pMalloc(iMax_Quad_Count * sizeof(Chess_Board_Quad));
	if (!pAll_Quad)
		goto END;
	iQuad_Count_1 = 0;	//这个才是最后的Quad_Cunbt?

	tStart = iGet_Tick_Count();
	for (i = 0; i < iQuad_Count; i++)
	{
		oContour = oResult.m_pContour[pContour_Quad[i]];

		Chess_Board_Quad* poQuad = &pAll_Quad[iQuad_Count_1++];
		memcpy(poQuad->Corner_i, oContour.m_pPoint, 4 * 2 * sizeof(unsigned short));
		//memset(poQuad->Neighbour, -1, 4 * sizeof(Quad_Neighbour_Item));
		for (int j = 0; j < 4; j++)
			poQuad->Neighbour[j] = { -1,-1 };

		poQuad->m_iNeighbour_Count = 0;
		poQuad->col = poQuad->row = 0;
		poQuad->ordered = 0;
		poQuad->m_iGroup_Index = -1;
		poQuad->edge_sqr_len = (float)0xFFFFFFFF;

		//此处求每一个四边形的边长平方，用以后面过滤邻接过大过小的四边形
		//if (q_k.edge_sqr_len > 16 * cur_quad.edge_sqr_len ||
		//cur_quad.edge_sqr_len > 16 * q_k.edge_sqr_len)
		//经过判断，过大国小都不要
		for (int j = 0; j < 4; j++)
		{//对于一个四边形，分别求四条边的边长平方和
			int iNext = (j + 1) & 3;
			int iEdge_Size = oContour.m_pPoint[j][0] - oContour.m_pPoint[iNext][0];
			unsigned int sqr = iEdge_Size * iEdge_Size;
			iEdge_Size = oContour.m_pPoint[j][1] - oContour.m_pPoint[iNext][1];
			sqr += iEdge_Size * iEdge_Size;

			/*unsigned int sqr = (oContour.m_pPoint[j][0] - oContour.m_pPoint[(j + 1) & 3][0]) *
				(oContour.m_pPoint[j][0] - oContour.m_pPoint[(j + 1) & 3][0]) +
				(oContour.m_pPoint[j][1] - oContour.m_pPoint[(j + 1) & 3][1]) *
				(oContour.m_pPoint[j][1] - oContour.m_pPoint[(j + 1) & 3][1]);*/
				//取四条边最小者的平方为edge_sqr_len

			//取边面积最小者，感觉越平越小
			if (sqr < poQuad->edge_sqr_len)
				poQuad->edge_sqr_len = (float)sqr;
			//printf("%f\n", poQuad->edge_sqr_len);
		}

		//此处还要根据腐蚀膨胀求个补偿，腐蚀越大，补偿越大
		const int edge_len_compensation = 2 * iDilation;
		poQuad->edge_sqr_len += (float)(2 * sqrt(poQuad->edge_sqr_len) * edge_len_compensation + edge_len_compensation * edge_len_compensation);
		//Disp_Quad(*poQuad);
	}

	iMax_Quad_Count = iQuad_Count_1 * 2;
	if (iMax_Quad_Count)
		Shrink(pAll_Quad, iMax_Quad_Count * sizeof(Chess_Board_Quad));
	else
		Free(pAll_Quad), pAll_Quad = NULL;

	Free(pContour_Quad);
	Free_Contour_Result(&oResult);

	*ppQuad = pAll_Quad;
	*piQuad_Count = iQuad_Count_1;
	*piMax_Quad_Count = iMax_Quad_Count;
	bRet = 1;
END:
	if (!bRet)
	{
		Free(pAll_Quad);
		Free(pContour_Quad);
		Free(pNew_Point);
	}
	return bRet;
}

void Quad_i16_2_float(Chess_Board_Quad* Quad, int iQuad_Count, Chess_Board_Corner Corner[], int* piCur_Corner)
{//没啥营养，就是整数转小数
	int iCur_Corner = *piCur_Corner;
	for (int i = 0; i < iQuad_Count; i++)
	{
		Chess_Board_Quad* poQuad = &Quad[i];
		for (int j = 3; j >= 0; j--)
		{
			Chess_Board_Corner* poCorner = &Corner[iCur_Corner++];
			*poCorner = {};
			poCorner->Pos[0] = poQuad->Corner_i[j][0];
			poCorner->Pos[1] = poQuad->Corner_i[j][1];
			poQuad->Corner[j] = poCorner;
		}
	}
	*piCur_Corner = iCur_Corner;
	return;
}

template<typename _T>_T fGet_Sqr(_T fDiff_x, _T fDiff_y)
{
	return fDiff_x * fDiff_x + fDiff_y * fDiff_y;
}
template<typename _T>int bPoints_On_Same_Side_From_Line(float Line_Point_1[2], float Line_Point_2[2], _T Point_1, _T Point_2)
{
	double line_direction_vector[] = { Line_Point_2[0] - Line_Point_1[0],Line_Point_2[1] - Line_Point_1[1] };
	double vector1[] = { Point_1[0] - Line_Point_1[0], Point_1[1] - Line_Point_1[1] };
	double vector2[] = { Point_2[0] - Line_Point_1[0],Point_2[1] - Line_Point_1[1] };
	double fValue = line_direction_vector[0] * vector1[1] - line_direction_vector[1] * vector1[0];
	fValue *= line_direction_vector[0] * vector2[1] - line_direction_vector[1] * vector2[0];
	return fValue > 0;
}
static int iQuick_Sort_Partition(Quad_Neighbour_Item pBuffer[], int left, int right)
{//小到大的顺序
 //Real fRef;
	Quad_Neighbour_Item iValue, oTemp;
	int pos = right;

	//试一下将中间元素交换到头，多一步可能挽救了极端差形态
	oTemp = pBuffer[right];
	pBuffer[right] = pBuffer[(left + right) >> 1];
	pBuffer[(left + right) >> 1] = oTemp;

	right--;
	iValue = pBuffer[pos];
	while (left <= right)
	{
		while (left < pos && pBuffer[left].m_fDist <= iValue.m_fDist)
			left++;
		while (right >= 0 && pBuffer[right].m_fDist > iValue.m_fDist)
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
int iAdjust_Left(Quad_Neighbour_Item* pStart, Quad_Neighbour_Item* pEnd)
{
	Quad_Neighbour_Item oTemp, * pCur_Left = pEnd - 1,
		* pCur_Right;
	Quad_Neighbour_Item oRef = *pEnd;

	//为了减少一次判断，此处先扫过去
	while (pCur_Left >= pStart && (pCur_Left->m_iCorner_Index == oRef.m_iCorner_Index && pCur_Left->m_iQuad_Index == oRef.m_iQuad_Index))
		pCur_Left--;
	pCur_Right = pCur_Left;
	pCur_Left--;

	while (pCur_Left >= pStart)
	{
		if (pCur_Left->m_iCorner_Index == oRef.m_iCorner_Index && pCur_Left->m_iQuad_Index == oRef.m_iQuad_Index)
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
static void Quick_Sort(Quad_Neighbour_Item Seq[], int iStart, int iEnd)
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

int bFind_Neighbour(Chess_Board_Quad Quad[], int iQuad_Count, float To_Find[2], int iCur_Quad_Index, int iCur_Corner_Index, float sqr_radius,
	int* piClosest_Quad_Index, int* piClosest_Corner_Index, float* piDist)
{//寻找半径范围内最近邻
	const float thresh_sqr_scale = 2.0;
	float fMin_Dist = (float)0xFFFFFFF,
		iDist = 0, bFound = 0;
	int  iClosest_Quad = -1, iClosest_Corner = -1, iClosest_Neighbour_Index = -1,
		bRet = 1;
#define MAX_NEIGHBOUR_COUNT 1024
	Quad_Neighbour_Item* Neighbour = (Quad_Neighbour_Item*)pMalloc(MAX_NEIGHBOUR_COUNT * sizeof(Quad_Neighbour_Item));
	if (!Neighbour)
	{
		bRet = 0;
		goto END;
	}

	int i, j, iNeighbour_Count;
	iNeighbour_Count = 0;
	for (i = 0; i < iQuad_Count; i++)
	{//慢速查找
		if (i == iCur_Quad_Index)
			continue;
		for (j = 0; j < 4; j++)
		{
			float* pCorner = Quad[i].Corner[j]->Pos;
			iDist = fGet_Sqr(pCorner[0] - To_Find[0], pCorner[1] - To_Find[1]);
			if (iDist < sqr_radius)
				Neighbour[iNeighbour_Count++] = { (short)i,(short)j,iDist };
			if (iNeighbour_Count >= MAX_NEIGHBOUR_COUNT)
			{
				printf("Insufficient Neighbour Count in bFind_Neighbour\n");
				bRet = 0;
				goto END;
			}
		}
	}

	//此处对Neighbour排个序
	Quick_Sort(Neighbour, 0, iNeighbour_Count - 1);

	Chess_Board_Quad oCur_Quad;
	oCur_Quad = Quad[iCur_Quad_Index];
	for (i = 0; i < iNeighbour_Count; i++)
	{
		Quad_Neighbour_Item oNeighbour = Neighbour[i];
		//int iTemp = (oNeighbour.m_iQuad_Index << 2) + oNeighbour.m_iCorner_Index;
		/*if (iCur_Quad_Index == 2 && iCur_Corner_Index == 1)
		printf("here");*/

		float sqr_dist = oNeighbour.m_fDist;
		Chess_Board_Quad q_k = Quad[oNeighbour.m_iQuad_Index];
		if (sqr_dist <= oCur_Quad.edge_sqr_len * thresh_sqr_scale &&
			sqr_dist <= q_k.edge_sqr_len * thresh_sqr_scale)
		{
			if (q_k.edge_sqr_len > 16 * oCur_Quad.edge_sqr_len ||
				oCur_Quad.edge_sqr_len > 16 * q_k.edge_sqr_len)
				continue;

			float mid_pt1[2] = { (oCur_Quad.Corner[iCur_Corner_Index]->Pos[0] + oCur_Quad.Corner[(iCur_Corner_Index + 1) & 3]->Pos[0]) / 2.f,
				(oCur_Quad.Corner[iCur_Corner_Index]->Pos[1] + oCur_Quad.Corner[(iCur_Corner_Index + 1) & 3]->Pos[1]) / 2.f };
			float mid_pt2[2] = { (oCur_Quad.Corner[(iCur_Corner_Index + 2) & 3]->Pos[0] + oCur_Quad.Corner[(iCur_Corner_Index + 3) & 3]->Pos[0]) / 2.f,
				(oCur_Quad.Corner[(iCur_Corner_Index + 2) & 3]->Pos[1] + oCur_Quad.Corner[(iCur_Corner_Index + 3) & 3]->Pos[1]) / 2.f };

			if (!bPoints_On_Same_Side_From_Line(mid_pt1, mid_pt2, To_Find, q_k.Corner[oNeighbour.m_iCorner_Index]->Pos))
				continue;

			float mid_pt3[2] = { (oCur_Quad.Corner[(iCur_Corner_Index + 1) & 3]->Pos[0] + oCur_Quad.Corner[(iCur_Corner_Index + 2) & 3]->Pos[0]) / 2.f,
				(oCur_Quad.Corner[(iCur_Corner_Index + 1) & 3]->Pos[1] + oCur_Quad.Corner[(iCur_Corner_Index + 2) & 3]->Pos[1]) / 2.f };
			float mid_pt4[2] = { (oCur_Quad.Corner[(iCur_Corner_Index + 3) & 3]->Pos[0] + oCur_Quad.Corner[iCur_Corner_Index]->Pos[0]) / 2.f,
				(oCur_Quad.Corner[(iCur_Corner_Index + 3) & 3]->Pos[1] + oCur_Quad.Corner[iCur_Corner_Index]->Pos[1]) / 2.f };

			if (!bPoints_On_Same_Side_From_Line(mid_pt3, mid_pt4, To_Find, q_k.Corner[oNeighbour.m_iCorner_Index]->Pos))
				continue;
			float neighbor_pt_diagonal[] = { q_k.Corner[(oNeighbour.m_iCorner_Index + 2) & 3]->Pos[0],q_k.Corner[(oNeighbour.m_iCorner_Index + 2) & 3]->Pos[1] };
			if (!bPoints_On_Same_Side_From_Line(mid_pt1, mid_pt2, To_Find, neighbor_pt_diagonal))
				continue;

			if (!bPoints_On_Same_Side_From_Line(mid_pt3, mid_pt4, To_Find, neighbor_pt_diagonal))
				continue;

			iClosest_Neighbour_Index = i;
			iClosest_Quad = oNeighbour.m_iQuad_Index;
			iClosest_Corner = oNeighbour.m_iCorner_Index;
			fMin_Dist = sqr_dist;
			break;
		}
	}

	*piClosest_Corner_Index = iClosest_Corner;
	*piClosest_Quad_Index = iClosest_Quad;
	*piDist = fMin_Dist;

	if (iClosest_Neighbour_Index >= 0 && iClosest_Quad >= 0 && iClosest_Corner >= 0 && fMin_Dist < FLT_MAX)
	{
		//if (cur_quad.count >= 4 || closest_quad->count >= 4)
		//return false;
		//unsigned short* pClosest_Point = Quad[iClosest_Quad].Corner[iClosest_Corner];
		//float pClosest_Point = Quad[iClosest_Quad].Corner_f[iClosest_Corner];
		for (j = 0; j < 4; j++)
		{
			if (oCur_Quad.Neighbour[j].m_iQuad_Index == iClosest_Quad)
				break;
			float* pClosest_Point = Quad[iClosest_Quad].Corner[iClosest_Corner]->Pos;

			if (fGet_Sqr(pClosest_Point[0] - oCur_Quad.Corner[j]->Pos[0], pClosest_Point[1] - oCur_Quad.Corner[j]->Pos[1]) < fMin_Dist)
				break;
		}
		if (j < 4)
			goto END;

		Chess_Board_Quad oClosest_Quad = Quad[iClosest_Quad];
		for (j = 0; j < 4; j++)
		{
			if (oClosest_Quad.Neighbour[j].m_iQuad_Index == iCur_Quad_Index)
				break;
		}
		if (j < 4)
			goto END;
		bRet = 1;
		goto END;
	}
	bRet = 0;
END:
	Free(Neighbour);
	return bRet;
}

void Find_Quad_Neighbour(Chess_Board_Quad Quad[], int iQuad_Count)
{
	static int iCounter = 0;
	const int thresh_sqr_scaleb = 2;
	for (int i = 0; i < iQuad_Count; i++)
	{
		Chess_Board_Quad* poCur_Quad = &Quad[i], oCur = *poCur_Quad;
		for (int j = 0; j < 4; j++)
		{
			if (oCur.Neighbour[j].m_iQuad_Index != -1)
				continue;	//已有主

			//先初始化为无效值
			float min_sqr_dist = (float)0xFFFFFFF;
			int closest_quad_idx = -1;
			int closest_corner_idx = -1;
			float sqr_radius = oCur.edge_sqr_len * thresh_sqr_scaleb + 1;
			/*if (iCounter == 46827)
				printf("here");*/
			int bFound = bFind_Neighbour(Quad,
				iQuad_Count,
				oCur.Corner[j]->Pos,
				i, j,
				sqr_radius,
				&closest_quad_idx,
				&closest_corner_idx,
				&min_sqr_dist);
			iCounter++;
			if (!bFound)
				continue;

			sqr_radius = min_sqr_dist + 1;
			min_sqr_dist = FLT_MAX;

			int closest_closest_quad_idx = -1;
			int closest_closest_corner_idx = -1;

			Chess_Board_Quad* poClosest_Quad = &Quad[closest_quad_idx];
			float* pCloest_Corner = poClosest_Quad->Corner[closest_corner_idx]->Pos;
			

			bFound = bFind_Neighbour(Quad,
				iQuad_Count,
				pCloest_Corner,
				closest_quad_idx, closest_corner_idx,
				sqr_radius,
				&closest_closest_quad_idx,
				&closest_closest_corner_idx,
				&min_sqr_dist);
			if (!bFound)
				continue;

			if (closest_closest_quad_idx != i || closest_closest_corner_idx != j)
				continue;

			pCloest_Corner[0] = (oCur.Corner[j]->Pos[0] + pCloest_Corner[0]) * 0.5f;
			pCloest_Corner[1] = (oCur.Corner[j]->Pos[1] + pCloest_Corner[1]) * 0.5f;

			poCur_Quad->m_iNeighbour_Count++;
			poCur_Quad->Neighbour[j] = { (short)closest_quad_idx,(short)closest_corner_idx,min_sqr_dist,&Quad[closest_quad_idx] };
			//poCur_Quad->Corner[j]->Pos[0] = pCloest_Corner[0];
			//poCur_Quad->Corner[j]->Pos[1] = pCloest_Corner[1];
			poCur_Quad->Corner[j] = poClosest_Quad->Corner[closest_corner_idx];

			poClosest_Quad->m_iNeighbour_Count++;
			poClosest_Quad->Neighbour[closest_corner_idx] = { (short)i,(short)j,min_sqr_dist,&Quad[i] };
			
		}
	}
	return;
}

int bFind_Connected_Quads(Chess_Board_Quad Quad[], int iQuad_Coubt, Chess_Board_Quad* Quad_Group[], int iGroup_Index, int* piGroup_Size)
{//其实就是找Quad级的连通域，没啥新鲜的
	Chess_Board_Quad* poQuad;
	Chess_Board_Quad** Stack = (Chess_Board_Quad**)pMalloc(iQuad_Coubt * sizeof(Chess_Board_Quad*));
	if (!Stack)
		return 0;

	int iTop = 0,
		iGroup_Size = 0;
	for (int i = 0; i < iQuad_Coubt; i++)
	{
		/*if (i == 120)
			printf("here");*/
		poQuad = &Quad[i];
		if (poQuad->m_iNeighbour_Count <= 0 || poQuad->m_iGroup_Index >= 0)
			continue;
		//Disp_Quad(*poQuad, 0);
		//压入栈
		Stack[iTop++] = Quad_Group[iGroup_Size++] = poQuad;
		poQuad->m_iGroup_Index = iGroup_Index;

		while (iTop > 0)
		{
			poQuad = Stack[--iTop];
			for (int j = 0; j < 4; j++)
			{
				Quad_Neighbour_Item oNeighbour = poQuad->Neighbour[j];
				if (oNeighbour.m_iQuad_Index != -1 && Quad[oNeighbour.m_iQuad_Index].m_iGroup_Index == -1)
				{//有邻居
					Stack[iTop++] = Quad_Group[iGroup_Size++] = &Quad[oNeighbour.m_iQuad_Index];
					Quad[oNeighbour.m_iQuad_Index].m_iGroup_Index = iGroup_Index;
				}
			}
		}
		break;
	}
	*piGroup_Size = iGroup_Size;
	Free(Stack);
	return 1;
}

void Order_Quad(Chess_Board_Quad Quad[], Chess_Board_Quad* poQuad, float Corner[2], int common)
{//暂时不知其义
	int tc = 0;
	for (tc; tc < 4; ++tc)
		if (poQuad->Corner[tc]->Pos[0] == Corner[0] && poQuad->Corner[tc]->Pos[1] == Corner[1])
			break;

	//感觉以下做了个旋转
	while (tc != common)
	{
		Chess_Board_Corner* tempc = poQuad->Corner[3];
		Quad_Neighbour_Item tempq = poQuad->Neighbour[3];
		for (int i = 3; i > 0; --i)
		{
			poQuad->Corner[i] = poQuad->Corner[i - 1];			poQuad->Neighbour[i] = poQuad->Neighbour[i - 1];
		}
		poQuad->Corner[0] = tempc;
		poQuad->Neighbour[0] = tempq;

		tc = (tc + 1) & 3;
	}
	return;
}

int Add_Outer_Quad(Chess_Board_Quad Quad[], int* piQuad_Count, int iMax_Quad_Count,
	Chess_Board_Corner All_Corner[], int* piCorner_Count,int iMax_Corner_Count,
	Chess_Board_Quad* poQuad, Chess_Board_Quad* Quad_Group[], int* piGroup_Size)
{
	static int iCounter = 0;
	int iQuad_Count = *piQuad_Count,
		iGroup_Size = *piGroup_Size,
		iCorner_Count = *piCorner_Count;

	int added = 0;
	int max_quad_buf_size = iMax_Quad_Count;
	for (int i = 0; i < 4 && iQuad_Count < max_quad_buf_size; i++) // find no-neighbor corners
	{
		if (poQuad->Neighbour[i].m_iQuad_Index == -1)    // ok, create and add neighbor
		{
			int j = (i + 2) & 3;
			//printf("Adding quad as neighbor 2\n");
			int q_index = iQuad_Count++;
			Chess_Board_Quad* q = &Quad[q_index];
			Quad_Group[iGroup_Size++] = q;
			*q = {};
			q->Neighbour[0] = { 0,-1,-1,NULL };
			q->Neighbour[1] = { 0,-1,-1,NULL };
			q->Neighbour[2] = { 0,-1,-1,NULL };
			q->Neighbour[3] = { 0,-1,-1,NULL };
			added++;

			// set neighbor and group id
			poQuad->Neighbour[i].m_iQuad_Index = q_index;
			poQuad->Neighbour[i].ptr = q;
			poQuad->m_iNeighbour_Count++;
			q->Neighbour[j].ptr = poQuad;
			q->Neighbour[j].m_iQuad_Index = (short)(poQuad - Quad);
			q->m_iGroup_Index = poQuad->m_iGroup_Index;
			q->m_iNeighbour_Count = 1;   // number of neighbors
			q->ordered = false;
			q->edge_sqr_len = poQuad->edge_sqr_len;

			// make corners of new quad
			// same as neighbor quad, but offset
			float pt_offset[2] = { poQuad->Corner[i]->Pos[0] - poQuad->Corner[j]->Pos[0],
				poQuad->Corner[i]->Pos[1] - poQuad->Corner[j]->Pos[1] };
			for (int k = 0; k < 4; k++)
			{
				/*if (iCounter == 172)
					printf("here");*/

				//Chess_Board_Corner_1 *corner = &All_Corner[q_index * 4 + k];
				Chess_Board_Corner* corner = &All_Corner[iCorner_Count++];
				if (iCorner_Count >= iMax_Corner_Count)
				{
					printf("Err");
					exit(0);
				}

				float* pt = poQuad->Corner[k]->Pos;
				corner->Pos[0] = pt[0], corner->Pos[1] = pt[1],
					q->Corner[k] = corner;
				corner->Pos[0] += pt_offset[0], corner->Pos[1] += pt_offset[1];
				iCounter++;
			}
			q->Corner[j] = poQuad->Corner[i];

			// set row and col for next step check
			switch (i)
			{
			case 0:
				q->col = poQuad->col - 1; q->row = poQuad->row - 1;
				break;
			case 1:
				q->col = poQuad->col + 1; q->row = poQuad->row - 1;
				break;
			case 2:
				q->col = poQuad->col + 1; q->row = poQuad->row - 1;
				break;
			case 3:
				q->col = poQuad->col - 1; q->row = poQuad->row + 1;
				break;
			}

			// now find other neighbor and add it, if possible
			for (int k = 1; k <= 3; k += 2)
			{
				int next_i = (i + k) % 4;
				int prev_i = (i + k + 2) % 4;
				Chess_Board_Quad* quad_prev = (Chess_Board_Quad*)poQuad->Neighbour[prev_i].ptr;
				if (quad_prev &&
					quad_prev->ordered &&
					quad_prev->Neighbour[i].m_iQuad_Index != -1 &&
					((Chess_Board_Quad*)quad_prev->Neighbour[i].ptr)->ordered &&
					std::abs(((Chess_Board_Quad*)quad_prev->Neighbour[i].ptr)->col - q->col) == 1 &&
					std::abs(((Chess_Board_Quad*)quad_prev->Neighbour[i].ptr)->row - q->row) == 1)
				{
					Chess_Board_Quad* qn = (Chess_Board_Quad*)quad_prev->Neighbour[i].ptr;
					q->m_iNeighbour_Count = 2;
					q->Neighbour[prev_i].ptr = qn;
					qn->Neighbour[next_i].ptr = q;
					qn->m_iNeighbour_Count += 1;
					// have to set exact corner
					q->Corner[prev_i] = qn->Corner[next_i];
				}
			}
		}
	}
	*piQuad_Count = iQuad_Count;
	*piCorner_Count = iCorner_Count;
	//iCounter++;
	return added;
}

void Remove_Quad_From_Group(Chess_Board_Quad* Quad_Group[], int* piGroup_Size, Chess_Board_Quad* q0)
{
	int iGroup_Size = *piGroup_Size;
	const int count = iGroup_Size;
	int self_idx = -1;
	// remove any references to this quad as a neighbor
	for (int i = 0; i < count; ++i)
	{
		Chess_Board_Quad* q = Quad_Group[i];
		if (q == q0)
			self_idx = i;
		for (int j = 0; j < 4; j++)
		{
			if (q->Neighbour[j].ptr == q0)
			{
				q->Neighbour[j].ptr = NULL;
				q->Neighbour[j].m_iCorner_Index = -1;
				q->Neighbour[j].m_iQuad_Index = -1;
				q->m_iGroup_Index = -1;
				q->m_iNeighbour_Count--;
				for (int k = 0; k < 4; ++k)
				{
					if (q0->Neighbour[k].ptr == q)
					{
						q0->Neighbour[k].ptr = NULL;
						q0->Neighbour[k].m_iCorner_Index = -1;
						q0->Neighbour[k].m_iQuad_Index = -1;
						q0->m_iNeighbour_Count--;
					}
				}

				break;
			}
		}
	}

	if (self_idx != count - 1)
		Quad_Group[self_idx] = Quad_Group[count - 1];
	*piGroup_Size = count - 1;
}

int iOrder_Found_Connected_Quads(Chess_Board_Quad Quad[], int* piQuad_Count, int iMax_Quad_Count,
	Chess_Board_Corner All_Corner[], int* piCorner_Count,int iMax_Corner_Count,
	Chess_Board_Quad* Quad_Group[],	int* piGroup_Size, unsigned char Pattern_Size[2], int iParent_Counter = -1)
{//将乱七八糟的四边形组排列好，具体怎么做还不知道
 //返回个数
	int i;
	//首先确定开始的块
	int iQuad_Count = *piQuad_Count,
		iGroup_Size = *piGroup_Size;

	Chess_Board_Quad* poStart = NULL;
	Chess_Board_Quad* Stack[128];	//战
	int iTop = 0;

	//先找找有没有内部节点，所谓内杯节点就是四个方向上都有邻居
	for (i = 0; i < iGroup_Size; i++)
	{
		if (Quad_Group[i]->m_iNeighbour_Count == 4)
		{
			poStart = Quad_Group[i];
			break;
		}
	}
	//Disp_Quad(*poStart, 0);
	if (!poStart)
		return 0;

	//未知其义
	int row_min = 0, col_min = 0, row_max = 0, col_max = 0;
	//int col_hist[16], row_hist[16];	//又搞直方图？

	Stack[iTop++] = poStart;
	poStart->row = 0;
	poStart->col = 0;
	poStart->ordered = true;
	static int iCounter = 0;
	//if (iParent_Counter == 4)
		//Disp_Quads(Quad_Group, iGroup_Size,0);

	//Disp_Quad(Quad[15],0);
	while (iTop > 0)
	{
		Chess_Board_Quad* q = Stack[--iTop];
		int col = q->col;
		int row = q->row;

		if (row > row_max)
			row_max = row;
		if (row < row_min)
			row_min = row;
		if (col > col_max)
			col_max = col;
		if (col < col_min)
			col_min = col;
		//if (iParent_Counter == 4 /*&& iCounter==8*/)
		//{
		//	printf("Counter:%d\n", iCounter);
		//	Disp_Quad(*q,0);
		//}

		for (int i = 0; i < 4; i++)
		{
			Chess_Board_Quad* poNeighbour = q->Neighbour[i].m_iQuad_Index == -1 ? NULL : &Quad[q->Neighbour[i].m_iQuad_Index];
			//Disp_Quad(*poNeighbour,0);
			switch (i)
			{
			case 0:
				row--; col--;
				break;
			case 1:
				col += 2;
				break;
			case 2:
				row += 2;
				break;
			case 3:
				col -= 2;
				break;
			}
			if (poNeighbour && poNeighbour->ordered == 0 && poNeighbour->m_iNeighbour_Count == 4)
			{
				Order_Quad(Quad, poNeighbour, q->Corner[i]->Pos, (i + 2) & 3);
				poNeighbour->ordered = 1;
				poNeighbour->row = row;
				poNeighbour->col = col;
				Stack[iTop++] = poNeighbour;
			}
			//iCounter++;
		}
	}

	int w = Pattern_Size[0] - 1;
	int h = Pattern_Size[1] - 1;
	int drow = row_max - row_min + 1;
	int dcol = col_max - col_min + 1;

	if ((w > h && dcol < drow) || (w < h && drow < dcol))
	{
		h = Pattern_Size[0] - 1;
		w = Pattern_Size[1] - 1;
	}

	if (dcol < w || drow < h)   // found enough inner quads?
	{
		//printf("Too few inner quad rows/cols\n");
		return 0;   // no, return
	}

	int bFound = 0;
	for (int i = 0; i < iGroup_Size; ++i)
	{
		Chess_Board_Quad* q = Quad_Group[i];
		/*if (i == 48 && iParent_Counter == 6)
			Disp_Quad(*q, 0);*/

		if (q->m_iNeighbour_Count != 4)
			continue;
		int col = q->col;
		int row = q->row;

		for (int j = 0; j < 4; j++)
		{
			/*if (iParent_Counter == 18 && i==30 && j==1)
				printf("i:%d order:%d \n",i, (Quad_Group[32])->ordered);*/
			switch (j)   // adjust col, row for this quad
			{           // start at top left, go clockwise
			case 0:
				row--; col--;
				break;
			case 1:
				col += 2;
				break;
			case 2:
				row += 2;
				break;
			case 3:
				col -= 2;
				break;
			}
			Chess_Board_Quad* poNeighbour = q->Neighbour[j].m_iQuad_Index == -1 ? NULL : &Quad[q->Neighbour[j].m_iQuad_Index];
			if (poNeighbour && !poNeighbour->ordered && // is it an inner quad?
				col <= col_max && col >= col_min && row <= row_max && row >= row_min)
			{
				bFound++;
				Order_Quad(Quad, poNeighbour, q->Corner[j]->Pos, (j + 2) & 3);
				poNeighbour->ordered = 1;
				poNeighbour->row = row;
				poNeighbour->col = col;
			}
		}
	}
	int max_quad_buf_size = iMax_Quad_Count;

	iCounter++;
	if (bFound > 0)
	{
		//printf("Found %d inner quads not connected to outer quads, repairing\n", bFound);
		//Disp_Quads(Quad_Group, iGroup_Size, 0);
		for (int i = 0; i < iGroup_Size && iQuad_Count < max_quad_buf_size; i++)
		{
			Chess_Board_Quad* q = Quad_Group[i];
			if (q->m_iNeighbour_Count < 4 && q->ordered)
			{
				int added = Add_Outer_Quad(Quad, &iQuad_Count, iMax_Quad_Count, All_Corner, piCorner_Count, iMax_Corner_Count,q, Quad_Group, &iGroup_Size);
				iGroup_Size += added;
			}			
		}
		if (iQuad_Count >= max_quad_buf_size)
			return 0;
	}
	if (dcol == w && drow == h) // found correct inner quads
	{
		for (int i = iGroup_Size - 1; i >= 0; i--) // eliminate any quad not connected to an ordered quad
		{
			Chess_Board_Quad* q = Quad_Group[i];
			//Disp_Quad(*q, 0);

			if (q->ordered == 0)
			{
				int outer = 0;
				for (int j = 0; j < 4; j++) // any neighbors that are ordered?
				{
					/*if (iParent_Counter == 18 && i == 53 && j == 2)
						printf("Here");*/
					if (q->Neighbour[j].m_iQuad_Index != -1 && Quad[q->Neighbour[j].m_iQuad_Index].ordered)
						outer = 1;
				}
				if (!outer) // not an outer quad, eliminate
				{
					//Disp_Quad(*q, 0);
					Remove_Quad_From_Group(Quad_Group, &iGroup_Size, q);
					//Disp_Quad(*q, 0);
				}
			}
		}
		*piGroup_Size = iGroup_Size;
		return iGroup_Size;
	}
	
	return 0;
}

void Clean_Found_Connected_Quads(Chess_Board_Quad* Quad_Group[], int iGroup_Size, unsigned char Grid_Size[2])
{
	if (iGroup_Size <= ((Grid_Size[0] + 1) * (Grid_Size[1] + 1) + 1) / 2)
		return;	//这句是废话
	{
		//Draw_Quad_Group("c:\\tmp\\1.bmp", Quad_Group, iGroup_Size);
		printf("Not implemented yet\n");
		return;
	}
}
float fSum_Dist(Chess_Board_Corner oCorner, int* pn)
{
	float fTotal = 0;
	int n = 0;
	for (int i = 0; i < 4; i++)
	{
		if (oCorner.Neighbour[i])
		{
			fTotal += (float)sqrt(fGet_Distance(oCorner.Pos, oCorner.Neighbour[i]->Pos, 2));
			n++;
		}
	}
	*pn = n;
	return fTotal;
}

int bCheck_Board_Monotony(float Corner[][2], int iCount, unsigned char Grid_Size[2])
{//当算法经过一系列的四边形聚类、排序，最终凑齐了数量契合要求（例如 8x6）的角点后，
//它并不会直接返回结果。因为它需要防止一种情况：有些背景噪点被错误地拼进了网格，
// 导致格子发生了不可思议的“拧麻花”或交叉错位
//做最终的几何防错校验，剔除由于伪角点错序、扭曲或严重自交导致的“畸形”网格

	for (int k = 0; k < 2; ++k)
	{
		int max_i = (k == 0 ? Grid_Size[1] : Grid_Size[0]);
		int max_j = (k == 0 ? Grid_Size[0] : Grid_Size[1]) - 1;
		for (int i = 0; i < max_i; ++i)
		{
			float* a = k == 0 ? Corner[i * Grid_Size[0]] : Corner[i];
			float* b = k == 0 ? Corner[(i + 1) * Grid_Size[0] - 1]
				: Corner[(Grid_Size[1] - 1) * Grid_Size[0] + i];
			float dx0 = b[0] - a[0], dy0 = b[1] - a[1];
			if (fabs(dx0) + fabs(dy0) < FLT_EPSILON)
				return 0;
			float prevt = 0;
			for (int j = 1; j < max_j; ++j)
			{
				float* c = k == 0 ? Corner[i * Grid_Size[0] + j]
					: Corner[j * Grid_Size[0] + i];
				float t = ((c[0] - a[0]) * dx0 + (c[1] - a[1]) * dy0) / (dx0 * dx0 + dy0 * dy0);
				if (t < prevt || t > 1)
					return 0;
				prevt = t;
			}
		}
	}
	return 1;
}

int Check_Quad_Group(Chess_Board_Quad Quad[], int iQuad_Count, Chess_Board_Quad* Quad_Group[],
	int iQuad_Group_Size, Chess_Board_Corner* Corner_Group_Out[], unsigned char Grid_Size[2])
{//暂时不知其义
	int bRet = 0;
	const int ROW1 = 1000000;
	const int ROW2 = 2000000;
	const int ROW_ = 3000000;

	int iCorner_Count = 0, iCorner_Out_Count = 0;
	int result = 0;

	int width = 0, height = 0;
	int hist[5] = { 0,0,0,0,0 };	//此处的直方图有没有用?没用就不要了
	Chess_Board_Corner* Corner_Group[128];

	for (int i = 0; i < iQuad_Group_Size; ++i)
	{
		Chess_Board_Quad* q = Quad_Group[i];
		for (int j = 0; j < 4; ++j)
		{
			/*if (i == 45 && j == 0)
				printf("here");*/
			if (q->Neighbour[j].m_iCorner_Index != -1)
			{
				int next_j = (j + 1) & 3;
				Chess_Board_Corner* a = q->Corner[j], * b = q->Corner[next_j];
				int row_flag = q->m_iNeighbour_Count == 1 ? ROW1 : q->m_iNeighbour_Count == 2 ? ROW2 : ROW_;
				if (a->row == 0)
				{
					/*if(iCorner_Count==81)
						printf("%d %f %f\n", iCorner_Count,a->Pos[0],a->Pos[1]);*/
					Corner_Group[iCorner_Count++] = a;
					a->row = row_flag;
				}
				else if (a->row > (unsigned int)row_flag)
				{
					a->row = row_flag;
				}
				if (q->Neighbour[next_j].m_iQuad_Index != -1)
				{
					if (a->m_iCount >= 4 || b->m_iCount >= 4)
						goto END;
					for (int k = 0; k < 4; ++k)
					{
						if (a->Neighbour[k] == b)
							goto END;
						if (b->Neighbour[k] == a)
							goto END;
					}
					a->Neighbour[a->m_iCount++] = b;
					b->Neighbour[b->m_iCount++] = a;
				}
			}
		}
	}
	if (iCorner_Count != Grid_Size[0] * Grid_Size[1])
		goto END;

	{
		Chess_Board_Corner* first = NULL, * first2 = NULL;
		for (int i = 0; i < iCorner_Count; ++i)
		{
			int n = Corner_Group[i]->m_iCount;

			hist[n]++;
			if (!first && n == 2)
			{
				if (Corner_Group[i]->row == ROW1)
					first = Corner_Group[i];
				else if (!first2 && Corner_Group[i]->row == ROW2)
					first2 = Corner_Group[i];
			}
		}

		if (!first)
			first = first2;

		if (!first || hist[0] != 0 || hist[1] != 0 || hist[2] != 4 ||
			hist[3] != (Grid_Size[0] + Grid_Size[1]) * 2 - 8)
			goto END;

		Chess_Board_Corner* cur = first;
		Chess_Board_Corner* right = NULL;
		Chess_Board_Corner* below = NULL;
		Corner_Group_Out[iCorner_Out_Count++] = (cur);

		for (int k = 0; k < 4; ++k)
		{
			Chess_Board_Corner* c = cur->Neighbour[k];
			if (c)
			{
				if (!right)
					right = c;
				else if (!below)
					below = c;
			}
		}
		if (!right || (right->m_iCount != 2 && right->m_iCount != 3) ||
			!below || (below->m_iCount != 2 && below->m_iCount != 3))
			goto END;

		cur->row = 0;
		first = below; // remember the first corner in the next row
		while (1)
		{
			right->row = 0;
			Corner_Group_Out[iCorner_Out_Count++] = right;
			if (right->m_iCount == 2)
				break;
			if (right->m_iCount != 3 || iCorner_Out_Count >= Max(Grid_Size[0], Grid_Size[1]))
				goto END;
			cur = right;
			for (int k = 0; k < 4; ++k)
			{
				Chess_Board_Corner* c = cur->Neighbour[k];
				if (c && c->row > 0)
				{
					int kk = 0;
					for (; kk < 4; ++kk)
					{
						if (c->Neighbour[kk] == below)
							break;
					}
					if (kk < 4)
						below = c;
					else
						right = c;
				}
			}
		}

		width = iCorner_Out_Count;
		if (width == Grid_Size[0])
			height = Grid_Size[1];
		else if (width == Grid_Size[1])
			height = Grid_Size[0];
		else
			goto END;

		for (int i = 1; ; ++i)
		{
			if (!first)
				break;
			cur = first;
			first = 0;
			int j = 0;
			for (; ; ++j)
			{
				cur->row = i;
				Corner_Group_Out[iCorner_Out_Count++] = cur;
				if (cur->m_iCount == 2 + (i < height - 1) && j > 0)
					break;

				right = 0;

				// find a neighbor that has not been processed yet
				// and that has a neighbor from the previous row
				for (int k = 0; k < 4; ++k)
				{
					Chess_Board_Corner* c = cur->Neighbour[k];
					if (c && c->row > (unsigned int)i)
					{
						int kk = 0;
						for (; kk < 4; ++kk)
						{
							if (c->Neighbour[kk] && c->Neighbour[kk]->row == i - 1)
								break;
						}
						if (kk < 4)
						{
							right = c;
							if (j > 0)
								break;
						}
						else if (j == 0)
							first = c;
					}
				}
				if (!right)
					goto END;
				cur = right;
			}

			if (j != width - 1)
				goto END;
		}
	}

	if ((int)iCorner_Out_Count != iCorner_Count)
		goto END;

	if (width != Grid_Size[0])
	{
		std::swap(width, height);

		Chess_Board_Corner** tmp = (Chess_Board_Corner**)pMalloc(iCorner_Out_Count * sizeof(Chess_Board_Corner*));
		if (!tmp)
		{
			bRet = -1;
			goto END;
		}

		memcpy(tmp, Corner_Group_Out, iCorner_Out_Count * sizeof(Chess_Board_Corner*));
		
		for (int i = 0; i < height; ++i)
			for (int j = 0; j < width; ++j)
				Corner_Group_Out[i * width + j] = tmp[j * height + i];
		Free(tmp);
	}

	{
		float* p0 = Corner_Group_Out[0]->Pos,
			* p1 = Corner_Group_Out[Grid_Size[0] - 1]->Pos,
			* p2 = Corner_Group_Out[Grid_Size[0]]->Pos;
		if ((p1[0] - p0[0]) * (p2[1] - p1[1]) - (p1[1] - p0[1]) * (p2[0] - p1[0]) < 0)
		{
			if (width % 2 == 0)
			{
				for (int i = 0; i < height; ++i)
					for (int j = 0; j < width / 2; ++j)
						std::swap(Corner_Group_Out[i * width + j], Corner_Group_Out[i * width + width - j - 1]);
			}
			else
			{
				for (int j = 0; j < width; ++j)
					for (int i = 0; i < height / 2; ++i)
						std::swap(Corner_Group_Out[i * width + j], Corner_Group_Out[(height - i - 1) * width + j]);
			}
		}
	}
	result = iCorner_Count;
	bRet = 1;
END:
	if (bRet == -1)
		return bRet;

	if (result <= 0)
	{
		iCorner_Count = Min(iCorner_Count, Grid_Size[0] * Grid_Size[1]);
		iCorner_Out_Count = iCorner_Count;
		for (int i = 0; i < iCorner_Count; i++)
			Corner_Group_Out[i] = Corner_Group[i];
		result = -iCorner_Count;

		if (result == -Grid_Size[0] * Grid_Size[1])
			result = -result;
	}

	return result;
}

static void Get_Bounding_Box(Chess_Board_Quad *Quad_Group[], int iSize,float Bounding_Box[2][2])
{
	if (!iSize)
	{
		memset(Bounding_Box, 0, 2 * 2 * sizeof(float));
		return;
	}

	//此处多手搞一个范例，寻找最大最小值的范例
	Bounding_Box[0][0] = Bounding_Box[0][1] = MAX_FLOAT;
	Bounding_Box[1][0] = Bounding_Box[1][1] = 0;
	for (int i = 0; i < iSize; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			float *pPos = Quad_Group[i]->Corner[j]->Pos;
			if (pPos[0] < Bounding_Box[0][0])
				Bounding_Box[0][0] = pPos[0];
			if(pPos[0]> Bounding_Box[1][0])
				Bounding_Box[1][0] = pPos[0];
			if (pPos[1] < Bounding_Box[0][1])
				Bounding_Box[0][1] = pPos[1];
			if (pPos[1] > Bounding_Box[1][1])
				Bounding_Box[1][1] = pPos[1];
		}
	}
	return;
}
int iProcess_Quad(Chess_Board_Quad Quad[], int* piQuad_Count, int iMax_Quad_Count,
	Chess_Board_Corner All_Corner[], int* piCorner_Count, int iMax_Corner_Count,
	unsigned char Grid_Size[2], float Out_Corner[][2], int iParent_Counter = -1,
	float Bounding_Box[2][2]=NULL)
{//寻找棋盘格
//返回值分三种情况，不够内存，返回-1
//找到，返回1，找不多返回0
	static int iCount = 0;
	int iQuad_Count = *piQuad_Count;
	if (!iQuad_Count)
		return 0;

	Find_Quad_Neighbour(Quad, iQuad_Count);

	int iGroup_Size, iRet = 0;
	const int iMax_Group_Size = Grid_Size[0] * Grid_Size[1];;
	Chess_Board_Quad** Quad_Group = (Chess_Board_Quad**)pMalloc(iMax_Group_Size * sizeof(Chess_Board_Quad*));
	Chess_Board_Corner** Corner_Group = (Chess_Board_Corner**)pMalloc(iMax_Group_Size * 4 * sizeof(Chess_Board_Corner*));
	if (!Quad_Group || !Corner_Group)
	{
		iRet = -1;
		goto END;
	}

	static int iCounter = 0;
	int iCorner_Count;
	iCorner_Count = *piCorner_Count;

	//以下从Quad里不断循环找出一组连通Group
	for (int iGroup_Index = 0;; iGroup_Index++)
	{
		iCounter++;
		/*if (iCounter == 101)
			printf("Here");*/

		iGroup_Size = 0;
		//前面已经做了所有四边形的近邻搜索，以下找出每一组连刀肉
		if (!bFind_Connected_Quads(Quad, iQuad_Count, Quad_Group, iGroup_Index, &iGroup_Size))
		{
			iRet = -1;
			goto END;
		}
		//Draw_Quad_Group("c:\\tmp\\1.bmp", Quad_Group, iGroup_Size);

		if (!iGroup_Size)
			break;

		int iCount = iOrder_Found_Connected_Quads(Quad, &iQuad_Count, iMax_Quad_Count,
			All_Corner, piCorner_Count,iMax_Corner_Count, Quad_Group, &iGroup_Size, Grid_Size, iCounter);
		if (!iCount)
			continue;
		//Draw_Quad_Group("c:\\tmp\\2.bmp", Quad_Group, iGroup_Size);;

		//此处还差一道工序，把多余的Quad去掉
		Clean_Found_Connected_Quads(Quad_Group, iGroup_Size, Grid_Size);

		iCount = Check_Quad_Group(Quad, iQuad_Count, Quad_Group, iGroup_Size, Corner_Group, Grid_Size);
		if (iCount == -1)
		{
			iRet = -1;
			goto END;
		}

		int n = iCount > 0 ? Grid_Size[0] * Grid_Size[1] : -iCount;
		float sum_dist = 0;
		int total = 0;
		/*if (iCounter == 3)
			printf("here");*/
		for (int i = 0; i < n; i++)
		{
			int ni = 0;
			float sum = fSum_Dist(*Corner_Group[i], &ni);
			sum_dist += sum;
			total += ni;
		}
		float prev_sqr_size = (float)round((double)sum_dist / Max(total, 1));
		for (int i = 0; i < n; i++)
			Out_Corner[i][0] = Corner_Group[i]->Pos[0], Out_Corner[i][1] = Corner_Group[i]->Pos[1];

		if (iCount == Grid_Size[0] * Grid_Size[1] && bCheck_Board_Monotony(Out_Corner, n, Grid_Size))
		{
			///Draw_Quad_Group("c:\\tmp\\1.bmp", Quad_Group, iGroup_Size);
			if(Bounding_Box)
				Get_Bounding_Box(Quad_Group, iGroup_Size, Bounding_Box);
			iRet = 1;
			break;
		}
		iCounter++;
	}
END:
	if (Quad_Group)
		Free(Quad_Group);
	if (Corner_Group)
		Free(Corner_Group);
	iCount++;
	return iRet;
}

void Adaptive_Threshold(Image oSource, Image* poDest, int iBox_Filter_r, int iDelta)
{//搞个阈值生成二值化图像
	if (iBox_Filter_r <= 127)
		Box_Filter_S(oSource, poDest, iBox_Filter_r);
	else
	{
		Init_Image_Dup(poDest, oSource);
		Box_Filter(oSource, *poDest, iBox_Filter_r);
	}
	if (!poDest->m_pBuffer)
		return;

	unsigned char tab[768];	//什么玩意？
	int i;
	const int iMax_Val = 255;
	//什么意义？为什么要搞这么麻烦？
	for (i = 0; i < 768; i++)
		tab[i] = (unsigned char)(i - 255 > -iDelta ? iMax_Val : 0);

	int iSize = oSource.m_iWidth * oSource.m_iHeight;
	for (int iChannel = 0; iChannel < oSource.m_iChannel_Count; iChannel++)
	{
		unsigned char* pSource = oSource.m_pChannel[iChannel],
			* pDest = poDest->m_pChannel[iChannel];
		for (i = 0; i < iSize; i++)
			pDest[i] = tab[pSource[i] - pDest[i] + 255];
	}
	//bSave_Image("c:\\tmp\\1.bmp", *poDest);
	return;
}

int bClose_To_Border(float Corner[][2], int iCount, Image oImage)
{
	const int BORDER = 8;
	for (int k = 0; k < iCount; ++k)
	{
		if (Corner[k][0] <= BORDER || Corner[k][0] > oImage.m_iWidth - BORDER ||
			Corner[k][1] <= BORDER || Corner[k][1] > oImage.m_iHeight - BORDER)
			return 1;
	}
	return 0;
}

void Adjust_Corner(float Corner[][2], int iCount, unsigned char Grid_Size[2])
{	//长与高都是偶数才需要调整
	if ((Grid_Size[0] & 1) == 0 && (Grid_Size[1] & 1) == 0)
	{
		int last_row = (Grid_Size[1] - 1) * Grid_Size[0];
		double dy0 = Corner[last_row][1] - Corner[0][1];
		if (dy0 < 0)
		{//显然，头向下尾朝天了，来个头尾交换
			int n = Grid_Size[0] * Grid_Size[1];
			for (int i = 0; i < n / 2; i++)
				std::swap(Corner[i], Corner[n - i - 1]);
		}
	}
}

void Get_Rect_Sub_Pix(Image oImage, int Patch_Size[2], float Center_0[2],
	float Patch[], int Patch_Type = 5)
{
	float Center[] = { Center_0[0],Center_0[1] };
	float* dst = Patch;
	int* win_size = Patch_Size, ip[2];

	Center[0] -= (win_size[0] - 1) * 0.5f;
	Center[1] -= (win_size[1] - 1) * 0.5f;

	ip[0] = (int)Center[0];
	ip[1] = (int)Center[1];

	float a = Center[0] - ip[0];
	float b = Center[1] - ip[1];
	a = Max(a, 0.0001f);
	float a12 = a * (1.f - b);
	float a22 = a * b;
	float b1 = 1.f - b;
	float b2 = b;
	double s = (1. - a) / a;
	unsigned char* src = oImage.m_pChannel[0];

	//src_step /= sizeof(src[0]);
	//dst_step /= sizeof(dst[0]);
	int src_step = oImage.m_iWidth, dst_step = Patch_Size[0];
	src += ip[1] * src_step + ip[0];

	for (; win_size[1]--; src += src_step, dst += dst_step)
	{
		float prev = (1 - a) * (b1 * src[0] + b2 * src[src_step]);
		for (int j = 0; j < win_size[0]; j++)
		{
			float t = a12 * src[j + 1] + a22 * src[j + 1 + src_step];
			dst[j] = prev + t;
			prev = (float)(t * s);
		}
	}

	return;
}

int Corner_SubPix(Image oImage, float Corner[][2], int iCorner_Count,
	int Win[2], int Zero_Zone[2], Term_Criteria criteria = {})
{
	int bRet = 0;
	const int MAX_ITERS = 100;
	int win_w = Win[0] * 2 + 1, win_h = Win[1] * 2 + 1;
	int i, j, k;
	int max_iters = 15;
	double eps = 0.1f;
	eps *= eps;

	float* mask, * maskm = (float*)pMalloc(win_w * win_h * sizeof(float));
	if (!maskm)
		goto END;

	mask = maskm;	//啥玩意

	for (i = 0; i < win_h; i++)
	{
		float y = (float)(i - Win[1]) / Win[1];
		float vy = (float)exp(-y * y);
		for (j = 0; j < win_w; j++)
		{
			float x = (float)(j - Win[0]) / Win[0];
			mask[i * win_w + j] = (float)(vy * exp(-x * x));
		}
	}
	if (Zero_Zone[0] >= 0 && Zero_Zone[1] >= 0 && Zero_Zone[0] * 2 + 1 < win_w && Zero_Zone[1] * 2 + 1 < win_h)
	{
		for (i = Win[1] - Zero_Zone[1]; i <= Win[1] + Zero_Zone[1]; i++)
			for (j = Win[0] - Zero_Zone[0]; j <= Win[0] + Zero_Zone[0]; j++)
				mask[i * win_w + j] = 0;
	}
	float Subpix_Buf[7 * 7];
	float cI2[2];
	cI2[0] = cI2[1] = 0;

	// do optimization loop for all the points
	for (int pt_i = 0; pt_i < iCorner_Count; pt_i++)
	{
		//float *cT = Corner[pt_i], *cI = cT;
		float cT[2] = { Corner[pt_i][0],Corner[pt_i][1] }, cI[2] = { cT[0],cT[1] };
		int iter = 0;
		double err = 0;
		do
		{
			double a = 0, b = 0, c = 0, bb1 = 0, bb2 = 0;
			int Patch_Size[] = { win_w + 2, win_h + 2 };
			Get_Rect_Sub_Pix(oImage, Patch_Size, cI, Subpix_Buf);
			//Disp(Subpix_Buf, 7, 7, "Patch");
			const float* subpix = &Subpix_Buf[1 * Patch_Size[0] + 1];
			for (i = 0, k = 0; i < win_h; i++, subpix += win_w + 2)
			{
				double py = i - Win[1];
				for (j = 0; j < win_w; j++, k++)
				{
					double m = mask[k];
					double tgx = subpix[j + 1] - subpix[j - 1];
					double tgy = subpix[j + win_w + 2] - subpix[j - win_w - 2];
					double gxx = tgx * tgx * m;
					double gxy = tgx * tgy * m;
					double gyy = tgy * tgy * m;
					double px = j - Win[0];

					a += gxx;
					b += gxy;
					c += gyy;

					bb1 += gxx * px + gxy * py;
					bb2 += gxy * px + gyy * py;
				}
			}

			double det = a * c - b * b;
			if (fabs(det) <= DBL_EPSILON * DBL_EPSILON)
				break;

			// 2x2 matrix inversion
			double scale = 1.0 / det;
			cI2[0] = (float)(cI[0] + c * scale * bb1 - b * scale * bb2);
			cI2[1] = (float)(cI[1] - b * scale * bb1 + a * scale * bb2);
			err = (cI2[0] - cI[0]) * (cI2[0] - cI[0]) + (cI2[1] - cI[1]) * (cI2[1] - cI[1]);
			// if new point is out of image, leave previous point as the result
			if (cI2[0]<0 || cI2[0]>oImage.m_iWidth - 1 || cI2[1]<0 || cI2[1]>oImage.m_iHeight - 1)
				break;
			//cI = cI2;
			cI[0] = cI2[0], cI[1] = cI2[1];
		} while (++iter < max_iters && err > eps);
		/*if (pt_i == 46)
			printf("here");*/
		if (fabs(cI[0] - cT[0]) > Win[0] || fabs(cI[1] - cT[1]) > Win[1])
		{
			//cI[0] = cI2[0], cI[1] = cI2[1];
			//cI = cT;
			cI[0] = cT[0], cI[1] = cT[1];
		}
		Corner[pt_i][0] = cI[0];
		Corner[pt_i][1] = cI[1];
	}
	if (maskm)
		Free(maskm);
	bRet = 1;
END:

	return bRet;
}
void Draw_Corner(Image oImage, float Corner[][2], int iCount,int r=3)
{
	for (int i = 0; i < iCount; i++)
		Draw_Arc(oImage, r, (int)Corner[i][0], (int)Corner[i][1]);
	return;
}
int bFind_Chess_Board_Corner(Image oImage, float Corner[][2], int iGrid_Size_w, int iGrid_Size_h, float Bounding_Box[][2])
{
	int bRet = 0;
	unsigned char Grid_Size[2] = { (unsigned char)iGrid_Size_w ,(unsigned char)iGrid_Size_h };

	//备份一下原图
	Image oImage_1, oImage_2 = {};	// oPre_Image = {}, ;
	//Init_Image(&oImage_1, oImage.m_iWidth, oImage.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Init_Image_Dup(&oImage_1, oImage);
	if (!oImage_1.m_pBuffer)
		goto END;
	Box_Filter(oImage, oImage,1);

	//先来个二值化
	Binerize(oImage, oImage_1);

	//做一次提出孤点
	//Remove_Orphan(oImage_1);
	Chess_Board_Quad* pQuad;
	int iMin_Dilations, iMax_Dilations;
	pQuad = NULL;
	iMin_Dilations = 0;
	iMax_Dilations = 7;
	//第一部分，简单判断，可以看出，只是简单膨胀，不做二值化的阀值调整，
	//可以想象，万一棋盘黑白分不出来，无论膨胀多少也没用
	int iQuad_Count, iDilation, iResult, iCur_Corner, iCorner_Count;
	iResult = 0, iCur_Corner = 0, iCorner_Count = Grid_Size[0] * Grid_Size[1];
	for (iDilation = iMin_Dilations; !iResult && iDilation <= iMax_Dilations; iDilation++)
	{//不断膨胀收缩，以便截断方块与方块之间的连通
		pQuad = NULL;
		if (iDilation > 0)
			Dilate_Bin(oImage_1);

		//加一条3像素的边
		Draw_Rect(oImage_1, 0, 0, oImage_1.m_iWidth, oImage_1.m_iHeight, 3);
				
		int iMax_Quad_Count, iQuad_Count=0;
		if (!bGenerate_Quad(oImage_1, &pQuad, &iQuad_Count, &iMax_Quad_Count, iDilation))
			continue;

		int iMax_Corner_Count = iMax_Quad_Count * 5;
		Chess_Board_Corner* pCorner = (Chess_Board_Corner*)pMalloc(iMax_Corner_Count * sizeof(Chess_Board_Corner));
		iCur_Corner = 0;
		Quad_i16_2_float(pQuad, iQuad_Count, pCorner, &iCur_Corner);

		if ((iResult = iProcess_Quad(pQuad, &iQuad_Count, iMax_Quad_Count, pCorner, &iCur_Corner,iMax_Corner_Count, Grid_Size, Corner, iDilation, Bounding_Box)) == -1)
			goto END;
		//bSave_Image("c:\\tmp\\1.bmp", oImage_1);
		Free(pCorner);
		Free(pQuad);
	}
	Free_Image(&oImage_1);

	Chess_Board_Corner* pCorner;
	pCorner = NULL;
	if (!iResult)
	{//前面尚未成功，还需努力
		int iMax_K = 6;				//自适应，就是改个二值化阀值
		int iPrev_Sqr_Size = 0,k, iDilation;
		iMax_Dilations = 7;
		for (k = 0; k < iMax_K && !iResult; k++)
		{
			int iPrev_Block_Size = -1;
			for (iDilation = iMin_Dilations; !iResult && iDilation <= iMax_Dilations; iDilation++)
			{
				int block_size = (int)round(iPrev_Sqr_Size == 0
					? min(oImage.m_iWidth, oImage.m_iHeight) * (k % 2 == 0 ? 0.2 : 0.1)
					: iPrev_Sqr_Size * 2);
				block_size = block_size | 1;
				if (block_size != iPrev_Block_Size)
				{
					//Box_Filter(oImage, oImage, 1);
					Adaptive_Threshold(oImage, &oImage_2, block_size >> 1, (k / 2) * 5);
					if (!oImage_2.m_pBuffer)
						goto END;
					//bSave_Image("c:\\tmp\\2.bmp", oImage_2);
					if (iDilation)
						Dilate_Bin(oImage_2, iDilation);
					//memcpy(oPre_Image.m_pBuffer, oImage_2.m_pBuffer, oImage_2.m_iWidth * oImage_2.m_iHeight);
				}
				else if (iDilation > 0)
				{
					Dilate_Bin(oImage_2, 1);
					//以下代码不足以不断消除噪声
					//Dilate_Bin(oPre_Image, 1);
					//Init_Image_Dup(&oImage_2, oImage);
					//memcpy(oImage_2.m_pBuffer, oPre_Image.m_pBuffer, oImage_2.m_iWidth * oImage_2.m_iHeight);
				}
				
				iPrev_Block_Size = block_size;
				Draw_Rect(oImage_2, 0, 0, oImage_2.m_iWidth, oImage_2.m_iHeight, 3);
				int iMax_Quad_Count;
				iResult = bGenerate_Quad(oImage_2, &pQuad, &iQuad_Count, &iMax_Quad_Count, iDilation);
				if (!iResult)
					continue;

				int iMax_Corner_Count = iMax_Quad_Count * 4;
				pCorner = (Chess_Board_Corner*)pMalloc(iMax_Corner_Count * sizeof(Chess_Board_Corner));
				if (!pCorner)
					goto END;

				iCur_Corner = 0;
				Quad_i16_2_float(pQuad, iQuad_Count, pCorner, &iCur_Corner);
				if ((iResult = iProcess_Quad(pQuad, &iQuad_Count, iMax_Quad_Count, pCorner, &iCur_Corner,iMax_Corner_Count, Grid_Size, Corner, iDilation,Bounding_Box)) == -1)
					goto END;
				//printf("k:%d Dilate:%d Result:%d\n", k, iDilation, iResult);
				Free(pCorner), pCorner = NULL;
				Free(pQuad), pQuad = NULL;
			}
			Free_Image(&oImage_2);
		}
		//Free_Image(&oPre_Image);
	}

	//判断角点是否在图像边缘
	if (iResult)
		iResult = !bClose_To_Border(Corner, iCorner_Count, oImage);

	if (iResult)
	{
		Adjust_Corner(Corner, iCorner_Count, Grid_Size);
		int Win[] = { 2,2, }, Zero_Zone[] = { -1,-1 };
		if (!Corner_SubPix(oImage, Corner, iCorner_Count, Win, Zero_Zone))
			goto END;
		bRet = 1;
	}

	////Disp((float*)Corner, iCorner_Count, 2,"Corner");
	//for (int i = 1; i <= iCorner_Count; i++)
	//{
	//	char File[256];
	//	sprintf(File,"c:\\tmp\\temp\\%02d.bmp", i);
	//	Draw_Corner(oImage, Corner, i, 10);
	//	bSave_Image(File, oImage);
	//}
	
END:
	Free_Image(&oImage_1);
	Free_Image(&oImage_2);
	//Free_Image(&oPre_Image);
	return bRet;
}

template<typename _T> void Get_Size(_T Bouding_Box[2][2], _T* pfWidth, _T *pfHeight)
{
	*pfWidth = Bouding_Box[1][0] - Bouding_Box[0][0];
	*pfHeight = Bouding_Box[1][1] - Bouding_Box[0][1];
	return;
}

int bFind_Chess_Board_2_Step(Image oImage, float Corner[][2], float fScale, int iGrid_Size_w, int iGrid_Size_h)
{//留给自以为是的人，能算出第一阶段的小图边长scale
	Image oDest = {};
	//float fScale = 640.f / oImage.m_iWidth;
	int bRet = 0, iCorner_Count = iGrid_Size_w * iGrid_Size_h;

	Init_Image(&oDest, (int)ceil(fScale * oImage.m_iWidth),
		(int)ceil(fScale * oImage.m_iHeight), Image::IMAGE_TYPE_BMP, 8);
	Bi_Linear_cv(oImage, oDest, fScale, fScale);

	float Bounding_Box[2][2], fWidth, fHeight;
	int iResult;
	iResult = bFind_Chess_Board_Corner(oDest, Corner, iGrid_Size_w, iGrid_Size_h, Bounding_Box);
	if (!iResult)
		goto END;

	Bounding_Box[0][0] -= 20;
	Bounding_Box[1][0] += 20;
	Bounding_Box[0][1] -= 20;
	Bounding_Box[1][1] += 20;
	//将Boudng Box 再放大回原图
	Vector_Multiply((float*)Bounding_Box, 2 * 2, 1.f / fScale, (float*)Bounding_Box);

	Free_Image(&oDest);
	Get_Size(Bounding_Box, &fWidth, &fHeight);
	Init_Image(&oDest, (int)ceil(fWidth), (int)ceil(fHeight), Image::IMAGE_TYPE_BMP, oImage.m_iBit_Count);
	Crop_Image(oImage, (int)Bounding_Box[0][0], (int)Bounding_Box[0][1],
		oDest.m_iWidth, oDest.m_iHeight, oDest);
	
	iResult = bFind_Chess_Board_Corner(oDest, Corner, iGrid_Size_w, iGrid_Size_h);
	if (!iResult)
	{
		printf("Step 2 fail\n");
		goto END;
	}
	bRet = 1;
	
END:
	Free_Image(&oDest);
	return bRet;
}

int bFind_Chess_Board_2_Step(char File[], float Corner[][2], int iStep_1_Image_Size, int iGrid_Size_w, int iGrid_Size_h)
{//对图两次读入，用时间换空间，避免大图的站内用
	int bRet = 0, iCorner_Count = iGrid_Size_w * iGrid_Size_h;
	Image oImage,oDest;
	if (!bLoad_Image(File, &oImage))
		goto END;

	float fScale, Bounding_Box[2][2], fWidth, fHeight;
	fScale = (iStep_1_Image_Size == -1 ? 1 : (float)sqrt((float)iStep_1_Image_Size / (oImage.m_iWidth * oImage.m_iHeight)));
	int iResult;
	if (fScale >= 1)
	{//图太小，一次搞定
		iResult = bFind_Chess_Board_Corner(oImage, Corner, iGrid_Size_w, iGrid_Size_h, Bounding_Box);
		if (iResult)
			bRet = 1;
		goto END;
	}

	//Pass 1
	Init_Image(&oDest, (int)ceil(fScale * oImage.m_iWidth),
		(int)ceil(fScale * oImage.m_iHeight), Image::IMAGE_TYPE_BMP, 8);
	if (!oDest.m_pBuffer)
		goto END;
	Bi_Linear_cv(oImage, oDest, fScale, fScale);
	Free_Image(&oImage);
	if(!(iResult = bFind_Chess_Board_Corner(oDest, Corner, iGrid_Size_w, iGrid_Size_h, Bounding_Box)))
		goto END;
	Free_Image(&oDest);	//第一时间释放内存

	//Pass 2
	//留有余地
	Bounding_Box[0][0] -= 20;
	Bounding_Box[1][0] += 20;
	Bounding_Box[0][1] -= 20;
	Bounding_Box[1][1] += 20;

	//将Boudng Box 再放大回原图
	Vector_Multiply((float*)Bounding_Box, 2 * 2, 1.f / fScale, (float*)Bounding_Box);
	Bounding_Box[0][0] = (float)(int)(Bounding_Box[0][0]);
	Bounding_Box[0][1] = (float)(int)(Bounding_Box[0][1]);
	Bounding_Box[1][0] = (float)(int)(Bounding_Box[1][0]);
	Bounding_Box[1][1] = (float)(int)(Bounding_Box[1][1]);

	Get_Size(Bounding_Box, &fWidth, &fHeight);
	Init_Image(&oDest, (int)ceil(fWidth), (int)ceil(fHeight), Image::IMAGE_TYPE_BMP, 8);
	if (!bLoad_Image(File, &oImage))
		goto END;

	Crop_Image(oImage, (int)Bounding_Box[0][0], (int)Bounding_Box[0][1],
		oDest.m_iWidth, oDest.m_iHeight, oDest);
	Free_Image(&oImage);
	if(!(iResult = bFind_Chess_Board_Corner(oDest, Corner, iGrid_Size_w, iGrid_Size_h)))
	{
		printf("Step 2 fail\n");
		goto END;
	}

	//恢复坐标
	for (int i = 0; i < iCorner_Count; i++)
		Corner[i][0] += Bounding_Box[0][0], Corner[i][1] += Bounding_Box[0][1];
	bRet = 1;

END:
	Free_Image(&oImage);
	Free_Image(&oDest);
	return bRet;
}
int bFind_Chess_Board_2_Step(Image oImage, float Corner[][2], int iStep_1_Image_Size, int iGrid_Size_w, int iGrid_Size_h)
{//给懒人用，iStep_1_Image_Size：自己拍脑袋一个小图大小，剩下的自动算Scale
	float fScale = (iStep_1_Image_Size == -1 ? 1 : (float)sqrt((float)iStep_1_Image_Size / (oImage.m_iWidth * oImage.m_iHeight)));
	Image oDest = {};
	//float fScale = 640.f / oImage.m_iWidth;
	int bRet = 0, iCorner_Count = iGrid_Size_w * iGrid_Size_h;

	float Bounding_Box[2][2], fWidth, fHeight;
	int iResult;
	if (fScale >= 1)
	{//图太小，一次搞定
		iResult = bFind_Chess_Board_Corner(oImage, Corner, iGrid_Size_w, iGrid_Size_h, Bounding_Box);
		if (iResult)
			bRet = 1;
		goto END;
	}

	Init_Image(&oDest, (int)ceil(fScale * oImage.m_iWidth),
		(int)ceil(fScale * oImage.m_iHeight), Image::IMAGE_TYPE_BMP, 8);
	Bi_Linear_cv(oImage, oDest, fScale, fScale);
	iResult = bFind_Chess_Board_Corner(oDest, Corner, iGrid_Size_w, iGrid_Size_h, Bounding_Box);
	if (!iResult)
		goto END;

	//留有余地
	Bounding_Box[0][0] -= 20;
	Bounding_Box[1][0] += 20;
	Bounding_Box[0][1] -= 20;
	Bounding_Box[1][1] += 20;
	
	//将Boudng Box 再放大回原图
	Vector_Multiply((float*)Bounding_Box, 2 * 2, 1.f / fScale, (float*)Bounding_Box);
	Bounding_Box[0][0] = (float)(int)(Bounding_Box[0][0]);
	Bounding_Box[0][1] = (float)(int)(Bounding_Box[0][1]);
	Bounding_Box[1][0] = (float)(int)(Bounding_Box[1][0]);
	Bounding_Box[1][1] = (float)(int)(Bounding_Box[1][1]);

	Free_Image(&oDest);
	Get_Size(Bounding_Box, &fWidth, &fHeight);
	Init_Image(&oDest, (int)ceil(fWidth), (int)ceil(fHeight), Image::IMAGE_TYPE_BMP, oImage.m_iBit_Count);
	Crop_Image(oImage, (int)Bounding_Box[0][0], (int)Bounding_Box[0][1],
		oDest.m_iWidth, oDest.m_iHeight, oDest);

	iResult = bFind_Chess_Board_Corner(oDest, Corner, iGrid_Size_w, iGrid_Size_h);
	if (!iResult)
	{
		printf("Step 2 fail\n");
		goto END;
	}

	//恢复坐标
	for (int i = 0; i < iCorner_Count; i++)
		Corner[i][0] += Bounding_Box[0][0], Corner[i][1] += Bounding_Box[0][1];
	
	bRet = 1;
END:
	Free_Image(&oDest);
	return bRet;
}
//**************************第二部分，棋盘检测****************************************************/

//***********************张正友标定***********************************/
template void Load_Poine_2D(const char* pcFile, int* piImage_Count, int iCorner_Per_Image, int w_In_Point, int h_In_Point, float(**ppCorner_Point_2D)[2]);
template void Load_Poine_2D(const char* pcFile, int* piImage_Count, int iCorner_Per_Image, int w_In_Point, int h_In_Point, double(**ppCorner_Point_2D)[2]);
template<typename _T>void Load_Poine_2D(const char* pcFile, int* piImage_Count, int iCorner_Per_Image, int w_In_Point, int h_In_Point, _T(**ppCorner_Point_2D)[2])
{//装入角点数据，临时代码而已
	int iSize;
	float* pConer_Point_2D;
	int iImage_Count;
	bLoad_Raw_Data(pcFile, (unsigned char**)&pConer_Point_2D, &iSize);
	iImage_Count = iSize / (iCorner_Per_Image * 2 * sizeof(float));
	if (iSize != iImage_Count * iCorner_Per_Image * 2 * sizeof(float))
	{
		printf("Invalid size:%d\n", iSize);
		return;
	}
	if (*piImage_Count)
		iImage_Count = *piImage_Count;

	_T(*pCur_Corner)[2], (*pConer_Point_2D_1)[2] = (_T(*)[2])pMalloc(iImage_Count * iCorner_Per_Image * 2 * sizeof(_T));
	for (int i = 0; i < iImage_Count; i++)
	{
		pCur_Corner = &pConer_Point_2D_1[i * iCorner_Per_Image];
		for (int j = 0; j < iCorner_Per_Image; j++)
		{
			/*int x = j / h_In_Point,
				y = j % h_In_Point;
			pCur_Corner[y * w_In_Point + x][0] = ((float(*)[2])pConer_Point_2D)[i * iCorner_Per_Image + j][0],
				pCur_Corner[y * w_In_Point + x][1] = ((float(*)[2])pConer_Point_2D)[i * iCorner_Per_Image + j][1];*/
			pCur_Corner[j][0] = ((float(*)[2])pConer_Point_2D)[i * iCorner_Per_Image +j][0],
				pCur_Corner[j][1] = ((float(*)[2])pConer_Point_2D)[i * iCorner_Per_Image+j][1];

		}
		//pConer_Point_2D_1[i] = pConer_Point_2D[i];
	}
	*ppCorner_Point_2D = (_T(*)[2])pConer_Point_2D_1;
	*piImage_Count = iImage_Count;
	Free(pConer_Point_2D);
	return;
}
//***********************张正友标定***********************************/

void Find_Chess_Board_Corner_Test_1()
{
	//一般A4纸打印2厘米变成格子，为12x9,沿用opencv管理，为11x9
	const int Grid_Size_h = 8, Grid_Size_w = 11;
	char File[256] = "c:\\tmp\\Sample_3.bmp";
	Image oImage;
	if (!bLoad_Image(File, &oImage))
		return;

	float Corner[Grid_Size_w * Grid_Size_h][2];
	int iResult = bFind_Chess_Board_Corner(oImage, Corner);
	//printf("Result:%d\n", iResult);

	Free_Image(&oImage);
	return;
}

static void Find_Max_Contour_Test()
{//画出最大连通域
	Image oImage, oBin;
	if (!bLoad_Image("c:\\tmp\\Sample_2.bmp", &oImage))
		return;
	Init_Image_Dup(&oBin, oImage);

	//二值化
	Binerize(oImage, oBin);

	Get_Contour_Info oInfo;
	if (!bGet_Contour(oBin, &oInfo))
		goto END;

END:
	Free_Image(&oImage);
	Free_Image(&oBin);
	Free_Contour_Info(&oInfo);
	return;
}

static void Find_Outline_Test()
{//找出所有连通域的轮廓
	Image oImage, oBin;
	if (!bLoad_Image("c:\\tmp\\Sample_3.bmp", &oImage))
		return;
	Init_Image_Dup(&oBin, oImage);

	//二值化
	Binerize(oImage, oBin);

	Get_Contour_Result oResult;
	if (!bGet_Contour(oBin, &oResult))
		return;
	
	Free_Contour_Result(&oResult);
	Free_Image(&oImage);
	Free_Image(&oBin);
	return;
}

void Find_Chess_Board_Corner_Test_4()
{//最后的挣扎，将棋盘检测改造为从文件读入，希望能减少大图的占用空间
	const int Grid_Size_h = 8, Grid_Size_w = 11;
	const  int iStep_1_Size = 1000000;
	float Corner[Grid_Size_w * Grid_Size_h][2];

	bFind_Chess_Board_2_Step((char*)"C:\\tmp\\dev_env\\sample\\06.bmp", Corner, iStep_1_Size, Grid_Size_w, Grid_Size_h);

	return;
}
void Find_Chess_Board_Corner_Test_3()
{//缩放一下看看如何
	const int Grid_Size_h = 8, Grid_Size_w = 11;
	float Corner[Grid_Size_w * Grid_Size_h][2];

	Image oImage;
	if (!bLoad_Image("C:\\tmp\\dev_env\\sample\\06.bmp", &oImage))
		return;

	if (oImage.m_iChannel_Count== 3 && !RGB_2_Gray_1(oImage, oImage))
		goto END;

	bRe_Init_Image(&oImage, oImage.m_iWidth, oImage.m_iHeight, (Image::Type)oImage.m_iImage_Type, 8);

	//第一步采样的最大图像面积
	int iStep_1_Size,
		iOrg_Size,
		iResult;
	float fScale;
	iStep_1_Size = 1000000;
	iOrg_Size = oImage.m_iWidth * oImage.m_iHeight;
	fScale = (float)sqrt((float)iStep_1_Size / iOrg_Size);
	
	unsigned long long tStart;
	tStart = iGet_Tick_Count();
	if ( !(iResult = bFind_Chess_Board_2_Step(oImage, Corner,fScale)))
		goto END;

END:
	printf("Result:%d %lld\n", iResult, iGet_Tick_Count() - tStart);
	Free_Image(&oImage);
	return;
}

void Find_Chess_Board_Corner_Test_2()
{
	const int Grid_Size_h = 8, Grid_Size_w = 11;
	const int iCorner_Count = Grid_Size_w * Grid_Size_h;
	float Corner[iCorner_Count][2];
	char File[256];
	Image oImage;

	int i, /*iStep_1_Size = 800*600,*/
		iFound_Count = 0;
	for (i = 1; i <= 12; i++)
	{
		//sprintf(File, "C:/tmp/Dev_Env/mono_img/%d.bmp", i);
		sprintf(File, "C:/tmp/Dev_Env/Sample/%02d.bmp", i);
		if (!bLoad_Image(File, &oImage))
			return;
		int iResult = bFind_Chess_Board_2_Step(oImage, Corner);
		printf("%s Result:%d\n", File, iResult);
		Free_Image(&oImage);
		iFound_Count += iResult;
	}

	printf("%d Found\n", iFound_Count);
	Free_Image(&oImage);
	return;
}

template void Gen_Corner_Ref(int w_In_Point, int h_In_Point, float fGrid_Size, float pCorner_3D[][2]);
template void Gen_Corner_Ref(int w_In_Point, int h_In_Point, double fGrid_Size, double pCorner_3D[][2]);
template<typename _T>void Gen_Corner_Ref(int w_In_Point, int h_In_Point, _T fGrid_Size, _T pCorner_3D[][2])
{//造棋盘的理论角点，世界坐标在此建立，故此z=0
	int x, y;
	//以下做一张棋盘的空间点位置数据，虽然数组为2维，但是z恒为0，故此
	//这实际上是三维坐标。即U,V,Z
	for (y = 0; y < h_In_Point; y++)
		for (x = 0; x < w_In_Point; x++)
			pCorner_3D[y * w_In_Point + x][0] = x * fGrid_Size,
			pCorner_3D[y * w_In_Point + x][1] = y * fGrid_Size;
	return;
}

template<typename _T>void Gen_Corner_Norm_Ref(int w_In_Point, int h_In_Point, _T fGrid_Size,
	_T Corner_Norm[][2],_T K[4],int bNormalize=1,Normalize_Method iMethod = Dev)
{//不但生成棋盘角点，而且归一化
	Gen_Corner_Ref(w_In_Point, h_In_Point, fGrid_Size, Corner_Norm);
	//Disp((_T*)Corner_Norm, w_In_Point * h_In_Point, 2, "Corner");
	if(bNormalize)
	{
		int n = w_In_Point * h_In_Point;
		Normalize_2d<_T>(Corner_Norm, n, Corner_Norm, iMethod, K);
	}
	return;
}

template<typename _T>void Get_vij(_T H[3 * 3], int i, int j, _T vij[6])
{
	//H1i*H1j
	vij[0] = H[i] * H[j];
	//H1i*H2j + H2i*H1j
	vij[1] = H[i] * H[1 * 3 + j] + H[1 * 3 + i] * H[j];
	//H2i*H2j
	vij[2] = H[1 * 3 + i] * H[1 * 3 + j];
	//H1i*H3j + H3i*H1j
	vij[3] = H[i] * H[2 * 3 + j] + H[2 * 3 + i] * H[j];
	//H2i*H3j + H3i*H2j +  h31 * h32
	vij[4] = H[1 * 3 + i] * H[2 * 3 + j] + H[2 * 3 + i] * H[1 * 3 + j] + H[2 * 3 + i] * H[2 * 3 + j];
	//H3i*H3j
	vij[5] = H[2 * 3 + i] * H[2 * 3 + j];
}

template<typename _T>int bSolve_B(_T H[][3 * 3], int iImage_Count, _T B[9])
{//通过H 借出B[9];还是解一个不定方程
	//本质是求解一个最小二乘问题组成的不定方程 Ax = 0
	//故此何以转换为 求 A'A的最小特征值对应的特征向量，用反幂法
	_T B1[6], * pCur, * A = (_T*)pMalloc(iImage_Count * 6 * 2 * sizeof(_T));
	int i,iResult;

	//K 是由B矩阵推出，一个B为6维向量，一张图能构造两条式子
	// 故此此处揭示了K的生成条件，起码要3张图才能构造出一个K
	//每张图片的H矩阵能构成两条式子，重要的是搞对这两条式子
	for (i = 0; i < iImage_Count; i++)
	{
		pCur = &A[i * 6 * 2];
		Get_vij(H[i], 0, 1, pCur);	//v12
		pCur += 6;
		_T v22[6];
		Get_vij(H[i], 0, 0, pCur);
		Get_vij(H[i], 1, 1, v22);
		Vector_Minus(pCur, v22, 6, pCur);
	}
	
	Solve_Linear_Contradictory<_T>(A, iImage_Count * 2, 6, NULL, B1, &iResult);
	if (iResult)
	{
		//再装配成对称矩阵
		B[0] = B1[0];
		B[1] = B[3] = B1[1];
		B[4] = B1[2];
		B[6] = B[2] = B1[3];
		B[7] = B[5] = B1[4];
		B[8] = B1[5];
	}	

	Free(A);
	return iResult;
}

template<typename _T>int Cal_K(_T H[][3 * 3], int iImage_Count, _T K[3 * 3])
{
	_T B[9], lamda;
	int iResult;
	//Solve_B_SVD(H, iImage_Count, B);
	iResult = bSolve_B(H, iImage_Count, B);
	if (!iResult)
		return 0;
		
	{//此处留白，以后有正常点的数据再LLt分解
		//Decompose_B
	}

	{//张正友直接符号法，理论伤更快
		//Disp(B, 3, 3, "B");
		//cy = v0 =(b12*b13 - b11*b23)/(b11*b22-b12*b12)
		_T b12b13_Minus_b11b23 = B[1] * B[2] - B[0] * B[5];
		_T b11b22_Minus_b12b12 = B[0] * B[4] - B[1] * B[1];
		K[5] = b12b13_Minus_b11b23 / b11b22_Minus_b12b12;

		//lamda = b33 - [b13*b13 + v0*(b12*b13 - b11*b23)]/b11
		lamda = B[8] - (B[2] * B[2] + K[5] * b12b13_Minus_b11b23) / B[0];

		//a = sqrt(lamda/b11)
		K[0] = sqrt(lamda / B[0]);

		//b= sqrt[lamda*b11/(b11*b22 - b12*b12)]
		K[4] = sqrt(lamda * B[0] / b11b22_Minus_b12b12);

		//gama = -b12*aab/lamda
		K[1] = -B[1] * K[0] * K[0] * K[4] / lamda;

		//u0 = gama*v0/b - b13*aa/lamda
		K[2] = K[1] * K[5] / K[4] - B[2] * K[0] * K[0] / lamda;

		K[3] = K[6] = K[7] = 0;
		K[8] = 1;
	}

	return 1;
}

template<typename _T>void Get_K_Inv_With_gama(_T K[], _T K_Inv[])
{//对内参快速求逆，其实没啥用，gama 捣乱，没有普遍性
	_T fx = K[0], fy = K[4],
		cx = K[2], cy = K[5],
		gama = K[1];
	K_Inv[0] = 1.f / fx;
	K_Inv[4] = 1.f / fy;
	K_Inv[1] = -gama / (fx * fy);
	K_Inv[2] = gama * cy / (fx * fy) - cx / fx;
	K_Inv[5] = -cy / fy;
	K_Inv[3] = K_Inv[6] = K_Inv[7] = 0;
	K_Inv[8] = 1;
	return;
}

template<typename _T>static void Cal_T(_T H[][3 * 3], _T K[3 * 3], int n, _T T[][4 * 4])
{//已经求出K,H, 再求出T
	_T K_Inv[3 * 3];
	Get_K_Inv_With_gama(K, K_Inv);

	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);

	for (int i = 0; i < n; i++)
	{
		_T* pH = H[i];
		_T R1R2t[3 * 3];
		Matrix_Multiply_3x3(K_Inv, pH, R1R2t);
		_T R1[3] = { R1R2t[0],R1R2t[3],R1R2t[6] },
			R2[3] = { R1R2t[1],R1R2t[4],R1R2t[7] },
			t[] = { R1R2t[2],R1R2t[5],R1R2t[8] },R3[3];

		//******以下这部分代码原版没有，单推导过程却是这么回事，待验算*****/
		//虽然对旋转矩阵R 的影响暂时没有，但是对t有一点点,对数据时要注释掉
		//然而神奇的是，着部分矫正对总体误差有着匪夷所思的提高
		_T s = 1.f/((fGet_Mod(R1, 3) + fGet_Mod(R2, 3)) / 2);
		Vector_Multiply(R1, 3, s, R1);
		Vector_Multiply(R2, 3, s, R2);
		Vector_Multiply(t, 3, s, t);	//关键是这里，对位移的修正
		//******以下这部分代码原版没有，单推导过程却是这么回事，待验算*****/

		Cross_Product(R1, R2, R3);
		_T R[3 * 3] = { R1[0],R2[0],R3[0],
			R1[1],R2[1],R3[1],
			R1[2],R2[2],R3[2] };
		//Disp(R, 3, 3, "R");

		int iResult;
		svd_3(R, oSVD, &iResult);
		Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T*)oSVD.Vt, 3, R);
		Gen_Pose_By_R_t(R, t, T[i]);

		////此处应该有一道工序，所有点是否都满足z>0,否则位姿无效
		//if (t[2] < 0)
		//	Vector_Multiply<_T>(T[i], 12, -1, T[i]);
		//Disp(T[i], 4, 4, "T");
	}

	Free_SVD(&oSVD);
	return;
}

template<typename _T>void Estimate_Distort_Coeff_5(_T K[3 * 3], _T T[][4 * 4], _T P[][2], _T uv[][2], int iCorner_Per_Image, int iImage_Count, _T Distort[5])
{//搞个满血版的畸变参数估计
	int iSize = ALIGN_SIZE_8(iCorner_Per_Image * 2 * 5 * iImage_Count * sizeof(_T)) +
		ALIGN_SIZE_8(iCorner_Per_Image * iImage_Count * sizeof(_T));

	_T* A = (_T*)pMalloc(iSize),
	* b = (_T*)(((unsigned char*)A) + ALIGN_SIZE_8(iCorner_Per_Image * 2 * 5 * iImage_Count * sizeof(_T)));
	//Disp((_T*)uv, iCorner_Per_Image, 2, "uv");

	for (int i = 0; i < iImage_Count; i++)
	{
		_T* T1 = T[i], *uv_Cur = uv[i * iCorner_Per_Image];
		_T* pA_Cur = &A[iCorner_Per_Image * 5 * 2 * i];
		_T *pP_Cur = (_T*)P;
		_T* b_Cur = &b[i * iCorner_Per_Image*2];
		//Disp(K, 3, 3, "K");
		//Disp(T1, 4, 4, "T");
		for (int j = 0; j < iCorner_Per_Image; j++)
		{//每点产生两条式子
			//将空间点投影到归一化平面
			_T P1[4] = { pP_Cur[0],pP_Cur[1],0,1 }, P2[3];

			Matrix_Multiply(T1, 3, 4, P1, 1, P2);
			_T x = P2[0] / P2[2], y = P2[1] / P2[2];

			//列方程
			_T r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
			pA_Cur[0] = x * r2;
			pA_Cur[1] = x * r4;
			pA_Cur[2] = x * r6;
			pA_Cur[3] = pA_Cur[3 + 6] = 2 * x * y;
			pA_Cur[4] = r2 + 2 * x * x;
			pA_Cur += 5;

			pA_Cur[0] = y * r2;
			pA_Cur[1] = y * r4;
			pA_Cur[2] = y * r6;
			pA_Cur[3] = r2 + 2 * y * y;
			
			//将uv点反向投影到归一化平面上
			_T uv_1[2] = { (uv_Cur[0] - K[2]) / K[0], (uv_Cur[1] - K[5]) / K[4] };
			b_Cur[0] = uv_1[0] - x, b_Cur[1] = uv_1[1] - y;

			pA_Cur += 5;
			pP_Cur += 2;
			uv_Cur += 2;
			b_Cur += 2;
		}
	}
	//Disp(A, iCorner_Per_Image * 2, 5, "A");
	int iResult;
	//注意，ax = b 是一个病态方程，其中国AtA 的条件数非常恐怖
	//为了给AtA 治病，必须让它对角线加上一个eps
	//注意，此处要用 岭回归 调整病态方程
	Solve_Linear_Contradictory<_T>(A, iCorner_Per_Image * iImage_Count * 2, 5, b, Distort, &iResult);
	Free(A);
	return;
}
void float_2_double(float* pSource, int n, double* pDest = NULL)
{//
	if (!pDest)
		pDest = (double*)pSource;

	for (int i = n - 1; i >= 0; i--)
		pDest[i] = pSource[i];
	return;
}
template<typename _T>_T Zhang_Get_Error(_T K[4], _T Distort[5], _T T[][3 * 4], _T P[][2],Point_2D<_T> uv[], int iCount)
{//算个误差和
	_T K9[3 * 3];
	K4_2_K9(K, K9);
	_T fError = 0;
	for (int i = 0; i < iCount; i++)
	{
		Point_2D<_T>oUV = uv[i];
		_T P1[4] = {P[oUV.m_iPoint_Index][0],P[oUV.m_iPoint_Index][1],0,1};
		_T uv1[3];
		//if (i == 1)
			//printf("Here");
		Get_uv_Ref<_T>(P1, T[oUV.m_iCamera_Index], K9, Distort, uv1);
		/*Disp(K9, 3, 3, "K");
		Disp(Distort, 1, 5, "D");
		Disp(T[oUV.m_iCamera_Index], 3, 4, "T");*/
		//Disp(uv1, 1, 2, "uv");
		_T E[2] = { oUV.m_Pos[0] - uv1[0],oUV.m_Pos[1] - uv1[1] };
		//printf("uv:%f %f uv':%f %f E:%f %f\n", uv1[0], uv1[1], oUV.m_Pos[0], oUV.m_Pos[1], E[0], E[1]);
		fError += E[0] * E[0] + E[1] * E[1];
	}
	return fError;
}
template<typename _T>_T Zhang_Get_Error(_T K[3 * 3], _T Distort[5], _T T[][4 * 4], _T Corner_3D[][2], _T Corner_2D[][2], int iImage_Count, int iCorner_Per_Image)
{//算个误差和
	int i, j, iPos;
	_T fError = 0, fTotal = 0;
	//k1 = k2 = 0;
	for (i = 0; i < iImage_Count; i++)
	{
		for (j = 0; j < iCorner_Per_Image; j++)
		{
			iPos = i * iCorner_Per_Image + j;
			_T Point_3D[4] = { Corner_3D[j][0], Corner_3D[j][1],0,1 };

			_T uv[2];
			Get_uv_Ref<_T>(Point_3D, T[i], K, Distort, uv);

			/*Disp(Point_3D, 1, 3, "P");
			Disp(T[i], 4, 4, "T");
			Disp(Corner_2D[iPos], 1, 2, "uv");*/
			_T E[2];
			Get_PnP_Deriv<_T>(Point_3D,Corner_2D[iPos],  T[i], K, Distort,NULL,NULL,NULL,NULL,E);

			//fError = sqrt((uv[0] - Corner_2D[iPos][0]) * (uv[0] - Corner_2D[iPos][0]) + (uv[1] - Corner_2D[iPos][1]) * (uv[1] - Corner_2D[iPos][1]));
			fError = E[0] * E[0] + E[1] * E[1];
			fTotal += fError;
		}
		//printf("Total:%f\n", fTotal);
	}
	return fTotal;	// (iImage_Count * iCorner_Per_Image);
}

template<typename _T>void Test_T(_T K[3 * 3], _T T[4 * 4], _T P[][2], _T uv[][2], int iCount)
{
	Point_Cloud<_T> oPC;
	Init_Point_Cloud(&oPC, 10000, 1);
	Image oImage;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 24);
	Vector_Multiply<_T>(T, 12, -1, T);

	//Disp(T, 4, 4, "T");
	Draw_Camera<_T>(&oPC, T);
	for (int i = 0; i < iCount; i++)
	{
		_T Point_3d[] = { P[i][0], P[i][1],0,1 };
		_T uv_1[3];	// = { uv[i][0],uv[i][1],1 };

		Matrix_Multiply(T, 4, 4, Point_3d, 1, Point_3d);
		/*printf("Orf:(%f, %f) Cur:(%f, %f, %f, %f)\n", P[i][0], P[i][1],
			Point_3d[0], Point_3d[1], Point_3d[2], Point_3d[3]);*/

		Draw_Point<_T>(&oPC, P[i][0], P[i][1], 0, 255, 0, 0);
		Matrix_Multiply_3x1(K, Point_3d, uv_1);
		uv_1[0] /= uv_1[2], uv_1[1] /= uv_1[2];
		//Disp(uv_1, 1, 2, "uv");
		//Draw_Arc(oImage, 3, uv_1[0], uv_1[1], 0, 2 * PI, 255);
	}

	//bSave_PLY("c:\\tmp\\1.ply", oPC);
	//bSave_Image("c:\\tmp\\1.bmp", oImage);
	return;
}

template<typename _T>void Save_Param(_T K[3 * 3], _T Distort[5], _T T[][16], _T Corner_Ref[2], _T uv[][2], int n)
{
	char File[256];
	sprintf(File, "c:\\tmp\\temp\\K_Distort_T.bin");
	FILE* pFile = fopen(File, "wb");
	int iResult = (int)fwrite(K, 1, 9 * sizeof(_T), pFile);
	iResult = (int)fwrite(Distort, 1, 5 * sizeof(_T), pFile);
	iResult = (int)fwrite(T, 1, n * 16 * sizeof(_T),pFile);
	fclose(pFile);

	sprintf(File, "c:\\tmp\\temp\\point.bin");
	pFile = fopen(File, "wb");
	iResult = (int)fwrite(Corner_Ref, 1, 88 * 2 * sizeof(_T),pFile);
	iResult = (int)fwrite(uv, 1, n * 88 * 2 * sizeof(_T),pFile);
	fclose(pFile);
	return;
}


template<typename _T>int Zhang(_T uv[][2], int iImage_Count, int w_In_Point=11, int h_In_Point=8, _T fGrid_Size=0.02)
{//讲讲废弃
	int iCorner_Count = w_In_Point * h_In_Point, bRet = 0;
	
	//************一股脑全非陪*********************
	int iSize = ALIGN_SIZE_128(iCorner_Count * 2 * sizeof(_T)) +	//pCorner_3D
		ALIGN_SIZE_128(iImage_Count * 9 * sizeof(_T)) +				//pH
		ALIGN_SIZE_128(iImage_Count * 16 * sizeof(_T));				//pT

	unsigned char* pBuffer = (unsigned char*)pMalloc(iSize), *p = pBuffer;;
	if (!pBuffer) goto END;
	
	_T(*pCorner_Ref)[2], (*pH)[3 * 3], (*pT)[4 * 4];
	pCorner_Ref = (_T(*)[2])p, p += ALIGN_SIZE_128(iCorner_Count * 2 * sizeof(_T));
	pH = (_T(*)[3 * 3])p, p += ALIGN_SIZE_128(iImage_Count * 9 * sizeof(_T));
	pT = (_T(*)[4 * 4])p;
	//************一股脑全非陪*********************
	
	Gen_Corner_Ref(w_In_Point, h_In_Point, fGrid_Size, pCorner_Ref);
	//Disp((_T*)pCorner_Ref, iCorner_Count, 2, "Corner");

	//建立一个假象的标准棋盘，位于(0,0,0)
	_T fError;
	fError = 0;
	unsigned long long tStart;
	tStart = iGet_Tick_Count();

	//只用0.22 ms，没有优必要
	for(int i=0;i<iImage_Count;i++)
	{
		int iPos = i * iCorner_Count;
		int iResult = Estimate_H_Ref<_T>(pCorner_Ref, uv+ iPos, iCorner_Count, pH[i], 1, 0);
		if(!iResult)
		{
			Estimate_H_Ref(pCorner_Ref, uv + iPos, iCorner_Count, pH[i], 1, 1);
			printf("Use SVD\n");
		}
		//printf("Result:%d %f\n", iResult, Test_H_2d<_T>(pCorner_Ref, uv + iPos, iCorner_Count,pH[i]));
		fError += Test_H_2d<_T>(pCorner_Ref, uv + iPos, iCorner_Count, pH[i]);
	}
	printf("Error Sum:%f\n", fError);

END:
	if (!bRet)
	{
		Free(pBuffer);
	}
	return bRet;
}

template<typename _T>static void Sort_Observation(Point_2D<_T> Observation[], int iCamera_Count, int iObservation_Count)
{//来个桶排序了事
	int i;
	union {
		int* pPoint_Per_Cam;
		int* pStart;
	};

	pPoint_Per_Cam = (int*)pMalloc(iCamera_Count * sizeof(_T));
	memset(pPoint_Per_Cam, 0, iCamera_Count * sizeof(_T));
	for (i = 0; i < iObservation_Count; i++)
		pPoint_Per_Cam[Observation[i].m_iCamera_Index]++;

	int iCur = 0;
	for (i = 0; i < iObservation_Count; i++)
	{
		int iCount = pPoint_Per_Cam[i];
		pStart[i] = iCur;
		iCur += iCount;
	}
	Point_2D<_T>* pObservation = (Point_2D<_T>*)pMalloc(iObservation_Count * sizeof(Point_2D<_T>));
	for (i = 0; i < iObservation_Count; i++)
	{
		Point_2D<_T> oPoint = Observation[i];
		pObservation[pStart[oPoint.m_iCamera_Index]++] = oPoint;
	}
	memcpy(Observation, pObservation, iObservation_Count * sizeof(Point_2D<_T>));
	Free(pObservation);
	Free(pPoint_Per_Cam);
	return;
}

//template<typename _T>static void Init_LM_Param_Ceres(LM_Param_Ceres<_T>* poParam, int iCamera_Count, int iObservation_Count)
//{
//	LM_Param_Ceres<_T> oParam;
//	//显示初始化值，免得东一块西一块神出鬼没
//	oParam.reuse_diagonal = 0;		//初始化为什么，待考
//	oParam.bIsStepSuccessful = 0;	//迭代是否成功
//	oParam.bStep_is_valid = 0;
//	oParam.m_iIter = 0;
//	oParam.num_consecutive_nonmonotonic_steps = 0;
//	oParam.decrease_factor = 2.f;
//	oParam.accumulated_reference_model_cost_change = 0;
//	oParam.accumulated_candidate_model_cost_change = 0;
//	oParam.radius = 10000.f;
//
//	int iSize = iObservation_Count * 15 * 2 +	//J
//		iObservation_Count * 2;					//Residual
//	oParam.J = (_T(*)[2][15])pMalloc(iObservation_Count * 15 * 2 * sizeof(_T));
//	oParam.Residual = (_T(*)[2])pMalloc(iObservation_Count * 2 * sizeof(_T));
//	iSize = iCamera_Count * 6 + 9;
//	oParam.m_pDiag = (_T*)pMalloc(iSize * sizeof(_T));
//	oParam.JtE = (_T*)pMalloc(iSize * sizeof(_T));
//	iSize *= iSize;
//	oParam.H = (_T*)pMalloc(iSize * sizeof(_T));
//	memset(oParam.H, 0, iSize * sizeof(_T));
//
//	*poParam = oParam;
//}

//template<typename _T>static void Get_J_Residual(_T T[][16], _T K[],_T D[], _T P[][2], Point_2D<_T> uv[], int iObservation_Count,
//	_T J[][2][15], _T E[][2], _T* pfSum_e)
//{//先对所有样本求个导
//	_T fSum_e = 0;
//	for (int i = 0; i < iObservation_Count; i++)
//	{
//		Point_2D<_T> oUV = uv[i];
//		_T* P1 = P[oUV.m_iPoint_Index];
//		_T P2[4] = { P1[0], P1[1], 0,1 },
//			dE_dK[2 * 4], dE_dD[2 * 5], dE_dKsi[2 * 6];
//		
//		//Disp(P1, 1, 3, "P");
//		//Disp(T[oUV.m_iCamera_Index], 4, 4, "T");
//		//Disp(oUV.m_Pos, 1, 2, "uv");
//		Get_PnP_Deriv<_T>(P2, oUV.m_Pos, T[oUV.m_iCamera_Index], K, D, dE_dK, dE_dKsi, dE_dD, NULL, E[i]);
//		//if (i == 0)
//		{
//			_T uv[3];
//			Get_uv_Ref(P2, T[oUV.m_iCamera_Index], K, D, uv);
//			Disp(uv, 1, 2);
//			//printf("E:%f %f %f %f\n", oUV.m_Pos[0] - uv[0], oUV.m_Pos[1] - uv[1],E[i][0], E[i][1]);
//		}
//		/*Disp(K, 3, 3, "K");
//		Disp(D, 1, 5, "D");
//		Disp(T[0], 4, 4, "T");
//		Disp(P2, 1, 3, "P");
//		Disp(uv[0].m_Pos, 1, 2, "uv");*/
//
//		//Disp(dE_dK, 2, 4, "dE/dK");
//		//Disp(dE_dD, 2, 5, "dE/dD");
//		//Disp(dE_dKsi, 2, 6, "dE/dKsi");
//		//Disp(E[i], 1, 2, "E");
//		_T* pJ = (_T*)J[i];
//		Copy_Matrix_Partial(dE_dKsi, 2, 6, (_T*)J[i], 15, 0, 0);
//		Copy_Matrix_Partial(dE_dK, 2, 4, (_T*)J[i], 15, 6, 0);
//		Copy_Matrix_Partial(dE_dD, 2, 5, (_T*)J[i], 15, 10, 0);
//		fSum_e += E[i][0] * E[i][0] + E[i][1] * E[i][1];
//		//printf("i:%d Error:%f\n", i, fSum_e);
//		//Disp((_T*)pJ, 2, 15, "J");
//		
//	}
//	if (pfSum_e)
//		*pfSum_e = fSum_e;
//	return;
//}

//template<typename _T>static void Get_H_1(_T J[][2][15], Point_2D<_T>uv[], int iCamera_Count, int iCorner_Per_Image, _T Sigma_H[])
//{
//	int i, j, x, y;
//	int iWidth_H = iCamera_Count * 6 + 9;
//	memset(Sigma_H, 0, iWidth_H * iWidth_H * sizeof(_T));
//	//一个JtJ分4块，只搞3块
//
//	_T Corner[9 * 9] = { 0 };
//	{
//		for (i = 0; i < iCamera_Count; i++)
//		{
//			_T Camera[6 * 6] = { 0 }, Cam_Corner[6 * 9] = { 0 };
//			for (j = 0; j < iCorner_Per_Image; j++)
//				Get_H_Block(J[i * iCorner_Per_Image + j], Camera, Cam_Corner, Corner);
//
//			//补下三角
//			for (y = 1; y < 6; y++)
//			{
//				for (x = 0; x < y; x++)
//				{
//					int iDest_Pos = y * 6 + x,
//						iSource_Pos = x * 6 + y;
//					Camera[iDest_Pos] = Camera[iSource_Pos];
//				}
//			}
//
//			for (y = 1; y < 9; y++)
//			{
//				for (x = 0; x < y; x++)
//				{
//					int iDest_Pos = y * 9 + x,
//						iSource_Pos = x * 9 + y;
//					Corner[iDest_Pos] = Corner[iSource_Pos];
//				}
//			}
//
//
//			//将块拷到sigma_H
//			Copy_Matrix_Partial(Camera, 6, 6, Sigma_H, iWidth_H, (i * 6), (i * 6));
//			Copy_Matrix_Partial(Cam_Corner, 6, 9, Sigma_H, iWidth_H,  iWidth_H - 9, (i * 6));
//
//			_T *pDest_1 = &Sigma_H[(iWidth_H - 9) * iWidth_H + i * 6];
//			for (y = 0; y < 9; y++, pDest_1 += iWidth_H)
//			{
//				for (x = 0; x < 6; x++)
//					pDest_1[x] = Cam_Corner[x * 9 + y];
//			}
//			Copy_Matrix_Partial(Corner, 9, 9, Sigma_H, iWidth_H, iWidth_H - 9, iWidth_H - 9);
//		}
//	}	
//	return;
//}

//template<typename _T>static void Get_JtE(_T J[][2][15], _T Residual[][2], Point_2D<_T> uv[], int iCamera_Count, int iObservation_Count, _T JtE[])
//{
//	int i, iIntrinsic_Pos = iCamera_Count * 6;
//	_T JtE_1[15];
//	memset(JtE, 0, (iCamera_Count * 6 + 9) * sizeof(_T));
//	_T* pIntrinsic = &JtE[iIntrinsic_Pos];
//
//	for (i = 0; i < iObservation_Count; i++)
//	{
//		Point_2D<_T> oPoint = uv[i];
//		for (int j = 0; j < 15; j++)
//			JtE_1[j] = J[i][0][j] * Residual[i][0] + J[i][1][j] * Residual[i][1];
//		Vector_Add(&JtE[oPoint.m_iCamera_Index * 6], JtE_1, 6, &JtE[oPoint.m_iCamera_Index * 6]);
//		Vector_Add(&JtE[iIntrinsic_Pos], &JtE_1[6], 9, &JtE[iIntrinsic_Pos]);
//	}
//	//记得乘以-1才是Ax = b中的b
//	int iOrder = iCamera_Count * 6 + 9;
//	for (i = 0; i < iOrder; i++)
//		JtE[i] = JtE[i];
//	//Disp(JtE, iOrder, 1, "J'E");
//	return;
//}

//template<typename _T>static void Get_Diag(LM_Param_Ceres<_T> oParam, int iOrder, _T lm_diagonal[])
//{
//	int i;
//	//先来个对角线
//	oParam.m_pDiag[0] = 1;
//	if (!oParam.reuse_diagonal)
//	{
//		for (i = 0; i < iOrder; i++)
//		{
//			oParam.m_pDiag[i] = oParam.H[i * iOrder + i];	// sqrt(Abs(A[i * iOrder + i]) / oParam.radius);
//			//限个幅
//			oParam.m_pDiag[i] = Clip3(1e-6, 1e32, oParam.m_pDiag[i]);
//		}
//	}
//	for (i = 0; i < iOrder; i++)
//		lm_diagonal[i] = sqrt(oParam.m_pDiag[i] / oParam.radius);
//}

//template<typename _T>void Solve_Linear_Gause(_T* A, int iOrder, _T* B, _T* X, int* pbSuccess)
//{//用高斯列主元法求解线性方程组, 要点：
//	//1，高斯法完全等价人肉行变换，只不过没有用人肉的公倍数法，而是步步都是将主元变为1
//	//2，选列主元的原因是保证在 其他系数/aij时分母不至于太小。分母小误差大
//	//3,在列主元太小（<eps)的情况下算不满秩，退出。实际上主元太小会导致后面的除法严重误差
//	int y, x, i, iRow_Size;
//	int iMax, iTemp, iPos, * pQ;
//	_T fMax, * pfMax_Row, fValue;
//	const _T eps = (_T)1e-10;
//	int bSuccess = 1;
//	pQ = (int*)pMalloc(iOrder * sizeof(int));
//	_T* Ai = (_T*)pMalloc((iOrder + 1) * iOrder * sizeof(_T));
//	if (!pQ || !Ai)
//	{
//		bSuccess = 0;
//		goto END;
//	}
//
//	iPos = 0;
//	for (y = 0; y < iOrder; y++)
//	{
//		for (x = 0; x < iOrder; x++, iPos++)
//			Ai[iPos] = A[y * iOrder + x];
//		Ai[iPos++] = B[y];
//		pQ[y] = y;	//每次主元所在的行
//	}
//
//	//Disp(Ai, iOrder, iOrder + 1,"\n");
//	iRow_Size = iOrder + 1;
//	//为了便于理解，以下iMax表示Q中的索引，而不是Ai中的行号
//	for (y = 0; y < iOrder; y++)
//	{
//		iMax = y;	//感觉这里错了，应该是 iMax = pQ[i]
//		fMax = Ai[pQ[iMax] * iRow_Size + y];
//		for (i = y + 1; i < iOrder; i++)
//		{//寻找列主元
//			if (abs(Ai[iPos = pQ[i] * iRow_Size + y]) > abs(fMax))
//			{
//				fMax = Ai[iPos];
//				iMax = i;
//			}
//		}
//		/*if (iMax != y)
//			printf("%d\n", iMax);*/
//
//		if (abs(fMax) < eps)
//		{//列主元为0，显然不满秩，该方程没有唯一解
//			printf("不满秩,列主元为：%f\n", fMax);
//			bSuccess = 0;
//			goto END;
//		}
//
//		//将最大元SWAP到Q的当前位置上
//		iTemp = pQ[y];
//		pQ[y] = pQ[iMax];
//		pQ[iMax] = iTemp;
//
//		//对iMax所在的行进行系数计算，新系数/=A[y][y]
//		pfMax_Row = &Ai[pQ[y] * iRow_Size];
//		pfMax_Row[y] = 1.f;
//		for (x = y + 1; x < iRow_Size; x++)
//			pfMax_Row[x] /= fMax;
//
//		//Disp(Ai, iOrder, iOrder + 1, "\n");
//
//		//对后面所有行代入
//		for (i = y + 1; i < iOrder; i++)
//		{//i表示第i行
//			iPos = pQ[i] * iRow_Size;
//			if ((fValue = Ai[iPos + y]) != 0)
//			{//对于对应元不为0才有算的意义
//				for (x = y + 1; x < iRow_Size; x++)
//					Ai[iPos + x] -= fValue * pfMax_Row[x];
//				Ai[iPos + y] = 0;	//此处也不是必须的，置零只是好看
//			}
//			//Disp(Ai, iOrder, iOrder + 1, "\n");
//		}
//	}
//
//	//第一个解
//	X[iOrder - 1] = Ai[pQ[iOrder - 1] * iRow_Size + iOrder];
//
//	//回代，从Q[iOrder-1]开始回代，从最下一行向上回代
//	for (y = iOrder - 2; y >= 0; y--)
//	{
//		//将解向上回代
//		iPos = pQ[y] * iRow_Size;
//		//fValue=b
//		fValue = Ai[iPos + iOrder];
//		for (x = y + 1; x < iOrder; x++)
//		{
//			fValue -= Ai[iPos + x] * X[x];
//			Ai[iPos + x] = 0;		//此处不是必须的，算完置0，好看一些而已
//		}
//		X[y] = fValue;
//		Ai[iPos + iOrder] = fValue;	//此处不是必须，好看而已
//	}
//
//END:
//	//Disp(Ai, iOrder, iOrder + 1, "\n");
//	//*pbSuccess = 1;
//	*pbSuccess = bSuccess;
//	if (pQ)
//		Free(pQ);
//	if (Ai)
//		Free(Ai);
//	//验算
//	//Linear_Equation_Check(A, iOrder, B, X, (_T)ZERO_APPROCIATE);
//	return;
//}

//template<typename _T>_T fGet_model_cost_change(Point_2D<_T> Observation[], int iObservation_Count, _T Residual[][2], _T J[][2][15], _T x[], _T JtE[], int iCamera_Count)
//{
//	int iIntrinsic_Pos = iCamera_Count * 6;
//	_T model_cost_change = 0;
//	_T x1[12];
//	memcpy(&x1[6], &x[iIntrinsic_Pos], 6 * sizeof(_T));
//	for (int i = 0; i < iObservation_Count; i++)
//	{
//		//此处有重大改动，否则改了原来的Residual，是一个Bug
//		//_T* E1 = Residual[i];
//		_T E1[] = { Residual[i][0],Residual[i][1] };
//		Point_2D<_T> oPoint = Observation[i];
//
//		_T E2[2];//, x1[12];
//		memcpy(x1, &x[oPoint.m_iCamera_Index * 6], 6 * sizeof(_T));
//		//memcpy(&x1[6], &x[iIntrinsic_Pos], 6 * sizeof(_T));
//		Matrix_Multiply((_T*)J[i], 2, 6, x1, 1, E2);
//
//		E1[0] = (E1[0] + E2[0] / 2.f);
//		E1[1] = (E1[1] + E2[1] / 2.f);
//		//Disp(E1, 2, 1, "E2");
//		model_cost_change += E2[0] * E1[0] + (E2[1] * E1[1]);
//	}
//
//	return model_cost_change;
//}

//template<typename _T>_T fGet_candidate_cost(_T T[][4 * 4],_T K[], _T D[], Point_2D<_T> uv[], _T x[], _T P[][2],
//	int iCamera_Count, int iPoint_3D_Count, int iObservation_Count, _T J[][2][15], _T Residual[][2])
//{//由于ceres很难跟，干脆自己做一个看看
////x为解出来的扰动，加上扰动，看误差到
//	_T candidate_cost = 0;
//	int i;
//	for (i = 0; i < iCamera_Count; i++)
//	{
//		_T* pT_Delta_6 = &x[i * 6];
//		_T T_Delta_4x4[4 * 4];
//		//Disp(pPose_Delta_6, 6, 1, "pPose_Delta_6");
//		//此处更新很有可能是错的！！！
//		se3_2_SE3(pT_Delta_6, T_Delta_4x4);
//		//Disp(T_Delta_4x4, 4, 4, "Delta");
//		Matrix_Multiply(T_Delta_4x4, 4, 4, T[i], 4, T[i]);
//		Disp(T[i], 4, 4, "T");
//	}
//	//对内参也进行修正
//	//Vector_Add(Intrinsic, &x[iCamera_Count * 6], 6, Intrinsic);
//	_T K1[9], D1[5];
//	memcpy(K1, K, 9 * sizeof(_T));
//	memcpy(D1, D, 5 * sizeof(_T));
//	K1[0] += x[iCamera_Count * 6];
//	K1[2] += x[iCamera_Count * 6 + 1];
//	K1[4] += x[iCamera_Count * 6 + 2];
//	K1[5] += x[iCamera_Count * 6 + 3];
//	Vector_Add(D1, &x[iCamera_Count * 6 + 4], 5,D1);
//	//Disp(K1, 3, 3, "K");
//	//Disp(D1, 1, 5, "D");
//
//	Get_J_Residual<_T>(T, K1,D1, P,uv, iObservation_Count,
//		J, Residual, &candidate_cost);
//	//printf("Error:%f\n", candidate_cost);
//	return candidate_cost;
//}

//template<typename _T>void Update_Param_Ceres(LM_Param_Ceres<_T>* poParam, _T candidate_cost, _T model_cost_change)
//{//更新一下Cerer的Param
//
//	LM_Param_Ceres<_T> oParam = *poParam;
//	_T relative_decrease;
//	if (oParam.current_cost > MAX_FLOAT)
//		relative_decrease = -MAX_FLOAT;
//	/*if (model_cost_change == 0)
//	printf("Error");*/
//	relative_decrease = (oParam.current_cost - candidate_cost) / model_cost_change;
//	const _T historical_relative_decrease =
//		(oParam.reference_cost - candidate_cost) /
//		(oParam.accumulated_reference_model_cost_change + model_cost_change);
//	//printf("relative_decrease:%f\n", relative_decrease);
//	relative_decrease = Max(relative_decrease, historical_relative_decrease);
//	//printf(" %f\n", relative_decrease);
//	oParam.bIsStepSuccessful = relative_decrease > 0.001;
//
//	//注意，relative_decrease也是Step Quality
//	if (oParam.bIsStepSuccessful)
//	{
//		oParam.radius = (_T)(oParam.radius / max(1.0 / 3.0, 1.0 - pow(2.f * relative_decrease - 1.0, 3)));
//		oParam.radius = (_T)Min(10000000000000000., oParam.radius);
//		//printf("radius:%f relative_decrease:%f\n", oParam.radius,relative_decrease);
//
//		oParam.decrease_factor = 2.f;
//		oParam.reuse_diagonal = 0;
//
//		//candidate_cost_, model_cost_change_
//		oParam.current_cost = candidate_cost;
//		oParam.accumulated_candidate_model_cost_change += model_cost_change;
//		oParam.accumulated_reference_model_cost_change += model_cost_change;
//		//printf("minimum_cost:%f\n", oParam.minimum_cost);
//		if (oParam.current_cost < oParam.minimum_cost)
//		{
//			oParam.minimum_cost = oParam.current_cost;
//			oParam.candidate_cost = oParam.current_cost;
//			oParam.accumulated_candidate_model_cost_change = 0;
//		}
//		else
//		{	//这个奇怪
//			//printf("here");
//			oParam.num_consecutive_nonmonotonic_steps++;
//			if (oParam.current_cost > candidate_cost)
//			{
//				oParam.candidate_cost = oParam.current_cost;
//				oParam.accumulated_candidate_model_cost_change = 0.0;
//			}
//		}
//		if (oParam.num_consecutive_nonmonotonic_steps == 0)
//		{
//			oParam.reference_cost = candidate_cost;
//			oParam.accumulated_reference_model_cost_change =
//				oParam.accumulated_candidate_model_cost_change;
//		}
//	}else
//	{//失败
//		oParam.radius = oParam.radius / oParam.decrease_factor;
//		oParam.decrease_factor *= 2.0;
//		oParam.reuse_diagonal = 1;
//	}
//	*poParam = oParam;
//
//}

//template<typename _T>_T Temp_Update(_T T[][4 * 4], _T K[], _T D[], Point_2D<_T> uv[], _T x[], _T P[][2],
//	int iCamera_Count, int iPoint_3D_Count, int iObservation_Count, _T J[][2][15], _T Residual[][2])
//{//
//	_T candidate_cost = 0;
//	int i;
//	for (i = 0; i < iCamera_Count; i++)
//	{
//		_T* pT_Delta_6 = &x[i * 6];
//		_T T_Delta_4x4[4 * 4];
//		//Disp(pT_Delta_6, 6, 1, "pPose_Delta_6");
//		se3_2_SE3(pT_Delta_6, T_Delta_4x4);
//		//Disp(T_Delta_4x4, 4, 4, "Delta");
//		Matrix_Multiply(T_Delta_4x4, 4, 4, T[i], 4, T[i]);
//		//Disp(T[i], 4, 4, "T");
//	}
//
//	//对内参也进行修正
//	K[0] += x[iCamera_Count * 6];
//	K[2] += x[iCamera_Count * 6 + 1];
//	K[4] += x[iCamera_Count * 6 + 2];
//	K[5] += x[iCamera_Count * 6 + 3];
//	Vector_Add(D, &x[iCamera_Count * 6 + 4], 5, D);
//	////此处姆队对K3限幅
//	//D[0] += x[iCamera_Count * 6 + 4];
//	//D[1] += x[iCamera_Count * 6 + 5];
//	//_T r3_delta = x[iCamera_Count * 6 + 6];
//	//D[2] += Clip3(-0.1,0.1,r3_delta);	//x[iCamera_Count * 6 + 6];
//	//D[3] += x[iCamera_Count * 6 + 7];
//	//D[4] += x[iCamera_Count * 6 + 8];
//
//	Get_J_Residual<_T>(T, K, D, P, uv, iObservation_Count,
//		J, Residual, &candidate_cost);
//	//Disp((_T*)J, iObservation_Count * 2, 15, "J");
//	return candidate_cost;
//}


//template<typename _T>void BA_PnP_Zhang_LM_1(LM_Param_Ceres<_T>* poParam,_T T[][16], int iCamera_Count, _T K[], _T D[], _T P[][2], int iPoint_3D_Count, Point_2D<_T> uv[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
//{//可以废弃了，实践证明，在像素平面上搞是愚蠢的
//	LM_Param_Ceres<_T> oParam = *poParam;
//
//	//试探阻尼因子
//	int iOrder = iCamera_Count * 6 + 9;
//	_T lamda = 1e-3, * pDiag = (_T*)pMalloc(iOrder * sizeof(_T)),
//		*pDiag_Sqrt = (_T*)pMalloc(iOrder * sizeof(_T)),
//		* x = (_T*)pMalloc(iOrder * sizeof(_T)),
//		(*J)[2][15] = (_T(*)[2][15])pMalloc(iObservation_Count * 15 * 2 * sizeof(_T)),
//		*JtE = (_T*)pMalloc((iCamera_Count * 6 + 9) * sizeof(_T)),
//		(* pResidual_1)[2] = (_T(*)[2])pMalloc(iObservation_Count * 2 * sizeof(_T)),
//		(*pT_1)[16] = (_T(*)[16])pMalloc(iCamera_Count * 16 * sizeof(_T));
//	_T K1[9], D1[5];
//	memcpy(K1, K, 9 * sizeof(_T));
//	memcpy(D1, D, 5 * sizeof(_T));
//	memcpy(pT_1, T, iCamera_Count * 16 * sizeof(_T));
//
//	//首先求出J'J, J'E
//	Get_H_1(oParam.J, uv, iCamera_Count, iObservation_Count / iCamera_Count, oParam.H);
//	Get_JtE(oParam.J, oParam.Residual, uv, iCamera_Count, iObservation_Count, oParam.JtE);
//	//Disp(oParam.H, iOrder, iOrder, "H");
//	/*Disp(oParam.JtE, iOrder, 1, "JtE");
//	Disp(K, 3, 3, "K");
//	Disp(D, 1, 5, "D");
//	Disp(T[0], 4, 4, "T");*/
//	//Get_Diag(oParam, iOrder, lm_diagonal);
//	//新备份对角线
//	for (int i = 0; i < iOrder; i++)
//	{
//		pDiag[i] = oParam.H[i * iOrder + i];
//		pDiag_Sqrt[i] = sqrt(pDiag[i]);
//	}
//	for (int i = 0; i < iOrder; i++)
//		printf("%f\t", oParam.H[i * iOrder + i]);
//	printf("\n");
//	//Disp(lm_diagonal, 1, iOrder, "Diag");
//	//Disp(pDiag_Sqrt, 1, iOrder, "Diag Sqrt");
//
//	//printf("%f\n", fGet_Cond_Num(oParam.H, iOrder));
//	for (int i = 0; i < iOrder; i++)
//		oParam.H[i * iOrder + i] = pDiag[i] * (1.f + lamda);   // *Diag[i];
//
//	//for (int i = 0; i < iOrder; i++)
//	//	printf("%.8f\t", oParam.H[i * iOrder + i]);
//	//printf("%f\n", fGet_Cond_Num(oParam.H, iOrder));
//	memcpy(J, oParam.J, iObservation_Count * 15 * 2 * sizeof(_T));
//	memcpy(JtE, oParam.JtE, (iCamera_Count * 6 + 9) * sizeof(_T));
//	_T nn = 2;
//	for (int i = 0;i<200; i++)
//	{
//		int iResult;
//		Solve_Linear_Gause_AAt(oParam.H, iOrder, JtE, x, &iResult);
//		if (!iResult)
//			return;
//		//Disp(x, 1, 15, "x");
//		//Disp(oParam.JtE, iOrder, 1, "JtE");
//		//Disp(oParam.H, iOrder, iOrder, "H");
//
//		memcpy(K1, K, 9 * sizeof(_T));
//		memcpy(D1, D, 5 * sizeof(_T));
//		memcpy(pT_1, T, iCamera_Count * 16 * sizeof(_T));
//		
//		_T fError= Temp_Update<_T>(pT_1, K1, D1, uv, x, P, iCamera_Count, iPoint_3D_Count, iObservation_Count, J, pResidual_1);
//
//		for (int i = 0; i < iOrder; i++)
//			printf("%.8f\t", pDiag[i]);
//		printf("\n");
//		for (int j = 0; j < iOrder; j++)
//			oParam.H[j * iOrder + j] = pDiag[j]*(1 + lamda * nn);
//		
//		for (int i = 0; i < iOrder; i++)
//			printf("%.8f\t", oParam.H[i * iOrder + i]);
//		printf("\n");
//		if (i == 2)
//		{
//			Disp(oParam.H, 15, 15, "H");
//			Disp(JtE, 15, 1, "JtE");
//		}
//		memcpy(J, oParam.J, iObservation_Count * 15 * 2 * sizeof(_T));
//		nn *= 2;
//	}
//
//	return;
//}

//template<typename _T>void BA_PnP_Zhang_LM_Ceres_1(LM_Param_Ceres<_T>* poParam, _T T[][16], int iCamera_Count, _T K[], _T D[], _T Point_3D[][2], int iPoint_3D_Count, Point_2D<_T> uv[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
//{//重新做一个LM，此处包括几部分，1，解矛盾方程Jx = e;	2,更新Param各种数据状态，3，确定是否成功
//	LM_Param_Ceres<_T>oParam = *poParam;
//	int i, bResult, iOrder = iCamera_Count * 6 + 9;
//	
//	//开内存
//	_T* lm_diagonal = (_T*)pMalloc(iOrder * sizeof(_T));
//	_T* x = (_T*)pMalloc(iOrder * sizeof(_T));
//	_T(*J_1)[2][15] = (_T(*)[2][15])pMalloc(iObservation_Count * 2 * 15 * sizeof(_T));
//	_T(*Residual_1)[2] = (_T(*)[2])pMalloc(iObservation_Count * 2 * sizeof(_T));
//	
//	if (!oParam.reuse_diagonal)
//	{//当前面失败的时候，这里可以偷懒不干
//		//先搞个H矩阵 H = JtJ，不能简单一乘了之
//		Get_H_1(oParam.J, uv, iCamera_Count, iObservation_Count / iCamera_Count, oParam.H);
//
//		Get_JtE(oParam.J, oParam.Residual, uv, iCamera_Count, iObservation_Count, oParam.JtE);
//	}
//
//	Get_Diag(oParam, iOrder, lm_diagonal);
//
//	//第一步，修改对角线元素，这一步与g2o有根本区别，g2o对角线加一个统一的值
//	for (i = 0; i < iOrder; i++)
//		oParam.H[i * iOrder + i] += lm_diagonal[i] * lm_diagonal[i];   // *Diag[i];
//
//	Solve_Linear_Gause_AAt(oParam.H, iOrder, oParam.JtE, x, &bResult);
//	//Disp(oParam.JtE, 15, 1);
//
//	_T model_cost_change = fGet_model_cost_change<_T>(uv, iObservation_Count, oParam.Residual, oParam.J, x, oParam.JtE, iCamera_Count);
//	int bStep_is_valid = model_cost_change > 0;
//	oParam.bStep_is_valid = bStep_is_valid;
//
//	_T(*pT_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));
//	_T K1[9], D1[5];
//	memcpy(pT_1, T, iCamera_Count * 16 * sizeof(_T));
//	memcpy(K1, K, 9 * sizeof(_T));
//	memcpy(D1, D, 5 * sizeof(_T));
//	//Disp(Intrinsic_1, 9, 1, "K_D");
//
//	_T candidate_cost = fGet_candidate_cost<_T>(pT_1, K1,D1, uv,
//		x, Point_3D, iCamera_Count, iPoint_3D_Count, iObservation_Count, J_1, Residual_1);
//	printf("Cost:%f", candidate_cost);
//	Update_Param_Ceres(&oParam, candidate_cost, model_cost_change);
//
//	//加入成功了，就把新威姿，新内参，新雅可比，新误差更新到Param中
//	if (oParam.bIsStepSuccessful)
//	{
//		memcpy(T, pT_1, iCamera_Count * 4 * 4 * sizeof(_T));
//		memcpy(K, K1, 9 * sizeof(_T));
//		memcpy(D, K1, 5 * sizeof(_T));
//		memcpy(oParam.J, J_1, iObservation_Count * 2 * 12 * sizeof(_T));
//		memcpy(oParam.Residual, Residual_1, iObservation_Count * 2 * sizeof(_T));
//	}
//	else
//	{//当前面失败的时候，这里必须恢复方程
//		for (i = 0; i < iOrder; i++)
//			oParam.H[i * iOrder + i] -= lm_diagonal[i] * lm_diagonal[i];   // *Diag[i];
//	}
//	if (x)Free(x);
//	if (pT_1)Free(pT_1);
//	if (lm_diagonal)Free(lm_diagonal);
//	if (J_1)Free(J_1);
//	if (Residual_1)Free(Residual_1);
//	*poParam = oParam;
//	return;
//}

//template<typename _T>static void Free_LM_Param_Ceres(LM_Param_Ceres<_T>* poParam)
//{
//	LM_Param_Ceres<_T>oParam = *poParam;
//	if (oParam.J)Free(oParam.J);
//	if (oParam.Residual)Free(oParam.Residual);
//	if (oParam.m_pDiag)Free(oParam.m_pDiag);
//	if (oParam.JtE)Free(oParam.JtE);
//	if (oParam.H)Free(oParam.H);
//	*poParam = {};
//}

//template<typename _T>void BA_PnP_Zhang_Ceres(_T T[][16], int iCamera_Count, _T K[3*3],_T D[5], _T P[][2], int iPoint_3D_Count, Point_2D<_T> uv[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
//{//重写一个，原来的已经乱了 
//	//先将可能乱排的点重排，就是将点按照相机一团一团聚类
//	Sort_Observation(uv, iCamera_Count, iObservation_Count);
//
//	LM_Param_Ceres<_T> oParam;
//	Init_LM_Param_Ceres(&oParam, iCamera_Count, iObservation_Count);
//
//	//先把所有的雅可比，Residual求出来
//	Get_J_Residual<_T>(T, K,D, P, uv, iObservation_Count,oParam.J, oParam.Residual, &oParam.current_cost);
//	//printf("Error:%f\n", oParam.current_cost);
//	oParam.reference_cost = oParam.candidate_cost = oParam.minimum_cost = oParam.current_cost;
//
//	int iIter;
//	_T fPrevious_Cost = 1e20;
//	_T fLoss_Diff_eps = (_T)1e-10;
//	fLoss_Diff_eps *= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大
//
//	for (iIter = 1;; iIter++)
//	{
//		oParam.m_iIter = iIter;
//		//BA_PnP_Zhang_LM_Ceres_1(&oParam, T, iCamera_Count, K, D, P, iPoint_3D_Count, uv, iObservation_Count);
//		BA_PnP_Zhang_LM_1(&oParam,T, iCamera_Count, K, D, P, iPoint_3D_Count, uv, iObservation_Count);
//		if (oParam.bIsStepSuccessful)
//		{
//			printf("Iter:%d Error:%e bStep_is_valid:%d bIsStepSuccessful:%d\n", oParam.m_iIter, oParam.current_cost, (int)oParam.bStep_is_valid, (int)oParam.bIsStepSuccessful);
//			//printf("radius:%f\n", oParam.radius);
//		}
//		if (!oParam.bStep_is_valid && !oParam.bIsStepSuccessful)
//			break;
//		if (abs(oParam.current_cost - fPrevious_Cost) < fLoss_Diff_eps && oParam.bIsStepSuccessful)
//		{
//			printf("Iter:%d Error:%e bStep_is_valid:%d bIsStepSuccessful:%d\n", oParam.m_iIter, oParam.current_cost, (int)oParam.bStep_is_valid, (int)oParam.bIsStepSuccessful);
//			break;
//		}
//		fPrevious_Cost = oParam.candidate_cost;
//	}
//	Free_LM_Param_Ceres(&oParam);
//	return;
//}

template<typename _T>void Get_Diag(_T A[], int iOrder, _T Diag[])
{
	for (int i = 0; i < iOrder; i++)
		Diag[i] = A[i * iOrder + i];
	return;
}

template<typename _T>void Zhang_Update_K_D_T(_T Delta[], int iCamera_Count, _T K[], _T D[], _T T[][3 * 4])
{//更新一下K,D,T
	for (int i = 0; i < 4; i++)
		K[i] += Delta[iCamera_Count * 6 + i];
	for (int i = 0; i < 5; i++)
		D[i] += Delta[iCamera_Count * 6 + 4 + i];
	for (int i = 0; i < iCamera_Count; i++)
	{
		_T* pT_Delta_6 = &Delta[i * 6];
		_T T_Delta_4x4[4 * 4];
		_T T_Org[4 * 4];
		memcpy(T_Org, T[i], 3 * 4 * sizeof(_T));
		T_Org[12] = T_Org[13] = T_Org[14] = 0, T_Org[15] = 1;
		//Disp(T_Org, 4, 4, "Torg");
		Gen_Pose_By_V3_t(&pT_Delta_6[3], pT_Delta_6, T_Delta_4x4);
		//Disp(T_Delta_4x4, 4, 4, "Delta_T");

		Matrix_Multiply(T_Delta_4x4, 4, 4,T_Org , 4, T_Org);
		//Disp(T_Org, 4, 4, "T");

		/*_T R[9], t[3], delta_R[9], delta_t[3];
		T_2_R9_t <_T>(T[i], R, t);
		T_2_R9_t(T_Delta_4x4, delta_R, delta_t);
		Matrix_Multiply_3x3(delta_R, R, R);
		Matrix_Multiply_3x1(delta_R, t, t);
		Vector_Add(t, delta_t, 3,t);
		Disp(R, 3, 3, "R");
		Disp(t, 3, 1, "t");*/
		
		memcpy(T[i], T_Org, 3 * 4 * sizeof(_T));
	}
	return;
}

template<typename _T>int BA_PnP_Zhang_1(_T T[][3 * 4], int iCamera_Count, _T K[4], _T D[5], _T P[][2],
	int iPoint_3D_Count, Point_2D<_T> uv[], int iObservation_Count,
	_T* pfError,
	_T fLoss_Delta_esp,			//两次误差收敛的差到这个数可以停机
	_T fLoss_eps,	//误差迭代到这个数可以停机
	int iMax_Iter = 100)		//最多迭代次数
{//自己来，手搓一下

	//可以开始试探了
	int bRet = 0;
	_T K9[3 * 3], fError, fPre_Error, fError_Delta;
	K4_2_K9(K, K9);
	fPre_Error =Zhang_Get_Error<_T>(K, D, T, P, uv, iObservation_Count);

	int iOrder = iCamera_Count * 6 + 9;
	_T* H = (_T*)pMalloc(iOrder * iOrder * sizeof(_T)),
		* JtE = (_T*)pMalloc(iOrder * sizeof(_T));

	//备份
	_T K1[4], D1[5], (*pT1)[3 * 4] = (_T(*)[3 * 4])pMalloc(iCamera_Count * 3 * 4 * sizeof(_T));
	//Get_H_JtE<_T>(T, K, D, P, uv,iObservation_Count, iOrder,H, JtE);

	const _T lamda_init = 0.0001, lamda_min = 1e-7;
	_T lambda=lamda_init, * pDiag = (_T*)pMalloc(iOrder * sizeof(_T)),
		*pDiag_Scale = (_T*)pMalloc(iOrder * sizeof(_T)),
		* x = (_T*)pMalloc(iOrder * sizeof(_T));
	
	//Disp(pDiag_Scale, iOrder, 1, "D_Scale");
	int bAccepted;
	const int iMax_Retry = 20;
	for (int iIter = 0; iIter < iMax_Iter; iIter++)
	{
		Get_H_JtE<_T>(T, K, D, P, uv, iObservation_Count, iOrder, H, JtE);
		//Disp(H, iOrder, iOrder, "H");
		Get_Diag(H, iOrder, pDiag);
		
		for (int i = 0; i < iOrder; i++)
			pDiag_Scale[i] = sqrt(pDiag[i] + 1e-6);

		//对H，JtE 做某种意义的归一化
		for (int i = 0; i < iOrder; ++i)
		{
			JtE[i] /= pDiag_Scale[i];
			for (int j = 0; j < iOrder; ++j)
				H[i * iOrder + j] /= (pDiag_Scale[i] * pDiag_Scale[j]);
		}

		Get_Diag(H, iOrder, pDiag);
		bAccepted = 0;
		int iRetry = 0;
		while (!bAccepted && iRetry<iMax_Retry)
		{
			for (int i = 0; i < iOrder; ++i)
				H[i * iOrder + i] = pDiag[i] + lambda; // lambda 初始可以设为 0.001 

			int iResult;
			//Solve_Linear_Gause(H, iOrder, JtE, x, &iResult);
			//由于修改后的H 矩阵无比良性，故此可以大胆用快速解方程
			Solve_Linear_Gause_AAt(H, iOrder, JtE, x, &iResult);
			iRetry++;
			//Disp(x, iOrder, 1, "x");
			if (!iResult)
			{
				lambda *= 10;
				continue;
			}

			//恢复真实解
			for (int i = 0; i < iOrder; i++)
				x[i] = -x[i] / pDiag_Scale[i];

			memcpy(K1, K, 4 * sizeof(_T));
			memcpy(D1, D, 5 * sizeof(_T));
			memcpy(pT1, T, iCamera_Count * 3 * 4 * sizeof(_T));
			Zhang_Update_K_D_T(x, iCamera_Count, K1, D1, pT1);
			//验一下所有数据是否有
			if(!(bIs_Finite(x, iOrder) && bIs_Finite(K1, 4) && bIs_Finite(D1, 5) && bIs_Finite((_T*)pT1, iCamera_Count * 3 * 4)))
			{
				lambda *= 10;
				continue;
			}
	
			fError = Zhang_Get_Error<_T>(K1, D1, pT1, P, uv, iObservation_Count);
			fError_Delta = fPre_Error - fError;
			printf("iter:%d Error:%f average:%f Delta:%f\n", iIter, fError, fError / iObservation_Count, fError_Delta);
			if (fError_Delta < 0)
			{
				lambda *= 10;
				continue;
			}else if(fError_Delta<fLoss_Delta_esp || fError < fLoss_eps)
				iIter = iMax_Iter;

			//已经成功 可以更新J, E, 等了
			memcpy(K, K1, 4 * sizeof(_T));
			memcpy(D, D1, 5 * sizeof(_T));
			memcpy(T, pT1, iCamera_Count * 3 * 4 * sizeof(_T));
			
			bAccepted = 1;
			fPre_Error = fError;
		}

		if (iRetry >= iMax_Retry)
			break;

		if (lambda > lamda_min)
			lambda /= 10;		//lambda 越小越接近高斯牛顿法，越激进
		else
			printf("lambda unchanged\n");

	}
		
	if(pfError)
		*pfError = fError < fPre_Error ? fError :fPre_Error;
	bRet = 1;

	Free(H);
	Free(JtE);
	Free(pT1);
	Free(pDiag);
	Free(pDiag_Scale);
	Free(x);
	return bRet;
}

template<typename _T>int Optimize_1(_T P[][2], _T uv[][2], _T K[4],
	_T D[5], _T T[][3 * 4], int iImage_Count, int iCorner_Per_Image,_T *pfError)
{//有必要尽可能减少Size，习惯用短款K, T
	unsigned int i, j, bRet=0, iPos, iObservation_Count = iCorner_Per_Image * iImage_Count;
	Point_2D<_T>* uv_1 = (Point_2D<_T>*)pMalloc(iObservation_Count * sizeof(Point_2D<_T>));
	for (iPos = i = 0; i < (unsigned int)iImage_Count; i++)
		for (j = 0; j < (unsigned int)iCorner_Per_Image; j++, iPos++)
			uv_1[iPos] = { i,j,uv[iPos][0],uv[iPos][1] };
	
	if (!BA_PnP_Zhang_1<_T>(T, iImage_Count, K, D, P, iCorner_Per_Image,
		uv_1, iObservation_Count,pfError, 0.001, 0.001 * iObservation_Count))
		goto END;

	//printf("%f\n", Zhang_Get_Error<_T>(K, D, T, P, uv_1, iImage_Count * iCorner_Per_Image));
	bRet = 1;
END:
	Free(uv_1);
	return bRet;
}


//template<typename _T>void Optimize(_T P[][2], _T uv[][2], _T K[3 * 3], 
//	_T D[5], _T T[][4 * 4], int iImage_Count, int iCorner_Per_Image)
//{//在此做BA 优化
//	unsigned int i, j, iPos, iObservation_Count = iCorner_Per_Image * iImage_Count;
//	Point_2D<_T>* uv_1 = (Point_2D<_T>*)pMalloc(iObservation_Count * sizeof(Point_2D<_T>));
//	for (iPos = i = 0; i < (unsigned int)iImage_Count; i++)
//		for (j = 0; j < (unsigned int)iCorner_Per_Image; j++, iPos++)
//			uv_1[iPos] = { i,j,uv[iPos][0],uv[iPos][1] };
//	//Disp((_T*)uv, iCorner_Per_Image,  2, "uv");
//	//Disp((_T*)(uv + 88), iCorner_Per_Image, 2, "uv");
//
//	/*BA_PnP_Zhang_1(T, iImage_Count, K, D, P, iCorner_Per_Image,
//		uv_1, iObservation_Count);*/
//
//	//全部内参，4个K + 5个畸变
//	BA_PnP_Zhang_Ceres(T, iImage_Count, K,D, P, iCorner_Per_Image,
//		uv_1, iObservation_Count);
//	return;
//}

template<typename _T>void Verify_T(_T P[][2], _T T[][4 * 4], int iT_Count,int iPoint_Per_T, int *piRemain, _T uv[][2])
{
	
	unsigned char* pMark_Delete = (unsigned char*)pMalloc(iT_Count);
	memset(pMark_Delete, 0, iT_Count);

	for (int i = 0; i < iT_Count; i++)
	{
		_T* T1 = T[i], *pCur = P[0];
		int iNeg_Count = 0;
		for (int j = 0; j < iPoint_Per_T; j++,pCur+=2)
		{
			_T P1[4] = { pCur[0],pCur[1],0,1 }, P2[4];
			Matrix_Multiply(T1, 3, 4, P1, 1, P2);
			if (P2[2] < 0)
				iNeg_Count++;
		}
		if (iNeg_Count == iPoint_Per_T)
		{//全负号，保留，取反即可
			Vector_Multiply<_T>(T1, 4 * 4, -1, T1);
		}else if (iNeg_Count)
		{//邮政有负，不要
			pMark_Delete[i] = 1;
			printf("Image %d removed\n", i);
		}
	}

	int j = 0;
	for (int i = 0; i < iT_Count; i++)
	{
		if (!pMark_Delete[i])
		{
			memcpy(T[j], T[i], 4 * 4 * sizeof(_T));
			memcpy(&uv[j * iPoint_Per_T], &uv[i * iPoint_Per_T], iPoint_Per_T * 2 * sizeof(_T));
			j++;
		}//else	//此位姿与uv 一并删除
	}
	*piRemain = j;
	Free(pMark_Delete);
	return;
}
template<typename _T>int Zhang_1(_T uv[][2], int iImage_Count,
	_T K4[4], _T Distort[5],
	int w_In_Point = 11, int h_In_Point = 8, _T fGrid_Size = 0.02)
{//这个函数结构并不算太好，不清晰，但求快点
	const Normalize_Method iNorm_Method = Normalize_Method::Dev;
	const int bNormalize = 1;

	int iCorner_Per_Image = w_In_Point * h_In_Point, bRet = 0;
	//************一股脑全非陪*********************
	int iSize = ALIGN_SIZE_128(iCorner_Per_Image * 2 * sizeof(_T)) * 2 +	//pCorner_Ref + Norm_Ref
		ALIGN_SIZE_128(iImage_Count * 9 * sizeof(_T)) +				//pH
		ALIGN_SIZE_128(iImage_Count * 16 * sizeof(_T));				//pT

	unsigned char* pBuffer = (unsigned char*)pMalloc(iSize), * p = pBuffer;;
	if (!pBuffer) goto END;

	_T(*pCorner_Ref)[2], (*pNorm_Ref)[2], (*pH)[3 * 3], (*pT)[4 * 4];
	pCorner_Ref = (_T(*)[2])p, p += ALIGN_SIZE_128(iCorner_Per_Image * 2 * sizeof(_T));
	pNorm_Ref = (_T(*)[2])p, p += ALIGN_SIZE_128(iCorner_Per_Image * 2 * sizeof(_T));
	pH = (_T(*)[3 * 3])p, p += ALIGN_SIZE_128(iImage_Count * 9 * sizeof(_T));
	pT = (_T(*)[4 * 4])p;
	//************一股脑全非陪*********************

	//************算H***************************************/
	{
		_T K[4];
		Gen_Corner_Ref(w_In_Point, h_In_Point, fGrid_Size, pCorner_Ref);
		Normalize_2d(pCorner_Ref, iCorner_Per_Image, pNorm_Ref,
			bNormalize ? iNorm_Method : None, K);

		for (int i = 0; i < iImage_Count; i++)
		{//每一张图产生一个H矩阵
			int iPos = i * iCorner_Per_Image;
			int iResult = Estimate_H_Zhang<_T>(pNorm_Ref, uv + iPos, iCorner_Per_Image, pH[i], K, bNormalize, iNorm_Method);
			if (!iResult)
				goto END;
		}
	}

	{//测试代码
		_T fError;
		fError = 0;
		//测试代码，最后注释掉
		_T(*pCorner_Org)[2] = (_T(*)[2])pMalloc(iCorner_Per_Image * 2 * sizeof(_T));
		for (int i = 0; i < iImage_Count; i++)
		{
			int iPos = i * iCorner_Per_Image;
			Gen_Corner_Ref(w_In_Point, h_In_Point, fGrid_Size, pCorner_Org);
			fError += Test_H_2d<_T>(pCorner_Org, uv + iPos, iCorner_Per_Image, pH[i]);
		}
		printf("Error Sum:%f\n", fError);
		Free(pCorner_Org);
	}
	//************算H***************************************/
	_T K[3 * 3];	//至少3张图才能求出一个K,图多多益善
	Cal_K(pH, iImage_Count, K);
	K[1] = 0;	//K并非必须K[1] = 0;	//K并非必须

	//此处用等效焦距来代替标定内参，发现在图不变的情况下，收敛到基本一致的结果
	//Get_K9_by_eq_focal<_T>(35, 4000, 1808, K);
	
	//Disp(K, 3, 3, "K");
		
	Cal_T<_T>(pH, K, iImage_Count, pT);

	//此处要验算位姿，共三种情况，z全正，对。z全负，T取反。正负同时存在，丢弃
	Verify_T<_T>(pCorner_Ref, pT, iImage_Count, iCorner_Per_Image, &iImage_Count, uv);

	//*****************估计畸变参数************************************/
	//非常奇怪，没有参数比又参数误差更小，证明方程太垃圾
	memset(Distort, 0, 5 * sizeof(_T));
	//Estimate_Distort_Coeff_5<_T>(K, pT, pCorner_Ref, uv, iCorner_Per_Image, iImage_Count, Distort);
	//Disp(Distort, 1, 5, "D");
	//*****************估计畸变参数************************************/

	_T fError;
	//iImage_Count = 41;
	fError = Zhang_Get_Error<_T>(K, Distort, pT, pCorner_Ref, uv, iImage_Count, iCorner_Per_Image);
	if (isnan(fError))
		goto END;

	//做短款T
	_T(*pT1)[3 * 4];
	pT1 = (_T(*)[3 * 4])pT;
	for (int i = 0; i < iImage_Count; i++)
		memcpy(pT1[i], pT[i], 3 * 4 * sizeof(_T));
	K9_2_K4(K, K4);

	if (!Optimize_1<_T>(pCorner_Ref, uv, K4, Distort, pT1, iImage_Count, iCorner_Per_Image,&fError))
		goto END;
		
	bRet = 1;
END:
	Free(pBuffer);
	return bRet;
}

void Zhang_Test_1()
{//简单例子，纯吹装入数据，测试张正友算法的收敛
	typedef double _T;
	const int Grid_Size_h = 8, Grid_Size_w = 11;
	const _T Grid_Size = (_T)0.02;

	int iCorner_Count = Grid_Size_w * Grid_Size_h, iImage_Count = 0;
	_T(*pConner_Point_2D)[2];	
	Load_Poine_2D("c:\\tmp\\temp\\corner.bin", &iImage_Count, iCorner_Count, Grid_Size_w, Grid_Size_h, &pConner_Point_2D);
	
	unsigned long long tStart = iGet_Tick_Count();
	_T K4[4], Distort[5];
	Zhang_1<_T>(pConner_Point_2D, iImage_Count,K4,Distort, Grid_Size_w, Grid_Size_h, Grid_Size);
	printf("%lld\n", iGet_Tick_Count() - tStart);
	_T K9[3 * 3];
	K4_2_K9(K4, K9);
	//Disp(K9, 3, 3, "K");
	//Disp(Distort, 1, 5, "D");

	Free(pConner_Point_2D);
	return;
}

template<typename _T>int Find_Chess_Board(char Path[], _T(**ppCorner)[2], int *piImage_Count, const int Grid_Size_w = 11, const int Grid_Size_h = 8, _T Grid_Size = 0.02)
{
	int bRet = 0, iFile_Count,iCorner_Per_Image = Grid_Size_w* Grid_Size_h;
	char Path_1[256], * pFile_List = NULL;
	float(*pCorner)[2] = NULL;
	if (!(pFile_List = pGet_File_List(Path, &iFile_Count, NULL, 0)))
	{
		sprintf(Path_1, "%s\\*.bmp", Path);
		if (!(pFile_List = pGet_File_List(Path_1, &iFile_Count, NULL, 0)))
			return 0;
	}
	if (!(pCorner = (float(*)[2])pMalloc(iFile_Count * iCorner_Per_Image * 2 * sizeof(_T))))
		goto END;

	//开始逐张图进行棋盘检测
	char* pCur;
	int iCount, iResult;
	Image oImage;
	pCur = pFile_List;
	iCount = 0;
	oImage = {};
	for (int i = 0; i < iFile_Count; i++, pCur += strlen(pCur) + 1)
	{
		sprintf(Path_1, "%s\\%s", Path, pCur);
		if (!bLoad_Image(Path_1, &oImage))
			goto END;
		if (iResult = bFind_Chess_Board_2_Step(oImage, pCorner + iCount*iCorner_Per_Image, 1000000))
			iCount++;
		Free_Image(&oImage);
	}

	//至此，已经找完所有角点
	Free(pFile_List), pFile_List = NULL;
	if (!(pCorner = (float(*)[2])pMove_2_Front(pCorner)))
		goto END;
	//Disp((float*)pCorner[3], 88, 2, "Corner");
	if (std::is_same<_T, double>::value)
	{//其他数据类型
		int iValue_Count = iCount * iCorner_Per_Image * 2;
		float* pOrg_Cur = (float*)pCorner;
		_T* pNew_Cur = (_T*)pCorner;
		for (int i = iValue_Count - 1; i >= 0; i--)
			pNew_Cur[i] = pOrg_Cur[i];
	}

	*ppCorner = (_T(*)[2])pCorner;
	*piImage_Count = iCount;
	bRet = 1;
END:
	Free(pFile_List);
	if(!bRet)
		Free(pCorner);
	return bRet;
}

template<typename _T>int Zhang_Calib(const char Path[],_T K[4],_T D[5])
{//做最纯粹地相机标定，求解问题只有一个，相机内参。K：焦距+位移；D：畸变参数
	const int iMax_File_Count = 40, Grid_Size_h = 8, Grid_Size_w = 11;
	const _T Grid_Size = (_T)0.02;
	int iImage_Count = 0, iResult;;
	_T(*pCorner)[2];

	if (!(iResult = Find_Chess_Board<_T>((char*)Path, &pCorner, &iImage_Count, Grid_Size_w, Grid_Size_h, Grid_Size)))
		return 0;
	
	if (!(iResult = Zhang_1<_T>((_T(*)[2])pCorner, iImage_Count,K,D, Grid_Size_w, Grid_Size_h, Grid_Size)))
		return 0;

	Free(pCorner);
	return 1;
}

void Zhang_Test_2()
{//正经地连连上棋盘检测，一气呵成
	typedef double _T;
	_T K4[4] = {}, K9[3 * 3] = {}, D[5] = {};
	if (!Zhang_Calib<_T>("C:\\tmp\\dev_env\\Sample", K4, D))
		return;

	K4_2_K9(K4, K9);
	Disp(K9, 3, 3, "K");
	Disp(D, 1, 5, "Distort");

	return;
}
int Chess_Board_Detect_Main()
{//相当于入口
	/*if (!bInit_Env_CPU(200000000, 128, 97))
		return 0;*/
	//Find_Max_Contour_Test();
	//Find_Outline_Test();
	//Find_Chess_Board_Corner_Test_2();
	//Find_Chess_Board_Corner_Test_3();
	//Find_Chess_Board_Corner_Test_4();
	//Zhang_Test_1();
	Zhang_Test_2();
	//Free_Env_CPU();
	return 0;
}