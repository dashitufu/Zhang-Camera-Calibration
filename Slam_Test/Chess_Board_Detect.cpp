//试一下监测一个棋盘
#include "cmath"	//此处纯粹和opencv对数据而已
#include "Reconstruct.h"
#include "Chess_Board_Detect.h"

#pragma pack(1)
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

	_T(*J)[2][12], (*Residual)[2];
	_T *H,*JtE;	//H矩阵
};
typedef struct Contour_Tree_Node { //树形结构
	int m_iContour_ID;
	int m_iParent;
	int m_iFirst_Child;
	int m_iNext_Sibling;
}Contour_Tree_Node;
typedef struct Start_End {
	int m_iStart, m_iEnd;
}Start_End;
typedef struct Get_Contour_Info {
	typedef struct Contour {
		unsigned int m_iArea:31;		//相当于面积，共有多少个像素
		unsigned int m_iBlack_or_White : 1;	//该连通域什么颜色
		union {
			struct {
				unsigned int m_iPart_Of:24;		//属于哪个Domain
				unsigned int m_iReserve : 7;
			};
			struct {
				//unsigned short m_iReserve;
				unsigned short m_iLine_Count;	//相当于Height
				unsigned short m_iY_Start;		//该Contour的开始屏幕y
				unsigned int m_iFirst_Line:31;		//第二阶段画轮廓用，Top
				//unsigned int m_iBlack_or_White:1;	//黑白
			};
		};
	}Contour;

	typedef struct Strip {	//一个扫描行中的连续线段
		//union {
			struct {
				unsigned short m_iStart;		//线段的开始位置
				unsigned short m_iEnd;			//线段的结束位置，为有效数据而非下一个可用位置
				unsigned int m_iContour_ID:24;	//属于哪个连通域
				unsigned int m_iBlack_or_White : 1;		//该strip是黑还是白
				//unsigned int m_bVisited;		//初始化为0
			};
			unsigned short m_iY;			//第几行
		//};
	}Strip;
	typedef struct Line {
		unsigned int m_iFirst_Strip;		//一行当中第一个Strip的索引
		unsigned short m_iStrip_Count;		//该行一共有多少个Strip,最多256个
	}Line;
	Strip* m_pStrip;
	Line* m_pLine;			//一共有iHeight条线
	Contour* m_pContour;
	//unsigned char* m_pBuffer;	//所有分配的空间在此
	int m_iContour_Count;		//当前一共有多少个连通域
	int m_iStrip_Count;			//下一个可用Strip的ID
	int m_iMax_Strip_Count;		//一共可以有多少个Strip
	int m_iMax_Domain_Count;		//最多可以容纳多少个Domain
	unsigned short m_iWidth, m_iHeight;	//图像的长款
}Get_Contour_Info;

typedef struct Hash_Item {
	unsigned int A;
	unsigned int B;	//为减少数量，定义A<B
	//unsigned int m_iNeighbour_Count : 1;
	unsigned int m_iNext;
}Hash_Item;

typedef struct Neighbour {
	int m_iContour_ID;
	int m_iNext;
}Neighbour;

typedef struct Start_Count {
	int m_iContour_ID;
	int m_iStart;
	int m_iCount;
}Start_Count;

//****************************这堆要废弃***************************************
typedef struct Quad_Neighbour_Item {
	short m_iQuad_Index;
	short m_iCorner_Index;
	float m_fDist;
	//short m_iGroup_Index;	//缺省为-1，归入哪个组
}Quad_Neighbour_Item;

typedef struct Chess_Board_Corner {
	float Corner_f[2];
	unsigned int row;
	unsigned char m_iCount;
	Chess_Board_Corner* Neighbour[4];	//一个点与其他Quad的顶点形成的关系
}Chess_Board_Corner;

typedef struct Chess_Board_Quad {	//棋盘专用四元组
	union {
		float Corner_f[4][2];
		unsigned short Corner_i[4][2];	//4个角点
		Chess_Board_Corner Corner[4];
	};

	//int Neighbour[4];				//可以有4个邻居
	Quad_Neighbour_Item Neighbour[4];
	unsigned char m_iCount;			//邻居数
	unsigned char ordered;			
	short m_iGroup_Index;	//缺省为-1，归入哪个组
	short row, col;

	float edge_sqr_len;				//边长平方
}Chess_Board_Quad;
//****************************这堆要废弃***************************************

//重做一堆结构，前面理解太肤浅
typedef struct Quad_Neighbour_Item_1 {
	short m_iQuad_Index;
	short m_iCorner_Index;
	float m_fDist;
	void* ptr;				//该Neighbour的指针，可以直接访问过去
	//short m_iGroup_Index;	//缺省为-1，归入哪个组
}Quad_Neighbour_Item_1;

typedef struct Chess_Board_Corner_1 {
	float Pos[2];
	unsigned int row;
	unsigned char m_iCount;
	Chess_Board_Corner_1* Neighbour[4];	//一个点与其他Quad的顶点形成的关系
}Chess_Board_Corner_1;

typedef struct Chess_Board_Quad_1 {	//棋盘专用四元组
	union {
		unsigned short Corner_i[4][2];	//4个角点
		Chess_Board_Corner_1* Corner[4];
	};

	//int Neighbour[4];				//可以有4个邻居
	Quad_Neighbour_Item_1 Neighbour[4];
	unsigned char m_iNeighbour_Count;			//邻居数
	unsigned char ordered;			
	short m_iGroup_Index;	//缺省为-1，归入哪个组
	short row, col;

	float edge_sqr_len;				//边长平方
}Chess_Board_Quad_1;

#pragma pack()

void Free_Contour_Result(Get_Contour_Result* poResult)
{
	Get_Contour_Result oResult = *poResult;
	if (oResult.m_pAll_Point)
		Free(oResult.m_pAll_Point);
	if (oResult.m_pContour)
		Free(oResult.m_pContour);
	*poResult = {};
}
static void Draw_Contour_Outline(char* pcFile, unsigned short Point[][2], int iPoint_Count)
{//根据一组点画出个抡锅
	Image oImage;
	
	unsigned short Bounding_Box[2][2];
	Get_Bounding_Box(Point, iPoint_Count, Bounding_Box);
	Init_Image(&oImage, Bounding_Box[1][0]-Bounding_Box[0][0]+2, Bounding_Box[1][1]-Bounding_Box[0][1]+2, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (int i = 0; i < iPoint_Count; i++)
	{
		//Draw_Point(oImage, Point[i][0]-Bounding_Box[0][0]+1, Point[i][1]-Bounding_Box[0][0]+1);
		Mid_Point_Line(oImage, Point[i][0] - Bounding_Box[0][0] + 1, Point[i][1] - Bounding_Box[0][0] + 1,
			Point[(i+1)%iPoint_Count][0] - Bounding_Box[0][0] + 1, Point[(i+1)%iPoint_Count][1] - Bounding_Box[0][0] + 1);
	}
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
	return;	
}
void Disp_Quad(Chess_Board_Quad oQuad,int bi16=1)
{
	if (bi16)
		Disp((unsigned short*)oQuad.Corner_i, 4, 2, "Corner");
	else
		Disp((float*)oQuad.Corner_f, 4, 2, "Corner");

	for (int i = 0; i < 4; i++)
		printf("%d\n", oQuad.Neighbour[i].m_iQuad_Index);
	//Disp(oQuad.Neighbour, 4, 1, "Neighbour");
	printf("Count:%d edge_sqr_len:%f\n", oQuad.m_iCount,oQuad.edge_sqr_len);
	printf("\n");

	return;
}
void Disp_Quads(Chess_Board_Quad* pQuad, int iQuad_Count,int bi16=1)
{
	for (int i = 0; i < iQuad_Count; i++)
		Disp_Quad(pQuad[i],bi16);
	return;
}
void Disp_Quad(Chess_Board_Quad_1 oQuad,int bi16=1,int iIndex=-1)
{
	//printf("Corner:%d\n",iIndex);
	if (bi16)
	{
		for (int i = 0; i < 4; i++)
		{
			unsigned short* pPos = oQuad.Corner_i[i];;
			printf("%d %d\n", pPos[0], pPos[1]);
		}
	}else
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
		if(oQuad.Neighbour[i].m_iQuad_Index==-1)
			printf("%d\n", oQuad.Neighbour[i].m_iQuad_Index);
		else
		{
			for (int j = 0; j < 4; j++)
			{
				Chess_Board_Quad_1* poNeighbour =(Chess_Board_Quad_1*)oQuad.Neighbour[i].ptr;
				printf("\t%f %f\n", poNeighbour->Corner[j]->Pos[0],
					poNeighbour->Corner[j]->Pos[1]);
			}
		}
	}
		
	//Disp(oQuad.Neighbour, 4, 1, "Neighbour");
	printf("Count:%d edge_sqr_len:%f ordered:%d\n", oQuad.m_iNeighbour_Count,oQuad.edge_sqr_len,oQuad.ordered);
	printf("\n");

	return;
}

void Disp_Quads(Chess_Board_Quad_1* pQuad, int iQuad_Count,int bi16=1)
{
	for (int i = 0; i < iQuad_Count; i++)
	{
		printf("Contour:%d\n", i);
		Disp_Quad(pQuad[i],bi16,i);
	}		
	return;
}
void Disp_Quads(Chess_Board_Quad_1* Quad[], int iQuad_Count, int bi16 = 1)
{
	for (int i = 0; i < iQuad_Count; i++)
	{
		printf("Contour:%d\n", i);
		Disp_Quad(*Quad[i],bi16,i);
	}

	return;
}
static void Draw_Contour_Outline(char* pcFile, Get_Contour_Info oInfo, unsigned short Point[][2], int iPoint_Count)
{
	Image oImage;
	Init_Image(&oImage, oInfo.m_iWidth, oInfo.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (int i = 0; i < iPoint_Count; i++)
		Draw_Arc(oImage, 10, Point[i][0], Point[i][1]);

	//oImage.m_pChannel[0][Point[i][1] * oImage.m_iWidth + Point[i][0]] = 0xFF;
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
	return;
}

void Draw_Contour_Outline(char* pcFile, Get_Contour_Result oResult, int iContour_ID)
{//通过Result结构画轮廓
	Image oImage;
	Init_Image(&oImage, oResult.m_iWidth, oResult.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	unsigned short (*pPoint)[2] = oResult.m_pContour[iContour_ID].m_pPoint;
	int i,iPoint_Count = oResult.m_pContour[iContour_ID].m_iOutline_Point_Count;
	for (i = 0; i < iPoint_Count; i++)
		Draw_Arc(oImage, 10, pPoint[i][0], pPoint[i][1]);
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
}
static int bInit_Get_Contour_Info(Get_Contour_Info* poInfo, int iMax_Strip_Count, int iMax_Contour_Count, int iWidth,int iHeight)
{//初始化部分信息，Strip在这里开辟
	//unsigned char* pBuffer;
	poInfo->m_iMax_Strip_Count = iMax_Strip_Count;
	poInfo->m_iMax_Domain_Count = iMax_Contour_Count;
	poInfo->m_iContour_Count = 0;
	poInfo->m_iStrip_Count = 0;
	poInfo->m_iWidth = iWidth;
	poInfo->m_iHeight = iHeight;

	int iStrip_Count = 0;
	return 1;
}

static  int bGet_Strip(Get_Contour_Info *poInfo, Image oAlpha)
{//扫描整个图片，将所有的Strip找出来
	Get_Contour_Info oInfo = *poInfo;
	Get_Contour_Info::Line oCur_Line;
	Get_Contour_Info::Strip oStrip;

	const unsigned long long iPattern_White = { 0xFFFFFFFFFFFFFFFF },
		iPattern_Black = 0;
	oInfo.m_pStrip = (Get_Contour_Info::Strip*)pMalloc(oInfo.m_iMax_Strip_Count * sizeof(Get_Contour_Info::Strip));
	oInfo.m_pLine = (Get_Contour_Info::Line*)pMalloc(oInfo.m_iHeight * sizeof(Get_Contour_Info::Line));
	if (!oInfo.m_pStrip || !oInfo.m_pLine)
	{
		printf("bGet_Strip Error, Fail to allocated memeory\n");
		if (oInfo.m_pStrip)Free(oInfo.m_pStrip);
		if (oInfo.m_pLine)Free(oInfo.m_pLine);
		return 0;
	}
	

	unsigned char* pCur, * pCur_End;
	int y, x,iStrip_Count,iCur_Strip_Of_Previous_Line;
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
			while(pCur_1 < pCur_End && iFlag_1 == *(unsigned long long*)pCur_1)
				pCur_1 += 8;
			//**********这段在复杂纹理中没啥用，但在抠图中能加快速度**********

			while ( !!(*pCur_1)==iFlag && pCur_1<pCur_End+8)
				pCur_1++;

			oStrip.m_iStart = x;
			oStrip.m_iEnd = (unsigned short)(pCur_1 - pCur-1);
			oStrip.m_iContour_ID = 0xFFFFFF;
			oStrip.m_iY = y;	//省不了，因为还有后面的勾边
			oStrip.m_iBlack_or_White = iFlag;
			oInfo.m_pStrip[oInfo.m_iStrip_Count++] = oStrip;

			if (oInfo.m_iStrip_Count >= oInfo.m_iMax_Strip_Count)
			{
				printf("Insufficient strip allocated\n");
				return 0;
			}
			x = oStrip.m_iEnd+1;
		}
		oCur_Line.m_iStrip_Count = oInfo.m_iStrip_Count - oCur_Line.m_iFirst_Strip;
		oInfo.m_pLine[y] = oCur_Line;
	}

	Shrink(oInfo.m_pStrip, oInfo.m_iStrip_Count * sizeof(Get_Contour_Info::Strip));
	*poInfo = oInfo;
	
	return 1;
}
static  void Free_Contour_Info(Get_Contour_Info* poInfo)
{
	Get_Contour_Info oInfo = *poInfo;
	if (oInfo.m_pContour)
		Free(oInfo.m_pContour), oInfo.m_pContour = NULL;
	if (oInfo.m_pLine)
		Free(oInfo.m_pLine), oInfo.m_pLine = NULL;
	if (oInfo.m_pStrip)
		Free(oInfo.m_pStrip), oInfo.m_pStrip = NULL;
	oInfo.m_iContour_Count = 0;
	oInfo.m_iStrip_Count = 0;
	*poInfo = oInfo;
}

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
	}else
	{//黑色4连通域
		if (oA.m_iEnd < oB.m_iStart)
			return 0;
		else if (oB.m_iEnd < oA.m_iStart)
			return 0;
	}
	return 1;
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
		if(bConnect(oStrip, oPrevious_Strip))
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
		}else
		{//分开黑白两种情况
			if (oStrip.m_iBlack_or_White)
			{
				if (oPrevious_Strip.m_iEnd>= oStrip.m_iEnd+1)
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
			oStrip_More = oInfo.m_pStrip[oPrevious_Line.m_iFirst_Strip+i];
			if (iResult=bConnect(oStrip, oStrip_More))
			{//可以向上修改
				int iCur = oStrip_More.m_iContour_ID;
				if (!bFind_Node(oInfo.m_pContour, iCur, iRoot))
					Adjust_Link(oInfo.m_pContour, iCur, iRoot);					
			}

			//分黑白两种情况判断跳出条件
			if (oStrip.m_iBlack_or_White)
			{
				if (oStrip_More.m_iEnd>= oStrip.m_iEnd+1)
					break;
			}else if (oStrip_More.m_iEnd >= oStrip.m_iEnd)
				break;

			//原来的方法不再适用，因为黑白的连通数不一样
			//if (oStrip_More.m_iEnd >= oStrip.m_iEnd)
			//break;
		}
	}	
	*piCur_Strip_Of_Previous_Line = i;
	return iRoot;
}
static void Adjust_Contour(Get_Contour_Info *poInfo)
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
	int* pMap= (int*)pMalloc(oInfo.m_iContour_Count * sizeof(int));
	int j;
	for (i=j = 0; i < oInfo.m_iContour_Count; i++)
	{
		if (oInfo.m_pContour[i].m_iArea)
		{
			oInfo.m_pContour[j] = oInfo.m_pContour[i];
			pMap[i] = j++;
		}else
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
static void Get_Contour(Get_Contour_Info* poInfo,int iHeight)
{//此处根据strip生成所有的连通域
	Get_Contour_Info oInfo = *poInfo;
	Get_Contour_Info::Line oLine,oPrevious_Line = { 0 };
	Get_Contour_Info::Strip oStrip;
	Get_Contour_Info::Contour oContour;  //= { 0,-1 };;
	int y,i,iCur_Strip_Of_Previous_Line;
	oContour.m_iArea = 0;
	oContour.m_iPart_Of = 0xFFFFFF;
	oInfo.m_iMax_Domain_Count = oInfo.m_iStrip_Count;
	oInfo.m_pContour = (Get_Contour_Info::Contour*)pMalloc(oInfo.m_iMax_Domain_Count  * sizeof(Get_Contour_Info::Contour));
	if (!oInfo.m_pContour)
	{
		printf("Fail to allocate memory in Get_Contour\n");
		return;
	}
	memset(oInfo.m_pContour, 0, oInfo.m_iMax_Domain_Count * sizeof(Get_Contour_Info::Contour));
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
					printf("Too many domain");
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
	Adjust_Contour(&oInfo);	
END:
	*poInfo = oInfo;
	return;
}
static int iGet_Max_Contour(Get_Contour_Result oResult, int iBlack_or_White = -1)
{
	int iMax = 0, iMax_Contour=-1;
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
			if (oResult.m_pContour[i].m_iOutline_Point_Count > iMax && oResult.m_pContour[i].m_iBlack_or_White==iBlack_or_White)
			{
				iMax = oResult.m_pContour[i].m_iArea;
				iMax_Contour = i;
			}
		}
	}
	//printf("Max Contour:%d Size:%d\n",iMax_Contour, iMax);
	return iMax_Contour;
}
int iGet_Max_Contour(Get_Contour_Info oInfo,int iBlack_or_White=1)
{
	int iMax = 0, iMax_Contour=-1;
	for (int i = 0; i < oInfo.m_iContour_Count; i++)
	{
		if (oInfo.m_pContour[i].m_iArea > (unsigned int)iMax && oInfo.m_pContour[i].m_iBlack_or_White==iBlack_or_White)
		{
			iMax = oInfo.m_pContour[i].m_iArea;
			iMax_Contour = i;
		}
	}
	printf("Max Contour:%d Size:%d\n",iMax_Contour, iMax);
	return iMax_Contour;
}
void Draw_Contour_Step_1(const char* pcFile, Get_Contour_Info oInfo, int iContour_ID)
{//第一阶段只抠出Contour，尚未形成Contour->Line->Strip索引可以用这 个函数
	Get_Contour_Info::Strip oStrip;
	Get_Contour_Info::Line oLine;
	Image oImage;
	int y, i, j;
	Init_Image(&oImage, oInfo.m_iWidth, oInfo.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (y = 0; y < oInfo.m_iHeight; y++)
	{
		oLine = oInfo.m_pLine[y];
		for (i = 0; i < oLine.m_iStrip_Count; i++)
		{
			oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			if (oStrip.m_iContour_ID == iContour_ID)
			{
				for (j = oStrip.m_iStart; j <= oStrip.m_iEnd; j++)
					oImage.m_pChannel[0][y * oImage.m_iWidth + j] = 255;
			}
		}
	}
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
}
void Draw_Contour_Step_2(const char *pcFile,Get_Contour_Info oInfo,int iContour_ID)
{//这是给第二阶段做了完整的索引，形成Contour, Line, Strip的完整指向才用
	Get_Contour_Info::Contour oContour = oInfo.m_pContour[iContour_ID];
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Strip oStrip;
	Image oImage;
	int i, j,k;
	Init_Image(&oImage, oInfo.m_iWidth, oInfo.m_iHeight, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);

	for (i = 0; i < oContour.m_iLine_Count; i++)
	{
		oLine = oInfo.m_pLine[oContour.m_iFirst_Line + i];
		for (j = 0; j < oLine.m_iStrip_Count; j++)
		{
			oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + j];
			for (k = oStrip.m_iStart; k <= oStrip.m_iEnd; k++)
				oImage.m_pChannel[0][oStrip.m_iY * oImage.m_iWidth + k] = 255;
		}
	}
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
}
void Test_Contour(Get_Contour_Info oInfo, Get_Contour_Info::Contour* pTemp_Contour)
{//测试一下第二阶段画轮廓的预数据
	int i,j,k,iCount;
	Get_Contour_Info::Contour oContour;
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Strip oStrip;
	for (i = 0; i < oInfo.m_iContour_Count; i++)
	{
		iCount = 0;
		oContour = oInfo.m_pContour[i];
		for (j = 0; j < oContour.m_iLine_Count; j++)
		{
			oLine = oInfo.m_pLine[oContour.m_iFirst_Line + j];
			for (k = 0; k < oLine.m_iStrip_Count; k++)
			{
				oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + k];
				iCount += oStrip.m_iEnd - oStrip.m_iStart + 1;

				if (k > 0 && oStrip.m_iY != oInfo.m_pStrip[oLine.m_iFirst_Strip + k - 1].m_iY)
					printf("Error");	//两个strip不在同一行
			}			
		}
		if (pTemp_Contour[i].m_iArea != iCount)
			printf("error");
	}

	return;
}

static void Prepare_Outline(Get_Contour_Info* poInfo)
{
	Get_Contour_Info oInfo = *poInfo;
	//对strip扫描一遍寻找每个contour的行开始与行结束，多少个Strip
	Get_Contour_Info::Strip oStrip;
	Get_Contour_Info::Contour oContour;
	//[0]: Top	[1]: Bottom		[2]: Strip Count
	unsigned int *pCur_Contour, (*pContour_Strip)[3] = (unsigned int(*)[3])pMalloc(oInfo.m_iContour_Count * 3 * sizeof(unsigned int));
	int i;

	//测试用
	//Get_Contour_Info::Contour* pTemp_Contour = (Get_Contour_Info::Contour*)pMalloc(oInfo.m_iContour_Count * sizeof(Get_Contour_Info::Contour));
	//memcpy(pTemp_Contour, oInfo.m_pContour, oInfo.m_iContour_Count * sizeof(Get_Contour_Info::Contour));
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
		/*if (oStrip.m_iY >  pCur_Contour[1])
			pCur_Contour[1] = oStrip.m_iY;
		if (oStrip.m_iY < pCur_Contour[0])
			pCur_Contour[0] = oStrip.m_iY;*/

		//以下这样行不行，还得看最后结果，但上面真心很傻逼
		pCur_Contour[1] = oStrip.m_iY;
		if (pCur_Contour[0] == 0xFFFFFFFF)
			pCur_Contour[0] = oStrip.m_iY;

		pCur_Contour[2]++;
	}

	//确定所有Contour要多少行信息
	int iLine_Count = 0;
	for (i = 0; i < oInfo.m_iContour_Count; i++)
	{
		Get_Contour_Info::Contour *poContour = &oInfo.m_pContour[i];
		pCur_Contour = pContour_Strip[i];
		poContour->m_iLine_Count = pCur_Contour[1] - pCur_Contour[0]+1;
		poContour->m_iFirst_Line = iLine_Count;
		poContour->m_iY_Start = pCur_Contour[0];
		iLine_Count += poContour->m_iLine_Count;
	}

	////建立一个Contour->Line表, [Contour]可得到两个信息 [0]:行开始，[1]行数
	Get_Contour_Info::Line* pLine_Strip_Map = (Get_Contour_Info::Line*)pMalloc(iLine_Count * sizeof(Get_Contour_Info::Line));
	memset(pLine_Strip_Map, 0, iLine_Count * sizeof(Get_Contour_Info::Line));
		
	for (i = 0; i < oInfo.m_iStrip_Count; i++)
	{
		oStrip = oInfo.m_pStrip[i];
		oContour = oInfo.m_pContour[oStrip.m_iContour_ID];
		//算从当前strip到当前contour的顶的相对位置
		int iOffset = oStrip.m_iY - pContour_Strip[oStrip.m_iContour_ID][0];
		pLine_Strip_Map[oContour.m_iFirst_Line+iOffset].m_iStrip_Count++;
	}

	int iStrip_Count = 0;
	for (i = 1; i < iLine_Count; i++)
	{
		iStrip_Count += pLine_Strip_Map[i - 1].m_iStrip_Count;
		pLine_Strip_Map[i].m_iFirst_Strip = iStrip_Count;

		pLine_Strip_Map[i - 1].m_iStrip_Count = 0;	//后i面有用，所以又重新置零，很傻但必须
	}
	
	Get_Contour_Info::Strip* pNew_Strip = (Get_Contour_Info::Strip*)pMalloc(oInfo.m_iStrip_Count * sizeof(Get_Contour_Info::Strip));
	memset(pNew_Strip, 0, oInfo.m_iStrip_Count * sizeof(Get_Contour_Info::Strip));
	
	pLine_Strip_Map[i - 1].m_iStrip_Count = 0;
	for (i = 0; i < oInfo.m_iStrip_Count; i++)
	{
		oStrip = oInfo.m_pStrip[i];
		oContour = oInfo.m_pContour[oStrip.m_iContour_ID];
		int iOffset = oStrip.m_iY - pContour_Strip[oStrip.m_iContour_ID][0];
		Get_Contour_Info::Line *poLine = &pLine_Strip_Map[oContour.m_iFirst_Line + iOffset];
		pNew_Strip[poLine->m_iFirst_Strip + poLine->m_iStrip_Count++] = oStrip;
	}

	Free(pContour_Strip);
	Free(oInfo.m_pStrip);
	Free(oInfo.m_pLine);
	oInfo.m_pStrip = pNew_Strip;
	oInfo.m_pLine = pLine_Strip_Map;

	//测试代码
	//Test_Contour(oInfo,pTemp_Contour);
	//Free(pTemp_Contour);
	*poInfo = oInfo;
	return;
}

static void Outline_Start_White(Get_Contour_Info::Line* pLine, Get_Contour_Info::Strip* pStrip,
	int* pbRepeat_Visist,unsigned short Last_Point[2])
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
		if (oStrip_1.m_iEnd+1 == oStrip_0.m_iStart /*&& oStrip_0.m_iEnd>oStrip_0.m_iStart*/)
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
	}else
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
	int i,k,iDir = *piDir,iDir_1, bFound=0;
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
						if (oStrip_1.m_iEnd < x )
						{//有物阻挡
							if (oStrip_1.m_iEnd + 1 >= oStrip_0.m_iStart)
							{
								x1 = oStrip_1.m_iEnd+1;
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
		}else
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
						iDir_1 = iDir_of_Point_White(oStrip_1,iDir, x, &x1, 0);
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
				iDir = (iDir+1)%8;
			}else if (iDir == 1 || iDir == 2 || iDir == 3)
			{//三个都是向下找
				if (iCur_Line < oContour.m_iLine_Count - 1)
				{
					oLine = pLine[iCur_Line + 1];
					for (i = oLine.m_iStrip_Count - 1; i>=0; i--)
					{
						oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
						iDir_1 = iDir_of_Point_White(oStrip_1,iDir, x, &x1, 1);
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
						*piCur_Line = iCur_Line+1;
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

	Point[0][0]= iStart_x = oStrip.m_iStart;
	Point[0][1]=iStart_y = oStrip.m_iY;
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

	}else if (oContour.m_iArea == 2)
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
		if(oStrip_1.m_iEnd==oStrip_0.m_iStart)
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
	int i,k,iDir = *piDir, bFound=0;
	int x = *px;
	int iCur_Line = *piCur_Line;
	for (k = 0; k < 4; k++)
	{//最多需按照一周4个方向
		if (iDir == 0)
		{//向右挺进
			if (x == oStrip_0.m_iEnd)
			{//本来就走到尽头了,继续转一格
				iDir = (iDir + 1) % 4;
			}else if (iCur_Line)
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
			}else
			{//向右到最后
				*px = oStrip_0.m_iEnd;
				*piDir = 3;
				return 1;
			}
		}else if (iDir == 1)
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
			iDir = (iDir+1)%4;
		}else if (iDir == 2)
		{//向右
			if (x == oStrip_0.m_iStart)
			{//本来就走到尽头了,继续转一格
				iDir = (iDir + 1) % 4;
			}else
			{
				if (iCur_Line < oContour.m_iLine_Count - 1)
				{
					bFound = 0;
					oLine = pLine[iCur_Line + 1];
					for (i = oLine.m_iStrip_Count - 1; i >= 0; i--)
					{
						oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
						if (oStrip_1.m_iEnd < x )
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
		}else if (iDir == 3)
		{//向上找
			if (iCur_Line)
			{//有上边
				bFound = 0;
				oLine = pLine[iCur_Line - 1];
				for (i = 0; i < oLine.m_iStrip_Count; i++)
				{
					oStrip_1 = pStrip[oLine.m_iFirst_Strip + i];
					if ( oStrip_1.m_iEnd >= x)
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
			iDir = (iDir+1)%4;
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
	Point[0][0]= iStart_x = oStrip.m_iStart;
	Point[0][1]=iStart_y = oStrip.m_iY;
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

static void Gen_Contour_Outline(Get_Contour_Info oInfo, int iContour_ID, int *piPoint_Count, int iMax_Point_Count=1000000,unsigned short Point[][2]=NULL)
{
	Get_Contour_Info::Contour oContour = oInfo.m_pContour[iContour_ID];
	Get_Contour_Info::Line* pLine = &oInfo.m_pLine[oContour.m_iFirst_Line];
	int iPoint_Count = 0;
	unsigned short (*pPoint)[2];

	if (Point)
		pPoint = Point;
	else
		pPoint= (unsigned short (*)[2])pMalloc(iMax_Point_Count * 2 * sizeof(unsigned short));

	if(oContour.m_iBlack_or_White)
		Gen_Outline_White(oContour,pLine , oInfo.m_pStrip,pPoint,iMax_Point_Count,&iPoint_Count);
	else
		Gen_Outline_Black(oContour,pLine , oInfo.m_pStrip,pPoint,iMax_Point_Count,&iPoint_Count);

	if(!Point)
		Free(pPoint);
	*piPoint_Count = iPoint_Count;
	return;
}
static void Shrink_Point(unsigned short Point[][2], int* piPoint_Count)
{
	int i,j,iPoint_Count = *piPoint_Count;
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
	}else
	{
		fPre_Grad = (float)(Point[1][1] - Point[0][1]) / (Point[1][0] - Point[0][0]);
		if (fPre_Grad == 0)
			fPre_Grad = (float)(Point[1][0] > Point[0][0] ? MIN_FLOAT : -MIN_FLOAT);
	}
	
	for (i = 2,j=1; i < iPoint_Count; i++)
	{
		/*if (i == iPoint_Count-1)
			printf("here");*/
		if (Point[i][0] == Point[i - 1][0])
			fGrad =Point[i][1] > Point[i-1][1] ? MAX_FLOAT: -MAX_FLOAT;
		else
		{
			fGrad = (float)(Point[i][1] - Point[i - 1][1]) / (Point[i][0] - Point[i - 1][0]);
			if (fGrad == 0)
				fGrad =  (float)(Point[i][0] > Point[i - 1][0] ? MIN_FLOAT : -MIN_FLOAT);
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
			fGrad =Point[0][1] > Point[j][1] ? MAX_FLOAT: -MAX_FLOAT;
			//fGrad = MAX_FLOAT;
		else
		{
			fGrad = (float)(Point[j][1] - Point[0][1]) / (Point[j][0] - Point[0][0]);
			if(fGrad==0)
				fGrad =  (float)(Point[0][0] > Point[j][0] ? MIN_FLOAT : -MIN_FLOAT);
		}
		if (fGrad == fPre_Grad)
			j--;
	}
	*piPoint_Count = j+1;
	return;
}
void Disp_Contour(Get_Contour_Info oInfo,int iContour_ID)
{
	Get_Contour_Info::Contour oContour =oInfo.m_pContour[iContour_ID];
	Get_Contour_Info::Line oLine;
	Get_Contour_Info::Strip oStrip;
	int i, j;

	for (i = 0; i < oContour.m_iLine_Count; i++)
	{
		oLine = oInfo.m_pLine[oContour.m_iFirst_Line + i];
		printf("Line Pos:%d\n", oContour.m_iFirst_Line + i);
		for (j = 0; j < oLine.m_iStrip_Count; j++)
		{
			oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + j];
			printf("\tStrip Pos:%d Start:%d End:%d y:%d\n", oLine.m_iFirst_Strip + j,
				oStrip.m_iStart,oStrip.m_iEnd,oStrip.m_iY);
		}
	}
	return;
}
static void Gen_Outline(Get_Contour_Info oInfo,Get_Contour_Result *poResult,const int iMax_Point_Count = 3000000)
{//尝试逐个生成轮廓
	int i;
	Get_Contour_Info::Contour oContour;	
	Get_Contour_Result oResult = *poResult;
	//unsigned short (*pPoint)[2] = (unsigned short(*)[2])pMalloc(iMax_Point_Count * 2 * sizeof(unsigned short));
	oResult.m_pAll_Point = (unsigned short(*)[2])pMalloc(iMax_Point_Count * 2 * sizeof(unsigned short));
	unsigned short (*pPoint)[2] = oResult.m_pAll_Point;
	int iPoint_Count_Remain=iMax_Point_Count, iPoint_Count;
	int iMin_Strip_Count = 0xFFFFFFF,
		iMin_Line_Count = 0xFFFFFFF;

	//oResult.m_iNon_Root_Max_Area = 0;
	for (i = 0; i < oInfo.m_iContour_Count; i++)
	{
		int bBreak = 0;
		oContour = oInfo.m_pContour[i];

		/*if (oContour.m_iBlack_or_White)
			continue;*/
		/*if (i == 135816)
			printf("here");*/

		//由于连通数不一样，分黑白两种方式进行勾边
		if (oContour.m_iBlack_or_White)
			Gen_Outline_White(oContour, &oInfo.m_pLine[oContour.m_iFirst_Line], oInfo.m_pStrip, pPoint, iPoint_Count_Remain, &iPoint_Count, bBreak);
		else
			Gen_Outline_Black(oContour, &oInfo.m_pLine[oContour.m_iFirst_Line], oInfo.m_pStrip, pPoint, iPoint_Count_Remain, &iPoint_Count, bBreak);

		Get_Contour_Info::Line oLine = oInfo.m_pLine[oContour.m_iFirst_Line];
		Get_Contour_Info::Strip oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip];

		if (iPoint_Count>=iPoint_Count_Remain)
		{//超出最大点数
			printf("Insufficient memory\n");
			oResult.m_iMax_Point_Count = 0;
			oResult.m_pAll_Point = NULL;	//调用程序可通过这个判断
			break;
		}else
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
	
	return;
}
static int bInit_Get_Contour_Result(Get_Contour_Info oInfo,Get_Contour_Result* poResult)
{
	Get_Contour_Result oResult = { 0 };
	oResult.m_pContour = (Get_Contour_Result::Contour*)pMalloc(oInfo.m_iContour_Count * sizeof(Get_Contour_Result::Contour));
	oResult.m_iContour_Count = oInfo.m_iContour_Count;
	oResult.m_iWidth = oInfo.m_iWidth, oResult.m_iHeight = oInfo.m_iHeight;

	*poResult = oResult;
	if (!oResult.m_pContour /*|| !oResult.m_pAll_Point*/)
	{
		if (oResult.m_pContour)
			Free(oResult.m_pContour);
		return 0;
	}else
		return 1;
}

#define Add_To_Hash_Table_1(Hash_Table, Item, iHash_Size, iCur_Item, A1, B1) \
{	\
	int iPos, iHash_Pos;	\
	unsigned int A=A1, B=B1;			\
	Hash_Item oItem_Exist;	\
							\
	if (A > B)				\
		std::swap(A, B);	\
							\
	iHash_Pos = ((A<<4)^B)  % iHash_Size;		\
	if ( (iPos = Hash_Table[iHash_Pos])!=0)		\
	{											\
		do {									\
			oItem_Exist = Item[iPos];			\
			if (oItem_Exist.A == A && oItem_Exist.B==B)	\
			{											\
				goto END;								\
			}											\
		} while (iPos = oItem_Exist.m_iNext);			\
	}													\
	if (iCur_Item < iHash_Size)							\
	{	\
		Item[iCur_Item] = { A,B,(unsigned int)Hash_Table[iHash_Pos] };	\
		Hash_Table[iHash_Pos] = iCur_Item++;			\
	}else \
		printf("Insufficient memory");					\
END:													\
	;													\
}

static void Add_To_Hash_Table(int Hash_Table[], Hash_Item* Item, int iHash_Size, int *piCur_Item, unsigned int A, unsigned int B)
{//加入散列表
	unsigned long long iPos;
	int iHash_Pos, iCur_Item = *piCur_Item;
	Hash_Item oItem_Exist;

	if (A > B)
		std::swap(A, B);
	/*if (A == 8 && B == 32)
		printf("Here");*/
	iHash_Pos = ((A<<4)^B)  % iHash_Size;
	if ( (iPos = Hash_Table[iHash_Pos])!=0)
	{//散列表有东西
		do {
			oItem_Exist = Item[iPos];
			if (oItem_Exist.A == A && oItem_Exist.B==B)
			{//重复了，此点不加入散列表
				*piCur_Item = iCur_Item;
				return;
			}
		}while (iPos = oItem_Exist.m_iNext);
	}

	if (iCur_Item < iHash_Size)
	{
		Item[iCur_Item] = { A,B,(unsigned int)Hash_Table[iHash_Pos] };
		Hash_Table[iHash_Pos] = iCur_Item++;
	}else
		printf("Insufficient memory");

	*piCur_Item = iCur_Item;
}

void Test_Hash_Table(int Hash_Table[], Hash_Item Item[], int iHash_Size)
{//测试用而已，看看有没有太长的链接
	int iMax_Chain = 0, iMax_Pos;
	int i;
	for (i = 0; i < iHash_Size; i++)
	{
		int iPos = Hash_Table[i];
		if (iPos)
		{
			Hash_Item oItem = Item[iPos];
			int iChain_Count = 1;
			while (oItem.m_iNext)
			{
				iChain_Count++;
				oItem = Item[oItem.m_iNext];
			}
			if (iChain_Count > iMax_Chain)
			{
				iMax_Chain = iChain_Count;
				iMax_Pos = i;
			}
		}
	}
	printf("Max Chain:%d\n", iMax_Chain);
	//588980
	return;
}

static void Get_Neighbour_Start(Get_Contour_Info oInfo, Start_Count** ppNeighbour_Start_Count, int** ppNeighbour_Block, int* piCur_Neighbour)
{
	int iHash_Size = oInfo.m_iContour_Count&1 ? oInfo.m_iContour_Count + 2 : oInfo.m_iContour_Count + 1,
		* pHash_Table,
		iCur_Item = 1;
	pHash_Table = (int*)pMalloc(iHash_Size * sizeof(int));
	Hash_Item* pHash_Item = (Hash_Item*)pMalloc(iHash_Size * sizeof(Hash_Item));

	Get_Contour_Info::Line oLine, oNext_Line;
	Get_Contour_Info::Strip oStrip_0, oStrip_1;

	int i,j, y, iHeight_Minus_1 = oInfo.m_iHeight - 1;
	memset(pHash_Table, 0, iHash_Size * sizeof(int));

	int iAdd_Hash_Count = 0;
	for (y = 0; y < iHeight_Minus_1; y++)
	{
		oLine = oInfo.m_pLine[y];
		oNext_Line = oInfo.m_pLine[y + 1];

		i = j = 0;
		//int iCounter = 0;
		//while(i < oLine.m_iStrip_Count || j < oNext_Line.m_iStrip_Count)
		while(1)
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

			////这样写有点蹩脚
			//while (oStrip_0.m_iEnd < oStrip_1.m_iStart && i < oLine.m_iStrip_Count)
			//{
			//	i++;
			//	oStrip_0 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			//}
			//while (oStrip_1.m_iEnd < oStrip_0.m_iStart && j<oNext_Line.m_iStrip_Count)
			//{
			//	j++;
			//	oStrip_1 = oInfo.m_pStrip[oNext_Line.m_iFirst_Strip + j];
			//}

			if (oStrip_0.m_iContour_ID != oStrip_1.m_iContour_ID)
			{
				iAdd_Hash_Count++;
				Add_To_Hash_Table(pHash_Table, pHash_Item,iHash_Size, &iCur_Item,oStrip_0.m_iContour_ID, oStrip_1.m_iContour_ID);
			}
			i++;
			if (i >= oLine.m_iStrip_Count || j >= oNext_Line.m_iStrip_Count)
				break;
			//iCounter++;
		}

		//再横向加关系
		for (i = 1; i < oLine.m_iStrip_Count; i++)
		{
			oStrip_0 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i-1];
			oStrip_1 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			iAdd_Hash_Count++;
			Add_To_Hash_Table(pHash_Table, pHash_Item,iHash_Size, &iCur_Item,oStrip_0.m_iContour_ID, oStrip_1.m_iContour_ID);
		}
	}
	Free(pHash_Table);

	Start_Count* pNeighbour_Start_Count = (Start_Count*)pMalloc(oInfo.m_iContour_Count * sizeof(Start_Count));
	memset(pNeighbour_Start_Count, 0, oInfo.m_iContour_Count * sizeof(Start_Count));
	pHash_Item++;	//调整到真正开始位置
	int iCount = iCur_Item - 1;
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
	int* pNeighbour_Block = (int*)pMalloc(iCount * 2 * sizeof(int));

	for (i = 0; i < iCount; i++)
	{
		Hash_Item oItem = pHash_Item[i];
		/*if (oItem.A == 934 || oItem.B == 934)
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
	Free(pHash_Item);
	return;
}
static void Get_Neighbour_Start(Get_Contour_Info oInfo,Neighbour **ppNeighbour_Link,Start_Count **ppContour_Start_Count,int *piCur_Neighbour)
{//先找到所有Contour两两有关系的邻对，规格化为邻对前面ID小于后面
//这是一个查找问题
	int iHash_Size = oInfo.m_iContour_Count&1 ? oInfo.m_iContour_Count + 2 : oInfo.m_iContour_Count + 1,
		* pHash_Table,
		iCur_Item = 1;
	pHash_Table = (int*)pMalloc(iHash_Size * sizeof(int));
	Hash_Item* pHash_Item = (Hash_Item*)pMalloc(iHash_Size * sizeof(Hash_Item));

	Get_Contour_Info::Line oLine, oNext_Line;
	Get_Contour_Info::Strip oStrip_0, oStrip_1;

	int i,j, y, iHeight_Minus_1 = oInfo.m_iHeight - 1;
	memset(pHash_Table, 0, iHash_Size * sizeof(int));

	int iAdd_Hash_Count = 0;
	for (y = 0; y < iHeight_Minus_1; y++)
	{
		oLine = oInfo.m_pLine[y];
		oNext_Line = oInfo.m_pLine[y + 1];

		i = j = 0;
		//int iCounter = 0;
		//while(i < oLine.m_iStrip_Count || j < oNext_Line.m_iStrip_Count)
		while(1)
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

			////这样写有点蹩脚
			//while (oStrip_0.m_iEnd < oStrip_1.m_iStart && i < oLine.m_iStrip_Count)
			//{
			//	i++;
			//	oStrip_0 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			//}
			//while (oStrip_1.m_iEnd < oStrip_0.m_iStart && j<oNext_Line.m_iStrip_Count)
			//{
			//	j++;
			//	oStrip_1 = oInfo.m_pStrip[oNext_Line.m_iFirst_Strip + j];
			//}

			if (oStrip_0.m_iContour_ID != oStrip_1.m_iContour_ID)
			{
				iAdd_Hash_Count++;
				Add_To_Hash_Table(pHash_Table, pHash_Item,iHash_Size, &iCur_Item,oStrip_0.m_iContour_ID, oStrip_1.m_iContour_ID);
			}
			i++;
			if (i >= oLine.m_iStrip_Count || j >= oNext_Line.m_iStrip_Count)
				break;
			//iCounter++;
		}

		//再横向加关系
		for (i = 1; i < oLine.m_iStrip_Count; i++)
		{
			oStrip_0 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i-1];
			oStrip_1 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
			iAdd_Hash_Count++;
			Add_To_Hash_Table(pHash_Table, pHash_Item,iHash_Size, &iCur_Item,oStrip_0.m_iContour_ID, oStrip_1.m_iContour_ID);
		}
	}

	//最后一行也搞搞
	oLine = oInfo.m_pLine[y];
	//再横向加关系
	for (i = 1; i < oLine.m_iStrip_Count; i++)
	{
		oStrip_0 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i-1];
		oStrip_1 = oInfo.m_pStrip[oLine.m_iFirst_Strip + i];
		iAdd_Hash_Count++;
		Add_To_Hash_Table(pHash_Table, pHash_Item,iHash_Size, &iCur_Item,oStrip_0.m_iContour_ID, oStrip_1.m_iContour_ID);
	}
	//printf("Hash_Item_Count:%d\n", iCur_Item);

	//此处是不是线释放了
	if (pHash_Table)
		Free(pHash_Table);
	//Test_Hash_Table(pHash_Table, pHash_Item, iHash_Size);

	//理论上，此处找到的关系必然是连通数-1,但怎么证？数学归纳法？
	if (iCur_Item != oInfo.m_iContour_Count)
	{
		printf("Error");
		return;
	}

	int iRalation_Count = oInfo.m_iContour_Count - 1;
	int iCur_Neighbour = 1;
	Neighbour oNeighbour,* pNeighbour_Link = (Neighbour*)pMalloc(oInfo.m_iContour_Count * 2 * sizeof(Neighbour));
	Start_Count *pContour_Start_Count = (Start_Count*)pMalloc(oInfo.m_iContour_Count *  sizeof(Start_Count));
	for (i = 0; i < oInfo.m_iContour_Count; i++)
		pContour_Start_Count[i] = { i,0,0 };

	pHash_Item++;	//调整到真正开始位置
	pNeighbour_Link--;
	//pNeighbour_Next是一个链表；任意一个Contour都有若干个邻居
	//这个Contour的第一个邻居记录在pContour_Start_Count->m_iStart，
	// 邻居的个数记录在 pContour_Start_Count->m_iCount
	//后面就靠这条链找
	for (i = 0; i < iRalation_Count; i++)
	{
		oNeighbour.m_iContour_ID = pHash_Item[i].A;
		/*if (pHash_Item[i].B == 32)
			printf("%d ", pHash_Item[i].A);*/
		oNeighbour.m_iNext = pContour_Start_Count[pHash_Item[i].B].m_iStart;
		pNeighbour_Link[iCur_Neighbour] = oNeighbour;
		pContour_Start_Count[pHash_Item[i].B].m_iCount++;
		pContour_Start_Count[pHash_Item[i].B].m_iStart = iCur_Neighbour;
		iCur_Neighbour++;

		oNeighbour.m_iContour_ID = pHash_Item[i].B;
		oNeighbour.m_iNext = pContour_Start_Count[pHash_Item[i].A].m_iStart;
		pNeighbour_Link[iCur_Neighbour] = oNeighbour;
		pContour_Start_Count[pHash_Item[i].A].m_iCount++;
		pContour_Start_Count[pHash_Item[i].A].m_iStart = iCur_Neighbour;
		iCur_Neighbour++;
	}
	Free(pHash_Item-1);

	/*int iCount = 0;
	for (i = 0; i < oInfo.m_iContour_Count; i++)
	{
		Start_Count oContour = pContour_Start_Count[i];
		oNeighbour = pNeighbour_Link[oContour.m_iStart];
		if (i == 1635)
			printf("Here");
		iCount = 0;
		while (1)
		{
			iCount++;
			if (oNeighbour.m_iNext)
				oNeighbour = pNeighbour_Link[oNeighbour.m_iNext];
			else
				break;
		}
		if (iCount != oContour.m_iCount)
			printf("error");
	}*/
	*ppContour_Start_Count = pContour_Start_Count;
	*ppNeighbour_Link = pNeighbour_Link;
	*piCur_Neighbour = iCur_Neighbour;
	return;
}

static void Gen_Neighbour_Block(Get_Contour_Info oInfo,Start_Count* pNeighbour_Start_Count, Neighbour* pNeighbour_Link, int* pNeighbour_Block)
{
	//以下整齐排列Start Count的形式
	//先搞第一个
	Start_Count oNode_Start_Count = pNeighbour_Start_Count[0];
	Neighbour oNeighbour = pNeighbour_Link[oNode_Start_Count.m_iStart];
	int i, j;	// , iCur_Neighbour = 0;

	//以下先搞第一个是有理由的，因为后面是需要前一个元素
	for (j = 0; j < oNode_Start_Count.m_iCount; j++)
	{
		//pNeighbour_Block[oNode_Start_Count.m_iStart+j] = oNeighbour.m_iContour_ID;
		pNeighbour_Block[j] = oNeighbour.m_iContour_ID;
		oNeighbour = pNeighbour_Link[oNeighbour.m_iNext];
	}
	oNode_Start_Count.m_iStart = 0;
	pNeighbour_Start_Count[0] = oNode_Start_Count;

	for (i = 1; i < oInfo.m_iContour_Count; i++)
	{
		/*if (i == 1635)
		printf("Here");*/
		oNode_Start_Count = pNeighbour_Start_Count[i];		
		oNeighbour = pNeighbour_Link[oNode_Start_Count.m_iStart];
		oNode_Start_Count.m_iStart = pNeighbour_Start_Count[i - 1].m_iStart + pNeighbour_Start_Count[i - 1].m_iCount;
		for (j = 0; j < oNode_Start_Count.m_iCount; j++)
		{
			pNeighbour_Block[oNode_Start_Count.m_iStart + j] = oNeighbour.m_iContour_ID;
			/*if (!oNeighbour.m_iNext && j < oNode_Start_Count.m_iCount - 1)
			printf("Error");*/
			//if (oNode_Start_Count.m_iStart + j >= 3277)
				//printf("err");
			oNeighbour = pNeighbour_Link[oNeighbour.m_iNext];		
		}
		pNeighbour_Start_Count[i] = oNode_Start_Count;
	}
	Free(pNeighbour_Link+1);
	return;
}


//static void Gen_Neighbour_Block(Get_Contour_Info oInfo,Start_Count* pNeighbour_Start_Count, Neighbour* pNeighbour_Link, int* pNeighbour_Block)
//{
//	//以下整齐排列Start Count的形式
//	//先搞第一个
//	Start_Count oNode_Start_Count = pNeighbour_Start_Count[0];
//	Neighbour oNeighbour = pNeighbour_Link[oNode_Start_Count.m_iStart];
//	int i,j,iCur_Neighbour = 1;
//
//	/*int iCount = 0;
//	for (i = 0; i < oInfo.m_iContour_Count; i++)
//		iCount += pNeighbour_Start_Count[i].m_iCount;*/
//
//	//以下先搞第一个是有理由的，因为后面是需要前一个元素
//	for (j = 0; j < oNode_Start_Count.m_iCount; j++)
//	{
//		pNeighbour_Block[oNode_Start_Count.m_iStart+j] = oNeighbour.m_iContour_ID;
//		oNeighbour = pNeighbour_Link[oNeighbour.m_iNext];
//	}
//
//	for (i = 1; i < oInfo.m_iContour_Count; i++)
//	{
//		/*if (i == 1635)
//			printf("Here");*/
//		oNode_Start_Count = pNeighbour_Start_Count[i];		
//		oNeighbour = pNeighbour_Link[oNode_Start_Count.m_iStart];
//		oNode_Start_Count.m_iStart = pNeighbour_Start_Count[i - 1].m_iStart + pNeighbour_Start_Count[i - 1].m_iCount;
//
//		for (j = 0; j < oNode_Start_Count.m_iCount; j++)
//		{
//			pNeighbour_Block[oNode_Start_Count.m_iStart + j] = oNeighbour.m_iContour_ID;
//			/*if (!oNeighbour.m_iNext && j < oNode_Start_Count.m_iCount - 1)
//				printf("Error");*/
//			if (oNode_Start_Count.m_iStart + j >= 3277)
//				printf("err");
//
//			oNeighbour = pNeighbour_Link[oNeighbour.m_iNext];		
//		}
//		pNeighbour_Start_Count[i] = oNode_Start_Count;
//	}
//	Free(pNeighbour_Link+1);
//	return;
//}

void Tree_2_Hierachy(Contour_Tree_Node *pTree, Get_Contour_Result oResult)
{//将树结构转换为类似OpenCV四元组
	Contour_Tree_Node oNode,* pQueue = (Contour_Tree_Node*)pMalloc(oResult.m_iContour_Count * sizeof(Contour_Tree_Node));
	int iQueue_Head=0, iQueue_End,  iQueue_Count;
	oNode = pTree[0];
	while (oNode.m_iParent != -1)
		oNode = pTree[oNode.m_iParent];

	//设置根节点的所有值
	int* pNeighbour = oResult.m_pContour[oNode.m_iContour_ID].hierachy;
	//[0]为前一个兄弟
	pNeighbour[0] = -1;
	//[1]为下一个兄弟
	pNeighbour[1] = oNode.m_iNext_Sibling;
	//[2]为第一个子轮廓
	pNeighbour[2] = oNode.m_iFirst_Child;
	//[3]为父轮
	pNeighbour[3] = oNode.m_iParent;

	pQueue[0] = oNode;
	iQueue_End =(iQueue_Head + 1) % oResult.m_iContour_Count;
	iQueue_Count = 1;
	//int iCounter = 0;
	int iMax = 0;
	while (iQueue_Count)
	{
		oNode = pQueue[iQueue_Head % oResult.m_iContour_Count];
		iQueue_Head = (iQueue_Head + 1) % oResult.m_iContour_Count;
		iQueue_Count--;

		//in queue
		if (oNode.m_iFirst_Child!=-1)
		{
			oNode = pTree[oNode.m_iFirst_Child];
			/*if (oNode.m_iContour_ID == 227)
				printf("here");*/
			//这里得到节点，赋值
			int* pNeighbour = oResult.m_pContour[oNode.m_iContour_ID].hierachy;
			int iPre_Contour_ID = oNode.m_iContour_ID;
			//[0]为前一个兄弟
			pNeighbour[0] = -1;
			//[1]为下一个兄弟
			pNeighbour[1] = oNode.m_iNext_Sibling;
			//[2]为第一个子轮廓
			pNeighbour[2] = oNode.m_iFirst_Child;
			//[3]为父轮
			pNeighbour[3] = oNode.m_iParent;

			pQueue[iQueue_End] = oNode;
			iQueue_Count++;
			iQueue_End = (iQueue_End + 1) % oResult.m_iContour_Count;
			while (oNode.m_iNext_Sibling != -1)
			{
				oNode = pTree[oNode.m_iNext_Sibling];
				/*if (oNode.m_iContour_ID == 227)
					printf("here");*/

				//设置所有的值
				pNeighbour = oResult.m_pContour[oNode.m_iContour_ID].hierachy;
				//[0]为前一个兄弟
				pNeighbour[0] = iPre_Contour_ID;;
				//[1]为下一个兄弟
				pNeighbour[1] = oNode.m_iNext_Sibling;
				//[2]为第一个子轮廓
				pNeighbour[2] = oNode.m_iFirst_Child;
				//[3]为父轮
				pNeighbour[3] = oNode.m_iParent;
				iPre_Contour_ID = oNode.m_iContour_ID;

				pQueue[iQueue_End] = oNode;
				iQueue_End = (iQueue_End + 1) % oResult.m_iContour_Count;
				iQueue_Count++;
				//iCounter++;
			}
		}
	}
	Free(pQueue);
	return;
}

void Disp_Hierachy(Get_Contour_Result oResult)
{
	int i;
	int iRoot, iNode;
	for (i = 0; i < oResult.m_iContour_Count; i++)
	{
		if (oResult.m_pContour[i].hierachy[3] == -1)
			iRoot = i;	//这个是根
	}
	Disp((unsigned short*)oResult.m_pContour[iRoot].m_pPoint, oResult.m_pContour[iRoot].m_iOutline_Point_Count, 2, "Root");

	iNode = oResult.m_pContour[iRoot].hierachy[2];
	while (iNode != -1)
	{
		Disp((unsigned short*)oResult.m_pContour[iNode].m_pPoint, oResult.m_pContour[iNode].m_iOutline_Point_Count, 2, "Child");
		if (oResult.m_pContour[iNode].hierachy[1] != -1)
			iNode = oResult.m_pContour[iNode].hierachy[1];
		else
			break;
	}
	return;
}
void Traverse(Contour_Tree_Node *pTree,int iCount)
{//简单测试一下，遍历树，形态太烂，不能用递归，来个队列吧
	Contour_Tree_Node* poRoot;
	Contour_Tree_Node oNode,* pQueue = (Contour_Tree_Node*)pMalloc(iCount * sizeof(Contour_Tree_Node));
	int iQueue_Head = 0, iQueue_End,  iQueue_Count;
	memset(pQueue, 0, iCount * sizeof(int));
	//第一步先找Root
	poRoot = &pTree[0];
	while (poRoot->m_iParent!=-1)
		poRoot = &pTree[poRoot->m_iParent];

	pQueue[0] = *poRoot;
	iQueue_End =(iQueue_Head + 1) % iCount;
	iQueue_Count = 1;
	int iCounter = 0;
	while (iQueue_Count)
	{
		//out queue
		/*if (iQueue_Count == 40671)
		printf("here");*/
		oNode = pQueue[iQueue_Head % iCount];
		iQueue_Head = (iQueue_Head+1) % iCount;
		iQueue_Count--;
		//printf("%d\n", oNode.m_iContour_ID);

		//in queue
		if (oNode.m_iFirst_Child!=-1)
		{
			/*if (oNode.m_iContour_ID == 589390)
			printf("here");*/
			oNode = pTree[oNode.m_iFirst_Child];
			pQueue[iQueue_End] = oNode;
			iQueue_Count++;
			iQueue_End = (iQueue_End + 1) % iCount;
			while (oNode.m_iNext_Sibling != -1)
			{
				/*if (iCounter == 350888)
				printf("here");*/
				oNode = pTree[oNode.m_iNext_Sibling];
				pQueue[iQueue_End] = oNode;
				iQueue_End = (iQueue_End + 1) % iCount;
				iQueue_Count++;
				iCounter++;
				if (iQueue_Count >= iCount)
					printf("error");
			}
		}
	}
	return;
}
static void Start_Count_2_Hierachy(Start_Count* pNeighbour_Start_Count,int *pNeighbour_Block, Get_Contour_Result oResult)
{
	//一趟趟扫描表，把点修正
	int* pNode_Undone = (int*)pMalloc(oResult.m_iContour_Count * sizeof(int));
	int iUndone_Count,iCur_Item = 0;
	int i, j;
	int (*pHierachy)[4] = (int(*)[4])pMalloc(oResult.m_iContour_Count * 4 * sizeof(int));

	//先把树初始化为秃节点，无父无子无兄弟
	memset(pHierachy, -1, oResult.m_iContour_Count * 4 * sizeof(int));

	//pNode_Undone是一张表，记录所有还未干的节点，初始时为所有拥有两个邻居的才搞
	for (i =j = 0; i < oResult.m_iContour_Count; i++)
		if (pNeighbour_Start_Count[i].m_iCount >= 2)
		{
			pNode_Undone[j++] = i;
			//printf("Contour:%d Neighbour Count:%d\n",i, pNeighbour_Start_Count[i].m_iCount);
		}

	iUndone_Count = j;
	int iRound = 0;
	int iCounter = 0;
	
	while (iUndone_Count)
	{//此处不断调整，直到邻居数为0
		for (i = 0; i < iUndone_Count; i++)
		{
			Start_Count oParent_Start_Count = pNeighbour_Start_Count[pNode_Undone[i]];
			/*if (oParent_Start_Count.m_iContour_ID == 32)
			{
				for (int k = 0; k < oParent_Start_Count.m_iCount; k++)
				{
					printf("%d\n", pNeighbour_Block[oParent_Start_Count.m_iStart + k]);
				}
			}*/
			int* pParent = pHierachy[oParent_Start_Count.m_iContour_ID];	//oResult.m_pContour[oParent_Start_Count.m_iContour_ID].hierachy;
			//对Parent所有的孩子进行调整
			for (j = 0; j < oParent_Start_Count.m_iCount; j++)
			{
				/*if (i == 17 && j==701)
					printf("Here");*/
				Start_Count oChild_Start_Count = pNeighbour_Start_Count[pNeighbour_Block[oParent_Start_Count.m_iStart + j]];
				//if (oChild_Start_Count.m_iContour_ID == 8)
					//printf("Here");

				if (oChild_Start_Count.m_iCount == 1)
				{//该节点的邻居只有一个，以前的孩子都处理完了，那么表示它还有个父节点
					int* pNode = pHierachy[oChild_Start_Count.m_iContour_ID];	//oResult.m_pContour[oChild_Start_Count.m_iContour_ID].hierachy;
					//设置节点的父亲
					pNode[3]=oParent_Start_Count.m_iContour_ID;
					//节点的Next Sibling为父亲的第一个孩子
					pNode[1] = pParent[2];
					//节点的前一个兄弟与孩子都未知，暂设-1
					pNode[0] = -1;

					if (pParent[2]!=-1)
					{//有Next Sibling才需要设
						//int* pNext_Sibling = pHierachy[pParent[2]];	//oResult.m_pContour[pParent[2]].hierachy;
						//Next Sibling的前一个孩子是这个节点
						pHierachy[pParent[2]][0] = oChild_Start_Count.m_iContour_ID;
					}
					
					//父亲的第一个孩子设为这个节点
					pParent[2]=oChild_Start_Count.m_iContour_ID;

					//修改pNeighbour_2
					pNeighbour_Block[oParent_Start_Count.m_iStart + j] = -1;

					//更新Child_Start_Count
					pNeighbour_Start_Count[oChild_Start_Count.m_iContour_ID].m_iCount=0;
				}
				/*if (pHierachy[32][2] == -1)
					printf("here");*/
			}

			//更新oParent_Start_Count
			int k;
			for (j=k = 0; j < oParent_Start_Count.m_iCount; j++)
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
		for (i =j= 0; i < iUndone_Count; i++)
		{
			Start_Count oParent_Start_Count = pNeighbour_Start_Count[pNode_Undone[i]];
			if (oParent_Start_Count.m_iCount >= 2)
				pNode_Undone[j++] = pNode_Undone[i];
		}
		iUndone_Count = j;
		iCounter++;
		iRound++;
	}

	typedef struct Value_16{
		char Value[16];
	}Value_16;
	for (i = 0; i < oResult.m_iContour_Count; i++)
	{
		*(Value_16*)oResult.m_pContour[i].hierachy =
			*(Value_16*)pHierachy[i];
	}

	Free(pHierachy);
	Free(pNode_Undone);
	Free(pNeighbour_Block);
	Free(pNeighbour_Start_Count);
	return;
}

static void Build_Tree(Get_Contour_Info oInfo,Contour_Tree_Node **ppTree,Start_Count* pNeighbour_Start_Count, int *pNeighbour_Block)
{//根据Neighbour生成一棵树。代码不够简洁，引入了一个Undone数组，实际上没啥用，要去掉
	Contour_Tree_Node oNode,oParent,* pTree = (Contour_Tree_Node*)pMalloc(oInfo.m_iContour_Count * sizeof(Contour_Tree_Node));
	int* pNode_Undone = (int*)pMalloc(oInfo.m_iContour_Count * sizeof(int));
	int iUndone_Count,iCur_Item = 0;
	int i, j;
	//先把树初始化为秃节点，无父无子无兄弟
	for (i = 0; i < oInfo.m_iContour_Count; i++)
		pTree[i] = {i,-1,-1,-1 };

	//pNode_Undone是一张表，记录所有还未干的节点，初始时为所有拥有两个邻居的才搞
	for (i =j = 0; i < oInfo.m_iContour_Count; i++)
		if (pNeighbour_Start_Count[i].m_iCount >= 2)
			pNode_Undone[j++] = i;
	iUndone_Count = j;

	while (iUndone_Count)
	{//此处不断调整，直到邻居数为0
	 //重写，慢到死，要杜绝平方级时间复杂度
		for (i = 0; i < iUndone_Count; i++)
		{
			Start_Count oParent_Start_Count = pNeighbour_Start_Count[pNode_Undone[i]];
			oParent = pTree[oParent_Start_Count.m_iContour_ID];
			for (j = 0; j < oParent_Start_Count.m_iCount; j++)
			{
				Start_Count oChild_Start_Count = pNeighbour_Start_Count[pNeighbour_Block[oParent_Start_Count.m_iStart + j]];
				if (oChild_Start_Count.m_iCount == 1)
				{//可以处理了
					oNode = pTree[oChild_Start_Count.m_iContour_ID];
					oNode.m_iParent = oParent_Start_Count.m_iContour_ID;
					oNode.m_iNext_Sibling = oParent.m_iFirst_Child;
					oParent.m_iFirst_Child = oChild_Start_Count.m_iContour_ID;
					pTree[oChild_Start_Count.m_iContour_ID] = oNode;

					//修改pNeighbour_2
					pNeighbour_Block[oParent_Start_Count.m_iStart + j] = -1;

					//更新Child_Start_Count
					pNeighbour_Start_Count[oChild_Start_Count.m_iContour_ID].m_iCount=0;
				}
			}
			pTree[oParent_Start_Count.m_iContour_ID] = oParent;

			//更新oParent_Start_Count
			int k;
			for (j=k = 0; j < oParent_Start_Count.m_iCount; j++)
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
		for (i =j= 0; i < iUndone_Count; i++)
		{
			Start_Count oParent_Start_Count = pNeighbour_Start_Count[pNode_Undone[i]];
			if (oParent_Start_Count.m_iCount >= 2)
				pNode_Undone[j++] = pNode_Undone[i];
		}
		iUndone_Count = j;
	}

	Free(pNode_Undone);
	Free(pNeighbour_Block);
	Free(pNeighbour_Start_Count);
	//理论上只剩下一棵树了
	*ppTree = pTree;
}
void Test_Hierarch(Get_Contour_Result oResult)
{//看看四元组对不对
	int i;
	for (i = 0; i < oResult.m_iContour_Count; i++)
	{
		int* pNode = oResult.m_pContour[i].hierachy;

		//先测试兄弟关系
		int* pSibline, * pParent;
		if (pNode[0] != -1)
		{
			pSibline = oResult.m_pContour[pNode[0]].hierachy;
			if (pSibline[1] != i)
				printf("Error");
		}
		if (pNode[1] != -1)
		{
			pSibline = oResult.m_pContour[pNode[1]].hierachy;
			if (pSibline[0] !=i)
				printf("error");
		}
		if (pNode[0] == -1 && pNode[3]!=-1)
		{
			pParent = oResult.m_pContour[pNode[3]].hierachy;
			if (i != pParent[2])
				printf("Error");
		}
	}
	return;
}

static void Gen_Hierarchy(Get_Contour_Info oInfo, Contour_Tree_Node** ppTree, Get_Contour_Result oResult)
{//生成一棵树，谁和谁一级，谁和谁是父子关系
	//第一步，找到所有的两两联系
	int iCur_Neighbour;

	//Neighbour* pNeighbour_Link;					//pNeighbour_Next是一个链表；任意一个Contour都有若干个邻居
	//Start_Count* pNeighbour_Start_Count;		//这个Contour的第一个邻居记录在pContour_Start_Count->m_iStart，
	//											// 邻居的个数记录在 pContour_Start_Count->m_iCount
	//											//后面就靠这条链找
	//Get_Neighbour_Start(oInfo, &pNeighbour_Link,&pNeighbour_Start_Count,&iCur_Neighbour);

	//int* pNeighbour_Block=(int*)pMalloc(iCur_Neighbour * sizeof(int));	//将所有Neighbour从链表变成整齐的块状
	//Gen_Neighbour_Block(oInfo, pNeighbour_Start_Count, pNeighbour_Link, pNeighbour_Block);

	Start_Count* pNeighbour_Start_Count;
	int* pNeighbour_Block;
	Get_Neighbour_Start(oInfo, &pNeighbour_Start_Count, &pNeighbour_Block, &iCur_Neighbour);
		
	//for (int i = 0; i < oInfo.m_iContour_Count; i++)
		//printf("i:%d Start:%d Count:%d\n", i, pNeighbour_Start_Count[i].m_iStart, pNeighbour_Start_Count[i].m_iCount);

	//一气呵成，还慢了
	Start_Count_2_Hierachy(pNeighbour_Start_Count, pNeighbour_Block, oResult);
	//Test_Hierarch(oResult);

	////**************革命分两步走，先建树，这个树始终难以割舍，码农至爱**************
	//Build_Tree(oInfo, ppTree, pNeighbour_Start_Count, pNeighbour_Block);
	//Tree_2_Hierachy(*ppTree, oResult);
	//
	////简单测试一下树，这个必须放在Free之前，否则出错
	////Traverse(*ppTree,oInfo.m_iContour_Count);

	//Free(*ppTree);
	////**************革命分两步走，先建树，这个树始终难以割舍，码农至爱**************
	return;
}
void Find_Contour(Get_Contour_Info oInfo, int x, int y)
{
	int i;
	for (i = 0; i < oInfo.m_iContour_Count; i++)
	{
		Get_Contour_Info::Contour oContour = oInfo.m_pContour[i];
		if (y<oContour.m_iY_Start || y>=oContour.m_iY_Start+oContour.m_iLine_Count)
			continue;
		for (int j = 0; j < oContour.m_iLine_Count; j++)
		{
			Get_Contour_Info::Line oLine = oInfo.m_pLine[oContour.m_iFirst_Line+j];
			for (int k = 0; k < oLine.m_iStrip_Count; k++)
			{
				Get_Contour_Info::Strip oStrip = oInfo.m_pStrip[oLine.m_iFirst_Strip + k];
				if (x >= oStrip.m_iStart && x <= oStrip.m_iEnd && oContour.m_iY_Start + j == y)
				{
					printf("Contour %d Found\n", i);
					return;
				}	
			}
		}
	}
	printf("Fail to find contour contain:%d,%d\n", x, y);
	return;
}
//应该有一组不同的获取接口，有的人就不喜欢要内部结构
void Get_Contour(Image oImage,Get_Contour_Result *poResult)
{//oImage: 图片数据		piContour_Count：返回一共找到多少个连通域
	//OpenCV的瓶颈代码如文件结束注释，放在OpenCV环境下运行即可，要用8秒
	Get_Contour_Info oInfo = {};
	unsigned long long tStart = iGet_Tick_Count();
	////打算不搞预分配
	if (!bInit_Get_Contour_Info(&oInfo, 4000000, 10000, oImage.m_iWidth,oImage.m_iHeight))
		return;

	//***********由于要应付大内存消耗，第一趟先扫出所有Strip，然后Compact内存********
	if (!bGet_Strip(&oInfo, oImage))
		goto END;
	////测试bGet_Strip的性能
	//tStart = iGet_Tick_Count();
	//for (i = 0; i < 100; i++)
	//{
	//	if (!bGet_Strip(&oInfo, oImage))
	//		goto END;
	//	if (i == 99)
	//		break;
	//	Free_Contour_Info(&oInfo);
	//}
	//printf("Strip Count:%d Time span:%lldms\n",oInfo.m_iStrip_Count, iGet_Tick_Count() - tStart);
	//***********由于要应付大内存消耗，第一趟先扫出所有Strip，然后Compact内存********

	//******************根据Strip找到所有的连通域，但是尚未勾边**********************
	Get_Contour(&oInfo,oImage.m_iHeight);
	//Draw_Contour_Step_1("c:\\tmp\\1.bmp", oInfo, iFind_Max_Contour(oInfo,0));
	////测试性能
	//tStart = iGet_Tick_Count();
	//for (i = 0; i < 10; i++)
	//{
	//	Get_Contour(&oInfo,oImage.m_iHeight);
	//	if (i == 9)
	//		break;
	//	Free(oInfo.m_pContour),oInfo.m_iContour_Count = 0;
	//}	
	//printf("Time span: %lld ms Contour:%d\n", iGet_Tick_Count() - tStart,oInfo.m_iContour_Count);
	//******************根据Strip找到所有的连通域，但是尚未勾边**********************
	//printf("%d\n", iGet_Max_Contour(oInfo));
	bInit_Get_Contour_Result(oInfo, poResult);

	////*****************将所有的Contour组织成一棵树***********************************
	Contour_Tree_Node* pTree;
	Gen_Hierarchy(oInfo,&pTree,*poResult);
	//Free(pTree);
	//Test_Hierarch(*poResult);
	
	////以下为测试性能
	//tStart = iGet_Tick_Count();
	//for (int i = 0; i < 5000; i++)
	//{
	//	Gen_Hierarchy(oInfo,&pTree,*poResult);
	//	//Free(pTree);
	//}
	//printf("Time span: %lld ms Contour:%d\n", iGet_Tick_Count() - tStart,oInfo.m_iContour_Count);
	//////*****************将所有的Contour组织成一棵树***********************************


	//******************为后面的轮廓勾勒建立拓扑，此处不能循环测试*******************
	//tStart = iGet_Tick_Count();
	Prepare_Outline(&oInfo);
	//printf("Time span: %lld ms Contour:%d\n", iGet_Tick_Count() - tStart,oInfo.m_iContour_Count);
	//******************为后面的轮廓勾勒建立拓扑，此处不能循环测试*******************
	//Find_Contour(oInfo, 2847, 4992);

	//*********************************勾勒轮廓**************************************
	Gen_Outline(oInfo,poResult,4000000);
	////以下测试性能
	//tStart = iGet_Tick_Count();
	//for (int i = 0; i < 40; i++)
	//{
	//	Gen_Outline(oInfo,poResult,3000000);
	//	Free(poResult->m_pAll_Point);
	//}
	//printf("Time span: %lld ms Point Count:%d\n", iGet_Tick_Count() - tStart,poResult->m_iMax_Point_Count);
	////*********************************勾勒轮廓**************************************

	////点数存个盘
	//{
	//	FILE* pFile = fopen("c:\\tmp\\1.txt", "wb");
	//	for (int i = 0; i < oInfo.m_iContour_Count; i++)
	//	{
	//		fprintf(pFile, "%d %d\n", i, poResult->m_pContour[i].m_iOutline_Point_Count);
	//	}
	//	fclose(pFile);
	//}	
END:
	Free_Contour_Info(&oInfo);
}

////OpenCV瓶颈代码，一共要运行8秒
//void Opencv_Test()
//{
//	//装入二值图
//	cv::Mat pic = cv::imread("C:\\tmp\\Chess_Board_2.bmp", cv::IMREAD_GRAYSCALE);
//	//搞两个变量
//	std::vector<std::vector<cv::Point> > contours;  //连通域
//	std::vector<cv::Vec4i> hierarchy;               //拓扑
//
//	//主函数
//	cv::findContours(pic, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
//	//cout << contours.size() << endl;               //显示连通域个数
//	return;
//}

Get_Contour_Result::Contour oFind_Contour(Get_Contour_Result oResult, int x, int y)
{
	int i, j;
	for (i = 0; i < oResult.m_iContour_Count; i++)
	{
		Get_Contour_Result::Contour oContour = oResult.m_pContour[i];
		for (j = 0; j < oContour.m_iOutline_Point_Count; j++)
		{
			if (oContour.m_pPoint[j][0] == x && oContour.m_pPoint[j][1] == y)
				return oContour;
		}
	}
	return {};
}
void Gen_Keyboard_Detect_Data(Image *poImage)
{//生成连通域数据，对比opencv的结果
	Image oImage;
	Init_Image(&oImage, 500, 500, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	
	////	------   -----
	//// -----------------
	//Mid_Point_Line(oImage, 50, 100, 100, 100);
	//Mid_Point_Line(oImage, 150, 100, 200, 100);
	//Mid_Point_Line(oImage, 40, 101, 210, 101);

	//三竖
	Mid_Point_Line(oImage,100, 50, 100, 100);
	Mid_Point_Line(oImage,100, 150, 100, 200);
	Mid_Point_Line(oImage,101, 40, 101, 210);

	bSave_Image("c:\\tmp\\1.bmp", oImage);
	*poImage = oImage;
}
void Find_Contour_Test_2()
{
	Image oImage;
	Gen_Keyboard_Detect_Data(&oImage);
	Get_Contour_Result oResult;
	Get_Contour_Result::Contour oContour;
	Get_Contour(oImage, &oResult);
	for (int i = 0; i < oResult.m_iContour_Count; i++)
	{
		oContour = oResult.m_pContour[i];
		Disp((unsigned short*)oContour.m_pPoint, oContour.m_iOutline_Point_Count, 2, "Outline");
	}
	return;
	//Image oImage;
	//Get_Contour_Result oResult = {};
	//if (!bLoad_Image("c:\\tmp\\Chess_Board_5.bmp",&oImage))
	//	return;
	//Get_Contour(oImage, &oResult);
	////Free_Image(&oImage);

	//Get_Contour_Result::Contour* pOrg = (Get_Contour_Result::Contour*)pMalloc(oResult.m_iContour_Count * sizeof(Get_Contour_Result::Contour));
	//unsigned short (*pCur)[2], (*pOrg_Point)[2] = (unsigned short(*)[2])pMalloc(2000000 * 2 * sizeof(unsigned short));
	//FILE* pFile = fopen("c:\\tmp\\contour.txt", "rb");
	//int i, j, iPoint_Count=0;
	//pCur = pOrg_Point;
	//for (i = 0; i < oResult.m_iContour_Count; i++)
	//{
	//	Get_Contour_Result::Contour oContour = {};		
	//	fscanf(pFile, "%d", &iPoint_Count);
	//	oContour.m_iOutline_Point_Count = iPoint_Count;
	//	oContour.m_pPoint = pCur;
	//	for (j = 0; j < oContour.m_iOutline_Point_Count; j++)
	//	{
	//		fscanf(pFile, "%d,%d", &oContour.m_pPoint[j][0], &oContour.m_pPoint[j][1]);
	//	}
	//	pCur += oContour.m_iOutline_Point_Count;
	//	pOrg[i] = oContour;
	//}
	//iPoint_Count = pCur - pOrg_Point;
	//Shrink(pOrg_Point, iPoint_Count * 2*sizeof(unsigned short));

	////开始找点
	//for (i = 0; i < oResult.m_iContour_Count; i++)
	//{
	//	Get_Contour_Result::Contour oOrg = pOrg[i];
	//	if (i == 9)
	//		printf("here");
	//	Get_Contour_Result::Contour oNew = oFind_Contour(oResult, oOrg.m_pPoint[0][0], oOrg.m_pPoint[0][1] );
	//	if (oNew.m_iOutline_Point_Count == 0)
	//		printf("err");
	//}
	return;
}
void Find_Contour_Test_1()
{//此处针对OpenCV的棋盘检测过程中最耗时操作findcontour进行重新设计，OpenCV的代码在上面
//所需样本chess_board_2.bmp已经在项目的目录中，此样本OpenCV需7000 ms以上，本方案只需250 ms，
//比OpenCV方案快30倍，纯c，没有调用任何加速指令

	Image oImage;
	Get_Contour_Result oResult = {};
	if (!bLoad_Image("c:\\tmp\\Chess_Board.bmp",&oImage))
		return;

	unsigned long long tStart = iGet_Tick_Count();
	//for (int i = 0; i < 10; i++)
	{
		Get_Contour(oImage, &oResult);
		//释放
		//Free_Contour_Result(&oResult);
	}	
	printf("Time span: %lld ms Contour Count:%d Point Count:%d\n", iGet_Tick_Count() - tStart,oResult.m_iContour_Count, oResult.m_iMax_Point_Count);

	//显示连通域的轮廓点
	for (int i = 0; i < oResult.m_iContour_Count; i++)
		Disp((unsigned short*)oResult.m_pContour[i].m_pPoint, oResult.m_pContour[i].m_iOutline_Point_Count, 2, "Contour");

	//简单显示一下Root和第一层孩子
	//Disp_Hierachy(*poResult);

	//此处对最大连通域存个轮廓，注意只画点，没有连线
	//int iContour_ID = iGet_Max_Contour(oResult);
	//Draw_Contour_Outline((char*)"c:\\tmp\\1.bmp", oResult,iContour_ID);
	Free_Contour_Result(&oResult);
	Free_Image(&oImage);
}

template<typename _T>static void Find_Max_Dist_Pair(_T Point[][2], int iCount, float eps, Start_End *poRight_Slice, Start_End *poSlice,int *pbLE_eps)
{//近似寻找最远两点,暂时只管闭合
	Start_End oSlice, oRight_Slice = { 0,0 };	//暂时未知意义
	const int iIter_Count = 3;
	int i, j,bLE_eps=1, iPos = 0;
	//注意，以下m_iStart并不是在Point的位置，而是旋转中j的值
	//是一个相对于原Pos的位置
	//oRight_Slice.m_iStart = 0;
	for (i = 0; i < iIter_Count; i++)
	{
		iPos = (iPos + oRight_Slice.m_iStart) % iCount;
		_T Start_Point[2] = { Point[iPos][0],Point[iPos][1] };
		_T fMax = 0;
		iPos = (iPos + 1) % iCount;
		for (j = 1; j < iCount; j++,iPos=(iPos+1)%iCount)
		{//绕场一周, 找出与Start_Point距离最远点
			_T fDist,dx=Point[iPos][0]-Start_Point[0],	//x方向距离，y方向距离，当然理解为delta也可
				dy=Point[iPos][1]-Start_Point[1];	
			fDist = dx * dx + dy * dy;
			if( fDist > fMax )
			{
				fMax = fDist;
				oRight_Slice.m_iStart = j;
			}
		}
		//Less than or equal
		if (fMax > eps)
			bLE_eps = 0;
	}

	//bLE_eps其实就是有没有找到这个点对
	if (!bLE_eps)
	{
		//oRight_Slice.m_iEnd = oSlice.m_iStart = iPos % iCount;
		//oSlice.m_iEnd = oRight_Slice.m_iStart = (oRight_Slice.m_iStart + oSlice.m_iStart) % iCount;
		//感觉这个就行了
		oSlice.m_iStart = oRight_Slice.m_iEnd = iPos % iCount;
		oRight_Slice.m_iStart =  (oRight_Slice.m_iStart + oSlice.m_iStart) % iCount;
		oSlice.m_iEnd = oRight_Slice.m_iStart;
		*poSlice = oSlice;
		*poRight_Slice=oRight_Slice;
	}
	*pbLE_eps = bLE_eps;
	return;
}
template<typename _T>static void Normalize(_T Point_2D[][2], int iPoint_Count,
	_T Norm_Point[][2],_T Scale[2],_T Offset[2])
{//对一组点归一化，这个和Colmap又不一样，不求Max,Min
	//毫无营养
	_T Mean[2] = { 0 };
	int i;
	for (i = 0; i < iPoint_Count; i++)
		Mean[0] += Point_2D[i][0],	Mean[1] += Point_2D[i][1];

	Mean[0] /= iPoint_Count;
	Mean[1] /= iPoint_Count;

	_T Dev[2] = {};	//再求个偏离度
	for (i = 0; i < iPoint_Count; i++)
		Dev[0] += abs(Point_2D[i][0] - Mean[0]),Dev[1] += abs(Point_2D[i][1] - Mean[1]);
	Dev[0] /= iPoint_Count;
	Dev[1] /= iPoint_Count;

	_T s[2] = { 1.f / Dev[0],  1.f / Dev[1] };
	for (i = 0; i < iPoint_Count; i++)
	{
		Norm_Point[i][0] = s[0] * Point_2D[i][0]  - Mean[0] * s[0];
		Norm_Point[i][1] = s[1] * Point_2D[i][1]  - Mean[1] * s[1];
	}
	Scale[0] = s[0], Scale[1] = s[1];
	Offset[0] = -Mean[0]*s[0], Offset[1] = -Mean[1]*s[1];
	return;
}

template<typename _T>void Test_H(_T Point_3D[][2], _T Point_2D[][2], _T H[3 * 3], int iPoint_Count)
{
	_T(*pPoint_3D_1)[3] = (_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));
	_T (*pResult)[3] = (_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));

	int i;
	for (i = 0; i < iPoint_Count; i++)
	{
		pPoint_3D_1[i][0] = Point_3D[i][0];
		pPoint_3D_1[i][1] = Point_3D[i][1];
		pPoint_3D_1[i][2] = 1;
	}
	Matrix_Multiply((_T*)pPoint_3D_1, iPoint_Count, 3, H, 3, (_T*)pResult);

	//以下估计s
	_T s,fError = 0;
	for (i = 0; i < iPoint_Count; i++)
	{
		s = (pResult[i][0] + pResult[i][1] + pResult[i][2]) / (Point_2D[i][0] + Point_2D[i][1] + 1);
		//printf("s:%f\n", s);
		fError += (s*Point_2D[i][0] - pResult[i][0]) * (s*Point_2D[i][0] - pResult[i][0]);
		fError += (s*Point_2D[i][1] - pResult[i][1]) * (s*Point_2D[i][1] - pResult[i][1]);
		fError +=	(s - pResult[i][2]) * (s - pResult[i][2]);
	}
	printf("Avg Error:%f\n", sqrt(fError) / iPoint_Count);


	Free(pPoint_3D_1);
	Free(pResult);
	return;
}
template<typename _T>static int bZhang_Estimate_H_Inv_Power(_T Point_3D[][2], _T Point_2D[][2], int iPoint_Count,_T H[3*3])
{//试一下另一种方法，基于一个原理
	//对于A 的svd分解，其Vt最后一行为这么一个解x
	// Ax = 0	其中 |x|=1 这是个矛盾方程组，求最小二乘解
	//然而，这个x又是 A'A 最小特征值对应的特征向量，显然可以用
	//幂法求解 A'A的特征向量，此处就用这个方法
	_T(*pNorm_Point_3D)[2] = (_T(*)[2])pMalloc(iPoint_Count * 2 * sizeof(_T)),
		(*pNorm_Point_2D)[2] = (_T(*)[2])pMalloc(iPoint_Count * 2 * sizeof(_T));

	_T Mean_3D[2], Mean_2D[2], Scale_3D[2], Scale_2D[2];
	////方法1，先老老实实用归一化方法
	Normalize(Point_3D, iPoint_Count, pNorm_Point_3D, Scale_3D, Mean_3D);
	Normalize(Point_2D, iPoint_Count, pNorm_Point_2D, Scale_2D, Mean_2D);

	int w = 11, h = 8;
	//接下来，没点搞两条式子
	_T *A1,* A = (_T*)pMalloc(iPoint_Count * 2 * 9 * sizeof(_T));
	int i, iResult;
	for (i = 0; i < iPoint_Count; i++)
	{
		_T Px = pNorm_Point_3D[i][0], Py = pNorm_Point_3D[i][1] ,
			u = pNorm_Point_2D[i][0], v = pNorm_Point_2D[i][1];

		A1 = &A[i * 9*2];
		//Px*h0 + Py*h1 + h2 					- Px*u*h6 - Py*u*h7 - u*h8	= 0
		A1[0] = Px, A1[1] = Py, A1[2] = 1;
		A1[3] = A1[4] = A1[5] = 0;
		A1[6] = -Px * u, A1[7] = -Py * u, A1[8] = -u;

		A1 += 9;
		//				Py*h3 + Py*h4 + h5 	- Px*v*h6 - Py*v*h7 - v*h8 = 0
		A1[0] = A1[1] = A1[2] = 0;
		A1[3] = Px, A1[4] = Py, A1[5] = 1;
		A1[6] = -Px * v, A1[7] = -Py * v, A1[8] = -v;
	}

	_T AtA[9 * 9];
	Transpose_Multiply(A, iPoint_Count * 2, 9,AtA,0);
	//Disp(AtA, 9, 9, "AtA");

	iResult = bInverse_Power(AtA,9, (_T*)NULL, H);
	//Disp(H, 3, 3, "H");
	//Test_H(pNorm_Point_3D, pNorm_Point_2D,H, iPoint_Count);

	//搞糟3D和2D点的投影矩阵
	_T K_3D[3 * 3] = { Scale_3D[0],0,Mean_3D[0],
		0,Scale_3D[1],Mean_3D[1],
		0,0,1 };
	_T K_2D[3 * 3] = { Scale_2D[0],0,Mean_2D[0],
		0,Scale_2D[1],Mean_2D[1],
		0,0,1 },
	K_2D_Inv[3 * 3];

	Get_Inv_Matrix_Row_Op(K_2D, K_2D_Inv, 3, &iResult);
	Matrix_Multiply(K_2D_Inv, 3, 3, H, 3, H);
	Matrix_Multiply(H, 3, 3, K_3D, 3, H);

	Free(pNorm_Point_3D);
	Free(pNorm_Point_2D);
	Free(A);

	return iResult;
}
template<typename _T>static void Zhang_Estimate_H_SVD(_T Point_3D[][2], _T Point_2D[][2], int iPoint_Count,_T H[3*3])
{//设空间点维Point_3D，像素平面点为Point_2D，估计出一个H矩阵
	_T(*pNorm_Point_3D)[2] = (_T(*)[2])pMalloc(iPoint_Count * 2 * sizeof(_T)),
		(*pNorm_Point_2D)[2] = (_T(*)[2])pMalloc(iPoint_Count * 2 * sizeof(_T));

	_T Mean_3D[2], Mean_2D[2], Scale_3D[2], Scale_2D[2];
	////方法1，先老老实实用归一化方法
	Normalize(Point_3D, iPoint_Count,pNorm_Point_3D,Scale_3D,Mean_3D);
	Normalize(Point_2D, iPoint_Count,pNorm_Point_2D,Scale_2D,Mean_2D);

	//实践证明，用Normalize得方法要比直接求解得精度高
	//memcpy(pNorm_Point_3D, Point_3D, iPoint_Count * 2 * sizeof(_T));
	//memcpy(pNorm_Point_2D, Point_2D, iPoint_Count * 2 * sizeof(_T));

	//int w = 11, h = 8;
	//接下来，没点搞两条式子
	_T *A1,* A = (_T*)pMalloc(iPoint_Count * 2 * 9 * sizeof(_T));
	int i, iResult;
	for (i = 0; i < iPoint_Count; i++)
	{
		_T Px = pNorm_Point_3D[i][0], Py = pNorm_Point_3D[i][1] ,
			u = pNorm_Point_2D[i][0], v = pNorm_Point_2D[i][1];

		A1 = &A[i * 9*2];
		//Px*h0 + Py*h1 + h2 					- Px*u*h6 - Py*u*h7 - u*h8	= 0
		A1[0] = Px, A1[1] = Py, A1[2] = 1;
		A1[3] = A1[4] = A1[5] = 0;
		A1[6] = -Px * u, A1[7] = -Py * u, A1[8] = -u;
		
		A1 += 9;
		//				Py*h3 + Py*h4 + h5 	- Px*v*h6 - Py*v*h7 - v*h8 = 0
		A1[0] = A1[1] = A1[2] = 0;
		A1[3] = Px, A1[4] = Py, A1[5] = 1;
		A1[6] = -Px * v, A1[7] = -Py * v, A1[8] = -v;
	}

	//显示一下这个大矩阵
	//Disp(A, iPoint_Count * 2, 9, "A\z");
	SVD_Info oSVD;
	SVD_Alloc<_T>(iPoint_Count * 2, 9, &oSVD);
	svd_3(A, oSVD, &iResult);

	//Disp((_T*)oSVD.Vt, 9, 9, "Vt");
	memcpy(H, &((_T*)oSVD.Vt)[8*9], 9 * sizeof(_T));

	//搞糟3D和2D点的投影矩阵
	_T K_3D[3 * 3] = { Scale_3D[0],0,Mean_3D[0],
						0,Scale_3D[1],Mean_3D[1],
						0,0,1 };
	_T K_2D[3 * 3] = { Scale_2D[0],0,Mean_2D[0],
		0,Scale_2D[1],Mean_2D[1],
		0,0,1 },
		K_2D_Inv[3 * 3];

	Get_Inv_Matrix_Row_Op(K_2D, K_2D_Inv, 3, &iResult);
	Matrix_Multiply(K_2D_Inv, 3, 3, H, 3, H);
	Matrix_Multiply(H, 3, 3, K_3D, 3, H);

	//不能直接干归一化点，因为优化s的时候会出现分母为0
	//Disp(H, 3, 3, "H");
	//Test_H(pNorm_Point_3D, pNorm_Point_2D,H, iPoint_Count);

	Free(pNorm_Point_3D);
	Free(pNorm_Point_2D);
	Free_SVD(&oSVD);
	Free(A);
	return;
}

static void Draw_Chess_Board(const char* pcFile, int iWidth_In_Grad, int iHeight_In_Grad,int iGrid_Size,int x_Start,int y_Start)
{//画一个棋盘，从(x_Start,y_Start)开始画
	int x, y,iColor;
	Image oImage;
	Init_Image(&oImage, 800, 800, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage, 255, 255, 255);

	for (y = 0; y < iHeight_In_Grad; y++)
	{
		iColor = (y & 1);
		int y_Start_1 = y_Start + y * iGrid_Size;
		for (x = 0; x < iWidth_In_Grad;x++)
		{
			int x1,y1,x_Start_1 = x_Start + x * iGrid_Size;
			for (y1 = y_Start_1; y1 < y_Start_1 + iGrid_Size; y1++)
				for (x1 = x_Start_1; x1 < x_Start_1 + iGrid_Size; x1++)
					oImage.m_pChannel[0][y1 * oImage.m_iWidth + x1] = iColor*255;

			iColor = !iColor;
		}
	}
	bSave_Image(pcFile, oImage);
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

template<typename _T>int bSolve_B_Inv_Power(_T H[][3 * 3], int iImage_Count, _T B[9])
{//通过H 借出B[9];还是解一个不定方程
	//本质是求解一个最小二乘问题组成的不定方程 Ax = 0
	//故此何以转换为 求 A'A的最小特征值对应的特征向量，用反幂法
	_T B1[6],*pCur,*A = (_T*)pMalloc(iImage_Count * 6 * 2 * sizeof(_T));
	_T AtA[6 * 6];
	int i;

	//每张图片的H矩阵能构成两条式子，重要的是搞对这两条式子
	for (i = 0; i < iImage_Count; i++)
	{
		pCur = &A[i * 6 * 2];
		Get_vij(H[i], 0, 1, pCur);	//v12
		//Disp(pCur, 1, 6, "v11");
		pCur += 6;
		_T v22[6];
		Get_vij(H[i], 0, 0, pCur);
		Get_vij(H[i], 1, 1, v22);
		Vector_Minus(pCur, v22, 6, pCur);
		//Disp(pCur, 1, 6, "v11+v22");
	}
	//求A'A
	Transpose_Multiply(A, iImage_Count * 2, 6, AtA, 0);

	int iResult=bInverse_Power(AtA,6, (_T*)NULL, B1);
	if (iResult)
	{
		//再装配成对称矩阵
		B[0] = B1[0];
		B[1] = B[3] = B1[1];
		B[4] = B1[2];
		B[6]=B[2] = B1[3];
		B[7]=B[5] = B1[4];
		B[8] = B1[5];
	}
	Free(A);
	return iResult;
}
template<typename _T>void Solve_B_SVD(_T H[][3 * 3], int iImage_Count,_T B[9])
{//通过H 借出B[9];还是解一个不定方程
	_T B1[6],*pCur,*A = (_T*)pMalloc(iImage_Count * 6 * 2 * sizeof(_T));
	int i;

	//每张图片的H矩阵能构成两条式子，重要的是搞对这两条式子
	for (i = 0; i < iImage_Count; i++)
	{
		pCur = &A[i * 6 * 2];
		Get_vij(H[i], 0, 1, pCur);	//v12
		//Disp(pCur, 1, 6, "v11");
		pCur += 6;
		_T v22[6];
		Get_vij(H[i], 0, 0, pCur);
		Get_vij(H[i], 1, 1, v22);
		Vector_Minus(pCur, v22, 6, pCur);
		//Disp(pCur, 1, 6, "v11+v22");
	}

	//Disp(A, iImage_Count * 2, 6, "A");
	SVD_Info oSVD;
	int iResult;
	SVD_Alloc<_T>(iImage_Count * 2,6, &oSVD);
	svd_3(A, oSVD, &iResult);
	memcpy(B1, &((_T*)oSVD.Vt)[5 * 6], 6 * sizeof(_T));
	//Disp(B1, 6, 1, "B");

	//再装配成对称矩阵
	B[0] = B1[0];
	B[1] = B[3] = B1[1];
	B[4] = B1[2];
	B[6]=B[2] = B1[3];
	B[7]=B[5] = B1[4];
	B[8] = B1[5];
	//Disp(B, 3, 3, "B");
	Free_SVD(&oSVD);
	Free(A);
	return;
}
template<typename _T>void Cal_K(_T H[][3 * 3],int iImage_Count, _T K[3*3])
{
	_T B[9], lamda;
	int iResult;
	//Solve_B_SVD(H, iImage_Count, B);
	iResult=bSolve_B_Inv_Power(H, iImage_Count, B);

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

	return;
}
template<typename _T>void Load_Poine_2D(const char* pcFile, int *piImage_Count, int iCorner_Per_Image,int w_In_Point, int h_In_Point, _T  (**ppCorner_Point_2D)[2])
{//装入角点数据
	int iSize;
	float *pConer_Point_2D;
	int iImage_Count;
	bLoad_Raw_Data(pcFile, (unsigned char**)&pConer_Point_2D, &iSize);
	iImage_Count = iSize / (iCorner_Per_Image * 2 * sizeof(float));

	if (iSize != iImage_Count * iCorner_Per_Image * 2 * sizeof(float))
	{
		printf("Invalid size:%d\n", iSize);
		return;
	}
	_T (*pCur_Corner)[2],(*pConer_Point_2D_1)[2] = (_T(*)[2])pMalloc(iImage_Count * iCorner_Per_Image * 2 * sizeof(_T));
	for (int i = 0; i < iImage_Count; i++)
	{
		pCur_Corner = &pConer_Point_2D_1[i * iCorner_Per_Image];
		for (int j = 0; j < iCorner_Per_Image; j++)
		{
			int x = j / h_In_Point,
				y = j % h_In_Point;
			pCur_Corner[y*w_In_Point +x][0] =  ((float(*)[2])pConer_Point_2D)[i*iCorner_Per_Image+j][0],
				pCur_Corner[y*w_In_Point +x][1] = ((float(*)[2])pConer_Point_2D)[i*iCorner_Per_Image+j][1];
		}
		//pConer_Point_2D_1[i] = pConer_Point_2D[i];
	}
	*ppCorner_Point_2D = (_T(*)[2])pConer_Point_2D_1;
	*piImage_Count = iImage_Count;
	free(pConer_Point_2D);
	return;
}

template<typename _T>void Gen_Corner_3D(int w_In_Point, int h_In_Point,_T fGrid_Size,_T (**ppCorner_3D)[2])
{//造棋盘的理论角点，世界坐标在此建立，故此z=0
	int x, y;
	_T(*pCorner_3D)[2] = (_T(*)[2])pMalloc(w_In_Point * h_In_Point * 2 * sizeof(_T));

	//以下做一张棋盘的空间点位置数据，虽然数组为2维，但是z恒为0，故此
	//这实际上是三维坐标。即U,V,Z
	for (y = 0; y < h_In_Point; y++)
		for (x = 0; x < w_In_Point; x++)
			pCorner_3D[y * w_In_Point + x][0] = x * fGrid_Size,
			pCorner_3D[y * w_In_Point + x][1] = y * fGrid_Size;
	*ppCorner_3D = pCorner_3D;
	return;
}
template<typename _T>void Cal_T(_T H[][3*3], _T K[3 * 3], int iImage_Count, _T T[][4 * 4])
{//已知H,K,求T
	//H = K(R1,R2,t), (R1,R2,t)=K(-1)*H
	int i, iResult;
	_T K_Inv[3 * 3];
	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);

	Get_K_Inv_With_gama(K, K_Inv);
	//Disp(K_Inv, 3, 3, "K_Inv");
	for (i = 0; i < iImage_Count; i++)
	{
		_T* pH = H[i];
		_T R1R2t[3 * 3];
		Matrix_Multiply(K_Inv, 3, 3, pH, 3, R1R2t);
		_T R1[3] = { R1R2t[0],R1R2t[3],R1R2t[6] },
			R2[3] = { R1R2t[1],R1R2t[4],R1R2t[7] }, R3[3];
		Cross_Product(R1, R2, R3);
		//Disp(R3, 1, 3, "R3");
		_T R[3 * 3] = { R1[0],R2[0],R3[0],
			R1[1],R2[1],R3[1],
			R1[2],R2[2],R3[2] };
		//Disp(R, 3, 3, "R");

		//此时，R并不是正交矩阵，还得根据这个R求一个真正的旋转矩阵，
		//这部分的理论待补充，只直到如何求
		//按照svd理论，S,Vt都是正交矩阵，都可以视为旋转矩阵？
		svd_3(R,oSVD, &iResult);
		Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T*)oSVD.Vt, 3, R);
		_T t[] = { R1R2t[2],R1R2t[5],R1R2t[8] };
		Gen_Homo_Matrix(R, t, T[i]);

		//Disp(T[i], 4, 4, "T");
	}
	Free_SVD(&oSVD);
	return;
}

template<typename _T>void Estimate_Distort_Coeff(_T K[3*3],_T T[4*4],_T Corner_3D[][2], _T Corner_2D[][2],int iPoint_Count, _T *pfK1, _T *pfK2)
{//估计相机畸变参数，我的方法与基准程序不大一样，单元估出来的一样
//用矛盾方程的观点看待，	u'-u =0
//							v'-v =0

	int i;
	_T* A = (_T*)pMalloc(iPoint_Count * 2 * 2* sizeof(_T));
	_T* b = (_T*)pMalloc(iPoint_Count * 2 * sizeof(_T));

	for (i = 0; i < iPoint_Count; i++)
	{
		_T Point_3D[4] = { Corner_3D[i][0],Corner_3D[i][1],0,1 };
		Matrix_Multiply(T, 4, 4, Point_3D, 1, Point_3D);

		//再投影到归一化平面上，后面有用
		Point_3D[0] /= Point_3D[2], Point_3D[1] /= Point_3D[2], Point_3D[2] = 1;

		_T UV[3];	//相当于uv'
		_T r2 = Point_3D[0] * Point_3D[0] + Point_3D[1] * Point_3D[1];

		Matrix_Multiply(K, 3, 3, Point_3D, 1, UV);
		//UV[0] /= UV[2], UV[1] /= UV[2];
		
		//Disp(UV, 1, 2, "UV");
		//fx * x * r2  k1系数
		A[i * 4] = K[0] * Point_3D[0] * r2;
		//fx* x* r2* r2;
		A[i * 4 + 1] = A[i * 2] * r2;
	
		//u - u'
		b[i*2] = Corner_2D[i][0] - UV[0];

		//fy * y * r2  k1系数
		A[i * 4 + 2] = K[5] * Point_3D[1] * r2;

		//fy* y* r2* r2;
		A[i * 4 + 3] = A[(i + 1) * 2] * r2;

		//v - v'
		b[i*2+1] =  Corner_2D[i][1] - UV[1];
	}
	_T x[2];
	int iResult;
	//Disp(A, iPoint_Count * 2, 2, "A");
	//Disp(b, iPoint_Count, 1, "b");

	Solve_Linear_Contradictory(A, iPoint_Count * 2, 2, b, x, &iResult);
	*pfK1 = x[0];
	*pfK2 = x[1];
	Free(A);
	Free(b);
	return;
}
template<typename _T>void Zhang_Get_Error(_T K[3 * 3], _T k1, _T k2, _T T[][4 * 4], _T Corner_3D[][2], _T Corner_2D[][2], int iImage_Count,int iCorner_Per_Image,_T *pfError)
{//算个误差和
	int i, j, iPos;
	_T fError = 0, fTotal = 0;
	//k1 = k2 = 0;
	for (i = 0; i < iImage_Count; i++)
	{
		//Disp(T[i], 4, 4, "T");
		for (j = 0; j < iCorner_Per_Image; j++)
		{
			/*if (Corner_3D[j][0] == 0.2)
				printf("here");*/
			iPos = i * iCorner_Per_Image + j;
			_T Point_3D[4] = { Corner_3D[j][0], Corner_3D[j][1],0,1 };
			Matrix_Multiply(T[i], 4, 4, Point_3D, 1, Point_3D);
			_T x = Point_3D[0] / Point_3D[2], y = Point_3D[1] / Point_3D[2];
			_T r2 = x * x + y * y;
			//d = 1 + r2 * (k1 + k2*r2) 
			_T d = 1 + r2 * (k1 + k2 * r2);

			//u = fx * d * x + cx
			_T u = K[0] * d * x + K[2];
			//v = fy * d * y + cy
			_T v = K[4] * d * y + K[5];

			fError = sqrt((u - Corner_2D[iPos][0]) * (u - Corner_2D[iPos][0]) + (v - Corner_2D[iPos][1]) * (v - Corner_2D[iPos][1]));
			fTotal += fError;
			_T Point_2D[3];
			Matrix_Multiply(K, 3, 3, Point_3D, 1, Point_2D);
			Point_2D[0] /= Point_2D[2], Point_2D[1] /= Point_2D[2], Point_2D[2] = 1;

			//Disp(Point_2D, 3, 1, "Point_3D");
			//printf("Point_3D:%f %f Corner_2D:%f %f Error:%f\n", Corner_3D[j][0], Corner_3D[j][1], Corner_2D[iPos][0], Corner_2D[iPos][1], fError);
		}
	}
	*pfError = fTotal / (iCorner_Per_Image * iImage_Count);

	return;
}

template<typename _T>void Get_Deriv_E_TP(_T TP[3],_T fx, _T fy, _T k1, _T k2,_T J_E_TP[2*3])
{//重新做一次，原来的好像不好使，符号反了
	_T x = TP[0] / TP[2], y = TP[1] / TP[2];
	
	_T r2 = x * x + y * y;
	_T d = k1 * r2 + k2 * r2 * r2 + 1;

	_T Part_1 = 2 * (k1 + 2 * k2 * r2);
	//du/dTPx = fx*(2*(k1 + 2k2*r2) * x*x + d)/TPz
	_T du_dTPx = fx * (Part_1 * x * x + d) / TP[2];

	//du/dTPy=2*fx*x*y*(k1 + 2k2*r2)/TPz
	_T du_dTPy = fx * x * y * Part_1 / TP[2];

	//du/dTPz=-fx*(2*(x+y)* (k1+2k2*r2)*x +d*PTx)/TPz²
	_T du_dTPz =-fx * ((x + y) * Part_1*x + d*TP[0]) / (TP[2] * TP[2]);

	//dv/dTPx = 2*fy*x*(k1 + 2k2*r2)/TPz
	//_T dv_dTPx = 2 * fy * x * (k1 + 2 * k2 * r2) / TP[2];
	
	//dv/dTPx = 2*x*y*fy*(k1 + 2K2*r2)/TPz
	_T dv_dTPx =  x * y * fy * Part_1 / TP[2];

	//dv/dTPx = fy*(2*(k1 + 2k2*r2)*y*y + d)/TPz
	_T dv_dTPy = fy * (Part_1* y * y + d) / TP[2];

	//dv/dTPz = -fy*(2*(k1 + 2k2*r2)*(x+y) -d*TPy)/ (TPz*TPz)
	_T dv_dTPz = -fy * (Part_1 * (x + y)*y + d * TP[1]) / (TP[2] * TP[2]);

	J_E_TP[0] = du_dTPx;
	J_E_TP[1] = du_dTPy;
	J_E_TP[2] = du_dTPz;
	J_E_TP[3] = dv_dTPx;
	J_E_TP[4] = dv_dTPy;
	J_E_TP[5] = dv_dTPz;
	
}

template<typename _T>static void BA_PnP_Zhang_Get_J(_T Point_3D[3], _T Pose[4 * 4], _T Intrinsic[6], _T Point_2D[2], _T J_E_Ksi[2*6], _T J_E_Intrinsic[2*6], _T E[2])
{//根据相机位置，观察点，观察数据算个雅可比与误差
	_T Point_4D[4] = {Point_3D[0],Point_3D[1],Point_3D[2],1};
	_T Point_3D_1[4];
	union {
		_T Point_2D_1[3];
		_T J_UV_TP[2 * 3];  //duv/dP'
	};
	//Matrix_Multiply(Pose, 4, 4, Point_4D, 1, Point_3D_1);
	//Disp(Point_3D_1, 1, 3, "Point_3D_1");
	//展开Matrix_Multiply看看
	Point_3D_1[0] = Pose[0] * Point_4D[0] + Pose[1] * Point_4D[1] + Pose[2] * Point_4D[2] + Pose[3];
	Point_3D_1[1] = Pose[1*4+0] * Point_4D[0] + Pose[1*4+1] * Point_4D[1] + Pose[1*4+2] * Point_4D[2] + Pose[1*4+3];
	Point_3D_1[2] = Pose[2*4+0] * Point_4D[0] + Pose[2*4+1] * Point_4D[1] + Pose[2*4+2] * Point_4D[2] + Pose[2*4+3];

	_T fx = Intrinsic[0], fy = Intrinsic[1];
	_T cx = Intrinsic[2], cy = Intrinsic[3];
	_T k1 = Intrinsic[4], k2 = Intrinsic[5];	
	_T x = Point_3D_1[0] / Point_3D_1[2], y = Point_3D_1[1] / Point_3D_1[2];
	_T r2 = x * x + y * y;
	_T d = 1 + r2 * k1 + r2 * r2 * k2;
	_T J_TP_Ksi[3 * 6], J_E_TP[2 * 3];
	//_T J_E_Ksi[2 * 6];

	Get_Deriv_E_TP(Point_3D_1, fx, fy, k1, k2, J_E_TP);
	//Disp(J_E_TP, 2, 3, "J_E_TP");
		
	//验算而已
	//Get_Deriv_TP_Ksi_1(Pose, Point_3D, J_TP_Ksi);
	//Disp(J_TP_Ksi, 2, 6, "J_TP_Ksi");
	Get_Deriv_TP_Ksi(Pose, Point_3D_1, J_TP_Ksi);
	//Disp(J_TP_Ksi, 3, 6, "J_TP_Ksi");

	//得到误差对位姿的导数
	if(J_E_Ksi)
		Matrix_Multiply(J_E_TP, 2, 3, J_TP_Ksi,6, J_E_Ksi);

	//Disp(J_E_Ksi, 2, 6, "de/dksi");

	//{//Testing code
	//	_T K[3 * 3] = { fx, 0 ,cx,
	//		0, fy,cy,
	//		0, 0,1 };
	//	Get_Deriv_E_Ksi(K, Point_3D_1, J_E_Ksi);
	//	Disp(J_E_Ksi, 2, 6, "de/dksi");
	//}	

	//继续求误差对内参的导数
	//x*d	0	1	0	fx*x*r2	fx*x*r2²
	//0		y*d	0	1	fy*y*r2	fy*y*r2²
	//_T J_E_Intrinsic[2 * 6] = { x * d,0,1,0,0,0,
	//						0,y * d,0,1,0,0 };
	if (J_E_Intrinsic)
	{
		J_E_Intrinsic[0] = x * d;
		J_E_Intrinsic[1] = 0;
		J_E_Intrinsic[2] = 1;
		J_E_Intrinsic[3] = 0;
		J_E_Intrinsic[4] = fx * x * r2;
		J_E_Intrinsic[5] = J_E_Intrinsic[4] * r2;

		J_E_Intrinsic[6] = 0;
		J_E_Intrinsic[7] = y*d;
		J_E_Intrinsic[8] = 0;
		J_E_Intrinsic[9] = 1;
		J_E_Intrinsic[10] = fy * y * r2;
		J_E_Intrinsic[11] = J_E_Intrinsic[10] * r2;
	}
	//Disp(J_E_Intrinsic, 2, 6, "de/dIntrinsic");

	_T U = fx * d * x + cx,
		V = fy * d * y + cy;

	E[0] = U - Point_2D[0];
	E[1] = V - Point_2D[1];

	return;
}

template<typename _T>void BA_PnP_Zhang_LM_Ceres(
	_T A[], int iOrder, _T b[], _T J_All[], int iObservation_Count, _T x[], LM_Param_Ceres<_T>* poParam, int* pbResult,  //第一部分参数，解方程必备
	//第二部分参数，该问题的额外需要数据
	_T Pose[][16], int iCamera_Count, _T Intrinsic[6], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation[], int iPoint_2D_Count)
{//为BA_PnP_3D_2D_Pose提供LM解线性方程
 //先用g2o验证一下收敛与精度
	//要推翻重来！！、

	LM_Param_Ceres<_T> oParam = *poParam;
	int i, bResult = 0;

	//此处关键怎样更新radius
	if (oParam.m_iIter == 0)
	{
		oParam.radius = 10000.f;
		oParam.m_pDiag =(_T*)pMalloc(iOrder * sizeof(_T));
	}
	
	if (!oParam.reuse_diagonal)
	{
		for (i = 0; i < iOrder; i++)
		{
			oParam.m_pDiag[i] = A[i * iOrder + i];	// sqrt(Abs(A[i * iOrder + i]) / oParam.radius);
			//限个幅
			oParam.m_pDiag[i] = Clip3(1e-6, 1e32, oParam.m_pDiag[i]); 
		}
	}

	_T* lm_diagonal = (_T*)pMalloc(iOrder * sizeof(_T));
	for (i = 0; i < iOrder; i++)
		lm_diagonal[i] = sqrt(oParam.m_pDiag[i] / oParam.radius);

	int	qmax = 0;
	_T(*pPose_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));
	_T Intrinsic_1[6];

	memcpy(pPose_1, Pose, iCamera_Count * 16 * sizeof(_T));
	memcpy(Intrinsic_1, Intrinsic, 6 * sizeof(_T));

	//第一步，修改对角线元素，这一步与g2o有根本区别，g2o对角线加一个统一的值
	for (i = 0; i < iOrder; i++)
		A[i * iOrder + i] +=lm_diagonal[i];   // *Diag[i];

	Solve_Linear_Gause(A, iOrder, b, x, &bResult);

	//恢复方程
	for (i = 0; i < iOrder; i++)
		A[i * iOrder + i] -= lm_diagonal[i];   // *Diag[i];

	//接着求modal_residual
	// 留心看所解方程J'Jx = -J'e, 这和矛盾方程组仅差一个负号
	//优化问题可等价为 Jx = -e 这个矛盾方程组
	//求出x以后，用j'*x 可得一个e', 这个e'与 -e最接近，这个就是modal_residual
	//故此，要重新求一次J
	_T* px_Intrinsic = &x[iCamera_Count * 6];
	_T* modal_residual = (_T*)pMalloc(iPoint_2D_Count * 2 * sizeof(_T));
	_T* residual = (_T*)pMalloc(iPoint_2D_Count * 2 * sizeof(_T));
	_T model_cost_change = 0;
	for (i = 0; i < iPoint_2D_Count; i++)
	{
		Point_2D<_T> oPoint_2D = Observation[i];
		_T* pPoint_3D = Point_3D[oPoint_2D.m_iPoint_Index];
		_T E1[2],J_E_Ksi[2 * 6], J_E_Intrinsic[2 * 6];
		BA_PnP_Zhang_Get_J(Point_3D[oPoint_2D.m_iPoint_Index], Pose[oPoint_2D.m_iCamera_Index],
			Intrinsic, oPoint_2D.m_Pos, J_E_Ksi, J_E_Intrinsic,E1);

		Disp(J_E_Ksi, 2, 6, "J_E_Ksi");
		Disp(J_E_Intrinsic, 2, 6, "J_E_Intrinsic");
		memcpy(&residual[i * 2], E1, 2 * sizeof(_T));
		
		_T E2[2], E3[2];
		Matrix_Multiply(J_E_Ksi, 2, 6, &x[oPoint_2D.m_iCamera_Index * 6], 1, E2);
		Matrix_Multiply(J_E_Intrinsic, 2, 6, px_Intrinsic, 1, E3);
		Vector_Add(E2, E3, 2, E2);
		memcpy(&modal_residual[i * 2], E2, 2 * sizeof(_T));

		E1[0] = (E1[0] + E2[0] / 2.f);
		E1[1] = (E1[1] + E2[1] / 2.f);
		//Disp(E1, 2, 1, "E2");

		model_cost_change += -E2[0] * E1[0] + (-E2[1] * E1[1]);
	}

	int bStep_is_valid = model_cost_change > 0;
	_T* pDelta = (_T*)pMalloc(iOrder * sizeof(_T));
	memcpy(pDelta, x, iOrder * sizeof(_T));

	/*Disp((_T*)Pose, 1, 6, "Pose");
	Disp((_T*)Intrinsic, 1, 6, "Intrinsic");*/
	//此处要更新一下参数，相当于求	candidate_x_
	_T* candidate_x_ = (_T*)pMalloc(iOrder * sizeof(_T));

	//先更新一下Camera,即把x加上去
	Vector_Add((_T*)Pose, x, iOrder - 6,candidate_x_);
	Vector_Add(Intrinsic_1, &x[iOrder - 6], 6, &candidate_x_[iOrder - 6]);
	//Disp(candidate_x_, iOrder, 1, "candidate_x_");
	
	//再求一次误差
	//先更新一下Camera,即把x加上去
	memcpy(pPose_1, Pose, iCamera_Count*16 * sizeof(_T));
	memcpy(Intrinsic_1, Intrinsic, 6 * sizeof(_T));
	for (i = 0; i < iCamera_Count; i++)
	{
		_T* pPose_Delta_6 = &x[i * 6];
		_T Pose_Delta_4x4[4 * 4]; 
		//Disp(pPose_Delta_6, 6, 1, "pPose_Delta_6");
		//此处更新很有可能是错的！！！
		se3_2_SE3(pPose_Delta_6, Pose_Delta_4x4);
		Matrix_Multiply(Pose_Delta_4x4, 4, 4, pPose_1[i], 4, pPose_1[i]);
	}
	//对内参也进行修正
	Vector_Add(Intrinsic_1, &x[iOrder-6], 6, Intrinsic_1);
	_T candidate_cost = 0;
	for (i = 0; i < iPoint_2D_Count; i++)
	{
		Point_2D<_T> oPoint_2D = Observation[i];
		_T E1[2];
		_T J_E_Ksi[6], J_E_Intrinsic[6];
		BA_PnP_Zhang_Get_J(Point_3D[oPoint_2D.m_iPoint_Index], pPose_1[oPoint_2D.m_iCamera_Index],
			Intrinsic_1, oPoint_2D.m_Pos, J_E_Ksi, J_E_Intrinsic, E1);
		candidate_cost += E1[0] * E1[0] + E1[1] * E1[1];
	}


	//此处要做一个最麻烦的判断，更新一些状态
	_T relative_decrease;
	int bIsStepSuccessful;
	if (oParam.current_cost > MAX_FLOAT)
		relative_decrease = -MAX_FLOAT;
	//此处乘以1/2？
	//relative_decrease = (oParam.m_fCost - candidate_cost_)*0.5 / model_cost_change_;
	relative_decrease = (oParam.current_cost - candidate_cost) / model_cost_change;
	const double historical_relative_decrease =
		(oParam.reference_cost - oParam.candidate_cost) /
		(oParam.accumulated_reference_model_cost_change + model_cost_change);
	relative_decrease = Max(relative_decrease, historical_relative_decrease);
	bIsStepSuccessful = relative_decrease >	0.001;

	//注意，relative_decrease也是Step Quality
	// radius_ = radius_ / std::max(1.0 / 3.0, 1.0 - pow(2.0 * step_quality - 1.0, 3));
	if (bIsStepSuccessful)
	{
		oParam.radius= oParam.radius/std::max(1.0 / 3.0, 1.0 - pow(relative_decrease - 1.0, 3));
		oParam.radius = std::min(10000000000000000., oParam.radius);
		oParam.decrease_factor = 2.f;
		oParam.reuse_diagonal = 0;

		//candidate_cost_, model_cost_change_
		oParam.current_cost = candidate_cost;
		oParam.accumulated_candidate_model_cost_change += model_cost_change;
		oParam.accumulated_reference_model_cost_change += model_cost_change;
		if (oParam.current_cost < oParam.minimum_cost)
		{
			oParam.candidate_cost = oParam.current_cost;
			oParam.accumulated_candidate_model_cost_change = 0;
		}
		else
		{	//这个奇怪
			//printf("here");
			oParam.num_consecutive_nonmonotonic_steps++;
			if (oParam.current_cost > candidate_cost)
			{
				oParam.candidate_cost = oParam.current_cost;
				oParam.accumulated_candidate_model_cost_change = 0.0;
			}
		}
		if (oParam.num_consecutive_nonmonotonic_steps == 0)
		{
			oParam.reference_cost = candidate_cost;
			oParam.accumulated_reference_model_cost_change =
				oParam.accumulated_candidate_model_cost_change;
		}
	}else
	{//失败
		oParam.radius = oParam.radius / oParam.decrease_factor;
		oParam.decrease_factor *= 2.0;
		oParam.reuse_diagonal = 1;
	}

	*poParam = oParam;
	return;
}
template<typename _T>void BA_PnP_Zhang_LM_g2o(
	_T A[], int iOrder, _T b[], _T x[], LM_Param_g2o<_T>* poParam, int* pbResult,  //第一部分参数，解方程必备
	//第二部分参数，该问题的额外需要数据
	_T Pose[][16], int iCamera_Count, _T Intrinsic[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation[], int iPoint_2D_Count)
{//为BA_PnP_3D_2D_Pose提供LM解线性方程
 //先用g2o验证一下收敛与精度
	int i, bResult = 1;
	LM_Param_g2o<_T> oParam = *poParam;
	const int iMax_Size = 256;
	_T Buffer[iMax_Size],
		* f = Buffer;

	if (oParam.m_iIter == 0)
	{//第一次，特殊对待,取对角线最大值		
		_T tau = (_T)1e-5, fMax = (_T)0.f;
		for (i = 0; i < iOrder; i++)
			if (Abs(A[i * iOrder + i]) > fMax)
				fMax = Abs(A[i * iOrder + i]);
		//第一次迭代的时候，Lamda定位 0.00001*对角线最大元
		oParam.Lamda = tau * fMax;
		oParam._ni = 2;
	}

	_T rho = 0;
	int	qmax = 0;
	_T(*pPose_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));
	_T Intrinsic_1[6];

	do {
		//oParam.Lamda = 1;     //临时设置一下
		//解分方程看看        
		Add_I_Matrix(A, iOrder, oParam.Lamda);
		Solve_Linear_Gause(A, iOrder, b, x, &bResult);

		//恢复方程
		Add_I_Matrix(A, iOrder, -oParam.Lamda);

		memcpy(pPose_1, Pose, iCamera_Count*16 * sizeof(_T));
		memcpy(Intrinsic_1, Intrinsic, 6 * sizeof(_T));

		//先更新一下Camera,即把x加上去
		for (i = 0; i < iCamera_Count; i++)
		{
			_T* pPose_Delta_6 = &x[i * 6];
			_T Pose_Delta_4x4[4 * 4]; 
			//此处更新很有可能是错的！！！
			se3_2_SE3(pPose_Delta_6, Pose_Delta_4x4);
			Matrix_Multiply(Pose_Delta_4x4, 4, 4, pPose_1[i], 4, pPose_1[i]);
		}

		//对内参也进行修正
		Vector_Add(Intrinsic_1, &x[iOrder-6], 6, Intrinsic_1);

		//再将所有的点算一遍，求误差
		_T fSum_e = 0;
		for (i = 0; i < iPoint_2D_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation[i];
			_T E[2];
			BA_PnP_Zhang_Get_J(Point_3D[oPoint_2D.m_iPoint_Index],
				pPose_1[oPoint_2D.m_iCamera_Index],Intrinsic_1, oPoint_2D.m_Pos, (_T*)NULL, (_T*)NULL, E);
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
		}

		if (!bResult)   //没啥营养，就是为了跳出
			fSum_e = std::numeric_limits<_T>::max();

		//rho不过是个表示发散/收敛程度的参数
		rho = (oParam.m_fLoss - fSum_e);

		//搞个scale
		_T fScale;
		if (bResult)
		{
			fScale = (_T)1e-3;
			//从此处可以看出，Scale在解方程成功的时候起作用
			//可以想象，x就是增长步长，不断收敛，故此，fScale也不断减少
			for (int i = 0; i < iOrder; i++)
				fScale += x[i] * (oParam.Lamda * x[i] + b[i]);
		}else
			fScale = 1;

		rho /= fScale;  //显然, rho>0时表示解收敛

		if (rho > 0 && std::_Is_finite(fSum_e) && bResult)
		{//本次迭代的最后一次
		 //fScale递减，rho增大，pow增大，alpha减少，scaleFactor减少
			_T alpha = (_T)(1. - pow((2 * rho - 1), 3));
			alpha = Min(alpha, 2.f/3.f);
			_T scaleFactor = Max(1.f/3.f, alpha);

			//可见，scaleFactor在[1/3,2/3]之间，Lamda必然缩小
			oParam.Lamda *= scaleFactor;
			oParam._ni = 2;
			oParam.m_fLoss = fSum_e;
		}else
		{//加入对角线调整失败，则继续改下去
			oParam.Lamda*=oParam._ni;     //等于将原来的Lamda增大_ni倍，_ni则成倍增大  
			oParam._ni *= 2;            //理论上，对_ni加倍增大
			if (!std::_Is_finite(oParam.Lamda)) 
				break;
		}
		qmax++;

	}while (rho < 0  && qmax<10 && !poParam->m_bStop);	//此处本来还有个外部干涉信号可控制停止
	*pbResult = bResult;

	if (qmax >= 10)
		printf("Fail to converge:%d\n", qmax);
	else
		printf("qmax:%d\n", qmax);

	//若成功，则直接更新Camera了事，省了后续的更新
	memcpy((_T*)Pose, pPose_1, iCamera_Count * 16 * sizeof(_T));
	memcpy(Intrinsic, Intrinsic_1, 6 * sizeof(_T));

	*poParam = oParam;
	Free(pPose_1);

	return;
}
template<typename _T>static void Get_J_Residual(_T Pose[][16], int iCamera_Count, _T Intrinsic[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count,
	_T J[][2][12], _T Residual[][2],_T *pfSum_e)
{//先对所有样本求个导
	int i;
	_T fSum_e = 0;
	for (i = 0; i < iObservation_Count; i++)
	{
		/*if (i == 263)
			printf("here");*/
		Point_2D<_T> oPoint_2D = Observation_2D[i];
		_T J_E_Ksi[2 * 6], J_E_Intrinsic[2 * 6];

		BA_PnP_Zhang_Get_J(Point_3D[oPoint_2D.m_iPoint_Index], Pose[oPoint_2D.m_iCamera_Index], Intrinsic, oPoint_2D.m_Pos, 
			J_E_Ksi,J_E_Intrinsic, Residual[i]);
		fSum_e += Residual[i][0] * Residual[i][0] + Residual[i][1] * Residual[i][1];
		Copy_Matrix_Partial(J_E_Ksi, 2, 6, (_T*)J[i], 12, 0, 0);
		Copy_Matrix_Partial(J_E_Intrinsic, 2, 6, (_T*)J[i], 12, 6, 0);
	}

	*pfSum_e = fSum_e/2.f;
	return;
}
template<typename _T>static void Get_JtE(_T J[][2][12], _T Residual[][2], Point_2D<_T> Observation[], int iCamera_Count, int iObservation_Count,_T JtE[])
{
	int i, iIntrinsic_Pos = iCamera_Count * 6;
	_T JtE_1[12];
	memset(JtE,0, (iCamera_Count * 6 + 6) * sizeof(_T));
	_T* pIntrinsic = &JtE[iIntrinsic_Pos];

	for (i = 0; i < iObservation_Count; i++)
	{
		Point_2D<_T> oPoint = Observation[i];
		////此处看上去很慢，干它！
		//Matrix_Transpose((_T*)J[i], 2, 12, Jt);
		//Matrix_Multiply(Jt, 12, 2, Residual[i], 1, JtE_1);
		//Disp(JtE_1, 12, 1,"JtE");
		for (int j = 0; j < 12; j++)
			JtE_1[j] = J[i][0][j] * Residual[i][0] + J[i][1][j] * Residual[i][1];
		Vector_Add(&JtE[oPoint.m_iCamera_Index * 6], JtE_1, 6, &JtE[oPoint.m_iCamera_Index * 6]);
		Vector_Add(&JtE[iIntrinsic_Pos], &JtE_1[6], 6, &JtE[iIntrinsic_Pos]);
		/*_T* pJtE = &JtE[oPoint.m_iCamera_Index * 6];
		for (int j = 0; j < 6; j++)
			pJtE[j]+=J[i][0][j] *  Residual[i][0] + J[i][1][j] * Residual[i][1];
		for(int j=0;j<6;j++)
			pIntrinsic[j]+=J[i][0][j+6] *  Residual[i][0] + J[i][1][j+6] * Residual[i][1];*/
	}
	//记得乘以-1才是Ax = b中的b
	int iOrder = iCamera_Count * 6 + 6;
	for (i = 0; i < iOrder; i++)
		JtE[i] = -JtE[i];
	return;
}
template<typename _T>static void Get_Diag(LM_Param_Ceres<_T> oParam, int iOrder, _T lm_diagonal[])
{
	int i;
	//先来个对角线
	oParam.m_pDiag[0] = 1;
	if (!oParam.reuse_diagonal)
	{
		for (i = 0; i < iOrder; i++)
		{
			oParam.m_pDiag[i] = oParam.H[i * iOrder + i];	// sqrt(Abs(A[i * iOrder + i]) / oParam.radius);
			//限个幅
			oParam.m_pDiag[i] = Clip3(1e-6, 1e32, oParam.m_pDiag[i]);
		}
	}
	for (i = 0; i < iOrder; i++)
		lm_diagonal[i] = sqrt(oParam.m_pDiag[i] / oParam.radius);
}
template<typename _T>_T fGet_model_cost_change(Point_2D<_T> Observation[], int iObservation_Count,_T Residual[][2],_T J[][2][12], _T x[], _T JtE[],int iCamera_Count)
{
	int iIntrinsic_Pos = iCamera_Count * 6;
	_T model_cost_change = 0;
	_T x1[12];
	memcpy(&x1[6], &x[iIntrinsic_Pos], 6 * sizeof(_T));
	for (int i = 0; i < iObservation_Count; i++)
	{
		//此处有重大改动，否则改了原来的Residual，是一个Bug
		//_T* E1 = Residual[i];
		_T E1[] = { Residual[i][0],Residual[i][1] };
		Point_2D<_T> oPoint = Observation[i];

		_T E2[2];//, x1[12];
		memcpy(x1, &x[oPoint.m_iCamera_Index * 6], 6 * sizeof(_T));
		//memcpy(&x1[6], &x[iIntrinsic_Pos], 6 * sizeof(_T));
		Matrix_Multiply((_T*)J[i], 2, 12, x1, 1, E2);

		E1[0] = (E1[0] + E2[0] / 2.f);
		E1[1] = (E1[1] + E2[1] / 2.f);
		//Disp(E1, 2, 1, "E2");
		model_cost_change += -E2[0] * E1[0] + (-E2[1] * E1[1]);
	}

	return model_cost_change;
}
template<typename _T>_T fGet_candidate_cost(_T Pose[][4 * 4], _T Intrinsic[6], Point_2D<_T> Observation[], _T x[], _T Point_3D[][3], 
	int iCamera_Count, int iPoint_3D_Count, int iObservation_Count, _T J[][2][12], _T Residual[][2])
{//由于ceres很难跟，干脆自己做一个看看
//x为解出来的扰动，加上扰动，看误差到
	_T candidate_cost = 0;
	int i;
	for (i = 0; i < iCamera_Count; i++)
	{
		_T* pPose_Delta_6 = &x[i * 6];
		_T Pose_Delta_4x4[4 * 4]; 
		//Disp(pPose_Delta_6, 6, 1, "pPose_Delta_6");
		//此处更新很有可能是错的！！！
		se3_2_SE3(pPose_Delta_6, Pose_Delta_4x4);
		Matrix_Multiply(Pose_Delta_4x4, 4, 4, Pose[i], 4, Pose[i]);
	}
	//对内参也进行修正
	Vector_Add(Intrinsic, &x[iCamera_Count*6], 6, Intrinsic);
	Get_J_Residual(Pose, iCamera_Count, Intrinsic, Point_3D, iPoint_3D_Count, Observation, iObservation_Count,
		J, Residual,&candidate_cost);

	return candidate_cost;
}

template<typename _T>void Update_Param_Ceres(LM_Param_Ceres<_T> *poParam,_T candidate_cost,_T model_cost_change)
{//更新一下Cerer的Param

	LM_Param_Ceres<_T> oParam = *poParam;
	_T relative_decrease;
	if (oParam.current_cost > MAX_FLOAT)
		relative_decrease = -MAX_FLOAT;
	/*if (model_cost_change == 0)
	printf("Error");*/
	relative_decrease = (oParam.current_cost - candidate_cost) / model_cost_change;
	const _T historical_relative_decrease =
		(oParam.reference_cost - candidate_cost) /
		(oParam.accumulated_reference_model_cost_change + model_cost_change);
	//printf("relative_decrease:%f\n", relative_decrease);
	relative_decrease = Max(relative_decrease, historical_relative_decrease);
	//printf(" %f\n", relative_decrease);
	oParam.bIsStepSuccessful = relative_decrease >	0.001;

	//注意，relative_decrease也是Step Quality
	if (oParam.bIsStepSuccessful)
	{
		oParam.radius=(_T)(oParam.radius/std::max(1.0 / 3.0, 1.0 - pow(2.f*relative_decrease - 1.0, 3)));
		oParam.radius = (_T)Min(10000000000000000., oParam.radius);
		//printf("radius:%f relative_decrease:%f\n", oParam.radius,relative_decrease);

		oParam.decrease_factor = 2.f;
		oParam.reuse_diagonal = 0;

		//candidate_cost_, model_cost_change_
		oParam.current_cost = candidate_cost;
		oParam.accumulated_candidate_model_cost_change += model_cost_change;
		oParam.accumulated_reference_model_cost_change += model_cost_change;
		//printf("minimum_cost:%f\n", oParam.minimum_cost);
		if (oParam.current_cost < oParam.minimum_cost)
		{
			oParam.minimum_cost = oParam.current_cost;
			oParam.candidate_cost = oParam.current_cost;
			oParam.accumulated_candidate_model_cost_change = 0;
		}else
		{	//这个奇怪
			//printf("here");
			oParam.num_consecutive_nonmonotonic_steps++;
			if (oParam.current_cost > candidate_cost)
			{
				oParam.candidate_cost = oParam.current_cost;
				oParam.accumulated_candidate_model_cost_change = 0.0;
			}
		}
		if (oParam.num_consecutive_nonmonotonic_steps == 0)
		{
			oParam.reference_cost = candidate_cost;
			oParam.accumulated_reference_model_cost_change =
				oParam.accumulated_candidate_model_cost_change;
		}
	}else
	{//失败
		oParam.radius = oParam.radius / oParam.decrease_factor;
		oParam.decrease_factor *= 2.0;
		oParam.reuse_diagonal = 1;
	}
	*poParam = oParam;

	
	//LM_Param_Ceres<_T> oParam = *poParam;
	//_T relative_decrease;
	//if (oParam.current_cost > MAX_FLOAT)
	//	relative_decrease = -MAX_FLOAT;
	///*if (model_cost_change == 0)
	//	printf("Error");*/
	//relative_decrease = (oParam.current_cost - candidate_cost) / model_cost_change;
	//const _T historical_relative_decrease =
	//	(oParam.reference_cost - candidate_cost) /
	//	(oParam.accumulated_reference_model_cost_change + model_cost_change);

	//relative_decrease = Max(relative_decrease, historical_relative_decrease);
	//oParam.bIsStepSuccessful = relative_decrease >	0.001;

	////注意，relative_decrease也是Step Quality
	//if (oParam.bIsStepSuccessful)
	//{
	//	oParam.radius= oParam.radius/std::max(1.0 / 3.0, 1.0 - pow(relative_decrease - 1.0, 3));
	//	oParam.radius = std::min(10000000000000000., oParam.radius);
	//	//printf("radius:%f relative_decrease:%f\n", oParam.radius,relative_decrease);

	//	oParam.decrease_factor = 2.f;
	//	oParam.reuse_diagonal = 0;

	//	//candidate_cost_, model_cost_change_
	//	oParam.current_cost = candidate_cost;
	//	oParam.accumulated_candidate_model_cost_change += model_cost_change;
	//	oParam.accumulated_reference_model_cost_change += model_cost_change;
	//	if (oParam.current_cost < oParam.minimum_cost)
	//	{
	//		oParam.candidate_cost = oParam.current_cost;
	//		oParam.accumulated_candidate_model_cost_change = 0;
	//	}
	//	else
	//	{	//这个奇怪
	//		printf("here");
	//		oParam.num_consecutive_nonmonotonic_steps++;
	//		if (oParam.current_cost > candidate_cost)
	//		{
	//			oParam.candidate_cost = oParam.current_cost;
	//			oParam.accumulated_candidate_model_cost_change = 0.0;
	//		}
	//	}
	//	if (oParam.num_consecutive_nonmonotonic_steps == 0)
	//	{
	//		oParam.reference_cost = candidate_cost;
	//		oParam.accumulated_reference_model_cost_change =
	//			oParam.accumulated_candidate_model_cost_change;
	//	}
	//}else
	//{//失败
	//	oParam.radius = oParam.radius / oParam.decrease_factor;
	//	oParam.decrease_factor *= 2.0;
	//	oParam.reuse_diagonal = 1;
	//}
	////if (oParam.m_iIter > 39)
	//	//printf("current cost:%f candidate_cost:%f\n", oParam.current_cost, candidate_cost);
	//*poParam = oParam;
}

template<typename _T>void Get_H_Block(_T J[2][12], _T Camera[6 * 6], _T Cam_Corner[6 * 6], _T Corner[6 * 6])
{//单独拎出来搞块
	int i,j,iDest_Pos;

	for (i = 0; i < 6; i++)
	{
		iDest_Pos = i * 6 + i;
		for (j = i; j < 6; j++,iDest_Pos++)
		{
			//iDest_Pos = i * 6 + j;
			Camera[iDest_Pos] += J[0][i] * J[0][j] + J[1][i] * J[1][j];
			Corner[iDest_Pos] += J[0][i+6]*J[0][j+6]+ J[1][i+6] * J[1][j+6];
		}
	}

	for(i=0;i<6;i++)
		for(j=0;j<6;j++)
			Cam_Corner[i * 6 + j] += J[0][i] * J[0][j + 6] + J[1][i] * J[1][j + 6];
	return;
#undef Add_To_Block
}
template<typename _T>static void Get_H_1(_T J[][2][12], Point_2D<_T>Observation[], int iCamera_Count, int iCorner_Per_Image, _T Sigma_H[])
{
	int i, j,x,y;
	int iWidth_H = iCamera_Count * 6 + 6;
	//_T Camera_1[6 * 6], Cam_Corner_1[6 * 6], Corner_1[6 * 6];
	memset(Sigma_H, 0, iWidth_H * iWidth_H * sizeof(_T));
	//一个JtJ分4块，只搞3块
	//unsigned long long tStart = iGet_Tick_Count();
	_T Corner[6 * 6] = { 0 };
	//for (int k = 0; k < 1000000; k++)
	{
		for (i = 0; i < iCamera_Count; i++)
		{
			_T Camera[6 * 6] = { 0 }, Cam_Corner[6 * 6] = { 0 };
			for (j = 0; j < iCorner_Per_Image; j++)
				Get_H_Block(J[i * iCorner_Per_Image+j], Camera, Cam_Corner, Corner);

			//补下三角
			for (y = 1; y < 6; y++)
			{
				for (x = 0; x < y; x++)
				{
					int iDest_Pos = y * 6 + x,
						iSource_Pos = x * 6 + y;
					Camera[iDest_Pos] = Camera[iSource_Pos];
					Corner[iDest_Pos] = Corner[iSource_Pos];
				}
			}

			//将块拷到sigma_H
			_T* pDest_1 = &Sigma_H[(i*6) * iWidth_H + (i * 6)],
				*pDest_2 = &Sigma_H[(i*6) * iWidth_H + iWidth_H - 6];
			for (y = 0; y < 6; y++,pDest_1+=iWidth_H,pDest_2+=iWidth_H)
			{
				memcpy(pDest_1, &Camera[y * 6], 6 * sizeof(_T));
				memcpy(pDest_2, &Cam_Corner[y * 6], 6 * sizeof(_T));
			}
						
			pDest_1 = &Sigma_H[(iWidth_H - 6) * iWidth_H + i * 6];
			pDest_2 = &Sigma_H[(iWidth_H - 6) * iWidth_H + iWidth_H - 6];
			for (y = 0; y < 6; y++,pDest_1+=iWidth_H,pDest_2+=iWidth_H)
			{
				for (x = 0; x < 6; x++)
					pDest_1[x] = Cam_Corner[x * 6 + y];
				memcpy(pDest_2, &Corner[y * 6], 6 * sizeof(_T));
			}
		}
	}
	//Disp(Sigma_H, iWidth_H, iWidth_H, "Sigma_H");
	//printf("%lld\n", iGet_Tick_Count() - tStart);
	//exit(0);
	return;
}
template<typename _T>void Get_H(_T J[][2][12], Point_2D<_T> Observation[], int iCamera_Count, int iObservation_Count,_T Sigma_H[])
{
	int i, iOrder = iCamera_Count * 6 + 6, iIntrinsic_Pos = iOrder - 6;
	memset(Sigma_H, 0, iOrder * iOrder * sizeof(_T));
	for (i = 0; i < iObservation_Count; i++)
	{
		Point_2D<_T> oPoint = Observation[i];
		_T H[12 * 12];
		Transpose_Multiply((_T*)J[i], 2, 12, H, 0);
		//Disp(H, 12, 12, "H");
		//分四部分将H分块拷贝到Sigma_H中
		Matrix_Add_Partial(H, 12, 0, 0, 6, 6, Sigma_H, iOrder, oPoint.m_iCamera_Index * 6, oPoint.m_iCamera_Index * 6);
		Matrix_Add_Partial(H, 12, 6, 0, 6, 6, Sigma_H, iOrder, iIntrinsic_Pos, oPoint.m_iCamera_Index * 6);
		Matrix_Add_Partial(H, 12, 0, 6, 6, 6, Sigma_H, iOrder, oPoint.m_iCamera_Index * 6, iIntrinsic_Pos);
		Matrix_Add_Partial(H, 12, 6, 6, 6, 6, Sigma_H, iOrder, iIntrinsic_Pos, iIntrinsic_Pos);
	}
	//Disp(Sigma_H, iOrder, iOrder, "Sigma_H");
	return;
}

template<typename _T>void BA_PnP_Zhang_LM_Ceres_1(LM_Param_Ceres<_T> *poParam,_T Pose[][16], int iCamera_Count, _T Intrinsic[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
{//重新做一个LM，此处包括几部分，1，解矛盾方程Jx = e;	2,更新Param各种数据状态，3，确定是否成功
	LM_Param_Ceres<_T>oParam = *poParam;
	int i, bResult,iOrder = iCamera_Count * 6 + 6;
	//开内存
	_T* lm_diagonal = (_T*)pMalloc(iOrder * sizeof(_T));
	_T* x = (_T*)pMalloc(iOrder * sizeof(_T));
	_T(*pPose_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iCamera_Count * 16 * sizeof(_T));
	_T Intrinsic_1[6];
	_T(*J_1)[2][12] = (_T(*)[2][12])pMalloc(iObservation_Count * 2 * 12 * sizeof(_T));
	_T(*Residual_1)[2] = (_T(*)[2])pMalloc(iObservation_Count * 2 * sizeof(_T));
		
	if (!oParam.reuse_diagonal)
	{//当前面失败的时候，这里可以偷懒不干
		//先搞个H矩阵 H = JtJ，不能简单一乘了之
		Get_H_1(oParam.J, Observation_2D, iCamera_Count, iObservation_Count / iCamera_Count, oParam.H);

		//Disp(oParam.H, iOrder, iOrder, "H");
		//Disp((_T*)oParam.J, iObservation_Count * 2, 12, "J");
		//再搞个JtE
		Get_JtE(oParam.J, oParam.Residual,Observation_2D,iCamera_Count, iObservation_Count,oParam.JtE);
		//Disp(oParam.JtE, iOrder, 1, "JtE");
	}
	Get_Diag(oParam, iOrder, lm_diagonal);
	//Disp(oParam.m_pDiag, iOrder, 1, "Diag");
	
	//第一步，修改对角线元素，这一步与g2o有根本区别，g2o对角线加一个统一的值
	for (i = 0; i < iOrder; i++)
		oParam.H[i * iOrder + i] += lm_diagonal[i]*lm_diagonal[i];   // *Diag[i];

	//Disp(b, iOrder, 1, "b");	
	//Solve_Linear_Gause(oParam.H, iOrder, oParam.JtE, x, &bResult);
	Solve_Linear_Gause_AAt(oParam.H, iOrder, oParam.JtE, x, &bResult);
	//Disp(&x[iCamera_Count*6], 6, 1, "intrinsic delta");
	//if (oParam.m_iIter == 2)
	//{
	//	Disp(lm_diagonal, iOrder, 1, "diag");
	//	//Disp(x, 1, iOrder, "x");
	//	//Disp(oParam.JtE, 1,iOrder, "JtE");
	//}

	/*if (oParam.m_iIter == 56)
	{
		Disp(x, iOrder, 1, "x");
	}*/
	//后面的计算是这样的思路, 既然Jx= -e，上面已经解到x,
	//那么将x代回到防城区，我们总希望Jx最接近 -e, 当然会有误差
	//这个误差记为model_cost_change
	_T model_cost_change = fGet_model_cost_change(Observation_2D, iObservation_Count,oParam.Residual, oParam.J, x, oParam.JtE,iCamera_Count);
	int bStep_is_valid = model_cost_change > 0;
	oParam.bStep_is_valid = bStep_is_valid;
	//printf("iter:%d model_cost_change:%f\n",oParam.m_iIter, model_cost_change);

	memcpy(pPose_1, Pose, iCamera_Count*16 * sizeof(_T));
	memcpy(Intrinsic_1, Intrinsic, 6 * sizeof(_T));
	//再求个candidate_cost
	_T candidate_cost = fGet_candidate_cost(pPose_1, Intrinsic_1, Observation_2D, 
		x,Point_3D, iCamera_Count, iPoint_3D_Count, iObservation_Count,J_1,Residual_1);
	//Disp(pPose_1[0], 4, 4, "Pose");
	//Disp(Intrinsic_1, 1, 6, "New Intrinsic");

	Update_Param_Ceres(&oParam, candidate_cost,model_cost_change);

	//加入成功了，就把新威姿，新内参，新雅可比，新误差更新到Param中
	if (oParam.bIsStepSuccessful)
	{
		memcpy(Pose, pPose_1, iCamera_Count * 4 * 4 * sizeof(_T));
		memcpy(Intrinsic, Intrinsic_1, 6 * sizeof(_T));
		memcpy(oParam.J, J_1, iObservation_Count * 2 * 12 * sizeof(_T));
		memcpy(oParam.Residual, Residual_1, iObservation_Count * 2 * sizeof(_T));
	}else
	{//当前面失败的时候，这里必须恢复方程
		for (i = 0; i < iOrder; i++)
			oParam.H[i * iOrder + i] -= lm_diagonal[i]*lm_diagonal[i];   // *Diag[i];
	}

	if(x)Free(x);
	if (pPose_1)Free(pPose_1);
	if (lm_diagonal)Free(lm_diagonal);
	if (J_1)Free(J_1);
	if (Residual_1)Free(Residual_1);
	*poParam = oParam;
	return;
}
template<typename _T>static void Free_LM_Param_Ceres(LM_Param_Ceres<_T>* poParam)
{
	LM_Param_Ceres<_T>oParam = *poParam;
	if (oParam.J)Free(oParam.J);
	if (oParam.Residual)Free(oParam.Residual);
	if (oParam.m_pDiag)Free(oParam.m_pDiag);
	if (oParam.JtE)Free(oParam.JtE);
	if (oParam.H)Free(oParam.H);
	*poParam = {};
}
template<typename _T>static void Init_LM_Param_Ceres(LM_Param_Ceres<_T>* poParam, int iCamera_Count, int iObservation_Count)
{
	LM_Param_Ceres<_T> oParam;
	//显示初始化值，免得东一块西一块神出鬼没
	oParam.reuse_diagonal = 0;		//初始化为什么，待考
	oParam.bIsStepSuccessful = 0;	//迭代是否成功
	oParam.bStep_is_valid = 0;
	oParam.m_iIter = 0;
	oParam.num_consecutive_nonmonotonic_steps = 0;
	oParam.decrease_factor = 2.f;
	oParam.accumulated_reference_model_cost_change = 0;
	oParam.accumulated_candidate_model_cost_change = 0;
	oParam.radius = 10000.f;

	int iSize = iObservation_Count * 12 * 2 +	//J
		iObservation_Count * 2;					//Residual
	oParam.J = (_T(*)[2][12])pMalloc(iObservation_Count * 12 * 2 * sizeof(_T));
	oParam.Residual = (_T(*)[2])pMalloc(iObservation_Count * 2 * sizeof(_T));
	iSize = iCamera_Count * 6 + 6;
	oParam.m_pDiag = (_T*)pMalloc(iSize * sizeof(_T));
	oParam.JtE = (_T*)pMalloc(iSize * sizeof(_T));
	iSize *= iSize;
	oParam.H = (_T*)pMalloc(iSize * sizeof(_T));
	
	*poParam = oParam;
}
template<typename _T>static void Sort_Observation(Point_2D<_T> Observation[], int iCamera_Count, int iObservation_Count)
{//来个桶排序了事
	int i;
	union {
		int * pPoint_Per_Cam ;
		int* pStart;
	};
	/*for (i = 0; i < iObservation_Count; i++)
	{
	Point_2D<_T>oPoint = Observation[i];
	printf("Camera Index:%d Point_Index:%d Pos:%f %f\n", oPoint.m_iCamera_Index, oPoint.m_iPoint_Index, oPoint.m_Pos[0], oPoint.m_Pos[1]);
	}*/
	pPoint_Per_Cam = (int*)pMalloc(iCamera_Count * sizeof(_T));
	memset(pPoint_Per_Cam, 0, iCamera_Count * sizeof(_T));
	for (i = 0; i < iObservation_Count; i++)
		pPoint_Per_Cam[Observation[i].m_iCamera_Index]++;

	int iCur = 0;
	for (i = 0; i < iObservation_Count; i++)
	{
		int iCount=pPoint_Per_Cam[i];	
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

	/*printf("\n");
	for (i = 0; i < iObservation_Count; i++)
	{
	Point_2D<_T>oPoint = Observation[i];
	printf("Camera Index:%d Point_Index:%d Pos:%f %f\n", oPoint.m_iCamera_Index, oPoint.m_iPoint_Index, oPoint.m_Pos[0], oPoint.m_Pos[1]);
	}*/
	return;
}
template<typename _T>void BA_PnP_Zhang_Ceres(_T Pose[][16], int iCamera_Count, _T Intrinsic[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
{//重写一个，原来的已经乱了 
	//先将可能乱排的点重排
	Sort_Observation(Observation_2D,iCamera_Count, iObservation_Count);

	LM_Param_Ceres<_T> oParam;
	Init_LM_Param_Ceres(&oParam,iCamera_Count, iObservation_Count);

	//先把所有的雅可比，Residual求出来
	Get_J_Residual(Pose, iCamera_Count, Intrinsic, Point_3D, iPoint_3D_Count, Observation_2D, iObservation_Count,
		oParam.J, oParam.Residual,&oParam.current_cost);
	//Disp((_T*)oParam.Residual,88,2,"Residual");
	oParam.reference_cost = oParam.candidate_cost=oParam.minimum_cost= oParam.current_cost;
	//printf("Iter:%d Error:%e bStep_is_valid:%d bIsStepSuccessful:%d\n", oParam.m_iIter, oParam.current_cost, (int)oParam.bStep_is_valid, (int)oParam.bIsStepSuccessful);

	int iIter;
	_T fPrevious_Cost = 1e20;
	_T fLoss_Diff_eps = (_T)1e-10;
	fLoss_Diff_eps*= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大
	for (iIter = 1;; iIter++)
	{
		oParam.m_iIter = iIter;
		//if (iIter == 39)
			//printf("here");
		BA_PnP_Zhang_LM_Ceres_1(&oParam, Pose, iCamera_Count, Intrinsic, Point_3D, iPoint_3D_Count,
			Observation_2D, iObservation_Count);

		if (oParam.bIsStepSuccessful)
		{
			printf("Iter:%d Error:%e bStep_is_valid:%d bIsStepSuccessful:%d\n", oParam.m_iIter, oParam.current_cost, (int)oParam.bStep_is_valid, (int)oParam.bIsStepSuccessful);
			//printf("radius:%f\n", oParam.radius);
		}//else
			//printf("Fail\n");

		if (!oParam.bStep_is_valid && !oParam.bIsStepSuccessful)
			break;

		if (abs(oParam.current_cost - fPrevious_Cost) < fLoss_Diff_eps && oParam.bIsStepSuccessful)
		{
			printf("Iter:%d Error:%e bStep_is_valid:%d bIsStepSuccessful:%d\n", oParam.m_iIter, oParam.current_cost, (int)oParam.bStep_is_valid, (int)oParam.bIsStepSuccessful);			
			break;
		}
		fPrevious_Cost = oParam.candidate_cost;
	}
	Free_LM_Param_Ceres(&oParam);	
	return;
}

//template<typename _T>void BA_PnP_Zhang_Ceres_1(_T Pose[][16], int iCamera_Count, _T Intrinsic[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
//{//要推翻重来！
//	const int iJ_w = iCamera_Count * 6 + 6;
//	_T* J = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
//		* Jt = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
//		* JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
//		* Delta_X = (_T*)pMalloc(iJ_w * sizeof(_T)),
//		* Sigma_JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
//		* Sigma_H = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T)),
//		* JtJ = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T));
//
//	_T* J_All = (_T*)pMalloc(iJ_w * 2 * iPoint_3D_Count * sizeof(_T));
//
//
//	_T fSum_e = 0, fSum_e_Pre = (_T)1e20;
//	int i,iIter,iResult;
//	LM_Param_Ceres<_T> oLM_Param;
//	
//	_T fLoss_Diff_eps = (_T)1e-20;
//	fLoss_Diff_eps*= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大
//
//	for (iIter = 0;; iIter++)
//	{
//		fSum_e = 0;
//		//将H, JtE清理
//		memset(Sigma_H, 0, iJ_w * iJ_w * sizeof(_T));
//		memset(Sigma_JtE, 0, iJ_w * sizeof(_T));
//		for (i = 0; i < iObservation_Count; i++)
//		{
//			Point_2D<_T> oPoint_2D = Observation_2D[i];
//			_T E[2], J_E_Ksi[2 * 6], J_E_Intrinsic[2 * 6];
//			BA_PnP_Zhang_Get_J(Point_3D[oPoint_2D.m_iPoint_Index], Pose[oPoint_2D.m_iCamera_Index], Intrinsic, oPoint_2D.m_Pos, J_E_Ksi, J_E_Intrinsic, E);
//			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
//			
//			//干脆造点数据与ceres一致
//			
//			//_T J_E_Ksi[] = { 0.00000000, 0.00000000, 0.00000000, -1124.80120129, 4.33386434, 48.17759141,
//			//	0.00000000, 0.00000000, 0.00000000, 4.13418871, -1071.92958756, -40.49924782 };
//			//_T J_E_Intrinsic[] = { 0.04285365, 0.00000000, 1.00000000, 0.00000000, 0.09721098, 0.00031469,
//			//	0.00000000, -0.03776372, 0.00000000, 1.00000000, -0.08171790, -0.00026454 };
//			//_T E[] = { 2.59277549,-0.18128312 };
//			//fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
//
//			////为了与Ceres对数据，临时改成ceres一致
//			//J_E_Ksi[3] = J_E_Ksi[0], J_E_Ksi[4] = J_E_Ksi[1], J_E_Ksi[5] = J_E_Ksi[2];
//			//J_E_Ksi[9] = J_E_Ksi[6], J_E_Ksi[10] = J_E_Ksi[7], J_E_Ksi[11] = J_E_Ksi[8];
//			//J_E_Ksi[0] = J_E_Ksi[1] = J_E_Ksi[2] = J_E_Ksi[6] = J_E_Ksi[7] = J_E_Ksi[8] = 0;
//			//Disp(J_E_Ksi, 2, 6, "J_E_Ksi");
//			//Disp(J_E_Intrinsic, 2, 6, "J_E_Intrinsic");
//
//			//将三组雅可比拷到目标位置
//			memset(J, 0, 2 * iJ_w * sizeof(_T));
//			Copy_Matrix_Partial(J_E_Ksi, 2, 6, J, iJ_w, oPoint_2D.m_iCamera_Index * 6, 0);
//			Copy_Matrix_Partial(J_E_Intrinsic, 2, 6, J, iJ_w, iJ_w - 6, 0);
//			//Disp(J, 2, iJ_w, "J");
//
//			//由于Ceres奇怪的LM计算，需要把所有的J留存
//			Copy_Matrix_Partial(J, 2, iJ_w, J_All, iJ_w, 0, i * 2);
//						
//			//累加Sigma_H
//			Transpose_Multiply(J, 2, iJ_w, JtJ, 0);
//			Matrix_Add(Sigma_H, JtJ, iJ_w, Sigma_H);
//			//Disp(JtJ, iJ_w, iJ_w, "JtJ");
//
//			//累加Sigma_JtE;
//			Matrix_Transpose(J, 2, iJ_w, Jt);
//			Matrix_Multiply(Jt, iJ_w, 2, E, 1, JtE);
//			Vector_Add(Sigma_JtE, JtE, iJ_w, Sigma_JtE);
//		}
//		printf("iIter:%d %e\n", iIter, 0.5f * fSum_e);
//		//Disp(Sigma_JtE, iJ_w, 1, "Sigma_JtE");
//		//Disp(Sigma_H, iJ_w, iJ_w, "Sigma_H");
//		//Disp(J_All, iObservation_Count * 2, iJ_w, "J_All");
//
//		//加上负号，右边才是 -J'E
//		Matrix_Multiply(Sigma_JtE, 1, iJ_w, (_T)-1, Sigma_JtE);
//		oLM_Param.m_iIter = iIter, oLM_Param.current_cost = oLM_Param.reference_cost = oLM_Param.candidate_cost=oLM_Param.minimum_cost= fSum_e;
//		BA_PnP_Zhang_LM_Ceres(Sigma_H, iJ_w, Sigma_JtE,J_All,iObservation_Count, Delta_X, &oLM_Param, &iResult,
//			Pose,iCamera_Count, Intrinsic, Point_3D, iPoint_3D_Count, Observation_2D, iObservation_Count);
//
//	}
//
//	Free(J);
//	Free(Jt);
//	Free(JtE);
//	Free(Delta_X);
//	Free(Sigma_JtE);
//	Free(Sigma_H);
//	Free(JtJ);
//	return;
//}

template<typename _T>void BA_PnP_Zhang_Ref(_T Pose[][16], int iCamera_Count, _T Intrinsic[], _T Point_3D[][3], int iPoint_3D_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
{//只针对张正友相机内外参优化，先不搞结构搞数学
	const int iJ_w = iCamera_Count * 6 + 6;
	_T* J = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* Jt = (_T*)pMalloc(iJ_w * 2 * sizeof(_T)),
		* JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Delta_X = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_JtE = (_T*)pMalloc(iJ_w * sizeof(_T)),
		* Sigma_H = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T)),
		* JtJ = (_T*)pMalloc(iJ_w * iJ_w * sizeof(_T));

	_T fSum_e = 0, fSum_e_Pre = (_T)1e20;
	int i,iIter,iResult;
	LM_Param_g2o<_T> oLM_Param;
	Init_LM_Param(&oLM_Param);

	_T fLoss_Diff_eps = (_T)1e-20;
	fLoss_Diff_eps*= iObservation_Count;   //两次之间的差，与点数有关，点数越多，eps越大

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		//将H, JtE清理
		memset(Sigma_H, 0, iJ_w * iJ_w * sizeof(_T));
		memset(Sigma_JtE, 0, iJ_w * sizeof(_T));
		for (i = 0; i < iObservation_Count; i++)
		{
			Point_2D<_T> oPoint_2D = Observation_2D[i];
			_T E[2], J_E_Ksi[2 * 6], J_E_Intrinsic[2 * 6];
			BA_PnP_Zhang_Get_J(Point_3D[oPoint_2D.m_iPoint_Index], Pose[oPoint_2D.m_iCamera_Index], Intrinsic, oPoint_2D.m_Pos, J_E_Ksi,J_E_Intrinsic, E);
			fSum_e += E[0] * E[0] + E[1] * E[1];    //得到误差
			//Disp(J_E_Ksi, 2, 6, "J_E_Ksi");
			//Disp(J_E_Intrinsic, 2, 6, "J_E_Intrinsic");

			//将三组雅可比拷到目标位置
			memset(J, 0, 2 * iJ_w * sizeof(_T));
			Copy_Matrix_Partial(J_E_Ksi, 2, 6, J, iJ_w, oPoint_2D.m_iCamera_Index * 6, 0);
			Copy_Matrix_Partial(J_E_Intrinsic, 2, 6, J, iJ_w, iJ_w-6, 0);
			//Disp(J, 2, iJ_w, "J");
			
			//累加Sigma_H
			Transpose_Multiply(J, 2, iJ_w, JtJ, 0);
			Matrix_Add(Sigma_H, JtJ, iJ_w, Sigma_H);

			//累加Sigma_JtE;
			Matrix_Transpose(J, 2, iJ_w, Jt);
			Matrix_Multiply(Jt, iJ_w, 2, E, 1, JtE);
			Vector_Add(Sigma_JtE, JtE, iJ_w, Sigma_JtE);
		}
		printf("iIter:%d %e\n", iIter, 0.5f*fSum_e);

		//加上负号，右边才是 -J'E
		Matrix_Multiply(Sigma_JtE, 1, iJ_w, (_T)-1, Sigma_JtE);

		//方法2，LM方法
		oLM_Param.m_fLoss = fSum_e, oLM_Param.m_iIter = iIter;
		BA_PnP_Zhang_LM_g2o(Sigma_H, iJ_w, Sigma_JtE, Delta_X, &oLM_Param, &iResult,
			Pose,iCamera_Count, Intrinsic, Point_3D, iPoint_3D_Count, Observation_2D, iObservation_Count);

		if (!iResult)
		{
			printf("Bundle Adjust LM解失败\n");
			break;
		}
		if (fSum_e<=oLM_Param.m_fLoss  || oLM_Param.m_fLoss < fLoss_eps || Abs(oLM_Param.m_fLoss-fSum_e)<fLoss_Diff_eps)
			break;
		fSum_e_Pre = oLM_Param.m_fLoss;  
	}

	Free(J);
	Free(Jt);
	Free(JtE);
	Free(Delta_X);
	Free(Sigma_JtE);
	Free(Sigma_H);
	Free(JtJ);
	return;
}
template<typename _T>void Optimize(_T Corner_3D[][2], _T Corner_2D[][2], _T K[3 * 3], _T *pk1, _T *pk2,
	_T Pose[][4 * 4], int iImage_Count, int iCorner_Per_Image)
{//要重新自己搞一套优化
	//先将所有点变成Point_3D, Observation形式
	_T k1=*pk1, k2=*pk2;

	unsigned int i,j,iPos,iObservation_Count = iCorner_Per_Image * iImage_Count;
	_T (*pCorner_3D)[3] = (_T(*)[3])pMalloc(iCorner_Per_Image * 3 * sizeof(_T));
	Point_2D<_T>* pCorner_2D = (Point_2D<_T>*)pMalloc(iObservation_Count * sizeof(Point_2D<_T>));
	
	//把Point_3D整理好
	for (j = 0; j < (unsigned int)iCorner_Per_Image; j++)
	{
		pCorner_3D[j][0] = Corner_3D[j][0];
		pCorner_3D[j][1] = Corner_3D[j][1];
		pCorner_3D[j][2] = 0;
	}

	for (iPos=i = 0; i < (unsigned int)iImage_Count; i++)
		for (j = 0; j < (unsigned int)iCorner_Per_Image; j++,iPos++)
			pCorner_2D[iPos] = { i,j,Corner_2D[iPos][0],Corner_2D[iPos][1] };
	//Disp((_T*)pPoint_3D, iCorner_Per_Image, 3, "Point_3D");

	//再将K,k1,k2放在一个Intrinsic向量里
	//_T Intrinsic[6] = { K[0],K[4],K[2],K[5],k1,k2 };
	//_T Intrinsic[6] = { K[0],K[4],K[2],K[5],0,0 };
	////iImage_Count = 41, iCorner_Per_Image = 88;
	////iObservation_Count = iCorner_Per_Image * iImage_Count;
	//BA_PnP_Zhang_Ref(Pose, iImage_Count, Intrinsic, pCorner_3D, iCorner_Per_Image,
	//	pCorner_2D, iObservation_Count);

	_T Intrinsic[6] = { K[0],K[4],K[2],K[5],k1,k2 };
	//_T Intrinsic[6] = { K[0],K[4],K[2],K[5],1.2,1.5 };
	//iImage_Count = 41, iCorner_Per_Image = 88;
	//iObservation_Count = iCorner_Per_Image * iImage_Count;
	BA_PnP_Zhang_Ceres(Pose, iImage_Count, Intrinsic, pCorner_3D, iCorner_Per_Image,
		pCorner_2D, iObservation_Count);

	K[0] = Intrinsic[0], K[4] = Intrinsic[1];
	K[2] = Intrinsic[2], K[5] = Intrinsic[3];
	*pk1 = Intrinsic[4], * pk2 = Intrinsic[5];
	if (pCorner_3D)Free(pCorner_3D);
	if (pCorner_2D)Free(pCorner_2D);
	return;
}
template<typename _T>void Zhang(_T Corner_2D[][2], int iImage_Count, int w_In_Point,int h_In_Point, _T fGrid_Size)
{
	//先把棋盘的3D点造出来
	_T(*pCorner_3D)[2],fError;
	Gen_Corner_3D(w_In_Point, h_In_Point,fGrid_Size, &pCorner_3D);

	_T(*pH)[3 * 3] = (_T(*)[3 * 3])pMalloc(iImage_Count * 9 * sizeof(_T));
	_T(*pT)[4 * 4] = (_T(*)[4 * 4])pMalloc(iImage_Count * 16 * sizeof(_T));
	_T k1, k2;
	_T K[3 * 3];

	_T(*pCur_Corner_2D)[2];
	int i, iCorner_Per_Image = w_In_Point * h_In_Point;
	for(i=0;i<iImage_Count;i++)
	{
		pCur_Corner_2D =&Corner_2D[i * iCorner_Per_Image];
		//Zhang_Estimate_H_SVD(pCorner_3D, pCur_Corner_2D, iCorner_Per_Image, pH[i]);
		bZhang_Estimate_H_Inv_Power(pCorner_3D, pCur_Corner_2D, iCorner_Per_Image, pH[i]);
		//Disp(pH[i], 3, 3, "H");
	}
	//整个求解过程一图一H矩阵，但是内参求得只有一个
	Cal_K(pH,iImage_Count,K);
	K[1] = 0;	//K并非必须

	//再通过H,K求各个T
	Cal_T(pH, K,iImage_Count, pT);

	//Disp(K, 3, 3, "K");
	//Disp((_T*)Corner_2D, iCorner_Per_Image, 2, "Point_2D");
	Estimate_Distort_Coeff(K, pT[0], &pCorner_3D[0 * iCorner_Per_Image], &Corner_2D[0 * iCorner_Per_Image], iCorner_Per_Image,&k1,&k2);
	
	//此处虽然似乎没有参考程序的误差小，但是感觉我的方程更符合理论
	//会不会参考程序误差小只是偶然现象？
	Zhang_Get_Error(K,k1,k2, pT, pCorner_3D, Corner_2D, iImage_Count,iCorner_Per_Image, &fError);
	//printf("k1,k2 Error:%f\n", fError);

	//最后一步，优化
	//printf("k1:%f k2:%f\n", k1, k2);
	//Disp(K, 3, 3, "K");
	//iImage_Count = 1;
	Optimize(pCorner_3D, Corner_2D, K, &k1, &k2, pT, iImage_Count, iCorner_Per_Image);
	//printf("k1:%f k2:%f\n", k1, k2);
	//Disp(K, 3, 3, "K");

	if (pH)Free(pH);
	if (pT)Free(pT);
	if (pCorner_3D)Free(pCorner_3D);
	return;
}
void Zhang_Test_1()
{//先做点数据吧
	//approxPolyDP_Test();
	//return;
	typedef double _T;
	//先做棋盘的空间数据
	const int w_In_Point = 11,	//横多少点
		h_In_Point = 8;			//纵多少点
	const _T Grid_Size = 0.02;
	//_T Corner_Point[w_In_Point * h_In_Point][2];
	_T(*pConner_Point_2D)[2];	//(*pCorner_Point_3D)[2] = (_T(*)[2])pMalloc(w_In_Point * h_In_Point * 2 * sizeof(_T));
	int iImage_Count = 0;
	Load_Poine_2D("Sample\\Corner.bin", &iImage_Count, w_In_Point * h_In_Point,w_In_Point,h_In_Point, &pConner_Point_2D);
	
	//iImage_Count = 2;
	unsigned long long tStart = iGet_Tick_Count();
	//for(int i=0;i<20;i++)
	
	//******************程序的主题**************************
	Zhang(pConner_Point_2D, iImage_Count,w_In_Point, h_In_Point, Grid_Size);
	//******************程序的主题**************************

	printf("Time span:%lld\n", iGet_Tick_Count() - tStart);

	//bLoad_Raw_Data("Sample\\Chess_Board_Point_2d.bin", (unsigned char**)&pBuffer, &iPoint_Count);
	//iPoint_Count /= (2 * sizeof(float));
	//pConner_Point_2D = (_T(*)[2])pMalloc(iPoint_Count * 2 * sizeof(_T));
	//for (i = 0; i < iPoint_Count; i++)
	//{//傻逼样本，此处要做个Transpose
	//	int x = i / h_In_Point,
	//		y = i % h_In_Point;
	//	pConner_Point_2D[y*w_In_Point +x][0] = ((float(*)[2])pBuffer)[i][0],
	//		pConner_Point_2D[y*w_In_Point +x][1] = ((float(*)[2])pBuffer)[i][1];
	//}

	////好了，有了一组空间点位置，一组对应的二维点位置，可以估计H矩阵了
	//_T H[3 * 3];
	//
	//Estimate_H_2(pCorner_Point_3D, pConner_Point_2D, iPoint_Count,H);
	////Disp(H, 3, 3, "H");

	////由于测试不充分，最后才换上
	////bEstimate_H_Inv_Power(pCorner_Point_3D, pConner_Point_2D, iPoint_Count,H);
	////Disp(H, 3, 3, "H");

	//Cal_K(H);

	//free(pBuffer);
	Free(pConner_Point_2D);
	return;
}

template<typename _T>static void approxPolyDP(_T Source_Point[][2],int iSource_Count,
	_T Dest_Point[][2],int *piDest_Count, float eps)
{//这段程序找不到任何破绽，优无可优，暂时拿来照用
	Start_End oSlice, oRight_Slice;
	int bLE_eps;
	if (Source_Point == Dest_Point)
	{
		printf("Source cannot be dest in approxPolyDP\n");
		exit(0);
	}
	Find_Max_Dist_Pair(Source_Point,iSource_Count,1.f,&oRight_Slice,&oSlice,&bLE_eps);

	const int iStack_Size = 128;
	Start_End Stack_Buffer[iStack_Size], * Stack;
	if (iSource_Count > iStack_Size)
		Stack = (Start_End*)pMalloc(iSource_Count*sizeof(Start_End));
	else
		Stack = Stack_Buffer;

	int iTop = 0;
	eps *= eps;
	oSlice = { oRight_Slice.m_iEnd,oRight_Slice.m_iStart };

	//讲两个Slice入栈
	Stack[iTop++] = oRight_Slice;
	Stack[iTop++] = oSlice;

	int pos, iDest_Count = 0;
	_T end_pt[2], start_pt[2], pt[2];
	while (iTop > 0)
	{
		//出栈一个
		oSlice = Stack[--iTop];
		end_pt[0] = Source_Point[oSlice.m_iEnd][0];
		end_pt[1] = Source_Point[oSlice.m_iEnd][1];

		pos = oSlice.m_iStart;
		start_pt[0] = Source_Point[pos][0];
		start_pt[1] = Source_Point[pos][1];
		pos = (pos + 1) % iSource_Count;
		if (pos != oSlice.m_iEnd)
		{
			double dx, dy, dist, max_dist = 0;
			dx = end_pt[0] - start_pt[0];
			dy = end_pt[1] - start_pt[1];
			while (pos != oSlice.m_iEnd)
			{
				pt[0] = Source_Point[pos][0];
				pt[1] = Source_Point[pos][1];
				pos = (pos + 1) % iSource_Count;
				dist = fabs((pt[1] - start_pt[1]) * dx - (pt[0] - start_pt[0]) * dy);
				if( dist > max_dist )
				{
					max_dist = dist;
					oRight_Slice.m_iStart = (pos+iSource_Count-1)%iSource_Count;
				}
			}
			bLE_eps = max_dist * max_dist <= eps * (dx * dx + dy * dy);
		}else
		{
			bLE_eps = true;
			// read starting point
			start_pt[0] = Source_Point[oSlice.m_iStart][0];
			start_pt[1] = Source_Point[oSlice.m_iStart][1];
		}
		if( bLE_eps )
		{
			//WRITE_PT(start_pt);
			Dest_Point[iDest_Count][0] = start_pt[0];
			Dest_Point[iDest_Count][1] = start_pt[1];
			iDest_Count++;
		}else
		{
			oRight_Slice.m_iEnd = oSlice.m_iEnd;
			oSlice.m_iEnd = oRight_Slice.m_iStart;
			Stack[iTop++] = oRight_Slice;
			Stack[iTop++] = oSlice;
		}
	}
	pos = 0;
	iSource_Count = iDest_Count;
	pos = iSource_Count - 1;
	start_pt[0] = Dest_Point[pos][0];
	start_pt[1] = Dest_Point[pos][1];
	pos = (pos + 1) % iSource_Count;
	int wpos = pos;

	pt[0] = Dest_Point[pos][0];
	pt[1] = Dest_Point[pos][1];
	pos = (pos + 1) % iSource_Count;

	for (int i = 0; i < iSource_Count  && iDest_Count > 2; i++)
	{
		/*if (i == 2)
		printf("here");*/
		double dx, dy, dist, successive_inner_product;
		end_pt[0] = Dest_Point[pos][0];
		end_pt[1] = Dest_Point[pos][1];
		pos = (pos + 1) % iSource_Count;
		dx = end_pt[0] - start_pt[0];
		dy = end_pt[1] - start_pt[1];
		dist = fabs((pt[0] - start_pt[0])*dy - (pt[1] - start_pt[1])*dx);
		successive_inner_product = (pt[0] - start_pt[0]) * (end_pt[0] - pt[0]) +
			(pt[1] - start_pt[1]) * (end_pt[1] - pt[1]);
		if (dist * dist <= 0.5 * eps * (dx * dx + dy * dy) && dx != 0 && dy != 0 &&
			successive_inner_product >= 0)
		{
			iDest_Count--;
			Dest_Point[wpos][0] = start_pt[0] = end_pt[0];
			Dest_Point[wpos][1] = start_pt[1] = end_pt[1];
			if(++wpos >= iSource_Count) 
				wpos = 0;
			pt[0] = Dest_Point[pos][0];
			pt[1] = Dest_Point[pos][1];
			pos = (pos + 1) % iSource_Count;
			i++;
			continue;
		}
		Dest_Point[wpos][0] = start_pt[0] = pt[0];
		Dest_Point[wpos][1] = start_pt[1] = pt[1];
		if(++wpos >= iSource_Count)
			wpos = 0;
		pt[0] = end_pt[0];
		pt[1] = end_pt[1];
	}

	//return iDest_Count;
	//Disp((_T*)Dest_Point, iDest_Count, 2, "Dest Point");
	*piDest_Count = iDest_Count;
	if (Stack != Stack_Buffer)
		Free(Stack);
	return;
}

template<typename _T>int bIs_Contour_Convex(_T Point[][2], int n)
{//判断一个域是否为凸，不求甚解，先抄为敬。后面要测测如果是瓶颈再想办法
	_T prev_pt[2],cur_pt[2] = { Point[n - 1][0],Point[n - 1][1] };
	//显然，prev_pt是倒数第2点
	memcpy(prev_pt, &Point[(n - 2 + n) % n], 2 * sizeof(_T));
	float dx0 = (float)(cur_pt[0] - prev_pt[0]);
	float dy0 =  (float)(cur_pt[1] - prev_pt[1]);
	int orientation = 0;
	for( int i = 0; i < n; i++ )
	{
		float dxdy0, dydx0;
		float dx, dy;

		prev_pt[0] = cur_pt[0];
		prev_pt[1] = cur_pt[1];

		cur_pt[0] = Point[i][0];
		cur_pt[1] = Point[i][1];

		dx =  (float)(cur_pt[0] - prev_pt[0]);
		dy =  (float)(cur_pt[1] - prev_pt[1]);
		dxdy0 = dx * dy0;
		dydx0 = dy * dx0;

		// find orientation
		// orient = -dy0 * dx + dx0 * dy;
		// orientation |= (orient > 0) ? 1 : 2;
		orientation |= (dydx0 > dxdy0) ? 1 : ((dydx0 < dxdy0) ? 2 : 3);
		if( orientation == 3 )
			return 0;

		dx0 = dx;
		dy0 = dy;
	}

	return 1;
}

static void approxPolyDP_Test()
{//减点逼近试验
	typedef unsigned short _T;
	_T Dest_Point[5][2], Source_Point[][2] = { 100,100,
		120,120,
		140,110,
		180,120,
		200,100};
	int iSource_Count = 5, iDest_Count;
	approxPolyDP(Source_Point, iSource_Count, Dest_Point, &iDest_Count, 10);
	//干脆画出来看看
	Draw_Contour_Outline((char*)"c:\\tmp\\1.bmp", Dest_Point, iDest_Count);
	printf("%d\n",bIs_Contour_Convex(Dest_Point, iDest_Count));
	return;
}
void Generate_Quad(Image oImage, Chess_Board_Quad_1** ppQuad, int* piQuad_Count, int *piMax_Quad_Count,int iDilation)
{
	const int Min_Level = 1, Max_Level = 7, Min_Area = 25;
	Get_Contour_Result oResult;
	Get_Contour(oImage, &oResult);
	//printf("%d Contour\n", oResult.m_iContour_Count);

	int i, iBoard_Index,iNew_Point_Count, iMax_Point_Count = 0, iQuad_Count = 0;

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

	unsigned int* pContour_Quad = (unsigned int*)pMalloc(iQuad_Count * sizeof(int));
	unsigned short (*pNew_Point)[2] = (unsigned short(*)[2])pMalloc(iMax_Point_Count * 2 * sizeof(unsigned short));
	//hierachy[0]：同级前一个轮廓 hierachy[1]：同级下一个轮廓
	//hierachy[2]:第一个子轮廓	hierachy[3]：父轮廓
	for (iQuad_Count=0,i =oResult.m_iContour_Count-1; i >=0; i--)
	{
		oContour = oResult.m_pContour[i];
		//如果是根节点，或者有孔，都不能作为块
		if (oContour.hierachy[3] == -1 ||oContour.hierachy[2] != -1)
			continue;

		unsigned short Contour_Rect[2][2];
		Get_Bounding_Box(oContour.m_pPoint, oContour.m_iOutline_Point_Count, Contour_Rect);
		if( (Contour_Rect[1][0]-Contour_Rect[0][0]+1)*(Contour_Rect[1][1]-Contour_Rect[0][1]+1)<Min_Area)
			continue;

		iNew_Point_Count = oContour.m_iOutline_Point_Count;
		//if(i==91)
			//Disp((unsigned short*)oContour.m_pPoint, oContour.m_iOutline_Point_Count, 2, "Point");

		int iLevel = Min_Level;
		for (; iLevel <= Max_Level && iNew_Point_Count>4; iLevel++)
		{
			approxPolyDP(oContour.m_pPoint,oContour.m_iOutline_Point_Count,	pNew_Point, &iNew_Point_Count,(float)iLevel);
			memcpy(oContour.m_pPoint, pNew_Point, iNew_Point_Count * 2 * sizeof(unsigned short));
			oContour.m_iOutline_Point_Count = iNew_Point_Count;
		}

		if (iNew_Point_Count != 4)
			continue;

		if (iLevel == Min_Level)
			memcpy(pNew_Point, oContour.m_pPoint, 4 * 2* sizeof(unsigned short));

		//if(i==7054)
			//Disp((unsigned short*)pNew_Point, iNew_Point_Count, 2, "New Point");
		//判断是否为凸，这里要搞搞
		if (!bIs_Contour_Convex(pNew_Point,iNew_Point_Count))
			continue;

		//memcpy(oResult.m_pContour[i].m_pPoint, pNew_Point, 4*2*sizeof(unsigned short));
		//oResult.m_pContour[i].m_iOutline_Point_Count = 4;

		/*printf("Quad:%d Contour idx:%d\n", iQuad_Count,i);
		for (int j = 0; j < 4; j++)
			printf("%d %d\n",pNew_Point[j][0],pNew_Point[j][1]);*/

		pContour_Quad[iQuad_Count++] = i;
		//printf("%d\n", i);
	}

	Free(pNew_Point);
	Shrink(pContour_Quad, iQuad_Count * sizeof(int));

	int iMax_Quad_Count= Max(2, iQuad_Count * 3);
	Chess_Board_Quad_1* pAll_Quad = (Chess_Board_Quad_1*)pMalloc(iMax_Quad_Count * sizeof(Chess_Board_Quad_1));
	int iQuad_Count_1=0;	//这个才是最后的Quad_Cunbt?
	for (i=0;i<iQuad_Count;i++)
	{
		oContour = oResult.m_pContour[pContour_Quad[i]];
		//此处可以忽略，因为源代码是有ifilterQuads的情况下猜判断
		//if (oContour.hierachy[3] != iBoard_Index)
			//continue;

		Chess_Board_Quad_1 *poQuad = &pAll_Quad[iQuad_Count_1++];
		memcpy(poQuad->Corner_i, oContour.m_pPoint, 4 * 2 * sizeof(unsigned short));
		memset(poQuad->Neighbour, -1, 4 * sizeof(Quad_Neighbour_Item_1));
		poQuad->m_iNeighbour_Count = 0;
		poQuad->col = poQuad->row = 0;
		poQuad->ordered = 0;
		poQuad->m_iGroup_Index = -1;
		poQuad->edge_sqr_len = (float)0xFFFFFFFF;

		for (int j = 0; j < 4; j++)
		{
			unsigned int sqr = (oContour.m_pPoint[j][0] - oContour.m_pPoint[(j + 1) & 3][0]) *
				(oContour.m_pPoint[j][0] - oContour.m_pPoint[(j + 1) & 3][0]) +
				(oContour.m_pPoint[j][1] - oContour.m_pPoint[(j + 1) & 3][1]) *
				(oContour.m_pPoint[j][1] - oContour.m_pPoint[(j + 1) & 3][1]);
			//取四条边最小者的平方为edge_sqr_len
			if (sqr < poQuad->edge_sqr_len)
				poQuad->edge_sqr_len = (float)sqr;
		}
		const int edge_len_compensation = 2 * iDilation;
		poQuad->edge_sqr_len += 2 * sqrt(poQuad->edge_sqr_len) * edge_len_compensation + edge_len_compensation * edge_len_compensation;
		//Disp_Quad(*poQuad);
	}
	iMax_Quad_Count = iQuad_Count_1 * 2;
	Shrink(pAll_Quad, iMax_Quad_Count * sizeof(Chess_Board_Quad));
	Free(pContour_Quad);
	Free_Contour_Result(&oResult);

	*ppQuad = pAll_Quad;
	*piQuad_Count = iQuad_Count_1;
	*piMax_Quad_Count = iMax_Quad_Count;
	/*for (int i = 0; i < iQuad_Count_1; i++)
	{
		printf("Quad:%d\n", i);
		for (int j = 0; j < 4; j++)
			printf("%d %d\n", pAll_Quad[i].Corner_i[j][0],pAll_Quad[i].Corner_i[j][1]);
	}*/

	return;
}
void Generate_Quad(Image oImage, Chess_Board_Quad **ppQuad, int *piQuad_Count)
{//根据一张二值图寻找一堆四边形，至于这些四边形是不是管用，后面再判断
	const int Min_Level = 1, Max_Level = 7, Min_Area = 25;
	Get_Contour_Result oResult;
	Get_Contour(oImage, &oResult);
	int i, iBoard_Index,iNew_Point_Count, iMax_Point_Count = 0, iQuad_Count = 0;

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
	unsigned int* pContour_Quad = (unsigned int*)pMalloc(iQuad_Count * sizeof(int));
	unsigned short (*pNew_Point)[2] = (unsigned short(*)[2])pMalloc(iMax_Point_Count * 2 * sizeof(unsigned short));
	//hierachy[0]：同级前一个轮廓 hierachy[1]：同级下一个轮廓
	//hierachy[2]:第一个子轮廓	hierachy[3]：父轮廓
	for (iQuad_Count=0,i = oResult.m_iContour_Count-1; i >=0; i--)
	{
		oContour = oResult.m_pContour[i];
		//如果是根节点，或者有孔，都不能作为块
		if (oContour.hierachy[3] == -1 && oContour.hierachy[2] != -1)
			continue;

		unsigned short Contour_Rect[2][2];
		Get_Bounding_Box(pNew_Point, 4, Contour_Rect);
		if( (Contour_Rect[1][0]-Contour_Rect[0][0]+1)*(Contour_Rect[1][1]-Contour_Rect[0][1]+1)<Min_Area)
			continue;

		for (int iLevel = Min_Level; iLevel < Max_Level; iLevel++)
			approxPolyDP(oContour.m_pPoint,oContour.m_iOutline_Point_Count,
				pNew_Point, &iNew_Point_Count,(float)iLevel);

		if (oContour.m_iOutline_Point_Count != 4)
			continue;

		//判断是否为凸，这里要搞搞
		if (!bIs_Contour_Convex(pNew_Point,iNew_Point_Count))
			continue;

		memcpy(oResult.m_pContour[i].m_pPoint, pNew_Point, 4);
		oResult.m_pContour[i].m_iOutline_Point_Count = 4;

		pContour_Quad[iQuad_Count++] = i;
	}
	Free(pNew_Point);
	Shrink(pContour_Quad, iQuad_Count * sizeof(int));
	
	int iMax_Quad_Count= Max(2, iQuad_Count * 3);
	Chess_Board_Quad* pAll_Quad = (Chess_Board_Quad*)pMalloc(iMax_Quad_Count * sizeof(Chess_Board_Quad));
	//memset(pAll_Quad, 0, iMax_Quad_Count * sizeof(Chess_Board_Quad));

	int iQuad_Count_1=0;	//这个才是最后的Quad_Cunbt?
	for (i=0;i<iQuad_Count;i++)
	{
		oContour = oResult.m_pContour[pContour_Quad[i]];
		//此处有异议，难道不是根节点的儿子就不能算棋盘格？
		if (oContour.hierachy[3] != iBoard_Index)
			continue;
		
		/*if (i == 4)
			printf("here");*/
		Chess_Board_Quad *poQuad = &pAll_Quad[iQuad_Count_1++];
		memcpy(poQuad->Corner_i, oContour.m_pPoint, 4 * 2 * sizeof(unsigned short));
		memset(poQuad->Neighbour, -1, 4 * sizeof(Quad_Neighbour_Item));
		poQuad->m_iCount = 0;
		poQuad->col = poQuad->row = 0;
		poQuad->ordered = 0;
		poQuad->m_iGroup_Index = -1;
		
		for (int j = 0; j < 4; j++)
		{
			unsigned int sqr = (oContour.m_pPoint[j][0] - oContour.m_pPoint[(j + 1) & 3][0]) *
				(oContour.m_pPoint[j][0] - oContour.m_pPoint[(j + 1) & 3][0]) +
				(oContour.m_pPoint[j][1] - oContour.m_pPoint[(j + 1) & 3][1]) *
				(oContour.m_pPoint[j][1] - oContour.m_pPoint[(j + 1) & 3][1]);
			if (sqr > poQuad->edge_sqr_len)
				poQuad->edge_sqr_len = (float)sqr;
		}
	}
	Shrink(pAll_Quad, iQuad_Count_1 * sizeof(Chess_Board_Quad));
	Free(pContour_Quad);
	Free_Contour_Result(&oResult);
	
	*ppQuad = pAll_Quad;
	*piQuad_Count = iQuad_Count_1;
	return;
}
static float iGet_Sqr(float iDiff_x, float iDiff_y)
{
	return iDiff_x * iDiff_x + iDiff_y * iDiff_y;
}
template<typename _T>int bPoints_On_Same_Side_From_Line(float Line_Point_1[2], float Line_Point_2[2], _T Point_1, _T Point_2)
{
	double line_direction_vector[] = {Line_Point_2[0] - Line_Point_1[0],Line_Point_2[1] - Line_Point_1[1]};
	double vector1[] = { Point_1[0] - Line_Point_1[0], Point_1[1] - Line_Point_1[1] };
	double vector2[] = { Point_2[0] - Line_Point_1[0],Point_2[1] - Line_Point_1[1] };
	double fValue = line_direction_vector[0] * vector1[1] - line_direction_vector[1] * vector1[0];
	fValue *= line_direction_vector[0] * vector2[1] - line_direction_vector[1] * vector2[0];
	return fValue>0;
}
static int iQuick_Sort_Partition(Quad_Neighbour_Item_1 pBuffer[], int left, int right)
{//小到大的顺序
 //Real fRef;
	Quad_Neighbour_Item_1 iValue, oTemp;
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
	while (pCur_Left >= pStart && (pCur_Left->m_iCorner_Index == oRef.m_iCorner_Index &&pCur_Left->m_iQuad_Index == oRef.m_iQuad_Index) )
		pCur_Left--;
	pCur_Right = pCur_Left;
	pCur_Left--;

	while (pCur_Left >= pStart)
	{
		if (pCur_Left->m_iCorner_Index == oRef.m_iCorner_Index &&pCur_Left->m_iQuad_Index == oRef.m_iQuad_Index)
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
int iAdjust_Left(Quad_Neighbour_Item_1* pStart, Quad_Neighbour_Item_1* pEnd)
{
	Quad_Neighbour_Item_1 oTemp, * pCur_Left = pEnd - 1,
		* pCur_Right;
	Quad_Neighbour_Item_1 oRef = *pEnd;

	//为了减少一次判断，此处先扫过去
	while (pCur_Left >= pStart && (pCur_Left->m_iCorner_Index == oRef.m_iCorner_Index &&pCur_Left->m_iQuad_Index == oRef.m_iQuad_Index) )
		pCur_Left--;
	pCur_Right = pCur_Left;
	pCur_Left--;

	while (pCur_Left >= pStart)
	{
		if (pCur_Left->m_iCorner_Index == oRef.m_iCorner_Index &&pCur_Left->m_iQuad_Index == oRef.m_iQuad_Index)
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
static void Quick_Sort(Quad_Neighbour_Item_1 Seq[], int iStart, int iEnd)
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

int bFind_Neighbour(Chess_Board_Quad Quad[], int iQuad_Count, float To_Find[2], int iCur_Quad_Index, int iCur_Corner_Index,float sqr_radius,
	int *piClosest_Quad_Index, int *piClosest_Corner_Index,float *piDist)
{//寻找半径范围内最近邻
	float fMin_Dist = (float)0xFFFFFFF,
		iDist = 0,bFound=0;	
	int  iClosest_Quad = -1, iClosest_Corner = -1, iClosest_Neighbour_Index = -1;
	Quad_Neighbour_Item Neighbour[128];
	int i,j,iNeighbour_Count = 0;

	for (i = 0; i < iQuad_Count; i++)
	{//慢速查找
		if (i == iCur_Quad_Index)
			continue;
		for (j = 0; j < 4; j++)
		{
			float *pCorner =  Quad[i].Corner_f[j];
			iDist=iGet_Sqr(pCorner[0]-To_Find[0], pCorner[1]-To_Find[1]);
			if (iDist < sqr_radius)
				Neighbour[iNeighbour_Count++] = { (short)i,(short)j,iDist };
		}
	}

	//此处对Neighbour排个序
	Quick_Sort(Neighbour, 0, iNeighbour_Count - 1);

	Chess_Board_Quad oCur_Quad = Quad[iCur_Quad_Index];
	const float thresh_sqr_scale = 2.0;
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
			
			float mid_pt1[2] = { (oCur_Quad.Corner_f[iCur_Corner_Index][0] + oCur_Quad.Corner_f[(iCur_Corner_Index + 1) & 3][0]) / 2.f,
				(oCur_Quad.Corner_f[iCur_Corner_Index][1] + oCur_Quad.Corner_f[(iCur_Corner_Index + 1) & 3][1]) / 2.f };
			float mid_pt2[2] = { (oCur_Quad.Corner_f[(iCur_Corner_Index + 2) & 3][0] + oCur_Quad.Corner_f[(iCur_Corner_Index + 3) & 3][0]) / 2.f,
				(oCur_Quad.Corner_f[(iCur_Corner_Index + 2) & 3][1] + oCur_Quad.Corner_f[(iCur_Corner_Index + 3) & 3][1]) / 2.f };
			
			if (!bPoints_On_Same_Side_From_Line(mid_pt1, mid_pt2, To_Find, q_k.Corner_f[oNeighbour.m_iCorner_Index]))
				continue;
			
			float mid_pt3[2] = { (oCur_Quad.Corner_f[(iCur_Corner_Index + 1) & 3][0] + oCur_Quad.Corner_f[(iCur_Corner_Index + 2) & 3][0]) / 2.f,
				(oCur_Quad.Corner_f[(iCur_Corner_Index + 1) & 3][1] + oCur_Quad.Corner_f[(iCur_Corner_Index + 2) & 3][1]) / 2.f };
			float mid_pt4[2] = { (oCur_Quad.Corner_f[(iCur_Corner_Index + 3) & 3][0] + oCur_Quad.Corner_f[iCur_Corner_Index][0]) / 2.f,
				(oCur_Quad.Corner_f[(iCur_Corner_Index + 3) & 3][1] + oCur_Quad.Corner_f[iCur_Corner_Index][1]) / 2.f };
			/*if (i == 2)
				printf("here");*/
			if (!bPoints_On_Same_Side_From_Line(mid_pt3, mid_pt4, To_Find,  q_k.Corner_f[oNeighbour.m_iCorner_Index]))
				continue;
			float neighbor_pt_diagonal[] = { q_k.Corner_f[(oNeighbour.m_iCorner_Index + 2) & 3][0],q_k.Corner_f[(oNeighbour.m_iCorner_Index + 2) & 3][1] };
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
		for (j=0; j < 4; j++)
		{
			if (oCur_Quad.Neighbour[j].m_iQuad_Index == iClosest_Quad)
				break;
			float* pClosest_Point = Quad[iClosest_Quad].Corner_f[iClosest_Corner];

			if(iGet_Sqr(pClosest_Point[0]-oCur_Quad.Corner_f[j][0],pClosest_Point[1]-oCur_Quad.Corner_f[j][1]) < fMin_Dist)
				break;
		}
		if (j < 4)
			return 0;
		Chess_Board_Quad oClosest_Quad = Quad[iClosest_Quad];
		for(j = 0; j < 4; j++ )
		{
			if (oClosest_Quad.Neighbour[j].m_iQuad_Index == iCur_Quad_Index)
				break;
		}
		if (j < 4)
			return 0;
		return 1;
	}
	return 0;	
}
void Find_Quad_Neighbour(Chess_Board_Quad Quad[], int iQuad_Count)
{
	const int thresh_sqr_scaleb = 2;
	for (int i = 0; i < iQuad_Count; i++)
	{
		Chess_Board_Quad *poCur_Quad=&Quad[i], oCur = *poCur_Quad;
		for (int j = 0; j < 4; j++)
		{
			if (oCur.Neighbour[j].m_iQuad_Index!=-1)
				continue;	//已有主

			//先初始化为无效值
			float min_sqr_dist = (float)0xFFFFFFF;
			int closest_quad_idx = -1;
			int closest_corner_idx = -1;
			float sqr_radius = oCur.edge_sqr_len * thresh_sqr_scaleb + 1;
			/*if (i == 2 && j == 1)
				printf("Here");*/
			//先用个最慢玩意
			int bFound=bFind_Neighbour(Quad, 
				iQuad_Count, 
				oCur.Corner_f[j],
				i,j,
				sqr_radius,
				&closest_quad_idx,
				&closest_corner_idx,
				&min_sqr_dist);

			if (!bFound)
				continue;

			sqr_radius = min_sqr_dist + 1;
			min_sqr_dist = FLT_MAX;

			int closest_closest_quad_idx = -1;
			int closest_closest_corner_idx = -1;

			Chess_Board_Quad* poClosest_Quad = &Quad[closest_quad_idx];
			//unsigned short* pCloest_Corner =poClosest_Quad->Corner[closest_corner_idx];
			float *pCloest_Corner = poClosest_Quad->Corner_f[closest_corner_idx];

			bFound = bFind_Neighbour(Quad, 
				iQuad_Count, 
				pCloest_Corner,
				closest_quad_idx,closest_corner_idx,
				sqr_radius, 
				&closest_closest_quad_idx, 
				&closest_closest_corner_idx, 
				&min_sqr_dist);
			if (!bFound)
				continue;

			if (closest_closest_quad_idx != i || closest_closest_corner_idx != j )
				continue;

			pCloest_Corner[0] = (oCur.Corner_f[j][0] + pCloest_Corner[0]) * 0.5f;
			pCloest_Corner[1] = (oCur.Corner_f[j][1] + pCloest_Corner[1]) * 0.5f;

			poCur_Quad->m_iCount++;
			poCur_Quad->Neighbour[j] = { (short)closest_quad_idx,(short)closest_corner_idx,min_sqr_dist};
			poCur_Quad->Corner_f[j][0] = pCloest_Corner[0];
			poCur_Quad->Corner_f[j][1] = pCloest_Corner[1];

			poClosest_Quad->m_iCount++;
			//poClosest_Quad->Neighbour[closest_corner_idx].m_iQuad_Index = i;
			poClosest_Quad->Neighbour[closest_corner_idx] = { (short)i,(short)j,min_sqr_dist};

		}
	}
	return;
}

void Check_Quad_Group(Chess_Board_Quad Quad[], int iQuad_Count, Chess_Board_Quad* Quad_Group[], int iQuad_Group_Size,Chess_Board_Corner* Corner_Group_Out[],unsigned char Pattern_Size[2])
{//暂时不知其义
	const int ROW1 = 1000000;	
	const int ROW2 = 2000000;
	const int ROW_ = 3000000;

	int iCorner_Count = 0, iCorner_Out_Count = 0;
	int result = 0;

	int width = 0, height = 0;
	int hist[5] = {0,0,0,0,0};	//此处的直方图有没有用?没用就不要了
	Chess_Board_Corner* Corner_Group[128];

	for (int i = 0; i < iQuad_Count; ++i)
	{
		Chess_Board_Quad* q = Quad_Group[i];
		for (int j = 0; j < 4; ++j)
		{
			//if (i == 1 && j==2)
				//printf("here");
			if (q->Neighbour[j].m_iCorner_Index!=-1)
			{
				int next_j = (j + 1) & 3;
				Chess_Board_Corner *a = &q->Corner[j], *b = &q->Corner[next_j];
				int row_flag = q->m_iCount == 1 ? ROW1 : q->m_iCount == 2 ? ROW2 : ROW_;
				if (a->row == 0)
				{
					Corner_Group[iCorner_Count++] = a;
					a->row = row_flag;
				}else if (a->row > (unsigned int)row_flag)
				{
					a->row = row_flag;
				}
				if (q->Neighbour[next_j].m_iQuad_Index!=-1)
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
	if (iCorner_Count != Pattern_Size[0] * Pattern_Size[1])
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
			hist[3] != (Pattern_Size[0] + Pattern_Size[1]) * 2 - 8)
			goto END;

		Chess_Board_Corner* cur = first;
		Chess_Board_Corner* right = NULL;
		Chess_Board_Corner* below = NULL;
		Corner_Group_Out[iCorner_Out_Count++] = (cur);
	}


END:
	
	return;
}
void Find_Connected_Quads(Chess_Board_Quad Quad[], int iQuad_Coubt,Chess_Board_Quad *Quad_Group[],int iGroup_Index,int *piGroup_Size)
{//其实就是找Quad级的连通域，没啥新鲜的
	Chess_Board_Quad* poQuad;
	Chess_Board_Quad** Stack = (Chess_Board_Quad**)pMalloc(iQuad_Coubt * sizeof(Chess_Board_Quad*));
	int iTop = 0,
		iGroup_Size = 0;

	for (int i = 0; i < iQuad_Coubt; i++)
	{
		poQuad = &Quad[i];
		if (poQuad->m_iCount <= 0 ||  poQuad->m_iGroup_Index>= 0) 
			continue;
		//压入栈
		Stack[iTop++] = Quad_Group[iGroup_Size++] = poQuad;
		poQuad->m_iGroup_Index = iGroup_Index;

		while (iTop > 0)
		{
			poQuad = Stack[--iTop];
			for (int j = 0; j < 4; j++)
			{
				Quad_Neighbour_Item oNeighbour = poQuad->Neighbour[j];
				if (oNeighbour.m_iQuad_Index != -1 && Quad[oNeighbour.m_iQuad_Index].m_iGroup_Index ==-1)
				{//有邻居
					Stack[iTop++] =Quad_Group[iGroup_Size++]= &Quad[oNeighbour.m_iQuad_Index];
					//poQuad->Neighbour[i].m_iGroup_Index = iGroup_Index; //group_idx;
					Quad[oNeighbour.m_iQuad_Index].m_iGroup_Index = iGroup_Index;
				}
			}
		}
	}
	*piGroup_Size = iGroup_Size;
	Free(Stack);
	return;
}
void Order_Quad(Chess_Board_Quad Quad[], Chess_Board_Quad* poQuad, float Corner[2], int common)
{//暂时不知其义
	int tc = 0;
	for (tc; tc < 4; ++tc)
		if (poQuad->Corner_f[tc][0] == Corner[0] &&poQuad->Corner_f[tc][1] == Corner[1])
			break;

	//感觉以下做了个旋转
	while (tc != common)
	{
		float*tempc = poQuad->Corner_f[3];
		Quad_Neighbour_Item* tempq = &poQuad->Neighbour[3];
		for (int i = 3; i > 0; --i)
		{
			poQuad->Corner_f[i][0] = poQuad->Corner_f[i - 1][0];
			poQuad->Corner_f[i][1] = poQuad->Corner_f[i - 1][1];
			poQuad->Neighbour[i] = poQuad->Neighbour[i - 1];
		}
		poQuad->Corner_f[0][0] = tempc[0];
		poQuad->Corner_f[0][1] = tempc[1];
		poQuad->Neighbour[0] = *tempq;

		tc = (tc + 1) & 3;
	}
	return;
}
int iOrder_Found_Connected_Quads(Chess_Board_Quad Quad[], int iQuad_Count, Chess_Board_Quad* Quad_Group[],int iGroup_Size,unsigned char Pattern_Size[2])
{//将乱七八糟的四边形组排列好，具体怎么做还不知道
//返回个数
	int i;
	//首先确定开始的块
	Chess_Board_Quad* poStart = NULL;
	Chess_Board_Quad* Stack[128];	//战
	int iTop = 0;

	for (i = 0; i < iGroup_Size; i++)
	{
		if (Quad_Group[i]->m_iCount == 4)
		{
			poStart = Quad_Group[i];
			break;
		}
	}
	//Disp_Quad(*poStart, 0);
	if (!poStart)
		return 0;

	//未知其义
	int row_min = 0, col_min = 0, row_max=0, col_max = 0;
	//int col_hist[16], row_hist[16];	//又搞直方图？

	Stack[iTop++] = poStart;
	poStart->row = 0;
	poStart->col = 0;
	poStart->ordered = true;

	while (iTop>0)
	{
		Chess_Board_Quad* q = Stack[--iTop];
		int col = q->col;
		int row = q->row;

		//col_hist[col]++;
		//row_hist[row]++;

		if (row > row_max) 
			row_max = row;
		if (row < row_min)
			row_min = row;
		if (col > col_max) 
			col_max = col;
		if (col < col_min)
			col_min = col;
		//Disp_Quad(*q,0);
		for (int i = 0; i < 4; i++)
		{
			Chess_Board_Quad* poNeighbour =q->Neighbour[i].m_iQuad_Index==-1?NULL:&Quad[q->Neighbour[i].m_iQuad_Index];
			//Disp_Quad(*poNeighbour,0);
			switch(i)
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
			if (poNeighbour && poNeighbour->ordered == 0 && poNeighbour->m_iCount == 4)
			{
				Order_Quad(Quad, poNeighbour, q->Corner_f[i], (i + 2) & 3);
				poNeighbour->ordered = 1;
				poNeighbour->row = row;
				poNeighbour->col = col;
				Stack[iTop++] = poNeighbour;
			}
		}
	}
	//Disp_Quad(Quad[5],0);

	int w = Pattern_Size[0] - 1;
	int h = Pattern_Size[1] - 1;
	int drow = row_max - row_min + 1;
	int dcol = col_max - col_min + 1;

	if ((w > h && dcol < drow) || (w < h && drow < dcol))
	{
		h = Pattern_Size[0] - 1;
		w = Pattern_Size[1]- 1;
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
		if (q->m_iCount != 4)
			continue;
		int col = q->col;
		int row = q->row;
		for (int j = 0; j < 4; j++)
		{
			switch(j)   // adjust col, row for this quad
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
			Chess_Board_Quad* poNeighbour = q->Neighbour[i].m_iQuad_Index==-1?NULL:&Quad[q->Neighbour[j].m_iQuad_Index];
			if (poNeighbour && !poNeighbour->ordered && // is it an inner quad?
				col <= col_max && col >= col_min && row <= row_max && row >= row_min)
			{
				bFound++;
				Order_Quad(Quad,poNeighbour, q->Corner_f[j], (j+2)&3);
				poNeighbour->ordered = 1;
				poNeighbour->row = row;
				poNeighbour->col = col;
			}
		}		
	}
	int max_quad_buf_size = iQuad_Count * 4;
	if (bFound > 0)
	{
		printf("Found %d inner quads not connected to outer quads, repairing\n", bFound);
		exit(0);
		/*for (int i = 0; i < iGroup_Size && iQuad_Count < max_quad_buf_size; i++)
		{
			ChessBoardQuad& q = *quads[i];
			if (q.count < 4 && q.ordered)
			{
				int added = addOuterQuad(q, quads);
				quad_count += added;
			}
		}*/
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
				for (int j=0; j<4; j++) // any neighbors that are ordered?
				{
					if (q->Neighbour[j].m_iCorner_Index!=-1 && Quad[q->Neighbour[j].m_iQuad_Index].ordered)
						outer = 1;
				}
				if (!outer) // not an outer quad, eliminate
				{
					printf("Not implemented yet\n");
					exit(0);
					//removeQuadFromGroup(quads, q);
				}
			}
		}
		return iGroup_Size;
	}
	return 0;
}
void Convert_Corner(Chess_Board_Quad *Quad_Group[], int iGroup_Size)
{
	for (int i =iGroup_Size -1; i >=0; i--)
	{
		for (int j = 3; j >=0; j--)
		{
			Chess_Board_Corner* pCorner = &Quad_Group[i]->Corner[j];
			float* pCorner_f = Quad_Group[i]->Corner_f[j];
			pCorner->Corner_f[1] = pCorner_f[1];
			pCorner->Corner_f[0] = pCorner_f[0];
			pCorner->row = 0;
			pCorner->m_iCount = 0;
			memset(pCorner->Neighbour, 0, 4 * sizeof(Chess_Board_Corner*));
		}
	}
	return;
}

int bFind_Neighbour(Chess_Board_Quad_1 Quad[], int iQuad_Count, float To_Find[2], int iCur_Quad_Index, int iCur_Corner_Index, float sqr_radius,
	int* piClosest_Quad_Index, int* piClosest_Corner_Index, float* piDist)
{//寻找半径范围内最近邻
	float fMin_Dist = (float)0xFFFFFFF,
		iDist = 0, bFound = 0;
	int  iClosest_Quad = -1, iClosest_Corner = -1, iClosest_Neighbour_Index = -1;
#define MAX_NEIGHBOUR_COUNT 256
	Quad_Neighbour_Item_1 Neighbour[MAX_NEIGHBOUR_COUNT];
	int i, j, iNeighbour_Count = 0;

	for (i = 0; i < iQuad_Count; i++)
	{//慢速查找
		if (i == iCur_Quad_Index)
			continue;
		for (j = 0; j < 4; j++)
		{
			float* pCorner = Quad[i].Corner[j]->Pos;
			iDist = iGet_Sqr(pCorner[0] - To_Find[0], pCorner[1] - To_Find[1]);
			if (iDist < sqr_radius)
				Neighbour[iNeighbour_Count++] = { (short)i,(short)j,iDist };
			if (iNeighbour_Count > MAX_NEIGHBOUR_COUNT)
			{
				printf("Insufficient Neighbour Count in bFind_Neighbour\n");
				exit(0);
			}
		}
	}

	//此处对Neighbour排个序
	Quick_Sort(Neighbour, 0, iNeighbour_Count - 1);

	Chess_Board_Quad_1 oCur_Quad = Quad[iCur_Quad_Index];
	const float thresh_sqr_scale = 2.0;
	for (i = 0; i < iNeighbour_Count; i++)
	{
		Quad_Neighbour_Item_1 oNeighbour = Neighbour[i];
		//int iTemp = (oNeighbour.m_iQuad_Index << 2) + oNeighbour.m_iCorner_Index;
		/*if (iCur_Quad_Index == 2 && iCur_Corner_Index == 1)
		printf("here");*/

		float sqr_dist = oNeighbour.m_fDist;
		Chess_Board_Quad_1 q_k = Quad[oNeighbour.m_iQuad_Index];
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

			if (!bPoints_On_Same_Side_From_Line(mid_pt3, mid_pt4, To_Find,  q_k.Corner[oNeighbour.m_iCorner_Index]->Pos))
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
		for (j=0; j < 4; j++)
		{
			if (oCur_Quad.Neighbour[j].m_iQuad_Index == iClosest_Quad)
				break;
			float* pClosest_Point = Quad[iClosest_Quad].Corner[iClosest_Corner]->Pos;

			if(iGet_Sqr(pClosest_Point[0]-oCur_Quad.Corner[j]->Pos[0],pClosest_Point[1]-oCur_Quad.Corner[j]->Pos[1]) < fMin_Dist)
				break;
		}
		if (j < 4)
			return 0;
		Chess_Board_Quad_1 oClosest_Quad = Quad[iClosest_Quad];
		for(j = 0; j < 4; j++ )
		{
			if (oClosest_Quad.Neighbour[j].m_iQuad_Index == iCur_Quad_Index)
				break;
		}
		if (j < 4)
			return 0;
		return 1;
	}
	return 0;
}
void Find_Quad_Neighbour(Chess_Board_Quad_1 Quad[], int iQuad_Count)
{
	const int thresh_sqr_scaleb = 2;
	for (int i = 0; i < iQuad_Count; i++)
	{
		Chess_Board_Quad_1 *poCur_Quad=&Quad[i], oCur = *poCur_Quad;
		for (int j = 0; j < 4; j++)
		{
			if (oCur.Neighbour[j].m_iQuad_Index != -1)
				continue;	//已有主

			//先初始化为无效值
			float min_sqr_dist = (float)0xFFFFFFF;
			int closest_quad_idx = -1;
			int closest_corner_idx = -1;
			float sqr_radius = oCur.edge_sqr_len * thresh_sqr_scaleb + 1;
			
			int bFound=bFind_Neighbour(Quad, 
				iQuad_Count, 
				oCur.Corner[j]->Pos,
				i,j,
				sqr_radius,
				&closest_quad_idx,
				&closest_corner_idx,
				&min_sqr_dist);

			if (!bFound)
				continue;

			sqr_radius = min_sqr_dist + 1;
			min_sqr_dist = FLT_MAX;

			int closest_closest_quad_idx = -1;
			int closest_closest_corner_idx = -1;

			Chess_Board_Quad_1* poClosest_Quad = &Quad[closest_quad_idx];
			float *pCloest_Corner = poClosest_Quad->Corner[closest_corner_idx]->Pos;

			bFound = bFind_Neighbour(Quad, 
				iQuad_Count, 
				pCloest_Corner,
				closest_quad_idx,closest_corner_idx,
				sqr_radius, 
				&closest_closest_quad_idx, 
				&closest_closest_corner_idx, 
				&min_sqr_dist);
			if (!bFound)
				continue;

			if (closest_closest_quad_idx != i || closest_closest_corner_idx != j )
				continue;

			pCloest_Corner[0] = (oCur.Corner[j]->Pos[0] + pCloest_Corner[0]) * 0.5f;
			pCloest_Corner[1] = (oCur.Corner[j]->Pos[1] + pCloest_Corner[1]) * 0.5f;

			poCur_Quad->m_iNeighbour_Count++;
			poCur_Quad->Neighbour[j] = { (short)closest_quad_idx,(short)closest_corner_idx,min_sqr_dist,&Quad[closest_quad_idx]};
			//poCur_Quad->Corner[j]->Pos[0] = pCloest_Corner[0];
			//poCur_Quad->Corner[j]->Pos[1] = pCloest_Corner[1];
			poCur_Quad->Corner[j] = poClosest_Quad->Corner[closest_corner_idx];

			poClosest_Quad->m_iNeighbour_Count++;
			poClosest_Quad->Neighbour[closest_corner_idx] = { (short)i,(short)j,min_sqr_dist,&Quad[i]};
		}
	}
	return;
}

void Find_Connected_Quads(Chess_Board_Quad_1 Quad[], int iQuad_Coubt, Chess_Board_Quad_1* Quad_Group[], int iGroup_Index, int* piGroup_Size)
{//其实就是找Quad级的连通域，没啥新鲜的
	Chess_Board_Quad_1* poQuad;
	Chess_Board_Quad_1** Stack = (Chess_Board_Quad_1**)pMalloc(iQuad_Coubt * sizeof(Chess_Board_Quad_1*));
	int iTop = 0,
		iGroup_Size = 0;
	for (int i = 0; i < iQuad_Coubt; i++)
	{
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
				Quad_Neighbour_Item_1 oNeighbour = poQuad->Neighbour[j];
				if (oNeighbour.m_iQuad_Index != -1 && Quad[oNeighbour.m_iQuad_Index].m_iGroup_Index == -1)
				{//有邻居
					Stack[iTop++] =Quad_Group[iGroup_Size++]= &Quad[oNeighbour.m_iQuad_Index];
					Quad[oNeighbour.m_iQuad_Index].m_iGroup_Index = iGroup_Index;
				}
			}
		}
		break;
	}
	*piGroup_Size = iGroup_Size;
	Free(Stack);
}
void Order_Quad(Chess_Board_Quad_1 Quad[], Chess_Board_Quad_1* poQuad, float Corner[2], int common)
{//暂时不知其义
	int tc = 0;
	for (tc; tc < 4; ++tc)
		if (poQuad->Corner[tc]->Pos[0] == Corner[0] &&poQuad->Corner[tc]->Pos[1] == Corner[1])
			break;

	//感觉以下做了个旋转
	while (tc != common)
	{
		//float tempc[2] = {poQuad->Corner[3]->Pos[0],poQuad->Corner[3]->Pos[1]};
		Chess_Board_Corner_1* tempc = poQuad->Corner[3];
		Quad_Neighbour_Item_1 tempq = poQuad->Neighbour[3];
		for (int i = 3; i > 0; --i)
		{
			//poQuad->Corner[i]->Pos[0] = poQuad->Corner[i - 1]->Pos[0];
			//poQuad->Corner[i]->Pos[1] = poQuad->Corner[i - 1]->Pos[1];
			poQuad->Corner[i] = poQuad->Corner[i - 1];			poQuad->Neighbour[i] = poQuad->Neighbour[i - 1];
		}
		//poQuad->Corner[0]->Pos[0] = tempc[0];
		//poQuad->Corner[0]->Pos[1] = tempc[1];
		poQuad->Corner[0] = tempc;
		poQuad->Neighbour[0] = tempq;

		tc = (tc + 1) & 3;
	}
	return;
}
int Add_Outer_Quad(Chess_Board_Quad_1 Quad[],int *piQuad_Count,int iMax_Quad_Count,
	Chess_Board_Corner_1 All_Corner[],int *piCorner_Count,	
	Chess_Board_Quad_1 *poQuad, Chess_Board_Quad_1 *Quad_Group[],int *piGroup_Size)
{
	int iQuad_Count = *piQuad_Count,
		iGroup_Size = *piGroup_Size,
		iCorner_Count = *piCorner_Count;

	int added = 0;
	int max_quad_buf_size = iMax_Quad_Count;
	for (int i = 0; i < 4 && iQuad_Count < max_quad_buf_size; i++) // find no-neighbor corners
	{
		if (poQuad->Neighbour[i].m_iQuad_Index==-1)    // ok, create and add neighbor
		{
			int j = (i+2)&3;
			//printf("Adding quad as neighbor 2\n");
			int q_index = iQuad_Count++;
			Chess_Board_Quad_1 *q = &Quad[q_index];
			Quad_Group[iGroup_Size++] = q;
			*q = {};
			q->Neighbour[0] = { 0,-1,-1,NULL };
			q->Neighbour[1] = { 0,-1,-1,NULL };
			q->Neighbour[2] = { 0,-1,-1,NULL };
			q->Neighbour[3] = { 0,-1,-1,NULL};
			added++;
			
			// set neighbor and group id
			poQuad->Neighbour[i].m_iQuad_Index = q_index;
			poQuad->Neighbour[i].ptr = q;
			poQuad->m_iNeighbour_Count++;
			q->Neighbour[j].ptr = poQuad;
			q->Neighbour[j].m_iQuad_Index = (short)(poQuad - Quad);			
			q->m_iGroup_Index= poQuad->m_iGroup_Index;
			q->m_iNeighbour_Count = 1;   // number of neighbors
			q->ordered = false;
			q->edge_sqr_len = poQuad->edge_sqr_len;

			// make corners of new quad
			// same as neighbor quad, but offset
			float pt_offset[2] = { poQuad->Corner[i]->Pos[0] - poQuad->Corner[j]->Pos[0],
				poQuad->Corner[i]->Pos[1] - poQuad->Corner[j]->Pos[1] };
			for (int k = 0; k < 4; k++)
			{
				//Chess_Board_Corner_1 *corner = &All_Corner[q_index * 4 + k];
				Chess_Board_Corner_1 *corner = &All_Corner[iCorner_Count++];
				float *pt = poQuad->Corner[k]->Pos;
				corner->Pos[0] = pt[0],corner->Pos[1] = pt[1],
				q->Corner[k] = corner;
				corner->Pos[0] += pt_offset[0], corner->Pos[1] += pt_offset[1];
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
				Chess_Board_Quad_1* quad_prev = (Chess_Board_Quad_1*)poQuad->Neighbour[prev_i].ptr;
				if (quad_prev &&
					quad_prev->ordered &&
					quad_prev->Neighbour[i].m_iQuad_Index!=-1 &&
					((Chess_Board_Quad_1*)quad_prev->Neighbour[i].ptr)->ordered &&
					std::abs(((Chess_Board_Quad_1*)quad_prev->Neighbour[i].ptr)->col - q->col) == 1 &&
					std::abs(((Chess_Board_Quad_1*)quad_prev->Neighbour[i].ptr)->row  - q->row) == 1)
				{
					Chess_Board_Quad_1* qn = (Chess_Board_Quad_1*)quad_prev->Neighbour[i].ptr;
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
	return added;
}
void Remove_Quad_From_Group(Chess_Board_Quad_1 *Quad_Group[],int *piGroup_Size,Chess_Board_Quad_1 *q0)
{
	int iGroup_Size = *piGroup_Size;
	const int count = iGroup_Size;
	int self_idx = -1;
	// remove any references to this quad as a neighbor
	for (int i = 0; i < count; ++i)
	{
		Chess_Board_Quad_1* q = Quad_Group[i];
		if (q == q0)
			self_idx = i;
		for (int j = 0; j < 4; j++)
		{
			if (q->Neighbour[j].ptr == q0)
			{
				q->Neighbour[j].ptr = NULL;
				q->m_iGroup_Index = -1;
				q->m_iNeighbour_Count--;
				for (int k = 0; k < 4; ++k)
				{
					if (q0->Neighbour[k].ptr == q)
					{
						q0->Neighbour[k].ptr = NULL;
						q0->Neighbour[k].m_iCorner_Index = -1;
						q0->m_iNeighbour_Count--;
					}
				}
				break;
			}
		}
	}

	if (self_idx != count-1)
		Quad_Group[self_idx] = Quad_Group[count-1];
	*piGroup_Size = count - 1;
}
int iOrder_Found_Connected_Quads(Chess_Board_Quad_1 Quad[], int *piQuad_Count, int iMax_Quad_Count,
	Chess_Board_Corner_1 All_Corner[], int *piCorner_Count, 
	Chess_Board_Quad_1* Quad_Group[],int *piGroup_Size,unsigned char Pattern_Size[2],int iParent_Counter=-1)
{//将乱七八糟的四边形组排列好，具体怎么做还不知道
 //返回个数
	int i;
	//首先确定开始的块
	int iQuad_Count = *piQuad_Count,
		iGroup_Size = *piGroup_Size;

	Chess_Board_Quad_1* poStart = NULL;
	Chess_Board_Quad_1* Stack[128];	//战
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
	int row_min = 0, col_min = 0, row_max=0, col_max = 0;
	//int col_hist[16], row_hist[16];	//又搞直方图？

	Stack[iTop++] = poStart;
	poStart->row = 0;
	poStart->col = 0;
	poStart->ordered = true;
	int iCounter = 0;
	//if (iParent_Counter == 4)
		//Disp_Quads(Quad_Group, iGroup_Size,0);

	//Disp_Quad(Quad[15],0);
	while (iTop>0)
	{
		Chess_Board_Quad_1* q = Stack[--iTop];
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
			Chess_Board_Quad_1* poNeighbour =q->Neighbour[i].m_iQuad_Index==-1?NULL:&Quad[q->Neighbour[i].m_iQuad_Index];
			//Disp_Quad(*poNeighbour,0);
			switch(i)
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
				//if (iParent_Counter == 4 && iCounter == 6)
				//{//已知此处错误
				//	Disp_Quad(*poNeighbour, 0);
				//}
				Order_Quad(Quad, poNeighbour, q->Corner[i]->Pos, (i + 2) & 3);
				if (iParent_Counter == 4/* && iCounter == 6*/)
				{
					printf("Counter:%d\n", iCounter);
					Disp_Quad(*poNeighbour, 0);
				}

				poNeighbour->ordered = 1;
				poNeighbour->row = row;
				poNeighbour->col = col;
				Stack[iTop++] = poNeighbour;
			}
			iCounter++;
		}		
	}
	//if (iParent_Counter == 4)
		//printf("here");
	//Disp_Quad(Quad[5],0);

	int w = Pattern_Size[0] - 1;
	int h = Pattern_Size[1] - 1;
	int drow = row_max - row_min + 1;
	int dcol = col_max - col_min + 1;

	if ((w > h && dcol < drow) || (w < h && drow < dcol))
	{
		h = Pattern_Size[0] - 1;
		w = Pattern_Size[1]- 1;
	}

	if (dcol < w || drow < h)   // found enough inner quads?
	{
		//printf("Too few inner quad rows/cols\n");
		return 0;   // no, return
	}

	int bFound = 0;
	for (int i = 0; i < iGroup_Size; ++i)
	{
		Chess_Board_Quad_1* q = Quad_Group[i];
		if (q->m_iNeighbour_Count != 4)
			continue;
		int col = q->col;
		int row = q->row;
		
		for (int j = 0; j < 4; j++)
		{
			/*if (iParent_Counter == 18 && i==30 && j==1)
				printf("i:%d order:%d \n",i, (Quad_Group[32])->ordered);*/
			switch(j)   // adjust col, row for this quad
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
			Chess_Board_Quad_1* poNeighbour = q->Neighbour[j].m_iQuad_Index==-1?NULL:&Quad[q->Neighbour[j].m_iQuad_Index];
			if (poNeighbour && !poNeighbour->ordered && // is it an inner quad?
				col <= col_max && col >= col_min && row <= row_max && row >= row_min)
			{
				bFound++;
				Order_Quad(Quad,poNeighbour, q->Corner[j]->Pos, (j+2)&3);
				poNeighbour->ordered = 1;
				poNeighbour->row = row;
				poNeighbour->col = col;
			}
		}		
	}
	int max_quad_buf_size = iMax_Quad_Count;
	
	if (bFound > 0)
	{
		//printf("Found %d inner quads not connected to outer quads, repairing\n", bFound);
		//Disp_Quads(Quad_Group, iGroup_Size, 0);
		for (int i = 0; i < iGroup_Size && iQuad_Count < max_quad_buf_size; i++)
		{
			Chess_Board_Quad_1 *q = Quad_Group[i];
			/*if (i == 32)
			{
				printf("Contour:%d\n", i);
				Disp_Quad(*q,0);
			}*/
				
			if (q->m_iNeighbour_Count < 4 && q->ordered)
			{
				/*if (i == 48)
					printf("here");*/
				int added = Add_Outer_Quad(Quad,&iQuad_Count,iMax_Quad_Count, All_Corner, piCorner_Count, q, Quad_Group,&iGroup_Size);
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
			Chess_Board_Quad_1* q = Quad_Group[i];
			//Disp_Quad(*q, 0);

			if (q->ordered == 0)
			{
				int outer = 0;
				for (int j=0; j<4; j++) // any neighbors that are ordered?
				{
					/*if (iParent_Counter == 18 && i == 53 && j == 2)
						printf("Here");*/
					if (q->Neighbour[j].m_iQuad_Index!=-1 && Quad[q->Neighbour[j].m_iQuad_Index].ordered)
						outer = 1;
				}
				if (!outer) // not an outer quad, eliminate
				{
					//printf("Not implemented yet:%d\n",iParent_Counter);
					//exit(0);
					Remove_Quad_From_Group(Quad_Group, &iGroup_Size, q);
					//removeQuadFromGroup(quads, q);
				}
			}
		}		
		return iGroup_Size;
	}
	return 0;
}
int Check_Quad_Group(Chess_Board_Quad_1 Quad[], int iQuad_Count, Chess_Board_Quad_1* Quad_Group[], int iQuad_Group_Size,Chess_Board_Corner_1* Corner_Group_Out[],unsigned char Grid_Size[2])
{//暂时不知其义
	const int ROW1 = 1000000;	
	const int ROW2 = 2000000;
	const int ROW_ = 3000000;

	int iCorner_Count = 0, iCorner_Out_Count = 0;
	int result = 0;

	int width = 0, height = 0;
	int hist[5] = {0,0,0,0,0};	//此处的直方图有没有用?没用就不要了
	Chess_Board_Corner_1* Corner_Group[128];

	for (int i = 0; i < iQuad_Group_Size; ++i)
	{
		Chess_Board_Quad_1* q = Quad_Group[i];
		for (int j = 0; j < 4; ++j)
		{
			//if (i == 23 && j==0)
				//printf("here");
			if (q->Neighbour[j].m_iCorner_Index!=-1)
			{
				int next_j = (j + 1) & 3;
				Chess_Board_Corner_1 *a = q->Corner[j], *b = q->Corner[next_j];
				int row_flag = q->m_iNeighbour_Count == 1 ? ROW1 : q->m_iNeighbour_Count == 2 ? ROW2 : ROW_;
				if (a->row == 0)
				{
					Corner_Group[iCorner_Count++] = a;
					a->row = row_flag;
				}else if (a->row > (unsigned int)row_flag)
				{
					a->row = row_flag;
				}
				if (q->Neighbour[next_j].m_iQuad_Index!=-1)
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
		Chess_Board_Corner_1* first = NULL, * first2 = NULL;
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

		Chess_Board_Corner_1* cur = first;
		Chess_Board_Corner_1* right = NULL;
		Chess_Board_Corner_1* below = NULL;
		Corner_Group_Out[iCorner_Out_Count++] = (cur);

		for (int k = 0; k < 4; ++k)
		{
			Chess_Board_Corner_1* c = cur->Neighbour[k];
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
				Chess_Board_Corner_1* c = cur->Neighbour[k];
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
					Chess_Board_Corner_1* c = cur->Neighbour[k];
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

		Chess_Board_Corner_1** tmp = (Chess_Board_Corner_1**)pMalloc(iCorner_Out_Count * sizeof(Chess_Board_Corner_1*));
		memcpy(tmp, Corner_Group_Out, iCorner_Out_Count * sizeof(Chess_Board_Corner_1*));

		for (int i = 0; i < height; ++i)
			for (int j = 0; j < width; ++j)
				Corner_Group_Out[i*width + j] = tmp[j*height + i];
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

END:
	if (result <= 0)
	{
		iCorner_Count = Min(iCorner_Count, Grid_Size[0]*Grid_Size[1]);
		iCorner_Out_Count = iCorner_Count;
		for (int i = 0; i < iCorner_Count; i++)
			Corner_Group_Out[i] = Corner_Group[i];
		result = -iCorner_Count;

		if (result == -Grid_Size[0]*Grid_Size[1])
			result = -result;
	}

	return result;
}
float fSum_Dist(Chess_Board_Corner_1 oCorner,int *pn)
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
int bCheck_Board_Monotony(float Corner[][2],int iCount, unsigned char Grid_Size[2])
{
	for (int k = 0; k < 2; ++k)
	{
		int max_i = (k == 0 ? Grid_Size[1] : Grid_Size[0]);
		int max_j = (k == 0 ? Grid_Size[0] : Grid_Size[1]) - 1;
		for (int i = 0; i < max_i; ++i)
		{
			float* a = k == 0 ? Corner[i * Grid_Size[0]] : Corner[i];
			float* b = k == 0 ? Corner[(i + 1) * Grid_Size[0] - 1]
			: Corner[( Grid_Size[1]-1)* Grid_Size[0] + i];
			float dx0 = b[0] - a[0], dy0 = b[1] - a[1];
			if (fabs(dx0) + fabs(dy0) < FLT_EPSILON)
				return 0;
			float prevt = 0;
			for (int j = 1; j < max_j; ++j)
			{
				float *c = k == 0 ? Corner[i*Grid_Size[0] + j]
					: Corner[j*Grid_Size[0] + i];
				float t = ((c[0] - a[0])*dx0 + (c[1] - a[1])*dy0)/(dx0*dx0 + dy0*dy0);
				if (t < prevt || t > 1)
					return 0;
				prevt = t;
			}
		}
	}
	return 1;
}

void Draw_Quads(char* pcFile, Chess_Board_Quad_1 Quad[], int iQuad_Count, int bi16=1,int w=1920,int h=1080)
{
	Image oImage;
	Init_Image(&oImage, w, h, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (int i = 0; i < iQuad_Count; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if(bi16)
				Mid_Point_Line(oImage, Quad[i].Corner_i[j][0],Quad[i].Corner_i[j][1],
					Quad[i].Corner_i[(j+1)%4][0],Quad[i].Corner_i[(j+1)%4][1]);
			else
				Mid_Point_Line(oImage,(int)(Quad[i].Corner[j]->Pos[0]), (int)(Quad[i].Corner[j]->Pos[1]),
					(int)(Quad[i].Corner[(j + 1) % 4]->Pos[0]), (int)(Quad[i].Corner[(j + 1) % 4]->Pos[1]));
		}
	}
	bSave_Image(pcFile, oImage);
	return;
}

void Draw_Quads(char* pcFile, Chess_Board_Quad_1 *Quad[], int iQuad_Count,int w=1920,int h=1080)
{
	Image oImage;
	Init_Image(&oImage, w, h, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (int i = 0; i < iQuad_Count; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			Mid_Point_Line(oImage, (int)(Quad[i]->Corner[j]->Pos[0]), (int)(Quad[i]->Corner[j]->Pos[1]),
				(int)(Quad[i]->Corner[(j + 1) % 4]->Pos[0]),(int)( Quad[i]->Corner[(j + 1) % 4]->Pos[1]));
		}
	}
	bSave_Image(pcFile, oImage);
	Free_Image(&oImage);
	return;
}
void Clean_Found_Connected_Quads(Chess_Board_Quad_1 *Quad_Group[], int iGroup_Size, unsigned char Grid_Size[2])
{
	if (iGroup_Size <= ((Grid_Size[0] + 1) * (Grid_Size[1] + 1) + 1) / 2)
		return;
	else
	{
		printf("Not implemented yet\n");
		exit(0);
	}
}
int bProcess_Quad(Chess_Board_Quad_1 Quad[], int* piQuad_Count, int iMax_Quad_Count,
	Chess_Board_Corner_1 All_Corner[], int* piCorner_Count,
	unsigned char Grid_Size[2], 	float Out_Corner[][2],int iParent_Counter=-1)
{//寻找棋盘格
	int iQuad_Count = *piQuad_Count;
	Find_Quad_Neighbour(Quad, iQuad_Count);
	//Disp_Quads(Quad, iQuad_Count,0);

	int iGroup_Size, bRet = 0;
	const int iMax_Group_Size = Grid_Size[0] * Grid_Size[1];;
	Chess_Board_Quad_1** Quad_Group = (Chess_Board_Quad_1**)pMalloc(iMax_Group_Size * sizeof(Chess_Board_Quad_1*));
	Chess_Board_Corner_1** Corner_Group =(Chess_Board_Corner_1**)pMalloc(iMax_Group_Size * 4 * sizeof(Chess_Board_Corner_1*));
	static int iCounter = 0;
	int iCorner_Count = *piCorner_Count;
	/*if (iParent_Counter == 2)
		printf("Here");*/
	for (int iGroup_Index = 0;; iGroup_Index++)
	{
		iCounter++;
		//前面已经做了所有四边形的近邻搜索，以下找出每一组连刀肉
		Find_Connected_Quads(Quad,iQuad_Count,Quad_Group,iGroup_Index,&iGroup_Size);
		//printf("Group Size:%d\n", iGroup_Size);
		//Disp_Quads(Quad_Group, iGroup_Size, 0);
		//Draw_Quads("c:\\tmp\\1.bmp", Quad_Group,iGroup_Size);

		if (!iGroup_Size)
			break;
		//if (iParent_Counter == 5 && iGroup_Index == 0)
			//printf("here\n");
		//	//Disp_Quad(*Quad_Group[0],0);
		/*if(iCounter==18)
			Disp_Quads(Quad_Group, iGroup_Size, 0);*/

		int iCount = iOrder_Found_Connected_Quads(Quad, &iQuad_Count, iMax_Quad_Count,
			All_Corner, piCorner_Count, Quad_Group,&iGroup_Size,Grid_Size,iCounter);
		//if (iParent_Counter == 1 && iGroup_Index == 1)
		//	//printf("here\n");
		//	//Disp_Quad(*Quad_Group[0],0);
		//	Disp_Quads(Quad_Group, iGroup_Size, 0);
		if (!iCount)
			continue;

		//此处还差一道工序，把多余的Quad去掉
		Clean_Found_Connected_Quads(Quad_Group, iGroup_Size, Grid_Size);

		//Disp_Quads(Quad_Group, iGroup_Size,0);

		iCount = Check_Quad_Group(Quad, iQuad_Count, Quad_Group, iGroup_Size, Corner_Group,Grid_Size);

		int n = iCount > 0 ? Grid_Size[0] * Grid_Size[1] : -iCount;
		float sum_dist = 0;
		int total = 0;

		for(int i = 0; i < n; i++ )
		{
			int ni = 0;
			float sum = fSum_Dist(*Corner_Group[i],&ni);
			sum_dist += sum;
			total += ni;
		}
		float prev_sqr_size = (float)round((double)sum_dist/Max(total, 1));
		for (int i = 0; i < n; i++)
			Out_Corner[i][0] = Corner_Group[i]->Pos[0], Out_Corner[i][1] = Corner_Group[i]->Pos[1];

		if (iCount == Grid_Size[0] * Grid_Size[1] && bCheck_Board_Monotony(Out_Corner, n, Grid_Size))
		{
			bRet = 1;
			break;
		}
		iCounter++;
	}
	if (Quad_Group)
		Free(Quad_Group);
	if (Corner_Group)
		Free(Corner_Group);
	return bRet;
}
void bProcess_Quad(Chess_Board_Quad Quad[], int iQuad_Count,unsigned char Pattern_Size[2])
{//寻找棋盘格
	Find_Quad_Neighbour(Quad, iQuad_Count);
	//Disp_Quads(Quad, iQuad_Count,0);
	Chess_Board_Quad* Quad_Group[128];
	Chess_Board_Corner* Corner_Group[128];

	int iGroup_Size;
	for (int iGroup_Index = 0;; iGroup_Index++)
	{
		Find_Connected_Quads(Quad,iQuad_Count,Quad_Group,iGroup_Index,&iGroup_Size);
		if (!iGroup_Size)
			break;

		int iCount = iOrder_Found_Connected_Quads(Quad, iQuad_Count, Quad_Group,iGroup_Size,Pattern_Size);
		Convert_Corner(Quad_Group, iGroup_Size);
		Check_Quad_Group(Quad, iQuad_Count, Quad_Group, iGroup_Size, Corner_Group,Pattern_Size);
	}

	return;
}
void Quad_i16_2_float(Chess_Board_Quad* pQuad, int iQuad_Count)
{
	for (int i = 0; i < iQuad_Count; i++)
	{
		Chess_Board_Quad *poQuad = &pQuad[i];
		unsigned short* pValue_16 = &poQuad->Corner_i[3][1];
		float* pValue_f = &poQuad->Corner_f[3][1];
		//Disp_Quad(oQuad);
		for (int j = 0; j < 8; j++)
			*pValue_f-- = *pValue_16--;
		//Disp_Quad(oQuad,0);
	}
	return;
}
void Quad_i16_2_float(Chess_Board_Quad_1* Quad, int iQuad_Count,Chess_Board_Corner_1 Corner[],int *piCur_Corner)
{
	int iCur_Corner = *piCur_Corner;
	for (int i = 0; i < iQuad_Count; i++)
	{
		Chess_Board_Quad_1 *poQuad = &Quad[i];
		for (int j = 3; j >= 0; j--)
		{
			Chess_Board_Corner_1* poCorner = &Corner[iCur_Corner++];
			*poCorner = {};
			poCorner->Pos[0] = poQuad->Corner_i[j][0];
			poCorner->Pos[1] = poQuad->Corner_i[j][1];
			poQuad->Corner[j] = poCorner;
		}
	}
	*piCur_Corner = iCur_Corner;
	return;
}
typedef struct Term_Criteria{
	char m_iType;
	char m_iMax_Count;
	float eps;
}Term_Criteria;

void Get_Rect_Sub_Pix(Image oImage, int Patch_Size[2], float Center_0[2],
	float Patch[], int Patch_Type=5)
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
	a = Max(a,0.0001f);
	float a12 = a*(1.f-b);
	float a22 = a*b;
	float b1 = 1.f - b;
	float b2 = b;
	double s = (1. - a)/a;
	unsigned char *src = oImage.m_pChannel[0];

	//src_step /= sizeof(src[0]);
	//dst_step /= sizeof(dst[0]);
	int src_step = oImage.m_iWidth, dst_step = Patch_Size[0];
	src += ip[1] * src_step + ip[0];

	for (; win_size[1]--; src += src_step, dst += dst_step)
	{
		float prev = (1 - a)*(b1*src[0] + b2*src[src_step]);
		for( int j = 0; j < win_size[0]; j++ )
		{
			float t = a12*src[j+1] + a22*src[j+1+src_step];
			dst[j] = prev + t;
			prev = (float)(t*s);
		}
	}

	return;
}
void Corner_SubPix(Image oImage, float Corner[][2],int iCorner_Count,
	int Win[2], int Zero_Zone[2],Term_Criteria criteria = {})
{
	const int MAX_ITERS = 100;
	int win_w = Win[0] * 2 + 1, win_h =Win[1] * 2 + 1;
	int i, j, k;
	int max_iters = 15;
	double eps = 0.1f;
	eps *= eps;

	float *mask,* maskm = (float*)pMalloc(win_w * win_h * sizeof(float));
	mask = maskm;	//啥玩意

	for (i = 0; i < win_h; i++)
	{
		float y = (float)(i - Win[1]) / Win[1];
		float vy = (float)exp(-y * y);
		for( j = 0; j < win_w; j++ )
		{
			float x = (float)(j - Win[0]) / Win[0];
			mask[i * win_w + j] = (float)(vy*exp(-x*x));
		}
	}
	if (Zero_Zone[0] >= 0 && Zero_Zone[1] >= 0 && Zero_Zone[0] * 2 + 1 < win_w && Zero_Zone[1] * 2 + 1 < win_h)
	{
		for( i = Win[1] - Zero_Zone[1]; i <= Win[1] + Zero_Zone[1]; i++)
			for( j = Win[0] - Zero_Zone[0]; j <= Win[0] + Zero_Zone[0]; j++ )
				mask[i * win_w + j] = 0;
	}
	float Subpix_Buf[7 * 7];
	float cI2[2] = {};
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
			const float* subpix = &Subpix_Buf[1*Patch_Size[0] + 1];
			for (i = 0, k = 0; i < win_h; i++, subpix += win_w + 2)
			{
				double py = i - Win[1];
				for (j = 0; j < win_w; j++, k++)
				{
					double m = mask[k];
					double tgx = subpix[j+1] - subpix[j-1];
					double tgy = subpix[j+win_w+2] - subpix[j-win_w-2];
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

			double det=a*c-b*b;
			if( fabs( det ) <= DBL_EPSILON*DBL_EPSILON )
				break;

			// 2x2 matrix inversion
			double scale=1.0/det;
			cI2[0] = (float)(cI[0] + c * scale * bb1 - b * scale * bb2);
			cI2[1] = (float)(cI[1] - b*scale*bb1 + a*scale*bb2);
			err = (cI2[0] - cI[0]) * (cI2[0] - cI[0]) + (cI2[1] - cI[1]) * (cI2[1] - cI[1]);
			// if new point is out of image, leave previous point as the result
			if(cI2[0]<0 || cI2[0]>oImage.m_iWidth-1 || cI2[1]<0 || cI2[1]>oImage.m_iHeight-1 )
				break;
			//cI = cI2;
			cI[0] = cI2[0], cI[1] = cI2[1];
		}while( ++iter < max_iters && err > eps );
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
	return;
}
static void generateQuads_Test()
{
	Image oImage;
	bLoad_Image("c:\\tmp\\chess_board.bmp", &oImage);
	Chess_Board_Quad_1* pQuad;
	int iMax_Quad_Count, iQuad_Count, iCur_Corner = 0;
	Generate_Quad(oImage,&pQuad,&iQuad_Count,&iMax_Quad_Count,0);

	/*Chess_Board_Corner_1* pCorner = (Chess_Board_Corner_1*)pMalloc(iQuad_Count * 4);
	Quad_i16_2_float(pQuad, iQuad_Count, pCorner, &iCur_Corner);

	unsigned char Grid_Size[2] = { 3,3 };
	float Corner[128][2];
	bProcess_Quad(pQuad, iQuad_Count,Grid_Size,Corner);

	int Win[] = { 2,2, }, Zero_Zone[] = {-1,-1};
	Corner_SubPix(oImage, Corner, Grid_Size[0] * Grid_Size[1],Win,Zero_Zone);
	Disp((float*)Corner, Grid_Size[0] * Grid_Size[1], 2, "Corner");*/

	Free(pQuad);
	return;
}
void Box_Filter_Test()
{
	Image oSource, oDest;
	bLoad_Image("c:\\tmp\\Chess_Board_1.bmp", &oSource);
	Init_Image(&oDest, oSource.m_iWidth, oSource.m_iHeight, Image::IMAGE_TYPE_BMP, oSource.m_iBit_Count);
	unsigned long long tStart = iGet_Tick_Count();
	for(int i=0;i<1000;i++)
		Box_Filter(oSource, oDest,60);
	printf("%lld\n",iGet_Tick_Count()-tStart);
	bSave_Image("c:\\tmp\\1.bmp", oDest);
	Free_Image(&oSource);
	Free_Image(&oDest);

	bLoad_Image("c:\\tmp\\1.bmp", &oSource);
	bLoad_Image("c:\\tmp\\2.bmp", &oDest);
	Compare_Image(oSource, oDest,1);
	
	return;
}
void Adaptive_Threshold(Image oSource, Image oDest, int iBox_Filter_r,int iDelta)
{//搞个阈值生成二值化图像

	Box_Filter(oSource, oDest, iBox_Filter_r);
	//bSave_Image("c:\\tmp\\1.bmp", oDest);
	//Compare_Image("c:\\tmp\\1.bmp", "c:\\tmp\\2.bmp");
	unsigned char tab[768];	//什么玩意？
	int i;
	const int iMax_Val = 255;
	//什么意义？为什么要搞这么麻烦？
	for( i = 0; i < 768; i++ )
		tab[i] = (unsigned char)(i - 255 > -iDelta ? iMax_Val : 0);

	int iSize = oSource.m_iWidth * oSource.m_iHeight;
	for (int iChannel = 0; iChannel < oSource.m_iChannel_Count; iChannel++)
	{
		unsigned char* pSource = oSource.m_pChannel[iChannel],
			* pDest = oDest.m_pChannel[iChannel];
		for (i = 0; i < iSize; i++)
			pDest[i] = tab[pSource[i] - pDest[i] + 255];
	}

	//bSave_Image("c:\\tmp\\1.bmp", oDest);
	return;
}
void adaptiveThreshold_Test()
{
	Image oSource, oDest;
	bLoad_Image("c:\\tmp\\Chess_Board.bmp", &oSource);
	Init_Image(&oDest, oSource.m_iWidth, oSource.m_iHeight, Image::IMAGE_TYPE_BMP, oSource.m_iBit_Count);
	Adaptive_Threshold(oSource, oDest, 60,0);
	return;
}

//void Dilate_1(Image oSource, Image oDest, int r)
//{//更慢，失败
//	typedef struct Elem {
//		unsigned char m_iOrg : 1;	//原图值
//		unsigned char m_iValue : 1;	//新值
//	}Elem;
//	Elem * pTranspose = (Elem*)pMalloc(oSource.m_iWidth * oSource.m_iHeight*sizeof(Elem));
//	const int iWidth_Minus_1 = oSource.m_iWidth - 1,
//		iHeight_Minus_1 = oSource.m_iHeight - 1;
//	int bFast_Row_Process = oSource.m_iWidth >= 2 * r + 1?1 : 0,
//		bFast_Col_Process = oSource.m_iHeight >= 2 * r + 1 ? 1 : 0;
//
//	//先搞行
//	for (int y = 0; y < oSource.m_iHeight; y++)
//	{
//		int x, iLeft = -r;	// , iRight = r;l
//		unsigned char* pSource_Line = &oSource.m_pChannel[0][y * oSource.m_iWidth];
//		//此处Dest视为转置矩阵
//		Elem* pDest_Line = &pTranspose[y];
//		unsigned int iTotal = r * (!!pSource_Line[0]);
//		if (oSource.m_iWidth < r + 1)
//		{//根本不够像素,草草了事
//			for (x = 0; x < oSource.m_iWidth; x++)
//				iTotal += !!pSource_Line[x];
//			//剩下不足的补行尾
//			iTotal += (!!pSource_Line[iWidth_Minus_1]) * (r + 1 - oSource.m_iWidth);
//		}else
//		{
//			for (x = 0; x <= r; x++)
//				iTotal += !!pSource_Line[x];
//		}
//		pDest_Line[0] = { !!pSource_Line[0],!!iTotal };
//		pDest_Line += oDest.m_iHeight;
//
//		int iRight = r + 1;
//		if (bFast_Row_Process)
//		{
//			//从[0]移动到[r]
//			int iMove_To = r + 1;
//			for (x = 1; x < iMove_To; x++)
//			{
//				iTotal += (!!pSource_Line[iRight++]) - (!!pSource_Line[0]);
//				*pDest_Line = { !!pSource_Line[x], !!iTotal };
//				iLeft++;
//				pDest_Line += oDest.m_iHeight;
//			}
//
//			//从[r+1]移动到[width-1-r]
//			iMove_To = iWidth_Minus_1 - r;
//			for (; x < iMove_To; x++)
//			{
//				/*if (y == 110 && x == 100)
//				{
//					printf("Here");
//					*pDest_Line = {};
//				}*/
//
//				iTotal += (!!pSource_Line[iRight++]) - (!!pSource_Line[iLeft++]);
//				*pDest_Line = { !!pSource_Line[x],!!iTotal };
//				pDest_Line += oDest.m_iHeight;
//			}
//
//			//从[width-1-r]移动到[width-1]
//			for (; x < oSource.m_iWidth; x++)
//			{
//				iTotal += (!!pSource_Line[iWidth_Minus_1]) - (!!pSource_Line[iLeft++]);
//				*pDest_Line = { !!pSource_Line[x],!!iTotal };
//				pDest_Line += oDest.m_iHeight;
//			}
//		}else
//		{
//			for (x = 1; x < oSource.m_iWidth; x++)
//			{
//				iTotal -= iLeft < 0 ? !!pSource_Line[0] : !!pSource_Line[iLeft];
//				iTotal += iRight >= oSource.m_iWidth ? !!pSource_Line[iWidth_Minus_1] : !!pSource_Line[iRight];
//				*pDest_Line = { !!pSource_Line[x],!!iTotal };
//				iLeft++, iRight++;
//				pDest_Line += oDest.m_iHeight;
//			}
//		}
//	}
//
//	//再来一次，还是行推进
//	for (int y = 0; y < oSource.m_iWidth; y++)
//	{
//		int x, iLeft = -r;	// , iRight = r;
//		Elem* pSource_Line = &pTranspose[y * oSource.m_iHeight];
//		//此处要改改，将Dest视为转置矩阵
//		unsigned char* pDest_Line = &oDest.m_pChannel[0][y];
//		unsigned int iTotal = r * pSource_Line[0].m_iOrg;
//		if (oSource.m_iHeight < r + 1)
//		{//根本不够像素,草草了事
//			for (x = 0; x < oSource.m_iHeight; x++)
//				iTotal += pSource_Line[x].m_iOrg;
//			iTotal += pSource_Line[iHeight_Minus_1].m_iOrg * (r + 1 - oSource.m_iHeight);
//		}else
//		{
//			for (x = 0; x <= r; x++)
//				iTotal += pSource_Line[x].m_iOrg;
//		}
//		pDest_Line[0] = (pSource_Line[0].m_iValue || (!!iTotal))*255;
//		pDest_Line += oDest.m_iWidth;
//		int iRight = r + 1;
//		//以下要非常清醒，是从转置当中算数据
//		if (bFast_Col_Process)
//		{//此处尽可能用快速方法
//			int iMove_To = r + 1;
//			for (x = 1; x < iMove_To; x++)
//			{
//				iTotal += pSource_Line[iRight++].m_iOrg - pSource_Line[0].m_iOrg;
//				*pDest_Line = (pSource_Line[x].m_iValue || (!!iTotal)) * 255;
//				pDest_Line += oDest.m_iWidth;
//			}
//			iLeft += r;
//
//			iMove_To = iHeight_Minus_1 - r;
//			for (; x < iMove_To; x++)
//			{
//				/*if (y == 100 && x == 110)
//					printf("here");*/
//
//				iTotal += pSource_Line[iRight++].m_iOrg - pSource_Line[iLeft++].m_iOrg;
//				*pDest_Line = (pSource_Line[x].m_iValue || (!!iTotal)) * 255;;
//				pDest_Line += oDest.m_iWidth;
//			}
//
//			//从[Height-1-r]移动到[Height-1]
//			for (; x < oSource.m_iHeight; x++)
//			{
//				iTotal += pSource_Line[iHeight_Minus_1].m_iOrg - pSource_Line[iLeft++].m_iOrg;
//				*pDest_Line =  (pSource_Line[x].m_iValue || (!!iTotal)) * 255;;
//				pDest_Line += oDest.m_iWidth;
//			}
//		}else
//		{
//			for (x = 1; x < oSource.m_iHeight; x++)
//			{
//				iTotal -= iLeft < 0 ? pSource_Line[0].m_iOrg : pSource_Line[iLeft].m_iOrg;
//				iTotal += iRight >= oSource.m_iHeight ? pSource_Line[iHeight_Minus_1].m_iOrg : pSource_Line[iRight].m_iOrg;
//				//注意不要删下面这行，用来对数据
//				*pDest_Line = (pSource_Line[x].m_iValue || (!!iTotal)) * 255;;
//				iLeft++, iRight++;
//				pDest_Line += oDest.m_iWidth;
//			}
//		}	
//	}
//
//	Free(pTranspose);
//	return;
//}
//void Dilate_Copy(Image oSource, Image oDest)
//{
//	//先抄到Dest
//	union {
//		struct {
//			unsigned char* pSource_8;
//			unsigned char* pDest_8;
//		};
//		struct {
//			unsigned long long* pSource_64;
//			unsigned long long* pDest_64;
//		};
//	};
//
//	int i,iSize;
//	iSize = oSource.m_iWidth * oSource.m_iHeight / 8;
//	pSource_64 = (unsigned long long*)oSource.m_pChannel[0];
//	pDest_64 = (unsigned long long*)oDest.m_pChannel[0];
//	for (i = 0; i < iSize; i++,pSource_64++,pDest_64++)
//	{
//		switch (*pSource_64)
//		{
//		case 0x0:
//			*pDest_64 = 0;
//			break;
//		case 0xFFFFFFFFFFFFFFFF:
//			*pDest_64 = 0x0101010101010101;	//等于超入org=1
//			break;
//		default:
//			for (int j = 0; j < 8; j++)
//				pDest_8[j] = !!pSource_8[j];
//			break;
//		}
//	}
//	i *= 8;
//	iSize = oSource.m_iWidth * oSource.m_iHeight;
//	for (; i < iSize; i++)
//		oDest.m_pChannel[0][i] = !!oSource.m_pChannel[0][i];
//
//	return;
//}

void Dilate_Test()
{
	Image oSource, oDest;
	bLoad_Image("c:\\tmp\\1920_1080_2.bmp", &oSource);
	Init_Image(&oDest, oSource.m_iWidth, oSource.m_iHeight, Image::IMAGE_TYPE_BMP, oSource.m_iBit_Count);
	unsigned long long tStart = iGet_Tick_Count();
	for(int i=0;i<1000;i++)
		Dilate_Bin(oSource,1, oDest);
	printf("%lld\n", iGet_Tick_Count() - tStart);
	/*bSave_Image("c:\\tmp\\1.bmp", oDest);

	bLoad_Image("c:\\tmp\\1.bmp", &oSource);
	bLoad_Image("c:\\tmp\\2.bmp", &oDest);
	Compare_Image(oSource, oDest);*/
	return;
}
void Get_Hist_256(Image oImage, int Hist[256])
{//只搞灰度图，求个直方图，毫无营养
	int iSize = oImage.m_iWidth * oImage.m_iHeight;
	memset(Hist, 0, 256 * sizeof(int));
	for (int i = 0; i < iSize; i++)
		Hist[oImage.m_pChannel[0][i]]++;
	return;
}
void Smooth_Hist(int Source[], int Dest[], int iCount, int r)
{//r为窗口半径，其实就是取窗口的平均，没啥营养
	for (int i = 0; i < iCount; i++)
	{
		//以下和传统的窗口有点不一样
		int iIdx_min = Max(0, i - r);
		int iIdx_max = Min(255, i + r);
		int iSmooth = 0;
		for (int iIdx = iIdx_min; iIdx <= iIdx_max; ++iIdx)
			iSmooth += Source[iIdx];
		Dest[i] = iSmooth/(iIdx_max-iIdx_min+1);
	}
	return;
}
void Grad_Hist(int Hist[],int Grad[],int iCount)
{//算个梯度？
	Grad[0] = 0;
	int prev_grad = 0;
	for (int i = 1; i < iCount; ++i)
	{
		//没毛病，零个直方图项之间的距离恒为1，所以不用除
		int grad = Hist[i - 1] - Hist[i + 1];
		if (Abs(grad) < 100)
		{
			if (prev_grad == 0)
				grad = -100;
			else
				grad = prev_grad;
		}
		Grad[i] = grad;
		prev_grad = grad;
	}
	Grad[255] = 0;
}
void Binerize(Image oSource,Image oDest)
{
	int Hist[256], Hist_Smooth[256], Hist_Grad[256];
	//先来个直方图
	Get_Hist_256(oSource, Hist);
	//加窗平滑直方图
	Smooth_Hist(Hist, Hist_Smooth,256,1);
	//给平滑直方图算个梯度
	Grad_Hist(Hist_Smooth, Hist_Grad, 256);

	int iMax_Pix = oSource.m_iWidth*oSource.m_iHeight;
	int iMax_Pix_1 = iMax_Pix/100;
	int Max_Pos[20] = {};

	//感觉也没啥乱用，只是挑出些概率高的
	unsigned iMaximun_Count = 0;
	for (int i = 256-2; (i > 2) && (iMaximun_Count < 20); --i)
	{
		if ((Hist_Grad[i-1] < 0) && (Hist_Grad[i] > 0))
		{
			//窗口和
			int iSum_Around_Max = Hist_Smooth[i-1] + Hist_Smooth[i] + Hist_Smooth[i+1];
			//放入Max_Pos条件：1，窗口和>像素总数的百分之一
			//					2，灰度要在64以上
			if (!(iSum_Around_Max < iMax_Pix_1 && i < 64))
				Max_Pos[iMaximun_Count++] = i;
		}
	}

	//Disp(Max_Pos, 1, 20, "Max_Pos");
	int iThresh = 0;	//阀值？初始化为0
	if (iMaximun_Count == 0)
	{//这又是什么情况？
		const int iMax_Pix_2 = iMax_Pix / 2;
		for (int iSum = 0, i = 0; i < 256; ++i) // select mean intensity
		{
			iSum += Hist[i];
			if (iSum > iMax_Pix_2)
			{
				iThresh = i;
				break;
			}
		}
	}else if (iMaximun_Count == 1)
		iThresh = Max_Pos[0]/2;
	else if (iMaximun_Count == 2)
		iThresh = (Max_Pos[0] + Max_Pos[1])/2;
	else // iCntMaxima >= 3
	{
		int iIdx_Acc_Sum = 0, iAccum = 0;
		for (int i = 256 - 1; i > 0; --i)
		{
			iAccum += Hist[i];
			//此处有个很傻逼的判断，从最亮向下数起，一旦》5%左右，1/18，
			//就当这个颜色是棋盘颜色
			if (iAccum > (iMax_Pix / 18))
			{
				iIdx_Acc_Sum = i;
				break;
			}
		}

		//寻找北京色，白色
		unsigned iIdx_BG_Max = 0;
		int iBright_Max = Max_Pos[0];
		for (unsigned n = 0; n < iMaximun_Count - 1; ++n)
		{
			iIdx_BG_Max = n + 1;
			if ( Max_Pos[n] < iIdx_Acc_Sum )
				break;
			iBright_Max = Max_Pos[n];
		}

		// CHECKING THRESHOLD FOR BLACK
		int iMax_Val = Hist[Max_Pos[iIdx_BG_Max]];

		//IF TOO CLOSE TO 255, jump to next maximum
		if (Max_Pos[iIdx_BG_Max] >= 250 && iIdx_BG_Max + 1 < iMaximun_Count)
		{
			iIdx_BG_Max++;
			iMax_Val = Hist[Max_Pos[iIdx_BG_Max]];
		}

		for (unsigned n = iIdx_BG_Max + 1; n < iMaximun_Count; n++)
		{//不断向暗处推进，看看有没有更暗的区间占比更大
			if (Hist[Max_Pos[n]] >= iMax_Val)
			{
				iMax_Val = Hist[Max_Pos[n]];
				iIdx_BG_Max = n;
			}
		}
		//取其中点
		int iDist2 = (iBright_Max - Max_Pos[iIdx_BG_Max])/2;
		iThresh = iBright_Max - iDist2;
	}

	if (iThresh > 0)
	{
		int iSize = oSource.m_iWidth * oSource.m_iHeight;
		for (int i = 0; i < iSize; i++)
			oDest.m_pChannel[0][i]=oSource.m_pChannel[0][i] >= iThresh ? 255 : 0;
	}
	return;
}
void Binarize_Test()
{//二值化测试
	Image oSource, oDest;
	bLoad_Image("c:\\tmp\\Scene_Cut_A_Mono.bmp", &oSource);	
	Init_Image(&oDest, oSource.m_iWidth, oSource.m_iHeight, Image::IMAGE_TYPE_BMP, 8);;

	Binerize(oSource,oDest);
	bSave_Image("c:\\tmp\\1.bmp", oDest);
	Free_Image(&oDest);

	bLoad_Image("c:\\tmp\\1.bmp", &oSource);
	bLoad_Image("c:\\tmp\\2.bmp", &oDest);
	Compare_Image(oSource, oDest);

	return;
}

int bClose_To_Border(float Corner[][2], int iCount, Image oImage)
{
	const int BORDER = 8;
	for (int k = 0; k < iCount; ++k)
	{
		if (Corner[k][0] <= BORDER || Corner[k][0] > oImage.m_iWidth - BORDER ||
			Corner[k][1] <= BORDER || Corner[k][1] > oImage.m_iHeight - BORDER )
			return 1;
	}
	return 0;
}
void Draw_Corner(Image oImage, float Corner[][2], int iCount)
{
	int i;
	for (i = 0; i < iCount; i++)
		Draw_Point(oImage, (int)Corner[i][0], (int)Corner[i][1]);
}
void Draw_Corner(char* pcFile, Image oImage, float Corner[][2], int iCount)
{
	Draw_Corner(oImage, Corner, iCount);
	bSave_Image(pcFile, oImage);
}

void Adjust_Corner(float Corner[][2], int iCount,unsigned char Grid_Size[2])
{
	//长与高都是偶数才需要调整
	if ((Grid_Size[0] & 1) == 0 && (Grid_Size[1] & 1) == 0)
	{
		int last_row = (Grid_Size[1] - 1) * Grid_Size[0];
		double dy0 = Corner[last_row][1] - Corner[0][1];
		if (dy0 < 0)
		{//显然，头向下尾朝天了，来个头尾交换
			int n = Grid_Size[0] * Grid_Size[1];
			for(int i = 0; i < n/2; i++ )
				std::swap(Corner[i], Corner[n-i-1]);
		}
	}
}
int bFind_Chess_Board_Corner(Image oImage,unsigned char Grid_Size[2], float Corner[][2])
{
	//备份一下原图
	Image oImage_1;
	Init_Image(&oImage_1, oImage.m_iWidth, oImage.m_iHeight, Image::IMAGE_TYPE_BMP, 8);

	//先来个二值化
	Binerize(oImage, oImage_1);
	//bSave_Image("c:\\tmp\\1.bmp", oImage_1);

	Chess_Board_Quad_1* pQuad=NULL;
	int iQuad_Count, iCur_Corner = 0, iCorner_Count = Grid_Size[0] * Grid_Size[1],
		iResult = 0, iMin_Dilations = 0, iMax_Dilations = 7;

	//第一部分，简单判断，可以看出，只是简单膨胀，不做二值化的阀值调整，
	//可以想象，万一棋盘黑白分不出来，无论膨胀多少也没用
	int iDilation;
	for (iDilation = iMin_Dilations; !iResult && iDilation <= iMax_Dilations; iDilation++)
	{
		/*if (iDilation == 6)
			printf("Here");*/

		if (iDilation > 0)
			Dilate_Bin(oImage_1);

		//加一条3像素的边
		Draw_Rect(oImage_1, 0, 0, oImage_1.m_iWidth, oImage_1.m_iHeight, 3);

		//Compare_Image(oImage, oRef);
		int iMax_Quad_Count;
		Generate_Quad(oImage_1, &pQuad, &iQuad_Count,&iMax_Quad_Count,iDilation);
		//Draw_Quads("c:\\tmp\\1.bmp", pQuad, iQuad_Count, 1,oImage.m_iWidth, oImage.m_iHeight);
		//Disp_Quad(pQuad[0]);
		//Disp_Quads(pQuad, iQuad_Count);

		int iMax_Corner_Count = iMax_Quad_Count * 4;
		Chess_Board_Corner_1* pCorner = (Chess_Board_Corner_1*)pMalloc(iMax_Corner_Count * sizeof(Chess_Board_Corner_1));
		iCur_Corner = 0;
		Quad_i16_2_float(pQuad, iQuad_Count, pCorner, &iCur_Corner);

		iResult = bProcess_Quad(pQuad, &iQuad_Count,iMax_Quad_Count,pCorner, &iCur_Corner, Grid_Size, Corner,iDilation);

		Free(pCorner);
		Free(pQuad);
	}
	//Disp((float*)Corner, iCorner_Count, 2, "Corner");
	//感觉此处不必做 checkBoardMonotony，因为在bProcess_Quad已经做完
	//iResult = 0;
	if (!iResult)
	{//前面尚未成功，还需努力
		Image oPre_Image;
		Init_Image(&oPre_Image, oImage.m_iWidth, oImage.m_iHeight, Image::IMAGE_TYPE_BMP, 8);

		int iMax_K = 6;	//自适应，就是改个二值化阀值
		int iPrev_Sqr_Size = 0;
		for (int k = 0; k < iMax_K && !iResult; k++)
		{
			int iPrev_Block_Size = -1;
			for (int iDilation = iMin_Dilations;!iResult && iDilation <= iMax_Dilations; iDilation++)
			{
				int block_size = (int)round(iPrev_Sqr_Size == 0
					? std::min(oImage_1.m_iWidth, oImage_1.m_iHeight) * (k % 2 == 0 ? 0.2 : 0.1)
					: iPrev_Sqr_Size * 2);
				block_size = block_size | 1;
				if (block_size != iPrev_Block_Size)
				{
					Adaptive_Threshold( oImage, oImage_1, block_size>>1, (k/2)*5 );
					if(iDilation)
						Dilate_Bin( oImage_1,iDilation);
					//bSave_Image("c:\\tmp\\1.bmp", oImage_1);
					//Compare_Image("c:\\tmp\\1.bmp", "c:\\tmp\\2.bmp");

					memcpy(oPre_Image.m_pBuffer, oImage_1.m_pBuffer, oImage_1.m_iWidth * oImage_1.m_iHeight);
				}else if (iDilation > 0)
				{
					Dilate_Bin( oPre_Image,  1 );
					memcpy(oImage_1.m_pBuffer, oPre_Image.m_pBuffer, oImage_1.m_iWidth * oImage_1.m_iHeight);
				}
				iPrev_Block_Size = block_size;

				Draw_Rect(oImage_1, 0, 0, oImage_1.m_iWidth, oImage_1.m_iHeight, 3);
				//bSave_Image("c:\\tmp\\1.bmp", oImage_1);
				//Compare_Image("c:\\tmp\\1.bmp", "c:\\tmp\\2.bmp");
				int iMax_Quad_Count;
				Generate_Quad(oImage_1, &pQuad, &iQuad_Count,&iMax_Quad_Count,iDilation);
				//Disp_Quads(pQuad, iQuad_Count);

				int iMax_Corner_Count = iMax_Quad_Count * 4;
				Chess_Board_Corner_1* pCorner = (Chess_Board_Corner_1*)pMalloc(iMax_Corner_Count * sizeof(Chess_Board_Corner_1));
				iCur_Corner = 0;
				Quad_i16_2_float(pQuad, iQuad_Count, pCorner, &iCur_Corner);

				iResult = bProcess_Quad(pQuad, &iQuad_Count,iMax_Quad_Count, pCorner, &iMax_Corner_Count,Grid_Size, Corner,iDilation);
				//Disp(Corner[0], 1, 2, "Corner");
				//Disp_Quads(pQuad, iQuad_Count, 0);

				Free(pCorner);
				Free(pQuad);

			}
		}
		Free_Image(&oPre_Image);		
	}

	//判断角点是否在图像边缘
	if(iResult)
		iResult = !bClose_To_Border(Corner,iCorner_Count, oImage_1);

	//Draw_Corner("c:\\tmp\\1.bmp", oImage, Corner, iCorner_Count);
	//偶数格数边长要多道工序
	//Disp(Corner[0], 1, 2, "Corner");
	//bSave_Image("c:\\tmp\\1.bmp", oImage);
	//Compare_Image("c:\\tmp\\1.bmp", "c:\\tmp\\2.bmp");

	if (iResult)
	{
		Adjust_Corner(Corner, iCorner_Count,Grid_Size);
		int Win[] = { 2,2, }, Zero_Zone[] = {-1,-1};
		Corner_SubPix(oImage, Corner, iCorner_Count,Win,Zero_Zone);
		//Disp((float*)Corner, iCorner_Count, 2, "Corner");
		//Draw_Corner(oImage, Corner, iCorner_Count);
		//bSave_Image("c:\\tmp\\1.bmp", oImage);
		//Disp_Mem();
	}

	Free_Image(&oImage_1);
	return iResult;
}

void Find_Chess_Board_Corner_Test_1()
{
	Image oImage;
	if (!bLoad_Image("c:\\tmp\\Chess_Board_1.bmp", &oImage))
		return;

#define Grid_Size_h 8
#define Grid_Size_W 11
	unsigned char Grid_Size[2] = { Grid_Size_W,Grid_Size_h };
	float Corner[Grid_Size_W * Grid_Size_h][2];
	int iResult= bFind_Chess_Board_Corner(oImage, Grid_Size, Corner);
	printf("Result:%d\n", iResult);

	Free_Image(&oImage);
	return;
}

void Find_Chess_Board_Corner_Test_2()
{
#define Grid_Size_h 8
#define Grid_Size_W 11

	unsigned char Grid_Size[2] = { Grid_Size_W,Grid_Size_h };
	int iGrid_Size = Grid_Size[0] * Grid_Size[1];
	float Corner[Grid_Size_W * Grid_Size_h][2];
	char File[256];
	for (int i = 0; i <=40; i++)
	{
		//自己换目录
		sprintf(File, "mono_img\\%d.bmp", i);
		Image oImage;
		if (!bLoad_Image(File, &oImage))
			return;

		int iResult= bFind_Chess_Board_Corner(oImage, Grid_Size, Corner);
		//显示一下角点坐标
		//Disp((float*)Corner, iGrid_Size, 2, "Corner");

		printf("i:%d Result:%d\n", i,iResult);
		if (!iResult)
		{
			printf("Cannot find corners:%s\n",File);
			break;
		}   
		Free_Image(&oImage);
	}
	return;
}
//void Chess_Board_Test()
void main()
{
	Init_Env();
	//Draw_Chess_Board("c:\\tmp\\Chess_Board_1.bmp", 5, 5, 100);
	//Find_Chess_Board_Corner_Test_2();
	//Binarize_Test();
	//Dilate_Test();
	//adaptiveThreshold_Test();
	//Box_Filter_Test();
	//generateQuads_Test();
	//approxPolyDP_Test();
	//Find_Contour_Test_1();
	//Find_Contour_Test_2();
	Zhang_Test_1();
	Free_Env();
}