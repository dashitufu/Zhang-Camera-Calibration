#pragma once
#include "stdio.h"
#include "memory.h"
#include "math.h"
#include "float.h"
#include <typeinfo>
#include <type_traits>

extern "C"
{
#include "Buddy_System.h"
}

#ifdef PI
#undef PI
#endif

#define PI 3.14159265358979323846

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
#define ALIGN_SIZE_16(iSize) ((( (unsigned long long)(iSize)+15)>>4)<<4) 
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

#define Attach_Light_Ptr(oPtr,pBuffer,iSize,iGPU_ID) oPtr = { (int)0,(int)(iSize),pBuffer,iGPU_ID }
#define MAX_FLOAT ((float)0xFFFFFFFFFFFFFFFF)	//仅仅给一个足够大的数字，并不是iEEE的浮点数最大值

#define Attach_Light_Ptr(oPtr,pBuffer,iSize,iGPU_ID) oPtr = { (int)0,(int)(iSize),pBuffer,iGPU_ID }

//Light Ptr在此分配
#define Malloc(oPtr, iSize,pBuffer) \
{ \
	(pBuffer)= (unsigned char*)(oPtr).m_pBuffer+(oPtr).m_iCur; \
	(oPtr).m_iCur +=ALIGN_SIZE_128((iSize)); \
	if ( (oPtr).m_iCur > (oPtr).m_iMax_Buffer_Size) \
	{ \
		(oPtr).m_iCur -= ALIGN_SIZE_128((iSize));\
		(pBuffer)=NULL; \
		printf("Fail to allocate memory in Malloc, Total:%d Remain:%d Need:%zu\n",(oPtr).m_iMax_Buffer_Size,(oPtr).m_iMax_Buffer_Size-(oPtr).m_iCur,(size_t)(iSize)); \
	} \
}

/************************网络常数***********************************/
#ifdef WIN32
	#define Close_Socket closesocket
#else
	#define Close_Socket close
#endif

//以下为常用命令，等同于管理
//首当其冲，文件操作
#define CMD_SHAKE_HAND			10001		//获得一个问及那
#define CMD_GET_FILE			10002		//获得一个问及那
#define CMD_UPLOAD_FILE			10003		//上传一个文件
#define CMD_DELETE_FILE			10004		//删除一个文件
#define CMD_GET_FILE_LIST		10005		//获得某个目录文件列表
#define CMD_MD					10006		//创建目录
#define CMD_RD					10007		//删除目录
#define CMD_CAPTURE				10008		//从相机取一帧数据
#define CMD_SET_CAM_FRAME_SIZE	10009		//设置相机分辨率
#define CMD_ROTATE_CAM			10010		//相机旋转
#define REPLY_RECV			0		//收到
/************************网络常数***********************************/

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

typedef struct Semaphore_For_Thread {
	short s;	//信号量
	//mutex m_oLock;		
	void* m_poLock;	//排斥量, 由操作系统提供
}Semaphore_For_Thread;

typedef struct Start_End {
	int m_iStart, m_iEnd;
}Start_End;

typedef enum Key {
	Arrow_Up,
	Arrow_Down,
	Arrow_Left,
	Arrow_Right,
	Esc,
	Invalid_Key
}Key;

typedef struct Mem_Mgr_Ex {
	Mem_Mgr oMem_Mgr = {};
	Semaphore_For_Thread m_oS;	//信号量
}Mem_Mgr_Ex;

template<typename _T>struct  Queue {
public:
	_T* m_pBuffer;
	int m_iHead;	//队头
	int m_iEnd;		//队尾
	int m_iCount;	//当前队列的元素个数
	int m_iBuffer_Size;	//当前队列可以装多少个元素，也是循环队列的模，一般来说，可以只装1/2的节点
};

template<typename _T>struct Pos_3 {
	union {
		_T m_Pos[3];
		struct {
			_T x, y, z;
		};
	};
};

template<typename _T> struct PCC_Point {
public:
	union {
		_T m_Pos[3];
		Pos_3<_T> m_oPos;
	};

	union {
		unsigned char m_Color[4];		//预留一个空白色，方便向量计算
		unsigned int m_iColor;
		struct {
			unsigned int m_bHas_Normal : 1;	//是否已经算完法向量
			unsigned int m_bAdd_To_Queue : 1;
			unsigned int m_iParent : 30;		//只在KD_树种有效
		};
		float m_fDistance;  //该顶点到面的距离
	};

	int m_iNext;					//在八叉树划分中，因为要为8组数据分组，此处用链表形式，此处放下一个在Point中的Index
	int m_iNearest_Point_Index;
};

typedef struct PCC_Face {
	int m_Vertex[3];	//三角形的3个顶点
}PCC_Face;

typedef struct BitPtr {
	unsigned char* m_pBuffer;
	int m_iCur;		//当前字节,位于m_oBuffer[]中的第m_iCur Byte.
	int m_iBitPtr;	//当前字节中的当前位
	int m_iEnd;		//m_Buffer的合法数据是有范围的, 合法数据的最后一个字节位于m_iEnd-1, 如果m_iCur>=m_iEnd则为越界
}BitPtr;

//尝试做一个可以增加增加删除点的结构
template<typename _T> struct Point_Cloud {
	_T(*m_pPoint)[3];				//记录点的位置(x,y,z)
	unsigned char (*m_pColor)[3];	//RGB
	unsigned char* m_pBuffer;		//内存所在
	unsigned char m_bHas_Color : 1;	//标志是否有颜色
	int m_iMax_Count;				//m_pBuffer最多容纳多少点
	int m_iCount;					//目前有多少点
};

typedef struct Line_1 {		//重做直线结构，全部用浮点表示
	enum Mode {
		Degree_Other = 0,
		Degree_90,
		Degree_180
	};
	float x0, y0, x1, y1;	//未必有值，因为直线也可以表示为y=kx+m
	float a, b, c;
	float k, m;		//y=kx+m表示
	float theta;	//倾角，当theta=PI/2时，y=kx+m无效,只能表示为x=m
	Mode m_iMode;	//=0时， y=kx+m =1时，x=m;
}Line_1;

/*********************************KD Tree**************************/
template<typename _T>struct KD_Point_3D {
public:
	union {
		Pos_3<_T> m_oPos;
		_T m_Pos[3];
		struct {
			_T x, y, z;
		};
	};
	int m_iOrg_Index;		//原来的位置
};

template<typename _T>struct  Neighbour_Item {
public:
	int m_iIndex;
	float m_fDistance;
};

template<typename _T, int K> struct Neighbour_K {
public:
	Neighbour_Item<_T> m_Buffer[K];
	typedef struct Part_2 {
		unsigned short m_iCount;
		float m_fWorst_Distance;
	}Part_2;
	union {
		struct {
			unsigned short m_iCount;
			float m_fWorst_Distance;
		};
		Part_2 m_oPart_2;
	};
};

template<typename _T>struct KD_Tree_Item {
public:
	struct {
		unsigned char m_bIs_Point;
		unsigned char x_Dim;		//最高256维
	};
	_T m_fDiv_Low, m_fDiv_High;	//栅栏，左边栅栏的值，右边栅栏的值
	struct {
		//注意，分两种情况，如果点为子树节点，为左右两子树节点的索引
		//如果为叶节点，则为点区间的左右索引
		unsigned int m_iLeft;
		unsigned int m_iRight;	//下一个有效节点，故此[iRight]无效
	};
};

template<typename _T>class KD_Tree_3D {					//简单KD树
public:
	KD_Point_3D<_T>* m_pPoint;

	KD_Tree_Item<_T>* m_pBuffer;
	KD_Tree_Item<_T>* m_pBuffer_GPU;
	int m_iRoot;
	int m_iNode_Count;
	int m_iPoint_Count;
	int m_Max_Size[3];

	KD_Tree_3D<_T>* m_poTree_GPU;	//整个结构放在GPU中
};
/*********************************KD Tree**************************/
extern Mem_Mgr_Ex oMem_Mgr;;

//超高频使用模板
template<typename _T>void Disp(_T* M, int iHeight, int iWidth, const char* pcCaption = NULL)
{
	int i, j;
	if (pcCaption)
		printf("%s\n", (char*)pcCaption);

	for (i = 0; i < iHeight; i++)
	{
		for (j = 0; j < iWidth; j++)
		{
			if (std::is_same<_T, float>::value)
			{
				if (M[i * iWidth + j] - (int)M[i * iWidth + j])
					//printf("%.10ef, ", (double)M[i * iWidth + j]);
					printf("%f, ", (double)M[i * iWidth + j]);
					//printf("%e, ", (double)M[i * iWidth + j]);
				else
					printf("%d, ", (int)M[i * iWidth + j]);
					//printf("%.4e, ", (double)M[i * iWidth + j]);
			}else if (std::is_same<_T, double>::value)
			{
				if (M[i * iWidth + j] - (int)M[i * iWidth + j])
					printf("%.6f, ", (double)M[i * iWidth + j]);
				//printf("%.10ef, ", (double)M[i * iWidth + j]);
				else
					printf("%d, ", (int)M[i * iWidth + j]);
				//printf("%f,", (double)M[i * iWidth + j]);
			}
			else if(  std::is_same<_T, unsigned int>::value ||
				std::is_same<_T, int>::value ||
				std::is_same<_T, short>::value ||
				std::is_same<_T, unsigned short>::value ||
				std::is_same<_T, unsigned char>::value 	)
				//printf("%d   ", (int)M[i * iWidth + j]);
				printf("%d,", (int)M[i * iWidth + j]);
		}
		printf("\n");
	}
	return;
}
template<typename _T>void Disp_Fillness(_T A[], int m, int n, const char Caption[]=NULL);

void Lock_Semaphore_For_Thread(Semaphore_For_Thread* ps, int iThreadID = 0);
void Unlock_Semaphore_For_Thread(Semaphore_For_Thread* ps, int iThreadID = 0);

Key iGet_Key();
int bGet_Line(FILE* pFile, char* pLine);
int iRead_Line(FILE* pFile, char Line[], int iLine_Size);
unsigned long long iGet_Tick_Count();
int bStricmp(char* pStr_0, char* pStr_1);
int bGet_Value(char* pText, int iSize, const char* pKey, char* pValue);

//上三角坐标转换为索引值
int iUpper_Triangle_Cord_2_Index(int x, int y, int w);
//上三角有效元数个数
int iGet_Upper_Triangle_Size(int w);

//一组有的没的随机数生成
int iRandom(int iStart, int iEnd);
int iRandom();
unsigned long long iGet_Random_No_cv(unsigned long long* piState);

void Init_BitPtr(BitPtr* poBitPtr, unsigned char* pBuffer, int iSize);
int iGetBits(BitPtr* poBitPtr, int iLen);

//解决两组基本数据类型的第n大与及快速排序，待优化
template<typename _T> _T oGet_Nth_Elem(_T Seq[], int iCount, int iNth);
template<typename _T> void Quick_Sort(_T Seq[], int iStart, int iEnd);

template<typename _T>void Init_Point_Cloud(Point_Cloud<_T>* poPC, int iMax_Count, int bHas_Color = 0);
template<typename _T>void Free_Point_Cloud(Point_Cloud<_T>* poPC);
template<typename _T>void Draw_Point(Point_Cloud<_T>* poPC, _T x, _T y, _T z, int R = 255, int G = 255, int B = 255);
template<typename _T>void Draw_Sphere(Point_Cloud<_T>* poPC, _T x, _T y, _T z, _T r = 1.f, int iStep_Count = 40, int R = 255, int G = 255, int B = 255);
template<typename _T>void Draw_Line(Point_Cloud<_T>* poPC, _T x0, _T y0, _T z0, _T x1, _T y1, _T z1, int iCount = 50, int R = 255, int G = 255, int B = 255);
template<typename _T>void Draw_Camera(Point_Cloud<_T>* poPC, _T T[4 * 4], int R = 255, int G = 255, int B = 255);
template<typename _T>void Draw_Rect(Point_Cloud<_T>* poPC, _T x, _T y, _T z, _T w, _T h, int iStep = 1, int R = 255, int G = 255, int B = 255);


//直线函数
void Cal_Line(Line_1* poLine, float x0, float y0, float x1, float y1);
float fGet_Line_x(Line_1* poLine, float y);
float fGet_Line_y(Line_1* poLine, float x);

template<typename _T>_T fAngle_2_Radian(_T fAngle);
template<typename _T>_T fRadian_2_Angle(_T fRadian);
template<typename _T> _T fGet_Distance(_T V_1[], _T V_2[], int n);

/************************************KD_Tree*************************************************************/
template<typename _T>void Get_Nearest_Point_Ref(KD_Tree_3D<_T>* poTree, _T Pos[3], _T Neighbour[3], float* pfDist = NULL, int* piIndex = NULL);
template<typename _T>void Get_Nearest_Point_Ref(KD_Point_3D<_T> Point[], int iPoint_Count, _T Pos[3], _T Neighbour[3], float* pfDist = NULL, int* piIndex = NULL, int* piNew_Index = NULL);
template<typename _T>void Get_Nearest_Point(KD_Tree_3D<_T>* poTree, _T Pos[3], _T Neighbour[3], float* pfDist = NULL, int* piIndex = NULL);
template<typename _T>void Free_KD_Tree(KD_Tree_3D<_T>* poTree);
template<typename _T>void Build_Tree(KD_Point_3D<_T> Point[], int iPoint_Count, KD_Tree_3D<_T>* poTree);
template<typename _T>void Build_Tree(_T Point[][3], int iPoint_Count, KD_Tree_3D<_T>* poTree);
template<typename _T, int K>static void _Get_Nearest_K_Point(KD_Tree_3D<_T>* poTree, int iNode, _T Pos[3], Neighbour_K<_T, K>* poNeighbour, float Dist[], float fMin_Distsq)
{//从iNode开始搜索起，一直找到足够的点
	KD_Tree_Item<_T> oNode = poTree->m_pBuffer[iNode];
	KD_Point_3D<_T>* pPoint = poTree->m_pPoint;
	float fWorst, fDistance;
	int i;

	if (oNode.m_bIs_Point)
	{//到达叶子节点，孩子是点
		fWorst = poNeighbour->m_fWorst_Distance;
		Neighbour_Item<_T>* pCur, * pBuffer = poNeighbour->m_Buffer;
		for (i = (int)oNode.m_iLeft; i < (int)oNode.m_iRight; ++i)
		{
			KD_Point_3D<_T> oPoint = pPoint[i];
			fDistance = (float)fGet_Distance(oPoint.m_Pos, Pos, 3);

			if (fDistance < fWorst)
			{	//add point
				if (poNeighbour->m_iCount < K)
					poNeighbour->m_iCount++;
				for (pCur = pBuffer + poNeighbour->m_iCount - 1; pCur > pBuffer; pCur--)
				{
					if (pCur[-1].m_fDistance > fDistance)
						*pCur = pCur[-1];
					else
						break;
				}
				pCur->m_fDistance = fDistance;
				pCur->m_iIndex = i;
				fWorst = pBuffer[K - 1].m_fDistance;
			}
		}
		poNeighbour->m_fWorst_Distance = fWorst;
		return;
	}

	//未到叶节点
	_T fValue = (_T)Pos[oNode.x_Dim];
	float cut_dist;
	int bestChild, otherChild;

	if (fValue <= oNode.m_fDiv_Low)	//窃以为这个判断更直观
	{
		bestChild = oNode.m_iLeft;
		otherChild = oNode.m_iRight;
		cut_dist = (float)(fValue - oNode.m_fDiv_High);
	}
	else
	{
		bestChild = oNode.m_iRight;
		otherChild = oNode.m_iLeft;
		cut_dist = (float)(fValue - oNode.m_fDiv_Low);
	}

	cut_dist *= cut_dist;
	_Get_Nearest_K_Point(poTree, bestChild, Pos, poNeighbour, Dist, fMin_Distsq);
	float dst = Dist[oNode.x_Dim];
	fMin_Distsq = fMin_Distsq + cut_dist - dst;
	Dist[oNode.x_Dim] = cut_dist;

	//经测试，以下判断确实最优,比单纯判断cut_dist<poNeighbour->m_fWorst_Distance 更优，但是原理尚未明
	if (fMin_Distsq /** 1.0*/ <= poNeighbour->m_fWorst_Distance)
		_Get_Nearest_K_Point(poTree, otherChild, Pos, poNeighbour, Dist, fMin_Distsq);
	Dist[oNode.x_Dim] = (float)dst;
	return;
}
template<typename _T, int K>static void Re_Init_Neighbour_K(Neighbour_K<_T, K>* poNeighbour)
{
	//*poNeighbour = {};
	poNeighbour->m_iCount = 0;
	poNeighbour->m_Buffer[K - 1].m_fDistance = poNeighbour->m_fWorst_Distance = MAX_FLOAT;
	return;
}
template<typename _T, int K>void Get_Nearest_K_Point(KD_Tree_3D<_T>* poTree, _T Pos[3], Neighbour_K<_T, K>* poNeighbour)
{
	Re_Init_Neighbour_K(poNeighbour);

	float Dist[K] = {};
	_Get_Nearest_K_Point(poTree, poTree->m_iRoot, Pos, poNeighbour, Dist, (_T)0);

	return;
}
/************************************KD_Tree*************************************************************/

/*******************************简单队列****************************/
template<typename _T>void Init_Queue(Queue<_T>* poQueue, int iBuffer_Size);
template<typename _T>void In_Queue(Queue<_T>* poQueue, _T oItem);
template<typename _T>_T Out_Queue(Queue<_T>* poQueue);
template<typename _T>void Free_Queue(Queue<_T>* poQueue);
#define Clear_Queue(oQueue) \
{\
	oQueue.m_iCount = oQueue.m_iEnd = oQueue.m_iHead = 0; \
}
/*******************************简单队列****************************/

/*******************************Mem_Mgr***********************************/
int bInit_Env_CPU(unsigned long long iSize = 1000000000, int iBlock_Size = 2048, int iMax_Piece_Count = 997);
void Free_Env();
void Free_Env_CPU();
void* pMalloc(unsigned int iSize);
void Shrink(void* p, unsigned int iSize);
int bExpand(void* p, unsigned int iSize);
void* pMove_2_Front(void* p);
void Free(void* p);
void Disp_Mem();
/*******************************Mem_Mgr***********************************/

/*****************************一组文件操作********************************/
template<typename _T>void Get_Random_Norm_Vec(_T V[], int n);
template<typename _T>int bRead_PLY_File(const char* pcFile, PCC_Point<_T>** ppPoint, int* piPoint_Count, PCC_Face** ppFace, int* piFace_Count); template
<typename _T>int bSave_PLY(const char* pcFile, _T Point[][3], int iPoint_Count, unsigned char Color[][3] = NULL, int bText = 1);
template<typename _T>void bSave_PLY(const char* pcFile, Point_Cloud<_T> oPC);
int iGet_File_Count(const char* pcPath);
unsigned long long iGet_File_Length(char* pcFile);
int iCreate_Dir(char Path[]);
int iDelete_Dir(char Path[]);
char* pGet_File_List(const char* pcPath, int* piFile_Count, int* piBuffer_Size=NULL, int bInclude_Dir=1);
void Disp_File_List(char* pBuffer, int iSize, int bInclude_Dir = 0);
int bSave_Bin(const char* pcFile, float* pData, int iSize);
int bSave_Raw_Data(const char* pcFile, void* pBuffer, int iSize);
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer, int* piSize = NULL);
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer, int iSize = 0, int bNeed_Malloc = 1, int iFrame_No = 0);
int bLoad_Text_File(const char* pcFile, char** ppBuffer, int* piSize = NULL);
/*****************************一组文件操作********************************/

/*****************************网络函数*************************************/
void Init_Socket_Env();
void Free_Socket_Env();
int iSendEx(int iSocket, void* buf, int iLen);
int iRecvEx(int iSocket, void* buf, int iLen);
int iInit_Connection(char IP[], int iPort);
void Close_Connection(int iSocket, int bDestroy = 1);
void Destroy_Conn(int iSocket);
void Listen(char IP[], int iPort, void* pCall_Back, int iTime_Out = 0);
//以下一些通用功能
int iSend_Buffer(int iSocket, unsigned char* pBuffer, int iSize);
int iRecv_Buffer(int iSocket, unsigned char** ppBuffer, int* piSize);
int iReply_Recv(int iSocket);
int iCmd_Shake_Hand_Client(int iSocket);
int iCmd_Shake_Hand_Server(int iSocket);
int iCmd_Get_File_Client(int iSocket, const char* pcRemove_File, const char* pcLocal_File);
int iCmd_Get_File_Server(int iSocket);
int iCmd_Upload_File_Client(int iSocket, const char Local_File[], const char Remote_File[]);
int iCmd_Upload_File_Server(int iSocket);
int iCmd_Delete_File_Client(int iSocket, const char File[]);
int iCmd_Delete_File_Server(int iSocket);
int iCmd_rd_Client(int iSocket, const char Path[]);
int iCmd_rd_Server(int iSocket);
int iCmd_md_Client(int iSocket, const char File[]);
int iCmd_md_Server(int iSocket);;
int iCmd_Get_File_List_Client(int iSocket, const char Path[], char** ppBuffer, int* piSize);
int iCmd_Get_File_List_Server(int iSocket);
int iCmd_Capture_Client(int iSocket, unsigned char** ppBuffer, int* piSize, unsigned long long* ptTime_Stamp = NULL);
int iCmd_Capture_Server(int iSocket);
int iCmd_Set_Cam_Frame_Size_Client(int iSocket, int w, int h);
int iCmd_Set_Cam_Frame_Size_Server(int iSocket);
int iCmd_Rotate_Cam_Server(int iSocket);
int iCmd_Rotate_Cam_Client(int iSocket, int iDir, float fDelta);
/*****************************网络函数*************************************/
