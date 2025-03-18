#pragma once

#ifndef Min
	#define Min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef Max
	#define Max(a,b) ((a)>(b)?(a):(b))
#endif

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
		printf("Fail to allocate memory in Malloc, Total:%d Remain:%d Need:%zu\n",(oPtr).m_iMax_Buffer_Size,(oPtr).m_iMax_Buffer_Size-(oPtr).m_iCur,(unsigned long long)(iSize)); \
	} \
}

//索引块子块的三种状态，0层的子块只用一位表示，所以此枚举结构没0层什么事
//1层以上每一个子块由两位组成，可以表示三种状态
typedef enum Occupancy_Type {
	Empty = 0,				//子块为空闲状态，可以分配
	Half_Full = 1,			//子块为半空闲状态，具体那些能用哪些不能用，得下探一层分析
	Full = 3				//子块已经被占用
}Occupancy_Type;

//在某一层推进的过程中，得到左中右三个区域的情况之后，可以分析出三种情况
typedef enum Free_Status {	//有三种情况
	Free_Status_Enough = 0,					//有足够的空间
	Free_Status_Need_To_Try = 1,			//需要进一步下探是否有足够的子块
	Free_Status_Not_Enough = 3				//不够空间
}Free_Status;

typedef struct Mem_Mgr_Layer {	//该结构为某一层的描述
	unsigned long long	m_iBytes_Per_Block;		//该层一个索引块表示多少字节的内存
	unsigned int		m_iBytes_Per_Sub_Block;	//该层一个索引子块表示多少字节内存
	unsigned int		m_iBlock_Count;			//本层共又多少索引块
												//但是对于0层以上，块不需要填满，子块可以位 01,即连子块都不填满
	unsigned int m_iSub_Block_Count;			//本层共有多少个子块
	unsigned int m_iCur_Layer : 3;				//当前层号（索引号）
	unsigned int m_iSub_Block_Per_Block;		//一个Block分为多少个Sub Block，0层是64，1层以上是32

	unsigned char* m_pIndex;					//Block的起始位置，64位对齐，对于0层，一个Sub_Block由一位构成，0：空，1：已占
												//对于0层以上的层，一个Sub_Block由两位构成：0:空； 11:已占； 01：半占
}Mem_Mgr_Layer;

//以下位散列表部分。一个内存池必须指定最多能分配多少项（Piece)。比如如果指定只能非陪1023项目，
	//那么当分配第1023项的时候将不予分配。这些已分配的项用一个散列表来组织
typedef struct Mem_Mgr_Hash_Item {							//散列表的每一项所在
	unsigned long long m_iPos : 24;					//第0层的哪一个Sub_Block，最多只能非陪1^24个0层索引子块
	unsigned long long m_iSub_Block_Count : 24;		//一共使用了第0层的多少个Sub_Block
	unsigned long long m_iNext : 16;				//链表
}Mem_Mgr_Hash_Item;

//注意：块的概念有两种，一种是用户观点的块，类似扇区的概念，一块可以自定义为多少字节，比如1024字节。以下命名为用户块
//另一种是程序内部索引用的块，以下称为索引块。索引块用64位无符号整数表示。一个索引块由若干个索引子块组成
//0层一个索引块由64个索引子块组成，一个索引子块用一位表示，0位空闲，1为占用
//1层以上索引块由32个索引子块组成，一个索引子块用两位表示
typedef struct Mem_Mgr {
	//最底层为0层，一块可分为64个子块。64位代表一块。每一块为m_iBytes_Per_Bottom_Sub_Block字节
	//上面层每层的块分为32子块
	Mem_Mgr_Layer m_Layer[8];	//最多容纳8层

	int m_iLayer_Count;					//整个索引由多少层组成
	int m_iBytes_Per_Bottom_Sub_Block;	//最底层的一位对应所分配的字节数
	unsigned char* m_pIndex_Buffer;		//所有层的索引数据
	unsigned long long m_iSize;			//整个Mem_Mgr共申请多少内存
	unsigned char* m_pBuffer;			//对齐的分配的内存
	unsigned char* m_pOrg_Buffer;		//系统分配的内存开始位置
	
	Mem_Mgr_Hash_Item *m_pHash_Item;
	int m_iHash_Size;	//最多可以分配多少片内存。为了缩小Index的占用，用散列表来组织
	int* m_pHash_Table;	//开散列表
	int m_iPiece_Count;	//一共分配了多少片，极限为m_iHash_Size
	int m_iCur_Item;	//m_pHash_Item当前项号，下一个可用位置

	//int m_iGPU_ID;		//在那个GPU上分配
}Mem_Mgr;

typedef struct Light_Ptr {	//顺便做一个轻量级的显存指针，用于顺序分配
	int m_iCur;					//不能超过m_iMax_Buffer_Size
	int m_iMax_Buffer_Size;
	unsigned char* m_pBuffer;	//内存指针
	int m_iGPU_ID;				//内存所在GPU
}Light_Ptr;

//取随机数
//unsigned int iGet_Random_No();

//分配内存
//Input： iSize: 要分配内存的字节数
//return: void 指针，用户自己转换为自己的类型
void* pMalloc(Mem_Mgr* poMem_Mgr, unsigned int iSize);

void* pMalloc_1(Mem_Mgr* poMem_Mgr, unsigned int iSize);

//释放一项
//Input:	p: 原来分配的指针
void Free(Mem_Mgr* poMem_Mgr, void* p);

//初始化内存池
//Input:	iSize：			整个内存池管理的内存大小，小于等于最后能分配的内存
//			iBlock_Size:	用户观点的用户块块大小
//			iMax_Piece_Count: 最多能分配多少片
//return:	若成功poMem_Mgr->m_pBuffer有数据，否则为NULL，以此判断是否成功即可
void Init_Mem_Mgr(Mem_Mgr* poMem_Mgr, unsigned long long iSize, int iBlock_Size, int iMax_Piece_Count);

//释放整个内存池
void Free_Mem_Mgr(Mem_Mgr* poMem_Mgr);

//将原来分配的大小缩小到iSize，原来分配大小必须大于等于iSize，否则啥也不干
//Input: iSize: 新的大小
void Shrink(Mem_Mgr* poMem_Mgr, void* p, unsigned int iSize);

//将原来p分配到的内存扩大到iSize，入宫iSize<=原来大小，则啥也不干
//Input:	p: 原来内存指针
//			iSize: 新的大小
int bExpand(Mem_Mgr* poMem_Mgr, void* p, unsigned int iSize);

//显示内存的使用情况，对所有的Item按position排个序
void Disp_Mem(Mem_Mgr* poMem_Mgr, int iLayer_Count);
