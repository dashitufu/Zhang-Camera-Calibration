//搞一个好使好用而且轻量的sift，连Mem_Mgr与Image都不用
#pragma once
extern "C"
{
#include "Buddy_System.h"
}

#define MAX_KEY_POINT_COUNT 10240
typedef struct Sift_Feature {	
	short x_Keypoint, y_Keypoint;	//原来在该分辨率下Keypoint的位置
	float angle;		//主方向，这个源代码没有，但是觉得有必要保留下来，对进一步分析有用
	float x, y;			//这个位置的计算尚有意义，感觉对不上
	short o;			//在哪一轮，从这一版开始，octave指的是图像分辨率轮次
	short s;			//在某一轮的哪一层。理论上是指梯度层？
	unsigned char m_Desc_i[128];
	//float a11, a12, a21, a22;	//这几个有异议
}Sift_Feature_Byte;

typedef struct Sift_Match_Item {	//图像对，这个结构虽然简单，但是不完备，只有配对的索引信息没有位置
	short m_iImage_A, m_iImage_B;
	unsigned short m_iMatch_Count;	//足够大了
	unsigned short(*m_Match)[2];	//两图对应的关键点
}Sift_Match_Item;

typedef struct Sift_Image {			//一张图像的Sift Feature
	Sift_Feature* m_pFeature;
	union {
		unsigned char(*m_pDesc)[128];	//为了加速最近邻匹配，这里紧凑存储一张图的128维特征
		unsigned char *m_pDesc_1;		//16个Sample一组
	};
	
	Sift_Match_Item* m_pMatch;		//本图与其他图的匹配
	char* m_pFile_Name;				//本图的文件名
	int m_iCount;					//一共与多少个特征点
}Sift_Image;

//看看能否搞一个更简版本描述匹配
typedef struct Sift_Simple_Match_Item {	// 这是一个完备的配对信息，直接给出对应位置，最简表达
	short m_iImage_A, m_iImage_B;
	unsigned short m_iMatch_Count;	//足够大了
	float(*m_pPoint_1)[2],			//改成这种形式更具普遍性
		(*m_pPoint_2)[2];
}Sift_Match_Item_1;

typedef struct Image_Info {
	char* m_pFile_Name;
}Image_Info;

typedef struct Sift_Match_Map {	//完全图结构
	Image_Info* m_pImage_Arr;
	Sift_Simple_Match_Item* m_pMatch;	//一共有 m_iImage_Count* m_iImage_Count个
	int m_iImage_Count;				//一共有多少张图
	unsigned char* m_pBuffer;		//此处为整个Match_Map所有的缓冲开始位置，分配释放在这搞即可
}Sift_Match;

//根据图像大小判断要用多少内存
int iGet_Sift_Detect_Size(int iWidth, int iHeight, int o_min=-1);
//计算一组图要用多少内存用于匹配
int iGet_Sift_Match_Size(Sift_Image Sift_Image_Arr[], int iImage_Count);
//取一张图像的特征点
void Get_Sift_Feature(float* pImage, int iWidth, int iHeight, int o_min, unsigned char* pBuffer, int iBuffer_Size, int* piFeature_Count);

//这里可以做一组高级Sift接口
//一个文件，找出所有的特征点
void Get_Sift_Feature(const char* pcFile, float(**ppFeature)[2], int* piCount, int o_min=-1);
//要特征的完整信息
void Get_Sift_Feature(const char* pcFile, Sift_Feature** ppFeature, int* piCount, int o_min = -1);
//两图比较，得匹配点
//void Sift_Match_2_Image(const char* pcFile_1, const char* pcFile_2, float(**ppPoint_1)[2], float(**ppPoint_2)[2], int* piCount, int o_min = -1);
template<typename _T>void Sift_Match_2_Image(const char* pcFile_1, const char* pcFile_2, _T(**ppPoint_1)[2], _T(**ppPoint_2)[2], int* piCount, int o_min=-1);
//遍历目录，两两匹配
void Sift_Match_Path(const char* pcPath, Sift_Match_Map* poMap, Mem_Mgr* poMem_Mgr, int o_min = -1);

//如果不想沾Mem_Mgr，以下接口更清爽
void Sift_Match_Path(const char* pcPath, Sift_Match_Map* poMap, int o_min = -1);
void Sift_Match_Path_1(const char* pcPath, Sift_Match_Map* poMap, Mem_Mgr* poMem_Mgr, int o_min = -1);

//对匹配点挑出有用，释放多余内存
template<typename _T>void Shrink_Match_Point(_T(**ppPoint_1)[2], _T(**ppPoint_2)[2], unsigned char Mask[], int iCount);
//临时实验
void Sift_Temp_Test_1();