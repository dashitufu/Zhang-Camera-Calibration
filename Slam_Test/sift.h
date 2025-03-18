//��һ����ʹ���ö���������sift����Mem_Mgr��Image������
#pragma once
extern "C"
{
#include "Buddy_System.h"
}

#define MAX_KEY_POINT_COUNT 10240
typedef struct Sift_Feature {	
	short x_Keypoint, y_Keypoint;	//ԭ���ڸ÷ֱ�����Keypoint��λ��
	float angle;		//���������Դ����û�У����Ǿ����б�Ҫ�����������Խ�һ����������
	float x, y;			//���λ�õļ����������壬�о��Բ���
	short o;			//����һ�֣�����һ�濪ʼ��octaveָ����ͼ��ֱ����ִ�
	short s;			//��ĳһ�ֵ���һ�㡣��������ָ�ݶȲ㣿
	unsigned char m_Desc_i[128];
	//float a11, a12, a21, a22;	//�⼸��������
}Sift_Feature_Byte;

typedef struct Sift_Match_Item {	//ͼ��ԣ�����ṹ��Ȼ�򵥣����ǲ��걸��ֻ����Ե�������Ϣû��λ��
	short m_iImage_A, m_iImage_B;
	unsigned short m_iMatch_Count;	//�㹻����
	unsigned short(*m_Match)[2];	//��ͼ��Ӧ�Ĺؼ���
}Sift_Match_Item;

typedef struct Sift_Image {			//һ��ͼ���Sift Feature
	Sift_Feature* m_pFeature;
	union {
		unsigned char(*m_pDesc)[128];	//Ϊ�˼��������ƥ�䣬������մ洢һ��ͼ��128ά����
		unsigned char *m_pDesc_1;		//16��Sampleһ��
	};
	
	Sift_Match_Item* m_pMatch;		//��ͼ������ͼ��ƥ��
	char* m_pFile_Name;				//��ͼ���ļ���
	int m_iCount;					//һ������ٸ�������
}Sift_Image;

//�����ܷ��һ������汾����ƥ��
typedef struct Sift_Simple_Match_Item {	// ����һ���걸�������Ϣ��ֱ�Ӹ�����Ӧλ�ã������
	short m_iImage_A, m_iImage_B;
	unsigned short m_iMatch_Count;	//�㹻����
	float(*m_pPoint_1)[2],			//�ĳ�������ʽ�����ձ���
		(*m_pPoint_2)[2];
}Sift_Match_Item_1;

typedef struct Image_Info {
	char* m_pFile_Name;
}Image_Info;

typedef struct Sift_Match_Map {	//��ȫͼ�ṹ
	Image_Info* m_pImage_Arr;
	Sift_Simple_Match_Item* m_pMatch;	//һ���� m_iImage_Count* m_iImage_Count��
	int m_iImage_Count;				//һ���ж�����ͼ
	unsigned char* m_pBuffer;		//�˴�Ϊ����Match_Map���еĻ��忪ʼλ�ã������ͷ�����㼴��
}Sift_Match;

//����ͼ���С�ж�Ҫ�ö����ڴ�
int iGet_Sift_Detect_Size(int iWidth, int iHeight, int o_min=-1);
//����һ��ͼҪ�ö����ڴ�����ƥ��
int iGet_Sift_Match_Size(Sift_Image Sift_Image_Arr[], int iImage_Count);
//ȡһ��ͼ���������
void Get_Sift_Feature(float* pImage, int iWidth, int iHeight, int o_min, unsigned char* pBuffer, int iBuffer_Size, int* piFeature_Count);

//���������һ��߼�Sift�ӿ�
//һ���ļ����ҳ����е�������
void Get_Sift_Feature(const char* pcFile, float(**ppFeature)[2], int* piCount, int o_min=-1);
//Ҫ������������Ϣ
void Get_Sift_Feature(const char* pcFile, Sift_Feature** ppFeature, int* piCount, int o_min = -1);
//��ͼ�Ƚϣ���ƥ���
//void Sift_Match_2_Image(const char* pcFile_1, const char* pcFile_2, float(**ppPoint_1)[2], float(**ppPoint_2)[2], int* piCount, int o_min = -1);
template<typename _T>void Sift_Match_2_Image(const char* pcFile_1, const char* pcFile_2, _T(**ppPoint_1)[2], _T(**ppPoint_2)[2], int* piCount, int o_min=-1);
//����Ŀ¼������ƥ��
void Sift_Match_Path(const char* pcPath, Sift_Match_Map* poMap, Mem_Mgr* poMem_Mgr, int o_min = -1);

//�������մMem_Mgr�����½ӿڸ���ˬ
void Sift_Match_Path(const char* pcPath, Sift_Match_Map* poMap, int o_min = -1);
void Sift_Match_Path_1(const char* pcPath, Sift_Match_Map* poMap, Mem_Mgr* poMem_Mgr, int o_min = -1);

//��ƥ����������ã��ͷŶ����ڴ�
template<typename _T>void Shrink_Match_Point(_T(**ppPoint_1)[2], _T(**ppPoint_2)[2], unsigned char Mask[], int iCount);
//��ʱʵ��
void Sift_Temp_Test_1();