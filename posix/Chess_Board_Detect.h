#pragma once
#include "image.h"
#include "Matrix.h"

typedef struct Get_Contour_Info {
	typedef struct Contour {
		unsigned int m_iArea : 31;					//相当于面积，共有多少个像素
		unsigned int m_iBlack_or_White : 1;			//该连通域什么颜色，白为1，黑为0
		union {
			struct {
				unsigned int m_iPart_Of : 24;		//属于哪个Contour
				unsigned int m_iReserve : 7;
			};
			struct {
				unsigned short m_iLine_Count;		//相当于Height
				unsigned short m_iY_Start;			//该Contour的开始屏幕y
				unsigned int m_iFirst_Line : 31;	//第二阶段画轮廓用，Top
			};
			struct {
				unsigned int m_iNew_ID : 24;		//新ID(位置）
				unsigned int m_bMark_Deleted : 8;	//用于第一阶段完成后的Remove 太小的contour
			};
		};
	}Contour;

	typedef struct Strip {	//一个扫描行中的连续线段
		//union {
		struct {
			unsigned short m_iStart;		//线段的开始位置
			unsigned short m_iEnd;			//线段的结束位置，为有效数据而非下一个可用位置
			unsigned int m_iContour_ID : 24;	//属于哪个连通域
			unsigned int m_iBlack_or_White : 1;		//该strip是黑还是白
		};
		unsigned short m_iY;			//第几行
		//};
	}Strip;

	typedef struct Line {
		unsigned int m_iFirst_Strip;		//一行当中第一个Strip的索引
		union {
			struct {
				unsigned short m_iCount : 15;
				unsigned short m_iBlack_or_White : 1;
			};
			unsigned short m_iStrip_Count;		//该行一共有多少个Strip
		};

	}Line;

	Strip* m_pStrip;
	Line* m_pLine;			//一共有iHeight条线
	Contour* m_pContour;

	unsigned short m_iWidth, m_iHeight;	//图像的长款
	int m_iMax_Strip_Count;		//一共可以有多少个Strip

	int m_iContour_Count;		//当前一共有多少个连通域
	int m_iStrip_Count;			//下一个可用Strip的ID
	int m_iMax_Domain_Count;		//最多可以容纳多少个Domain
}Get_Contour_Info;

typedef struct Get_Contour_Result
{
	typedef struct Contour {
		int m_iArea;		//面积，像素点数
		int m_iOutline_Point_Count : 31;
		int m_iBlack_or_White : 1;
		unsigned short (*m_pPoint)[2];
		int hierachy[4];
	}Contour;
	int m_iContour_Count;
	int m_iWidth, m_iHeight;
	int m_iMax_Point_Count;
	//int m_iNon_Root_Max_Area;	//非根节点最大面积

	Contour* m_pContour;
	unsigned short (*m_pAll_Point)[2];
}Get_Contour_Result;
#pragma pack()

int bGet_Contour(Image oImage, Get_Contour_Result* poResult);	//取得Contour的外形
int bGet_Contour(Image oImage, Get_Contour_Info* poInfo);		//最简形式，只求连通域
void Free_Contour_Result(Get_Contour_Result* poResult);
static void Draw_Chess_Board(const char* pcFile, int iWidth_In_Grad, int iHeight_In_Grad, int iGrid_Size, int x_Start = 100, int y_Start = 100);
int bFind_Chess_Board_Corner(Image oImage, float Corner[][2], int iGrid_Size_w=11, int iGrid_Size_h=8,  float Bounding_Box[][2] = NULL);
template<typename _T>void Gen_Corner_Ref(int w_In_Point, int h_In_Point, _T fGrid_Size, _T pCorner_3D[][2]);

//给懒人用，iStep_1_Image_Size：自己拍脑袋一个小图大小，剩下的自动算Scale
int bFind_Chess_Board_2_Step(Image oImage, float Corner[][2], float fScale, int iGrid_Size_w = 11, int iGrid_Size_h = 8);

//留给自以为是的人，能算出第一阶段的小图边长scale
int bFind_Chess_Board_2_Step(Image oImage, float Corner[][2], int iStep_1_Image_Size=-1, int iGrid_Size_w = 11, int iGrid_Size_h = 8);

//相当于入口
template<typename _T>void Load_Poine_2D(const char* pcFile, int* piImage_Count, int iCorner_Per_Image, int w_In_Point, int h_In_Point, _T(**ppCorner_Point_2D)[2]);
int Chess_Board_Detect_Main();