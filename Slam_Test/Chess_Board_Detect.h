#pragma once
#include "Image.h"
#pragma pack(1)
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

void Get_Contour(Image oImage, Get_Contour_Result* poResult);
void Free_Contour_Result(Get_Contour_Result* poResult);
static void Draw_Chess_Board(const char* pcFile, int iWidth_In_Grad, int iHeight_In_Grad, int iGrid_Size, int x_Start = 100, int y_Start = 100);

#pragma pack()