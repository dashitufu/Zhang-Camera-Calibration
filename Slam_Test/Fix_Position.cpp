#pragma once
#include <iostream>
#include "Common.h"
#include "Image.h"
#include "sift.h"
#include "Matrix.h"
#include "Reconstruct.h"

void Y_2_Gray(unsigned char* pSource, float* pDest, int iSize)
{
	unsigned char* pSource_Cur = pSource, * pSource_End = pSource_Cur + iSize;
	float* pDest_Cur = pDest;

	while (pSource_Cur < pSource_End)
		*pDest_Cur++ = *pSource_Cur++ / 255.f;
	return;
}
void Fix_Test_Main()
{
	//{//Test 1, 单张
	//	Image oImage_A, oImage_B, oBack;

	//	bLoad_Image("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A17.bmp", &oImage_A);
	//	bLoad_Image("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A18.bmp", &oImage_B);

	//	float(*pPoint_1)[2] = NULL, (*pPoint_2)[2];
	//	int i, iMatch_Count;

	//	Sift_Match_2_Image("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A17.bmp",
	//		"C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A18.bmp", &pPoint_1, &pPoint_2, &iMatch_Count);


	//	//for (int i = 0; i < iMatch_Count; i++)
	//		//printf("Point 1:%f %f Point 2:%f %f\n", pPoint_1[i][0], pPoint_1[i][1], pPoint_2[i][0], pPoint_2[i][1]);

	//	Ransac_Report oReport;
	//	Ransac_Estimate_E(pPoint_1, pPoint_2, iMatch_Count, 540.f, 240.f, 320.f, &oReport);

	//	//搞张大图放下两图
	//	Init_Image(&oBack, 1920, Max(oImage_A.m_iHeight, oImage_B.m_iHeight), Image::IMAGE_TYPE_BMP, 24);
	//	Set_Color(oBack);
	//	Place_Image(oImage_A, oBack, 0, 0);
	//	Place_Image(oImage_B, oBack, 960, 0);
	//	for (i = 0; i < iMatch_Count; i++)
	//	{
	//		if (oReport.m_pInlier_Mask[i])
	//		{
	//			float x1, y1, x2, y2;
	//			x1 = pPoint_1[i][0];
	//			y1 = pPoint_1[i][1];
	//			x2 = pPoint_2[i][0] + 960;
	//			y2 = pPoint_2[i][1];
	//			//Draw_Point(oBack, x1, y1);
	//			//Draw_Point(oBack, x2, y2);
	//			if (i % 5 == 0)
	//				Mid_Point_Line(oBack, x1, y1, x2, y2);
	//		}
	//	}
	//	bSave_Image("c:\\tmp\\1.bmp", oBack);
	//	Free_Image(&oImage_A);
	//	Free_Image(&oImage_B);
	//	free(pPoint_1);
	//}
	
	/*Image oImage_A, oImage_B, oBack;
	const int iWidth = 960, iHeight = 540;
	int i,iMatch_Count;
	char File_A[256], File_B[256];
	float(*pPoint_1)[2] = NULL, (*pPoint_2)[2];
	Init_Image(&oImage_A, iWidth, iHeight, Image::IMAGE_TYPE_BMP, 32);
	Init_Image(&oImage_B, iWidth, iHeight, Image::IMAGE_TYPE_BMP, 32);
	Init_Image(&oBack, iWidth, iHeight, Image::IMAGE_TYPE_BMP, 24);

	float fPlace_x;

	for (i = 0;; i++)
	{
		sprintf(File_A, "c:\\tmp\\temp_1\\%03d.bmp", i);
		sprintf(File_B, "c:\\tmp\\temp_1\\%03d.bmp", i + 1);
		bLoad_Image(File_A, &oImage_A,0,0,0,0);
		if (!bLoad_Image(File_B, &oImage_B,0,0,0,0))
			break;
		Sift_Match_2_Image(File_A, File_B, &pPoint_1, &pPoint_2, &iMatch_Count);

		Ransac_Report oReport;
		Ransac_Estimate_E(pPoint_1, pPoint_2, iMatch_Count, 960.f, 480.f, 270.f, &oReport);
		if (!oReport.m_bSuccess)
		{
			printf("error");
			break;
		}
		
		float x_A = 0.f, y_A = 0.f,
			x_B = 0.f, y_B = 0.f;
		int iCount = 0;
		for (int j = 0; j<iMatch_Count; j++)
		{
			if (oReport.m_pInlier_Mask[j])
			{
				x_A += pPoint_1[j][0];
				y_A += pPoint_1[j][1];
				x_B += pPoint_2[j][0];
				y_B += pPoint_2[j][1];
				iCount++;
			}
		}
		if (iCount != oReport.m_oSupport.m_iInlier_Count)
			printf("error");

		x_A /= oReport.m_oSupport.m_iInlier_Count;
		y_A /= oReport.m_oSupport.m_iInlier_Count;
		x_B /= oReport.m_oSupport.m_iInlier_Count;
		y_B /= oReport.m_oSupport.m_iInlier_Count;

		if (i == 0)
		{
			fPlace_x = (oImage_B.m_iWidth >> 1) - x_A;
			Place_Image(oImage_B, oBack,fPlace_x,0);
			sprintf(File_A, "c:\\tmp\\temp_2\\%03d.bmp", i);
			bSave_Image(File_A, oBack);
			Set_Color(oBack);
		}
		
		fPlace_x += x_A-x_B;
		sprintf(File_B, "c:\\tmp\\temp_2\\%03d.bmp", i+1);
		Place_Image(oImage_B, oBack, fPlace_x, 0);
		bSave_Image(File_B, oBack);

		printf("Offset x:%f Place_x:%f \n",x_B-x_A, fPlace_x);
		free(pPoint_1);
	}*/

	Image oImage;
	bLoad_Image("c:\\tmp\\1.bmp", &oImage);
	for (int i = 0; i < oImage.m_iWidth * oImage.m_iHeight; i++)
	{
		oImage.m_pChannel[0][i] *= 30;
	}
	bSave_Image("c:\\tmp\\2.bmp",oImage);

	float xy[3][2] = { {0.f,0.f},
		{20.f,1.f},
		{200.f,20.f} };
	float A[3 * 3] = { xy[0][0] * xy[0][0], xy[0][0],1.f,
		xy[1][0] * xy[1][0], xy[1][0],1.f,
		xy[2][0] * xy[2][0], xy[2][0],1.f };
	float B[3] = { 0,1,20 },X[3];
	int iResult;
	Solve_Linear_Gause(A, 3, B, X, &iResult);
	Disp(X, 1, 3);
	for (int i = 0; i < 200; i++)
	{
		printf("x:%f y:%f\n", (float)i, X[0] * i * i + X[1] * i + X[2]);
	}
	return;
}