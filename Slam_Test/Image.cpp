//这版将淡化BitMap，面向Image，通用型结构
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "sys/timeb.h"
#include "Image.h"
#include "Common.h"
#include "immintrin.h"

extern "C"
{
#include "Buddy_System.h"
}

int bStricmp(char* pStr_0, char* pStr_1)
{//由于Linux与Windows不一致，被迫搞一个同一无大小写比较函数
#ifndef WIN32
#include "strings.h"
	return strcasecmp(pStr_0, pStr_1);
#else
	return _stricmp(pStr_0, pStr_1);
#endif
}
float fNaN()
{
	unsigned int iRet = 0x7FFFFFFF;
	return *(float*)(&iRet);
}
void Init_Image(Image* poImage, int iWidth, int iHeight, Image::Type iType, int iBit_Count)
{
	int i, iSize, iSize_With_Remain;;
	poImage->m_iWidth = iWidth;
	poImage->m_iHeight = iHeight;
	poImage->m_iBit_Count = iBit_Count;
	poImage->m_iChannel_Count = iBit_Count >> 3;
	poImage->m_iMem_Src = Mem_Src::CPU;
	poImage->m_bIs_Attached = 0;
	iSize = iWidth * iHeight;

	poImage->m_iMax_Buffer_Size = iSize * poImage->m_iChannel_Count;

	//留有余地，1，起码留64字节以上；2，128字节倍数，以便GPU对齐
	iSize_With_Remain = ((poImage->m_iMax_Buffer_Size + 127) / 128) * 128;
	if (iSize_With_Remain - poImage->m_iMax_Buffer_Size < 64)
		iSize_With_Remain += 128;

	poImage->m_pChannel[0] = (unsigned char*)malloc(iSize_With_Remain);
	if (!poImage->m_pChannel[0])
	{
		printf("Fail to malloc in Init_Image\n");
		*poImage = {};
		return;
	}

	for (i = 1; i < poImage->m_iChannel_Count; i++)
		poImage->m_pChannel[i] = poImage->m_pChannel[i - 1] + iSize;
	for (; i < 4; i++)
		poImage->m_pChannel[i] = NULL;

	poImage->m_iImage_Type = iType;
	poImage->m_pBuffer = poImage->m_oBitMap.m_pRGBA[0];
}

int bLoad_Image(const char* pcFile, BitMap_8Bit* poBitMap, int bNeed_Malloc,Mem_Mgr *poMem_Mgr)
{//装入BMP图，未经优化龟慢，仅供测试不供商用
	BitMap_8Bit::BITMAPFILEHEADER oFile_Header;
	BitMap_8Bit::BITMAPINFOHEADER oInfo_Header;
	unsigned char* pLine = NULL, * pCur, * RGBA[4], * RGBA_1[4];
	int i, j, k, iSize, iResult, iSize_Width_Remain, iRet = 1;
	int iCount;
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
	{
		printf("Fail to open:%s\n", pcFile);
		return 0;
	}
	iResult = (int)fread(&oFile_Header, 1, sizeof(BitMap_8Bit::BITMAPFILEHEADER), pFile);
	iResult = (int)fread(&oInfo_Header, 1, sizeof(BitMap_8Bit::BITMAPINFOHEADER), pFile);
	if (!iResult)
	{
		printf("Fail to read header in bLoad_Image\n");
		iRet = 0;
		goto END;
	}
	if (/*oInfo_Header.biBitCount!=1 &&*/ oInfo_Header.biBitCount != 32 && oInfo_Header.biBitCount != 24 && oInfo_Header.biBitCount != 8)
	{//暂时不管其他的位图
		printf("Invalid Bit-Depth in bitmap:%d\n", oInfo_Header.biBitCount);
		iRet = 0;
		goto END;
	}
	poBitMap->m_iBit_Count = oInfo_Header.biBitCount;
	poBitMap->m_iChannel_Count = poBitMap->m_iBit_Count / 8;
	poBitMap->m_iWidth = oInfo_Header.biWidth;
	poBitMap->m_iHeight = oInfo_Header.biHeight;
	poBitMap->m_iMem_Src = Mem_Src::CPU;
	if (bNeed_Malloc)
		poBitMap->m_bIs_Attached = 0;

	iSize = oInfo_Header.biWidth * oInfo_Header.biHeight;
	poBitMap->m_iMax_Buffer_Size = iSize * poBitMap->m_iChannel_Count;

	//留有余地，1，起码留64字节以上；2，128字节倍数，以便GPU对齐
	iSize_Width_Remain = ((poBitMap->m_iMax_Buffer_Size + 127) / 128) * 128;
	if (iSize_Width_Remain - poBitMap->m_iMax_Buffer_Size < 64)
		iSize_Width_Remain += 128;

	if (bNeed_Malloc)
	{
		if (poMem_Mgr)
			poBitMap->m_pRGBA[0] = (unsigned char*)pMalloc(poMem_Mgr, iSize_Width_Remain);
		else
			poBitMap->m_pRGBA[0] = (unsigned char*)malloc(iSize_Width_Remain);
	}

	for (i = 1; i < poBitMap->m_iChannel_Count; i++)
		poBitMap->m_pRGBA[i] = poBitMap->m_pRGBA[i - 1] + iSize;
	for (; i < 4; i++)
		poBitMap->m_pRGBA[i] = NULL;

	if(poBitMap->m_iBit_Count>=8)
		iSize = ((oInfo_Header.biWidth * poBitMap->m_iChannel_Count + 3) / 4) * 4;

	if(poMem_Mgr)
		pLine = (unsigned char*)pMalloc(poMem_Mgr,iSize + 4);
	else
		pLine = (unsigned char*)malloc(iSize + 4);
	if (!poBitMap->m_pRGBA[0] || !pLine)
	{
		printf("Fail to allocate memory in Init_Image\n");
		iRet = 0;
		goto END;
	}

	iCount = Min(3, poBitMap->m_iChannel_Count);
	for (i = 0; i < iCount; i++)//文件内是BGR排列
		RGBA_1[i] = poBitMap->m_pRGBA[iCount - i - 1] + poBitMap->m_iWidth * poBitMap->m_iHeight - oInfo_Header.biWidth;
	if (poBitMap->m_iChannel_Count == 4)	//带Alpha
		RGBA_1[3] = poBitMap->m_pRGBA[3] + poBitMap->m_iWidth * poBitMap->m_iHeight - oInfo_Header.biWidth;

	fseek(pFile, oFile_Header.bfOffBits, SEEK_SET);
	for (i = 0; i < oInfo_Header.biHeight; i++)
	{
		iResult = (int)fread(pLine, 1, iSize, pFile);
		pCur = pLine;
		for (k = 0; k < poBitMap->m_iChannel_Count; k++)
			RGBA[k] = RGBA_1[k];

		for (j = 0; j < oInfo_Header.biWidth; j++)
		{
			for (k = 0; k < poBitMap->m_iChannel_Count; k++)
			{
				*RGBA[k] = *pCur++;
				RGBA[k]++;
			}
		}
		for (k = 0; k < poBitMap->m_iChannel_Count; k++)
			RGBA_1[k] -= oInfo_Header.biWidth;
	}

END:
	fclose(pFile);
	if (pLine)
	{
		if (poMem_Mgr)
			Free(poMem_Mgr, pLine);
		else
			free(pLine);
	}
	if (!iRet)
		free(poBitMap->m_pRGBA[0]);

	return iRet;
}

void Init_Image(YUV_Image_420_8Bit* poYUV_Image, int iWidth, int iHeight)
{
	int iSize, iSize_With_Remain;
	int iChroma_Width, iChroma_Height;

	poYUV_Image->m_iMem_Src = Mem_Src::CPU;
	poYUV_Image->m_iWidth = iWidth;
	poYUV_Image->m_iHeight = iHeight;
	iChroma_Width = (iWidth + 1) >> 1;
	iChroma_Height = (iHeight + 1) >> 1;

	iSize = poYUV_Image->m_iWidth * poYUV_Image->m_iHeight;
	poYUV_Image->m_iMax_Buffer_Size = iSize + iChroma_Width * iChroma_Height * 2;

	//留有余地，1，起码留64字节以上；2，128字节倍数，以便GPU对齐
	iSize_With_Remain = ((poYUV_Image->m_iMax_Buffer_Size + 127) / 128) * 128;
	if (iSize_With_Remain - poYUV_Image->m_iMax_Buffer_Size < 64)
		iSize_With_Remain += 128;

	//此处还是不对路，必须要有创建真正内存空间的方法，不应纳入虚拟内存
	poYUV_Image->m_pYUV[0] = (unsigned char*)malloc(iSize_With_Remain);

	if (!poYUV_Image->m_pYUV[0])
	{
		printf("Fail to allocate memory in Init_Image\n");
		poYUV_Image->m_pYUV[1] = poYUV_Image->m_pYUV[2] = NULL;
		return;
	}
	poYUV_Image->m_pYUV[1] = poYUV_Image->m_pYUV[0] + iSize;
	poYUV_Image->m_pYUV[2] = poYUV_Image->m_pYUV[1] + iChroma_Width * iChroma_Height;
}

int bLoad_Image(const char* pcFile, YUV_Image_420_8Bit* poImage, int iWidth, int iHeight, int bNeed_Malloc, int iFrame)
{
	FILE* pFile = fopen(pcFile, "rb");
	int iResult, bRet = 1, iSize = iWidth * iHeight + ((iWidth + 1) >> 1) * ((iHeight + 1) >> 1) * 2;
	if (!pFile)
	{
		printf("Fail to open:%s\n", pcFile);
		return 0;
	}
	if (bNeed_Malloc)
		Init_Image(poImage, iWidth, iHeight);

	if (!poImage->m_pYUV[0])
	{
		bRet = 0;
		goto END;
	}

#ifdef WIN32
	_fseeki64(pFile, (unsigned long long)iSize * iFrame, SEEK_SET);
#else
	fseeko64(pFile, (unsigned long long)iSize * iFrame, SEEK_SET);
#endif

	iResult = (int)fread(poImage->m_pYUV[0], 1, iSize, pFile);
	if (iResult < iSize)
	{
		printf("Error in reading file:%s iSize:%d iResult:%d\n", pcFile, iSize, iResult);
		bRet = 0;
	}

END:
	if (pFile)
		fclose(pFile);
	return bRet;
}
void Get_Image_Info(const char* pcFile, Image* poImage)
{//获得BMP的图像特性
	if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"bmp") == 0)
	{
		BitMap_8Bit::BITMAPFILEHEADER oFile_Header;
		BitMap_8Bit::BITMAPINFOHEADER oInfo_Header;
		//unsigned char* pLine = NULL, * pCur, *RGBA[4], * RGBA_1[4];
		int iResult, iSize;
		FILE* pFile = fopen(pcFile, "rb");
		if (!pFile)
		{
			printf("Fail to open:%s\n", pcFile);
			return;
		}
		iResult = (int)fread(&oFile_Header, 1, sizeof(BitMap_8Bit::BITMAPFILEHEADER), pFile);
		iResult = (int)fread(&oInfo_Header, 1, sizeof(BitMap_8Bit::BITMAPINFOHEADER), pFile);
		fclose(pFile);
		if (!iResult)
		{
			printf("Fail to read header in bLoad_Image\n");			
			goto END;
		}
		if (oInfo_Header.biBitCount != 32 && oInfo_Header.biBitCount != 24 && oInfo_Header.biBitCount != 8)
		{//暂时不管其他的位图
			printf("Invalid Bit-Depth in bitmap:%d\n", oInfo_Header.biBitCount);
			goto END;
		}
		poImage->m_iBit_Count = oInfo_Header.biBitCount;
		poImage->m_iChannel_Count = poImage->m_iBit_Count / 8;
		poImage->m_iWidth = oInfo_Header.biWidth;
		poImage->m_iHeight = oInfo_Header.biHeight;
		poImage->m_iMem_Src = Mem_Src::CPU;
		
		iSize = oInfo_Header.biWidth * oInfo_Header.biHeight;
		poImage->m_iMax_Buffer_Size = iSize * poImage->m_iChannel_Count;
		poImage->m_iImage_Type = Image::IMAGE_TYPE_BMP;
	}else
	{
		printf("Fail to open file:%s\n", pcFile);
		return;
	}
END:

	return;
}
int bLoad_Image(const char* pcFile, Image* poImage, int iWidth, int iHeight, int iFrame, int bNeed_Malloc, Mem_Mgr* poMem_Mgr)
{//直接装入Image,不管什么类型
	int iResult;
	YUV_Image_420_8Bit oImage;
	if(bNeed_Malloc)
		*poImage = { };

	if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"bmp") == 0)
	{
		iResult = bLoad_Image(pcFile, &poImage->m_oBitMap, bNeed_Malloc, poMem_Mgr);
		poImage->m_iImage_Type = Image::IMAGE_TYPE_BMP;
		return iResult;
	}
	else if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"yuv") == 0)
	{
		iResult = bLoad_Image(pcFile, &oImage, iWidth, iHeight, bNeed_Malloc, iFrame);
		poImage->m_iImage_Type = Image::IMAGE_TYPE_YUV_444;
	}
	else
		iResult = 0;
	if (!iResult || poImage->m_iImage_Type == Image::IMAGE_TYPE_BMP)
		return iResult;

	//YUV还要进一步处理
	if(bNeed_Malloc)
		Init_Image(poImage, iWidth, iHeight, Image::IMAGE_TYPE_YUV_444, 24);

	if (!poImage->m_oYUV.m_pYUV[0])
	{
		iResult = 0;
		goto END;
	}
	

END:
	return iResult;
}

void Set_Color(Image oImage, int R, int G, int B, int A)
{
	int iSize = oImage.m_iWidth * oImage.m_iHeight;
	int Color[3];
	if (oImage.m_iImage_Type == Image::IMAGE_TYPE_BMP)
		Color[0] = R, Color[1] = G, Color[2] = B;
	else
		_RGB_2_YUV(R, G, B, Color[0], Color[1], Color[2]);

	if (oImage.m_pChannel[0])
		memset(oImage.m_pChannel[0], Color[0], iSize);
	if (oImage.m_pChannel[1])
		memset(oImage.m_pChannel[1], Color[1], iSize);
	if (oImage.m_pChannel[2])
		memset(oImage.m_pChannel[2], Color[2], iSize);
	
	if (oImage.m_pChannel[3])
		memset(oImage.m_pChannel[3], A, iSize);

	return;
}

void Set_BitMap_Header(BitMap_8Bit::BITMAPFILEHEADER* poFile_Header, BitMap_8Bit::BITMAPINFOHEADER* poInfo_Header, int iWidth, int iHeight, int iChannel_Count)
{//设置BMP头。这个头根本没用，只在文件级起着结构说明作用，运行时完全浪费空间
	//int iSize;
	BitMap_8Bit::BITMAPFILEHEADER oFile_Header;
	BitMap_8Bit::BITMAPINFOHEADER oInfo_Header;

	oFile_Header.bfReserved1 = oFile_Header.bfReserved2 = 0;
	oFile_Header.bfType = 19778;	//BM
	oFile_Header.bfOffBits = 54;

	oInfo_Header.biPlanes = 1;
	oInfo_Header.biBitCount = iChannel_Count * 8;
	oInfo_Header.biClrImportant = 0;
	oInfo_Header.biClrUsed = 0;
	oInfo_Header.biCompression = 0;
	oInfo_Header.biHeight = iHeight;
	oInfo_Header.biWidth = iWidth;
	oInfo_Header.biSize = 40;
	oInfo_Header.biYPelsPerMeter = oInfo_Header.biXPelsPerMeter = 0;
	oInfo_Header.biSizeImage = oInfo_Header.biWidth * oInfo_Header.biHeight * oInfo_Header.biBitCount / 8;
	oFile_Header.bfSize = oInfo_Header.biSizeImage + oFile_Header.bfOffBits;
	//iSize = oInfo_Header.biWidth * oInfo_Header.biHeight;
	if (iChannel_Count == 1)
	{
		oFile_Header.bfOffBits = sizeof(BitMap_8Bit::BITMAPINFOHEADER) + sizeof(BitMap_8Bit::BITMAPFILEHEADER) + 1024;	//此处要加上调色板
		oFile_Header.bfSize = oInfo_Header.biSizeImage + oFile_Header.bfOffBits;
	}

	*poFile_Header = oFile_Header;
	*poInfo_Header = oInfo_Header;
}

int bSave_Image(const char* pcFile, BitMap_8Bit oBitMap)
{//存盘，希望能大一统，连1通道的都统一在这个函数内，未完成，尚欠1通道的情况
	FILE* pFile = fopen(pcFile, "wb");
	int iSize, i, j, k, iResult, iRet = 1;
	unsigned char* pLine = NULL, * pCur;
	unsigned char* RGBA[4], * RGBA_1[4];
	unsigned char Palatte[1024];

	BitMap_8Bit::BITMAPFILEHEADER oFile_Header;
	BitMap_8Bit::BITMAPINFOHEADER oInfo_Header;

	if (!pFile)
		return 0;
	Set_BitMap_Header(&oFile_Header, &oInfo_Header, oBitMap.m_iWidth, oBitMap.m_iHeight, oBitMap.m_iChannel_Count);

	iSize = oInfo_Header.biWidth * oInfo_Header.biHeight;
	if (!iSize)
		return 0;

	if (oBitMap.m_iBit_Count < 32)
	{
		for (i = 0; i < oBitMap.m_iChannel_Count; i++)
			RGBA_1[i] = oBitMap.m_pRGBA[oBitMap.m_iChannel_Count - i - 1] + iSize - oInfo_Header.biWidth;
	}
	else
	{
		for (i = 0; i < 3; i++)
			RGBA_1[i] = oBitMap.m_pRGBA[3 - i - 1] + iSize - oInfo_Header.biWidth;
		RGBA_1[3] = oBitMap.m_pRGBA[3] + iSize - oInfo_Header.biWidth;
	}


	iSize = ((oBitMap.m_iWidth * oBitMap.m_iChannel_Count + 3) / 4) * 4;
	pLine = (unsigned char*)malloc(iSize);
	memset(pLine + iSize - 4, 0, 4);

	iResult = (int)fwrite(&oFile_Header, (size_t)1, (size_t)sizeof(oFile_Header), pFile);
	iResult = (int)fwrite(&oInfo_Header, (size_t)1, (size_t)sizeof(oInfo_Header), pFile);
	if (oBitMap.m_iChannel_Count == 1)
	{
		//调色板
		for (pCur = Palatte, i = 0; i < 256; i++)
		{
			pCur[0] = pCur[1] = pCur[2] = i; pCur[3] = 0;
			pCur += 4;
		}
		iResult = (int)fwrite(Palatte, 1, 1024, pFile);
	}

	for (i = 0; i < oInfo_Header.biHeight; i++)
	{
		pCur = pLine;
		for (k = 0; k < oBitMap.m_iChannel_Count; k++)
			RGBA[k] = RGBA_1[k];

		for (j = 0; j < oInfo_Header.biWidth; j++)
		{
			for (k = 0; k < oBitMap.m_iChannel_Count; k++)
			{
				*pCur++ = *RGBA[k];
				RGBA[k]++;
			}
		}
		iResult = (int)fwrite(pLine, (size_t)1, (size_t)iSize, pFile);
		if (iResult != iSize)
		{
			iRet = 0;
			goto END;
		}
		for (k = 0; k < oBitMap.m_iChannel_Count; k++)
			RGBA_1[k] -= oInfo_Header.biWidth;
	}

END:
	if (pLine)
		free(pLine);
	if (pFile)
		fclose(pFile);
	return iRet;
}
void Init_Image(BitMap_8Bit* poBitMap, int iWidth, int iHeight, int iBit_Count)
{//初始化BMP
	int i, iSize, iSize_With_Remain;;
	poBitMap->m_iWidth = iWidth;
	poBitMap->m_iHeight = iHeight;
	poBitMap->m_iBit_Count = iBit_Count;
	poBitMap->m_iChannel_Count = iBit_Count >> 3;
	poBitMap->m_iMem_Src = Mem_Src::CPU;
	poBitMap->m_bIs_Attached = 0;
	iSize = iWidth * iHeight;

	poBitMap->m_iMax_Buffer_Size = iSize * poBitMap->m_iChannel_Count;

	//留有余地，1，起码留64字节以上；2，128字节倍数，以便GPU对齐
	iSize_With_Remain = ((poBitMap->m_iMax_Buffer_Size + 127) / 128) * 128;
	if (iSize_With_Remain - poBitMap->m_iMax_Buffer_Size < 64)
		iSize_With_Remain += 128;

	poBitMap->m_pRGBA[0] = (unsigned char*)malloc(iSize_With_Remain);
	if (!poBitMap->m_pRGBA[0])
	{
		printf("Fail to malloc in Init_Image\n");
		*poBitMap = { };
		return;
	}

	for (i = 1; i < poBitMap->m_iChannel_Count; i++)
		poBitMap->m_pRGBA[i] = poBitMap->m_pRGBA[i - 1] + iSize;
	for (; i < 4; i++)
		poBitMap->m_pRGBA[i] = NULL;

	return;
}
void Free_Image(Mem_Mgr* poMem_Mgr, Image oImage)
{
	Free(poMem_Mgr, oImage.m_pChannel[0]);
	return;
}
void Free_Image(Image* poImage)
{
	if (poImage && poImage->m_pChannel[0])
	{
		if (poImage->m_iMem_Src == Mem_Src::CPU)
		{
			free(poImage->m_pChannel[0]);
			*poImage = {};	//不知哪个标准的c，要求这种写法才能消除Warning
		}else
			printf("Buffer of this image is in GPU\n");
	}else
		printf("Ptr is NULL\n");
}
void Free_Image(BitMap_8Bit* poBitMap)
{//释放BMP图
	if (poBitMap && poBitMap->m_pRGBA[0])
	{
		if (poBitMap->m_iMem_Src == Mem_Src::CPU)
		{
			free(poBitMap->m_pRGBA[0]);
			*poBitMap = {  };	//不知哪个标准的c，要求这种写法才能消除Warning
		}else
			printf("Buffer of this image is in GPU\n");
	}else
		printf("Ptr is NULL\n");
}
void RGB_2_YUV(BitMap_8Bit oBMP, YUV_Image_420_8Bit oYUV)
{//龟慢转换，作为基准程序
	unsigned char* pR, * pG, * pB, * pY, * pU, * pV;
	int R[4], G[4], B[4], Y[4], U[4], V[4];
	int y, x, iSource_Height, iSource_Width, iSource_Width_x2, iDest_Width, iDest_Width_x2, iDest_Height, iDest_Width_Half;
	pR = oBMP.m_pRGBA[0], pG = oBMP.m_pRGBA[1], pB = oBMP.m_pRGBA[2];
	pY = oYUV.m_pYUV[0], pU = oYUV.m_pYUV[1], pV = oYUV.m_pYUV[2];
	iDest_Width = oYUV.m_iWidth;
	iDest_Width_x2 = iDest_Width << 1;
	iDest_Width_Half = (iDest_Width + 1) >> 1;
	iDest_Height = oYUV.m_iHeight;
	iSource_Width = oBMP.m_iWidth;
	iSource_Height = oBMP.m_iHeight;
	iSource_Width_x2 = iSource_Width << 1;

	for (y = 0; y < iDest_Height; y += 2, pR += iSource_Width_x2, pG += iSource_Width_x2, pB += iSource_Width_x2, pY += iDest_Width_x2, pU += iDest_Width_Half, pV += iDest_Width_Half)
	{
		for (x = 0; x < iDest_Width; x += 2)
		{
			if (x < iSource_Width - 1)
			{
				R[0] = pR[x], R[1] = pR[x + 1];
				G[0] = pG[x], G[1] = pG[x + 1];
				B[0] = pB[x], B[1] = pB[x + 1];
				if (y < iSource_Height - 1)
				{
					R[2] = pR[x + iSource_Width], R[3] = pR[x + iSource_Width + 1];
					G[2] = pG[x + iSource_Width], G[3] = pG[x + iSource_Width + 1];
					B[2] = pB[x + iSource_Width], B[3] = pB[x + iSource_Width + 1];
				}
				else
				{
					R[2] = pR[x], R[3] = pR[x + 1];
					G[2] = pG[x], G[3] = pG[x + 1];
					B[2] = pB[x], B[3] = pB[x + 1];
				}
			}
			else
			{
				R[0] = pR[x]; R[1] = pR[x];
				G[0] = pG[x]; G[1] = pG[x];
				B[0] = pB[x]; B[1] = pB[x];
				if (y < iSource_Height - 1)
				{
					R[2] = pR[x + iSource_Width], R[3] = pR[x + iSource_Width];
					G[2] = pG[x + iSource_Width], G[3] = pG[x + iSource_Width];
					B[2] = pB[x + iSource_Width], B[3] = pB[x + iSource_Width];
				}
				else
				{
					R[2] = pR[x], R[3] = pR[x];
					G[2] = pG[x], G[3] = pG[x];
					B[2] = pB[x], B[3] = pB[x];
				}

			}
			_RGB_2_YUV(R[0], G[0], B[0], Y[0], U[0], V[0]);
			_RGB_2_YUV(R[1], G[1], B[1], Y[1], U[1], V[1]);
			_RGB_2_YUV(R[2], G[2], B[2], Y[2], U[2], V[2]);
			_RGB_2_YUV(R[3], G[3], B[3], Y[3], U[3], V[3]);

			if (x >= iDest_Width - 1)
			{	//到边界了，奇数宽度，不写出去了
			}
			else
			{
				pY[x + 1] = Y[1];
				if (y < iDest_Height - 1)
					pY[x + iDest_Width + 1] = Y[3];
			}

			pY[x] = Y[0];
			if (y < iDest_Height - 1)
				pY[x + iDest_Width] = Y[2];

			U[0] = (U[0] + U[1] + U[2] + U[3] + 2) >> 2;
			pU[x >> 1] = Clip(U[0]);
			V[0] = (V[0] + V[1] + V[2] + V[3] + 2) >> 2;
			pV[x >> 1] = Clip(V[0]);
		}
	}
	return;
}
void YUV_2_RGB(YUV_Image_444_8Bit oYUV, BitMap_8Bit oBMP)
{
	int iSize = oBMP.m_iWidth * oBMP.m_iHeight;
	unsigned char* pR, * pG, * pB, * pY, * pU, * pV;
	int i, Y, U, V;
	pR = oBMP.m_pRGBA[0], pG = oBMP.m_pRGBA[1], pB = oBMP.m_pRGBA[2];
	pY = oYUV.m_pYUV[0], pU = oYUV.m_pYUV[1], pV = oYUV.m_pYUV[2];
	
	for (i = 0; i < iSize; i++)
	{
		U = *pU;
		V = *pV;
		Y = *pY;
		_YUV_2_RGB(Y, U, V, *pR, *pG, *pB);
		pR++, pG++, pB++, pY++, pU++, pV++;
	}	
	return;
}
int bSave_Image(const char* pcFile, YUV_Image_420_8Bit oImage, int bSave_Append)
{
	int iResult, bRet = 1, iSize = oImage.m_iWidth * oImage.m_iHeight + ((oImage.m_iWidth + 1) >> 1) * ((oImage.m_iHeight + 1) >> 1) * 2;
	FILE* pFile;
	if (bSave_Append)
		pFile = fopen(pcFile, "ab");
	else
		pFile = fopen(pcFile, "wb");

	if (!pFile)
	{
		printf("Fail to save image\n");
		return 0;
	}
	iResult = (int)fwrite(oImage.m_pYUV[0], 1, iSize, pFile);
	if (iResult != iSize)
	{
		printf("Fail to save image\n");
		bRet = 0;
	}

	fclose(pFile);
	return bRet;
}
int bSave_Image(const char* pcFile, Image oImage, int iFormat)
{//存个盘，根据类型来存
	int bRet=1;
	int iTo_Save;

	if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"bmp") == 0)
		iTo_Save = Image::IMAGE_TYPE_BMP;
	else if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"yuv") == 0)
		iTo_Save = Image::IMAGE_TYPE_YUV_444;

	if (iFormat == -1)
	{//保持原格式
		if (iTo_Save == Image::IMAGE_TYPE_BMP)
		{
			if (oImage.m_iImage_Type == Image::IMAGE_TYPE_BMP || oImage.m_iChannel_Count == 1)
				return bSave_Image(pcFile, oImage.m_oBitMap);
			else
			{
				BitMap_8Bit oNew;
				Init_Image(&oNew, oImage.m_iWidth, oImage.m_iHeight, oImage.m_iBit_Count);
				YUV_2_RGB(oImage.m_oYUV, oNew);
				bRet = bSave_Image(pcFile, oNew);
				Free_Image(&oNew);
				return bRet;
			}
		}
		else
		{
			/*if (oImage.m_iImage_Type == Image::IMAGE_TYPE_YUV_444)
				return bSave_Image_GPU(pcFile, &oImage.m_oYUV);
			else*/
			{
				YUV_Image_420_8Bit oNew;
				Init_Image(&oNew, oImage.m_iWidth, oImage.m_iHeight);
				RGB_2_YUV(oImage.m_oBitMap, oNew);
				return bSave_Image(pcFile, oNew,0);
			}
		}
	}
	else if (iFormat == (int)Image_Format::RGB)
	{
		if (oImage.m_iImage_Type == Image::IMAGE_TYPE_BMP)
			return bSave_Image(pcFile, oImage.m_oBitMap);

		BitMap_8Bit oNew;
		Init_Image(&oNew, oImage.m_iWidth, oImage.m_iHeight, oImage.m_iBit_Count);
		YUV_2_RGB(oImage.m_oYUV, oNew);
		bRet = bSave_Image(pcFile, oNew);
		Free_Image(&oNew);
		return bRet;
	}
	/*else if (iFormat == (int)Image_Format::YUV_444 || iFormat == (int)Image_Format::YUVA_444)
	{
		if (oImage.m_iImage_Type == Image::IMAGE_TYPE_YUV_444)
			return bSave_Image_GPU(pcFile, &oImage.m_oYUV);

		printf("No implemented yet\n");
		return 0;
	}*/
	return 0;
}
int bSave_Image(const char* pcFile, float* pImage, int iWidth, int iHeight)
{//将浮点数灰度值存成图
	Image oImage;
	Init_Image(&oImage, iWidth, iHeight, Image::IMAGE_TYPE_BMP, 8);
	Float_2_Image(pImage, oImage);
	return bSave_Image(pcFile, oImage);
}
void Mid_Point_Line(Image oImage, int x0, int y0, int x1, int y1, int R, int G, int B)
{
	int d, d_upper, d_lower;
	int a, b, x, y, iPos;
	int iTemp, iWidth = oImage.m_iWidth, iHeight = oImage.m_iHeight;
	unsigned char* pR = oImage.m_pChannel[0],
		* pG = oImage.m_pChannel[1], * pB = oImage.m_pChannel[2];

	if (abs(y1 - y0) <= abs(x1 - x0))
	{//45度以内
		if (x0 > x1)
		{//若x非由小到大交换头尾
			iTemp = x0, x0 = x1, x1 = iTemp;
			iTemp = y0, y0 = y1; y1 = iTemp;
		}
		a = y0 - y1;
		b = x1 - x0;	//本算法不需要c，在推导过程中约去

		if (y1 >= y0)
		{
			d = a * 2 + b;//这个的确要这样算，从循环那才看出来。它不是为初点做判断，而是为第二点判断
			d_upper = (a + b) * 2;
		}
		else
		{
			d = 2 * a - b;
			d_upper = (a - b) * 2;
		}
		d_lower = a * 2;

		//画第0点
		if (y0 >= 0 && y0 < iHeight && x0 >= 0 && x0 < iWidth)
		{
			iPos = y0 * iWidth + x0;
			pR[iPos] = R;
			if (oImage.m_pChannel[1])
				pG[iPos] = G;
			if (oImage.m_pChannel[2])
				pB[iPos] = B;
		}
		if (y1 >= y0)
		{//0-45度
			for (x = x0, y = y0; x < x1;)
			{
				if (d > 0)//M点在直线上方，取下面点
					d += d_lower;	//y不变，和上一步一样
				else
				{
					d += d_upper;
					y++;
				}
				x++;
				if (y >= 0 && y < iHeight && x >= 0 && x < iWidth)
				{
					iPos = y * iWidth + x;
					pR[iPos] = R;
					if (oImage.m_pChannel[1])
						pG[iPos] = G;
					if (oImage.m_pChannel[2])
						pB[iPos] = B;
				}
			}
		}
		else
		{//-45度到0
			for (x = x0, y = y0; x < x1;)
			{
				if (d < 0)//M点在直线上方，取下面点
					d += d_lower;	//y不变，和上一步一样
				else
				{
					d += d_upper;
					y--;
				}
				x++;
				if (y >= 0 && y < iHeight && x >= 0 && x < iWidth)
				{
					iPos = y * iWidth + x;
					pR[iPos] = R;
					if (oImage.m_pChannel[1])
						pG[iPos] = G;
					if(oImage.m_pChannel[2])
						pB[iPos] = B;
				}
			}
		}
	}
	else
	{//以y为自变量，不断加一画线 大于45度
		if (y0 > y1)
		{//若x非由小到大交换头尾
			iTemp = x0, x0 = x1, x1 = iTemp;
			iTemp = y0, y0 = y1; y1 = iTemp;
		}
		a = x0 - x1;
		b = y1 - y0;	//本算法不需要c，在推导过程中约去
		if (x1 >= x0)
		{
			d = 2 * b + a;	//这个的确要这样算，从循环那才看出来。它不是为初点做判断，而是为第二点判断
			d_upper = (a + b) * 2;
		}
		else
		{
			d = 2 * a - b;
			d_upper = (a - b) * 2;
		}
		d_lower = a * 2;
		//画第0点
		if (y0 >= 0 && y0 < iHeight && x0 >= 0 && x0 < iWidth)
		{
			iPos = y0 * iWidth + x0;
			pR[iPos] = R;
			if (oImage.m_pChannel[1])
				pG[iPos] = G;
			if (oImage.m_pChannel[2])
				pB[iPos] = B;
		}

		if (x1 >= x0)
		{//45度-90度
			for (x = x0, y = y0; y <= y1;)
			{
				if (d > 0)//M点在直线上方，取下面点
					d += d_lower;	//y不变，和上一步一样
				else
				{
					d += d_upper;
					x++;
				}
				y++;
				if (x < 0 || y < 0)
					continue;
				else if (y < iHeight && x < iWidth)
				{
					iPos = y * iWidth + x;
					pR[iPos] = R;
					if (oImage.m_pChannel[1])
						pG[iPos] = G;
					if (oImage.m_pChannel[2])
						pB[iPos] = B;
				}
				else
					break;
			}
		}
		else
		{//90度到135度
			for (x = x0, y = y0; y < y1;)
			{
				if (d < 0)//M点在直线上方，取下面点
					d += d_lower;	//y不变，和上一步一样
				else
				{
					d += d_upper;
					x--;
				}
				y++;
				if (y > 0 && y < iHeight && x >= 0 && x < iWidth)
				{
					iPos = y * iWidth + x;
					pR[iPos] = R;
					if (oImage.m_pChannel[1])
						pG[iPos] = G;
					if (oImage.m_pChannel[2])
						pB[iPos] = B;
				}
				//else
					//break;
			}
		}
	}
	return;
}
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
			poLine->theta += PI;

		if (y0 == y1)
			poLine->m_iMode = Line_1::Mode::Degree_180;
		else
			poLine->m_iMode = Line_1::Mode::Degree_Other;
	}
	else
	{
		poLine->k = fNaN();
		poLine->m = x0;	//x=x0
		if (y0 > y1)
			poLine->theta = 3 * PI / 2;
		else
			poLine->theta = PI / 2;
		poLine->m_iMode = Line_1::Mode::Degree_90;
	}
	return;
}

void Draw_Arc(Image oImage, int r, int iCenter_x, int iCenter_y, float fAngle_Start, float fAngle_End, int R, int G, int B)
{//画圆弧，当然可以闭合了成为圆
	int Y, U, V;
	_RGB_2_YUV(R, G, B, Y, U, V);
	float fTheta, delta_theta = (float)(1.f * 0.99f / (r * 2.f));
	int y, x, iPos;
	float fy, fx;

	if (fAngle_End > fAngle_Start)
	{
		for (fTheta = fAngle_Start; fTheta <= fAngle_End; fTheta += delta_theta)
		{
			fy = (float)r * (float)sin(fTheta);
			fx = (float)r * (float)cos(fTheta);
			y = (fy >= 0 ? (int)(fy + 0.5) : (int)(fy - 0.5)) + iCenter_y;
			x = (fx >= 0 ? (int)(fx + 0.5) : (int)(fx - 0.5)) + iCenter_x;
			iPos = y * oImage.m_iWidth + x;
			if (y >= 0 && y < oImage.m_iHeight && x >= 0 && x < oImage.m_iWidth)
			{
				if(oImage.m_pChannel[0])
					oImage.m_pChannel[0][iPos] = R;
				if (oImage.m_pChannel[1])
					oImage.m_pChannel[1][iPos] = G;
				if (oImage.m_pChannel[2])
					oImage.m_pChannel[2][iPos] = B;
			}
		}
	}
	else
	{
		for (fTheta = fAngle_Start; fTheta >= fAngle_End; fTheta -= delta_theta)
		{
			fy = (float)r * (float)sin(fTheta);
			fx = (float)r * (float)cos(fTheta);
			y = (fy >= 0 ? (int)(fy + 0.5) : (int)(fy - 0.5)) + iCenter_y;
			x = (fx >= 0 ? (int)(fx + 0.5) : (int)(fx - 0.5)) + iCenter_x;
			iPos = y * oImage.m_iWidth + x;
			if (y >= 0 && y < oImage.m_iHeight && x >= 0 && x < oImage.m_iWidth)
			{
				oImage.m_pChannel[0][iPos] = R;
				oImage.m_pChannel[1][iPos] = G;
				oImage.m_pChannel[2][iPos] = B;
			}
		}
	}
	return;
}

void _Fill_Region(Image* poImage, int iSeed_x, int iSeed_y, int iBack_Color, int iFill_Color)
{
	int iPos = iSeed_y * poImage->m_iWidth,
		iWidth = poImage->m_iWidth;
	unsigned char* pCur_Next,
		* pCur_R = poImage->m_pChannel[0] ? &poImage->m_pChannel[0][iPos] : NULL,
		* pCur_G = poImage->m_pChannel[1] ? &poImage->m_pChannel[1][iPos] : NULL,
		* pCur_B = poImage->m_pChannel[2] ? &poImage->m_pChannel[2][iPos] : NULL;

	int x, x_Left, x_Right;	//最左边，最右边可填充区域

	if (iSeed_y < 0)
		return;
	//寻找最左
	for (x = iSeed_x; x >= 0 && pCur_R[x] == iBack_Color;)
	{
		if (pCur_R)
			pCur_R[x] = iFill_Color;
		if(pCur_G)
			pCur_G[x] = iFill_Color;
		if(pCur_G)
			pCur_B[x] = iFill_Color;
		x--;
	}

	x_Left = x + 1;
	x_Right = iSeed_x;

	//向上看
	if (iSeed_y > 0)
	{
		pCur_Next = pCur_R - iWidth;
		for (x = x_Left; x < x_Right;)
		{
			if (pCur_Next[x] == iBack_Color)
			{
				//向右寻找最右点
				while (x < iWidth && pCur_Next[x] == iBack_Color)
					x++;
				_Fill_Region(poImage, x - 1, iSeed_y - 1, iBack_Color, iFill_Color);
			}
			else
				x++;
		}
	}

	//向下望
	if (iSeed_y < iWidth - 1)
	{
		pCur_Next = pCur_R + iWidth;
		for (x = x_Left; x < x_Right;)
		{
			if (pCur_Next[x] == iBack_Color)
			{
				//向右寻找最右点
				while (x < iWidth && pCur_Next[x] == iBack_Color)
					x++;
				_Fill_Region(poImage, x - 1, iSeed_y + 1, iBack_Color, iFill_Color);
			}
			else
				x++;
		}
	}

	return;
}
void Fill_Region(Image oImage, int iSeed_x, int iSeed_y, int iBack_Color)
{//为了简化，只设一种背景色，只搞Y分量
	unsigned char* pCur = &oImage.m_pChannel[0][iSeed_y * oImage.m_iWidth];
	int x;

	if (iSeed_y >= 0)
	{
		if (pCur[iSeed_x] != iBack_Color)
		{
			printf("The Seed is not in back color:%d\n", oImage.m_pChannel[0][iSeed_y * oImage.m_iWidth + iSeed_x]);
			return;
		}
		//寻找最右
		for (x = iSeed_x; x < oImage.m_iWidth && pCur[x] == iBack_Color; x++);
		iSeed_x = x - 1;
	}
	_Fill_Region(&oImage, iSeed_x, iSeed_y, iBack_Color, 255);
}
void Draw_Point(Image oImage, int x, int y, int r, int R, int G, int B)
{//平面上画一个点
	if (x < 0 || x >= oImage.m_iWidth || y < 0 || y >= oImage.m_iHeight)
		return;

	Draw_Arc(oImage, r, x, y, 0, 2 * PI, R, G, B);

	//Fill_Region(oImage, x, y);
	if(oImage.m_pChannel[0])
		oImage.m_pChannel[0][y * oImage.m_iWidth + x] = R;
	if (oImage.m_pChannel[1])
		oImage.m_pChannel[1][y * oImage.m_iWidth + x] = G;
	if (oImage.m_pChannel[2])
		oImage.m_pChannel[2][y * oImage.m_iWidth + x] = B;
	return;
}
void Gen_Gauss_Filter(int r, float** ppFilter)
{//根据r的大小计算sigma
	float fValue, fSum = 0;
	int i, j, d = r + r + 1;
	float fSigma = 0.3f * ((d - 1.f) * 0.5f - 1.f) + 0.8f;
	float* pFilter = (float*)malloc(d * sizeof(float));

	for (i = 0; i < r; i++)
	{
		fValue = (float)(i - r) / fSigma;
		pFilter[i] = (float)exp(-0.5f * fValue * fValue);
		fSum += pFilter[i];
	}
	fValue = (float)(i - r) / fSigma;
	pFilter[i] = (float)exp(-0.5f * fValue * fValue);
	fSum = fSum * 2 + pFilter[i];

	//Temp Code
	//for (i = 0; i < r + 1; i++)
		//printf("%f\n", pFilter[i]);

	fValue = 1.f / fSum;
	j = d - 1;
	for (i = 0; i < r; i++, j--)
		pFilter[j] = (pFilter[i] *= fValue);
	pFilter[r] *= fValue;
	*ppFilter = pFilter;

	return;
}

void Gen_Gauss_Filter(int r, float fSigma,float **ppFilter)
{//fSigma对应正态分布中的sigma, 这样就能构造出一个完美的高斯核，其和为1
//此处还要进一步搞，fSigma的取值决定了核大小
	int i,j, d = r + r + 1;
	float* pFilter = (float*)malloc(d * sizeof(float));
	float fValue,fSum=0;

	////感觉这个核做得太费事，做一般就够了
	//for (i = 0; i < d; i++)
	//{
	//	fValue = (float)(i - r) / fSigma;
	//	pFilter[i] = exp(-0.5f * fValue * fValue);
	//	fSum += pFilter[i];
	//}
	//fValue = 1.f / fSum;
	//for (i = 0; i < d; i++)
	//	pFilter[i] *= fValue;

	for (i = 0; i < r; i++)
	{
		fValue = (float)(i - r) / fSigma;
		pFilter[i] = (float)exp(-0.5f * fValue * fValue);
		fSum += pFilter[i];
	}
	fValue = (float)(i - r) / fSigma;
	pFilter[i] =(float)exp(-0.5f * fValue * fValue);
	fSum = fSum*2+ pFilter[i];

	fValue = 1.f / fSum;
	j = d - 1;
	for (i = 0; i < r; i++,j--)
		pFilter[j] = (pFilter[i] *= fValue);
	pFilter[r] *= fValue;
	*ppFilter = pFilter;
	
	return;
}

void Transpose_AVX512(float* pSource, int iWidth, int iHeight, float* pDest)
{//尝试转置
	int y, x,i,iPos, iEnd;
	int iWidth_Align_16 = (iWidth >> 4) << 4,
		iHeight_Align_16 = (iHeight >> 4) << 4;
	int x_Remain_Mask = (1 << (iHeight - iHeight_Align_16)) - 1;
		//y_Remain_Mask = (1 << (iWidth - iWidth_Align_16)) - 1;

	union {
		__m512i vIndex;
		int _vIndex[16];
	};

	float Buffer[16][16];

	for (x = 0; x < 16; x++)
		_vIndex[x] = x * 16;
		//vIndex.m512i_i32[x] = x * 16;	//这行Intel 编译器报错
		
	for (y = 0; y < iHeight_Align_16; y += 16)
	{//快点
		for (x = 0; x < iWidth_Align_16; x += 16)
		{
			iPos = y * iWidth + x;
			iEnd = y<iHeight_Align_16?16:iHeight - y;
			for (i = 0; i < iEnd; i++, iPos += iWidth)
				_mm512_i32scatter_ps(&Buffer[0][i], vIndex, _mm512_loadu_ps(&pSource[iPos]), 4);

			//然后写入
			iPos = x * iHeight + y;
			iEnd = x<iWidth_Align_16?16:iWidth - x;
			for (i = 0; i < iEnd; i++, iPos += iHeight)
				_mm512_storeu_ps(&pDest[iPos], *(__m512*)Buffer[i]);
				//*(__m512*)&pDest[iPos] = *(__m512*)Buffer[i];
			//*(__m512*)&pDest[iPos] = *(__m512*)&Buffer[i][1];
		}
		if (x < iWidth)
		{//补
			iPos = y * iWidth + x;
			//_mm512_i32scatter_ps 是根据vIndex里的位置来进行写入，转置就在这里完成
			//读入的是一行16个数据，写入Buffer是一列
			for (i = 0; i < 16; i++, iPos += iWidth)
				_mm512_i32scatter_ps(&Buffer[0][i], vIndex, _mm512_loadu_ps(&pSource[iPos]), 4);

			//然后写入
			iPos = x * iHeight + y;
			iEnd = iWidth - x;
			for (i = 0; i < iEnd; i++, iPos += iHeight)
				*(__m512*)&pDest[iPos] = *(__m512*)Buffer[i];
		}
	}

	if (y < iHeight)
	{
		for (x = 0; x < iWidth_Align_16; x += 16)
		{
			iPos = y * iWidth + x;
			iEnd = y < iHeight_Align_16 ? 16 : iHeight - y;
			for (i = 0; i < iEnd; i++, iPos += iWidth)
				_mm512_i32scatter_ps(&Buffer[0][i], vIndex, _mm512_loadu_ps(&pSource[iPos]), 4);

			//然后写入
			iPos = x * iHeight + y;
			iEnd = x < iWidth_Align_16 ? 16 : iWidth - x;
			
			for (i = 0; i < iEnd; i++, iPos += iHeight)
				_mm512_mask_compressstoreu_ps(&pDest[iPos], x_Remain_Mask, *(__m512*)Buffer[i]);
		}
	}
	return;
}

void _Gause_Filter_AVX512(float* pSource, int iWidth, int iHeight, float* pDest, int r, float* pFilter)
{
	__m512 Pixel_0_16, Pixel_1_16, Sum_16;
	int iWidth_Align_16 = (iWidth >> 4) << 4;		//总是按照16对齐来做，快一些
		//iHeight_Align_16 = (iHeight >> 4) << 4;
	//以下是个掩码，只处理部分的意思
	int x_Remain_Mask = (1 << (iWidth - iWidth_Align_16)) - 1;
		//y_Remain_Mask = (1 << (iHeight - iHeight_Align_16)) - 1;
	int x, y, y1, i;	// , d = r + r + 1;
	//int d_minus_1 = d - 1;
	float* pFilter_1 = &pFilter[r];

	//对列方向做一维高斯滤波
	for (y = 0; y < iHeight; y++)
	{//大体上，像素处理的过程是先行后列
		for (x = 0; x < iWidth; x += 16)
		{
			Sum_16 = _mm512_set1_ps(0.f);	//表示将Sum_16中16个数据清空
			for (i = 1; i <= r; i++)
			{//以自己为中心，向上下两个方向逐步散出去
				y1 = y - i;	//y1表示自己上面第i行，Pixel_0_16表示从内存中连续读入16个数据进寄存器
				Pixel_0_16 = _mm512_loadu_ps(&pSource[x + iWidth * (y1 < 0 ? 0 : y1)]);
				y1 = y + i; //y1表示下面第i行，Pixel_1_16表示从内存中连续读入16个数据进寄存器
				Pixel_1_16 = _mm512_loadu_ps(&pSource[x + iWidth * (y1 >= iHeight ? iHeight - 1 : y1)]);
				//Sum = (Pixel_0 + Pixel_1) * Filter[i]，此处利用了一个性质，以自己为中心，高斯核竖着看
				//上面第i个数据与限免第i个数据对应同一个权值，这是由高斯核的对成性决定的。
				Sum_16 = _mm512_add_ps(Sum_16, _mm512_mul_ps(_mm512_add_ps(Pixel_0_16, Pixel_1_16), _mm512_set1_ps(pFilter_1[i])));
			}
			//以下两句把自己也加上
			Pixel_0_16 = _mm512_loadu_ps(&pSource[x + iWidth * y]);
			Sum_16 = _mm512_add_ps(Sum_16, _mm512_mul_ps(Pixel_0_16, _mm512_set1_ps(*pFilter_1)));

			//算到这里，16个加权求和已经得到，横着存回到目标内存位置中
			if (x + 16 > iWidth)	//这里是不够16的情况下，由
				_mm512_mask_compressstoreu_ps(&pDest[y * iWidth + x], x_Remain_Mask, Sum_16);
			else					//这里够16个数据，简单写入内存即可
				_mm512_storeu_ps(&pDest[y*iWidth+x],Sum_16);	//保持原图顺序
		}
	}
}

void Gauss_Filter_AVX512(float* pSource, int iWidth, int iHeight, float* pDest, float *pAux, int r, float* pFilter)
{//利用一个临时缓冲区pAux进行交换
	//依旧是行列分离的方法来做，所以要做两次,此处是第一次，高斯核列方向上做
	_Gause_Filter_AVX512(pSource, iWidth, iHeight, pAux, r, pFilter);
	//转置一下，内存的访问速度决定了经过这一步更快
	Transpose_AVX512(pAux, iWidth, iHeight, pDest);
	//转置完了以后，图转换为h*w的形状，用同样的方法做第二次，高斯核还是在列方向
	_Gause_Filter_AVX512(pDest, iHeight, iWidth,pAux, r, pFilter);
	//再转置，恢复原图
	Transpose_AVX512(pAux, iHeight, iWidth, pDest);
	return;
}
void Gauss_Filter_Ref(float* pSource, int iWidth, int iHeight,float *pDest,int r,float *pFilter)
{//对一张浮点数表示的图进行高斯模糊，这个作为基准程序，用以对数据
	float *pFilter_Mid;
	float* pTemp = (float*)malloc(iWidth * iHeight * sizeof(float));
	float fSum;
	int x, y,i,x1,y1,iPos;
	pFilter_Mid = pFilter + r;
	
	//先列
	for (x = 0; x < iWidth; x++)
	{
		for (y = 0; y < iHeight; y++)
		{
			for (fSum = 0, i = -r; i <= r; i++)
			{
				y1 = y + i;
				y1 = Clip3(0, iHeight - 1, y1);
				//if (y == 112 && x==0)
					//printf("%f\n", pSource[y1 * iWidth + x]);
				//fSum+=pTemp[y1 *iWidth+x]* pFilter_Mid[i];
				fSum += pSource[y1 * iWidth + x] * pFilter_Mid[i];
				//printf("pixel:%f weight:%f sum:%f\n", pSource[y1 * iWidth + x] * pFilter_Mid[i], pFilter_Mid[i], fSum);
			}
			//pDest[y * iWidth + x] = fSum;
			pTemp[y * iWidth + x] = fSum;
		}
	}
	//for (int y = 0; y <= r; y++)
		//printf("%f\n",pTemp[x]);

	//后行
	for (y = 0; y < iHeight; y++)
	{
		iPos = y * iWidth;
		for (x = 0; x < iWidth; x++)
		{
			for (fSum = 0, i = -r; i <= r; i++)
			{
				x1 = x + i;
				//fSum += pSource[iPos + Clip3(0, iWidth - 1, x1)] * pFilter_Mid[i];
				fSum += pTemp[iPos + Clip3(0, iWidth - 1, x1)] * pFilter_Mid[i];
				//printf("pixel:%f weight:%f sum:%f\n", pTemp[iPos + Clip3(0, iWidth - 1, x1)], pFilter_Mid[i], fSum);
			}
			//pTemp[iPos + x] = fSum;
			pDest[iPos + x] = fSum;
		}
	}
	free(pTemp);
	return;
}
void Float_2_Image(float* pImage, Image oImage)
{
	int i,j,iSize = oImage.m_iWidth * oImage.m_iHeight;
	int iValue;
	for (i = 0; i < iSize; i++)
	{
		iValue= (int)(pImage[i] * 255.f);
		for(j=0;j<oImage.m_iChannel_Count;j++)
			oImage.m_pChannel[j][i] = iValue;
	}
}
void RGB_2_Gray(Image oImage, float* pImage)
{//RGB 转换灰度图，grey=0.299r+0.587g+0.114b, 再/255.f, 对不上
//另一条转换公式： grey=0.2126r +0.7152g +0.0722b
	int i,iSize = oImage.m_iWidth * oImage.m_iHeight;
	if (oImage.m_iChannel_Count >= 3)
		for (i = 0; i < iSize; i++)
			pImage[i] = ((float)round(0.2126f * oImage.m_pChannel[0][i] + 0.7152 * oImage.m_pChannel[1][i] + 0.0722f * oImage.m_pChannel[2][i])) / 255.f;
	else if (oImage.m_iChannel_Count == 1)
		for (i = 0; i < iSize; i++)
			pImage[i] = oImage.m_pChannel[0][i]/255.f;

	
		//pImage[i] = (0.2126f / 255.f) * oImage.m_pChannel[0][i] + (0.7152 / 255.f) * oImage.m_pChannel[1][i] + (0.0722f / 255.f) * oImage.m_pChannel[2][i];
		//pImage[i] = (0.299f/255.f) * oImage.m_pChannel[0][i] + (0.587 / 255.f)* oImage.m_pChannel[1][i] + (0.114f / 255.f)* oImage.m_pChannel[2][i];
	return;
}
float fGet_Line_y(Line_1* poLine, float x)
{//代入x，求y
	if (poLine->m_iMode == Line_1::Mode::Degree_90)
	{
		if (x != poLine->m)
			return fNaN();	//此处不适用于cuda，得另想办法
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
			return fNaN();
	}
	else if (poLine->m_iMode == Line_1::Mode::Degree_90)
		return poLine->x0;
	else
		return (y - poLine->m) / poLine->k;
}
int iGet_Image_Size(int iWidth, int iHeight, int iChannel_Count)
{//给定一个长宽，求这个图像最多要用多少字节
	int iSize = iWidth * iHeight;
	int iMax_Buffer_Size = iSize * iChannel_Count;
	int iSize_With_Remain = ((iMax_Buffer_Size + 127) >> 7) << 7;
	if (iSize_With_Remain - iMax_Buffer_Size < 64)
		iSize_With_Remain += 128;
	return ALIGN_SIZE_1024(iSize_With_Remain);	//最后对齐内存管理器的块
}
void Attach_Buffer(Image* poImage, unsigned char* pBuffer, int iWidth, int iHeight, int iChannel_Count, int iImage_Type)
{
	if (!poImage || iWidth * iHeight == 0)
	{
		printf("Invalid parameter in Attach_Buffer_GPU\n");
		if (poImage)
			*poImage = { {0 } };
		return;
	}

	poImage->m_iWidth = iWidth;
	poImage->m_iHeight = iHeight;
	poImage->m_iBit_Count = iChannel_Count << 3;
	poImage->m_iChannel_Count = iChannel_Count;
	poImage->m_iMax_Buffer_Size = iWidth * iHeight * poImage->m_iChannel_Count;
	poImage->m_iMem_Src = Mem_Src::CPU;
	poImage->m_bIs_Attached = 1;
	poImage->m_iImage_Type = iImage_Type;

	int i;
	for (i = 0; i < poImage->m_iChannel_Count; i++)
		poImage->m_pChannel[i] = pBuffer + i * iWidth * iHeight;
	for (; i < 4; i++)
		poImage->m_pChannel[i] = NULL;
}

void Place_Image(Image oTile, Image oScreen, int x, int y)
{//x,y可以是负数，也可以超越屏幕以外
	int i,w,w_Align_8, h,
		iTile_Cur, iScreen_Cur,
		x_Screen_Start, y_Screen_Start,x_Screen_End,y_Screen_End,
		x_Tile_Start,y_Tile_Start,x_Tile_End,y_Tile_End;	//真正oScreen开始位置
	unsigned char* pTile_Cur, * pScreen_Cur;
	if (x < 0)
		x_Screen_Start = 0,x_Tile_Start = -x;
	else
		x_Tile_Start = 0,	x_Screen_Start = x;
	w = oTile.m_iWidth - x_Tile_Start;
	if (x_Screen_Start + w > oScreen.m_iWidth)
		w -= x_Screen_Start + w - oScreen.m_iWidth;
	x_Tile_End = x_Tile_Start + w;
	x_Screen_End = x_Screen_Start + w;	
	w_Align_8 = (w >> 3) << 3;

	if (y < 0)
		y_Screen_Start = 0, y_Tile_Start = -y;
	else
		y_Tile_Start = 0,	y_Screen_Start = y;
	h = oTile.m_iHeight - y_Tile_Start;
	if (y_Screen_Start + h > oScreen.m_iHeight)
		h -= y_Screen_Start + h - oScreen.m_iHeight;
	y_Tile_End =y_Tile_Start+ h, y_Screen_End =y_Screen_Start+ h;
	
	
	iTile_Cur = y_Tile_Start * oTile.m_iWidth + x_Tile_Start;
	iScreen_Cur = y_Screen_Start * oScreen.m_iWidth + x_Screen_Start;
	for (y = y_Tile_Start; y < y_Tile_End; y++,iTile_Cur+=oTile.m_iWidth,iScreen_Cur+=oScreen.m_iWidth)
	{
		for (i = 0; i < 3; i++)
		{
			pTile_Cur = &oTile.m_pChannel[i][iTile_Cur];
			pScreen_Cur = &oScreen.m_pChannel[i][iScreen_Cur];
			for (x = 0; x < w_Align_8; x += 8, pTile_Cur+=8,pScreen_Cur +=8)
				*(unsigned long long*)pScreen_Cur = *(unsigned long long*)pTile_Cur;
			for (; x < w; x++)
				*pScreen_Cur++ = *pTile_Cur++;
		}
	}
	return;
}
void Concat_Image(Image oA, Image oB, Image* poC, int iFlag)
{//两图拼一图C = A + B, iFlag =0, 水平。 iFlat=1: 垂直
	Image oC = *poC;
	int i, iBit_Count;
	iBit_Count = Min(oA.m_iBit_Count, oB.m_iBit_Count);
	iBit_Count = Min(iBit_Count, 24);

	if (!oC.m_pBuffer)
		if (iFlag == 0)   //水平
			Init_Image(&oC, oA.m_iWidth + oB.m_iWidth, Max(oA.m_iHeight, oB.m_iHeight), Image::IMAGE_TYPE_BMP, iBit_Count);
		else
			Init_Image(&oC, Max(oA.m_iWidth, oB.m_iWidth), oA.m_iHeight + oB.m_iHeight, Image::IMAGE_TYPE_BMP, iBit_Count);

	if (iFlag == 0)
	{//水平拼接
		for (i = 0; i < iBit_Count / 8; i++)
		{
			int y, x, iDest_Pos, iSource_Pos;
			for (iSource_Pos = 0, y = 0; y < oA.m_iHeight; y++)
			{
				iDest_Pos = y * oC.m_iWidth;
				for (x = 0; x < oA.m_iWidth; x++, iDest_Pos++, iSource_Pos++)
					oC.m_pChannel[i][iDest_Pos] = oA.m_pChannel[i][iSource_Pos];
			}
			for (iSource_Pos = 0, y = 0; y < oB.m_iHeight; y++)
			{
				iDest_Pos = y * oC.m_iWidth + oA.m_iWidth;
				for (x = 0; x < oB.m_iWidth; x++, iDest_Pos++, iSource_Pos++)
					oC.m_pChannel[i][iDest_Pos] = oB.m_pChannel[i][iSource_Pos];
			}
		}
	}
	*poC = oC;
	return;
}
template<typename _T>void Draw_Match_Point(_T Point_1[][2], _T Point_2[][2], int iCount, Image oImage)
{
	int iOrg_Image_Width = oImage.m_iWidth / 2;
	int i;
	for (i = 0; i < iCount; i++)
	{
		Draw_Point(oImage, (int)Point_1[i][0], (int)Point_1[i][1], 5, 255, 0, 0);
		Draw_Point(oImage, (int)Point_2[i][0] + iOrg_Image_Width, (int)Point_1[i][1], 5, 0, 255, 0);
		Mid_Point_Line(oImage, (int)Point_1[i][0], (int)Point_1[i][1],
			(int)Point_2[i][0] + iOrg_Image_Width, (int)Point_1[i][1]);
	}
	return;
}
template<typename _T>void Draw_Match_Point(_T Point_1[][2], _T Point_2[][2], unsigned char Mask[], int iCount, Image oImage)
{//oImage通常是两张图拼接而成
	int iOrg_Image_Width = oImage.m_iWidth / 2;
	int i;
	for (i = 0; i < iCount; i++)
	{
		if (Mask[i])
		{
			Draw_Point(oImage, (int)Point_1[i][0], (int)Point_1[i][1], 5, 255, 0, 0);
			Draw_Point(oImage, (int)Point_2[i][0] + iOrg_Image_Width, (int)Point_1[i][1], 5, 0, 255, 0);
			Mid_Point_Line(oImage, (int)Point_1[i][0], (int)Point_1[i][1],
				(int)Point_2[i][0] + iOrg_Image_Width, (int)Point_1[i][1]);
		}
	}
	return;
}
void SB_Image()
{
	Get_Bounding_Box((unsigned short(*)[2])NULL, 0, (unsigned short(*)[2])NULL);
	
	Draw_Match_Point((double(*)[2])NULL, (double(*)[2])NULL, 0, {});
	Draw_Match_Point((float(*)[2])NULL, (float(*)[2])NULL, 0, {});

	Draw_Match_Point((double(*)[2])NULL, (double(*)[2])NULL, NULL, 0, {});
	Draw_Match_Point((float(*)[2])NULL, (float(*)[2])NULL, NULL, 0, {});

}
template<typename _T>void Get_Bounding_Box(_T Point[][2], int iPoint_Count,_T Bounding_Box[2][2])
{
	int i;
	//此处多手搞一个范例，寻找最大最小值的范例
	Bounding_Box[0][0] = Bounding_Box[1][0] = Point[0][0];
	Bounding_Box[0][1] = Bounding_Box[1][1] = Point[0][1];
	for (i = 1; i < iPoint_Count; i++)
	{
		if (Point[i][0] < Bounding_Box[0][0])
			Bounding_Box[0][0] = Point[i][0];
		else if (Point[i][0] > Bounding_Box[1][0])
			Bounding_Box[1][0] = Point[i][0];

		if (Point[i][1] < Bounding_Box[0][1])
			Bounding_Box[0][1] = Point[i][1];
		else if (Point[i][1] > Bounding_Box[1][1])
			Bounding_Box[1][1] = Point[i][1];
	}
	return;
}
void Box_Filter_Ref(Image oSource,Image oDest,int r)
{//毫无利用价值，只有数据意义，故此作为参考程序
	int iHeight_Minus_1 = oSource.m_iHeight - 1,
		iWidth_Minus_1 = oSource.m_iWidth - 1,
		iStencil_Size = (2*r+1) * (2*r+1);
	for (int iChannel = 0; iChannel < oSource.m_iChannel_Count; iChannel++)
	{
		unsigned char* pSource = oSource.m_pChannel[iChannel],
			* pDest = oDest.m_pChannel[iChannel];
		for (int y = 0; y < oSource.m_iHeight; y++)
		{
			for (int x = 0; x < oSource.m_iWidth; x++)
			{//以及该确定了中心(x,y)位置
				/*if (y == 100 && x == 799)
					printf("here");*/
				int iTotal = 0;
				for (int i = -r; i <= r; i++)
				{
					int y_s = y + i;
					unsigned char *pSource_1 = &pSource[Clip3(0, iHeight_Minus_1, y_s) * oSource.m_iWidth];
					for (int j = -r; j <= r; j++)
					{
						int x_s = x + j;
						x_s = Clip3(0, iWidth_Minus_1, x_s);
						iTotal += pSource_1[x_s];
						/*if (y == 100 && x == 799 && pSource_1[x_s]!=255)
							printf("here");*/
					}
				}
				pDest[y*oDest.m_iWidth + x] = iTotal / iStencil_Size;
			}
		}
	}
	return;
}

void Box_Filter(Image oSource, Image oDest, int r)
{//不但行列分离，而且无所不用其极，感觉c范围内已经优无可优
	unsigned int* pSum_Buffer = (unsigned int*)pMalloc(oSource.m_iWidth * oSource.m_iHeight*sizeof(unsigned int));
	const int iWidth_Minus_1 = oSource.m_iWidth - 1,
		iHeight_Minus_1 = oSource.m_iHeight -1,
		iStencil_Size = (2 * r + 1) * (2 * r + 1);
	float iStencil_Size_Recip = 1.f / iStencil_Size;
	//memset(pSum_Buffer, 0, oSource.m_iWidth * oSource.m_iHeight * sizeof(unsigned int));
	int bFast_Row_Process = oSource.m_iWidth >= 2 * r + 1?1 : 0,
		bFast_Col_Process = oSource.m_iHeight >= 2 * r + 1 ? 1 : 0;

	for (int iChannel = 0; iChannel < oSource.m_iChannel_Count; iChannel++)
	{
		//先搞行
		for (int y = 0; y < oSource.m_iHeight; y++)
		{
			int x, iLeft = -r;	// , iRight = r;
			unsigned char* pSource_Line = &oSource.m_pChannel[iChannel][y * oSource.m_iWidth];

			//此处Dest视为转置矩阵
			unsigned int* pDest_Line = &pSum_Buffer[y];
			unsigned int iTotal = r * pSource_Line[0];
			if (oSource.m_iWidth < r + 1)
			{//根本不够像素,草草了事
				for (x = 0; x < oSource.m_iWidth; x++)
					iTotal += pSource_Line[x];
				//剩下不足的补行尾
				iTotal += pSource_Line[iWidth_Minus_1] * (r + 1 - oSource.m_iWidth);
			}else
			{
				for (x = 0; x <= r; x++)
					iTotal += pSource_Line[x];
			}
			pDest_Line[0] = iTotal;
			pDest_Line += oDest.m_iHeight;
			int iRight = r + 1;

			if (bFast_Row_Process)
			{
				//从[0]移动到[r]
				int iMove_To = r + 1;
				for (x = 1; x < iMove_To; x++)
				{
					iTotal += pSource_Line[iRight++] - pSource_Line[0];
					*pDest_Line = iTotal;
					pDest_Line += oDest.m_iHeight;
				}
				iLeft += r;

				//从[r+1]移动到[width-1-r]
				iMove_To = iWidth_Minus_1 - r;
				for (; x < iMove_To; x++)
				{
					iTotal += pSource_Line[iRight++] - pSource_Line[iLeft++];
					*pDest_Line = iTotal;
					pDest_Line += oDest.m_iHeight;
				}

				//从[width-1-r]移动到[width-1]
				for (; x < oSource.m_iWidth; x++)
				{
					iTotal += pSource_Line[iWidth_Minus_1] - pSource_Line[iLeft++];
					*pDest_Line = iTotal;
					pDest_Line += oDest.m_iHeight;
				}
			}else
			{
				for (x = 1; x < oSource.m_iWidth; x++)
				{
					iTotal -= iLeft < 0 ? pSource_Line[0] : pSource_Line[iLeft];
					iTotal += iRight >= oSource.m_iWidth ? pSource_Line[iWidth_Minus_1] : pSource_Line[iRight];
					*pDest_Line = iTotal;
					iLeft++, iRight++;
					pDest_Line += oDest.m_iHeight;
				}
			}
		}

		//再来一次，还是行推进
		for (int y = 0; y < oSource.m_iWidth; y++)
		{
			int x, iLeft = -r;	// , iRight = r;
			unsigned int* pSource_Line = &pSum_Buffer[y*oSource.m_iHeight];
			//此处要改改，将Dest视为转置矩阵
			unsigned char* pDest_Line = &oDest.m_pChannel[iChannel][y];
			unsigned int iTotal = r * pSource_Line[0];
			if (oSource.m_iHeight < r + 1)
			{//根本不够像素,草草了事
				for (x = 0; x < oSource.m_iHeight; x++)
					iTotal += pSource_Line[x];
				iTotal += pSource_Line[iHeight_Minus_1] * (r + 1 - oSource.m_iHeight);
			}else
			{
				for (x = 0; x <= r; x++)
					iTotal += pSource_Line[x];
			}
			pDest_Line[0] = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
			pDest_Line += oDest.m_iWidth;

			int iRight = r + 1;
			//以下要非常清醒，是从转置当中算数据
			if (bFast_Col_Process)
			{//此处尽可能用快速方法
				int iMove_To = r + 1;
				for (x = 1; x < iMove_To; x++)
				{
					iTotal += pSource_Line[iRight++] - pSource_Line[0];
					//注意不要删下面这行，用来对数据
					//*pDest_Line = iTotal / iStencil_Size;

					*pDest_Line =(unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
					pDest_Line += oDest.m_iWidth;
				}
				iLeft += r;

				iMove_To = iHeight_Minus_1 - r;
				for (; x < iMove_To; x++)
				{
					iTotal += pSource_Line[iRight++] - pSource_Line[iLeft++];
					//注意不要删下面这行，用来对数据
					//*pDest_Line = iTotal / iStencil_Size;

					*pDest_Line = (unsigned char)(iTotal * iStencil_Size_Recip +0.5f);
					pDest_Line += oDest.m_iWidth;
				}

				//从[Height-1-r]移动到[Height-1]
				for (; x < oSource.m_iHeight; x++)
				{
					iTotal += pSource_Line[iHeight_Minus_1] - pSource_Line[iLeft++];
					//注意不要删下面这行，用来对数据
					//*pDest_Line = iTotal / iStencil_Size;

					*pDest_Line = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
					pDest_Line += oDest.m_iWidth;
				}
			}else
			{
				for (x = 1; x < oSource.m_iHeight; x++)
				{
					//if (x == 599 && y == 150)
					//printf("here");
					iTotal -= iLeft < 0 ? pSource_Line[0] : pSource_Line[iLeft];
					iTotal += iRight >= oSource.m_iHeight ? pSource_Line[iHeight_Minus_1] : pSource_Line[iRight];
					//注意不要删下面这行，用来对数据
					*pDest_Line = iTotal / iStencil_Size;

					//对浮点数是不确定的，各家有各家的态度
					//*pDest_Line = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
					iLeft++, iRight++;
					pDest_Line += oDest.m_iWidth;
				}
			}			
		}
	}
	Free(pSum_Buffer);
	return;
}

void Box_Filter_1(Image oSource, Image oDest,int r)
{//尝试行列分离，这个是比较简单的做法，不求极致优化，但求算法验证
	unsigned int* pSum_Buffer = (unsigned int*)pMalloc(oSource.m_iWidth * oSource.m_iHeight*sizeof(unsigned int));
	const int iWidth_Minus_1 = oSource.m_iWidth - 1,
		iHeight_Minus_1 = oSource.m_iHeight -1,
		iStencil_Size = (2 * r + 1) * (2 * r + 1);

	for (int iChannel = 0; iChannel < oSource.m_iChannel_Count; iChannel++)
	{
		//先搞行
		for (int y = 0; y < oSource.m_iHeight; y++)
		{
			int x, iLeft = -r;	// , iRight = r;
			unsigned char* pSource_Line = &oSource.m_pChannel[iChannel][y*oSource.m_iWidth];
			unsigned int* pDest_Line = &pSum_Buffer[y * oSource.m_iWidth];
			/*if (y == 40)
			printf("here");*/
			//先搞第一个点
			unsigned int iTotal = r * pSource_Line[0];
			if (oSource.m_iWidth<r + 1 )
			{//根本不够像素,草草了事
				for (x = 0; x < oSource.m_iWidth; x++)
					iTotal += pSource_Line[x];
				iTotal += pSource_Line[oSource.m_iWidth - 1] * (r + 1 - oSource.m_iWidth);
			}else
			{
				for (x = 0; x <= r; x++)
					iTotal += pSource_Line[x];
			}
			pDest_Line[0] = iTotal;
			int iRight = r + 1;

			for (x = 1; x < oSource.m_iWidth; x++)
			{
				/*if (y == 40 && x == 40)
				printf("here");*/
				iTotal -= iLeft < 0 ? pSource_Line[0] : pSource_Line[iLeft];
				iTotal += iRight >= oSource.m_iWidth ? pSource_Line[iWidth_Minus_1] : pSource_Line[iRight];
				pDest_Line[x] = iTotal;
				iLeft++, iRight++;	
			}
		}

		//再搞列
		for (int x = 0; x < oSource.m_iWidth; x++)
		{
			/*if (x == 40)
			printf("here");*/
			int y, iTop = -r;//
			unsigned int* pSource_Start = &pSum_Buffer[x];
			unsigned char* pDest_Line = &oDest.m_pChannel[iChannel][x];
			unsigned int iTotal = r * pSource_Start[0];
			if (oSource.m_iHeight < r + 1)
			{//根本不够像素,草草了事
				for (y = 0; y < oSource.m_iHeight; y++)
					iTotal += pSource_Start[y*oSource.m_iWidth];
				iTotal += pSource_Start[iHeight_Minus_1*oSource.m_iWidth] * (r + 1 - oSource.m_iWidth);
			}else
			{
				for (y = 0; y <= r; y++)
					iTotal += pSource_Start[y*oSource.m_iWidth];
			}
			pDest_Line[0] = iTotal / iStencil_Size;
			int iBottom = r+1;

			for (y = 1; y < oSource.m_iHeight; y++)
			{
				/*if(y==40 && x==40)
				printf("here");*/
				iTotal -= iTop < 0 ? *pSource_Start : pSource_Start[iTop*oSource.m_iWidth];
				iTotal += iBottom >= oSource.m_iHeight ? pSource_Start[iHeight_Minus_1*oSource.m_iWidth] : pSource_Start[iBottom*oSource.m_iWidth];
				pDest_Line[y * oSource.m_iWidth] = iTotal / iStencil_Size;
				/*if (iTotal / iStencil_Size <100)
				printf("%d ",iTotal / iStencil_Size);*/
				iTop++, iBottom++;	
			}
		}
		//Disp(pSum_Buffer, oSource.m_iHeight, oSource.m_iWidth, "Data");
	}
	Free(pSum_Buffer);
	return;
}
void Compare_Image(Image oSource, Image oDest, int iDiff_Threshold)
{
	if (oSource.m_iWidth != oDest.m_iWidth || oSource.m_iHeight != oDest.m_iHeight || oSource.m_iChannel_Count != oDest.m_iChannel_Count)
	{
		printf("Size Mismatched\n");
		return;
	}
	int i, y, x;
	for (i = 0; i < oSource.m_iChannel_Count; i++)
		for (y = 0; y < oSource.m_iHeight; y++)
			for (x = 0; x < oSource.m_iWidth; x++)
				if (abs(oSource.m_pChannel[i][y * oSource.m_iWidth + x] - oDest.m_pChannel[i][y * oDest.m_iWidth + x]) > iDiff_Threshold)
				{
					//if(x%32==0 &&i==0)
					printf("Mismatched: i:%d y:%d x:%d Source:%d Dest:%d Diff:%d\n", i, y, x, oSource.m_pChannel[i][y * oSource.m_iWidth + x], oDest.m_pChannel[i][y * oDest.m_iWidth + x], abs(oSource.m_pChannel[i][y * oSource.m_iWidth + x] - oDest.m_pChannel[i][y * oDest.m_iWidth + x]));
				}
	return;
}
void Compare_Image(char* pcSource, char* pcDest,int iDiff_Threshold)
{
	Image oSource, oDest;
	bLoad_Image(pcSource, &oSource);
	bLoad_Image(pcDest, &oDest);
	Free_Image(&oSource);
	Free_Image(&oDest);
	Compare_Image(oSource, oDest,iDiff_Threshold);
	return;
}
void Dilate_Bin_Ref(Image oSource, Image oDest, int r)
{//用半径为r的结构元对oSource中颜色为白进行膨胀
	int iHeight_Minus_1 = oSource.m_iHeight - 1,
		iWidth_Minus_1 = oSource.m_iWidth - 1;
	unsigned char* pSource = oSource.m_pChannel[0],
		* pDest = oDest.m_pChannel[0];
	for (int y = 0; y < oSource.m_iHeight; y++)
	{
		for (int x = 0; x < oSource.m_iWidth; x++)
		{
			int bFound = 0;
			for (int i = -r; i <= r; i++)
			{
				int y_s = y + i;
				unsigned char* pSource_1 = &pSource[Clip3(0, iHeight_Minus_1, y_s) * oSource.m_iWidth];
				for (int j = -r; j <= r; j++)
				{
					int x_s = x + j;
					x_s = Clip3(0, iWidth_Minus_1, x_s);
					if (pSource_1[x_s])
					{//颜色不为0
						bFound = 1;
						goto SUCCESS;
					}
				}
			}
		SUCCESS:
			pDest[y * oDest.m_iWidth + x] = bFound * 255;
		}
	}
	return;
}
void Dilate_Bin(Image oSource, int r, Image oDest)
{//感觉都太麻烦了，重新想个更简单的，安全函数，源可以等于目的
	if (!oDest.m_pChannel[0])
		oDest = oSource;
	else if(oSource.m_pChannel[0]!=oDest.m_pChannel[0])
		memcpy(oDest.m_pChannel[0], oSource.m_pChannel[0], oSource.m_iWidth * oDest.m_iHeight);

	const int bFast_Line_Forward = !(oDest.m_iWidth & 0x7);
	int iMove_To, x, x1, y, y1;
	unsigned char* pLine;
	//先干行
	for (y = 0; y < oDest.m_iHeight; y++)
	{
		pLine = &oDest.m_pChannel[0][y*oDest.m_iWidth];
		for (x = 1; x < oDest.m_iWidth;/* x++*/)
		{
			if (bFast_Line_Forward && *(unsigned long long*) & pLine[x] == pLine[x - 1] * 0x0101010101010101)
			{
				x += 8;
				continue;
			}

			if (pLine[x] != pLine[x - 1])
			{//有变化，要处理
				if (pLine[x])
				{//白色，向后扩大
					iMove_To = x - r;
					if (iMove_To < 0)
						iMove_To = 0;
					for (x1 = x-1; x1 >= iMove_To && !pLine[x1]; x1--)
						pLine[x1] = 0xFF;
				}else
				{//黑色，向前推进
					iMove_To = x + r;
					if (iMove_To > oDest.m_iWidth)
						iMove_To = oDest.m_iWidth;
					for(x1=x;x1<iMove_To && !pLine[x1];x1++)
						pLine[x1] = 0xFF;
					x = x1;
				}
			}
			x++;
		}
	}

	//bSave_Image("c:\\tmp\\1.bmp", oDest);
	//先干列
	for (x = 0; x < oDest.m_iWidth; x++)
	{
		unsigned char* pCur = &oDest.m_pChannel[0][x+oDest.m_iWidth];
		for (y = 1; y < oDest.m_iHeight; y++,pCur+=oDest.m_iWidth)
		{
			//unsigned char* pCur = &oDest.m_pChannel[0][y*oDest.m_iWidth+x];
			if (*pCur != pCur[-oDest.m_iWidth])
			{
				if (*pCur)
				{//白色，向上
					iMove_To = y - r;
					if (iMove_To < 0)
						iMove_To = 0;
					unsigned char* pPrev = pCur-oDest.m_iWidth;
					for (y1 = y - 1; y1 >= iMove_To && !*pPrev; y1--)
						*pPrev = 0xFF, pPrev -= oDest.m_iWidth;
				}else
				{//黑色，向下
					iMove_To = y + r;
					if (iMove_To > oDest.m_iHeight)
						iMove_To = oDest.m_iHeight;
					unsigned char* pNext = pCur;
					for(y1=y;y1<iMove_To && !(*pNext);y1++,pNext+=oDest.m_iWidth)
						*pNext = 0xFF;
					y = y1;
					pCur = &oDest.m_pChannel[0][y*oDest.m_iWidth+x];
				}
			}
		}
	}
	//bSave_Image("c:\\tmp\\1.bmp", oDest);
	return;
}

void Draw_Rect(Image oImage, int x, int y, int w, int h, int iThickness,int r,int g,int b)
{//还不够健壮，没时间绣花，回头再收拾
	int x1, x1_End;
	unsigned char Color[] = { (unsigned char)r,(unsigned char)g,(unsigned char)b };

	int iWidth_Minus_1 = oImage.m_iWidth - 1,
		iHeight_Minus_1 = oImage.m_iHeight - 1;
	x1_End = x + w;
	if (x1_End >oImage.m_iWidth)
		x1_End = oImage.m_iWidth;

	int x2 = x, x2_End = x + iThickness;
	x2_End = Clip3(0, oImage.m_iWidth, x2_End);

	int x3 = x + w - iThickness, x3_End = x3 + iThickness;
	x3_End=Clip3(0, oImage.m_iWidth, x3_End);


	for (int iChannel = 0; iChannel < oImage.m_iChannel_Count; iChannel++)
	{
		unsigned char iColor = Color[iChannel];
		int y1, y_End;
		//画上面的水平直线
		y1 = Clip3(0, iHeight_Minus_1, y);
		y_End = Clip3(0, oImage.m_iHeight, y + iThickness);
		for (; y1 <y_End; y1++)
		{
			unsigned char* pLine = &oImage.m_pChannel[iChannel][y1 * oImage.m_iWidth];
			for (x1 = x; x1 < x1_End; x1++)
				pLine[x1] = iColor;
		}

		y1 = y + iThickness;
		y_End = y + h -iThickness;
		y1 = Clip3(0, iHeight_Minus_1,y1);
		y_End = Clip3(0,  oImage.m_iHeight, y_End);
		for (; y1 < y_End; y1++)
		{
			unsigned char* pLine = &oImage.m_pChannel[iChannel][y1 * oImage.m_iWidth];
			for (x1 = x2; x1 < x2_End; x1++)
				pLine[x1] = iColor;

			for (x1 = x3; x1 < x3_End; x1++)
				pLine[x1] = iColor;
		}

		y1 = y + h - iThickness;
		y_End = y + h;
		y1 = Clip3(0, iHeight_Minus_1,y1);
		y_End = Clip3(0,  oImage.m_iHeight, y_End);
		for (; y1 <y_End; y1++)
		{
			unsigned char* pLine = &oImage.m_pChannel[iChannel][y1 * oImage.m_iWidth];
			for (x1 = x; x1 < x1_End; x1++)
				pLine[x1] = iColor;
		}
	}
	//bSave_Image("c:\\tmp\\1.bmp", oImage);
}
