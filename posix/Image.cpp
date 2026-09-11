//这版将淡化BitMap，面向Image，通用型结构
#ifdef WIN32
#pragma once
#endif

#include "Image.h"
#ifdef WIN32
	#include "immintrin.h"
#endif

//extern "C"
//{
//#include "Buddy_System.h"
//}

template void approxPolyDP<unsigned short>(unsigned short Source_Point[][2], int iSource_Count,
	unsigned short Dest_Point[][2], int* piDest_Count, float eps);

float fNaN()
{
	unsigned int iRet = 0x7FFFFFFF;
	return *(float*)(&iRet);
}
void Init_Image_Dup(Image *poImage, Image oSource)
{//照着哦Source 初始化一个新的Image
	Init_Image(poImage, oSource.m_iWidth, oSource.m_iHeight, (Image::Type)oSource.m_iImage_Type, oSource.m_iBit_Count);
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

	//poImage->m_pChannel[0] = (unsigned char*)malloc(iSize_With_Remain);
	poImage->m_pChannel[0] = (unsigned char*)pMalloc(iSize_With_Remain);
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

	Set_Color(*poImage);
}
int bRe_Init_Image(Image* poImage, int iWidth, int iHeight, Image::Type iType, int iBit_Count)
{//重新定义Image尺寸
	int iNew_Channel_Size = iWidth * iHeight,
		iNew_Channel_Count = iBit_Count / 8,
		iNew_Size = iNew_Channel_Size * iNew_Channel_Count,
		iOrg_Width = poImage->m_iWidth,
		iOrg_Height = poImage->m_iHeight,
		iOrg_Channel_Size = poImage->m_iWidth * poImage->m_iHeight,
		iOrg_Channel_Count = poImage->m_iBit_Count / 8,
		iOrg_Size = iOrg_Channel_Size * iOrg_Channel_Count;
	Image oDest;
	Attach_Buffer(&oDest, poImage->m_pBuffer, iWidth, iHeight, iNew_Channel_Count, iType);
	if (iNew_Size <= poImage->m_iWidth * poImage->m_iHeight * poImage->m_iChannel_Count)
	{//够用
		for (int i = 0; i < iNew_Channel_Count; i++)
		{
			if (iWidth == iOrg_Width && iHeight == iOrg_Height && i < iOrg_Channel_Count)
				;	// memcpy(oDest.m_pChannel[i], poImage->m_pChannel[i], iOrg_Channel_Size);
			else
				printf("Not Implemented yet");
		}
		Shrink(oDest.m_pBuffer, ALIGN_SIZE_128(iNew_Size) + 64);
		*poImage = oDest;
	}else
	{
		printf("Not Implemented yet");
		return 0;
	}

	return 1;
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
			poBitMap->m_pRGBA[0] = (unsigned char*)pMalloc(iSize_Width_Remain);
		if (!poBitMap->m_pRGBA[0])
		{
			iRet = 0;
			goto END;
		}
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
		pLine = (unsigned char*)pMalloc(iSize + 4);
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
			Free(pLine);
	}
	if (!iRet)
	{
		Free(poBitMap->m_pRGBA[0]);
		*poBitMap = {};
	}

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
	fseeko(pFile, (unsigned long long)iSize * iFrame, SEEK_SET);
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

	if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"bmp"))
	{
		iResult = bLoad_Image(pcFile, &poImage->m_oBitMap, bNeed_Malloc, poMem_Mgr);
		poImage->m_pBuffer = poImage->m_oBitMap.m_pRGBA[0];
		poImage->m_iImage_Type = Image::IMAGE_TYPE_BMP;
		return iResult;
	}
	else if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"yuv"))
	{
		iResult = bLoad_Image(pcFile, &oImage, iWidth, iHeight, bNeed_Malloc, iFrame);
		poImage->m_iImage_Type = Image::IMAGE_TYPE_YUV_444;
	}
	else
	{
		printf("Fail to open file extension:%s\n", pcFile);
		iResult = 0;
	}

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
	remove(pcFile);
	FILE* pFile = fopen(pcFile, "wb");
	int iSize, i, j, k, iResult, iRet = 0;
	unsigned char* pLine = NULL, * pCur;
	unsigned char* RGBA[4], * RGBA_1[4];
	unsigned char Palatte[1024];
	const int iMax_Size = 100000000;

	BitMap_8Bit::BITMAPFILEHEADER oFile_Header;
	BitMap_8Bit::BITMAPINFOHEADER oInfo_Header;

	if (!pFile)
	{
		printf("Fail to save File :%s in bSave_Image\n", pcFile);
		goto END;
	}

	Set_BitMap_Header(&oFile_Header, &oInfo_Header, oBitMap.m_iWidth, oBitMap.m_iHeight, oBitMap.m_iChannel_Count);
	iSize = oInfo_Header.biWidth * oInfo_Header.biHeight;
	
	if (iSize<=0 || iSize> iMax_Size)
	{
		printf("Invalid siaze in bSave_Image\n");
		goto END;
	}

	if (oBitMap.m_iBit_Count < 32)
	{
		for (i = 0; i < oBitMap.m_iChannel_Count; i++)
			RGBA_1[i] = oBitMap.m_pRGBA[oBitMap.m_iChannel_Count - i - 1] + iSize - oInfo_Header.biWidth;
	}else
	{
		for (i = 0; i < 3; i++)
			RGBA_1[i] = oBitMap.m_pRGBA[3 - i - 1] + iSize - oInfo_Header.biWidth;
		RGBA_1[3] = oBitMap.m_pRGBA[3] + iSize - oInfo_Header.biWidth;
	}


	iSize = ((oBitMap.m_iWidth * oBitMap.m_iChannel_Count + 3) / 4) * 4;
	if(oBitMap.m_iChannel_Count>1)
	{
		pLine = (unsigned char*)pMalloc(iSize);
		memset(pLine + iSize - 4, 0, 4);
	}

	//此处要连写两处，不是bug，奇怪的格式
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
	if (oBitMap.m_iChannel_Count == 1)
	{
		//iResult = (int)fwrite(oBitMap.m_pRGBA[0], 1,oBitMap.m_iWidth * oBitMap.m_iHeight,pFile);
		unsigned char* pCur = &oBitMap.m_pRGBA[0][(oBitMap.m_iHeight - 1) * oBitMap.m_iWidth];
		for (int y = oBitMap.m_iHeight - 1; y >= 0; y--, pCur -= oBitMap.m_iWidth)
		{
			iResult = (int)fwrite(pCur, 1, oBitMap.m_iWidth, pFile);
			if (iResult < oBitMap.m_iWidth)
			{
				printf("Fail to write File in bSave_Image\n");
				goto END;
			}
		}
	}else
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
				printf("Fail to write File in bSave_Image\n");
				goto END;
			}
			for (k = 0; k < oBitMap.m_iChannel_Count; k++)
				RGBA_1[k] -= oInfo_Header.biWidth;
		}
	
	iRet = 1;
END:
	if (pLine)
		Free(pLine);
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
			//free(poImage->m_pChannel[0]);
			Free(poImage->m_pChannel[0]);
			*poImage = {};	//不知哪个标准的c，要求这种写法才能消除Warning
		}else
			printf("Buffer of this image is in GPU\n");
	}/*else
		printf("Ptr is NULL\n");*/
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
int bSave_Comp(const char* pcFile, Image oImage, int iComp)
{//存哪一层
	Image oImage_1;
	//相当于Attach
	Attach_Buffer(&oImage_1, oImage.m_pChannel[iComp], oImage.m_iWidth,
		oImage.m_iHeight, 1, Image::IMAGE_TYPE_BMP);
	return bSave_Image(pcFile, oImage_1);
}
int bSave_Image(const char* pcFile, Image oImage, int iFormat)
{//存个盘，根据类型来存
	int bRet=1;
	int iTo_Save;

	if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"bmp") == 1)
		iTo_Save = Image::IMAGE_TYPE_BMP;
	else if (bStricmp((char*)pcFile + strlen(pcFile) - 3, (char*)"yuv") == 1)
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
void Draw_Line(Image oImage, int x0, int y0, int x1, int y1, int R, int G, int B)
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

	Draw_Arc(oImage, r, x, y, 0, (float)(2 * PI), R, G, B);

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

#ifdef WIN32
void Transpose_AVX512(float* pSource, int iWidth, int iHeight, float* pDest)
{//尝试转置，注意，在高速机器上，这段比最烂的传统转置还烂
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
#endif

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
				fSum += pSource[y1 * iWidth + x] * pFilter_Mid[i];
				//printf("pixel:%f weight:%f sum:%f\n", pSource[y1 * iWidth + x] * pFilter_Mid[i], pFilter_Mid[i], fSum);
			}
			pTemp[y * iWidth + x] = fSum;
		}
	}
	//Disp(pTemp, iHeight, iWidth, "Mid");

	//后行
	for (y = 0; y < iHeight; y++)
	{
		iPos = y * iWidth;
		for (x = 0; x < iWidth; x++)
		{
			for (fSum = 0, i = -r; i <= r; i++)
			{
				x1 = x + i;
				fSum += pTemp[iPos + Clip3(0, iWidth - 1, x1)] * pFilter_Mid[i];
				//printf("pixel:%f weight:%f sum:%f\n", pTemp[iPos + Clip3(0, iWidth - 1, x1)], pFilter_Mid[i], fSum);
			}
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

template<typename _T>void RGB_2_Gray(Image oImage, _T* pImage)
{//RGB 转换灰度图，grey=0.299r+0.587g+0.114b, 再/255.f, 对不上
//另一条转换公式： grey=0.2126r +0.7152g +0.0722b
	int i,iSize = oImage.m_iWidth * oImage.m_iHeight;
	if (oImage.m_iChannel_Count >= 3)
		for (i = 0; i < iSize; i++)
			pImage[i] = ((_T)round(0.2126f * oImage.m_pChannel[0][i] + 0.7152 * oImage.m_pChannel[1][i] + 0.0722f * oImage.m_pChannel[2][i])) / 255.f;
	else if (oImage.m_iChannel_Count == 1)
		for (i = 0; i < iSize; i++)
			pImage[i] = oImage.m_pChannel[0][i]/255.f;

	return;
}

template<typename _T>void RGB_2_Gray(Image oImage, _T** ppImage)
{//用于偷懒，连分配都管了
	_T* pImage;
	*ppImage = pImage = (_T*)pMalloc(oImage.m_iWidth * oImage.m_iHeight * sizeof(_T));
	if (!pImage)
		return;

	RGB_2_Gray(oImage, pImage);
}

int RGB_2_Gray_1(Image oImage, Image oDest)
{
	/*float* pImage = NULL,*pImage_Cur;
	RGB_2_Gray(oImage, &pImage);
	if (!pImage)
	{
		oDest = {};
		return 0;
	}
	int iSize = oImage.m_iWidth * oImage.m_iHeight;
	unsigned char* pCur = oDest.m_pChannel[0], * pEnd = pCur + iSize;
	pImage_Cur = pImage;
	while (pCur < pEnd)
		*(pCur++) = (unsigned char)(*(pImage_Cur++) * 0xFF);
	Free(pImage);*/

	int i, iSize = oImage.m_iWidth * oImage.m_iHeight;
	if (oImage.m_iChannel_Count >= 3)
	{
		for (i = 0; i < iSize; i++)
		{
			int iValue = (int)round(0.2126f * oImage.m_pChannel[0][i] + 0.7152 * oImage.m_pChannel[1][i] + 0.0722f * oImage.m_pChannel[2][i]);
			oDest.m_pChannel[0][i] = (unsigned char)Clip(iValue);
		}
	}else if (oImage.m_iChannel_Count == 1)
		memcpy(oDest.m_pChannel[0], oImage.m_pChannel[0],iSize);
	return 1;
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
	poImage->m_pBuffer = pBuffer;
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
		Draw_Line(oImage, (int)Point_1[i][0], (int)Point_1[i][1],
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
			Draw_Line(oImage, (int)Point_1[i][0], (int)Point_1[i][1],
				(int)Point_2[i][0] + iOrg_Image_Width, (int)Point_1[i][1]);
		}
	}
	return;
}
void SB_Image()
{
	Coordinate_2_Screen<float>(0, 0, NULL, NULL);
	Coordinate_2_Screen<double>(0, 0, NULL, NULL);

	Get_Bounding_Box((unsigned short(*)[2])NULL, 0, (unsigned short(*)[2])NULL);
	Get_Bounding_Box<float>(NULL, 0, NULL);

	Draw_Match_Point((double(*)[2])NULL, (double(*)[2])NULL, 0, {});
	Draw_Match_Point((float(*)[2])NULL, (float(*)[2])NULL, 0, {});

	Draw_Match_Point((double(*)[2])NULL, (double(*)[2])NULL, NULL, 0, {});
	Draw_Match_Point((float(*)[2])NULL, (float(*)[2])NULL, NULL, 0, {});

	Bi_Linear<double>(NULL,0,0,0,0,NULL);
	Bi_Linear<float>(NULL, 0, 0, 0, 0, NULL);
	Bi_Linear<unsigned char>(NULL, 0, 0, 0, 0, NULL);

	Sep_Filter_2D<double>(NULL, 0, 0, NULL, NULL, 0, NULL);
	Sep_Filter_2D<float>(NULL, 0, 0, NULL, NULL, 0, NULL);

	Sep_Filter_2D_1<unsigned char, int, int>(NULL, 0, 0, NULL, NULL, 0, NULL);
	Sep_Filter_2D_1<unsigned char, short, int>(NULL, 0, 0, NULL, NULL, 0, NULL);

	Sep_Filter_2D_1<unsigned char, short, short>(NULL, 0, 0, NULL, NULL, 0, NULL);
	Sep_Filter_2D_1<unsigned char, unsigned char,char>(NULL, 0, 0, NULL, NULL, 0, NULL);
	Sep_Filter_2D_1<unsigned char, char, char>(NULL, 0, 0, NULL, NULL, 0, NULL);

	RGB_2_Gray<float>({}, NULL);
	RGB_2_Gray<double>({}, NULL);
}

template void Get_Bounding_Box(float Point[][2], int iPoint_Count, float Bounding_Box[2][2]);
template void Get_Bounding_Box(double Point[][2], int iPoint_Count, double Bounding_Box[2][2]);
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

template<typename _T>void _Box_Filter(Image oSource, Image oDest, int r, _T* pSum_Buffer)
{
	const int iWidth_Minus_1 = oSource.m_iWidth - 1,
		iHeight_Minus_1 = oSource.m_iHeight - 1,
		iStencil_Size = (2 * r + 1) * (2 * r + 1),
		r_Plus_1 = r + 1;
	float iStencil_Size_Recip = 1.f / iStencil_Size;

	int bFast_Row_Process = oSource.m_iWidth >= 2 * r + 1 ? 1 : 0,
		bFast_Col_Process = oSource.m_iHeight >= 2 * r + 1 ? 1 : 0;

	for (int iChannel = 0; iChannel < oSource.m_iChannel_Count; iChannel++)
	{
		unsigned char* pSource_Line = oSource.m_pChannel[iChannel];
		//先搞行
		for (int y = 0; y < oSource.m_iHeight; y++, pSource_Line += oSource.m_iWidth)
		{
			int x, iLeft = -r;	// , iRight = r;
			//此处Dest视为转置矩阵
			_T* pDest_Line = &pSum_Buffer[y];
			unsigned int iTotal = r * pSource_Line[0];
			if (oSource.m_iWidth < r_Plus_1)
			{//根本不够像素,草草了事
				for (x = 0; x < oSource.m_iWidth; x++)
					iTotal += pSource_Line[x];
				//剩下不足的补行尾
				iTotal += pSource_Line[iWidth_Minus_1] * (r_Plus_1 - oSource.m_iWidth);
			}
			else
			{
				for (x = 0; x <= r; x++)
					iTotal += pSource_Line[x];
			}
			pDest_Line[0] = iTotal;
			pDest_Line += oDest.m_iHeight;
			int iRight = r_Plus_1;

			if (bFast_Row_Process)
			{
				//从[0]移动到[r]
				int iMove_To = r_Plus_1;
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
			}
			else
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
			_T* pSource_Line = &pSum_Buffer[y * oSource.m_iHeight];
			//此处要改改，将Dest视为转置矩阵
			unsigned char* pDest_Line = &oDest.m_pChannel[iChannel][y];
			unsigned int iTotal = r * pSource_Line[0];
			if (oSource.m_iHeight < r_Plus_1)
			{//根本不够像素,草草了事
				for (x = 0; x < oSource.m_iHeight; x++)
					iTotal += pSource_Line[x];
				iTotal += pSource_Line[iHeight_Minus_1] * (r_Plus_1 - oSource.m_iHeight);
			}
			else
			{
				for (x = 0; x <= r; x++)
					iTotal += pSource_Line[x];
			}
			pDest_Line[0] = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
			pDest_Line += oDest.m_iWidth;

			int iRight = r_Plus_1;
			//以下要非常清醒，是从转置当中算数据
			if (bFast_Col_Process)
			{//此处尽可能用快速方法
				int iMove_To = r_Plus_1;
				for (x = 1; x < iMove_To; x++)
				{
					iTotal += pSource_Line[iRight++] - pSource_Line[0];
					//注意不要删下面这行，用来对数据
					//*pDest_Line = iTotal / iStencil_Size;

					*pDest_Line = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
					pDest_Line += oDest.m_iWidth;
				}
				iLeft += r;

				iMove_To = iHeight_Minus_1 - r;
				for (; x < iMove_To; x++)
				{
					iTotal += pSource_Line[iRight++] - pSource_Line[iLeft++];
					//注意不要删下面这行，用来对数据
					//*pDest_Line = iTotal / iStencil_Size;

					*pDest_Line = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
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
			}
			else
			{
				for (x = 1; x < oSource.m_iHeight; x++)
				{
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
}

void Box_Filter(Image oSource, Image oDest, int r)
{//不但行列分离，而且无所不用其极，感觉c范围内已经优无可优
	void* pSum_Buffer = NULL;
	int iSum_Pixel_Size = r <= 127 ? 2 : 4;
	pSum_Buffer = (void*)pMalloc(oSource.m_iWidth * oSource.m_iHeight * iSum_Pixel_Size);
	if (!pSum_Buffer)
	{
		printf("Fail to allocate memory in Box_Filter\n");
		return;
	}

	if (r <= 127)
		_Box_Filter<unsigned short>(oSource, oDest, r, (unsigned short*)pSum_Buffer);
	else
		_Box_Filter<unsigned int>(oSource, oDest, r, (unsigned int*)pSum_Buffer);
	Free(pSum_Buffer);
	return;
}

void Box_Filter_S(Image oSource, Image* poDest, int r)
{//直接包分配
	int r_Plus_1 = r + 1, d = r_Plus_1 + r;
	int w = oSource.m_iWidth, h = oSource.m_iHeight;
	if (d >= 256)
	{
		printf("r cannot exceed 127\n");
		return;
	}
	if (w < d || h < d)
	{
		printf("w or h < d\n");
		return;
	}
	if (oSource.m_iChannel_Count > 1)
	{
		printf("Not implemented in Box_Filter_S\n");
		return;
	}

	*poDest = {};
	unsigned short* pSum_Buffer = (unsigned short*)pMalloc(w * h * sizeof(unsigned short));
	if (!pSum_Buffer)return;

	typedef struct Win {
		int m_iCur = 0;
		int m_iSize;
		unsigned short m_Buffer[128];	//窗口大小应该是r
	}Win;

	Win oWin;
	int w1 = w - 1, iDown = h - r - 1;	//下一个可用位置，不包括自己
	int w_x_r1 = w * r, iMove_To = w1 - r, iSize = w * h;
	float iStencil_Size_Recip = 1.f / (d * d);
	oWin.m_iSize = r + 1;
	Attach_Buffer(poDest, (unsigned char*)pSum_Buffer, w, h, oSource.m_iChannel_Count, (Image::Type)oSource.m_iImage_Type);

	for (int i = 0; i < oSource.m_iChannel_Count; i++)
	{
		unsigned char* pSource_Line = oSource.m_pChannel[i];
		unsigned short* pDest_Line = pSum_Buffer;
		for (int y = 0; y < h; y++, pSource_Line += w, pDest_Line += w)
		{
			int x, iLeft = -r;	// , iRight = r;
			unsigned int iTotal = r * pSource_Line[0];
			for (x = 0; x <= r; x++)
				iTotal += pSource_Line[x];

			*pDest_Line = iTotal;
			int iRight = r + 1;

			//从[0]移动到[r]
			for (x = 1; x < r_Plus_1; x++)
			{
				iTotal += pSource_Line[iRight++] - pSource_Line[0];
				pDest_Line[x] = iTotal;
			}
			iLeft += r;

			//从[r+1]移动到[width-1-r]
			for (; x < iMove_To; x++)
			{
				iTotal += pSource_Line[iRight++] - pSource_Line[iLeft++];
				pDest_Line[x] = iTotal;
			}

			//从[width-1-r]移动到[width-1]
			for (; x < w; x++)
			{
				iTotal += pSource_Line[w1] - pSource_Line[iLeft++];
				pDest_Line[x] = iTotal;
			}
		}

		for (int x = 0; x < w; x++)
		{
			pDest_Line = &pSum_Buffer[x];
			oWin.m_iCur = 0;
			unsigned int iTotal = *pDest_Line * r_Plus_1;
			int y;
			for (y = 0; y < r_Plus_1; y++)
				oWin.m_Buffer[y] = *pDest_Line;
			pDest_Line += w;
			for (y = 1; y <= r; y++, pDest_Line += w)
				iTotal += *pDest_Line;
			pSum_Buffer[x] = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
			pDest_Line = &pSum_Buffer[x + w];
			for (y = 1; y < iDown; y++, pDest_Line += w)
			{
				iTotal += pDest_Line[w_x_r1] - oWin.m_Buffer[oWin.m_iCur];
				oWin.m_Buffer[oWin.m_iCur++] = *pDest_Line;
				*pDest_Line = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
				if (oWin.m_iCur >= oWin.m_iSize)
					oWin.m_iCur = 0;
			}

			int iValue = pDest_Line[w_x_r1];
			for (; y < h; y++, pDest_Line += w)
			{
				iTotal += iValue - oWin.m_Buffer[oWin.m_iCur];
				oWin.m_Buffer[oWin.m_iCur++] = *pDest_Line;
				*pDest_Line = (unsigned char)(iTotal * iStencil_Size_Recip + 0.5f);
				if (oWin.m_iCur >= oWin.m_iSize)
					oWin.m_iCur = 0;
			}
		}

		unsigned char* pDest_Cur = poDest->m_pChannel[i];
		for (int j = 0; j < iSize; j++)
			pDest_Cur[j] = (unsigned char)pSum_Buffer[j];
	}

	Shrink(pSum_Buffer, iSize * oSource.m_iChannel_Count);
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
int Compare_Image(Image oSource, Image oDest, int iDiff_Threshold)
{
	if (oSource.m_iWidth != oDest.m_iWidth || oSource.m_iHeight != oDest.m_iHeight /*|| oSource.m_iChannel_Count != oDest.m_iChannel_Count*/)
	{
		printf("Size Mismatched\n");
		return 0;
	}
	int i, y, x;
	int iCount = 0;
	for (i = 0; i < oSource.m_iChannel_Count; i++)
		for (y = 0; y < oSource.m_iHeight; y++)
			for (x = 0; x < oSource.m_iWidth; x++)
			{
				//printf("%d\n", oSource.m_pChannel[i][y * oSource.m_iWidth + x] - oDest.m_pChannel[i][y * oDest.m_iWidth + x]);
				if (abs(oSource.m_pChannel[i][y * oSource.m_iWidth + x] - oDest.m_pChannel[i][y * oDest.m_iWidth + x]) > iDiff_Threshold)
				{
					//if(x%32==0 &&i==0)
					printf("Mismatched: i:%d y:%d x:%d Source:%d Dest:%d Diff:%d\n", i, y, x, oSource.m_pChannel[i][y * oSource.m_iWidth + x], oDest.m_pChannel[i][y * oDest.m_iWidth + x], abs(oSource.m_pChannel[i][y * oSource.m_iWidth + x] - oDest.m_pChannel[i][y * oDest.m_iWidth + x]));
					return 0;
					iCount++;
				}
			}
	printf("Mismatched count:%d\n", iCount);

	return 1;
}
int Compare_Image(const char* pcSource, const char* pcDest,int iDiff_Threshold)
{
	Image oSource, oDest;
	int iResult = 0;
	bLoad_Image(pcSource, &oSource);
	bLoad_Image(pcDest, &oDest);
	iResult = Compare_Image(oSource, oDest, iDiff_Threshold);
	Free_Image(&oSource);
	Free_Image(&oDest);	
	return iResult;
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
	else if (oSource.m_pChannel[0] != oDest.m_pChannel[0])
		memcpy(oDest.m_pChannel[0], oSource.m_pChannel[0], oSource.m_iWidth * oDest.m_iHeight);

	const int bFast_Line_Forward = !(oDest.m_iWidth & 0x7);
	int iMove_To, x, x1, y, y1;
	unsigned char* pLine;
	//先干行
	for (y = 0; y < oDest.m_iHeight; y++)
	{
		pLine = &oDest.m_pChannel[0][y * oDest.m_iWidth];
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
					for (x1 = x - 1; x1 >= iMove_To && !pLine[x1]; x1--)
						pLine[x1] = 0xFF;
				}
				else
				{//黑色，向前推进
					iMove_To = x + r;
					if (iMove_To > oDest.m_iWidth)
						iMove_To = oDest.m_iWidth;
					for (x1 = x; x1 < iMove_To && !pLine[x1]; x1++)
						pLine[x1] = 0xFF;
					x = x1;
				}
			}
			x++;
		}
	}

	//bSave_Image("c:\\tmp\\1.bmp", oDest);
	//干列
	for (x = 0; x < oDest.m_iWidth; x++)
	{
		unsigned char* pCur = &oDest.m_pChannel[0][x + oDest.m_iWidth];
		for (y = 1; y < oDest.m_iHeight; y++, pCur += oDest.m_iWidth)
		{
			//unsigned char* pCur = &oDest.m_pChannel[0][y*oDest.m_iWidth+x];
			if (*pCur != pCur[-oDest.m_iWidth])
			{
				if (*pCur)
				{//白色，向上
					iMove_To = y - r;
					if (iMove_To < 0)
						iMove_To = 0;
					unsigned char* pPrev = pCur - oDest.m_iWidth;
					for (y1 = y - 1; y1 >= iMove_To && !*pPrev; y1--)
						*pPrev = 0xFF, pPrev -= oDest.m_iWidth;
				}else
				{//黑色，向下
					iMove_To = y + r;
					if (iMove_To > oDest.m_iHeight)
						iMove_To = oDest.m_iHeight;
					unsigned char* pNext = pCur;
					for (y1 = y; y1 < iMove_To && !(*pNext); y1++, pNext += oDest.m_iWidth)
						*pNext = 0xFF;
					y = y1;
					pCur = &oDest.m_pChannel[0][y * oDest.m_iWidth + x];
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

void Crop_Image(Image oSource, int x, int y, int w, int h, Image oDest)
{//将Source得一块拷贝到Dest
	if (oSource.m_iChannel_Count != oDest.m_iChannel_Count)
	{
		printf("Error in Crop_Image\n");
		return;
	}

	int n = oSource.m_iWidth,
		iDest_Stride = oDest.m_iWidth;

	for (int i = 0; i < oSource.m_iChannel_Count; i++)
	{
		unsigned char * pSource_Cur = &oSource.m_pChannel[i][y * n + x],
			* pSource_End = pSource_Cur + h * n,
			* pDest_Cur = oDest.m_pChannel[i];

		for (; pSource_Cur < pSource_End; pSource_Cur += n, pDest_Cur += iDest_Stride)
			for (int i = 0; i < w; i++)
				pDest_Cur[i] = pSource_Cur[i];
	}
}

template<typename _T>void Bi_Linear(_T Source[], int w_s, int h_s, int w_d, int h_d, _T Dest[])
{
	int x, y;
	float x_s_f, y_s_f, //目标x,y对应的源坐标
		w0, w1, w2, w3; //四组权重  w1 = 1-w0   w3 = 1-w2
	int x_s_0, x_s_1,   //目标x坐标对应的左右点坐标
		y_s_0, y_s_1;   //目标y坐标对应的上下坐标
	int w_1 = w_s - 1,             //w-1
		h_1 = h_s - 1;            //h-1

	//两组系数用于计算对应原位置
	float f_y = (float)w_s / w_d;
	float f_x = (float)w_s / w_d;

	_T* pDest_Cur = Dest;
	for (y = 0; y < h_d; y++)
	{
		//opencv方案
		//y_s_f = (y + 0.5f) * f_y - 0.5f;

		//一般方案
		y_s_f = y * f_y;

		y_s_0 = (int)y_s_f;
		y_s_1 = Min(y_s_0 + 1, h_1);
		w2 = (y_s_1 - y_s_f), w3 = 1.f - w2;
		_T* pCur_Line = &Source[y_s_0 * w_s];
		_T* pNext_Line = &Source[y_s_1 * w_s];

		//printf("%d %d\n",y_s_0,y_s_1);
		for (x = 0; x < w_d; x++, pDest_Cur++)
		{
			//opencv方案
			//x_s_f = (x + 0.5f) * f_x - 0.5f;

			//一般方案
			x_s_f = x  * f_x;

			x_s_0 = (int)x_s_f;
			x_s_1 = Min(x_s_0 + 1, w_1);
			w0 = (x_s_1 - x_s_f), w1 = 1.f - w0;
			float fValue_0 = (float)(w0 * pCur_Line[x_s_0] + w1 * pCur_Line[x_s_1]);
			float fValue_1 = (float)(w0 * pNext_Line[x_s_0] + w1 * pNext_Line[x_s_1]);
			*pDest_Cur =(_T)( w2 * fValue_0 + w3 * fValue_1);
		}
	}

	return;
}
int iGet_Border_x(int x, int iWidth,  Border_Type iBorder_Type)
{
	if (x < 0)
	{
		switch (iBorder_Type)
		{
		case Border_Type::BORDER_REFLECT:
			return -x - 1;
		case Border_Type::BORDER_REFLECT_101:
			return -x;
		case Border_Type::BORDER_CONSTANT:
		case Border_Type::BORDER_REPLICATE:
			return 0;
		}
	}
	else if (x >= iWidth)
	{
		switch (iBorder_Type)
		{
		case Border_Type::BORDER_REFLECT:
			return iWidth - (x - iWidth + 1);
		case Border_Type::BORDER_REFLECT_101:
			return iWidth - (x - iWidth + 2);
		case Border_Type::BORDER_CONSTANT:
		case Border_Type::BORDER_REPLICATE:
			return iWidth - 1;
		}
	}
	return x;

	/*if (x < 0)
	{
		switch (iBorder_Type)
		{
		case Border_Type::BORDER_REFLECT:
			return -x - 1;
		case Border_Type::BORDER_REFLECT_101:
			return -x;
		}
	}
	else if (x >= iWidth)
	{
		switch (iBorder_Type)
		{
		case Border_Type::BORDER_REFLECT:
			return iWidth - (x - iWidth + 1);
		case Border_Type::BORDER_REFLECT_101:
			return iWidth - (x - iWidth + 2);
		}
	}
	return x;*/
}

int iGet_Border_y(int y, int iHeight,  Border_Type iBorder_Type)
{
	if (y < 0)
	{
		switch (iBorder_Type)
		{
		case Border_Type::BORDER_REFLECT:
			return -y - 1;
		case Border_Type::BORDER_REFLECT_101:
			return -y;
		case Border_Type::BORDER_CONSTANT:
		case Border_Type::BORDER_REPLICATE:
			return 0;
		}
	}
	else if (y >= iHeight)
	{
		switch (iBorder_Type)
		{

		case Border_Type::BORDER_REFLECT:
			return iHeight - (y - iHeight + 1);
		case Border_Type::BORDER_REFLECT_101:
			return iHeight - (y - iHeight + 2);
		case Border_Type::BORDER_CONSTANT:
		case Border_Type::BORDER_REPLICATE:
			return iHeight - 1;
		}
	}
	return y;

	/*if (y < 0)
	{
		switch (iBorder_Type)
		{
		case Border_Type::BORDER_CONSTANT:
			return -1;
		case Border_Type::BORDER_REFLECT:
			return -y-1;
		case Border_Type::BORDER_REFLECT_101:
			return -y;
		}
	}else if(y>=iHeight)
	{
		switch (iBorder_Type)
		{
		case Border_Type::BORDER_CONSTANT:
			return -1;
		case Border_Type::BORDER_REFLECT:
			return iHeight - (y - iHeight + 1);
		case Border_Type::BORDER_REFLECT_101:
			return iHeight -(y - iHeight + 1);
		}
	}
	return y;*/
}
void Bi_Linear_cv(Image oSource, Image oDest, float fScale_x, float fScale_y, Border_Type iBorder_Type)
{//跟随Opencv,仅差1
	int i, x, y;
	float x_s_f, y_s_f, //目标x,y对应的源坐标
		w0, w1, w2, w3; //四组权重  w1 = 1-w0   w3 = 1-w2
	int x_s_0, x_s_1,   //目标x坐标对应的左右点坐标
		y_s_0, y_s_1;   //目标y坐标对应的上下坐标
	int w_1 = oSource.m_iWidth - 1,             //w-1
		h_1 = oSource.m_iHeight - 1;            //h-1

	if (oSource.m_iChannel_Count != oDest.m_iChannel_Count)
	{
		printf("Error in Resize_Bi_Linear\n");
		return;
	}

	//0.45643546458763845

	//两组系数用于计算对应原位置
	float f_y = 1.f / fScale_y;
	float f_x = 1.f / fScale_x;

	unsigned char* pDest_Cur = oDest.m_pChannel[0];
	for (i = 0; i < oDest.m_iChannel_Count; i++)
	{
		for (y = 0; y < oDest.m_iHeight; y++)
		{
			y_s_f = (y + 0.5f) * f_y - 0.5f;
			//y_s_f = y * f_y;

			y_s_0 = (int)floor(y_s_f);
			y_s_1 = Min(y_s_0 + 1, h_1);
			w2 = (y_s_1 - y_s_f), w3 = 1.f - w2;

			int iCur_y = iGet_Border_y(y_s_0, oSource.m_iHeight, iBorder_Type);
			int iNext_y = iGet_Border_y(y_s_1, oSource.m_iHeight, iBorder_Type);
			unsigned char* pCur_Line = &oSource.m_pChannel[i][iCur_y * oSource.m_iWidth];
			unsigned char* pNext_Line = &oSource.m_pChannel[i][iNext_y * oSource.m_iWidth];
			for (x = 0; x < oDest.m_iWidth; x++, pDest_Cur++)
			{
				x_s_f = (x + 0.5f) * f_x - 0.5f;
				//x_s_f = x  * f_x;
				x_s_0 = (int)floor(x_s_f);
				x_s_1 = Min(x_s_0 + 1, w_1);
				w0 = (x_s_1 - x_s_f), w1 = 1.f - w0;
				//以下为真实取值位置 
				int x_s_0_r = iGet_Border_x(x_s_0, oSource.m_iWidth, iBorder_Type),
					x_s_1_r = iGet_Border_x(x_s_1, oSource.m_iWidth, iBorder_Type);
				float fValue_0, fValue_1;

				fValue_0 = w0 * pCur_Line[x_s_0_r] + w1 * pCur_Line[x_s_1_r];
				fValue_1 = w0 * pNext_Line[x_s_0_r] + w1 * pNext_Line[x_s_1_r];
				*pDest_Cur = (unsigned char)(w2 * fValue_0 + w3 * fValue_1 + 0.5);
			}

		}
	}
}
void Bi_Linear_Ref(Image oSource, Image oDest, Border_Type iBorder_Type)
{//线建立一个参考程序，用错开半个相位法
//尚未对齐Opencv数据,尚待测试
	int i, x, y;
	float x_s_f, y_s_f, //目标x,y对应的源坐标
		w0, w1, w2, w3; //四组权重  w1 = 1-w0   w3 = 1-w2
	int x_s_0, x_s_1,   //目标x坐标对应的左右点坐标
		y_s_0, y_s_1;   //目标y坐标对应的上下坐标
	int w_1 = oSource.m_iWidth - 1,             //w-1
		h_1 = oSource.m_iHeight - 1;            //h-1

	/*if (oSource.m_iChannel_Count != oDest.m_iChannel_Count)
	{
		printf("Error in Resize_Bi_Linear\n");
		return;
	}*/

	//0.45643546458763845

	//两组系数用于计算对应原位置
	float f_y = (float)oSource.m_iHeight / oDest.m_iHeight;
	float f_x = (float)oSource.m_iWidth / oDest.m_iWidth;

	unsigned char* pDest_Cur = oDest.m_pChannel[0];
	int iNew_Channel_Count = Min(oSource.m_iChannel_Count, oDest.m_iChannel_Count);

	//for (i = 0; i < oDest.m_iChannel_Count; i++)
	for (i = 0; i < iNew_Channel_Count; i++)
	{
		for (y = 0; y < oDest.m_iHeight; y++)
		{
			/*if (y == 1)
				printf("here");*/
			y_s_f = (y + 0.5f) * f_y - 0.5f;
			//y_s_f = y * f_y;

			y_s_0 = (int)floor( y_s_f);
			y_s_1 = y_s_0 + 1;	//Min(y_s_0 + 1, h_1);
			w2 = (y_s_1 - y_s_f), w3 = 1.f - w2;

			unsigned char A=1, B=1, C=1, D=1;
			int iCur_y = iGet_Border_y(y_s_0, oSource.m_iHeight, iBorder_Type);
			int iNext_y = iGet_Border_y(y_s_1, oSource.m_iHeight, iBorder_Type);
			unsigned char* pCur_Line = &oSource.m_pChannel[i][iCur_y * oSource.m_iWidth];
			unsigned char* pNext_Line = &oSource.m_pChannel[i][iNext_y * oSource.m_iWidth];
			/*if (iCur_y < 0)
				A = B = 0;
			else if (iNext_y < 0)
				C = D = 0;*/
			for (x = 0; x < oDest.m_iWidth; x++, pDest_Cur++)
			{
				x_s_f = (x + 0.5f) * f_x - 0.5f;
				//x_s_f = x  * f_x;
				x_s_0 = (int)floor(x_s_f);
				x_s_1 = Min(x_s_0 + 1, w_1);
				w0 = (x_s_1 - x_s_f), w1 = 1.f - w0;
				//以下为真实取值位置 
				int x_s_0_r = iGet_Border_x(x_s_0, oSource.m_iWidth,  iBorder_Type),
					x_s_1_r = iGet_Border_x(x_s_1, oSource.m_iWidth,  iBorder_Type);
				float fValue_0, fValue_1;
				
				/*if (y == 0 && x == 114)
					printf("Here");*/
				/*if (x_s_0_r < 0)
					A = C = 0;
				else if (x_s_1_r < 0)
					B = D = 0;
				else
					C = D = 1;*/
				fValue_0 = w0 * pCur_Line[x_s_0_r]*A + w1 * pCur_Line[x_s_1_r]*B;
				fValue_1 = w0 * pNext_Line[x_s_0_r]*C + w1 * pNext_Line[x_s_1_r]*D;
				*pDest_Cur = (unsigned char)(w2 * fValue_0 + w3 * fValue_1 +0.5);
			}
		}
	}
	if (iNew_Channel_Count < oDest.m_iChannel_Count)
	{
		for (int i = iNew_Channel_Count; i < oDest.m_iChannel_Count; i++)
			memcpy(oDest.m_pChannel[i], oDest.m_pChannel[0], oDest.m_iWidth * oDest.m_iHeight);
	}
}

void Bi_Linear_Ref_1(Image oSource, Image oDest)
{//线建立一个参考程序，用错开半个相位法
//尚未对齐Opencv数据,尚待测试
	int i, x, y;
	float x_s_f, y_s_f, //目标x,y对应的源坐标
		w0, w1, w2, w3; //四组权重  w1 = 1-w0   w3 = 1-w2
	int x_s_0, x_s_1,   //目标x坐标对应的左右点坐标
		y_s_0, y_s_1;   //目标y坐标对应的上下坐标
	int w_1 = oSource.m_iWidth - 1,             //w-1
		h_1 = oSource.m_iHeight - 1;            //h-1

	if (oSource.m_iChannel_Count != oDest.m_iChannel_Count)
	{
		printf("Error in Resize_Bi_Linear\n");
		return;
	}

	//两组系数用于计算对应原位置
	float f_y = (float)oSource.m_iHeight / oDest.m_iHeight;
	float f_x = (float)oSource.m_iWidth / oDest.m_iWidth;

	unsigned char* pDest_Cur = oDest.m_pChannel[0];
	for (i = 0; i < oDest.m_iChannel_Count; i++)
	{
		for (y = 0; y < oDest.m_iHeight; y++)
		{
			y_s_f = (y + 0.5f) * f_y - 0.5f;
			//y_s_f = y * f_y;

			y_s_0 = (int)y_s_f;
			y_s_1 = Min(y_s_0 + 1, h_1);
			w2 = (y_s_1 - y_s_f), w3 = 1.f - w2;
			unsigned char* pCur_Line = &oSource.m_pChannel[i][y_s_0 * oSource.m_iWidth];
			unsigned char* pNext_Line = &oSource.m_pChannel[i][y_s_1 * oSource.m_iWidth];
			//printf("%d %d\n",y_s_0,y_s_1);
			for (x = 0; x < oDest.m_iWidth; x++, pDest_Cur++)
			{
				x_s_f = (x + 0.5f) * f_x - 0.5f;
				//x_s_f = x  * f_x;
				x_s_0 = (int)x_s_f;
				x_s_1 = Min(x_s_0 + 1, w_1);
				w0 = (x_s_1 - x_s_f), w1 = 1.f - w0;
				float fValue_0 = w0 * pCur_Line[x_s_0] + w1 * pCur_Line[x_s_1];
				float fValue_1 = w0 * pNext_Line[x_s_0] + w1 * pNext_Line[x_s_1];
				*pDest_Cur = (unsigned char)(w2 * fValue_0 + w3 * fValue_1);
			}
		}
	}

	/*bSave_Image("c:\\tmp\\1.bmp", oDest);
	Compare_Image((char*)"C:\\tmp\\2.bmp", (char*)"C:\\tmp\\1.bmp");*/
	return;
}

template<typename _T>static void Sep_Filter_2D_Line(_T A[], int w, int h, _T Kernel[], int iKernel_Size, _T B[])
{
	_T* pLine;
	int x, y, x1, i, w_1 = w - 1,   //w-1
		h_1 = h - 1;                //h-1
	int iPadding = iKernel_Size / 2;
	int x_End_No_Padding = w - iPadding;

	_T fTotal;
	//Disp(A, h, w);
	for (y = 0; y < h; y++)
	{//从上到下，逐条线搞
		pLine = &A[y * w];
		//先搞Padding
		for (x = 0; x < iPadding && x < w; x++)
		{
			for (fTotal = 0, i = 0, x1 = x - iPadding; i < iKernel_Size; x1++, i++)
			{
				if (x1 < 0)
				{
					if (-x1 >= w)
						fTotal += Kernel[i] * pLine[w_1];
					else
						fTotal += Kernel[i] * pLine[-x1];
				}
				else if (x1 >= w)
				{
					if (x - (x1 - x)<0)
						fTotal += Kernel[i] * pLine[0];
					else
						fTotal += Kernel[i] * pLine[w_1 - x1];
				}
				else
					fTotal += Kernel[i] * pLine[x1];
			}
			B[x * h + y] = fTotal;
		}

		//此处无Padding, 适合快速推进
		for (; x < x_End_No_Padding; x++)
		{
			for (fTotal = 0, x1 = x - iPadding, i = 0; i < iKernel_Size; i++, x1++)
				fTotal += Kernel[i] * pLine[x1];
			B[x * h + y] = fTotal;
		}

		for (; x < w; x++)
		{
			for (fTotal = 0, i = 0, x1 = x - iPadding; i < iKernel_Size; x1++, i++)
			{
				if (x1 < 0)
				{
					if (-x1 >= w)
						fTotal += Kernel[i] * pLine[w_1];
					else
						fTotal += Kernel[i] * pLine[-x1];
				}
				else if (x1 >= w)
				{
					int iDup_Pos = w_1 - (x1 - w_1);
					if (iDup_Pos < 0)
						fTotal += Kernel[i] * pLine[0];
					else
						fTotal += Kernel[i] * pLine[iDup_Pos];
				}
				else
					fTotal += Kernel[i] * pLine[x1];
			}
			B[x * h + y] = fTotal;
		}
		//printf("Here");
	}
	return;
}

template<typename _T>void Sep_Filter_2D(_T A[], int w, int h, _T Ker_x[], _T Ker_y[], int iKernel_Size, _T B[])
{//行列分离卷积，这个函数要求源与目的必须同样数据类型
//为安全函数，源与目标可以相同
	_T* pMid = (_T*)pMalloc(w * h * sizeof(_T));

	//Disp(A, h, w);
	Sep_Filter_2D_Line(A, w, h, Ker_x, iKernel_Size, pMid);
	//Disp(pMid, w,h);

	//Matrix_Transpose(pMid, w, h, pMid);
	//Disp(pMid, h,w);

	//出来以后，pMid 是一个宽为h, 高为w的转置矩阵，继续行推进即可
	Sep_Filter_2D_Line(pMid, h, w, Ker_y, iKernel_Size, B);
	//Disp(B, h, w);

	Free(pMid);
	return;
}

template<typename Source_Type, typename Dest_Type, typename Kernel_Typ>static void Sep_Filter_2D_Line
(Source_Type A[], int w, int h, Kernel_Typ Kernel[], int iKernel_Size, Dest_Type B[])
{
	Source_Type* pLine;
	int x, y, x1, i, w_1 = w - 1,   //w-1
		h_1 = h - 1;                //h-1
	int iPadding = iKernel_Size / 2;
	int x_End_No_Padding = w - iPadding;

	Dest_Type fTotal;
	for (y = 0; y < h; y++)
	{//从上到下，逐条线搞
		pLine = &A[y * w];

		//先搞Padding
		for (x = 0; x < iPadding && x < w; x++)
		{
			for (fTotal = 0, i = 0, x1 = x - iPadding; i < iKernel_Size; x1++, i++)
			{
				if (x1 < 0)
				{
					if (-x1 >= w)
						fTotal += Kernel[i] * pLine[w_1];
					else
						fTotal += Kernel[i] * pLine[-x1];
				}
				else if (x1 >= w)
				{
					if (-x < 0)
						fTotal += Kernel[i] * pLine[0];
					else
						fTotal += Kernel[i] * pLine[w_1 - x1];
				}
				else
					fTotal += Kernel[i] * pLine[x1];
			}
			B[x * h + y] = fTotal;
		}

		//此处无Padding, 适合快速推进
		for (; x < x_End_No_Padding; x++)
		{
			for (fTotal = 0, x1 = x - iPadding, i = 0; i < iKernel_Size; i++, x1++)
				fTotal += Kernel[i] * pLine[x1];
			B[x * h + y] = fTotal;
		}

		for (; x < w; x++)
		{
			for (fTotal = 0, i = 0, x1 = x - iPadding; i < iKernel_Size; x1++, i++)
			{
				if (x1 < 0)
				{
					if (-x1 >= w)
						fTotal += Kernel[i] * pLine[w_1];
					else
						fTotal += Kernel[i] * pLine[-x1];
				}
				else if (x1 >= w)
				{
					int iDup_Pos = w_1 - (x1 - w_1);
					if (iDup_Pos < 0)
						fTotal += Kernel[i] * pLine[0];
					else
						fTotal += Kernel[i] * pLine[iDup_Pos];
				}
				else
					fTotal += Kernel[i] * pLine[x1];
			}
			B[x * h + y] = fTotal;
		}
		//printf("Here");
	}
	return;
}



void Pix_Bi_Linear(Image::Part_1 oImage, float x, float y, unsigned char Pix[3], Border_Type iBorder_Type)
{
	short x1 = (short)floor(x),
		y1 = (short)floor(y);
	//h_1 = oImage.m_iHeight - 1,
	//w_1 = oImage.m_iWidth - 1;
	unsigned char A, B, C, D;
	int iCur_Line_Pos, iNext_Line_Pos;
	short xl_Pos, xr_Pos;
	//int iSize = oImage.m_iWidth * oImage.m_iHeight;

	if (iBorder_Type == BORDER_CONSTANT)
	{//单领出来做
		iCur_Line_Pos = iNext_Line_Pos = -1;
		int y2 = y1;
		if (y2 >= 0 && y2 < oImage.m_iHeight)
			iCur_Line_Pos = y2 * oImage.m_iWidth;
		y2++;
		if (y2 >= 0 && y2 < oImage.m_iHeight)
			iNext_Line_Pos = y2 * oImage.m_iWidth;

		xl_Pos = xr_Pos = -1;
		if (x1 >= 0 && x1 < oImage.m_iHeight)
			xl_Pos = x1;
		x1++;
		if (x1 >= 0 && x1 < oImage.m_iHeight)
			xr_Pos = x1;
	}
	else
	{
		iCur_Line_Pos = iGet_Border_y(y1, oImage.m_iHeight, iBorder_Type) * oImage.m_iWidth;
		iNext_Line_Pos = iGet_Border_y(y1 + 1, oImage.m_iHeight, iBorder_Type) * oImage.m_iWidth;
		xl_Pos = iGet_Border_x(x1, oImage.m_iWidth, iBorder_Type);
		xr_Pos = iGet_Border_x(x1 + 1, oImage.m_iWidth, iBorder_Type);
	}

	for (int i = 0; i < 3; i++)
	{
		if (oImage.m_pChannel[i])
		{
			if (iCur_Line_Pos >= 0)
			{
				A = xl_Pos >= 0 ? oImage.m_pChannel[i][iCur_Line_Pos] : 0;
				B = xr_Pos >= 0 ? oImage.m_pChannel[i][iCur_Line_Pos + 1] : 0;
			}
			else
				A = B = 0;
			if (iNext_Line_Pos >= 0)
			{
				C = xl_Pos >= 0 ? oImage.m_pChannel[i][iNext_Line_Pos] : 0;
				D = xr_Pos >= 0 ? oImage.m_pChannel[i][iNext_Line_Pos] : 0;
			}
			else
				C = D = 0;

			float fValue_0, fValue_1;
			{
				float w0 = (xr_Pos - x), w1 = 1.f - w0;
				fValue_0 = w0 * A + w1 * B;
				fValue_1 = w0 * C + w1 * D;
			}

			{
				float w3 = y - y1, w2 = 1.f - w3;
				Pix[i] = (unsigned char)(w2 * fValue_0 + w3 * fValue_1 + 0.5);
			}
		}
	}
	return;
}
template<typename Source_Type, typename Dest_Type, typename Kernel_Type>void Sep_Filter_2D_1
(Source_Type A[], int w, int h, Kernel_Type Ker_x[], Kernel_Type Ker_y[], int iKernel_Size, Dest_Type B[])
{
	Dest_Type* pMid = (Dest_Type*)pMalloc(w * h * sizeof(Dest_Type));

	Sep_Filter_2D_Line(A, w, h, Ker_x, iKernel_Size, pMid);
	//Disp(pMid, w,h);

	//Matrix_Transpose(pMid, w, h, pMid);
	//Disp(pMid, 1,w);

	//出来以后，pMid 是一个宽为h, 高为w的转置矩阵，继续行推进即可
	Sep_Filter_2D_Line(pMid, h, w, Ker_y, iKernel_Size, B);
	//Disp(B, 1, w);

	Free(pMid);
}

void Pyr_Up_Ref(Image oSource, Image oDest)
{//已经完全对齐opencv
	int i, x, y;
	int bHas_Remain_x = oDest.m_iWidth & 1,
		bHas_Remain_y;	//= oDest.m_iHeight & 1;

	if (oDest.m_iHeight > oSource.m_iHeight * 2)
		bHas_Remain_y = 1;
	else
		bHas_Remain_y = 0;

	//此处要分配中间结果空间
	short* pMid = (short*)pMalloc(oDest.m_iWidth * oSource.m_iHeight * sizeof(short));
	for (i = 0; i < oSource.m_iChannel_Count; i++)
	{
		for (y = 0; y < oSource.m_iHeight; y++)
		{
			int iPos_m = y * oDest.m_iWidth,
				iPos_s = y * oSource.m_iWidth;

			//先搞左边两点
			pMid[iPos_m] = oSource.m_pChannel[i][iPos_s] * 6 + oSource.m_pChannel[i][iPos_s + 1] * 2;
			pMid[iPos_m + 1] = (oSource.m_pChannel[i][iPos_s] + oSource.m_pChannel[i][iPos_s + 1]) * 4;

			//搞中间
			for (x = 1; x < oSource.m_iWidth - 1; x++)
			{
				pMid[iPos_m + (x << 1)] = oSource.m_pChannel[i][iPos_s + x - 1] + oSource.m_pChannel[i][iPos_s + x] * 6 + oSource.m_pChannel[i][iPos_s + x + 1];
				pMid[iPos_m + (x << 1) + 1] = (oSource.m_pChannel[i][iPos_s + x] + oSource.m_pChannel[i][iPos_s + x + 1]) * 4;
			}

			//搞右两点
			pMid[iPos_m + (x << 1)] = oSource.m_pChannel[i][iPos_s + x - 1] + oSource.m_pChannel[i][iPos_s + x] * 7;
			pMid[iPos_m + (x << 1) + 1] = oSource.m_pChannel[i][iPos_s + x] * 8;

			//搞余数
			if (bHas_Remain_x)
			{
				pMid[iPos_m + (x << 1) + 2] = pMid[iPos_m + (x << 1) + 1];
			}
		}

		//Disp(pMid, oSource.m_iHeight, oDest.m_iWidth, "Mid");

		int y0 = -1, y1 = 0, y2 = 1;
		//int h = std::min(oSource.m_iHeight * 2, oDest.m_iHeight) / 2;
		for (y = 0; y < oSource.m_iHeight; y++, y0++, y1++, y2++)
		{
			short* r0 = &pMid[iGet_Border_y(y0, oSource.m_iHeight, BORDER_REFLECT101) * oDest.m_iWidth];
			short* r1 = &pMid[iGet_Border_y(y1, oSource.m_iHeight, BORDER_REFLECT) * oDest.m_iWidth];
			short* r2 = &pMid[iGet_Border_y(y2, oSource.m_iHeight, BORDER_REFLECT) * oDest.m_iWidth];
			/*if (y == oSource.m_iHeight-1)
				printf("%d %d %d\n",r0[1163],r1[1163],r2[1163]);*/

			for (x = 0; x < oDest.m_iWidth; x++)
			{
				/*if (y == oSource.m_iHeight - 1 && x == 1416)
					printf("here");*/

				//d0行 = （r0 + r1*6 + r2 + 32)>>6
				oDest.m_pChannel[i][(y * 2) * oDest.m_iWidth + x] = (r0[x] + r1[x] * 6 + r2[x] + 32) >> 6;
				
				if(y*2+1<oDest.m_iHeight)
					//d1行 =   [(r1 + r2)*4 + 32]>>6
					oDest.m_pChannel[i][((y * 2) + 1) * oDest.m_iWidth + x] = ((r1[x] + r2[x]) * 4 + 32) >> 6;
			}
		}

		//如果目标高为基数，就要抄一行
		if (bHas_Remain_y)
		{
			unsigned char* r0 = &oDest.m_pChannel[i][(oDest.m_iHeight - 3) * oDest.m_iWidth];
			unsigned char* r1 = &oDest.m_pChannel[i][(oDest.m_iHeight - 1) * oDest.m_iWidth];
			memcpy(r1, r0, oDest.m_iWidth);
			//printf("Here");
		}
		//Disp(oDest.m_pChannel[i], oDest.m_iHeight, oDest.m_iWidth, "Dest");
	}
	if (pMid)
		Free(pMid);
	return;
}

void Pyr_Down_Ref(Image oSource, Image oDest, Border_Type iBorder_Type)
{//注意，边界类型用 BORDER_REFLECT_101
//已经完全对准opencv
	int i, x, y;
	//此处要分配中间结果空间
	unsigned short* pMid = (unsigned short*)pMalloc(oDest.m_iWidth * oSource.m_iHeight * sizeof(unsigned short));
	//先算水平插值
	for (i = 0; i < oSource.m_iChannel_Count; i++)
	{
		//Disp(oSource.m_pChannel[0], 1, 3);
		for (y = 0; y < oSource.m_iHeight; y++)
		{
			int iPos_m = y * oDest.m_iWidth,
				iPos_s = y * oSource.m_iWidth;

			for (x = 0; x < oDest.m_iWidth; x++)
			{
				int x1 = x * 2;
				pMid[iPos_m + x] = oSource.m_pChannel[i][iPos_s + x1] * 6 +  //C*6
					(oSource.m_pChannel[i][iPos_s + iGet_Border_x(x1 - 1, oSource.m_iWidth, iBorder_Type)] +            //B
						oSource.m_pChannel[i][iPos_s + iGet_Border_x(x1 + 1, oSource.m_iWidth, iBorder_Type)]) * 4 +     //D
					oSource.m_pChannel[i][iPos_s + iGet_Border_x(x1 - 2, oSource.m_iWidth, iBorder_Type)] +             //A
					oSource.m_pChannel[i][iPos_s + iGet_Border_x(x1 + 2, oSource.m_iWidth, iBorder_Type)];               //E
			}
		}
		//Disp(pMid, oSource.m_iHeight, oDest.m_iWidth, "Mid");

		//y方向上的卷积
		unsigned short row_ID[5];
		int row_Pos[5];
		for (int j = 0; j < 3; j++)
		{
			row_ID[j] = iGet_Border_y(j - 2, oSource.m_iHeight, iBorder_Type);
			row_Pos[j] = row_ID[j] * oDest.m_iWidth;
		}

		int iPos_d = 0;
		for (y = 0; y < oDest.m_iHeight; y++)
		{
			int y1 = y * 2;
			row_Pos[3] = iGet_Border_y(y1 + 1, oSource.m_iHeight, iBorder_Type) * oDest.m_iWidth;
			row_Pos[4] = iGet_Border_y(y1 + 2, oSource.m_iHeight, iBorder_Type) * oDest.m_iWidth;
			for (x = 0; x < oDest.m_iWidth; x++, iPos_d++)
			{
				unsigned short iValue;
				iValue = pMid[row_Pos[2] + x] * 6 +
					(pMid[row_Pos[1] + x] + pMid[row_Pos[3] + x]) * 4 +
					pMid[row_Pos[0] + x] + pMid[row_Pos[4] + x];
				oDest.m_pChannel[i][iPos_d] = (iValue + 128) >> 8;
			}

			//平移两行
			row_Pos[0] = row_Pos[2];
			row_Pos[1] = row_Pos[3];
			row_Pos[2] = row_Pos[4];
		}
	}

	//Disp(oDest.m_pChannel[0], oDest.m_iHeight, oDest.m_iWidth, "Dest");
	Free(pMid);
	return;
}

template<typename _T>void Coordinate_2_Screen(_T x, _T y, int* px_Screen, int* py_Screen, int iWidth, int iHeight)
{//直角坐标系坐标转换为屏幕坐标
	int iWidth_Half = iWidth >> 1,
		iHeight_Half = iHeight >> 1;
	if (px_Screen)
		*px_Screen = (int)(x + iWidth_Half);
	if (py_Screen)
		*py_Screen = (int)(iHeight_Half - y);
	return;
}

/********************小试贝塞尔曲线*****************/
template<typename _T>void Lerp(_T x0, _T y0, _T x1, _T y1, _T t, _T* px, _T* py)
{
	*px = x0 + t * (x1 - x0);
	*py = y0 + t * (y1 - y0);
}
template<typename _T>void Lerp(_T A[], _T B[], _T t, _T C[])
{//Lerp函数，A点与B之间，取比例t， 0<t<1，返回中间点C
	Lerp(A[0], A[1], B[0], B[1], t, &C[0], &C[1]);
	return;
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
					if (oImage.m_pChannel[2])
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

//贝塞尔曲线例子
//void Bezier_Test()
//{
//	typedef float _T;
//	Image oImage;
//	int x, y;
//	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
//	Set_Color(oImage);
//
//	_T A[] = { 100,100 },
//		B[] = { 200,300 },
//		C[] = { 400,100 }, \
//		D[2], E[2], F[2];
//
//	for (_T t = 0; t < 1; t += 0.01f)
//	{
//		Lerp(A, B, t, D);
//		Lerp(B, C, t, E);
//		Lerp(D, E, t, F);	//F才是要求的点
//		//Disp<_T>(F, 1, 2, "Point");
//		Rect_2_Screen_Coordinate(F[0], F[1], &x, &y, 1000, 1000);
//		Draw_Point(oImage, x, y, 1);
//	}
//	Rect_2_Screen_Coordinate(A[0], A[1], &x, &y, 1000, 1000);
//	Draw_Point(oImage, x, y);
//	Rect_2_Screen_Coordinate(B[0], B[1], &x, &y, 1000, 1000);
//	Draw_Point(oImage, x, y);
//	Rect_2_Screen_Coordinate(C[0], C[1], &x, &y, 1000, 1000);
//	Draw_Point(oImage, x, y);
//	bSave_Image("c:\\tmp\\1.bmp", oImage);
//}
/********************小试贝塞尔曲线*****************/

/*********************二值化*******************************/
static void Get_Hist_256(Image oImage, int Hist[256])
{//只搞灰度图，求个直方图，毫无营养
	int iSize = oImage.m_iWidth * oImage.m_iHeight;
	memset(Hist, 0, 256 * sizeof(int));
	unsigned char* pChannel_Cur = oImage.m_pChannel[0],
		* pChannel_End = pChannel_Cur + iSize;
	//没啥卵用，debug 下快一点
	while (pChannel_Cur < pChannel_End)
		Hist[*(pChannel_Cur++)]++;
	return;
}

static void Smooth_Hist(int Source[], int Dest[], int iCount, int r)
{//r为窗口半径，其实就是取窗口的平均，没啥营养
	int d = 2 * r + 1;
	if (iCount >= d)
	{	//不优不足以平民愤
		int i, iLeft = Min(r, iCount);
		for (i = 0; i < iLeft; i++)
		{
			//以下和传统的窗口有点不一样
			int iIdx_min = Max(0, i - r);
			int iIdx_max = Min(255, i + r);
			int iSmooth = 0;
			for (int iIdx = iIdx_min; iIdx <= iIdx_max; ++iIdx)
				iSmooth += Source[iIdx];
			Dest[i] = iSmooth / (iIdx_max - iIdx_min + 1);
		}
		int iSum = 0;
		for (int i = 0; i < d; i++)
			iSum += Source[i];
		Dest[r] = iSum;
		int iRight = iCount - r - 1;
		for (i = r; i < iRight; i++)
		{
			Dest[i + 1] = Dest[i] - Source[i - r] + Source[i + r + 1];
			Dest[i] /= d;
		}
		Dest[i] /= d;
		//for (i = r; i <= iRight; i++)
			//Dest[i] /= d;
		for (; i < iCount; i++)
		{
			//以下和传统的窗口有点不一样
			int iIdx_min = Max(0, i - r);
			int iIdx_max = Min(255, i + r);
			int iSmooth = 0;
			for (int iIdx = iIdx_min; iIdx <= iIdx_max; ++iIdx)
				iSmooth += Source[iIdx];
			Dest[i] = iSmooth / (iIdx_max - iIdx_min + 1);
		}
	}
	else
		for (int i = 0; i < iCount; i++)
		{
			//以下和传统的窗口有点不一样
			int iIdx_min = Max(0, i - r);
			int iIdx_max = Min(255, i + r);
			int iSmooth = 0;
			for (int iIdx = iIdx_min; iIdx <= iIdx_max; ++iIdx)
				iSmooth += Source[iIdx];
			Dest[i] = iSmooth / (iIdx_max - iIdx_min + 1);
		}
	return;
}

static void Grad_Hist(int Hist[], int Grad[], int iCount)
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

void Binerize(Image oSource, Image oDest)
{
	int Hist[256], Hist_Smooth[256], Hist_Grad[256];
	//先来个直方图
	Get_Hist_256(oSource, Hist);

	unsigned long long tStart = iGet_Tick_Count();
	Smooth_Hist(Hist, Hist_Smooth, 256, 1);
	//Disp(Hist_Smooth, 1, 256, "Hist_Smooth");

	//给平滑直方图算个梯度
	Grad_Hist(Hist_Smooth, Hist_Grad, 256);
	//Disp(Hist_Grad, 1, 256, "Hist_Grad");

	//虽然实际上的计算非常复杂，但是大致的算法还是很清晰的
	//1，棋盘块是黑块，从最黑开始，最初该值像素很少，到了黑块附近，像素躲起来，一直爬到最高
	//2，从最高开始，往后就是越来越亮，所以像素数量往下走，离开黑块的趋势就是梯度下拨
	//3，向着白色方块去，会形成另一个高峰，越来越多
	//	爬到最高处，同样梯度为0
	//4，于是，两个峰顶之间是山谷，最优解就在山谷导数=0处

	int iMax_Pix = oSource.m_iWidth * oSource.m_iHeight;
	int iMax_Pix_1 = iMax_Pix / 100;
	int Max_Pos[20] = {};

	//感觉也没啥卵用，只是挑出些概率高的
	unsigned iMaximun_Count = 0;
	for (int i = 256 - 2; (i > 2) && (iMaximun_Count < 20); --i)
	{
		if ((Hist_Grad[i - 1] < 0) && (Hist_Grad[i] > 0))
		{
			//窗口和
			int iSum_Around_Max = Hist_Smooth[i - 1] + Hist_Smooth[i] + Hist_Smooth[i + 1];
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
	}
	else if (iMaximun_Count == 1)
		iThresh = Max_Pos[0] / 2;
	else if (iMaximun_Count == 2)
		iThresh = (Max_Pos[0] + Max_Pos[1]) / 2;
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
			if (Max_Pos[n] < iIdx_Acc_Sum)
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
		int iDist2 = (iBright_Max - Max_Pos[iIdx_BG_Max]) / 2;
		iThresh = iBright_Max - iDist2;
	}

	if (iThresh > 0)
	{
		int iSize = oSource.m_iWidth * oSource.m_iHeight;
		for (int i = 0; i < iSize; i++)
			oDest.m_pChannel[0][i] = oSource.m_pChannel[0][i] >= iThresh ? 255 : 0;
	}
	return;
}

void Remove_Orphan(Image oImage)
{//二值图去除孤点
	int w1 = oImage.m_iWidth - 1,
		h1 = oImage.m_iHeight - 1;
	typedef struct Stencil {
		unsigned char m_iBit_0 : 1;
		unsigned char m_iBit_1 : 1;
		unsigned char m_iBit_2 : 1;
		unsigned char m_iBit_3 : 1;
		unsigned char m_iBit_4 : 1;
		unsigned char m_iBit_5 : 1;
		unsigned char m_iBit_6 : 1;
		unsigned char m_iBit_7 : 1;
		unsigned char m_iBit_8 : 1;
	}Stencil;

	union {
		Stencil oStencil;
		int iStencil;
	};

	iStencil = 0;
	oStencil.m_iBit_4 = 1;
	//int iOrphan_White = iStencil;
	int iOrphan_Mask = 0xBA;			//模板为	*
	//											  * * *
	//												*
												  
	int iOrphan_Black = 0xAA;			//			1
	//											  1 0 1
	//												1

	iStencil = 0;
	int iPos = oImage.m_iWidth + 1;;
	const int iDelta_0 = -oImage.m_iWidth - 1,
		iDelta_1 = -oImage.m_iWidth,
		iDelta_2 = -oImage.m_iWidth + 1,
		iDelta_6 = oImage.m_iWidth - 1,
		iDelta_7 = oImage.m_iWidth,
		iDelta_8 = oImage.m_iWidth + 1;

	unsigned char* pCur = oImage.m_pChannel[0];
	int iCount = 0;
	for (int y = 1; y < h1; y++)
	{
		iPos = y * oImage.m_iWidth + 1;
		for (int x = 1; x < w1; x++, iPos++)
		{
			oStencil.m_iBit_0 = pCur[iPos + iDelta_0];
			oStencil.m_iBit_1 = pCur[iPos + iDelta_1];
			oStencil.m_iBit_2 = pCur[iPos + iDelta_2];
			oStencil.m_iBit_3 = pCur[iPos - 1];
			oStencil.m_iBit_4 = pCur[iPos];
			oStencil.m_iBit_5 = pCur[iPos + 1];
			oStencil.m_iBit_6 = pCur[iPos + iDelta_6];
			oStencil.m_iBit_7 = pCur[iPos + iDelta_7];
			oStencil.m_iBit_8 = pCur[iPos + iDelta_8];

			/*if (iStencil == iOrphan_Black)
				pCur[iPos] = 0xFF, iCount++;
			else if ((iStencil & iOrphan_White_Mask) == iOrphan_White)
				pCur[iPos] = 0, iCount++;*/
			if ( (iStencil & iOrphan_Mask) == iOrphan_Black)
				pCur[iPos] = 0xFF;
		}
	}
	return;
}
/*********************二值化*******************************/

template<typename _T>static void Find_Max_Dist_Pair(_T Point[][2], int iCount, float eps, Start_End* poRight_Slice, Start_End* poSlice, int* pbLE_eps)
{//近似寻找最远两点,暂时只管闭合
	Start_End oSlice, oRight_Slice = { 0,0 };	//暂时未知意义
	const int iIter_Count = 3;
	int i, j, bLE_eps = 1, iPos = 0;
	//注意，以下m_iStart并不是在Point的位置，而是旋转中j的值
	//是一个相对于原Pos的位置
	//oRight_Slice.m_iStart = 0;
	for (i = 0; i < iIter_Count; i++)
	{
		iPos = (iPos + oRight_Slice.m_iStart) % iCount;
		_T Start_Point[2] = { Point[iPos][0],Point[iPos][1] };
		_T fMax = 0;
		iPos = (iPos + 1) % iCount;
		for (j = 1; j < iCount; j++, iPos = (iPos + 1) % iCount)
		{//绕场一周, 找出与Start_Point距离最远点
			_T fDist, dx = Point[iPos][0] - Start_Point[0],	//x方向距离，y方向距离，当然理解为delta也可
				dy = Point[iPos][1] - Start_Point[1];
			fDist = dx * dx + dy * dy;
			if (fDist > fMax)
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
		oRight_Slice.m_iStart = (oRight_Slice.m_iStart + oSlice.m_iStart) % iCount;
		oSlice.m_iEnd = oRight_Slice.m_iStart;
		*poSlice = oSlice;
		*poRight_Slice = oRight_Slice;
	}
	*pbLE_eps = bLE_eps;
	return;
}

//将一个不规则图形尽量拉直approxPolyDP
template<typename _T> void approxPolyDP(_T Source_Point[][2], int iSource_Count,
	_T Dest_Point[][2], int* piDest_Count, float eps)
{//这段程序找不到任何破绽，优无可优，暂时拿来照用
	Start_End oSlice, oRight_Slice;
	int bLE_eps;
	if (Source_Point == Dest_Point)
	{
		printf("Source cannot be dest in approxPolyDP\n");
		exit(0);
	}
	Find_Max_Dist_Pair(Source_Point, iSource_Count, 1.f, &oRight_Slice, &oSlice, &bLE_eps);

	const int iStack_Size = 128;
	Start_End Stack_Buffer[iStack_Size], * Stack;
	if (iSource_Count > iStack_Size)
		Stack = (Start_End*)pMalloc(iSource_Count * sizeof(Start_End));
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
				if (dist > max_dist)
				{
					max_dist = dist;
					oRight_Slice.m_iStart = (pos + iSource_Count - 1) % iSource_Count;
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
		if (bLE_eps)
		{
			//WRITE_PT(start_pt);
			Dest_Point[iDest_Count][0] = start_pt[0];
			Dest_Point[iDest_Count][1] = start_pt[1];
			iDest_Count++;
		}
		else
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

	for (int i = 0; i < iSource_Count && iDest_Count > 2; i++)
	{
		/*if (i == 2)
		printf("here");*/
		double dx, dy, dist, successive_inner_product;
		end_pt[0] = Dest_Point[pos][0];
		end_pt[1] = Dest_Point[pos][1];
		pos = (pos + 1) % iSource_Count;
		dx = end_pt[0] - start_pt[0];
		dy = end_pt[1] - start_pt[1];
		dist = fabs((pt[0] - start_pt[0]) * dy - (pt[1] - start_pt[1]) * dx);
		successive_inner_product = (pt[0] - start_pt[0]) * (end_pt[0] - pt[0]) +
			(pt[1] - start_pt[1]) * (end_pt[1] - pt[1]);
		if (dist * dist <= 0.5 * eps * (dx * dx + dy * dy) && dx != 0 && dy != 0 &&
			successive_inner_product >= 0)
		{
			iDest_Count--;
			Dest_Point[wpos][0] = start_pt[0] = end_pt[0];
			Dest_Point[wpos][1] = start_pt[1] = end_pt[1];
			if (++wpos >= iSource_Count)
				wpos = 0;
			pt[0] = Dest_Point[pos][0];
			pt[1] = Dest_Point[pos][1];
			pos = (pos + 1) % iSource_Count;
			i++;
			continue;
		}
		Dest_Point[wpos][0] = start_pt[0] = pt[0];
		Dest_Point[wpos][1] = start_pt[1] = pt[1];
		if (++wpos >= iSource_Count)
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