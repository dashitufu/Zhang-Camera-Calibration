#pragma once
#include "image.h"

#define WIN32_LEAN_AND_MEAN             // 从 Windows 头文件中排除极少使用的内容

//SDL接口部分
extern "C" void Create_Win(int x, int y, int w, int h);
extern "C" void Close_Win();
extern "C" void Paint_Win(Image oImage);
extern "C" void Paint_Win_RGB(unsigned char* pRGB, int iWidth, int iHeight);

//JPET接口部分
extern "C" int bInit_JPEG(void** ppDecoder);
extern "C" unsigned char* pDecode_JPEG(void* pDecoder, unsigned char* pBuffer, int iSize);
extern "C" void Free_JPEG(void* pDecoder);