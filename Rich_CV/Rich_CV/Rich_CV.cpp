#include "stdio.h"
#include "string.h"
#include <crtdbg.h>
#include "windows.h"


#include "Common.h"
#include "Common_eps32.h"
#include "Matrix.h"

#include "d:\Samp\Rich_CV\SDL_Lib\SDL_Lib\SDL_Lib.h"
#include "../esp_dsp_pc/esp_dsp.h"

#ifdef _DEBUG
#pragma comment(lib, "D:/Samp/Rich_CV/SDL_Lib/x64/Debug/SDL_Lib.lib")
#else
#pragma comment(lib, "D:/Samp/Rich_CV/SDL_Lib/x64/Release/SDL_Lib.lib")
#endif

void Client_Test_1()
{
	int iResult, iSize;
	unsigned long long tTime_Stamp, tPre_Time = 0;;
	while (1)
	{
		int iSocket = iInit_Connection((char*)"192.168.1.7", 10001);
		//iResult = iCmd_Shake_Hand_Client(iSocket);
		//iResult = iCmd_Get_File_Client(iSocket,"1.jpg","c:\\tmp\\1.jpg");

		char* pBuffer = NULL;
		//iResult = iCmd_Set_Cam_Frame_Size_Client(iSocket, 640, 480);
		iResult = iCmd_Capture_Client(iSocket, (unsigned char**)&pBuffer, &iSize, &tTime_Stamp);
		//存盘看看
		//bSave_Raw_Data("c:\\tmp\\1.jpg", pBuffer, iSize);
		//printf("%lld\n", (tTime_Stamp - tPre_Time)/1000);
		//tPre_Time = tTime_Stamp;
		//iResult = iCmd_Get_File_List_Client(iSocket, "c:\\tmp", &pBuffer, &iSize);
		//iResult = iCmd_Get_File_List_Client(iSocket, "/littlefs", &pBuffer, &iSize);
		//Disp_File_List(pBuffer, iSize, 1);
		Free(pBuffer);

		Close_Connection(iSocket, 1);
		Free_Socket_Env();
		Sleep(1);	
	}
	return;
}

void Server_Test_1()
{
	esp32_Start_Server();
	return;
}

void  Test_1()
{
	//方法二，自己放在内存中
	unsigned char* pSource;
	int iSize;
	bLoad_Raw_Data("c:\\tmp\\3.jpg", &pSource, &iSize);
	int i = 0;
	unsigned long long tStart = iGet_Tick_Count();
	//while(i<1000)
	{
		void* pDecoder;
		int iResult = bInit_JPEG(&pDecoder);
			
		while (i < 1000)
		{
			unsigned char* pRGB = pDecode_JPEG(pDecoder, pSource, iSize);
			if (pRGB)
				Free(pRGB);
			//printf("%d\n",i);
			i++;
		}
		Free_JPEG(pDecoder);
	}
	printf("%lld\n", iGet_Tick_Count() - tStart);
	//Create_Win(0, 0, 500, 500);
	//Sleep(INFINITE);

	if (pSource)
		Free(pSource);
}
void Test_2()
{
	unsigned char* pJPEG_Data;
	int iSize, iResult;
	void* pJPEG_Decoder;

	iResult = bLoad_Raw_Data("c:\\tmp\\3.jpg", &pJPEG_Data, &iSize);
	iResult = bInit_JPEG(&pJPEG_Decoder);
	unsigned char* pBuffer = pDecode_JPEG(pJPEG_Decoder, pJPEG_Data, iSize);

	Create_Win(0, 0, 1280, 720);
	Paint_Win_RGB(pBuffer, 1920, 1080);
	Sleep(2000);
	Free_JPEG(pJPEG_Decoder);
	Close_Win();

	return;
}
void Test_3()
{
	Init_Setting_Client oSetting;
	bLoad_Setting("D:\\Samp\\Rich_CV\\esp32\\Setting_Client.ini", &oSetting);
	
	const int w = 800, h = 600;
	Create_Win(0, 0, w, h);

	void* pJPEG_Decoder = NULL;
	int iResult = bInit_JPEG(&pJPEG_Decoder);

	//Set Camera Size
	int iSocket = iInit_Connection((char*)oSetting.m_IP, oSetting.m_iPort);
	iResult = iCmd_Set_Cam_Frame_Size_Client(iSocket, w, h);
	if (!iResult)
		return;

	while (1)
	{
		unsigned char* pJPEG_Data = NULL;
		int iSize = 0, bRet = 1;

		iSocket = iInit_Connection((char*)oSetting.m_IP, oSetting.m_iPort);
		iResult = iCmd_Capture_Client(iSocket, &pJPEG_Data, &iSize); 
		unsigned char* pRGB = pDecode_JPEG(pJPEG_Decoder, pJPEG_Data, iSize);
		//Paint_Win_RGB(pRGB, w, h);
		if (pRGB)
			Free(pRGB);
		if (pJPEG_Data)
			Free(pJPEG_Data);
		Sleep(30);
	}
	
	return;
}

//void Test_4()
//{
//	const int N = 256;
//	// 注意：向量指令集通常要求内存是对齐的（通常是 16 字节对齐）
//	short* array_a = (short*)heap_caps_malloc(N * sizeof(short), MALLOC_CAP_32BIT);
//	short* array_b = (short*)heap_caps_malloc(N * sizeof(short), MALLOC_CAP_32BIT);
//
//	// 初始化数据
//	for (int i = 0; i < N; i++) {
//		array_a[i] = i % 10;
//		array_b[i] = 2;
//	}
//
//	// 使用 ESP-S3 的硬件加速执行点积 (Dot Product)
//	short result = 0;
//
//	// 底层自动调用了 S3 的 EE.MUL.S16.X2 等向量指令
//	//esp_err_t ret = dsps_dotprod_s16(array_a, array_b, &result, N, 0);
//	return;
//}

int main()
{
	bInit_Env_CPU(8000000, 128, 97);
	//printf("%d\n", iAngle_To_Signal(0));
	//Client_Test_1();
	//Server_Test_1();
	Test_3();
	Free_Env_CPU();

#ifdef WIN32
	_CrtDumpMemoryLeaks();
#endif
    return 0;
}