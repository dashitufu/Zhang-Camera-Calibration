#include "stdio.h"
#include "string.h"
#include <crtdbg.h>
#include "windows.h"
#include <mutex>

#include "Common.h"
#include "Common_eps32.h"
#include "d:\Samp\Rich_CV\SDL_Lib\SDL_Lib\SDL_Lib.h"

#ifdef _DEBUG
#pragma comment(lib, "D:/Samp/Rich_CV/SDL_Lib/x64/Debug/SDL_Lib.lib")
#else
#pragma comment(lib, "D:/Samp/Rich_CV/SDL_Lib/x64/Release/SDL_Lib.lib")
#endif

using namespace std;
int iSet_Cam_Resolution(const char IP[], int iPort, int w, int h)
{
	int iSocket = iInit_Connection((char*)IP, iPort);
	int iResult = iCmd_Set_Cam_Frame_Size_Client(iSocket, w, h);
	if (iResult)
		printf("Set Cam resolution successfully:%dx%d\n", w, h);
	else
		printf("Fail to set Cam resolution:%dx%d\n", w, h);

	Close_Socket(iSocket);
	return iResult;
}

int iSet_Cam_Resolution(char Param[], const char IP[], int iPort)
{//参数例子 800x600
	int w, h;
	sscanf(Param, "%dx%d", &w, &h);
	return iSet_Cam_Resolution(IP, iPort, w, h);
}
void Rotate_Cam(int iDir, float fDelta, const char IP[], int iPort)
{
	int iSocket = iInit_Connection((char*)IP, iPort);
	iCmd_Rotate_Cam_Client(iSocket, iDir, fDelta);
	Close_Socket(iSocket);
}
void Rotate_Cam(char Dir[], char Delta[], const char IP[], int iPort)
{//旋转镜头, 0: 水平， 1：垂直
	int iDir;
	if (bStricmp(Dir, (char*)"hor") == 1)
		iDir = 0;
	else
		iDir = 1;
	float fDelta = (float)atof(Delta);
	Rotate_Cam(iDir, fDelta, IP, iPort);
}
void RD(char Path[], const char IP[], int iPort)
{
	int iSocket = iInit_Connection((char*)IP, iPort);
	int iResult = iCmd_rd_Client(iSocket, Path);
	switch (iResult)
	{
	case 0:
		printf("Network error\n");
		break;;
	case -1:
		printf("Fail to delete %s\n", Path);
		break;
	default:
		printf("%s deleted successfully\n", Path);
	}
	return;
}
void MD(char Path[], const char IP[], int iPort)
{
	int iSocket = iInit_Connection((char*)IP, iPort);
	int iResult = iCmd_md_Client(iSocket, Path);
	switch (iResult)
	{
	case 0:
		printf("Network error\n");
		break;;
	case -1:
		printf("Fail to create %s\n", Path);
		break;
	default:
		printf("%s created successfully\n", Path);
	}
	return;
}
void Delete(char File[], const char IP[], int iPort)
{
	int iSocket = iInit_Connection((char*)IP, iPort);
	int iResult = iCmd_Delete_File_Client(iSocket, File);
	switch (iResult)
	{
	case 0:
		printf("Network error\n");
		break;;
	case -1:
		printf("Fail to delete %s\n", File);
		break;
	default:
		printf("%s deleted successfully\n", File);
	}
	return;
}
int iCapture(char File[], const char IP[], int iPort)
{//参数例子 c:\tmp\1.jpg
	int iSocket = iInit_Connection((char*)IP, iPort);
	unsigned char* pBuffer = NULL;
	int iSize = 0, bRet = 1;
	int iResult = iCmd_Capture_Client(iSocket, &pBuffer, &iSize);
	if (!iResult)
	{
		printf("Fail to receive data from:%s\n", IP);
		bRet = 0;
		goto END;
	}
	bRet = bSave_Raw_Data(File, pBuffer, iSize);
	if (bRet)
		printf("Save %s successfully\n", File);
	else
		printf("Fail to save %s\n", File);

END:
	Free(pBuffer);
	Close_Socket(iSocket);
	return bRet;
}

void Upload(char Source[], char Dest[], const char IP[], int iPort)
{
	int iSocket = iInit_Connection((char*)IP, iPort);
	int iResult = iCmd_Upload_File_Client(iSocket, Source, Dest);
	if (!iResult)
		printf("Fail to upload file\n");
	else
		printf("File copied successfully\n");
	Close_Socket(iSocket);
	return;
}

void Download(char Source[], char Dest[], const char IP[], int iPort)
{//下载一个文件到本地
	int iSocket = iInit_Connection((char*)IP, iPort);

	//char Path[256];
	//sprintf(Path, "/littlefs/%s", Source);
	int iResult = iCmd_Get_File_Client(iSocket, Source, Dest);
	if (!iResult)
		printf("Fail to receive data from:%s\n", IP);
	else
		printf("%s copied successfully\n", Source);

	Close_Socket(iSocket);
	return;
}

void List(char Path[], const char IP[], int iPort)
{
	/*char Path_1[256];
	sprintf(Path_1, "/littlefs/%s", Path);*/
	int iSocket = iInit_Connection((char*)IP, iPort);
	char* pBuffer = NULL;
	int iSize = 0, bRet = 1;
	int iResult = iCmd_Get_File_List_Client(iSocket, Path, &pBuffer, &iSize);
	if (!iResult)
	{
		printf("Fail to receive data from:%s\n", IP);
		bRet = 0;
		goto END;
	}

	if (iSize)
		Disp_File_List(pBuffer, iSize);
	else
		printf("服务器查无该目录文件:%s\n", Path);

END:
	if (pBuffer)
		Free(pBuffer);
	Close_Socket(iSocket);
}
static void Test_1()
{

}

static void Win_Thread(int *pbStop, Semaphore_For_Thread oS, const char IP[],
	int iPort)
{
	const int w = 800, h = 600;
	Create_Win(0, 0, w, h);

	void* pJPEG_Decoder = NULL;
	int iResult = bInit_JPEG(&pJPEG_Decoder);

	//Set Camera Size
	int iSocket = iInit_Connection((char*)IP, iPort);
	iResult = iCmd_Set_Cam_Frame_Size_Client(iSocket, w, h);
	if (!iResult)
		return;
	Close_Socket(iSocket);

	while (!(*pbStop))
	{
		Lock_Semaphore_For_Thread(&oS);
		unsigned char* pJPEG_Data = NULL;
		int iSize = 0, bRet = 1;

		iSocket = iInit_Connection((char*)IP, iPort);
		iResult = iCmd_Capture_Client(iSocket, &pJPEG_Data, &iSize);
		unsigned char* pRGB = pDecode_JPEG(pJPEG_Decoder, pJPEG_Data, iSize);
		Paint_Win_RGB(pRGB, w, h);
		if (pRGB)
			Free(pRGB);
		if (pJPEG_Data)
			Free(pJPEG_Data);
		Close_Socket(iSocket);
		Unlock_Semaphore_For_Thread(&oS);
		Sleep(30);
	}

	return;
}
void Monitor(const char IP[], int iPort)
{//
	Semaphore_For_Thread oS;
	oS.m_poLock = new mutex();

	//先其一线程打开窗口
	thread* pWin_Thread;	//iPause: 信号，0：表示不打扰；1表示通知暂停；2：表示已经暂停

	int bStop = 0;
	pWin_Thread = new thread(Win_Thread, &bStop, oS, IP, iPort);
	pWin_Thread->detach();
	delete(pWin_Thread);

	
	const float fAngle_Step = 10;
	Key iKey;
	float fAngle_Hor = 0, fAngle_Ver = 0;
	//先归零
	Rotate_Cam(9, 0, IP, iPort);
	Rotate_Cam(1, 0, IP, iPort);

	while ((iKey = iGet_Key()) != Esc)
	{
		if (iKey == Invalid_Key)
		{
			Sleep(100);
			continue;
		}

		Lock_Semaphore_For_Thread(&oS);
		//等到可以开干了
		if (iKey == Arrow_Up || iKey == Arrow_Down)
		{
			if(iKey == Arrow_Up)
				fAngle_Ver -= fAngle_Step;
			else
				fAngle_Ver += fAngle_Step;
			Rotate_Cam(1, fAngle_Ver, IP, iPort);
		}else
		{
			if (iKey == Arrow_Right)
				fAngle_Hor -= fAngle_Step;
			else
				fAngle_Hor += fAngle_Step;
			Rotate_Cam(0, fAngle_Hor, IP, iPort);
		}
		
		Unlock_Semaphore_For_Thread(&oS);
	}
	if (oMem_Mgr.m_oS.m_poLock)
		delete (mutex*)oMem_Mgr.m_oS.m_poLock;
	return;
}

int main_2(int argc, char* argv[])
{
	bInit_Env_CPU();

	//Test_1();
	//return 0;

	Init_Setting_Client oSetting;
	bLoad_Setting("D:\\Samp\\Rich_CV\\esp32\\Setting_Client.ini", &oSetting);
#define TEST_ARGC(iValid_Count)			\
	if(argc<iValid_Count)				\
	{									\
		printf("Invalid parameter\n");	\
		return 0;						\
	}
	TEST_ARGC(2);

	//干脆一个程序搞定，自己加命令
	if (bStricmp(argv[1], (char*)"Upload") == 1)
	{	//上传文件
		TEST_ARGC(4);
		Upload(argv[2], argv[3], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"Set_Cam") == 1)
	{	//设置分辨率
		TEST_ARGC(3);
		int iResult = iSet_Cam_Resolution(argv[2], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"Capture") == 1)
	{	//捕捉一张
		TEST_ARGC(3);
		int iResult = iCapture(argv[2], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"List") == 1)
	{	//列出文件
		TEST_ARGC(3);
		List(argv[2], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"Download") == 1)
	{	//x下载文件
		TEST_ARGC(4);
		Download(argv[2], argv[3], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"Delete") == 1)
	{	//删除文件
		TEST_ARGC(3);
		Delete(argv[2], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"md") == 1)
	{	//删除文件
		TEST_ARGC(3);
		MD(argv[2], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"rd") == 1)
	{	//删除文件
		TEST_ARGC(3);
		RD(argv[2], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"Rotate_Cam") == 1)
	{// rotate hor=xxx ver=xxx
		TEST_ARGC(4);
		Rotate_Cam(argv[2], argv[3], oSetting.m_IP, oSetting.m_iPort);
	}
	else if (bStricmp(argv[1], (char*)"monitor") == 1)
		Monitor(oSetting.m_IP,oSetting.m_iPort);
	else
		printf("想干嘛？\n");

	Free_Env_CPU();
#ifdef WIN32
	_CrtDumpMemoryLeaks();
#endif
	return 0;
}
