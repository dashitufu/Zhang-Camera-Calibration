// Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "windows.h"
#include "SDL_Lib.h"
#include "Common.h"
#include "Image.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/SDL_Lib.lib")
#else
#pragma comment(lib, "../x64/Release/SDL_Lib.lib")
#endif

int main()
{
    bInit_Env_CPU();
    Image oImage;
    bLoad_Image("c:\\tmp\\Scene_Cut_A.bmp", &oImage);

    Create_Win(0, 0, 1280, 720);
    Paint_Win(oImage);
    //for (int i = 0; i < (1<<16); i++)
    //{
    //    //Set_Color(oImage, i >> 16, i >> 8, i);
    //    Paint_Win(oImage);
    //    //Sleep(1);
    //}
    Sleep(INFINITE);
    Close_Win();

    Free_Image(&oImage);
    Free_Env_CPU();
    _CrtDumpMemoryLeaks();
    return 0;


}