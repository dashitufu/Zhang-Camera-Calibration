#include <SDL3\SDL.h>
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include "Windows.h"
#include "Common.h"
#include "Image.h"

#include "pch.h"
//*****************************SDL 部分**************************/
#include "SDL_Lib.h"
#ifdef _DEBUG
#pragma comment(lib, "D:/Samp/Rich_CV/3rdparty/SDL3/Debug/SDL3-static.lib")
#else
#pragma comment(lib, "D:/Samp/Rich_CV/3rdparty/SDL3/Release/SDL3-static.lib")
#endif

#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "version.lib")     // 解决版本检测相关符号
#pragma comment(lib, "imm32.lib")       // 解决输入法相关符号
#pragma comment(lib, "Setupapi.lib")    // 解决手柄、硬件设备管理器相关符号
#pragma comment(lib, "Cfgmgr32.lib")    // 解决配置管理器相关符号
//*****************************SDL 部分**************************/

//****************************JPEG 部分**************************/
#include <wincodec.h>
#include <iostream>
#include <string>
#include <shlwapi.h>

#pragma comment(lib, "shlwapi.lib")
#pragma comment(lib, "Windowscodecs.lib")

typedef struct JPEG_Decoder {
    HRESULT hr;
    IWICImagingFactory* pFactory = nullptr;
    IWICBitmapDecoder* pDecoder = nullptr;
    IWICBitmapFrameDecode* pFrame = nullptr;
}JPEG_Decoder;

//****************************JPEG 部分**************************/
using namespace std;

//SDL 全局变量
using namespace std;
unsigned char* pImage = NULL;
int bStop = 0, bHas_Data = 0, bReady = 0;
int w, h;
Semaphore_For_Thread oS;

static void Win_Thread(int x, int y, int w1, int h1)
{//
    w = w1, h = h1;
    oS.m_poLock = new mutex();

    // 1. 初始化 SDL3 视频子系统
    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        std::cerr << "SDL 初始化失败: " << SDL_GetError() << std::endl;
        return;
    }

    // 2. 创建窗口和渲染器
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    if (!SDL_CreateWindowAndRenderer("SDL3 RGB Pixel Test", w1, h1, 0, &window, &renderer))
    {
        std::cerr << "创建窗口失败: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return;
    }

    // 3. 创建支持高频 CPU 写入的流式纹理，格式设为 RGB24
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, w1, h1);
    if (!texture) 
    {
        std::cerr << "创建纹理失败: " << SDL_GetError() << std::endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }

    //分配内存
    pImage = (unsigned char*)malloc(w1 * h1 * 3);
    memset(pImage, 0, w1 * h1 * 3);
    bReady = 1;

    bool running = true;
    SDL_Event event;

    int i = 0;
    while (!bStop)
    {
        // 处理退出事件 (按窗口右上角 X 退出)
        while (SDL_PollEvent(&event))
            if (event.type == SDL_EVENT_QUIT) 
                running = false;

        // 7. 将内存 Buffer 的数据高速同步到 GPU 纹理
        // width * 3 表示一行图像占用的字节数 (Pitch)
        if (!bHas_Data)
        {
            Sleep(10);
            continue;
        }
        SDL_UpdateTexture(texture, nullptr, pImage, w1 * 3);
        Lock_Semaphore_For_Thread(&oS);
        bHas_Data = 0;
        Unlock_Semaphore_For_Thread(&oS);

        // 8. 刷新屏幕显示
        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        // 稍作微小延迟避免 CPU 空转满载
        //SDL_Delay(10);

        //printf("i:%d\n", i++);
        Sleep(10);
    }

    if (pImage)
        free(pImage);

    // 9. 释放资源
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    delete (mutex*)oS.m_poLock;
    return;
}

extern "C" void Paint_Win_RGB(unsigned char *pRGB,int iWidth,int iHeight)
{
    int w1 = min(w, iWidth),
        h1 = min(h, iHeight);

    while (bHas_Data)
        Sleep(10);

    int iPos_s = 0, iPos_d = 0;
    for (int y = 0; y < h1; y++, iPos_s += 3 * iWidth, iPos_d += 3 * w)
        memcpy(&pImage[iPos_d], &pRGB[iPos_s], w1 * 3);

    Lock_Semaphore_For_Thread(&oS);
    bHas_Data = 1;
    Unlock_Semaphore_For_Thread(&oS);
}

extern "C" void Paint_Win(Image oImage)
{
    int iPos_d = 0, iPos_s;
    Lock_Semaphore_For_Thread(&oS);
    for (int y = 0; y < h; y++)
    {
        iPos_s = y * oImage.m_iWidth;
        for (int x = 0; x < w; x++, iPos_d += 3)
        {
            pImage[iPos_d] = oImage.m_pChannel[0][iPos_s];
            pImage[iPos_d + 1] = oImage.m_pChannel[1][iPos_s];
            pImage[iPos_d + 2] = oImage.m_pChannel[2][iPos_s++];
        }
    }
    bHas_Data = 1;
    Unlock_Semaphore_For_Thread(&oS);
}
extern "C" void Create_Win(int x, int y, int w1, int h1)
{//在(x,y)开始处建立一个窗口
    thread* pWin_Thread;
    pWin_Thread = new thread(Win_Thread, x, y, w1, h1);
    pWin_Thread->detach();
    delete(pWin_Thread);
    while (!bReady)
        Sleep(100);

    return;
}

extern "C" void Close_Win()
{
    bStop = 1;
}

/****************JPEG 部分*************************************/
// 1. 先定义转换函数（让编译器先记住它）
static std::wstring ConvertCharToWString(const char* charString) {
    if (!charString) return L"";
    int sizeNeeded = MultiByteToWideChar(CP_ACP, 0, charString, -1, NULL, 0);
    std::wstring wstrTo(sizeNeeded, 0);
    MultiByteToWideChar(CP_ACP, 0, charString, -1, &wstrTo[0], sizeNeeded);
    return wstrTo;
}

int bInit_JPEG(void **ppDecoder)
{
    JPEG_Decoder* poDecoder = new (JPEG_Decoder);
    HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
    if (FAILED(hr))
    {
        if (ppDecoder)
            *ppDecoder = NULL;
        return 0;
    }

    IWICImagingFactory* pFactory = nullptr;
    IWICBitmapDecoder* pDecoder = nullptr;
    IWICBitmapFrameDecode* pFrame = nullptr;

    hr = CoCreateInstance(
        CLSID_WICImagingFactory,
        NULL,
        CLSCTX_INPROC_SERVER,
        IID_PPV_ARGS(&pFactory)
    );
    poDecoder->pDecoder = pDecoder;
    poDecoder->pFactory = pFactory;
    poDecoder->pFrame = pFrame;
    poDecoder->hr = hr;

    if (ppDecoder)
        *ppDecoder = poDecoder;
    return 1;
}

unsigned char* pDecode_JPEG(void* pDecoder, unsigned char* pBuffer, int iSize)
{
    JPEG_Decoder* poDecoder = (JPEG_Decoder*)pDecoder;
    IStream* pStream = SHCreateMemStream(
        static_cast<const BYTE*>(pBuffer), // 你的数据指针
        static_cast<UINT>(iSize)     // 数据大小
    );
    HRESULT hr = poDecoder->hr;

    if (SUCCEEDED(hr))
    {
        hr = poDecoder->pFactory->CreateDecoderFromStream(
            pStream,
            NULL,
            WICDecodeMetadataCacheOnDemand,
            &poDecoder->pDecoder
        );
    }

    if (SUCCEEDED(hr)) {
        hr = poDecoder->pDecoder->GetFrame(0, &poDecoder->pFrame);
    }

    unsigned int width, height;
    poDecoder->pFrame->GetSize(&width, &height);

    UINT cbStride = width * 3;
    UINT cbBufferSize = cbStride * height; // 整个图片数据需要的字节数

    unsigned char* pRGB = (unsigned char*)pMalloc(cbBufferSize);

    hr = poDecoder->pFrame->CopyPixels(
        NULL,             // 复制整个区域（NULL代表全图）
        cbStride,         // 每一行像素的字节数
        cbBufferSize,     // 缓冲区总大小
        pRGB
    );

    if (pStream)
        pStream->Release();
    return pRGB;
}

void Free_JPEG(void * pDecoder)
{
    JPEG_Decoder* poDecoder =(JPEG_Decoder *)pDecoder;
    if (poDecoder->pFrame)
        poDecoder->pFrame->Release();
    if (poDecoder->pDecoder)
        poDecoder->pDecoder->Release();
    if (poDecoder->pFactory)
        poDecoder->pFactory->Release();
    // 6. 反初始化 COM 库
    CoUninitialize();

    delete(poDecoder);
}
/****************JPEG 部分*************************************/