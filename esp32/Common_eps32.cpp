#include "stdio.h"
#ifndef WIN32
#include <esp_camera.h>
#include <Arduino.h>
#include <driver/mcpwm.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <sys/socket.h>
#include <unistd.h>
#include <dirent.h>
#endif // !WIN32

#include "Common.h"
#include "Common_eps32.h"

ESP32_Env oESP32_Env;

//#define ESP32
#ifdef ESP32
//矩阵乘法
#define Matrix_Multiply(A, ma, na, B, nb,C) dspm_mult_f32(A, B, C, ma, na, nb)


/*******************************sg90舵机控制**************************/
//先不搞了，还没有多余两个电机的情况
void Init_Motor_mcpwm(int iPin_1, int iPin_2 = -1, int iUnit = 0, int iTimer = 0, int iFreq = 50)
{//注意了，用mcpwm 根本不需要量化级别，回归最朴素的区间划分
//一个unit 可以挂多个电机
    //mcpwm_gpio_init(iUnit, MCPWM0A, iPin_1);
    //if (iPin_2 > 0)
    //    mcpwm_gpio_init(iUnit, MCPWM0B, iPin_2);

    //// 2. 配置 MCPWM 定时器
    //mcpwm_config_t pwm_config;
    //pwm_config.frequency = iFreq;             // 舵机标准频率：50Hz (20ms周期)
    //pwm_config.cmpr_a = 0;                 // 初始占空比 0
    //pwm_config.cmpr_b = 0;
    //pwm_config.counter_mode = MCPWM_UP_COUNTER; // 向上计数
    //pwm_config.duty_mode = MCPWM_DUTY_MODE_0;   // 高电平有效

    //// 3. 应用配置到 MCPWM 单元 0，定时器 0
    //mcpwm_init(iUnit, iTimer, &pwm_config);
}

void Motor_Rotate_mcpwm(float fAngle, int iMotor_ID)
{//iChannel与舵机一一对应，可以视为舵机ID
    
}

void Init_Motor(int iChannel,int iPin, int iFreq, int iBits)
{//老板初始化，必须要用Channel，一个channel一个电机
    ledcSetup(iChannel, iFreq, iBits);
    ledcAttachPin(iPin, iChannel);
    Motor_Rotate(0, iChannel);
}

int iAngle_To_Signal(float fAngle, float fLeast_Time, float fMost_Time, int iFreq, int Qt)
{//角度转换位舵机量化值
    float fTime_Span = 1000.f / iFreq;
    float fStart = (fLeast_Time / fTime_Span) * Qt,
        fEnd = (fMost_Time / fTime_Span) * Qt;
    float fValue = (fAngle / 180.f) * (fEnd - fStart) + fStart;
    fValue = Clip3(fStart, fEnd, fValue);
    return (int)(fValue + 0.5f);
}

void Motor_Rotate(float fAngle, int iChannel)
{//iChannel与舵机一一对应，可以视为舵机ID
    fAngle = Clip3(0, 180, fAngle);
    printf("Channel:%d Angle:%f %d\n",iChannel, fAngle, iAngle_To_Signal(fAngle));
    ledcWrite(iChannel, iAngle_To_Signal(fAngle));
}
void Motor_Rotate_Hor(float fAngle)
{//相机的水平旋转
    fAngle = Clip3(0, 180, fAngle);
    Motor_Rotate(fAngle, oESP32_Env.m_iCam_Hor_Channel);
    oESP32_Env.m_fCur_Hor_Angle = fAngle;
}
void Motor_Rotate_Ver(float fAngle)
{//相机的垂直旋转
    fAngle = Clip3(0, 90, fAngle);
    Motor_Rotate(fAngle, oESP32_Env.m_iCam_Ver_Channel);
    oESP32_Env.m_fCur_Ver_Angle = fAngle;
}
void Motor_Rotate_Hor_Inc(float fDelta)
{//增量旋转
    float fAngle = oESP32_Env.m_fCur_Hor_Angle + fDelta;
    fAngle = Clip3(0, 180, fAngle);
    Motor_Rotate_Hor(fAngle);
}
void Motor_Rotate_Ver_Inc(float fDelta)
{//增量旋转
    float fAngle = oESP32_Env.m_fCur_Ver_Angle + fDelta;
    fAngle = Clip3(0, 90, fAngle);
    Motor_Rotate_Ver(fAngle);
}
/*******************************sg90舵机控制**************************/
int bSave_Raw_Data_eps32(const char* pcFile, void* pBuffer, int iSize)
{
    if (!LittleFS.begin(true, "/littlefs")) {
        Serial.println("Fail to init LittleFS");
        return 0;
    }
    int bResult = 0;
    bResult = bSave_Raw_Data(pcFile, pBuffer, iSize);
    LittleFS.end();
    return bResult;
}

framesize_t iGet_Frame_Size(int w, int h)
{
    framesize_t iFrame_Size = (framesize_t)-1;
    //printf("w:%d h:%d\n", w, h);
    if (w == 160 && h == 120)
    {
        printf("160x120\n");
        iFrame_Size = FRAMESIZE_QQVGA;
    }
    else if (w == 176 && h == 144)
    {
        printf("176x144\n");
        iFrame_Size = FRAMESIZE_HQVGA;
    }
    else if (w == 240 && h == 176)
    {
        printf("240x176\n");
        iFrame_Size = FRAMESIZE_HQVGA;
    }
    else if (w == 240 && h == 240)
    {
        printf("240x240\n");
        iFrame_Size = FRAMESIZE_240X240;
    }
    else if (w == 320 && h == 240)
    {
        printf("320x240\n");
        iFrame_Size = FRAMESIZE_QVGA;
    }
    else if (w == 400 && h == 296)
    {
        printf("400x296\n");
        iFrame_Size = FRAMESIZE_CIF;
    }
    else if (w == 640 && h == 480)
    {
        printf("640x480\n");
        iFrame_Size = FRAMESIZE_VGA;
    }
    else if (w == 800 && h == 600)
    {
        printf("800x600\n");
        iFrame_Size = FRAMESIZE_SVGA;
    }
    else if (w == 1024 && h == 768)
    {
        printf("1024x768\n");
        iFrame_Size = FRAMESIZE_XGA;
    }
    else if (w == 1280 && h == 720)
    {
        printf("1280x720\n");
        iFrame_Size = FRAMESIZE_HD;
    }
    else if (w == 1280 && h == 1024)
    {
        printf("1280x1024\n");
        iFrame_Size = FRAMESIZE_SXGA;
    }
    else if (w == 1600 && h == 1200)
    {
        printf("1600x1200\n");
        iFrame_Size = FRAMESIZE_UXGA;
    }
    else if (w == 2560 && h == 1440)
    {
        printf("2560x1440\n");
        iFrame_Size = FRAMESIZE_QHD;
    }
    else if (w == 2560 && h == 1600)
    {
        printf("2560x1600\n");
        iFrame_Size = FRAMESIZE_WQXGA;
    }
    else if (w == 1080 && h == 1920)
    {
        printf("1080x1920\n");
        iFrame_Size = FRAMESIZE_P_FHD;
    }
    else if (w == 2560 && h == 1920)
    {
        printf("2560x1920\n");
        iFrame_Size = FRAMESIZE_QSXGA;
    }
    else if (w == 1920 && h == 1080)
    {
        printf("1920x1080\n");
        iFrame_Size = FRAMESIZE_FHD;
    }
    else if (w == 720 && h == 1280)
    {
        printf("720x1280\n");
        iFrame_Size = FRAMESIZE_P_HD;
    }
    else if (w == 864 && h == 1536)
    {
        printf("864x1536\n");
        iFrame_Size = FRAMESIZE_P_3MP;
    }
    else if (w == 2048 && h == 1536)
    {
        printf("2048x1536\n");
        iFrame_Size = FRAMESIZE_QXGA;
    }
    else
        printf("Resolution not implemented yet in iGet_Frame_Size\n");

    return iFrame_Size;
}
void Free_Cam()
{//
    esp_camera_deinit();
}

int bInit_Cam_5640(int w, int h)
{//pin 占用： 4，5，6，7，8，9，11，12，13，15，16，17，18
//一旦用同一pin即冲突，要么相机死，要么外设死
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM    13

    framesize_t iFrame_Size = iGet_Frame_Size(w, h);
    if (iFrame_Size == (framesize_t)-1)
        return 0;
    static camera_config_t camera_config = {
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_sscb_sda = SIOD_GPIO_NUM,
        .pin_sscb_scl = SIOC_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM, 
        .xclk_freq_hz = 24000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = iFrame_Size,
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        printf("Fail to init Camere\n");
        return 0;
    }

    //清缓冲
    camera_fb_t* fb = NULL;
    for (int i = 0; i < 2; i++) {
        fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb); // 立即释放并清空当前这个缓冲区
            fb = NULL;
        }
    }

    sensor_t* s = esp_camera_sensor_get();
    if (s != NULL)
    {
        //s->set_whitebal(s, 0);
        //s->set_gain_ctrl(s, 0); 
        s->set_whitebal(s, 1); // 启用白平衡
        s->set_awb_gain(s, 1); // 启用自动增益
        s->set_hmirror(s, 1);

        // 1. 禁用特殊效果，确保无红光滤镜
        s->set_special_effect(s, 0);
        // 3. 设置白平衡模式为自动
        s->set_wb_mode(s, 0);
        // 4. 适当降低饱和度以抑制偏红
        s->set_saturation(s, -1);

        // 2 表示特效：Grayscale（灰度/黑白）
        s->set_special_effect(s, 2);

        //s->set_exposure_ctrl(s, 0);
        //s->set_aec_value(s, 200);
    }
    else
        printf("Fail to get sensor in bInit_Cam_5640\n");

    //printf("Camera Init successfully\n");
    return 1;
}

int Capture_5640(unsigned char** ppBuffer, int* piSize, unsigned long long* ptTime_Stamp)
{//Capture 一张照片
    camera_fb_t* poFrame = pCapture(ptTime_Stamp);
    *ppBuffer = NULL;
    if (!poFrame)
        return 0;

    unsigned char* pBuffer = (unsigned char*)pMalloc(poFrame->len);
    if (!pBuffer)
        return 0;
    
    memcpy(pBuffer, poFrame->buf, poFrame->len);
    Free_Frame(poFrame);
    *piSize = poFrame->len;
    *ppBuffer = pBuffer;
    return 1;
}

camera_fb_t* pCapture(unsigned long long* piTime_Stamp)
{//
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
        return NULL;
    }

    if (piTime_Stamp)
        *piTime_Stamp = fb->timestamp.tv_sec * 1000000 + fb->timestamp.tv_usec;

    return fb;
}

void Free_Frame(camera_fb_t* poFrame)
{//
    esp_camera_fb_return(poFrame);
}

void Send_Data()
{
    if (!LittleFS.begin(true, "/littlefs")) {
        Serial.println("Fail to init LittleFS ");
        return;
    }

    unsigned char* pBuffer = NULL, * pCur = NULL;
    int iSize = 0, bRet = 1, i;
    const int iPacket_Size = 100;
    if (!bLoad_Raw_Data("/littlefs/photo.jpg", &pBuffer, &iSize))
    {
        bRet = 0;
        goto END;
    }

    pCur = pBuffer;
    for (i = 0; i < iSize - iPacket_Size; i += iPacket_Size, pCur += iPacket_Size)
        Serial.write(pCur, iPacket_Size);
    if (i < iSize)
        Serial.write(pCur, iSize - i);

END:
    if (pBuffer)
        Free(pBuffer);
    LittleFS.end(); //
}
int iRecv_Packet(void* pBuffer, int iSize)
{//
    iSize = Serial.readBytes((unsigned char*)pBuffer, iSize);
    return iSize;
}

int iSend_Packet(void* pBuffer, int iSize)
{
    iSize = Serial.write((unsigned char*)pBuffer, iSize);
    return iSize;
}

int bSend_Data(void* pBuffer, int iSize)
{
    const int iPacket_Size = 1024;

    int iResult = iSend_Packet(&iSize, sizeof(int));
    unsigned char* pBuffer_1 = (unsigned char*)pBuffer;
    int i, iSize_1 = (iSize / iPacket_Size) * iPacket_Size;     //iSize - iPacket_Size;

    for (i = 0; i < iSize_1;)
    {
        iResult = iSend_Packet(&pBuffer_1[i], iPacket_Size);
        if (!iResult)
            return 0;
        i += iResult;
    }

    if (!iResult)
        return 0;


    iResult = iSend_Packet(&pBuffer_1[i], iSize - i);
    if (!iResult)
        return 0;
    if (iSize == 853)
        neopixelWrite(RGB_BUILTIN, 0, 255, 0);
    return 1;
}

int bInit_Wifi(const char User_Name[], const char Password[])
{//User_Name
    //"CMCC-9UxY";    
    //"34289546";

    if (!User_Name || !Password)
        return 0;

    WiFi.mode(WIFI_STA);

    Serial.printf("[网络] 正在尝试连接到 Wi-Fi: %s ...\n", User_Name);

    WiFi.begin(User_Name, Password);
    const int iMax_Retry = 10;
    int timeout_counter = 0, bRet = 1;

    while (WiFi.status() != WL_CONNECTED)
    {
        Sleep(500);
        Serial.print("."); 

        timeout_counter++;
        if (timeout_counter > iMax_Retry)
        { //
            timeout_counter = 0;
            bRet = 0;
            break;
        }
    }

    if (bRet)
    {
        Serial.print("[提示] 当前 Wi-Fi 信号强度: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm\n");
        Serial.println(WiFi.localIP());
        //Serial.println("====================================");
    }
    return bRet;
}

int bInit_FS()
{//
    if (!LittleFS.begin(true, "/littlefs")) {
        Serial.println("Fail to init LittleFS");
        return 0;
    }
    //printf("Lille FS inited\n");
    return 1;
}

int bInit_esp32_Env(const char Wifi_Usr[], const char Wifi_Pwd[], int iCam_w, int iCam_h, int iMem_Size,
    int iCam_Hor_Pin, int iCam_Hor_Channel,
    int iCam_Ver_Pin, int iCam_Ver_Channel)
{
    int iResult = bInit_FS();
    if (!iResult)return iResult;
    printf("Little File System inited\n");

    iResult = bInit_Wifi(Wifi_Usr, Wifi_Pwd);
    if (!iResult)return iResult;

    iResult = bInit_Cam_5640(iCam_w, iCam_h);
    if (!iResult)return iResult;
    printf("Cam 5640 Inited\n");

    Init_Motor(iCam_Hor_Channel, iCam_Hor_Pin);
    Init_Motor(iCam_Ver_Channel, iCam_Ver_Pin);
    printf("Hor Channel:%d Ver Channel:%d\n", iCam_Hor_Channel, iCam_Ver_Channel);
    oESP32_Env = { iCam_Hor_Channel,iCam_Ver_Channel,0,0 };

    //初始化TFT ST7789小屏
    Init_TFT(&oESP32_Env.m_oTFT_Setting, 320, 170, 1, 2, 3, 0);   // 320x170 => 横屏 ; SCL=1 SDA=2 DC=3 CS=0
    memset(oESP32_Env.m_oTFT_Setting.RGB[0], 0, 320 * 170);
    memset(oESP32_Env.m_oTFT_Setting.RGB[1], 0, 320 * 170);
    memset(oESP32_Env.m_oTFT_Setting.RGB[2], 0, 320 * 170);
    Flush(&oESP32_Env.m_oTFT_Setting);
    printf("TFT inited\n");

    //最后菜初始化内存管理器
    iResult = bInit_Env_CPU(iMem_Size, 128, 97);
    if (!iResult)return iResult;
    printf("Memory Pool inited\n");

    return 1;
}

int bChange_Frame_Size(int w, int h)
{
    sensor_t* poSensor = esp_camera_sensor_get(); //
    if (poSensor == NULL)
        return 0;

    framesize_t iFrame_Size = iGet_Frame_Size(w, h);
    if (iFrame_Size == (framesize_t)-1)
    {
        printf("Invalid Frame size:%dx%d\n", w, h);
        return 0;
    }
    // 核心步骤 A：直接通过指针写入新的常规分辨率
    poSensor->set_framesize(poSensor, iFrame_Size);

    //请缓冲
    for (int i = 0; i < 2; i++)
    {
        camera_fb_t* fb = esp_camera_fb_get(); // 抓出积压的旧帧
        if (fb)
            esp_camera_fb_return(fb);          // 立刻放回，完成刷新循环
        Sleep(30);      // 留出一点点时间让硬件切换稳定
    }
    return 1;
}
int iCmd_Rotate_Cam_Server(int iSocket)
{//协议		iDir:	4字节
//			Angle:	4字节，浮点数
//Response	iResult	4字节
    int iDir;
    float fAngle;
    int iResult = iRecvEx(iSocket,&iDir, 4);
    iResult = iRecvEx(iSocket,&fAngle, 4);
    if (!iResult)
        return 0;
    float fCur_Angle;
    if (iDir == 0)
    {
        Motor_Rotate_Hor(fAngle);
        //Motor_Rotate_Hor_Inc(fAngle);
        fCur_Angle = oESP32_Env.m_fCur_Hor_Angle;
    }else
    {
        Motor_Rotate_Ver(fAngle);
        //Motor_Rotate_Ver_Inc(fAngle);
        fCur_Angle = oESP32_Env.m_fCur_Ver_Angle;
    }
    iResult = 1;
    iResult = iSendEx(iSocket, &fCur_Angle, 4);
    return iResult;
}
int iCmd_Set_Cam_Frame_Size_Server(int iSocket)
{//设置摄像头分辨率，协议：
//  Recv        w, h    4字节，宽高
//  Response    Result:1:0      4字节，成功与否
//  Recv        REPLY_RECV		三次握手，表示收完
    int w, h, bRet = 1;
    int iResult = iRecvEx(iSocket, &w, 4);
    iResult = iRecvEx(iSocket, &h, 4);
    iResult = bChange_Frame_Size(w, h);
    if (!iResult)
    {
        printf("Fail to change frame size w:%d h:%d\n", w, h);
        return 0;
    }

    /*Free_Cam();
    iResult = bInit_Cam_5640(w, h);
    if (!iResult)
    {
        printf("Fail to set cam frame size: w:%d h:%d\n", w, h);
        return 0;
    }*/
    iResult = iSendEx(iSocket, &iResult, 4);

    //三次握手，收确认
    int iReply;
    iResult = iRecvEx(iSocket, &iReply, sizeof(int));
    if (!iResult)
        bRet = 0;
    printf("Set Cam Frame Size:%dx%d Successfully\n", w, h);
    return bRet;
}

int iCmd_Capture_Server(int iSocket)
{//协议：	
//	Response	iTime_Stamp		8字节，精确到毫秒
//				iSize:			4字节，文件大小
//				File_Conteng	iSize 个字节
// Recv			REPLY_RECV		三次握手，表示收完
    unsigned long long tTime_Stamp;
    camera_fb_t* poFrame = pCapture(&tTime_Stamp);
    if (!poFrame)
        return 0;

    int iResult = iSendEx(iSocket, &tTime_Stamp, 8);
    iResult = iSend_Buffer(iSocket, poFrame->buf, poFrame->len);
    Free_Frame(poFrame);

    int iReply, bRet = 1;
    iResult = iRecvEx(iSocket, &iReply, sizeof(int));
    if (!iResult)
        bRet = 0;

    return bRet;
}

/****************************JPEG Decode*******************************/
#include <JPEGDEC.h>

typedef struct {
    int srcW, srcH;
    int dstW, dstH;
    int bGray;
    unsigned char* pOut;
    unsigned char* pRow;  // staging row: srcW bytes (gray) or srcW*2 (rgb565)
    int curY;             // source row currently staged, -1 = none
} RESIZE_CTX;

// ESP32-S3 SIMD: broadcast byte v and store it n times (16 bytes per instr)
// fallback scalar on other targets
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ESP32S3)
static inline void simd_rep8(unsigned char* pDst, unsigned char v, int n)
{
    if (n <= 0) return;
    // ee.vst.128 needs a 16-byte aligned address, so do a scalar head/tail first
    size_t mis = (16u - ((size_t)pDst & 15u)) & 15u;
    while (n > 0 && mis) { *pDst++ = v; n--; mis--; }
    if (n >= 16)
    {
        unsigned char t = v;
        asm volatile (
            "ee.vldbc.8 q3, %0\n"      // broadcast v to all 16 lanes of q3
            :: "r"(&t) : "memory");
        unsigned char* pEnd = pDst + ((n >> 4) << 4);
        asm volatile (
            "1:\n"
            "ee.vst.128.ip q3, %0, 16\n"   // store 16 bytes, advance pDst
            "bltu %0, %1, 1b\n"
            : "+r"(pDst) : "r"(pEnd) : "memory");
        n &= 15;
    }
    while (n-- > 0)
        *pDst++ = v;
}
#else
static inline void simd_rep8(unsigned char* pDst, unsigned char v, int n)
{
    for (int i = 0; i < n; i++)
        pDst[i] = v;
}
#endif

// Fill n 16-bit pixels with one value using a doubling memcpy (aligned-safe, fast)
static inline void repeat16(uint16_t* pDst, uint16_t v, int n)
{
    if (n <= 0) return;
    pDst[0] = v;
    if (n == 1) return;
    size_t filled = 2;
    size_t total = (size_t)n * 2;
    unsigned char* pB = (unsigned char*)pDst;
    while (filled < total)
    {
        size_t chunk = filled;
        if (chunk > total - filled) chunk = total - filled;
        memcpy(pB + filled, pB, chunk);
        filled += chunk;
    }
}

static void resize_flush_row(RESIZE_CTX* pCtx)
{
    if (pCtx->curY < 0)
        return;
    int y0 = (int)(((long)pCtx->curY * pCtx->dstH) / pCtx->srcH);
    int y1 = (int)(((long)(pCtx->curY + 1) * pCtx->dstH) / pCtx->srcH);
    if (y0 < 0) y0 = 0;
    if (y1 > pCtx->dstH) y1 = pCtx->dstH;
    if (pCtx->bGray)
    {
        // horizontal NN upscale into the top output row once, then memcpy the repeats
        int hInt = (pCtx->dstW % pCtx->srcW) == 0 ? pCtx->dstW / pCtx->srcW : 0;
        unsigned char* pTop = pCtx->pOut + (long)y0 * pCtx->dstW;
        if (hInt > 0)
        {
            for (int sx = 0; sx < pCtx->srcW; sx++)
                simd_rep8(pTop + (long)sx * hInt, pCtx->pRow[sx], hInt);
        }
        else
        {
            for (int tx = 0; tx < pCtx->dstW; tx++)
            {
                int sx = (int)(((long)tx * pCtx->srcW) / pCtx->dstW);
                pTop[tx] = pCtx->pRow[sx];
            }
        }
        for (int ty = y0 + 1; ty < y1; ty++)
            memcpy(pCtx->pOut + (long)ty * pCtx->dstW, pTop, (size_t)pCtx->dstW);
    }
    else
    {
        const uint16_t* pSrc = (const uint16_t*)pCtx->pRow;
        uint16_t* pTop = (uint16_t*)pCtx->pOut + (long)y0 * pCtx->dstW;
        int hInt = (pCtx->dstW % pCtx->srcW) == 0 ? pCtx->dstW / pCtx->srcW : 0;
        if (hInt > 0)
        {
            for (int sx = 0; sx < pCtx->srcW; sx++)
                repeat16(pTop + (long)sx * hInt, pSrc[sx], hInt);
        }
        else
        {
            for (int tx = 0; tx < pCtx->dstW; tx++)
            {
                int sx = (int)(((long)tx * pCtx->srcW) / pCtx->dstW);
                pTop[tx] = pSrc[sx];
            }
        }
        for (int ty = y0 + 1; ty < y1; ty++)
            memcpy((uint8_t*)pCtx->pOut + (long)ty * pCtx->dstW * 2, pTop, (size_t)pCtx->dstW * 2);
    }
    pCtx->curY = -1;
}

static int jpeg_resize_draw(JPEGDRAW* pDraw)
{
    RESIZE_CTX* pCtx = (RESIZE_CTX*)pDraw->pUser;
    if (!pCtx || pDraw->iWidth <= 0 || pDraw->iHeight <= 0)
        return 1;                            // non-zero = keep decoding
    int x = pDraw->x;
    if (x >= pCtx->srcW)
        return 1;                            // nothing visible this block
    int cw = pDraw->iWidth;                  // pDraw->iWidth includes MCU padding
    if (x + cw > pCtx->srcW)
        cw = pCtx->srcW - x;                 // clamp to the real right edge
    if (cw <= 0)
        return 1;
    for (int j = 0; j < pDraw->iHeight; j++)
    {
        int y = pDraw->y + j;
        if (y < 0 || y >= pCtx->srcH)
            continue;
        if (y != pCtx->curY)
        {
            resize_flush_row(pCtx);
            pCtx->curY = y;
        }
        if (pCtx->bGray)                     // 8-bit luma bytes
        {
            const unsigned char* pPix = (const unsigned char*)pDraw->pPixels;
            memcpy(pCtx->pRow + x, pPix + j * pDraw->iWidth, (size_t)cw);
        }
        else                                 // 16-bit RGB565
        {
            const unsigned char* pPix = (const unsigned char*)pDraw->pPixels;
            memcpy(pCtx->pRow + x * 2, pPix + j * pDraw->iWidth * 2, (size_t)cw * 2);
        }
        if (x + cw == pCtx->srcW)
            resize_flush_row(pCtx);          // full row received
    }
    return 1;
}

int Decode_Resize_JPG(unsigned char* pBuffer, int iSize, int w, int h, int bGray, unsigned char* pDest)
{
    if (pBuffer == NULL || iSize <= 0 || w <= 0 || h <= 0 || pDest == NULL)
    {
        printf("DRJ: bad param\n");
        return 0;
    }
    JPEGDEC* jpeg = new JPEGDEC();              // ~18KB object, keep off the 8KB loop stack
    if (!jpeg)
    {
        printf("DRJ: no jpeg mem\n");
        return 0;
    }
    if (!jpeg->openRAM(pBuffer, iSize, jpeg_resize_draw))  // returns 1=ok, 0=fail
    {
        printf("DRJ: openRAM err=%d\n", jpeg->getLastError());
        delete jpeg;
        return 0;
    }
    int srcW = jpeg->getWidth();
    int srcH = jpeg->getHeight();
    if (srcW <= 0 || srcH <= 0)
    {
        delete jpeg;
        return 0;
    }
    unsigned char* pRow = (unsigned char*)malloc((size_t)srcW * (bGray ? 1 : 2));
    if (!pRow)
    {
        delete jpeg;
        printf("DRJ: no mem\n");
        return 0;
    }
    RESIZE_CTX ctx;
    ctx.srcW = srcW; ctx.srcH = srcH; ctx.dstW = w; ctx.dstH = h;
    ctx.bGray = bGray; ctx.pOut = pDest; ctx.pRow = pRow; ctx.curY = -1;

    jpeg->setUserPointer(&ctx);
    int rc;
    if (bGray)   // fastest: decode luma (Y) only, skips all chroma work
    {
        jpeg->setPixelType(EIGHT_BIT_GRAYSCALE);
        rc = jpeg->decode(0, 0, JPEG_LUMA_ONLY);
    }
    else
    {
        jpeg->setPixelType(RGB565_LITTLE_ENDIAN);
        rc = jpeg->decode(0, 0, 0);
    }
    if (rc == 0)   // decode returns 1=ok, 0=fail
    {
        printf("DRJ: decode err=%d\n", jpeg->getLastError());
        free(pRow);
        delete jpeg;
        return 0;
    }
    resize_flush_row(&ctx);                       // last partial row
    delete jpeg;
    free(pRow);
    printf("DRJ ok %dx%d -> %dx%d gray=%d\n", srcW, srcH, w, h, bGray);
    return 1;
}

int Decode_Resize_JPG(unsigned char* pBuffer, int iSize,
    int bGray, Image oImage)
{
    if (!oImage.m_pChannel[0])
    {
        printf("Error in Decode_Resize_JPG\n");
        return 0;
    }
    int iResult = Decode_Resize_JPG(pBuffer, iSize,
        oImage.m_iWidth, oImage.m_iHeight, bGray,
        oImage.m_pChannel[0]);
    if (!iResult)
        return 0;
    if (bGray && oImage.m_iChannel_Count > 1)
    {
        iSize = oImage.m_iWidth * oImage.m_iHeight;
        for (int i = 1; i < oImage.m_iChannel_Count; i++)
            memcpy(oImage.m_pChannel[i], oImage.m_pChannel[0], iSize);
    }
    return 1;
}

static void jpg_close_cb(void* pHandle)
{
    FILE* pf = (FILE*)pHandle;
    if (pf) fclose(pf);
}
static int32_t jpg_read_cb(JPEGFILE* pFile, uint8_t* pBuf, int32_t iLen)
{
    FILE* pf = (FILE*)pFile->fHandle;
    if (!pf) return 0;
    return (int32_t)fread(pBuf, 1, iLen, pf);
}
static int32_t jpg_seek_cb(JPEGFILE* pFile, int32_t iPosition)
{
    FILE* pf = (FILE*)pFile->fHandle;
    if (!pf) return 0;
    return fseek(pf, iPosition, SEEK_SET);
}

struct JPGPlaneCtx {
    unsigned char* R, * G, * B;
    int w, h;
};

static int jpg_draw_cb(JPEGDRAW* pDraw)
{
    JPGPlaneCtx* ctx = (JPGPlaneCtx*)pDraw->pUser;
    uint32_t* px = (uint32_t*)pDraw->pPixels;
    for (int y = 0; y < pDraw->iHeight; y++)
    {
        int py = pDraw->y + y;
        if (py >= ctx->h) continue;
        size_t row = (size_t)py * ctx->w;
        for (int x = 0; x < pDraw->iWidth; x++)
        {
            int pxl = pDraw->x + x;
            if (pxl >= ctx->w) continue;
            uint32_t c = px[y * pDraw->iWidth + x];
            size_t i = row + pxl;
            ctx->R[i] = (c >> 16) & 0xFF;
            ctx->G[i] = (c >> 8) & 0xFF;
            ctx->B[i] = c & 0xFF;
        }
    }
    return 1;
}

static void Decode_JPG(unsigned char* pSource, int iSource_Size,
    unsigned char** ppDest, int* w, int* h, int* iChannel)
{
    *ppDest = NULL; *w = *h = *iChannel = 0;

    JPEGDEC* jpeg = new JPEGDEC();
    if (!jpeg) return;
    if (!jpeg->openRAM(pSource, iSource_Size, jpg_draw_cb))   // 内存解码
    {
        delete jpeg; printf("Fail to open JPG (memory)...\n"); return;
    }

    *w = jpeg->getWidth();
    *h = jpeg->getHeight();
    *iChannel = (jpeg->getBpp() == 8) ? 1 : 3;

    size_t plane = (size_t)(*w) * (*h);
    *ppDest = (unsigned char*)pMalloc(plane * (*iChannel));
    if (!(*ppDest)) { jpeg->close(); delete jpeg; return; }    // 分配失败

    JPGPlaneCtx ctx = { *ppDest,
        (*iChannel == 3) ? *ppDest + plane : *ppDest,
        (*iChannel == 3) ? *ppDest + plane * 2 : *ppDest,
        *w, *h };
    jpeg->setUserPointer(&ctx);
    jpeg->setPixelType(RGB8888);
    int ok = jpeg->decode(0, 0, 0);
    jpeg->close();
    delete jpeg;
    if (!ok) 
    { 
        Free(*ppDest); 
        *ppDest = NULL;
        *w = *h = *iChannel = 0;
    }
}
int Decode_JPG(unsigned char Buffer[], int iSize, Image* poImage)
{
    int w, h, iChannel;
    unsigned char* pDest = NULL;
    Decode_JPG(Buffer, iSize, &pDest, &w, &h, &iChannel);
    if (!pDest)
        return 0;

    Attach_Buffer(poImage, pDest, w, h, iChannel, Image::IMAGE_TYPE_BMP);
    return 1;
}
int Decode_JPG(const char Path[], int* w, int* h, int* piChannel, unsigned char** ppBuffer)
{
    *ppBuffer = NULL;
    *piChannel = 0;

    FILE* fp = fopen(Path, "rb");
    if (!fp)
    {
        printf("Fail to open %s %d\n", Path, iGet_File_Length((char*)Path));
        return 0;
    }

    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    JPEGDEC* jpeg = new JPEGDEC();
    if (!jpeg->open((void*)fp, fsize, jpg_close_cb, jpg_read_cb, jpg_seek_cb, jpg_draw_cb))
    {
        delete jpeg; return 0;
    }

    *w = jpeg->getWidth();
    *h = jpeg->getHeight();
    *piChannel = (jpeg->getBpp() == 8) ? 1 : 3;

    size_t plane = (size_t)(*w) * (*h);
    *ppBuffer = (unsigned char*)pMalloc(plane * (*piChannel));
    if (!(*ppBuffer))
    {
        printf("Fail to allocate memory in Decode_JPG\n");
        jpeg->close();
        delete jpeg;
        return 0;
    }

    JPGPlaneCtx ctx;
    ctx.R = *ppBuffer;
    ctx.G = (*piChannel == 3) ? *ppBuffer + plane : ctx.R;
    ctx.B = (*piChannel == 3) ? *ppBuffer + plane * 2 : ctx.R;
    ctx.w = *w;
    ctx.h = *h;
    jpeg->setUserPointer(&ctx);
    jpeg->setPixelType(RGB8888);
    int ok = jpeg->decode(0, 0, 0);
    jpeg->close();
    delete jpeg;
    if (!ok)
    {
        free(*ppBuffer);
        *ppBuffer = NULL;
        return 0;
    }
    return 1;
}
int Decode_JPG(const char File[], Image* poImage)
{
    int iResult;
    unsigned char* pBuffer = NULL;
    int w, h, iChannel;

    if (!(iResult = Decode_JPG(File, &w, &h, &iChannel, &pBuffer)))
        return 0;

    Attach_Buffer(poImage, pBuffer, w, h, iChannel, Image::IMAGE_TYPE_BMP);
    return 1;
}
/****************************JPEG Decode*******************************/
/*******************************ST7789 TFT显示屏**********************/
#include <driver/spi_master.h>
#include <U8g2lib.h>

void Textout(int x, int y, const char* str, unsigned char R, unsigned char G, unsigned char B)
{
    Textout(&oESP32_Env.m_oTFT_Setting, x, y, 0xFFFF, str, R, G, B);
}

Image Get_Screen()
{
    Image oImage;
    Attach_Buffer(&oImage, oESP32_Env.m_oTFT_Setting.RGB[0],
        oESP32_Env.m_oTFT_Setting.LCD_W, oESP32_Env.m_oTFT_Setting.LCD_H,
        3, Image::IMAGE_TYPE_BMP);
    return oImage;
}

static void lcd_x(TFT_Setting* poSetting, const uint8_t* d, size_t n, uint8_t dc)
{
    if (!n) return;
    digitalWrite(poSetting->_cs, LOW);
    digitalWrite(poSetting->_dc, dc);
    size_t off = 0;
    while (off < n)
    {
        size_t sz = n - off;
        if (sz > 2048) sz = 2048;
        spi_transaction_t t = {};
        t.length = sz * 8;
        t.tx_buffer = d + off;
        esp_err_t e = spi_device_transmit(poSetting->hSPI, &t);
        if (e != ESP_OK) Serial.printf("tx FAIL @%u 0x%X\n", off, e);
        off += sz;
    }
    digitalWrite(poSetting->_cs, HIGH);
}

static void lcd_cmd(TFT_Setting* poSetting, uint8_t c)
{
    lcd_x(poSetting, &c, 1, 0);
}
static void lcd_data(TFT_Setting* poSetting, const uint8_t* d, int n)
{
    lcd_x(poSetting, d, n, 1);
}

static void set_window(TFT_Setting* poSetting, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    lcd_cmd(poSetting, 0x2A);
    uint8_t xb[] = { (uint8_t)((x0 + poSetting->XOFF) >> 8), (uint8_t)((x0 + poSetting->XOFF) & 0xFF),
                    (uint8_t)((x1 + poSetting->XOFF) >> 8), (uint8_t)((x1 + poSetting->XOFF) & 0xFF) };
    lcd_data(poSetting, xb, 4);
    lcd_cmd(poSetting, 0x2B);
    uint8_t yb[] = { (uint8_t)((y0 + poSetting->YOFF) >> 8), (uint8_t)((y0 + poSetting->YOFF) & 0xFF),
                    (uint8_t)((y1 + poSetting->YOFF) >> 8), (uint8_t)((y1 + poSetting->YOFF) & 0xFF) };
    lcd_data(poSetting, yb, 4);
    lcd_cmd(poSetting, 0x2C);
}

// U8g2 无硬件挂载: 只装配内存, 不改总线
static uint8_t u8x8_d_fb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr)
{
    (void)arg_int; (void)arg_ptr; (void)u8x8;
    switch (msg)
    {
        // display_info 已由 Init_TFT 直接写入 u8x8->display_info, 这里无需处理
    case U8X8_MSG_DISPLAY_INIT:
        u8x8_d_helper_display_init(u8x8);
        break;
    default:
        break;
    }
    return 1;
}

void Init_TFT(TFT_Setting* poSetting, int LCD_W, int LCD_H,
    int iPin_scl, int iPin_sda, int iPin_dc, int iPin_cs)
{
    // 自适应横竖屏: w>h 视为横屏(320x170), 否则竖屏(170x320)
    int oLandscape = (LCD_W > LCD_H);
    int XOFFL = oLandscape ? 0 : 35;
    int YOFFL = oLandscape ? 35 : 0;
    poSetting->LCD_W = LCD_W;
    poSetting->LCD_H = LCD_H;
    poSetting->XOFF = XOFFL;
    poSetting->YOFF = YOFFL;
    poSetting->_scl = iPin_scl;
    poSetting->_sda = iPin_sda;
    poSetting->_dc = iPin_dc;
    poSetting->_cs = iPin_cs;

    pinMode(poSetting->_cs, OUTPUT);
    pinMode(poSetting->_dc, OUTPUT);
    digitalWrite(poSetting->_cs, HIGH);
    digitalWrite(poSetting->_dc, HIGH);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = poSetting->_sda;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = poSetting->_scl;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = poSetting->LCD_W * poSetting->LCD_H * 2;
    esp_err_t e1 = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    Serial.printf("spi_bus_initialize 0x%X\n", e1);

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 40 * 1000 * 1000;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 4;
    esp_err_t e2 = spi_bus_add_device(SPI3_HOST, &devcfg, &poSetting->hSPI);
    Serial.printf("spi_bus_add_device 0x%X\n", e2);

    lcd_cmd(poSetting, 0x01); delay(150);   // SWRESET
    lcd_cmd(poSetting, 0x11); delay(120);   // SLPOUT
    uint8_t m36[] = { (uint8_t)(oLandscape ? 0x60 : 0x00) };
    lcd_cmd(poSetting, 0x36);
    lcd_data(poSetting, m36, 1);
    uint8_t m3a[] = { 0x05 };
    lcd_cmd(poSetting, 0x3A);
    lcd_data(poSetting, m3a, 1);
    uint8_t b2[] = { 0x0C, 0x0C, 0x00, 0x33, 0x33 };
    lcd_cmd(poSetting, 0xB2);
    lcd_data(poSetting, b2, 5);
    uint8_t b7[] = { 0x35 };
    lcd_cmd(poSetting, 0xB7);
    lcd_data(poSetting, b7, 1);
    uint8_t bb[] = { 0x1A };
    lcd_cmd(poSetting, 0xBB);
    lcd_data(poSetting, bb, 1);
    uint8_t c0[] = { 0x2C };
    lcd_cmd(poSetting, 0xC0);
    lcd_data(poSetting, c0, 1);
    uint8_t c2[] = { 0x01 };
    lcd_cmd(poSetting, 0xC2);
    lcd_data(poSetting, c2, 1);
    uint8_t c3[] = { 0x0B };
    lcd_cmd(poSetting, 0xC3);
    lcd_data(poSetting, c3, 1);
    uint8_t c4[] = { 0x20 };
    lcd_cmd(poSetting, 0xC4);
    lcd_data(poSetting, c4, 1);
    uint8_t c6[] = { 0x0F };
    lcd_cmd(poSetting, 0xC6);
    lcd_data(poSetting, c6, 1);
    uint8_t d0[] = { 0xA4, 0xA1 };
    lcd_cmd(poSetting, 0xD0);
    lcd_data(poSetting, d0, 2);
    uint8_t g0[] = { 0x00, 0x03, 0x07, 0x08, 0x07, 0x15, 0x2A, 0x44, 0x42, 0x0A, 0x17, 0x18, 0x25, 0x27 };
    lcd_cmd(poSetting, 0xE0);
    lcd_data(poSetting, g0, 14);
    uint8_t g1[] = { 0x00, 0x03, 0x08, 0x07, 0x07, 0x23, 0x2A, 0x43, 0x42, 0x09, 0x18, 0x17, 0x25, 0x27 };
    lcd_cmd(poSetting, 0xE1);
    lcd_data(poSetting, g1, 14);
    lcd_cmd(poSetting, 0x29);   // DISPON

    // ---- 合并缓冲: 必须内部 RAM+DMA (SPI DMA 不能读 PSRAM) ----
    poSetting->fbSz = (size_t)LCD_W * LCD_H * 2;
    poSetting->fb = (uint8_t*)heap_caps_malloc(poSetting->fbSz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!poSetting->fb) poSetting->fb = (uint8_t*)malloc(poSetting->fbSz);
    if (!poSetting->fb)
    {
        Serial.println("fb alloc FAILED");
        for (;;) delay(100);
    }

    // ---- RGB 三平面: 只被 CPU 读写, 放 PSRAM 省内部 RAM; 无 PSRAM 退回内部 ----
    poSetting->rgbSz = (size_t)LCD_W * LCD_H;
    for (int i = 0; i < 3; i++)
    {
        poSetting->RGB[i] = (unsigned char*)heap_caps_malloc(poSetting->rgbSz, MALLOC_CAP_SPIRAM);
        if (!poSetting->RGB[i]) poSetting->RGB[i] = (unsigned char*)malloc(poSetting->rgbSz);
        if (!poSetting->RGB[i])
        {
            Serial.println("RGB alloc FAILED");
            for (;;) delay(100);
        }
        memset(poSetting->RGB[i], 0, poSetting->rgbSz);
    }

    // ---- U8g2 画布 + 字库引擎 ----
    poSetting->U2_W = ((poSetting->LCD_W + 7) / 8) * 8;
    poSetting->U2_TILES = ((poSetting->LCD_H + 7) / 8);
    poSetting->u8g2_buf = (uint8_t*)malloc(poSetting->U2_W * poSetting->U2_TILES);
    if (!poSetting->u8g2_buf)
    {
        Serial.println("u8g2 buf alloc FAILED");
        for (;;) delay(100);
    }

    memset(&poSetting->u8g2, 0, sizeof(poSetting->u8g2));
    memset(&poSetting->di, 0, sizeof(poSetting->di));
    poSetting->di.chip_enable_level = 0;
    poSetting->di.chip_disable_level = 1;
    poSetting->di.sck_clock_hz = 4000000UL;
    poSetting->di.i2c_bus_clock_100kHz = 4;
    poSetting->di.tile_width = poSetting->U2_W / 8;
    poSetting->di.tile_height = poSetting->U2_TILES;
    poSetting->di.pixel_width = poSetting->LCD_W;
    poSetting->di.pixel_height = poSetting->LCD_H;
    u8g2_SetupDisplay(&poSetting->u8g2, u8x8_d_fb, u8x8_cad_empty, u8x8_cad_empty, u8x8_cad_empty);
    // U8g2 的 display_info 由外部直接写入本对象自己的 di, 不走全局/回调
    u8g2_GetU8x8(&poSetting->u8g2)->display_info = &poSetting->di;
    u8g2_SetupBuffer(&poSetting->u8g2, poSetting->u8g2_buf, poSetting->U2_TILES,
        u8g2_ll_hvline_vertical_top_lsb, U8G2_R0);

    Serial.printf("fb=%p rgbSz=%u\n", poSetting->fb, (unsigned)poSetting->rgbSz);
}

// 全屏清一色 (直接走 SPI, 不经 RGB)
void TFT_Clear(TFT_Setting* poSetting, uint16_t color)
{
    set_window(poSetting, 0, 0, poSetting->LCD_W - 1, poSetting->LCD_H - 1);
    byte h = ~(color >> 8), l = ~(color & 0xFF);
    int iSize = poSetting->LCD_W * poSetting->LCD_H;
    static uint8_t* buf = NULL;
    if (!buf) buf = (uint8_t*)malloc((size_t)iSize * 2);
    if (!buf)
    {
        Serial.println("MALLOC_FAIL");
        return;
    }
    for (int i = 0, j = 0; i < iSize; i++)
    {
        buf[j++] = h;
        buf[j++] = l;
    }
    lcd_x(poSetting, buf, (size_t)iSize * 2, 1);
}

// ---- 将 RGB[3] 三平面合并成 RGB565 并刷到屏上 ----
void Flush(TFT_Setting* poSetting)
{
    if (!poSetting)
        poSetting = &oESP32_Env.m_oTFT_Setting;
    const size_t n = poSetting->rgbSz;
    const unsigned char* R = poSetting->RGB[0];
    const unsigned char* G = poSetting->RGB[1];
    const unsigned char* B = poSetting->RGB[2];
    uint8_t* out = poSetting->fb;
    for (size_t i = 0; i < n; i++)
    {
        uint16_t c = ((uint16_t)((*R++) & 0xF8) << 8)
            | ((uint16_t)((*G++) & 0xFC) << 3)
            | ((*B++) >> 3);
        *out++ = ~(c >> 8);
        *out++ = ~(c & 0xFF);
    }
    set_window(poSetting, 0, 0, poSetting->LCD_W - 1, poSetting->LCD_H - 1);
    lcd_x(poSetting, poSetting->fb, poSetting->fbSz, 1);
}

static uint8_t utf8_seq_len(const char* s)
{
    const uint8_t c = (uint8_t)*s;
    if (c < 0x80) return 1;
    if ((c >> 5) == 0x06) return 2;
    if ((c >> 4) == 0x0E) return 3;
    if ((c >> 3) == 0x1E) return 4;
    return 1;
}

// 把 U8g2 位图画布拷进 RGB 平面 (置位像素 = 前景色 R/G/B)
static void text_copyout(TFT_Setting* p, unsigned char R, unsigned char G, unsigned char B)
{
    const unsigned int cw = p->U2_W;
    const unsigned int cheight = p->U2_TILES * 8;
    const size_t stride = p->LCD_W;
    for (uint32_t y = 0; y < cheight; y++)
    {
        const uint8_t bit = 1 << (y & 7);
        const uint8_t* row = &p->u8g2_buf[(y >> 3) * cw];
        if (y >= p->LCD_H) continue;
        const size_t off = (size_t)y * stride;
        for (uint32_t x = 0; x < cw && x < p->LCD_W; x++)
        {
            if (row[x] & bit)
            {
                p->RGB[0][off + x] = R;
                p->RGB[1][off + x] = G;
                p->RGB[2][off + x] = B;
            }
        }
    }
}

// 在 (x, yTop) 打印 UTF-8 到 RGB 平面, maxW 内自动换行; R/G/B 文字颜色
void Textout(TFT_Setting* poSetting, uint16_t x, uint16_t yTop, uint16_t maxW,
    const char* s, unsigned char R, unsigned char G, unsigned char B)
{
    u8g2_ClearBuffer(&poSetting->u8g2);
    u8g2_SetFontMode(&poSetting->u8g2, 1);
    u8g2_SetFontPosBaseline(&poSetting->u8g2);
    u8g2_SetFont(&poSetting->u8g2, u8g2_font_wqy16_t_gb2312);

    const int lineH = u8g2_GetAscent(&poSetting->u8g2) - u8g2_GetDescent(&poSetting->u8g2) + 2;
    const int rightEdge = x + maxW;
    int cx = x;
    int cy = yTop + u8g2_GetAscent(&poSetting->u8g2);

    while (*s)
    {
        if (*s == '\n')
        {
            cx = x;
            cy += lineH;
            s++;
            continue;
        }
        const uint8_t n = utf8_seq_len(s);
        char glyph[5];
        for (uint8_t i = 0; i < n; i++) glyph[i] = s[i];
        glyph[n] = 0;
        s += n;

        const int w = u8g2_GetUTF8Width(&poSetting->u8g2, glyph);
        if (w > 0 && cx + w > rightEdge && cx > x)
        {
            cx = x;
            cy += lineH;
        }
        u8g2_DrawUTF8(&poSetting->u8g2, cx, cy, glyph);
        cx += w;
    }
    text_copyout(poSetting, R, G, B);
}

// 在矩形区域 (x0,y0)-(x1,y1) 内 Wrap 显示 UTF-8 文字 (含括号)
// 区域先清成背景色, 文字超宽换行, 超出区域底部则停止; R/G/B 前景色

void Textout_Ex(TFT_Setting* p, short x0, short y0, short x1, short y1,
    const char* s, unsigned char R, unsigned char G, unsigned char B)
{
    if (x1 < x0 || y1 < y0) return;

    // 清空区域背景 (RGB 平面置黑), 避免重绘残留
    for (uint32_t yy = y0; yy <= y1 && yy < p->LCD_H; yy++)
    {
        const size_t off = (size_t)yy * p->LCD_W;
        for (uint32_t xx = x0; xx <= x1 && xx < p->LCD_W; xx++)
        {
            p->RGB[0][off + xx] = 0;
            p->RGB[1][off + xx] = 0;
            p->RGB[2][off + xx] = 0;
        }
    }

    u8g2_ClearBuffer(&p->u8g2);
    u8g2_SetFontMode(&p->u8g2, 1);
    u8g2_SetFontPosBaseline(&p->u8g2);
    u8g2_SetFont(&p->u8g2, u8g2_font_wqy16_t_gb2312);

    const int lineH = u8g2_GetAscent(&p->u8g2) - u8g2_GetDescent(&p->u8g2) + 2;
    const int ascent = u8g2_GetAscent(&p->u8g2);
    const int descent = u8g2_GetDescent(&p->u8g2);
    const int rightEdge = (int)x1;
    int cx = (int)x0;
    int cy = (int)y0 + ascent;

    while (*s)
    {
        if (*s == '\n')
        {
            cx = (int)x0;
            cy += lineH;
            s++;
            continue;
        }
        const uint8_t n = utf8_seq_len(s);
        char glyph[5];
        for (uint8_t i = 0; i < n; i++) glyph[i] = s[i];
        glyph[n] = 0;
        s += n;

        const int w = u8g2_GetUTF8Width(&p->u8g2, glyph);
        if (w == 0) continue;
        if (cx + w > rightEdge && cx > (int)x0)
        {
            cx = (int)x0;
            cy += lineH;
        }
        if (cy + descent > (int)y1) break;                    // 超出区域底部
        u8g2_DrawUTF8(&p->u8g2, cx, cy, glyph);
        cx += w;
    }

    // 拷进 RGB 平面, 只取区域内像素
    const unsigned int cw = p->U2_W;
    const unsigned int bufH = p->U2_TILES * 8;
    for (uint32_t y = y0; y <= y1 && y < p->LCD_H && y < bufH; y++)
    {
        const uint8_t bit = 1 << (y & 7);
        const uint8_t* row = &p->u8g2_buf[(y >> 3) * cw];
        const size_t off = (size_t)y * p->LCD_W;
        for (uint32_t x = x0; x <= x1 && x < p->LCD_W && x < cw; x++)
        {
            if (row[x] & bit)
            {
                p->RGB[0][off + x] = R;
                p->RGB[1][off + x] = G;
                p->RGB[2][off + x] = B;
            }
        }
    }
}
/*******************************ST7789 TFT显示屏**********************/


#endif // !WIN32

int bLoad_Setting(const char* pcFile, Init_Setting_Client* poSetting)
{
    char* pText = NULL;
    int iSize = 0, bRet = 1, iResult;
    char Value[32];

    if (!(iResult = bLoad_Raw_Data(pcFile, (unsigned char**)&pText, &iSize)))
        return 0;

    if (!bGet_Value(pText, iResult, "IP", poSetting->m_IP))
    {
        printf("Fail to load setting IP\n");
        bRet = 0;
        goto END;
    }
    if (!bGet_Value(pText, iResult, "Port", Value))
    {
        printf("Fail to load setting Port\n");
        bRet = 0;
        goto END;
    }
    poSetting->m_iPort = atoi(Value);
END:
    if (pText)
        Free(pText);

    return bRet;
}
int bLoad_Setting(const char* pcFile, Init_Setting_Server* poSetting)
{
    char* pText = NULL;
    int iSize = 0, bRet = 1, iResult;
    char Value[32];

    if (!(iResult = bLoad_Raw_Data(pcFile, (unsigned char**)&pText, &iSize)))
        return 0;

    if (!bGet_Value(pText, iResult, "IP", poSetting->m_IP))
    {
        printf("Fail to load setting IP\n");
        bRet = 0;
        goto END;
    }
    if (!bGet_Value(pText, iResult, "Port", Value))
    {
        printf("Fail to load setting Port\n");
        bRet = 0;
        goto END;
    }
    poSetting->m_iPort = atoi(Value);

    if (!bGet_Value(pText, iResult, "Wifi User Name", poSetting->wifi_User_Name))
    {
        printf("Fail to load setting Wifi User Name \n");
        bRet = 0;
        goto END;
    }

    if (!bGet_Value(pText, iResult, "Wifi Password", poSetting->wifi_Password))
    {
        printf("Fail to load setting Wifi Password\n");
        bRet = 0;
        goto END;
    }

    if (!bGet_Value(pText, iResult, "Resolution", Value))
    {
        printf("Fail to load setting Resolution\n");
        bRet = 0;
        goto END;
    }
    sscanf(Value, "%dx%d", &poSetting->m_iCam_w, &poSetting->m_iCam_h);

END:
    if (pText)
        Free(pText);

    return bRet;
}

static void Distribute_Cmd(int iSocket)
{//有必要对每一个Cmd做一个约定，返回值反映了处理结果
//0：    表示失败
//1：    表示成功
//-1:   表示网络错误,     由于工程浩大，营养不高，下一班再干

    int iCmd = 0;
    int iResult = iRecvEx(iSocket, &iCmd, 4);
    if (!iResult)
        return;	//出错了

    switch (iCmd)
    {
    case CMD_SHAKE_HAND:
        iResult = iCmd_Shake_Hand_Server(iSocket);
        break;
    case CMD_GET_FILE:
        iResult = iCmd_Get_File_Server(iSocket);
        break;
    case CMD_UPLOAD_FILE:
        iResult = iCmd_Upload_File_Server(iSocket);
        break;
    case CMD_GET_FILE_LIST:
        iResult = iCmd_Get_File_List_Server(iSocket);
        break;
    case CMD_DELETE_FILE:
        iResult = iCmd_Delete_File_Server(iSocket);
        break;
    case CMD_MD:
        iResult = iCmd_md_Server(iSocket);
        break;
    case CMD_RD:
        iResult = iCmd_rd_Server(iSocket);
        break;
#ifdef ESP32
    case CMD_CAPTURE:
        iResult = iCmd_Capture_Server(iSocket);
        break;
    case CMD_SET_CAM_FRAME_SIZE:
        iResult = iCmd_Set_Cam_Frame_Size_Server(iSocket);
        break;
    case CMD_ROTATE_CAM:
        iResult = iCmd_Rotate_Cam_Server(iSocket);
        break;
#endif
    default:
        printf("Invalid Cmd:%d\n", iCmd);
        break;
    }

    static int iCount = 0;
    if ((iCount++) % 1000 == 0)
        Disp_Mem();
    return;
}

void esp32_Start_Server()
{
#ifdef WIN32
#define SETTING_PATH "D:\\Samp\\Rich_CV\\esp32\\Setting_Server.ini"
#else
#define SETTING_PATH "/littlefs/Setting_Server.ini"
#endif
    Init_Setting_Server oSetting;
    int iResult = bLoad_Setting(SETTING_PATH, &oSetting);
    if (iResult)
        Listen((char*)oSetting.m_IP, oSetting.m_iPort, (void*)Distribute_Cmd);
    else
        Listen((char*)"127.0.0.1", 10001, (void*)Distribute_Cmd);

    return;
}
