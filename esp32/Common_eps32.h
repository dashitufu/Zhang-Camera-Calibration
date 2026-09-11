#include "image.h"
#ifndef  WIN32
	#include <esp_camera.h>
#include <driver/mcpwm.h>
#endif // ! WIN32

#ifdef ESP32
#include <driver/spi_master.h>
#include <U8g2lib.h>
typedef struct TFT_Setting {	//TFT 1.9寸小屏用
	unsigned int LCD_W, LCD_H, XOFF, YOFF;
	int _scl, _sda, _dc, _cs;
	unsigned char* RGB[3];   // R,G,B 各 LCD_W*LCD_H 字节 (用户直接写)

	spi_device_handle_t hSPI;
	size_t   rgbSz;
	uint8_t* fb;            // 合并缓冲 (内部 RAM + DMA), 仅 Flush 内部用
	size_t   fbSz;
	u8g2_t   u8g2;           // U8g2 文本引擎 (中文)
	uint8_t* u8g2_buf;      // U8g2 画布缓冲
	unsigned int U2_W;       // 画布宽, = ceil(LCD_W/8)*8
	unsigned int U2_TILES;   // 画布高度 tile 数 = ceil(LCD_H/8)
	u8x8_display_info_t di;  // U8g2 显示信息 (由 Init_TFT 按几何填入)
}TFT_Setting;

#endif

typedef struct Init_Setting_Server {
	char m_IP[16];	//15字节加'\0'
	int m_iPort;	//端口

	char wifi_User_Name[32];	//wifi 用户名/口令
	char wifi_Password[16];

	int m_iCam_w, m_iCam_h;		//相机w,h
}Init_Setting_Server;

typedef struct Init_Setting_Client {
	char m_IP[16];	//15字节加'\0'
	int m_iPort;	//端口

}Init_Setting_Client;

typedef struct ESP32_Env {
	//unsigned int m_iMem_Size;
	//char WIFI_usr[16], WIFI_pwd[16];
	int m_iCam_Hor_Channel, //水平旋转通道号，
		m_iCam_Ver_Channel; //垂直旋转通道号
	int m_fCur_Hor_Angle,	//当前电机的水平角度
		m_fCur_Ver_Angle;	//当前电机的垂直角度
#ifdef ESP32
	TFT_Setting m_oTFT_Setting;
#endif 

}ESP32_Env;

extern ESP32_Env oESP32_Env;

#ifndef  WIN32
	#define Sleep           delay
	camera_fb_t* pCapture(unsigned long long* piTime_Stamp = NULL);
	void Free_Frame(camera_fb_t* poFrame);
	int Capture_5640(unsigned char** ppBuffer, int* piSize, unsigned long long* ptTime_Stamp = NULL);
#endif // ! WIN32

int bInit_Cam_5640(int w, int h);
void Free_Cam();
int bSave_Raw_Data_eps32(const char* pcFile, void* pBuffer, int iSize);
void Send_Data();
int iRecv_Packet(void* pBuffer, int iSize);
int iSend_Packet(void* pBuffer, int iSize);
int bSend_Data(void* pBuffer, int iSize);
int bInit_Wifi(const char User_Name[], const char Password[]);
int bInit_FS();
int bChange_Frame_Size(int w, int h);
int bInit_esp32_Env(const char Wifi_Usr[], const char Wifi_Pwd[], int iCam_w, int iCam_h, int iMem_Size = 8000000,
	int iCam_Hor_Pin = 20, int iCam_Hor_Channel = 6,
	int iCam_Ver_Pin = 14, int iCam_Ver_Channel = 7);
int bLoad_Setting(const char* pcFile, Init_Setting_Server* poSetting);
int bLoad_Setting(const char* pcFile, Init_Setting_Client* poSetting);
void esp32_Start_Server();
int iAngle_To_Signal(float fAngle, float fLeast_Time = 0.5, float fMost_Time = 2.5, int iFreq = 50, int Qt = 4096);

/*******************************sg90舵机控制**************************/
void Motor_Rotate(float fAngle, int iChannel);
void Motor_Rotate(float fAngle, int iChannel);
void Init_Motor(int iChannel, int iPin, int iFreq = 50, int iBits = 12);
void Motor_Rotate_Hor(float fAngle);
void Motor_Rotate_Ver(float fAngle);
void Motor_Rotate_Hor_Inc(float fDelta);
void Motor_Rotate_Ver_Inc(float fDelta);
/*******************************sg90舵机控制**************************/

/*******************************ST7789 TFT显示屏**********************/
#ifdef ESP32
void Init_TFT(TFT_Setting* poSetting, int w, int h,
	int iPin_scl, int iPin_sda, int iPin_dc, int iPin_cs);
void Textout_Ex(TFT_Setting* p, short x0, short y0, short x1, short y1,
	const char* s, unsigned char R, unsigned char G, unsigned char B);
void Flush(TFT_Setting* poSetting=NULL);
//void Flush_TFT();
void Textout(TFT_Setting* poSetting, uint16_t x, uint16_t yTop, uint16_t maxW,
	const char* s, unsigned char R, unsigned char G, unsigned char B);
void Textout(int x, int y, const char* str, unsigned char R = 255, unsigned char G = 255, unsigned char B = 255);
Image Get_Screen();
#endif
/*******************************ST7789 TFT显示屏**********************/

//JPEG Decoder
int Decode_JPG(const char File[], Image* poImage);
int Decode_JPG(unsigned char Buffer[], int iSize, Image* poImage);
int Decode_Resize_JPG(unsigned char* pBuffer, int iSize,
	int bGray, Image oImage);
