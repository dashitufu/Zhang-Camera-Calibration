//搞个内存占用少点的Sift，相当于原算法的1/3
#include "immintrin.h"
//以下三个头文件是必须的，故此要加上相应的.cpp一起编译
#include "Common.h"		
#include "sift.h"
#include "Image.h"
using namespace std;

#define VL_SIFT_BILINEAR_ORIENTATIONS 1
typedef struct Sift_Keypoint {//关键点
	short ix, iy;		// x,y
	char is;			//第几层
	char o;				//Current octave，第几轮
	float x, y, s;		//经过泰勒修正后的小数值
	float sigma;		
}Sift_Keypoint;

typedef struct Sift_Feature_Level {
	Sift_Feature* m_pFeature;
	Sift_Keypoint* m_pKeypoint;
	int m_iFeature_Count, m_iKeypoint_Count;
}Sift_Feature_Level;

typedef struct Sift {
	int O;               //一共可以搞多少轮，好像没有必要留
	int S;               //每轮多少层，也是可以设置
	int o_min;           //可以-1，也可以更高一级。比如4K的情况下没有必要搞那么大
	int s_min;           /**< minimum level index. */
	int s_max;           /**< maximum level index. */
	int o_cur;           /**< current octave. */

	//分析结果，一共有6层多分辨核高斯滤波，5层DoG, 3层梯度
	//故此，按理说内存分配3层DoG,3层Octave即可
	//尝试以下内存安排
	//|  /   | G_0 | DoG_0 | G_1 | DoG_1 | G_2
	//|DoG_2 | G_3 | DoG_3 | G_4 | DoG_4 | G_5
	float* m_pDoG[3];		//current DoG data, DoG= difference of Gaussians 高斯差
	//注意，在m_pDoG与高斯图之间还要加上一行，用作最后一行的缓冲
	float* m_pGauss[3];		//装不同分辨率的高斯图。为避免二义性，将原来octave改名

	float* m_pGrad;			//重新安排内存试一下，m_pGrad与Octave DoG共用
	float* m_pTemp;			//对应 f->temp，重用当层的m_pDoG

	int grad_o;					//GSS gradient data octave.
	int m_iWidth, m_iHeight;	//源图的长宽

	//以下几个Sigma是关于高斯图的模糊程度，具体再代码中，后买你再总结
	float sigman;       //nominal image smoothing. 
	float sigma0;       //smoothing of pyramid base. 
	float sigmak;       //k-smoothing 
	float dsigma0;      //delta-smoothing. 

	//float* Gauss_Filter;  /**< current Gaussian filter */
	//float gaussFilterSigma;   /**< current Gaussian filter std */
	//int gaussFilterWidth;  /**< current Gaussian filter width */

	int octave_width;    // current octave width
	int octave_height;   // current octave height.

	int keys_res;        // size of the keys buffer. 

	float peak_thresh;  // peak threshold. 
	float edge_thresh;  // edge threshold. 
	float norm_thresh;  // norm threshold. 
	float magnif;       // magnification factor. 
	float windowSize;   // size of Gaussian window (in spatial bins) 

	int nkeys;           // Keypoint Count
	Sift_Keypoint* m_pKey_Point;	//Keypoint只是候选点，并不等于Feature

	int m_iMax_Feature_Count;
	int m_iFeature_Count;			//最终的Feature Count
	//此处存放最终的整数形式Feature
	Sift_Feature* m_pFeature;		//最终的Feature所在，要的只是这一点点结果

	unsigned char* m_pBuffer_Start;	//每一轮内存开始之处
}Sift;

typedef struct Neighbour_Item_1 {
	int m_iIndex;
	unsigned int m_iDistance;
}Neighbour_Item_1;

typedef struct Neighbour_K {
	Neighbour_Item_1 m_Buffer[2];
}Neighbour_K;

void SB_sift()
{
	Sift_Match_2_Image("", "", (float(**)[2])NULL, (float(**)[2])NULL, NULL);
	Sift_Match_2_Image("", "", (double(**)[2])NULL, (double(**)[2])NULL, NULL);

	Shrink_Match_Point((float((**)[2]))NULL, (float((**)[2]))NULL, NULL, 0);
	Shrink_Match_Point((double((**)[2]))NULL, (double((**)[2]))NULL, NULL, 0);
}

int iGet_Sift_Match_Size(Sift_Image Sift_Image_Arr[], int iImage_Count)
{//图匹配阶段，预计峰值大概要用多少内存
	int i, iSize;
	//第一部分，分配Match, 一共分走了 iImage_Count*iImage_Count*sizeof(Sift_Match_Item)
	iSize = iImage_Count * iImage_Count * sizeof(Sift_Match_Item);

	//第二部分，再分配一次Desc, 整齐点以便于后面提速
	for (i = 0; i < iImage_Count; i++)
		iSize += Sift_Image_Arr[i].m_iCount * 128;

	//第三部分，Match， 一共有 iImage_Count*(iImage_Count-1)/2 组比较，大概开个空间
	iSize += ALIGN_SIZE_128(iImage_Count * (MAX_KEY_POINT_COUNT * 4));
	iSize += 128;	//留有余地供其对齐
	return iSize;
}
int iGet_Sift_Detect_Size(int iWidth, int iHeight,int o_min)
{//检测特征点阶段，预计算一共要用多少内存
//o_min: 必须要考虑进来，一旦不是-1开始，内存就不一样
	unsigned int iSize=0, iSize_1;
	//Feature
	//iSize = (MAX_KEY_POINT_COUNT * 2 * (iWidth * iHeight / 307200))* sizeof(Sift_Feature);
	iSize = MAX_KEY_POINT_COUNT * 2 * sizeof(Sift_Feature);
	iSize = ALIGN_SIZE_128(iSize);
	//Keypoint
	iSize += MAX_KEY_POINT_COUNT * sizeof(Sift_Keypoint);
	iSize = ALIGN_SIZE_128(iSize);

	if (o_min<0)
	{
		iWidth <<= -(o_min);
		iHeight <<= -(o_min);
	}else if(o_min>0)
	{
		iWidth >>= o_min;
		iHeight >>= o_min;
	}
	iSize_1 = iWidth * iHeight * sizeof(float);
	iSize_1 = ALIGN_SIZE_128(iSize_1);

	//Octave
	iSize += iSize_1 * 3;

	//DoG，最小三层
	iSize += iSize_1 * 3 + iWidth*3*sizeof(float);

	//多加一行，用来算Grad最后一行
	//iSize += iWidth * 2 * sizeof(float);

	//grad 与Temp 共享
	//iSize += iSize_1 * 2;
	iSize = ALIGN_SIZE_128(iSize) + 128;	//留有余地128字节

	return iSize;
}

static void Init_Sift(Sift* poSift, int iOctave_Count, int iLayer_Count, 
	int o_min, int iWidth, int iHeight, unsigned char* pBuffer)
{//此处应该一次性分配好所有的内存
	Sift oSift;
	int iSize;
	//fast_expn_init();

	oSift.m_iWidth = iWidth;
	oSift.m_iHeight = iHeight;

	/*int w, h;
	if (o_min == -1)
	{
		w = iWidth << 1;
		h = iHeight << 1;
	}else
	{
		w = iWidth >> o_min;
		h = iHeight >> o_min;
	}	
	iSize = w * h;*/

	oSift.peak_thresh = 2.f / 300.f;
	oSift.edge_thresh = 10.f;

	oSift.O = 4;
	oSift.S = 3;
	oSift.o_min = o_min;
	oSift.s_min = -1;
	oSift.s_max = iLayer_Count + 1;
	oSift.o_cur = oSift.o_min;

	oSift.sigman = 0.5;
	oSift.sigmak = (float)pow(2.0, 1.0 / iLayer_Count);
	oSift.sigma0 = 1.6f * oSift.sigmak;
	oSift.dsigma0 = (float)(oSift.sigma0 * sqrt(1.0f - 1.0f / (oSift.sigmak * oSift.sigmak)));

	oSift.octave_width = 0;
	oSift.octave_height = 0;

	oSift.nkeys = 0;

	oSift.norm_thresh = 0.0;
	oSift.magnif = 3.0;
	oSift.windowSize = 4 / 2;

	//安排内存
	unsigned char* pCur = pBuffer;
	//oSift.m_iMax_Feature_Count = MAX_KEY_POINT_COUNT *2 * (iWidth * iHeight / 307200);
	oSift.m_iMax_Feature_Count = MAX_KEY_POINT_COUNT * 2;
	iSize = oSift.m_iMax_Feature_Count * sizeof(Sift_Feature);
	iSize = ALIGN_SIZE_128(iSize);

	oSift.m_iFeature_Count = 0;
	oSift.m_pFeature = (Sift_Feature*)pCur;
	pCur += iSize;

	iSize = MAX_KEY_POINT_COUNT * sizeof(Sift_Keypoint);
	iSize = ALIGN_SIZE_128(iSize);
	oSift.m_pKey_Point = (Sift_Keypoint*)pCur;
	pCur += iSize;

	oSift.m_pBuffer_Start = pCur;
	*poSift = oSift;
}

void Arrange_Mem(Sift* poSift, int w, int h, unsigned char* pCur)
{//对本轮进行 octave, DoG, grad进行内存安排
	Sift oSift = *poSift;
	int i, iSize , iSize_1;
	//unsigned char* pOrg = pCur;

	iSize = ALIGN_SIZE_128(w * h * sizeof(float));
	iSize_1 = ALIGN_SIZE_128(iSize + w * sizeof(float));

	//高斯图与DoG梅花间竹,尝试以下内存安排
	//|  /   | G_0 | DoG_0 | G_1 | DoG_1 | G_2
	//|DoG_2 | G_3 | DoG_3 | G_4 | DoG_4 | G_5
	if (oSift.o_cur == oSift.o_min)
	{
		for (i = 0; i < 3; i++)
		{
			oSift.m_pDoG[(i + 2) % 3] = (float*)pCur;
			pCur += iSize_1;
			oSift.m_pGauss[i] = (float*)pCur;
			pCur += iSize;
		}
	}else
	{//原来的[0]图做了一次的下采样,放在了[1],所以，循环左移一次
		float *pTemp[2] = { oSift.m_pGauss[0],oSift.m_pDoG[0] };
		for (i = 1; i < 3; i++)
		{
			oSift.m_pGauss[i-1] = oSift.m_pGauss[i];
			oSift.m_pDoG[i-1] = oSift.m_pDoG[i];
		}
		oSift.m_pGauss[2] = pTemp[0];
		oSift.m_pDoG[2] = pTemp[1];
	}
	//bSave_Image("c:\\tmp\\1.bmp", oSift.m_pGauss[0], oSift.octave_width, oSift.octave_height);

	////最后一行
	//oSift.m_pLast_Grad_Line = (float*)pCur;
	//pCur += oSift.octave_width * 2* sizeof(float);

	//oSift.m_pTemp = oSift.m_pDoG[0];	//Temp与当前重用
	//oSift.m_pGrad = oSift.m_pTemp;
	//pCur += iSize * 2;

	*poSift = oSift;
}
#define Get_w_h(iWidth, iHeight,  w,  h, o_cur) \
{ \
	if (o_cur < 0) \
	{	\
		w = iWidth << (-o_cur);		\
		h = iHeight << (-o_cur);	\
	}else \
	{ \
		w = iWidth >> o_cur; \
		h = iHeight >> o_cur; \
	} \
}
void Resize_Image_x2(float* pSource, int iWidth, int iHeight, float* pDest)
{//图像按边长扩大一倍
	//先行扩展
	int y, x, iWidth_x2 = iWidth << 1, iHeight_x2 = iHeight << 1,
		iHeight_x2_Minus_1 = iHeight_x2 - 1,
		iWidth_Minus_1 = iWidth - 1;
	float* pSource_Cur, * pDest_Cur;
	for (pSource_Cur = pSource, y = 0; y < iHeight; y++/*pSource_Cur+=iWidth*/)
	{
		pDest_Cur = &pDest[(y << 1) * iWidth_x2];
		for (x = 0; x < iWidth_Minus_1; x++, pSource_Cur++, pDest_Cur += 2)
		{
			pDest_Cur[0] = pSource_Cur[0];
			pDest_Cur[1] = (pSource_Cur[0] + pSource_Cur[1]) * 0.5f;
		}
		pDest_Cur[0] = pDest_Cur[1] = *pSource_Cur++;
	}

	//再对pDest再做一次的插值
	pDest_Cur = &pDest[iWidth_x2];
	for (y = 1; y < iHeight_x2_Minus_1; y += 2)
	{
		for (x = 0; x < iWidth_x2; x++, pDest_Cur++)
			pDest_Cur[0] = (pDest_Cur[-iWidth_x2] + pDest_Cur[iWidth_x2]) * 0.5f;
		pDest_Cur += iWidth_x2;
	}
	for (x = 0; x < iWidth_x2; x++, pDest_Cur++)
		*pDest_Cur = pDest_Cur[-iWidth_x2];
	return;
}
void Resize_Image_Half(float* pSource, int iWidth, int iHeight, float* pDest)
{//边长减半
	int iWidth_Half = iWidth >> 1,
		iHeight_Half = iHeight >> 1;
	int y;
	float* pSource_Cur, * pDest_Cur = pDest, * pDest_End;
	for (y = 0; y < iHeight_Half; y++)
	{
		pSource_Cur = &pSource[y * iWidth << 1];
		pDest_End = pDest_Cur + iWidth_Half;
		for (; pDest_Cur < pDest_End; pDest_Cur++, pSource_Cur += 2)
			*pDest_Cur = *pSource_Cur;
	}
	return;
}

static void Sift_Dectact_3(Sift* poSift, int iCur_Layer)
{//这个和原来的Sift_Detect不同，做三层DoG
	Sift oSift = *poSift;
	int i, iSize;

	//生成一个sigma
	float fSigma_a, fSigma_b;
	//从以下代码看出高斯核的大小与图像大小无关
	if (oSift.o_cur == oSift.o_min)
	{//相当于Process_First_Octave
		fSigma_a = oSift.sigma0 * (float)pow(oSift.sigmak, oSift.s_min);
		fSigma_b = oSift.sigman * (float)pow(2.0f, -oSift.o_min);
	}else
	{//相当于Process_Next_Octave
		int s_best = min(oSift.s_min + oSift.S, oSift.s_max);
		fSigma_a = (float)(oSift.sigma0 * pow((float)oSift.sigmak, (float)oSift.s_min));
		fSigma_b = (float)(oSift.sigma0 * pow((float)oSift.sigmak, (float)(s_best - oSift.S)));
	}
	float sd;
	int r;
	float* pFilter;

	//先搞第0层Octave，重用这一层对应的DoG
	oSift.m_pTemp = oSift.m_pDoG[0];
	if (fSigma_a > fSigma_b)
	{
		sd = (float)sqrt(fSigma_a * fSigma_a - fSigma_b * fSigma_b);
		//可见此处是r随sigma变化而变化
		r = (int)max((float)ceil(4.0 * sd), 1.f);
		Gen_Gauss_Filter(r, sd, &pFilter);
		Gauss_Filter_AVX512(oSift.m_pGauss[0], oSift.octave_width, oSift.octave_height, oSift.m_pGauss[0], oSift.m_pTemp, r, pFilter);
		free(pFilter);
	}
	union {
		struct {
			float* pt, * pOctave_0, * pOctave_1;
		};
		__m256i Addr_3;
	};
	float* pEnd;
	iSize = oSift.octave_width * oSift.octave_height;

	for (i = 0; i < 2; i++)
	{//连做两层的Octave和DoG
		pOctave_0 = oSift.m_pGauss[i];
		pOctave_1 = oSift.m_pGauss[1 + i];
		oSift.m_pTemp = oSift.m_pDoG[1 + i];
		sd = oSift.dsigma0 * (float)pow(oSift.sigmak, i);

		r = (int)max((float)ceil(4.0 * sd), 1.f);
		Gen_Gauss_Filter(r, sd, &pFilter);
		//从下面一层生成上面一层
		Gauss_Filter_AVX512(pOctave_0, oSift.octave_width, oSift.octave_height, pOctave_1, oSift.m_pTemp, r, pFilter);
		free(pFilter);

		//搞第0层DoG
		pt = oSift.m_pDoG[i];
		pEnd = pt + iSize;
		for (; pt < pEnd; Addr_3 = _mm256_add_epi64(Addr_3, _mm256_set1_epi64x(16 * 4))   /*pt += 16, pOctave_0 += 16, pOctave_1 += 16*/)
			*(__m512*)pt = _mm512_sub_ps(*(__m512*)pOctave_1, *(__m512*)pOctave_0);
	}
}

static void Get_Keypoint_1(Sift* poSift, int iDoG_Mid, int s, Sift_Feature_Level* poFeature_Level)
{//注意： iDoG_Mid指的是本次处理DoG中间层在oSift.m_pDoG中逻辑位置
//			s:对应sift源代码中的s
	Sift oSift = *poSift;
	int iSize = oSift.octave_width * oSift.octave_height;
	int const    xo = 1;					/* x-stride */
	int const    yo = oSift.octave_width;	/* y-stride */
	iSize = ALIGN_SIZE_128(iSize)*2;
	//此处可能要改，不能用原来的方法
	int iDoG_Mid_Index = iDoG_Mid % 3;
	int so_Next;	// = iDoG_Mid_Index == 2 ? -2 * iSize : iSize;
	int so_Pre;		// = iDoG_Mid_Index == 0 ? iSize * 2 : -iSize;
	float tp = oSift.peak_thresh * 0.8f;
	int x_End_Align_16 = 1 + (((oSift.octave_width - 2) >> 4) << 4),
		iMask = (1 << ((oSift.octave_width - 2) & 0xF)) - 1;
	int y, x;
	int i, j, k, ii, jj;
	float* pt;

	so_Next = (int)(oSift.m_pDoG[(iDoG_Mid + 1) % 3] - oSift.m_pDoG[iDoG_Mid_Index]);
	so_Pre = (int)(oSift.m_pDoG[(iDoG_Mid - 1) % 3] - oSift.m_pDoG[iDoG_Mid_Index]);

	//以DoG[1]层为中心层，通过26邻域确定最初的Key_Point
	pt = &oSift.m_pDoG[iDoG_Mid_Index][1 * oSift.octave_width + 1];
	Sift_Keypoint oKey_Point, * pPoint = poFeature_Level->m_pKeypoint;	//oSift.m_pKey_Point + oSift.nkeys;
	int nkeys = 0;
	int Offset_26[] = { -xo, xo,
						-yo - xo, -yo, -yo + xo,
						yo - xo,	yo, yo + xo,

						-xo + so_Pre, so_Pre, xo + so_Pre,
						-yo - xo + so_Pre, -yo + so_Pre, -yo + xo + so_Pre,
						yo - xo + so_Pre,	yo + so_Pre, yo + xo + so_Pre,

						-xo + so_Next, so_Next, xo + so_Next,
						-yo - xo + so_Next, -yo + so_Next, -yo + xo + so_Next,
						yo - xo + so_Next,	yo + so_Next, yo + xo + so_Next
	};

	__m512 v_16;

#define CHECK_NEIGHBORS_1(CMP,SGN)   (    \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[0]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[1]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[2]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[3]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[4]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[5]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[6]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[7]),CMP) && \
\
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[8]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[9]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[10]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[11]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[12]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[13]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[14]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[15]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[16]),CMP)  &&\
\
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[17]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[18]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[19]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[20]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[21]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[22]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[23]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[24]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[25]),CMP) && \
_mm512_cmp_ps_mask(v_16, _mm512_set1_ps(SGN tp),CMP)  \
		) \

#define CHECK_NEIGHBORS(CMP,SGN)   (    \
_mm512_cmp_ps_mask(v_16, _mm512_set1_ps(SGN tp),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[0]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[1]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[2]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[3]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[4]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[5]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[6]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[7]),CMP) & \
\
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[8]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[9]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[10]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[11]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[12]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[13]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[14]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[15]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[16]),CMP)  &\
\
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[17]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[18]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[19]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[20]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[21]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[22]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[23]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[24]),CMP) & \
_mm512_cmp_ps_mask(v_16, _mm512_loadu_ps(pt + Offset_26[25]),CMP)  \
		) \

	int iResult;
	for (y = 1; y < oSift.octave_height - 1; y++)
	{
		pt = &oSift.m_pDoG[iDoG_Mid_Index][y * oSift.octave_width + 1];
		for (x = 1; x < x_End_Align_16; x += 16, pt += 16)
		{
			//v_16 = *(__m512*)pt;
			v_16 = _mm512_loadu_ps(pt);

			if (/*iResult =*/ CHECK_NEIGHBORS_1(_CMP_GT_OS, +) || CHECK_NEIGHBORS_1(_CMP_LT_OS, -))
			{
				iResult = CHECK_NEIGHBORS(_CMP_GT_OS, +) | CHECK_NEIGHBORS(_CMP_LT_OS, -);
				for (i = 0; iResult; i++)
				{
					if (iResult & 1)
					{
						oKey_Point.ix = x + i;
						oKey_Point.iy = y;
						oKey_Point.is = s;
						pPoint[nkeys] = oKey_Point;
						nkeys++;
					}
					iResult >>= 1;
				}
			}
		}

		//搞余数
		//v_16 = *(__m512*)pt;
		v_16 = _mm512_loadu_ps(pt);
		iResult = (CHECK_NEIGHBORS(_CMP_GT_OS, +) | CHECK_NEIGHBORS(_CMP_LT_OS, -)) & iMask;
		for (i = 0; iResult; i++)
		{
			if (iResult & 1)
			{
				//if (oSift.o_cur == 1)
					//printf("here");
				oKey_Point.ix = x + i;
				oKey_Point.iy = y;
				oKey_Point.is = s;
				pPoint[nkeys] = oKey_Point;
				nkeys++;
			}
			iResult >>= 1;
		}
	}
	
	//对上一步的keypoints进一步筛选
	int dx, dy, iter;
	float Dx = 0, Dy = 0, Ds = 0, Dxx = 0, Dyy = 0, Dss = 0, Dxy = 0, Dxs = 0, Dys = 0;
	float A[3][3], b[3];	//正是求解Ax=b，可以直接调用解线性方程
	int iValid_Point_Count = 0;
	float* pDoG = oSift.m_pDoG[iDoG_Mid_Index];
	float xper = (float)pow(2.0, oSift.o_cur);

	for (k = 0; k < nkeys; k++)
	{
		oKey_Point = pPoint[k];
		//先来个一阶导
		Dx = 0, Dy = 0, Ds = 0, Dxx = 0, Dyy = 0, Dss = 0, Dxy = 0, Dxs = 0, Dys = 0;
		dx = dy = 0;
		x = oKey_Point.ix;
		y = oKey_Point.iy;

		for (iter = 0; iter < 5; iter++)
		{//感觉迭代多余，也有可能为了不落在局部极小点或着噪声影响等等原因，再继续搜索一下
			//而且一点的x,y各自方向上最大跨度不可能大于1，所以在9邻域上游走
			x += dx;
			y += dy;
			pt = pDoG + xo * x + yo * y;

			//显然就是求一阶偏导，雅比克
			Dx = 0.5f * (pt[1] - pt[-1]);
			Dy = 0.5f * (pt[oSift.octave_width] - pt[-oSift.octave_width]);
			Ds = 0.5f * (pt[so_Next] - pt[so_Pre]);

			//求一个Hesse矩阵，由二阶偏导组成
			Dxx = (pt[1] + pt[-1] - 2.0f * (*pt));
			Dyy = (pt[oSift.octave_width] + pt[-oSift.octave_width] - 2.0f * (*pt));
			Dss = (pt[so_Next] + pt[so_Pre] - 2.0f * (*pt));

			Dxy = 0.25f * (pt[oSift.octave_width + 1] + pt[-oSift.octave_width - 1] - pt[-oSift.octave_width + 1] - pt[oSift.octave_width - 1]);	//  at(+1, +1, 0) + at(-1, -1, 0) - at(-1, +1, 0) - at(+1, -1, 0));
			Dxs = 0.25f * (pt[so_Next + 1] + pt[so_Pre - 1] - pt[so_Pre + 1] - pt[so_Next - 1]);	//  (at(+1, 0, +1) + at(-1, 0, -1) - at(-1, 0, +1) - at(+1, 0, -1));
			Dys = 0.25f * (pt[so_Next + oSift.octave_width] + pt[so_Pre - oSift.octave_width] - pt[so_Pre + oSift.octave_width] - pt[so_Next - oSift.octave_width]);	//at(0, +1, +1) + at(0, -1, -1) - at(0, -1, +1) - at(0, +1, -1));

			//搞个对称阵hesse
			A[0][0] = Dxx;
			A[1][1] = Dyy;
			A[2][2] = Dss;
			A[0][1] = A[1][0] = Dxy;
			A[0][2] = A[2][0] = Dxs;
			A[1][2] = A[2][1] = Dys;

			b[0] = -Dx;
			b[1] = -Dy;
			b[2] = -Ds;

			//此处为泰勒修正。以上找到的关键点并非真正的极值点，在插值的条件下，极值点
			//应该是子像素，也就是落在以上关键点邻域上。故此要找在关键点周围做一个泰勒展开
			//得到一个近似 f(x)= f(x0) + J*(delta x) + 1/2(delta x') * H * (delta x)
			//对 f(x) 求极值 min f(x) 就是再来一次求偏导 J + H*(delta x) ，使其=0, 求 delta x
			//所以有 H*x= -J ,解出 x就是所求的 delta x
			for (j = 0; j < 3; j++)
			{//j为列
				float maxa = 0;
				float maxabsa = 0;
				int    maxi = -1;
				float tmp;

				/* look for the maximally stable pivot */
				for (i = j; i < 3; i++)
				{//显然找第j列最大绝对值，列主元
					float a = A[i][j];
					float absa = abs(a);
					if (absa > maxabsa)
					{
						maxa = a;
						maxabsa = absa;
						maxi = i;
					}
				}

				//A为二阶导矩阵，如果二阶导矩阵存在一列<eps则退出？表示这是这个方向上存在极值点?
				if (maxabsa < 1e-10f)
				{
					b[0] = 0;
					b[1] = 0;
					b[2] = 0;
					break;
				}
				i = maxi;

				//这里越看越象列主元法解线性方程 Ax=b
				/* swap j-th row with i-th row and normalize j-th row */
				for (jj = j; jj < 3; ++jj)
				{//将最大元素所在行与第j行交换，并且规格化
					tmp = A[i][jj]; A[i][jj] = A[j][jj]; A[j][jj] = tmp;
					A[j][jj] /= maxa;
				}
				tmp = b[j]; b[j] = b[i]; b[i] = tmp;
				b[j] /= maxa;

				for (ii = j + 1; ii < 3; ++ii)
				{
					float x = A[ii][j];
					for (jj = j; jj < 3; ++jj)
						A[ii][jj] -= x * A[j][jj];
					b[ii] -= x * b[j];
				}
			}

			// backward substitution 回代，瞅着眼熟
			for (i = 2; i > 0; --i)
			{
				float x = b[i];
				for (ii = i - 1; ii >= 0; --ii)
					b[ii] -= x * A[ii][i];
			}

			dx = ((b[0] > 0.6 && x < oSift.octave_width - 2) ? 1 : 0)
				+ ((b[0] < -0.6 && x > 1) ? -1 : 0);

			dy = ((b[1] > 0.6 && y < oSift.octave_height - 2) ? 1 : 0)
				+ ((b[1] < -0.6 && y > 1) ? -1 : 0);

			if (dx == 0 && dy == 0)
				break;
		}

		/* check threshold and other conditions */
		{
			float val = *pt + 0.5f * (Dx * b[0] + Dy * b[1] + Ds * b[2]);
			//这句的意义待分析
			float score = (Dxx + Dyy) * (Dxx + Dyy) / (Dxx * Dyy - Dxy * Dxy);
			float xn = x + b[0];
			float yn = y + b[1];
			float sn = s + b[2];
			// |H|=Dxx * Dyy - Dxy * Dxy ,

			int good = abs(val) > oSift.peak_thresh &&
				score < (oSift.edge_thresh + 1)* (oSift.edge_thresh + 1) / oSift.edge_thresh &&
				score >= 0 &&
				abs(b[0]) < 1.5 &&
				abs(b[1]) < 1.5 &&
				abs(b[2]) < 1.5 &&
				xn >= 0 &&
				xn <= oSift.octave_width - 1 &&
				yn >= 0 &&
				yn <= oSift.octave_height - 1 &&
				sn >= oSift.s_min &&
				sn <= oSift.s_max;

			if (good)
			{
				/*if (oSift.o_cur == 1)
					printf("here");*/
					//printf("i:%d y:%d x:%d\n", iValid_Point_Count, oKey_Point.iy, oKey_Point.ix, oKey_Point.ix);
				oKey_Point.ix = x;
				oKey_Point.iy = y;
				oKey_Point.is = s;
				oKey_Point.o = oSift.o_cur;
				oKey_Point.s = sn;
				oKey_Point.x = xn * xper;
				oKey_Point.y = yn * xper;
				oKey_Point.sigma = oSift.sigma0 * (float)pow(2.0, sn / oSift.S) * xper;
				pPoint[iValid_Point_Count++] = oKey_Point;
			}
		} /* done checking */
	}

	poFeature_Level->m_iKeypoint_Count = iValid_Point_Count;
	*poSift = oSift;

	return;
#undef CHECK_NEIGHBORS
}

void _Update_Gradiant_AVX512(float* src, float* grad, int w, int h)
{//个人认为此算法可以更简洁，从sift的特征点看，角点的意义最大，边缘点和孤立点都丢弃，故此落在
	//图像上下四边上的点基本上不会是角点。如果后面有充分的数据验证这个猜想，则边上的数值可以不算
	float* pSource, * pSource_1, * pEnd_1, * pEnd/*,*pEnd_Align_16*/;
	float* pGrad, * pGrad_1;
	__m512 gx, gy, Value;
	__m512i vIndex_Grad;
	int i, iResult, iValue, h_Minus_1 = h - 1, w_x2 = w * 2;
	for (i = 0; i < 16; i++)
		((int*)&vIndex_Grad)[i] = i * 8;
		//vIndex_Grad.m512i_i32[i] = i * 8;

	//先不管上下左右角4个点，有必要省略掉

	//第一点，没什么好搞了
	/*gx.m512_f32[0] = src[1] - src[0];
	gy.m512_f32[0] = src[w] - src[0];
	grad[0] = (float)sqrt(gx.m512_f32[0] * gx.m512_f32[0] + gy.m512_f32[0] * gy.m512_f32[0]);
	grad[1] = (float)atan(gy.m512_f32[0] / gx.m512_f32[0]);*/
	float fgx= src[1] - src[0],
		fgy= src[w] - src[0];
	grad[0] = (float)sqrt(fgx * fgx + fgy * fgy);
	grad[1] = (float)atan(fgy / fgx);
	if (grad[1] < 0)
		grad[1] += PI * 2.f;

	//搞第一行
	pGrad = grad + 2;
	pSource = src + 1;
	pEnd = src + w - 1;	//头尾两点不搞
	//pEnd_Align_16 = pSource + (unsigned int)((pEnd - pSource) & 0xFFFFFFF0);
	iValue = (pEnd - pSource) & 0xF;
	while (pSource < pEnd)
	{//放心推进，有余数无所谓，留有余地即可
		//gx = 0.5f * (src[+xo] - src[-xo]);
		gx = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + 1), *(__m512*)(pSource - 1)));
		//gy = src[+yo] - src[0];
		gy = _mm512_sub_ps(_mm512_loadu_ps(pSource + w), _mm512_loadu_ps(pSource));

		//sqrt(gx * gx + gy * gy)
		Value = _mm512_sqrt_ps(_mm512_add_ps(_mm512_mul_ps(gx, gx), _mm512_mul_ps(gy, gy)));
		_mm512_i32scatter_ps(pGrad, vIndex_Grad, Value, 1);

		//此处感觉可以简化 arctg的判断，值域为(-PI/2,PI/2);
		Value = _mm512_atan_ps(_mm512_div_ps(gy, gx));
		iResult = _mm512_cmp_ps_mask(Value, _mm512_set1_ps(0), _CMP_LT_OS);
		Value = _mm512_mask_add_ps(Value, iResult, Value, _mm512_set1_ps(PI * 2));
		_mm512_i32scatter_ps(pGrad + 1, vIndex_Grad, Value, 1);
		pSource += 16;
		pGrad += 32;
	}

	//第一行最后一点
	//gx = src[0] - src[-xo];
	//gy = src[+yo] - src[0];
	pSource = &src[w - 1];
	pGrad = &grad[w_x2 - 2];
	/*gx.m512_f32[0] = pSource[0] - pSource[-1];
	gy.m512_f32[0] = pSource[w] - pSource[0];
	pGrad[0] = (float)sqrt(gx.m512_f32[0] * gx.m512_f32[0] + gy.m512_f32[0] * gy.m512_f32[0]);
	pGrad[1] = (float)atan(gy.m512_f32[0] / gx.m512_f32[0]);*/
	fgx = pSource[0] - pSource[-1];
	fgy = pSource[w] - pSource[0];
	pGrad[1] = (float)sqrt(fgx * fgx + fgy * fgy);
	pGrad[1] = (float)atan(fgy / fgx);

	if (pGrad[1] < 0)
		pGrad[1] += PI * 2.f;

	//搞中间行
	pSource_1 = src + w + 1;
	pEnd_1 = src + h_Minus_1 * w;
	pGrad_1 = grad + w * 2 + 2;
	pEnd = pSource_1 + w - 2;
	//以下代码还有Bug, 以下循环中的最后一行还是会出现踩过界算错问题
	for (; pSource_1 < pEnd_1; pSource_1 += w, pGrad_1 += w_x2, pEnd += w)
	{
		pSource = pSource_1;
		pGrad = pGrad_1;

		//中间行第一点
		//gx = src[+xo] - src[0];
		//gy = 0.5f * (src[+yo] - src[-yo]);
		/*gx.m512_f32[0] = pSource[0] - pSource[-1];		
		gy.m512_f32[0] = 0.5f * (pSource[w - 1] - pSource[-1 - w]);
		pGrad[-2] = (float)sqrt(gx.m512_f32[0] * gx.m512_f32[0] + gy.m512_f32[0] * gy.m512_f32[0]);
		pGrad[-1] = (float)atan(gy.m512_f32[0] / gx.m512_f32[0]);*/
		fgx = pSource[0] - pSource[-1];
		fgy = 0.5f * (pSource[w - 1] - pSource[-1 - w]);
		pGrad[-2] = (float)sqrt(fgx * fgx + fgy * fgy);
		pGrad[-1] = (float)atan(fgy / fgx);
		if (pGrad[-1] < 0)
			pGrad[-1] += PI * 2.f;

		while (pSource < pEnd)
		{
			//gx = 0.5f * (src[+xo] - src[-xo]);	//根本没有按3x3模板搞，偷工减料做了一个一维的模板
			gx = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + 1), _mm512_loadu_ps(pSource - 1)));
			//gy = 0.5f * (src[+yo] - src[-yo]);	//同上
			gy = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + w), _mm512_loadu_ps(pSource - w)));

			//sqrt(gx * gx + gy * gy)
			Value = _mm512_sqrt_ps(_mm512_add_ps(_mm512_mul_ps(gx, gx), _mm512_mul_ps(gy, gy)));
			_mm512_i32scatter_ps(pGrad, vIndex_Grad, Value, 1);

			//此处感觉可以简化 arctg的判断，值域为(-PI/2,PI/2);
			Value = _mm512_atan_ps(_mm512_div_ps(gy, gx));
			iResult = _mm512_cmp_ps_mask(Value, _mm512_set1_ps(0), _CMP_LE_OS);
			Value = _mm512_mask_add_ps(Value, iResult, Value, _mm512_set1_ps(PI * 2));
			_mm512_i32scatter_ps(pGrad + 1, vIndex_Grad, Value, 1);

			pSource += 16;
			pGrad += 32;
		}

		//中间行最后一点
		pSource = &pSource_1[w - 2];
		pGrad = &pGrad_1[w_x2 - 4];
		//gx = src[0] - src[-xo];
		//gy = 0.5f * (src[+yo] - src[-yo]);
		/*gx.m512_f32[0] = pSource[0] - pSource[-1];
		gy.m512_f32[0] = 0.5f * (pSource[w] - pSource[-w]);
		pGrad[0] = (float)sqrt(gx.m512_f32[0] * gx.m512_f32[0] + gy.m512_f32[0] * gy.m512_f32[0]);
		pGrad[1] = (float)atan(gy.m512_f32[0] / gx.m512_f32[0]);*/
		fgx = pSource[0] - pSource[-1];
		fgy = 0.5f * (pSource[w] - pSource[-w]);
		pGrad[0] = (float)sqrt(fgx * fgx + fgy * fgy);
		pGrad[1] = (float)atan(fgy / fgx);

		if (pGrad[1] < 0)
			pGrad[1] += PI * 2.f;
	}

	//最后一行
	pGrad = pGrad_1;
	pSource = pSource_1;
	//pEnd = pSource_1 + w - 2;	//头尾两点不搞
	iValue = (((w - 2) >> 4) << 4);
	pEnd = pSource + iValue;
	iValue =(1<<( w - 2 - iValue))-1;

	//最后一行第一点
	//gx = src[+xo] - src[0];
	//gy = src[0] - src[-yo];
	/*gx.m512_f32[0] = pSource[0] - pSource[-1];
	gy.m512_f32[0] = pSource[-1] - pSource[-1 - w];
	pGrad[-2] = (float)sqrt(gx.m512_f32[0] * gx.m512_f32[0] + gy.m512_f32[0] * gy.m512_f32[0]);
	pGrad[-1] = (float)atan(gy.m512_f32[0] / gx.m512_f32[0]);*/
	fgx = pSource[0] - pSource[-1];
	fgy = pSource[-1] - pSource[-1 - w];
	pGrad[-2] = (float)sqrt(fgx * fgx + fgy * fgy);
	pGrad[-1] = (float)atan(fgy / fgx);
	if (pGrad[-1] < 0)
		pGrad[-1] += PI * 2.f;

	while (pSource < pEnd)
	{
		//gx = 0.5f * (src[+xo] - src[-xo]);
		gx = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + 1), _mm512_loadu_ps(pSource - 1)));
		//gy = src[0] - src[-yo];
		gy = _mm512_sub_ps(_mm512_loadu_ps(pSource), _mm512_loadu_ps(pSource - w));

		//sqrt(gx * gx + gy * gy)
		Value = _mm512_sqrt_ps(_mm512_add_ps(_mm512_mul_ps(gx, gx), _mm512_mul_ps(gy, gy)));
		_mm512_i32scatter_ps(pGrad, vIndex_Grad, Value, 1);

		//此处感觉可以简化 arctg的判断，值域为(-PI/2,PI/2);
		Value = _mm512_atan_ps(_mm512_div_ps(gy, gx));
		iResult = _mm512_cmp_ps_mask(Value, _mm512_set1_ps(0), _CMP_LT_OS);
		Value = _mm512_mask_add_ps(Value, iResult, Value, _mm512_set1_ps(PI * 2));
		_mm512_i32scatter_ps(pGrad + 1, vIndex_Grad, Value, 1);

		pSource += 16;
		pGrad += 32;
	}

	//最后一行余数
	gx = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + 1), _mm512_loadu_ps(pSource - 1)));
	gy = _mm512_sub_ps(_mm512_loadu_ps(pSource), _mm512_loadu_ps(pSource - w));

	Value = _mm512_sqrt_ps(_mm512_add_ps(_mm512_mul_ps(gx, gx), _mm512_mul_ps(gy, gy)));
	_mm512_mask_i32scatter_ps(pGrad,iValue, vIndex_Grad, Value, 1);

	//此处感觉可以简化 arctg的判断，值域为(-PI/2,PI/2);
	Value = _mm512_atan_ps(_mm512_div_ps(gy, gx));
	iResult = _mm512_cmp_ps_mask(Value, _mm512_set1_ps(0), _CMP_LT_OS);
	Value = _mm512_mask_add_ps(Value, iResult, Value, _mm512_set1_ps(PI * 2));
	_mm512_mask_i32scatter_ps(pGrad + 1,iValue, vIndex_Grad, Value, 1);

	//最后一行最后一点
	pSource = &pSource_1[w - 2];
	pGrad = &pGrad_1[w_x2 - 4];
	/*gx.m512_f32[0] = pSource[0] - pSource[-1];
	gy.m512_f32[0] = pSource[0] - pSource[-w];
	pGrad[0] = (float)sqrt(gx.m512_f32[0] * gx.m512_f32[0] + gy.m512_f32[0] * gy.m512_f32[0]);
	pGrad[1] = (float)atan(gy.m512_f32[0] / gx.m512_f32[0]);*/
	fgx = pSource[0] - pSource[-1];
	fgy = pSource[0] - pSource[-w];
	pGrad[0] = (float)sqrt(fgx * fgx + fgy * fgy);
	pGrad[1] = (float)atan(fgy / fgx);

	if (pGrad[-1] < 0)
		pGrad[-1] += PI * 2.f;

	////验算
	//FILE* pFile = fopen("c:\\tmp\\1.bin", "rb");
	//for (i = 0; i < w * h; i++)
	//{
	//	float Data[2];
	//	if (!fread(Data, 1, 2 * sizeof(float), pFile))
	//	{
	//		printf("error");
	//		break;
	//	}
	//	if (Data[0] != grad[i * 2] || Data[1] != grad[i * 2 + 1])
	//		printf("y:%d x:%d %f %f\n",i/w,i%w, Data[1], grad[i*2+1]);
	//}
	return;
}

int iCal_Key_Point_Oriatation(Sift* poSift, float Angle[4], Sift_Keypoint oKey_Point)
{//感觉此处是关键了
	float const winf = 1.5;
	float       xper = (float)pow(2.0, poSift->o_cur);

	int          w = poSift->octave_width;	//poSift->m_iWidth_x2;
	int          h = poSift->octave_height;	// poSift->m_iHeight_x2;
	int const    xo = 2;         /* x-stride */
	int const    yo = 2 * w;     /* y-stride */
	//int const    so = 2 * w * h; /* s-stride */
	float       x = oKey_Point.x / xper;
	float       y = oKey_Point.y / xper;
	float       sigma = oKey_Point.sigma / xper;

	int          xi = (int)(x + 0.5);
	int          yi = (int)(y + 0.5);
	int          si = oKey_Point.is;

	float const sigmaw = winf * sigma;
	int          W = (int)max((float)floor(3.0 * sigmaw), 1.f);
	int          nangles = 0;

	//enum { nbins = 36 };
	const int nbins = 36;
	float hist[nbins] = { 0 }, maxh;
	float const* pt;
	int xs, ys, iter, i;

	float dx;
	float dy;
	float r2;
	float wgt, mod, ang, fbin;
	/* skip if the keypoint octave is not current */
	if (oKey_Point.o != poSift->o_cur)
		return 0;

	//万一越界
	if (xi < 0 || xi > w - 1 || yi < 0 || yi > h - 1 ||
		si < poSift->s_min + 1 || si > poSift->s_max - 2)
		return 0;	//这个很奇怪，为什么会出现越界情况，感觉是伪判断

	/* clear histogram */
	//memset(hist, 0, sizeof(float) * nbins);

	/* compute orientation histogram */
	pt = poSift->m_pGrad + xo * xi + yo * yi;

#undef  at
#define at(dx,dy) (*(pt + xo * (dx) + yo * (dy)))
	//if (oKey_Point.o == 1 /*&& oKey_Point.is == 1 && oKey_Point.ix == 113 && oKey_Point.iy == 152*/)
		//printf("here");
	for (ys = max(-W, -yi); ys <= min(+W, h - 1 - yi); ++ys)
	{
		for (xs = max(-W, -xi); xs <= min(+W, w - 1 - xi); ++xs)
		{
			dx = (float)(xi + xs) - x;
			dy = (float)(yi + ys) - y;
			r2 = dx * dx + dy * dy;
			//wgt, mod, ang, fbin;

			/* limit to a circular window */
			if (r2 >= W * W + 0.6)
				continue;

			//wgt = fast_expn(r2 / (2 * sigmaw * sigmaw));
			wgt = (float)exp(-r2 / (2 * sigmaw * sigmaw));
			//预先已经算出来，在Update_Gradient中，问题是什么意义
			mod = *(pt + xs * xo + ys * yo);		//显然这是梯度，模长越长，落差越大
			ang = *(pt + xs * xo + ys * yo + 1);	//这个是角度，该点的梯度方向
			fbin = nbins * ang / (2 * PI);			//一个圆分36分，fbin即ang在哪一份

#if defined(VL_SIFT_BILINEAR_ORIENTATIONS)
			{
				int bin = (int)floor(fbin - 0.5f);
				float rbin = fbin - bin - 0.5f;
				//一个点的角度一般跨了两份角度，按照位置决定哪份多点哪份少点，然后
				//梯度的贡献也分开两部分，分配到相邻的两个角度上。这个计算就是个小学算数
				//画个图，如果落在两份之间，当然是各占0.5。如果向右多点，则左邻居少点。
				//按比例一分完事

				// 此处傻逼，摆了个乌龙，存在负位置
				//hist[(bin + nbins) % nbins] += (1 - rbin) * mod * wgt;
				//hist[(bin + 1) % nbins] += (rbin)*mod * wgt;

				hist[(bin + nbins) % nbins] += (1 - rbin) * mod * wgt;
				hist[(bin + 1 + nbins) % nbins] += (rbin)*mod * wgt;

				//if ((bin + nbins +36) % nbins < 0 || (bin + 1+36) % nbins < 0)
					//printf("here");
			}
#else
			{
				int    bin = vl_floor_d(fbin);
				bin = vl_floor_d(nbins * ang / (2 * PI));
				hist[(bin) % nbins] += mod * wgt;
			}
#endif

		} /* for xs */
	} /* for ys */

	/* smooth histogram */
	for (iter = 0; iter < 6; iter++)
	{
		float prev = hist[nbins - 1];
		float first = hist[0];
		int i;
		//printf("iter:%d\n", iter);
		for (i = 0; i < nbins - 1; i++)
		{
			//以自己为中心，加上左右邻居，再来个平均作为新值
			//然后来回折腾几次已达到平滑效果
			float newh = (prev + hist[i] + hist[(i + 1) % nbins]) / 3.0f;
			prev = hist[i];
			hist[i] = newh;
		}
		hist[i] = (prev + hist[i] + first) / 3.0f;
	}

	/* find the histogram maximum */
	maxh = 0;
	for (i = 0; i < nbins; ++i)
		maxh = max(maxh, hist[i]);

	/* find peaks within 80% from max */
	nangles = 0;
	for (i = 0; i < nbins; ++i)
	{
		float h0 = hist[i];
		float hm = hist[(i - 1 + nbins) % nbins];	//h0角度旁边的角度
		float hp = hist[(i + 1 + nbins) % nbins];

		/* is this a peak? */
		if (h0 > 0.8 * maxh && h0 > hm && h0 > hp)
		{//由于W足够大，表示关于本关键点的搜索范围足够大，比如半径达到8像素，这个时候完全可以右多个落差很大的路径
			//想都想到，下山的路径不止一条。那么这个时候接着看梯度是否足够大，如果大到一定程度，这里是最大梯度的0.8倍，
			//就能形成另一各特征,这个特征也是用方向来表达
			/* quadratic interpolation */
			float di = -0.5f * (hp - hm) / (hp + hm - 2.f * h0);
			float th = 2.f * PI * (i + di + 0.5f) / nbins;
			Angle[nangles++] = th;
			if (nangles == 4)
				goto enough_angles;
		}
	}
enough_angles:
	return nangles;
}

inline float Normalize_Histogram(float* begin, float* end)
{//基本上就是向量的单位化
#define eps 1.19209290E-07F
	float* iter;
	float  norm = 0.0;

	union {
		__m512 Total_16;
		__m256 Total_8[2];
		__m128 Total_4[2];
	};
	__m512 Norm_Recip;
	Total_16 = _mm512_mul_ps(*(__m512*)begin, *(__m512*)begin);
	for (iter = begin + 16; iter < end; iter += 16)
		Total_16 = _mm512_add_ps(Total_16, _mm512_mul_ps(*(__m512*)iter, *(__m512*)iter));
	Total_8[0] = _mm256_add_ps(Total_8[0], Total_8[1]);
	Total_4[0] = _mm_add_ps(Total_4[0], Total_4[1]);
	//norm = Total_4[0].m128_f32[0] + Total_4[0].m128_f32[1] + Total_4[0].m128_f32[2] + Total_4[0].m128_f32[3];
	norm = ((float*)&Total_4[0])[0] + ((float*)&Total_4[0])[1] + ((float*)&Total_4[0])[2] + ((float*)&Total_4[0])[3];
	norm = (float)sqrt(norm);
	Norm_Recip = _mm512_set1_ps(1.f / norm);

	for (iter = begin; iter != end; iter += 16)
		*(__m512*)iter = _mm512_mul_ps(*(__m512*)iter, Norm_Recip);
	return norm;
#undef eps
}

void Calc_Keypoint_Descriptor_AVX512(Sift* poSift, float* descr, Sift_Keypoint const* k, float angle0)
{//简单加速
#define eps 2.220446049250313e-16
#define NBO 8
#define NBP 4
	float const magnif = poSift->magnif;
	float       xper = (float)pow(2.0f, poSift->o_cur);
	int          w = poSift->octave_width;
	int          h = poSift->octave_height;
	int const    xo = 2;         /* x-stride */
	int const    yo = 2 * w;     /* y-stride */
	//int const    so = 2 * w * h; /* s-stride */
	float       x = k->x / xper;
	float       y = k->y / xper;
	float       sigma = k->sigma / xper;

	int          xi = (int)(x + 0.5);	//此处xi用的是子像素值，但是又四舍五入为整数，似乎没什么用
	int          yi = (int)(y + 0.5);
	int          si = k->is;

	float const st0 = (float)sin(angle0);
	float const ct0 = (float)cos(angle0);
	float const SBP = (float)(magnif * sigma + eps);
	//以下这个W就是r
	int    const W = (int)(floor(sqrt(2.0) * SBP * (NBP + 1) / 2.0 + 0.5));

	int const binto = 1;          /* bin theta-stride */
	int const binyo = NBO * NBP;  /* bin y-stride */
	int const binxo = NBO;        /* bin x-stride */

	int bin, dxi, dyi;
	float* pt;
	float* dpt;

	//越界检查
	if (k->o != poSift->o_cur || xi < 0 || xi >= w ||
		yi < 0 || yi >= h - 1 || si    <  poSift->s_min + 1 ||
		si    >  poSift->s_max - 2)
		return;

	//8x4x4块，128维Descriptor，要进一步分析其划分
	memset(descr, 0, sizeof(float) * NBO * NBP * NBP);

	//x0：每点耗费多少个样本， y0：每行多少个样本。 因为每点的梯度由两个样本构成
	//1，落差；2，方向  pt的含义是：point
	pt = &poSift->m_pGrad[xi * xo + yi * yo];

	//从以下这段猜测，一个Descriptor和 Surf一样，右4x4个sub block构成，每个
	//sub block的的大小为2x2=4, 所以一共有4x4lx4组信息。每组信息可能由两个字段
	//构成： 1，模长（落差）；2，方向。 所以一共128维
	//dpt的含义是 descriptor point，从数值上，dpt是位于descr[80], 为什么是80？
	//这是因为这个位置是在4x4的中间块， [2][2]上。 2*4*2*2 + 2*2*2=80，齐活
	dpt = descr + (NBP / 2) * binyo + (NBP / 2) * binxo;

#define atd(dbinx,dbiny,dbint) *(dpt + (dbint)*binto + (dbiny)*binyo + (dbinx)*binxo)

	//由于是简单优化，前面照抄，后面并行操作再优化
	int y_Start = max(-W, 1 - yi),
		y_End = min(+W, h - yi - 2),
		x_Start = max(-W, 1 - xi),
		x_End = min(+W, w - xi - 2),//iWin_Width = x_End - x_Start + 1,
		y1;
	float* pt_Cur, dy,
		SBP_recip = 1.f / SBP,
		st0_x_dy,	//st0*dy
		ct0_x_dy,	//ct0*dy
		wsigma_recip = 1.f / (2.0f * poSift->windowSize * poSift->windowSize);

	int iResult, dbinx, dbiny, dbint, iRemain;
	int iResult_x, iResult_y;
	__m512 mod_16, dx_16, nx_16, ny_16, win_16,
		rbinx_16, rbiny_16, rbint_16,
		weight_16;
	union { __m512 angle_16, theta_16, nt_16; };
	__m512i Index_16, dxi_16, binx_dbinx, biny_dbiny, bint_dbint,
		binx_16, biny_16, bint_16,
		seq_16 = _mm512_set_epi32(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);

	//整个方块窗口尽可能大
	for (dyi = y_Start; dyi <= y_End; dyi++)
	{
		pt_Cur = pt + dyi * yo;
		y1 = dyi * yo;
		//dy_16 = _mm512_set1_ps(yi + dyi - y);
		dy = yi + dyi - y;
		st0_x_dy = st0 * dy;
		ct0_x_dy = ct0 * dy;
		for (dxi = x_Start; dxi <= x_End; dxi += 16, pt_Cur += 16)
		{
			iRemain = (x_End - dxi + 1);
			if (iRemain > 16)
				iRemain = 16;

			//float mod = *(pt + dxi * xo + dyi * yo + 0);
			//Index_16 = _mm512_add_epi32(dxi_512, _mm512_set1_epi32(dxi));
			//Index_16 = _mm512_mullo_epi32(Index_16, _mm512_set1_epi32(xo));
			//Index_16 = _mm512_add_epi32(_mm512_set1_epi32(y1), Index_16);
			dxi_16 = _mm512_add_epi32(seq_16, _mm512_set1_epi32(dxi));
			Index_16 = _mm512_add_epi32(_mm512_set1_epi32(y1), _mm512_mullo_epi32(dxi_16, _mm512_set1_epi32(xo)));
			mod_16 = _mm512_i32gather_ps(Index_16, pt, 4);

			//Index_16=_mm512_add_epi32(Index_16, _mm512_set1_epi32(1));
			angle_16 = _mm512_i32gather_ps(_mm512_add_epi32(Index_16, _mm512_set1_epi32(1)), pt, 4);

			//float theta = vl_mod_2pi_f(angle - angle0);
			angle_16 = _mm512_sub_ps(angle_16, _mm512_set1_ps(angle0));
			iResult = _mm512_cmp_ps_mask(angle_16, _mm512_set1_ps(0), _CMP_LT_OS);
			theta_16 = _mm512_mask_add_ps(angle_16, iResult, angle_16, _mm512_set1_ps(PI * 2));

			//float dx = (float)(xi + dxi - x);
			dx_16 = _mm512_sub_ps(_mm512_cvtepi32_ps(_mm512_add_epi32(dxi_16, _mm512_set1_epi32(xi))), _mm512_set1_ps(x));

			//float nx = (ct0 * dx + st0 * dy) / SBP;
			//nx_16 = _mm512_mul_ps(dx_16, _mm512_set1_ps(ct0));
			//nx_16 = _mm512_add_ps(nx_16, st0_x_dy);
			//nx_16 = _mm512_mul_ps(nx_16, _mm512_set1_ps(SBP_recip));
			nx_16 = _mm512_mul_ps(_mm512_add_ps(_mm512_mul_ps(dx_16, _mm512_set1_ps(ct0)), _mm512_set1_ps(st0_x_dy)), _mm512_set1_ps(SBP_recip));

			//float ny = (-st0 * dx + ct0 * dy) / SBP;
			//ny_16 = _mm512_mul_ps(dx_16, _mm512_set1_ps(-st0));
			//ny_16 = _mm512_add_ps(ny_16, _mm512_set1_ps(ct0_x_dy));
			//ny_16 = _mm512_mul_ps(ny_16, _mm512_set1_ps(SBP_recip));
			ny_16 = _mm512_mul_ps(_mm512_add_ps(_mm512_mul_ps(dx_16, _mm512_set1_ps(-st0)), _mm512_set1_ps(ct0_x_dy)), _mm512_set1_ps(SBP_recip));

			//float nt = NBO * theta / (2 * PI);
			//nt_16 = _mm512_mul_ps(_mm512_set1_ps(NBO), theta_16);
			//nt_16 = _mm512_mul_ps(nt_16, _mm512_set1_ps(1.f / (2.f * PI)));
			nt_16 = _mm512_mul_ps(_mm512_mul_ps(_mm512_set1_ps(NBO), theta_16), _mm512_set1_ps(1.f / (2.f * PI)));

			//float win = fast_expn((nx * nx + ny * ny) / (2.0f * wsigma * wsigma));
			win_16 = _mm512_mul_ps(_mm512_add_ps(_mm512_mul_ps(nx_16, nx_16), _mm512_mul_ps(ny_16, ny_16)), _mm512_set1_ps(-wsigma_recip));
			win_16 = _mm512_exp_ps(win_16);

			//int binx = (int)vl_floor_f(nx - 0.5f);
			//int         biny = (int)vl_floor_f(ny - 0.5f);
			//int         bint = (int)vl_floor_f(nt);
			binx_16 = _mm512_cvtps_epi32(_mm512_floor_ps(_mm512_sub_ps(nx_16, _mm512_set1_ps(0.5f))));
			biny_16 = _mm512_cvtps_epi32(_mm512_floor_ps(_mm512_sub_ps(ny_16, _mm512_set1_ps(0.5f))));
			bint_16 = _mm512_cvtps_epi32(_mm512_floor_ps(nt_16));

			//float rbinx = nx - (binx + 0.5f);
			//float rbiny = ny - (biny + 0.5f);
			//float rbint = nt - bint;
			rbinx_16 = _mm512_sub_ps(nx_16, _mm512_add_ps(_mm512_cvtepi32_ps(binx_16), _mm512_set1_ps(0.5f)));
			rbiny_16 = _mm512_sub_ps(ny_16, _mm512_add_ps(_mm512_cvtepi32_ps(biny_16), _mm512_set1_ps(0.5f)));
			rbint_16 = _mm512_sub_ps(nt_16, _mm512_cvtepi32_ps(bint_16));
			for (dbinx = 0; dbinx < 2; ++dbinx)
			{
				//if (!(binx + dbinx >= -(NBP / 2) && binx + dbinx < (NBP / 2)))
					//continue;
				binx_dbinx = _mm512_add_epi32(binx_16, _mm512_set1_epi32(dbinx));
				iResult_x = _mm512_cmp_epi32_mask(binx_dbinx, _mm512_set1_epi32(-(NBP / 2)), _MM_CMPINT_GE);
				iResult_x &= _mm512_cmp_epi32_mask(binx_dbinx, _mm512_set1_epi32(NBP / 2), _MM_CMPINT_LT);
				if (!iResult_x)
					continue;

				for (dbiny = 0; dbiny < 2; ++dbiny)
				{
					//if (!(biny + dbiny >= -(NBP / 2) && biny + dbiny < (NBP / 2)))
						//continue;
					biny_dbiny = _mm512_add_epi32(biny_16, _mm512_set1_epi32(dbiny));
					iResult_y = iResult_x & _mm512_cmp_epi32_mask(biny_dbiny, _mm512_set1_epi32(-(NBP / 2)), _MM_CMPINT_GE);
					iResult_y &= _mm512_cmp_epi32_mask(biny_dbiny, _mm512_set1_epi32(NBP / 2), _MM_CMPINT_LT);
					if (!iResult_y)
						continue;
					for (dbint = 0; dbint < 2; dbint++)
					{
						iResult = (iResult_x & iResult_y) & ((1 << iRemain) - 1);
						//一条指令顶半边天
						//bint_dbint = _mm512_rem_epi32(_mm512_add_epi32(bint_16, _mm512_set1_epi32(dbint)), _mm512_set1_epi32(NBO));
						bint_dbint = _mm512_and_epi32(_mm512_add_epi32(bint_16, _mm512_set1_epi32(dbint)), _mm512_set1_epi32(0x7));
						weight_16 = _mm512_abs_ps(_mm512_sub_ps(_mm512_set1_ps(1.f - dbinx), rbinx_16));
						weight_16 = _mm512_mul_ps(weight_16, _mm512_abs_ps(_mm512_sub_ps(_mm512_set1_ps(1.f - dbiny), rbiny_16)));
						weight_16 = _mm512_mul_ps(weight_16, _mm512_abs_ps(_mm512_sub_ps(_mm512_set1_ps(1.f - dbint), rbint_16)));
						weight_16 = _mm512_mul_ps(weight_16, _mm512_mul_ps(win_16, mod_16));

						Index_16 = _mm512_mullo_epi32(binx_dbinx, _mm512_set1_epi32(binxo));
						Index_16 = _mm512_add_epi32(Index_16, _mm512_mullo_epi32(biny_dbiny, _mm512_set1_epi32(binyo)));
						Index_16 = _mm512_add_epi32(Index_16, _mm512_mullo_epi32(bint_dbint, _mm512_set1_epi32(binto)));

						//iResult &= (1 << iRemain) - 1;						
						for (int i = 0; iResult;)
						{
							if (iResult & 1)
								*(dpt + ((int*)&Index_16)[i]) += ((float*)&weight_16)[i];
								//*(dpt + Index_16.m512i_i32[i]) += weight_16.m512_f32[i];
								
							iResult >>= 1;
							i++;
						}
					}
				}
			}
		}
	}

	//Disp(descr, 128, 1);
	float norm = Normalize_Histogram(descr, descr + NBO * NBP * NBP);
	/* Set the descriptor to zero if it is lower than our norm_threshold */
	if (poSift->norm_thresh && norm < poSift->norm_thresh)
	{
		for (bin = 0; bin < NBO * NBP * NBP; ++bin)
			descr[bin] = 0;
	}
	else
	{
		for (bin = 0; bin < NBO * NBP * NBP; bin += 16)
		{
			iResult = _mm512_cmp_ps_mask(*(__m512*)&descr[bin], _mm512_set1_ps(0.2f), _MM_CMPINT_GE);
			if (iResult)
				_mm512_mask_i32scatter_ps(&descr[bin], iResult, seq_16, _mm512_set1_ps(0.2f), 4);
		}
		Normalize_Histogram(descr, descr + NBO * NBP * NBP);
	}
	return;
}

void L1RootNormalizeFeatureDescriptors(float Desc[])
{//对128维Descriptor进行另一种规格化。 模为一维范数
	float fFactor;
	int i;
	for (i = 0, fFactor = 0; i < 128; i++)
		fFactor += Desc[i];
	fFactor = 1.f / fFactor;
	for (i = 0; i < 128; i++)
		Desc[i] = (float)sqrt(Desc[i] * fFactor);
}
void FeatureDescriptorsToUnsignedByte(float Desc[128], unsigned char Desc_i[128])
{//只求数字一致，不管别的	0, 7, 6, 5, 4, 3, 2, 1, 8, 15, 14, 13, 12, 11, 10, 9
	__m512i Re_Order = _mm512_set_epi32(9, 10, 11, 12, 13, 14, 15, 8, 1, 2, 3, 4, 5, 6, 7, 0);
	__m512 Value_16 = _mm512_set1_ps(512.f);

	for (int i = 0; i < 128; i += 16)
		*(__m128i*)& Desc_i[i] = _mm512_cvtepi32_epi8(_mm512_cvt_roundps_epi32(_mm512_mul_ps(_mm512_i32gather_ps(Re_Order, &Desc[i], 4), Value_16), _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}
static void Get_Feature(Sift* poSift, Sift_Feature_Level* poFeature_Level, int iDoG_Top, int iOctave_Top)
{//继续向上走
	union {
		struct {
			float* pt, * pOctave_0, * pOctave_1;
		};
		__m256i Addr_3;
	};
	Sift oSift = *poSift;
	Sift_Feature_Level oFeature_Level = *poFeature_Level;
	Sift_Keypoint oKey_Point;
	Sift_Feature* poFeature_Byte;
	//Sift_Feature oFeature;
	float Desc[128];
	float* pEnd, * pFilter, angles[4];;
	float sd;
	int i, r, num_orientations, num_used_orientations,
		iSize = oSift.octave_width * oSift.octave_height;
	int iTemp,iOctave_Top_Index = iOctave_Top % 3;	//根据当前处理的最高层来推出在内存数组中的索引
	//做一层高斯滤波
	pOctave_0 = oSift.m_pGauss[iOctave_Top_Index];
	iTemp = (1 + iOctave_Top_Index) % 3;
	pOctave_1 = oSift.m_pGauss[iTemp];
	oSift.m_pTemp = oSift.m_pDoG[iOctave_Top_Index];	//共用本层DoG
	sd = oSift.dsigma0 * (float)pow(oSift.sigmak, iOctave_Top);
	r = (int)max((float)ceil(4.0 * sd), 1.f);
	Gen_Gauss_Filter(r, sd, &pFilter);
	//从下面一层生成上面一层
	Gauss_Filter_AVX512(pOctave_0, oSift.octave_width, oSift.octave_height, pOctave_1, oSift.m_pTemp, r, pFilter);
	free(pFilter);

	//做一层DoG
	pt = oSift.m_pDoG[(iDoG_Top + 1) % 3];
	pEnd = pt + iSize;
	for (; pt < pEnd; Addr_3 = _mm256_add_epi64(Addr_3, _mm256_set1_epi64x(16 * 4))   /*pt += 16, pOctave_0 += 16, pOctave_1 += 16*/)
		*(__m512*)pt = _mm512_sub_ps(*(__m512*)pOctave_1, *(__m512*)pOctave_0);

	//对已经做好的DoG做一次Keypoint检测
	Get_Keypoint_1(&oSift, iDoG_Top, iOctave_Top - 2, &oFeature_Level);
	if (iOctave_Top == oSift.s_max)
	{//若当前的高斯层已经到了最高层，则将当前层做一个下采样，存在下一层
		Resize_Image_Half(oSift.m_pGauss[(iOctave_Top-1) % 3], oSift.octave_width, oSift.octave_height, oSift.m_pGauss[iOctave_Top%3]);
		//bSave_Image("c:\\tmp\\1.bmp", oSift.m_pGauss[iOctave_Top % 3], oSift.octave_width>>1, oSift.octave_height>>1);
	}
	//更新梯度, 其实应该用Octave[iOctave_Top_Index-1]这帧，由于减法-1 再求模会有负数问题，
	//所以将 -1求模改成 +2 %3，对等
	pOctave_0 = oSift.m_pGauss[iDoG_Top % 3];
	oSift.m_pGrad = oSift.m_pDoG[(iDoG_Top-1) % 3];
	_Update_Gradiant_AVX512(pOctave_0, oSift.m_pGrad, oSift.octave_width, oSift.octave_height);
	
	for (i = 0; i < oFeature_Level.m_iKeypoint_Count; i++)
	{
		oKey_Point = oFeature_Level.m_pKeypoint[i];
		num_orientations = iCal_Key_Point_Oriatation(&oSift, angles, oKey_Point);
		num_used_orientations = min(num_orientations, 2);
		for (int o = 0; o < num_used_orientations; o++)
		{
			poFeature_Byte = &oFeature_Level.m_pFeature[oFeature_Level.m_iFeature_Count];
			poFeature_Byte->x = oKey_Point.x + 0.5f;
			poFeature_Byte->y = oKey_Point.y + 0.5f;
			poFeature_Byte->x_Keypoint = oKey_Point.ix;
			poFeature_Byte->y_Keypoint = oKey_Point.iy;
			poFeature_Byte->angle = angles[o];
			poFeature_Byte->o = oSift.o_cur;;
			poFeature_Byte->s = oKey_Point.is;
			Calc_Keypoint_Descriptor_AVX512(&oSift, Desc, &oKey_Point, angles[o]);
			L1RootNormalizeFeatureDescriptors(Desc);
			FeatureDescriptorsToUnsignedByte(Desc, poFeature_Byte->m_Desc_i);
			//Disp_Feature(*poFeature_Byte);
			oFeature_Level.m_iFeature_Count++;
		}
	}
	* poFeature_Level = oFeature_Level;
	return;
}

static void Update_Sift(Sift* poSift, int iOctave_Reserve_Index)
{//此时要将Octave[iOctave_Reserve_Index]放在Octave[0]处，并缩小一半
	Sift oSift = *poSift;
	//Resize_Image_Half(oSift.m_pGauss[iOctave_Reserve_Index], oSift.octave_width, oSift.octave_height, oSift.m_pGauss[0]);
	oSift.o_cur += 1;
	oSift.octave_width >>= 1, oSift.octave_height >>= 1;
	Arrange_Mem(&oSift, oSift.octave_width, oSift.octave_height, oSift.m_pBuffer_Start);
	*poSift = oSift;
	return;
}

void Get_Sift_Feature(float* pImage, int iWidth, int iHeight, int o_min,
	unsigned char* pBuffer, int iBuffer_Size, int *piFeature_Count)
{//Sift计算接口，尽可能清爽。 返回特征集及数量
	Sift oSift;
	Sift_Feature_Level oFeature_Level,Feature_Level[4][3];
	int i,iFeature_Count = 0, iKeypoint_Count = 0;

	Init_Sift(&oSift, 4, 3,o_min, iWidth, iHeight, pBuffer);
	Get_w_h(oSift.m_iWidth, oSift.m_iHeight, oSift.octave_width, oSift.octave_height, oSift.o_cur);
	Arrange_Mem(&oSift, oSift.octave_width, oSift.octave_height, oSift.m_pBuffer_Start);
	oFeature_Level = { oSift.m_pFeature,oSift.m_pKey_Point,0,0 };

	//根据o_min将图缩放到Gauss[0]中
	if (oSift.o_min == -1)
		Resize_Image_x2(pImage, oSift.m_iWidth, oSift.m_iHeight, oSift.m_pGauss[0]);
	else if (oSift.o_min == 1)
		Resize_Image_Half(pImage, oSift.m_iWidth, oSift.m_iHeight, oSift.m_pGauss[0]);
	else
		memcpy(oSift.m_pGauss[0], pImage, oSift.m_iWidth * oSift.m_iHeight * sizeof(float));

	//bSave_Image("c:\\tmp\\1.bmp", oSift.m_pGauss[0], oSift.octave_width, oSift.octave_height);
	
	while (1)
	{//逐个分辨率做Sift分析，w*2*h*2->w*h->w/2 * h/2 -> w/4 * h/4
		//此处要一改前人方法，省点内存
		Sift_Dectact_3(&oSift, 0);
		for (i = 0; i < 3; i++)
		{
			Get_Feature(&oSift, &oFeature_Level, i + 1, i + 2);
			Feature_Level[oSift.o_cur + 1][i] = oFeature_Level;
			oFeature_Level.m_pFeature += oFeature_Level.m_iFeature_Count;
			oFeature_Level.m_pKeypoint += oFeature_Level.m_iKeypoint_Count;
			//printf("o:%d s:%d Keypoint:%d Feature:%d\n", oSift.o_cur, i, oFeature_Level.m_iKeypoint_Count,oFeature_Level.m_iFeature_Count);
			iFeature_Count += oFeature_Level.m_iFeature_Count;
			iKeypoint_Count += oFeature_Level.m_iKeypoint_Count;
			oFeature_Level.m_iFeature_Count = oFeature_Level.m_iKeypoint_Count = 0;
			if (iKeypoint_Count >= MAX_KEY_POINT_COUNT)
			{//此处还不安全，要在里面判断超过点数
				*piFeature_Count = 0;
				printf("Keypoint Count exceed Max size:%d\n", iKeypoint_Count);
				return;
			}
			if (iFeature_Count >= MAX_KEY_POINT_COUNT * 2)
			{
				*piFeature_Count = 0;
				printf("Feature Count exceed Max size:%d\n", iFeature_Count);
				goto END;
			}
		}
		if (oSift.o_cur == 2)	//目前只看到最多搞到第2层octave，还要进一步验证
			break;
		//更新一下oSift
		Update_Sift(&oSift, 0);
	}
END:

	if (piFeature_Count)
		*piFeature_Count = iFeature_Count;
	return;
}
void Get_Sift_Feature(const char* pcFile, float(**ppFeature)[2], int* piCount, int o_min)
{
	Mem_Mgr oMem_Mgr;
	Image oImage;
	float* pImage;
	int iCount, iSize, iSift_Size;
	unsigned char* pBuffer;

	Get_Image_Info(pcFile, &oImage);
	iSize = oImage.m_iWidth * oImage.m_iHeight * sizeof(float);
	iSift_Size = iGet_Sift_Detect_Size(oImage.m_iWidth, oImage.m_iHeight, o_min);

	*ppFeature = NULL, * piCount = 0;
	Init_Mem_Mgr(&oMem_Mgr, iSize + iSift_Size, 1024, 997);
	if (!oMem_Mgr.m_pBuffer)
		return;
	pImage = (float*)pMalloc(&oMem_Mgr, oImage.m_iWidth * oImage.m_iHeight * sizeof(float));
	//分配足够的点数据，放在最前面，因为最后只有这组数据有用
	if (!bLoad_Image(pcFile, &oImage, 0, 0, 0, 1, &oMem_Mgr))
		return;
	RGB_2_Gray(oImage, pImage);
	Free_Image(&oMem_Mgr, oImage);

	pBuffer = (unsigned char*)pMalloc(&oMem_Mgr, iSift_Size);
	Get_Sift_Feature(pImage, oImage.m_iWidth, oImage.m_iHeight, o_min, pBuffer, iSize, &iCount);
	if (ppFeature)
	{
		float (*pPoint)[2] = (float(*)[2])malloc(iCount * 2 * sizeof(float));
		for (int i = 0; i < iCount; i++)
		{
			pPoint[i][0] = ((Sift_Feature*)pBuffer)[i].x;
			pPoint[i][0] = ((Sift_Feature*)pBuffer)[i].y;
		}
		*ppFeature = pPoint;
		*piCount = iCount;
	}
	//Disp_Mem(&oMem_Mgr, 0);
	Free_Mem_Mgr(&oMem_Mgr);
}
void Get_Sift_Feature(const char* pcFile, Sift_Feature** ppFeature, int* piCount, int o_min)
{//此乃接口1，调用函数自己负责释放内存
	Mem_Mgr oMem_Mgr;
	Image oImage;
	float* pImage;
	int iCount, iSize, iSift_Size;
	unsigned char* pBuffer;

	Get_Image_Info(pcFile, &oImage);
	iSize = oImage.m_iWidth * oImage.m_iHeight * sizeof(float);
	iSift_Size = iGet_Sift_Detect_Size(oImage.m_iWidth, oImage.m_iHeight, o_min);

	*ppFeature = NULL, * piCount = 0;
	Init_Mem_Mgr(&oMem_Mgr, iSize + iSift_Size, 1024, 997);
	if (!oMem_Mgr.m_pBuffer)
		return;

	pImage = (float*)pMalloc(&oMem_Mgr, oImage.m_iWidth * oImage.m_iHeight * sizeof(float));
	//分配足够的点数据，放在最前面，因为最后只有这组数据有用
	if (!bLoad_Image(pcFile, &oImage, 0, 0, 0, 1, &oMem_Mgr))
		return;
	RGB_2_Gray(oImage, pImage);
	Free_Image(&oMem_Mgr, oImage);

	pBuffer = (unsigned char*)pMalloc(&oMem_Mgr, iSift_Size);
	Get_Sift_Feature(pImage, oImage.m_iWidth, oImage.m_iHeight, o_min, pBuffer, iSize, &iCount);
	if (ppFeature)
	{
		*ppFeature = (Sift_Feature*)malloc(iCount * sizeof(Sift_Feature));
		memcpy(*ppFeature, pBuffer, iCount * sizeof(Sift_Feature));
		*piCount = iCount;
	}
	//Disp_Mem(&oMem_Mgr, 0);
	Free_Mem_Mgr(&oMem_Mgr);
}

static void Copy_Desc_1(Sift_Image Sift_Image_Arr[], int iImage_Count, unsigned char* pCur)
{//做一个转置，16个样本一组， 一组的结构是 128行 x 16 个字节， 样本竖着排
	//再构造个大数组
	int i,j,k,l,iGroup_16_Count;
	Sift_Image oSift_Image;
	unsigned char* pSource, * pDest;
	for (i = 0; i < iImage_Count; i++)
	{
		oSift_Image = Sift_Image_Arr[i];
		oSift_Image.m_pDesc = (unsigned char(*)[128])pCur;
		iGroup_16_Count = (((oSift_Image.m_iCount + 15) >> 4) << 4);
		for (j = 0; j < iGroup_16_Count; j+=16)
		{
			for (k = 0; k < 16; k++)
			{
				pSource = oSift_Image.m_pFeature[j * 16 + k].m_Desc_i;
				pDest = &oSift_Image.m_pDesc_1[j * 128*16 + k];
				for (l = 0; l < 128; l++,pDest+=128)
					*pDest= pSource[l];
			}
		}
		Sift_Image_Arr[i].m_pDesc = oSift_Image.m_pDesc;
		pCur += (((Sift_Image_Arr[i].m_iCount+15)>>4)<<4) * 128;
	}
	return;
}
static void Copy_Desc(Sift_Image Sift_Image_Arr[], int iImage_Count, unsigned char* pCur)
{
	//再构造个大数组
	int i;
	Sift_Image oSift_Image;
	for (i = 0; i < iImage_Count; i++)
	{
		oSift_Image = Sift_Image_Arr[i];
		oSift_Image.m_pDesc = (unsigned char(*)[128])pCur;
		for (int j = 0; j < oSift_Image.m_iCount; j++)
			for (int k = 0; k < 128; k++)
				oSift_Image.m_pDesc[j][k] = oSift_Image.m_pFeature[j].m_Desc_i[k];
		Sift_Image_Arr[i].m_pDesc = oSift_Image.m_pDesc;
		pCur += Sift_Image_Arr[i].m_iCount * 128;
	}
}

int iGet_Distance_AVX512(unsigned char A[], unsigned char  B[])
{//计算欧氏距离
#pragma pack(16)
	register __m512i Value_32;
	register __m512i Sum;
	union {
		__m512i Sum_32;
		__m256i Sum_16[2];
	};
#pragma pack()

	//此处还要改一改，改成128字节对齐，可能会快些
	Value_32 = _mm512_sub_epi16(_mm512_cvtepu8_epi16(*(__m256i*) &A[0]), _mm512_cvtepu8_epi16(*(__m256i*) & B[0]));
	Sum_32 = _mm512_mullo_epi16(Value_32, Value_32);

	Value_32 = _mm512_sub_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A[32]), _mm512_cvtepu8_epi16(*(__m256i*) & B[32]));
	Sum_32 = _mm512_adds_epu16(Sum_32, _mm512_mullo_epi16(Value_32, Value_32));
	Sum = _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1]));
	
	Value_32 = _mm512_sub_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A[64]), _mm512_cvtepu8_epi16(*(__m256i*) & B[64]));
	Sum_32 = _mm512_mullo_epi16(Value_32, Value_32);

	Value_32 = _mm512_sub_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A[96]), _mm512_cvtepu8_epi16(*(__m256i*) & B[96]));
	Sum_32 = _mm512_adds_epu16(Sum_32, _mm512_mullo_epi16(Value_32, Value_32));
	Sum = _mm512_add_epi32(Sum, _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1])));

	return _mm512_reduce_add_epi32(Sum);	//快些
}

void Get_Nearest_2_Point_1(unsigned char Pos[128], unsigned char(*Point)[128], int iPoint_Count, Neighbour_K* poNeighbour, int iIndex_A, Neighbour_K Neighbour_B[])
{//Neighbour_B 是这么一个集，每一点的Neighbour表示
	Neighbour_K oNeighbour;
	int i;
	unsigned int iDistance;
	unsigned char* pCur_Point;
	if (iPoint_Count < 2)
	{
		printf("Point Count less then 2\n");
		return;	//不干也罢
	}
	pCur_Point = Point[0];
	iDistance = iGet_Distance_AVX512(Pos, pCur_Point);
	oNeighbour.m_Buffer[0] = { 0, (unsigned int)iDistance };

	//再反向给Neighbour_B反向赋值
	if (iDistance < Neighbour_B[0].m_Buffer[1].m_iDistance)
	{
		if (iDistance < Neighbour_B[0].m_Buffer[0].m_iDistance)
		{
			Neighbour_B[0].m_Buffer[1] = Neighbour_B[0].m_Buffer[0];
			Neighbour_B[0].m_Buffer[0] = { iIndex_A,iDistance };
		}else
			Neighbour_B[0].m_Buffer[1] = { iIndex_A,iDistance };
	}
	//Neighbour_B[0].m_Buffer[0] = {iIndex_A,(unsigned int)iDistance};

	pCur_Point = Point[1];
	iDistance = iGet_Distance_AVX512(Pos, pCur_Point);
	oNeighbour.m_Buffer[1] = { 1,iDistance };
	if (oNeighbour.m_Buffer[1].m_iDistance < oNeighbour.m_Buffer[0].m_iDistance)
		swap(oNeighbour.m_Buffer[0], oNeighbour.m_Buffer[1]);
	//再反向给Neighbour_B反向赋值
	if (iDistance < Neighbour_B[1].m_Buffer[1].m_iDistance)
	{
		if (iDistance < Neighbour_B[1].m_Buffer[0].m_iDistance)
		{
			Neighbour_B[1].m_Buffer[1] = Neighbour_B[1].m_Buffer[0];
			Neighbour_B[1].m_Buffer[0] = { iIndex_A,iDistance };
		}else
			Neighbour_B[1].m_Buffer[1] = { iIndex_A,iDistance };
	}
	//Neighbour_B[1].m_Buffer[0] = { iIndex_A,iDistance };
	
	for (i = 2; i < iPoint_Count; i++)
	{
		pCur_Point = Point[i];
		iDistance = iGet_Distance_AVX512(Pos, pCur_Point);
		if (iDistance < (int)oNeighbour.m_Buffer[1].m_iDistance)
		{
			if (iDistance < (int)oNeighbour.m_Buffer[0].m_iDistance)
			{
				oNeighbour.m_Buffer[1] = oNeighbour.m_Buffer[0];
				oNeighbour.m_Buffer[0] = { i,iDistance };
			}else
				oNeighbour.m_Buffer[1] = { i,iDistance };
		}
		//Neighbour_B的第i点，也要判断一下是否为最近点
		if (iDistance < Neighbour_B[i].m_Buffer[1].m_iDistance)
		{
			if (iDistance < Neighbour_B[i].m_Buffer[0].m_iDistance)
			{
				Neighbour_B[i].m_Buffer[1] = Neighbour_B[i].m_Buffer[0];
				Neighbour_B[i].m_Buffer[0] = { iIndex_A,iDistance };
			}else
				Neighbour_B[i].m_Buffer[1] = { iIndex_A,iDistance };
		}
	}
	*poNeighbour = oNeighbour;

	return;
}

void Get_Nearest_2_Point(unsigned char Pos[128], unsigned char(*Point)[128], int iPoint_Count, Neighbour_K* poNeighbour)
{//从Point集中种找到与Pos匹配的点
	Neighbour_K oNeighbour;
	int i, iDistance;
	unsigned char* pCur_Point;
	if (iPoint_Count < 2)
	{
		printf("Point Count less then 2\n");
		return;	//不干也罢
	}
	pCur_Point = Point[0];
	iDistance = iGet_Distance_AVX512(Pos, pCur_Point);
	oNeighbour.m_Buffer[0] = { 0, (unsigned int)iDistance };
	pCur_Point = Point[1];
	iDistance = iGet_Distance_AVX512(Pos, pCur_Point);
	oNeighbour.m_Buffer[1] = { 1,(unsigned int)iDistance };
	if (oNeighbour.m_Buffer[1].m_iDistance < oNeighbour.m_Buffer[0].m_iDistance)
		swap(oNeighbour.m_Buffer[0], oNeighbour.m_Buffer[1]);
	for (i = 2; i < iPoint_Count; i++)
	{
		pCur_Point = Point[i];
		iDistance = iGet_Distance_AVX512(Pos, pCur_Point);
		if (iDistance < (int)oNeighbour.m_Buffer[1].m_iDistance)
		{
			if (iDistance < (int)oNeighbour.m_Buffer[0].m_iDistance)
			{
				oNeighbour.m_Buffer[1] = oNeighbour.m_Buffer[0];
				oNeighbour.m_Buffer[0] = { i,(unsigned int)iDistance };
			}
			else
				oNeighbour.m_Buffer[1] = { i,(unsigned int)iDistance };
		}
	}
	*poNeighbour = oNeighbour;
	return;
}

unsigned int iDot(unsigned char A[], unsigned char B[])
{
	__m512i Sum;
	union {
		__m512i Sum_32;
		__m256i Sum_16[2];
	};
	Sum_32 = _mm512_mullo_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A[0]), _mm512_cvtepu8_epi16(*(__m256i*) & B[0]));
	Sum = _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1]));
	Sum_32 = _mm512_mullo_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A[32]), _mm512_cvtepu8_epi16(*(__m256i*) & B[32]));
	Sum = _mm512_add_epi32(Sum, _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1])));
	Sum_32 = _mm512_mullo_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A[64]), _mm512_cvtepu8_epi16(*(__m256i*) & B[64]));
	Sum = _mm512_add_epi32(Sum, _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1])));
	Sum_32 = _mm512_mullo_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A[96]), _mm512_cvtepu8_epi16(*(__m256i*) & B[96]));
	Sum = _mm512_add_epi32(Sum, _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1])));
	return _mm512_reduce_add_epi32(Sum);
}
#define Dot_1(A, B, iDistance) \
{ \
	unsigned char *A1=A,*B1=B;	\
	__m512i Sum;	\
	union {	\
		__m512i Sum_32;	\
		__m256i Sum_16[2];	\
	};	\
	Sum_32 = _mm512_mullo_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A1[0]), _mm512_cvtepu8_epi16(*(__m256i*) & B1[0]));	\
	Sum = _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1]));	\
	Sum_32 = _mm512_mullo_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A1[32]), _mm512_cvtepu8_epi16(*(__m256i*) & B1[32]));	\
	Sum = _mm512_add_epi32(Sum, _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1])));	\
	Sum_32 = _mm512_mullo_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A1[64]), _mm512_cvtepu8_epi16(*(__m256i*) & B1[64]));	\
	Sum = _mm512_add_epi32(Sum, _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1])));	\
	Sum_32 = _mm512_mullo_epi16(_mm512_cvtepu8_epi16(*(__m256i*) & A1[96]), _mm512_cvtepu8_epi16(*(__m256i*) & B1[96]));	\
	Sum = _mm512_add_epi32(Sum, _mm512_add_epi32(_mm512_cvtepu16_epi32(Sum_16[0]), _mm512_cvtepu16_epi32(Sum_16[1])));	\
	(iDistance)=_mm512_reduce_add_epi32(Sum);\
}

#define Dot(A, B, iDistance) { \
	unsigned char *B_1=(B); \
	(iDistance)=0;	\
	for (int j = 0; j < 128; j++) \
		(iDistance)+= A[j] * B_1[j]; \
}

int iGet_Match_1(unsigned char Desc_A[128], unsigned char(*pDesc_B)[128], int iCount_B,int iIndex_A,Neighbour_K Neighbour_B[])
{//从B中找到最近邻，返回B中最合适的索引
	Neighbour_K oNeighbour;
	const float kDistNorm = 1.0f / (512.0f * 512.0f);
	const float max_distance = 0.7f,
		max_ratio = 0.8f;
	float best_dist_normed, second_best_dist_normed;

	Get_Nearest_2_Point_1(Desc_A, pDesc_B, iCount_B, &oNeighbour,iIndex_A, Neighbour_B);
	
	//将欧式距离改为点积
	Dot_1(Desc_A, pDesc_B[oNeighbour.m_Buffer[0].m_iIndex], oNeighbour.m_Buffer[0].m_iDistance);
	Dot_1(Desc_A, pDesc_B[oNeighbour.m_Buffer[1].m_iIndex], oNeighbour.m_Buffer[1].m_iDistance);
	//oNeighbour.m_Buffer[0].m_iDistance = iDot(Desc_A, pDesc_B[oNeighbour.m_Buffer[0].m_iIndex]);
	//oNeighbour.m_Buffer[1].m_iDistance = iDot(Desc_A, pDesc_B[oNeighbour.m_Buffer[1].m_iIndex]);

	//再将距离大者放在[0],次之放在[1]
	if (oNeighbour.m_Buffer[0].m_iDistance < oNeighbour.m_Buffer[1].m_iDistance)
		swap(oNeighbour.m_Buffer[0], oNeighbour.m_Buffer[1]);

	best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[0].m_iDistance, 1.0f));
	second_best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[1].m_iDistance, 1.0f));

	//首先判断最优点是否大于阈值max_distance,如果大了就跳过，没毛病
	if (best_dist_normed > max_distance)
		return -1;
	//再判断最优点是否比次优点的0.8倍还大，表示最优点离此优点不能太远，
	//这个判断尚未明白。次优点不是来捣乱的吗？
	if (best_dist_normed >= max_ratio * second_best_dist_normed)
		return -1;

	return oNeighbour.m_Buffer[0].m_iIndex;
}

int iGet_Match(unsigned char Desc_A[128], unsigned char(*pDesc_B)[128], int iCount_B)
{//从B中找到最近邻，返回B中最合适的索引
	Neighbour_K oNeighbour;
	const float kDistNorm = 1.0f / (512.0f * 512.0f);
	const float max_distance = 0.7f,
		max_ratio = 0.8f;
	float best_dist_normed, second_best_dist_normed;

	Get_Nearest_2_Point(Desc_A, pDesc_B, iCount_B, &oNeighbour);

	//将欧式距离改为点积
	Dot(Desc_A, pDesc_B[oNeighbour.m_Buffer[0].m_iIndex], oNeighbour.m_Buffer[0].m_iDistance);
	Dot(Desc_A, pDesc_B[oNeighbour.m_Buffer[1].m_iIndex], oNeighbour.m_Buffer[1].m_iDistance);

	//再将距离大者放在[0],次之放在[1]
	if (oNeighbour.m_Buffer[0].m_iDistance < oNeighbour.m_Buffer[1].m_iDistance)
		swap(oNeighbour.m_Buffer[0], oNeighbour.m_Buffer[1]);

	best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[0].m_iDistance, 1.0f));
	second_best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[1].m_iDistance, 1.0f));

	//首先判断最优点是否大于阈值max_distance,如果大了就跳过，没毛病
	if (best_dist_normed > max_distance)
		return -1;
	//再判断最优点是否比次优点的0.8倍还大，表示最优点离此优点不能太远，
	//这个判断尚未明白。次优点不是来捣乱的吗？
	if (best_dist_normed >= max_ratio * second_best_dist_normed)
		return -1;
	return oNeighbour.m_Buffer[0].m_iIndex;
}

void Sift_Match_2_Image_1(Sift_Image oImage_A, Sift_Image oImage_B, int iA_Index, int iB_Index, Sift_Match_Item* poMatch,Neighbour_K Neighbour_B[])
{//尝试用另一种方法
	int i, iMatch_B;
	Sift_Match_Item oMatch = { (short)iA_Index,(short)iB_Index,0,poMatch->m_Match };
	Neighbour_K oNeighbour;
	memset(Neighbour_B, 0xFF, oImage_B.m_iCount * sizeof(Neighbour_K));
	//以下直接找，不经过多轮查找
	for (i = 0; i < oImage_A.m_iCount; i++)
	{//此处为优化关键
		if ((iMatch_B = iGet_Match_1(oImage_A.m_pDesc[i], oImage_B.m_pDesc, oImage_B.m_iCount,i, Neighbour_B)) != -1)
		{
			oMatch.m_Match[oMatch.m_iMatch_Count][0] = i;
			oMatch.m_Match[oMatch.m_iMatch_Count++][1] = iMatch_B;
		}
	}

	//再找一次从B到A的查找
	int j,iMatch_A;
	const float kDistNorm = 1.0f / (512.0f * 512.0f);
	const float max_distance = 0.7f,
		max_ratio = 0.8f;
	float best_dist_normed, second_best_dist_normed;

	for (i = j = 0; i < oMatch.m_iMatch_Count; i++)
	{
		iMatch_A = oMatch.m_Match[i][0];
		iMatch_B = oMatch.m_Match[i][1];

		//对于点集B的第iMatch_B点，反向处理其Neighbour
		oNeighbour = Neighbour_B[iMatch_B];

		//将欧式距离改为点积
		Dot_1(oImage_A.m_pDesc[oNeighbour.m_Buffer[0].m_iIndex], oImage_B.m_pDesc[iMatch_B], oNeighbour.m_Buffer[0].m_iDistance);
		Dot_1(oImage_A.m_pDesc[oNeighbour.m_Buffer[1].m_iIndex], oImage_B.m_pDesc[iMatch_B], oNeighbour.m_Buffer[1].m_iDistance);
		//oNeighbour.m_Buffer[0].m_iDistance = iDot(oImage_A.m_pDesc[oNeighbour.m_Buffer[0].m_iIndex], oImage_B.m_pDesc[iMatch_B]);
		//oNeighbour.m_Buffer[1].m_iDistance = iDot(oImage_A.m_pDesc[oNeighbour.m_Buffer[1].m_iIndex], oImage_B.m_pDesc[iMatch_B]);
		//再将距离大者放在[0],次之放在[1]
		if (oNeighbour.m_Buffer[0].m_iDistance < oNeighbour.m_Buffer[1].m_iDistance)
			swap(oNeighbour.m_Buffer[0], oNeighbour.m_Buffer[1]);

		if (oNeighbour.m_Buffer[0].m_iIndex != iMatch_A)
			continue;

		best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[0].m_iDistance, 1.0f));
		//首先判断最优点是否大于阈值max_distance,如果大了就跳过，没毛病
		if (best_dist_normed > max_distance)
			continue;
		second_best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[1].m_iDistance, 1.0f));
		//再判断最优点是否比次优点的0.8倍还大，表示最优点离此优点不能太远，
		//这个判断尚未明白。次优点不是来捣乱的吗？
		if (best_dist_normed >= max_ratio * second_best_dist_normed)
			continue;

		oMatch.m_Match[j][0] = iMatch_A;
		oMatch.m_Match[j][1] = iMatch_B;
		j++;
		//printf("A:%d B:%d\n", iMatch_A, iMatch_B);
	}
	oMatch.m_iMatch_Count = j;
	*poMatch = oMatch;
	return;
}

void Sift_Match_2_Image(Sift_Image oImage_A, Sift_Image oImage_B, int iA_Index, int iB_Index, Sift_Match_Item* poMatch)
{
	int i, iMatch_B;
	Sift_Match_Item oMatch = { (short)iA_Index,(short)iB_Index,0,poMatch->m_Match };
	//以下直接找，不经过多轮查找
	for (i = 0; i < oImage_A.m_iCount; i++)
	{//此处为优化关键
		if ((iMatch_B = iGet_Match(oImage_A.m_pDesc[i], oImage_B.m_pDesc, oImage_B.m_iCount)) != -1)
		{
			if (iGet_Match(oImage_B.m_pDesc[iMatch_B], oImage_A.m_pDesc, oImage_A.m_iCount) == i)
			{
				oMatch.m_Match[oMatch.m_iMatch_Count][0] = i;
				oMatch.m_Match[oMatch.m_iMatch_Count++][1] = iMatch_B;
				printf("A:%d B:%d\n", i, iMatch_B);
			}
		}
	}

	////临时存储代码
	//FILE* pFile = fopen("c:\\tmp\\A.bin", "wb");
	//fwrite(oImage_A.m_pDesc, 1, oImage_A.m_iCount * 128, pFile);
	//fclose(pFile);
	//pFile = fopen("c:\\tmp\\B.bin", "wb");
	//fwrite(oImage_B.m_pDesc, 1, oImage_B.m_iCount * 128, pFile);
	//fclose(pFile);

	*poMatch = oMatch;
	return;
}
template<typename _T>void Sift_Match_2_Image(const char* pcFile_1, const char* pcFile_2, _T(**ppPoint_1)[2], _T(**ppPoint_2)[2], int* piCount, int o_min)
{//比较两张图的关键点，找出匹配对
	//return pPoint_1, pPoint_2， 但是调用函数只需释放pPoint_1即可
	_T(*pPoint_1)[2] = NULL, (*pPoint_2)[2]=NULL;
	Sift_Feature* pFeature_1, * pFeature_2;
	Mem_Mgr oMem_Mgr;
	Image oImage;
	float* pImage;
	int i, iSift_Size, iMax_Size;
	unsigned char* pBuffer;
	Init_Mem_Mgr(&oMem_Mgr, 1000000000, 1024, 997);
	if (!oMem_Mgr.m_pBuffer)
		return;

	//分配Sift_Image
	Sift_Image oSift_Image, * pSift_Image_Arr = (Sift_Image*)pMalloc(&oMem_Mgr, 2 * sizeof(Sift_Image));
	pSift_Image_Arr[0].m_pFile_Name = (char*)pcFile_1;
	pSift_Image_Arr[1].m_pFile_Name = (char*)pcFile_2;

	for (i = 0; i < 2; i++)
	{
		oSift_Image = pSift_Image_Arr[i];
		//先给Feature Point分配些空间
		Get_Image_Info(oSift_Image.m_pFile_Name, &oImage);
		iMax_Size = oImage.m_iWidth * oImage.m_iHeight * oImage.m_iChannel_Count;
		if (MAX_KEY_POINT_COUNT * sizeof(Sift_Feature) > iMax_Size)
			iMax_Size = MAX_KEY_POINT_COUNT * sizeof(Sift_Feature);

		oSift_Image.m_pFeature = (Sift_Feature*)pMalloc(&oMem_Mgr, iMax_Size);
		pImage = (float*)oSift_Image.m_pFeature;
		if (!bLoad_Image((const char*)oSift_Image.m_pFile_Name, &oImage, 0, 0, 0, 1, &oMem_Mgr))
			return;
		RGB_2_Gray(oImage, pImage);
		Free_Image(&oMem_Mgr, oImage);
		iSift_Size = iGet_Sift_Detect_Size(oImage.m_iWidth, oImage.m_iHeight);
		pBuffer = (unsigned char*)pMalloc(&oMem_Mgr, iSift_Size);
		Get_Sift_Feature(pImage, oImage.m_iWidth, oImage.m_iHeight, o_min, pBuffer, iSift_Size, &oSift_Image.m_iCount);
		if (oSift_Image.m_iCount)
			memcpy(oSift_Image.m_pFeature, pBuffer, oSift_Image.m_iCount * sizeof(Sift_Feature));
		Free(&oMem_Mgr, pBuffer);
		Shrink(&oMem_Mgr, oSift_Image.m_pFeature, oSift_Image.m_iCount * sizeof(Sift_Feature));
		pSift_Image_Arr[i] = oSift_Image;
	}

	//出来以后OK了，可以搞匹配，先预先分配空间
	iSift_Size = iGet_Sift_Match_Size(pSift_Image_Arr, 2);
	pBuffer = (unsigned char*)pMalloc(&oMem_Mgr, iSift_Size);
	if (!pBuffer)
		return;	//不够内存，严重错误
	unsigned char* pStart = pBuffer;
	//分配Match, 一共分走了 iImage_Count*iImage_Count*sizeof(Sift_Match_Item)
	for (i = 0; i < 2; i++)
	{
		pSift_Image_Arr[i].m_pMatch = (Sift_Match_Item*)pBuffer;
		memset(pBuffer, 0, 2 * sizeof(Sift_Match_Item));
		pBuffer += 2 * sizeof(Sift_Match_Item);
	}//出来以后，内存移到 pStart + iImage_Count*iImage_Count*sizeof(Sift_Match_Item)

	 //将pBuffer向前移动对齐128字节
	pBuffer = (unsigned char*) (((unsigned long long)pBuffer / 128 + 1) * 128);

	//再分配一次Desc, 整齐点以便于后面提速
	for (i = 0; i < 2; i++)
	{
		pSift_Image_Arr[i].m_pDesc = (unsigned char(*)[128])pBuffer;
		pBuffer += pSift_Image_Arr[i].m_iCount * 128;
	}
	Copy_Desc(pSift_Image_Arr, 2, pSift_Image_Arr->m_pDesc[0]);

	Sift_Match_Item oMatch_Item;
	oMatch_Item = { (short)0,(short)1,0,(unsigned short(*)[2])pBuffer };
	if (pBuffer + pSift_Image_Arr[0].m_iCount * sizeof(unsigned short) * 2 - pStart > iSift_Size)
	{//判断是否越界
		printf("Insufficient memory\n");
		return;
	}
	iMax_Size = Max(pSift_Image_Arr[0].m_iCount, pSift_Image_Arr[1].m_iCount);
	//到此处要分配一块内存用于匹配的临时空间
	Neighbour_K* pNeighbour_B;	//点集A的最近邻	
	pNeighbour_B = (Neighbour_K*)pMalloc(&oMem_Mgr, iMax_Size);
	if (!pNeighbour_B)
	{
		printf("Insufficient memory\n");
		goto END;
	}

	Sift_Match_2_Image_1(pSift_Image_Arr[0], pSift_Image_Arr[1], 0, 1, &oMatch_Item,pNeighbour_B);
	pBuffer += ALIGN_SIZE_128(oMatch_Item.m_iMatch_Count * 2 * sizeof(unsigned short));
	pSift_Image_Arr[1].m_pMatch[0] = pSift_Image_Arr[0].m_pMatch[1] = oMatch_Item;
	//printf("Match_Count:%d \n", oMatch_Item.m_iMatch_Count);

	pPoint_1 = (_T(*)[2])malloc(oMatch_Item.m_iMatch_Count * 4 * sizeof(_T));
	pPoint_2 = pPoint_1 + oMatch_Item.m_iMatch_Count;

	pFeature_1 = pSift_Image_Arr[0].m_pFeature;
	pFeature_2 = pSift_Image_Arr[1].m_pFeature;
	for (i = 0; i < oMatch_Item.m_iMatch_Count; i++)
	{
		int Index[2] = { oMatch_Item.m_Match[i][0],oMatch_Item.m_Match[i][1] };
		pPoint_1[i][0] = pFeature_1[Index[0]].x;
		pPoint_1[i][1] = pFeature_1[Index[0]].y;
		pPoint_2[i][0] = pFeature_2[Index[1]].x;
		pPoint_2[i][1] = pFeature_2[Index[1]].y;
	}
END:

	Free_Mem_Mgr(&oMem_Mgr);
	if (ppPoint_1)
		*ppPoint_1 = pPoint_1;
	else
		free(pPoint_1);
	if (ppPoint_2)
		*ppPoint_2 = pPoint_2;

	if (piCount)
		*piCount = oMatch_Item.m_iMatch_Count;
}

void Init_Sift_Match_Map(Mem_Mgr* poMem_Mgr,Sift_Match_Map* poMap,int iImage_Count)
{
	int iSize;
	unsigned char* pBuffer;
	iSize = iImage_Count * sizeof(Image_Info);
	iSize += iImage_Count * iImage_Count * sizeof(Sift_Simple_Match_Item);
	iSize = ALIGN_SIZE_128(iSize);
	pBuffer = (unsigned char*)pMalloc(poMem_Mgr, iSize);
	memset(pBuffer, 0, iSize);
	poMap->m_iImage_Count = iImage_Count;
	poMap->m_pImage_Arr = (Image_Info*)pBuffer;
	pBuffer += iImage_Count * sizeof(Image_Info);
	poMap->m_pMatch = (Sift_Simple_Match_Item*)pBuffer;
	return;
}

int iGet_Simple_Match_Size(int iImage_Count)
{//预分配一个足够大的空间，装所有图像两两配对信息
	int iSize,iUpper_Triangle_Size;
	//Image_Info数组
	iSize = iImage_Count * sizeof(Image_Info);
	//File_Name
	iSize += 256 * iImage_Count;
	//Match Item
	//iSize += iImage_Count * iImage_Count * sizeof(Sift_Simple_Match_Item);
	iUpper_Triangle_Size = iGet_Upper_Triangle_Size(iImage_Count);
	iSize += iUpper_Triangle_Size * sizeof(Sift_Simple_Match_Item);

	//Match Point Pair
	iSize += iImage_Count * MAX_KEY_POINT_COUNT * 4 * sizeof(float);
	return iSize;
}
void Init_Simple_Match(const char *pcPath,Sift_Match_Map *poMap, Mem_Mgr* poMem_Mgr)
{
	char File[256];
	int iSize = (int)strlen(pcPath);
	if (pcPath[iSize - 1] != '\\')
		sprintf(File, "%s\\*.bmp", pcPath);
	else
		sprintf(File, "%s*.bmp", pcPath);
	*poMap = { 0 };
	int i, iImage_Count = iGet_File_Count(File);
	if (!iImage_Count)
		return;
	iSize = iGet_Simple_Match_Size(iImage_Count);
	poMap->m_pBuffer = (unsigned char*)pMalloc(poMem_Mgr, iSize);
	if (!poMap->m_pBuffer)
		return;
	poMap->m_pImage_Arr = (Image_Info*)poMap->m_pBuffer;

	//文件名
	unsigned char* pCur;
	char* pFile_Name;
	pFile_Name = (char*)poMap->m_pBuffer + iImage_Count * sizeof(Image_Info);
	Get_All_File(File, pFile_Name);

	//先把File_Name赋予poMap
	for (pCur = (unsigned char*)pFile_Name, i = 0; i < iImage_Count; i++)
	{
		poMap->m_pImage_Arr[i].m_pFile_Name = (char*)pCur;
		pCur += strlen((char*)pCur) + 1;
	}
	pCur += 4-((unsigned long long)pCur) & 3;

	iSize = (int)((char*)pCur - pFile_Name);
	poMap->m_pMatch = (Sift_Simple_Match_Item*)pCur;
	int iUpper_Triangle_Size = iGet_Upper_Triangle_Size(iImage_Count);
	pCur += iUpper_Triangle_Size * sizeof(Sift_Simple_Match_Item);
	//poMap->m_pMatch[0].m_pMatch = (float(*)[2][2])pCur;
	poMap->m_pMatch[0].m_pPoint_1 = (float(*)[2])pCur;
	iSize += iImage_Count * MAX_KEY_POINT_COUNT * 4 * sizeof(float);
	Shrink(poMem_Mgr, poMap->m_pBuffer, iSize);
	poMap->m_iImage_Count = iImage_Count;
	return;
}

void Sift_Match_Path_1(const char* pcPath, Sift_Match_Map* poMap, Mem_Mgr* poMem_Mgr, int o_min)
{//尝试在匹配阶段提速
	char File[256];
	Sift_Image* pSift_Image_Arr, oSift_Image;
	float* pImage;
	int i, j, iSize, iMax_Match_Size, iImage_Count;
	unsigned char* pCur, * pStart;
	char* pFile_Name;
	Image oImage;

	//Disp_Mem(poMem_Mgr, 0);
	Init_Simple_Match(pcPath, poMap, poMem_Mgr);
	if (!poMap->m_pBuffer)
		return;
	iImage_Count = poMap->m_iImage_Count;
	//Disp_Mem(poMem_Mgr, 0);

	int iMax_Size;
	//再给pSift_Image_Arr分配File_Name内存
	pSift_Image_Arr = (Sift_Image*)pMalloc(poMem_Mgr, iImage_Count * sizeof(Sift_Image));
	pCur = (unsigned char*)poMap->m_pImage_Arr[0].m_pFile_Name;
	iSize = iImage_Count * 256;
	pFile_Name = (char*)pMalloc(poMem_Mgr, iSize);
	memcpy(pFile_Name, pCur, iSize);
	for (pCur = (unsigned char*)pFile_Name, i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pFile_Name = (char*)pCur;
		pCur += strlen((char*)pCur) + 1;
	}
	iSize = (int)((char*)pCur - pFile_Name);
	Shrink(poMem_Mgr, pFile_Name, iSize);

	iMax_Match_Size = MAX_KEY_POINT_COUNT * sizeof(Sift_Feature);
	for (i = 0; i < iImage_Count; i++)
	{
		sprintf(File, "%s\\%s", pcPath, poMap->m_pImage_Arr[i].m_pFile_Name);
		oSift_Image = pSift_Image_Arr[i];
		sprintf(File, "%s\\%s", pcPath, oSift_Image.m_pFile_Name);
		Get_Image_Info(File, &oImage);
		iMax_Size = max((int)iMax_Match_Size, (int)(oImage.m_iWidth * oImage.m_iHeight * sizeof(float)));
		pImage = (float*)pMalloc(poMem_Mgr, iMax_Size);
		if (!pImage)
		{
			printf("Fail to pMalloc in Sift_Match_Path\n");
			return;
		}
		if (!bLoad_Image(File, &oImage, 0, 0, 0, 1, poMem_Mgr))
			return;
		RGB_2_Gray(oImage, pImage);
		Free_Image(poMem_Mgr, oImage);

		iSize = iGet_Sift_Detect_Size(oImage.m_iWidth, oImage.m_iHeight);
		pCur = (unsigned char*)pMalloc(poMem_Mgr, iSize);
		if (!pCur)
			return;
		Get_Sift_Feature(pImage, oImage.m_iWidth, oImage.m_iHeight, o_min, pCur, iSize, &oSift_Image.m_iCount);
		//printf("Detect %d\n", i);
		Free(poMem_Mgr, pImage);
		oSift_Image.m_pFeature = (Sift_Feature*)pCur;
		pSift_Image_Arr[i] = oSift_Image;
		iSize = oSift_Image.m_iCount * sizeof(Sift_Feature);
		Shrink(poMem_Mgr, pCur, iSize);
		pSift_Image_Arr[i] = oSift_Image;
		//printf(".");
		printf("Feature Count:%d\n", oSift_Image.m_iCount);
	}

	//此处算Sift_Image的Match
	//Disp_Mem(poMem_Mgr, 0);
	iSize = iGet_Sift_Match_Size(pSift_Image_Arr, iImage_Count);
	//Temp code
	iSize *= 2;

	pCur = pStart = (unsigned char*)pMalloc(poMem_Mgr, iSize);
	if (!pCur)
		return;	//不够内存，严重错误

	//分配Match, 一共分走了 iImage_Count*iImage_Count*sizeof(Sift_Match_Item)
	for (i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pMatch = (Sift_Match_Item*)pCur;
		memset(pCur, 0, iImage_Count * sizeof(Sift_Match_Item));
		pCur += iImage_Count * sizeof(Sift_Match_Item);
	}//出来以后，内存移到 pStart + iImage_Count*iImage_Count*sizeof(Sift_Match_Item)

	//将pBuffer向前移动对齐128字节
	pCur = (unsigned char*)(((unsigned long long)pCur / 128 + 1) * 128);

	//再分配一次Desc, 整齐点以便于后面提速
	iMax_Size = 0;
	for (i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pDesc = (unsigned char(*)[128])pCur;
		pCur += pSift_Image_Arr[i].m_iCount * 128;

		if (pSift_Image_Arr[i].m_iCount > iMax_Size)//寻找最大开辟空间
			iMax_Size = pSift_Image_Arr[i].m_iCount;
	}
	iMax_Size *= sizeof(Neighbour_K);

	Copy_Desc(pSift_Image_Arr, iImage_Count, pSift_Image_Arr->m_pDesc[0]);

	//到此处要分配一块内存用于匹配的临时空间
	Neighbour_K* pNeighbour_B;	//点集A的最近邻	
	pNeighbour_B = (Neighbour_K*)pMalloc(poMem_Mgr, iMax_Size);
	if (!pNeighbour_B)
	{
		printf("Insufficient memory\n");
		goto END;
	}
	Sift_Match_Item oMatch_Item;
	for (i = 0; i < iImage_Count; i++)
	{
		for (j = i + 1; j < iImage_Count; j++)
		{
			oMatch_Item = { (short)i,(short)j,0,(unsigned short(*)[2])pCur };
			if (pCur + pSift_Image_Arr[i].m_iCount * sizeof(unsigned short) * 2 - pStart > iSize)
			{//判断是否越界
				printf("Insufficient memory\n");
				return;
			}
			Sift_Match_2_Image_1(pSift_Image_Arr[i], pSift_Image_Arr[j], i, j, &oMatch_Item, pNeighbour_B);
			pCur += ALIGN_SIZE_128(oMatch_Item.m_iMatch_Count * 2 * sizeof(unsigned short));
			pSift_Image_Arr[j].m_pMatch[i] = pSift_Image_Arr[i].m_pMatch[j] = oMatch_Item;
			printf("i:%d j:%d Match_Count:%d \n", i, j, oMatch_Item.m_iMatch_Count);
		}
	}

	//最后将Feature 抄到 Match_Map
	pCur = (unsigned char*)poMap->m_pMatch[0].m_pPoint_1;
	for (i = 0; i < iImage_Count; i++)
	{
		oSift_Image = pSift_Image_Arr[i];
		Sift_Feature* pFeature_A = oSift_Image.m_pFeature;
		for (j = i + 1; j < iImage_Count; j++)
		{
			oSift_Image = pSift_Image_Arr[j];
			Sift_Feature* pFeature_B = oSift_Image.m_pFeature;
			oMatch_Item = pSift_Image_Arr[i].m_pMatch[j];
			int iIndex = iUpper_Triangle_Cord_2_Index(j, i, iImage_Count);
			//Sift_Simple_Match_Item oMatch_Item_1 = poMap->m_pMatch[i * iImage_Count + j];
			Sift_Simple_Match_Item oMatch_Item_1 = poMap->m_pMatch[iIndex];
			oMatch_Item_1.m_iImage_A = i;
			oMatch_Item_1.m_iImage_B = j;
			oMatch_Item_1.m_iMatch_Count = oMatch_Item.m_iMatch_Count;
			oMatch_Item_1.m_pPoint_1 = (float(*)[2])pCur;
			oMatch_Item_1.m_pPoint_2 = oMatch_Item_1.m_pPoint_1 + oMatch_Item.m_iMatch_Count;
			//oMatch_Item_1.m_pMatch =(float(*)[2][2])pCur;
			for (int k = 0; k < oMatch_Item.m_iMatch_Count; k++)
			{
				oMatch_Item_1.m_pPoint_1[k][0] = pFeature_A[oMatch_Item.m_Match[k][0]].x;
				oMatch_Item_1.m_pPoint_1[k][1] = pFeature_A[oMatch_Item.m_Match[k][0]].y;
				oMatch_Item_1.m_pPoint_2[k][0] = pFeature_B[oMatch_Item.m_Match[k][1]].x;
				oMatch_Item_1.m_pPoint_2[k][1] = pFeature_B[oMatch_Item.m_Match[k][1]].y;
			}
			//poMap->m_pMatch[j*iImage_Count + i]= poMap->m_pMatch[i * iImage_Count + j] = oMatch_Item_1;
			poMap->m_pMatch[iIndex] = oMatch_Item_1;
			pCur += oMatch_Item.m_iMatch_Count * 4 * sizeof(float);
		}
	}
	iSize = (int)(pCur - poMap->m_pBuffer);	
	Shrink(poMem_Mgr, poMap->m_pBuffer, iSize);	
	//再将Sift_Image_Arr删除

END:
	if(pNeighbour_B)
		Free(poMem_Mgr, pNeighbour_B);
	Free(poMem_Mgr, pSift_Image_Arr[0].m_pMatch);
	for (i = 0; i < iImage_Count; i++)
		Free(poMem_Mgr, pSift_Image_Arr[i].m_pFeature);
	Free(poMem_Mgr, pSift_Image_Arr);
	Free(poMem_Mgr, pSift_Image_Arr[0].m_pFile_Name);
	//Disp_Mem(poMem_Mgr, 0);
	poMap->m_iImage_Count = iImage_Count;
	return;
}

void Sift_Match_Path(const char* pcPath, Sift_Match_Map* poMap, Mem_Mgr* poMem_Mgr, int o_min)
{//对某一个目录进行扫描，所有的bmp文件都进行Sift
	char File[256];
	Sift_Image* pSift_Image_Arr, oSift_Image;
	float* pImage;
	int i,j, iSize, iMax_Match_Size,iImage_Count;
	unsigned char* pCur,*pStart;
	char *pFile_Name;
	Image oImage;
		
	//Disp_Mem(poMem_Mgr, 0);
	Init_Simple_Match(pcPath, poMap, poMem_Mgr);
	if (!poMap->m_pBuffer)
		return;
	iImage_Count = poMap->m_iImage_Count;
	//Disp_Mem(poMem_Mgr, 0);

	int iMax_Size;
	//再给pSift_Image_Arr分配File_Name内存
	pSift_Image_Arr = (Sift_Image*)pMalloc(poMem_Mgr, iImage_Count * sizeof(Sift_Image));
	pCur = (unsigned char*)poMap->m_pImage_Arr[0].m_pFile_Name;
	iSize = iImage_Count * 256;
	pFile_Name = (char*)pMalloc(poMem_Mgr, iSize);
	memcpy(pFile_Name, pCur, iSize);
	for (pCur = (unsigned char*)pFile_Name, i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pFile_Name = (char*)pCur;
		pCur += strlen((char*)pCur) + 1;
	}
	iSize = (int)((char*)pCur - pFile_Name);
	Shrink(poMem_Mgr, pFile_Name, iSize);

	iMax_Match_Size = MAX_KEY_POINT_COUNT * sizeof(Sift_Feature);
	for (i = 0; i < iImage_Count; i++)
	{
		sprintf(File, "%s\\%s", pcPath, poMap->m_pImage_Arr[i].m_pFile_Name);
		oSift_Image = pSift_Image_Arr[i];
		sprintf(File, "%s\\%s", pcPath, oSift_Image.m_pFile_Name);
		Get_Image_Info(File, &oImage);
		iMax_Size = max((int)iMax_Match_Size, (int)(oImage.m_iWidth * oImage.m_iHeight * sizeof(float)));
		pImage = (float*)pMalloc(poMem_Mgr, iMax_Size);
		if (!pImage)
		{
			printf("Fail to pMalloc in Sift_Match_Path\n");
			return;
		}
		if (!bLoad_Image(File, &oImage, 0, 0, 0, 1, poMem_Mgr))
			return;
		RGB_2_Gray(oImage, pImage);
		Free_Image(poMem_Mgr, oImage);

		iSize = iGet_Sift_Detect_Size(oImage.m_iWidth, oImage.m_iHeight);
		pCur = (unsigned char*)pMalloc(poMem_Mgr, iSize);
		if (!pCur)
			return;
		Get_Sift_Feature(pImage, oImage.m_iWidth, oImage.m_iHeight,o_min, pCur, iSize,&oSift_Image.m_iCount);
		//printf("Detect %d\n", i);
		Free(poMem_Mgr, pImage);
		oSift_Image.m_pFeature = (Sift_Feature*)pCur;
		pSift_Image_Arr[i] = oSift_Image;
		iSize = oSift_Image.m_iCount * sizeof(Sift_Feature);
		Shrink(poMem_Mgr, pCur, iSize);
		pSift_Image_Arr[i] = oSift_Image;
		//printf(".");
		printf("Feature Count:%d\n", oSift_Image.m_iCount);
	}

	//此处算Sift_Image的Match
	//Disp_Mem(poMem_Mgr, 0);
	iSize = iGet_Sift_Match_Size(pSift_Image_Arr, iImage_Count);
	//Temp code
	iSize *= 2;

	pCur = pStart = (unsigned char*)pMalloc(poMem_Mgr, iSize);
	if (!pCur)
		return;	//不够内存，严重错误

	//分配Match, 一共分走了 iImage_Count*iImage_Count*sizeof(Sift_Match_Item)
	for (i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pMatch = (Sift_Match_Item*)pCur;
		memset(pCur, 0, iImage_Count * sizeof(Sift_Match_Item));
		pCur += iImage_Count * sizeof(Sift_Match_Item);
	}//出来以后，内存移到 pStart + iImage_Count*iImage_Count*sizeof(Sift_Match_Item)

	//再分配一次Desc, 整齐点以便于后面提速
	for (i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pDesc = (unsigned char(*)[128])pCur;
		pCur += pSift_Image_Arr[i].m_iCount * 128;
	}
	Copy_Desc(pSift_Image_Arr, iImage_Count, pSift_Image_Arr->m_pDesc[0]);

	Sift_Match_Item oMatch_Item;
	for (i = 0; i < iImage_Count; i++)
	{
		for (j = i + 1; j < iImage_Count; j++)
		{
			oMatch_Item = { (short)i,(short)j,0,(unsigned short(*)[2])pCur };
			if (pCur + pSift_Image_Arr[i].m_iCount * sizeof(unsigned short) * 2 - pStart > iSize)
			{//判断是否越界
				printf("Insufficient memory\n");
				return;
			}
			Sift_Match_2_Image(pSift_Image_Arr[i], pSift_Image_Arr[j], i, j, &oMatch_Item);
			pCur += ALIGN_SIZE_128(oMatch_Item.m_iMatch_Count * 2 * sizeof(unsigned short));
			pSift_Image_Arr[j].m_pMatch[i] = pSift_Image_Arr[i].m_pMatch[j] = oMatch_Item;
			printf("i:%d j:%d Match_Count:%d \n", i, j, oMatch_Item.m_iMatch_Count);
		}
	}

	////再分配一次Desc, 整齐点以便于后面提速
	//for (i = 0; i < iImage_Count; i++)
	//{
	//	pSift_Image_Arr[i].m_pDesc = (unsigned char(*)[128])pCur;
	//	pCur += (((pSift_Image_Arr[i].m_iCount + 15) >> 4) << 4) * 128;
	//}
	//Copy_Desc_1(pSift_Image_Arr, iImage_Count, pSift_Image_Arr->m_pDesc[0]);
	//Sift_Match_Item oMatch_Item;
	//for (i = 0; i < iImage_Count; i++)
	//{
	//	for (j = i + 1; j < iImage_Count; j++)
	//	{
	//		oMatch_Item = { (short)i,(short)j,0,(unsigned short(*)[2])pCur };
	//		if (pCur + pSift_Image_Arr[i].m_iCount * sizeof(unsigned short) * 2 - pStart > iSize)
	//		{//判断是否越界
	//			printf("Insufficient memory\n");
	//			return;
	//		}
	//		Sift_Match_2_Image_1(pSift_Image_Arr[i], pSift_Image_Arr[j], i, j, &oMatch_Item);
	//		pCur += ALIGN_SIZE_128(oMatch_Item.m_iMatch_Count * 2 * sizeof(unsigned short));
	//		pSift_Image_Arr[j].m_pMatch[i] = pSift_Image_Arr[i].m_pMatch[j] = oMatch_Item;
	//		printf("i:%d j:%d Match_Count:%d \n", i, j, oMatch_Item.m_iMatch_Count);
	//	}
	//}

	//最后将Feature 抄到 Match_Map
	//pCur = (unsigned char*)poMap->m_pMatch[0].m_pMatch;
	pCur = (unsigned char*)poMap->m_pMatch[0].m_pPoint_1;
	for (i = 0; i < iImage_Count; i++)
	{
		oSift_Image = pSift_Image_Arr[i];
		Sift_Feature* pFeature_A = oSift_Image.m_pFeature;
		for (j = i + 1; j < iImage_Count; j++)
		{
			oSift_Image = pSift_Image_Arr[j];
			Sift_Feature* pFeature_B = oSift_Image.m_pFeature;
			oMatch_Item = pSift_Image_Arr[i].m_pMatch[j];
			int iIndex = iUpper_Triangle_Cord_2_Index(j, i, iImage_Count);
			//Sift_Simple_Match_Item oMatch_Item_1 = poMap->m_pMatch[i * iImage_Count + j];
			Sift_Simple_Match_Item oMatch_Item_1 = poMap->m_pMatch[iIndex];
			oMatch_Item_1.m_iImage_A = i;
			oMatch_Item_1.m_iImage_B = j;
			oMatch_Item_1.m_iMatch_Count = oMatch_Item.m_iMatch_Count;
			oMatch_Item_1.m_pPoint_1 = (float(*)[2])pCur;
			oMatch_Item_1.m_pPoint_2 = oMatch_Item_1.m_pPoint_1 + oMatch_Item.m_iMatch_Count;
			//oMatch_Item_1.m_pMatch =(float(*)[2][2])pCur;
			for (int k = 0; k < oMatch_Item.m_iMatch_Count; k++)
			{
				oMatch_Item_1.m_pPoint_1[k][0] = pFeature_A[oMatch_Item.m_Match[k][0]].x;
				oMatch_Item_1.m_pPoint_1[k][1] = pFeature_A[oMatch_Item.m_Match[k][0]].y;
				oMatch_Item_1.m_pPoint_2[k][0] = pFeature_B[oMatch_Item.m_Match[k][1]].x;
				oMatch_Item_1.m_pPoint_2[k][1] = pFeature_B[oMatch_Item.m_Match[k][1]].y;
			}
			//poMap->m_pMatch[j*iImage_Count + i]= poMap->m_pMatch[i * iImage_Count + j] = oMatch_Item_1;
			poMap->m_pMatch[iIndex]= oMatch_Item_1;
			pCur += oMatch_Item.m_iMatch_Count * 4 * sizeof(float);
		}
	}
	iSize=(int)(pCur- poMap->m_pBuffer);
	Shrink(poMem_Mgr, poMap->m_pBuffer, iSize);
	//再将Sift_Image_Arr删除
	//Disp_Mem(poMem_Mgr, 0);
	Free(poMem_Mgr, pSift_Image_Arr[0].m_pMatch);
	for (i = 0; i < iImage_Count; i++)
		Free(poMem_Mgr, pSift_Image_Arr[i].m_pFeature);
	Free(poMem_Mgr, pSift_Image_Arr);
	Free(poMem_Mgr, pSift_Image_Arr[0].m_pFile_Name);
	//Disp_Mem(poMem_Mgr, 0);
	poMap->m_iImage_Count = iImage_Count;
	return;
}

void Copy_Match_Map(Sift_Match_Map oSource, Sift_Match_Map* poDest)
{//此处假定poDest虚有其壳
	int i,j, iIndex,iSize;	// = iGet_Simple_Match_Size(oSource.m_iImage_Count);
	char* pFile_Name;
	unsigned char* pCur;
	Sift_Simple_Match_Item oMatch;
	*poDest = oSource;

	iSize = (int)(((unsigned char*)oSource.m_pMatch[0].m_pPoint_1) - oSource.m_pBuffer);
	for (i = 0; i < oSource.m_iImage_Count; i++)
	{
		for (j = i + 1; j < oSource.m_iImage_Count; j++)
		{
			iIndex = iUpper_Triangle_Cord_2_Index(j, i, oSource.m_iImage_Count);
			iSize += oSource.m_pMatch[iIndex].m_iMatch_Count * 4 * sizeof(float);
		}
	}
	poDest->m_pBuffer = (unsigned char*)malloc(iSize);
	memcpy(poDest->m_pBuffer, oSource.m_pBuffer, iSize);
	poDest->m_pImage_Arr = (Image_Info*)poDest->m_pBuffer;
	pFile_Name =(char*)(poDest->m_pBuffer + oSource.m_iImage_Count * sizeof(Image_Info));
	for (i = 0; i < oSource.m_iImage_Count; i++)
	{
		poDest->m_pImage_Arr[i].m_pFile_Name = pFile_Name;
		pFile_Name += strlen(pFile_Name) + 1;
	}
	pCur = (unsigned char*)pFile_Name;
	pCur += 4 - ((unsigned long long)pCur) & 3;

	poDest->m_pMatch = (Sift_Simple_Match_Item*)pCur;

	pCur += iGet_Upper_Triangle_Size(oSource.m_iImage_Count)* sizeof(Sift_Simple_Match_Item);
	for (i = 0; i < oSource.m_iImage_Count; i++)
	{
		for (j = i + 1; j < oSource.m_iImage_Count; j++)
		{
			//oMatch = poDest->m_pMatch[i * oSource.m_iImage_Count + j];
			iIndex = iUpper_Triangle_Cord_2_Index(j, i, oSource.m_iImage_Count);
			oMatch = oSource.m_pMatch[iIndex];
			oMatch.m_pPoint_1 = (float(*)[2])pCur;
			oMatch.m_pPoint_2 = oMatch.m_pPoint_1 + oMatch.m_iMatch_Count;
			poDest->m_pMatch[iIndex] = oMatch;
			pCur += oMatch.m_iMatch_Count * 4 * sizeof(float);
		}
	}
	return;
}

void Sift_Match_Path(const char* pcPath, Sift_Match_Map* poMap, int o_min)
{//多用于实验，故此不做空间优化，搞个100兆了事
	Sift_Match_Map oMatch_Map;
	Mem_Mgr oMem_Mgr;
	int i,iImage_Count, iSize = (int)strlen(pcPath);
	unsigned char* pBuffer = (unsigned char*)malloc(1024000);
	char* pFile_Name, File[256];
	Image oImage, oLargest_Image = {};
	if (pcPath[iSize - 1] != '\\')
		sprintf(File, "%s\\*.bmp", pcPath);
	else
		sprintf(File, "%s*.bmp", pcPath);
	iImage_Count = iGet_File_Count(File);
	if (!iImage_Count)
	{
		printf("The path has no .bmp file\n");
		*poMap = {};
		return;
	}
	Get_All_File(File,(char*)pBuffer);
	pFile_Name = (char*)pBuffer;
	for (i = 0; i < iImage_Count; i++)
	{
		sprintf(File, "%s\\%s", pcPath, pFile_Name);
		Get_Image_Info(File, &oImage);
		if (oImage.m_iWidth * oImage.m_iHeight > oLargest_Image.m_iWidth * oLargest_Image.m_iHeight)
			oLargest_Image = oImage;
		pFile_Name += strlen(pFile_Name)+1;
	}
	free(pBuffer);
	iSize = iGet_Sift_Detect_Size(oLargest_Image.m_iWidth, oLargest_Image.m_iHeight);

	//大约，不精确估计
	Init_Mem_Mgr(&oMem_Mgr, (unsigned long long)(iSize * 1.2 + iImage_Count * MAX_KEY_POINT_COUNT * sizeof(Sift_Feature)), 1024, 997);
	Sift_Match_Path_1(pcPath, &oMatch_Map, &oMem_Mgr);

	Copy_Match_Map(oMatch_Map, poMap);

	Free_Mem_Mgr(&oMem_Mgr);
	return;
}

template<typename _T>void Shrink_Match_Point(_T(**ppPoint_1)[2], _T(**ppPoint_2)[2], unsigned char Mask[], int iCount)
{//将匹配的点留下，其他废掉，缩内存
	_T(*pPoint_1)[2], (*pPoint_2)[2], (*pCur_1)[2], (*pCur_2)[2];
	pCur_1 = pPoint_1 = *ppPoint_1, pCur_2 = pPoint_2 = *ppPoint_2;
	int i, iInlier_Count;
	for (iInlier_Count = i = 0; i < iCount; i++)
	{
		if (Mask[i])
		{
			pCur_1[iInlier_Count][0] = pPoint_1[i][0];
			pCur_1[iInlier_Count][1] = pPoint_1[i][1];
			pCur_2[iInlier_Count][0] = pPoint_2[i][0];
			pCur_2[iInlier_Count][1] = pPoint_2[i][1];
			iInlier_Count++;
		}
	}
	memcpy(pCur_1 + iInlier_Count, pCur_2, iInlier_Count * 2 * sizeof(_T));
	realloc(pPoint_1, iInlier_Count * 2 * 2 * sizeof(_T));
	*ppPoint_1 = pCur_1;
	*ppPoint_2 = pCur_1 + iInlier_Count;
	return;
}