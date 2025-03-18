//����ڴ�ռ���ٵ��Sift���൱��ԭ�㷨��1/3
#include "immintrin.h"
//��������ͷ�ļ��Ǳ���ģ��ʴ�Ҫ������Ӧ��.cppһ�����
#include "Common.h"		
#include "sift.h"
#include "Image.h"
using namespace std;

#define VL_SIFT_BILINEAR_ORIENTATIONS 1
typedef struct Sift_Keypoint {//�ؼ���
	short ix, iy;		// x,y
	char is;			//�ڼ���
	char o;				//Current octave���ڼ���
	float x, y, s;		//����̩���������С��ֵ
	float sigma;		
}Sift_Keypoint;

typedef struct Sift_Feature_Level {
	Sift_Feature* m_pFeature;
	Sift_Keypoint* m_pKeypoint;
	int m_iFeature_Count, m_iKeypoint_Count;
}Sift_Feature_Level;

typedef struct Sift {
	int O;               //һ�����Ը�����֣�����û�б�Ҫ��
	int S;               //ÿ�ֶ��ٲ㣬Ҳ�ǿ�������
	int o_min;           //����-1��Ҳ���Ը���һ��������4K�������û�б�Ҫ����ô��
	int s_min;           /**< minimum level index. */
	int s_max;           /**< maximum level index. */
	int o_cur;           /**< current octave. */

	//���������һ����6���ֱ�˸�˹�˲���5��DoG, 3���ݶ�
	//�ʴˣ�����˵�ڴ����3��DoG,3��Octave����
	//���������ڴ氲��
	//|  /   | G_0 | DoG_0 | G_1 | DoG_1 | G_2
	//|DoG_2 | G_3 | DoG_3 | G_4 | DoG_4 | G_5
	float* m_pDoG[3];		//current DoG data, DoG= difference of Gaussians ��˹��
	//ע�⣬��m_pDoG���˹ͼ֮�仹Ҫ����һ�У��������һ�еĻ���
	float* m_pGauss[3];		//װ��ͬ�ֱ��ʵĸ�˹ͼ��Ϊ��������ԣ���ԭ��octave����

	float* m_pGrad;			//���°����ڴ���һ�£�m_pGrad��Octave DoG����
	float* m_pTemp;			//��Ӧ f->temp�����õ����m_pDoG

	int grad_o;					//GSS gradient data octave.
	int m_iWidth, m_iHeight;	//Դͼ�ĳ���

	//���¼���Sigma�ǹ��ڸ�˹ͼ��ģ���̶ȣ������ٴ����У����������ܽ�
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
	Sift_Keypoint* m_pKey_Point;	//Keypointֻ�Ǻ�ѡ�㣬��������Feature

	int m_iMax_Feature_Count;
	int m_iFeature_Count;			//���յ�Feature Count
	//�˴�������յ�������ʽFeature
	Sift_Feature* m_pFeature;		//���յ�Feature���ڣ�Ҫ��ֻ����һ�����

	unsigned char* m_pBuffer_Start;	//ÿһ���ڴ濪ʼ֮��
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
{//ͼƥ��׶Σ�Ԥ�Ʒ�ֵ���Ҫ�ö����ڴ�
	int i, iSize;
	//��һ���֣�����Match, һ�������� iImage_Count*iImage_Count*sizeof(Sift_Match_Item)
	iSize = iImage_Count * iImage_Count * sizeof(Sift_Match_Item);

	//�ڶ����֣��ٷ���һ��Desc, ������Ա��ں�������
	for (i = 0; i < iImage_Count; i++)
		iSize += Sift_Image_Arr[i].m_iCount * 128;

	//�������֣�Match�� һ���� iImage_Count*(iImage_Count-1)/2 ��Ƚϣ���ſ����ռ�
	iSize += ALIGN_SIZE_128(iImage_Count * (MAX_KEY_POINT_COUNT * 4));
	iSize += 128;	//������ع������
	return iSize;
}
int iGet_Sift_Detect_Size(int iWidth, int iHeight,int o_min)
{//���������׶Σ�Ԥ����һ��Ҫ�ö����ڴ�
//o_min: ����Ҫ���ǽ�����һ������-1��ʼ���ڴ�Ͳ�һ��
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

	//DoG����С����
	iSize += iSize_1 * 3 + iWidth*3*sizeof(float);

	//���һ�У�������Grad���һ��
	//iSize += iWidth * 2 * sizeof(float);

	//grad ��Temp ����
	//iSize += iSize_1 * 2;
	iSize = ALIGN_SIZE_128(iSize) + 128;	//�������128�ֽ�

	return iSize;
}

static void Init_Sift(Sift* poSift, int iOctave_Count, int iLayer_Count, 
	int o_min, int iWidth, int iHeight, unsigned char* pBuffer)
{//�˴�Ӧ��һ���Է�������е��ڴ�
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

	//�����ڴ�
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
{//�Ա��ֽ��� octave, DoG, grad�����ڴ氲��
	Sift oSift = *poSift;
	int i, iSize , iSize_1;
	//unsigned char* pOrg = pCur;

	iSize = ALIGN_SIZE_128(w * h * sizeof(float));
	iSize_1 = ALIGN_SIZE_128(iSize + w * sizeof(float));

	//��˹ͼ��DoG÷������,���������ڴ氲��
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
	{//ԭ����[0]ͼ����һ�ε��²���,������[1],���ԣ�ѭ������һ��
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

	////���һ��
	//oSift.m_pLast_Grad_Line = (float*)pCur;
	//pCur += oSift.octave_width * 2* sizeof(float);

	//oSift.m_pTemp = oSift.m_pDoG[0];	//Temp�뵱ǰ����
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
{//ͼ�񰴱߳�����һ��
	//������չ
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

	//�ٶ�pDest����һ�εĲ�ֵ
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
{//�߳�����
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
{//�����ԭ����Sift_Detect��ͬ��������DoG
	Sift oSift = *poSift;
	int i, iSize;

	//����һ��sigma
	float fSigma_a, fSigma_b;
	//�����´��뿴����˹�˵Ĵ�С��ͼ���С�޹�
	if (oSift.o_cur == oSift.o_min)
	{//�൱��Process_First_Octave
		fSigma_a = oSift.sigma0 * (float)pow(oSift.sigmak, oSift.s_min);
		fSigma_b = oSift.sigman * (float)pow(2.0f, -oSift.o_min);
	}else
	{//�൱��Process_Next_Octave
		int s_best = min(oSift.s_min + oSift.S, oSift.s_max);
		fSigma_a = (float)(oSift.sigma0 * pow((float)oSift.sigmak, (float)oSift.s_min));
		fSigma_b = (float)(oSift.sigma0 * pow((float)oSift.sigmak, (float)(s_best - oSift.S)));
	}
	float sd;
	int r;
	float* pFilter;

	//�ȸ��0��Octave��������һ���Ӧ��DoG
	oSift.m_pTemp = oSift.m_pDoG[0];
	if (fSigma_a > fSigma_b)
	{
		sd = (float)sqrt(fSigma_a * fSigma_a - fSigma_b * fSigma_b);
		//�ɼ��˴���r��sigma�仯���仯
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
	{//���������Octave��DoG
		pOctave_0 = oSift.m_pGauss[i];
		pOctave_1 = oSift.m_pGauss[1 + i];
		oSift.m_pTemp = oSift.m_pDoG[1 + i];
		sd = oSift.dsigma0 * (float)pow(oSift.sigmak, i);

		r = (int)max((float)ceil(4.0 * sd), 1.f);
		Gen_Gauss_Filter(r, sd, &pFilter);
		//������һ����������һ��
		Gauss_Filter_AVX512(pOctave_0, oSift.octave_width, oSift.octave_height, pOctave_1, oSift.m_pTemp, r, pFilter);
		free(pFilter);

		//���0��DoG
		pt = oSift.m_pDoG[i];
		pEnd = pt + iSize;
		for (; pt < pEnd; Addr_3 = _mm256_add_epi64(Addr_3, _mm256_set1_epi64x(16 * 4))   /*pt += 16, pOctave_0 += 16, pOctave_1 += 16*/)
			*(__m512*)pt = _mm512_sub_ps(*(__m512*)pOctave_1, *(__m512*)pOctave_0);
	}
}

static void Get_Keypoint_1(Sift* poSift, int iDoG_Mid, int s, Sift_Feature_Level* poFeature_Level)
{//ע�⣺ iDoG_Midָ���Ǳ��δ���DoG�м����oSift.m_pDoG���߼�λ��
//			s:��ӦsiftԴ�����е�s
	Sift oSift = *poSift;
	int iSize = oSift.octave_width * oSift.octave_height;
	int const    xo = 1;					/* x-stride */
	int const    yo = oSift.octave_width;	/* y-stride */
	iSize = ALIGN_SIZE_128(iSize)*2;
	//�˴�����Ҫ�ģ�������ԭ���ķ���
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

	//��DoG[1]��Ϊ���Ĳ㣬ͨ��26����ȷ�������Key_Point
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

		//������
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
	
	//����һ����keypoints��һ��ɸѡ
	int dx, dy, iter;
	float Dx = 0, Dy = 0, Ds = 0, Dxx = 0, Dyy = 0, Dss = 0, Dxy = 0, Dxs = 0, Dys = 0;
	float A[3][3], b[3];	//�������Ax=b������ֱ�ӵ��ý����Է���
	int iValid_Point_Count = 0;
	float* pDoG = oSift.m_pDoG[iDoG_Mid_Index];
	float xper = (float)pow(2.0, oSift.o_cur);

	for (k = 0; k < nkeys; k++)
	{
		oKey_Point = pPoint[k];
		//������һ�׵�
		Dx = 0, Dy = 0, Ds = 0, Dxx = 0, Dyy = 0, Dss = 0, Dxy = 0, Dxs = 0, Dys = 0;
		dx = dy = 0;
		x = oKey_Point.ix;
		y = oKey_Point.iy;

		for (iter = 0; iter < 5; iter++)
		{//�о��������࣬Ҳ�п���Ϊ�˲����ھֲ���С���������Ӱ��ȵ�ԭ���ټ�������һ��
			//����һ���x,y���Է���������Ȳ����ܴ���1��������9����������
			x += dx;
			y += dy;
			pt = pDoG + xo * x + yo * y;

			//��Ȼ������һ��ƫ�����űȿ�
			Dx = 0.5f * (pt[1] - pt[-1]);
			Dy = 0.5f * (pt[oSift.octave_width] - pt[-oSift.octave_width]);
			Ds = 0.5f * (pt[so_Next] - pt[so_Pre]);

			//��һ��Hesse�����ɶ���ƫ�����
			Dxx = (pt[1] + pt[-1] - 2.0f * (*pt));
			Dyy = (pt[oSift.octave_width] + pt[-oSift.octave_width] - 2.0f * (*pt));
			Dss = (pt[so_Next] + pt[so_Pre] - 2.0f * (*pt));

			Dxy = 0.25f * (pt[oSift.octave_width + 1] + pt[-oSift.octave_width - 1] - pt[-oSift.octave_width + 1] - pt[oSift.octave_width - 1]);	//  at(+1, +1, 0) + at(-1, -1, 0) - at(-1, +1, 0) - at(+1, -1, 0));
			Dxs = 0.25f * (pt[so_Next + 1] + pt[so_Pre - 1] - pt[so_Pre + 1] - pt[so_Next - 1]);	//  (at(+1, 0, +1) + at(-1, 0, -1) - at(-1, 0, +1) - at(+1, 0, -1));
			Dys = 0.25f * (pt[so_Next + oSift.octave_width] + pt[so_Pre - oSift.octave_width] - pt[so_Pre + oSift.octave_width] - pt[so_Next - oSift.octave_width]);	//at(0, +1, +1) + at(0, -1, -1) - at(0, -1, +1) - at(0, +1, -1));

			//����Գ���hesse
			A[0][0] = Dxx;
			A[1][1] = Dyy;
			A[2][2] = Dss;
			A[0][1] = A[1][0] = Dxy;
			A[0][2] = A[2][0] = Dxs;
			A[1][2] = A[2][1] = Dys;

			b[0] = -Dx;
			b[1] = -Dy;
			b[2] = -Ds;

			//�˴�Ϊ̩�������������ҵ��Ĺؼ��㲢�������ļ�ֵ�㣬�ڲ�ֵ�������£���ֵ��
			//Ӧ���������أ�Ҳ�����������Ϲؼ��������ϡ��ʴ�Ҫ���ڹؼ�����Χ��һ��̩��չ��
			//�õ�һ������ f(x)= f(x0) + J*(delta x) + 1/2(delta x') * H * (delta x)
			//�� f(x) ��ֵ min f(x) ��������һ����ƫ�� J + H*(delta x) ��ʹ��=0, �� delta x
			//������ H*x= -J ,��� x��������� delta x
			for (j = 0; j < 3; j++)
			{//jΪ��
				float maxa = 0;
				float maxabsa = 0;
				int    maxi = -1;
				float tmp;

				/* look for the maximally stable pivot */
				for (i = j; i < 3; i++)
				{//��Ȼ�ҵ�j��������ֵ������Ԫ
					float a = A[i][j];
					float absa = abs(a);
					if (absa > maxabsa)
					{
						maxa = a;
						maxabsa = absa;
						maxi = i;
					}
				}

				//AΪ���׵�����������׵��������һ��<eps���˳�����ʾ������������ϴ��ڼ�ֵ��?
				if (maxabsa < 1e-10f)
				{
					b[0] = 0;
					b[1] = 0;
					b[2] = 0;
					break;
				}
				i = maxi;

				//����Խ��Խ������Ԫ�������Է��� Ax=b
				/* swap j-th row with i-th row and normalize j-th row */
				for (jj = j; jj < 3; ++jj)
				{//�����Ԫ�����������j�н��������ҹ��
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

			// backward substitution �ش�����������
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
			//�������������
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
{//������Ϊ���㷨���Ը���࣬��sift�������㿴���ǵ��������󣬱�Ե��͹����㶼�������ʴ�����
	//ͼ�������ı��ϵĵ�����ϲ����ǽǵ㡣��������г�ֵ�������֤������룬����ϵ���ֵ���Բ���
	float* pSource, * pSource_1, * pEnd_1, * pEnd/*,*pEnd_Align_16*/;
	float* pGrad, * pGrad_1;
	__m512 gx, gy, Value;
	__m512i vIndex_Grad;
	int i, iResult, iValue, h_Minus_1 = h - 1, w_x2 = w * 2;
	for (i = 0; i < 16; i++)
		((int*)&vIndex_Grad)[i] = i * 8;
		//vIndex_Grad.m512i_i32[i] = i * 8;

	//�Ȳ����������ҽ�4���㣬�б�Ҫʡ�Ե�

	//��һ�㣬ûʲô�ø���
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

	//���һ��
	pGrad = grad + 2;
	pSource = src + 1;
	pEnd = src + w - 1;	//ͷβ���㲻��
	//pEnd_Align_16 = pSource + (unsigned int)((pEnd - pSource) & 0xFFFFFFF0);
	iValue = (pEnd - pSource) & 0xF;
	while (pSource < pEnd)
	{//�����ƽ�������������ν��������ؼ���
		//gx = 0.5f * (src[+xo] - src[-xo]);
		gx = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + 1), *(__m512*)(pSource - 1)));
		//gy = src[+yo] - src[0];
		gy = _mm512_sub_ps(_mm512_loadu_ps(pSource + w), _mm512_loadu_ps(pSource));

		//sqrt(gx * gx + gy * gy)
		Value = _mm512_sqrt_ps(_mm512_add_ps(_mm512_mul_ps(gx, gx), _mm512_mul_ps(gy, gy)));
		_mm512_i32scatter_ps(pGrad, vIndex_Grad, Value, 1);

		//�˴��о����Լ� arctg���жϣ�ֵ��Ϊ(-PI/2,PI/2);
		Value = _mm512_atan_ps(_mm512_div_ps(gy, gx));
		iResult = _mm512_cmp_ps_mask(Value, _mm512_set1_ps(0), _CMP_LT_OS);
		Value = _mm512_mask_add_ps(Value, iResult, Value, _mm512_set1_ps(PI * 2));
		_mm512_i32scatter_ps(pGrad + 1, vIndex_Grad, Value, 1);
		pSource += 16;
		pGrad += 32;
	}

	//��һ�����һ��
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

	//���м���
	pSource_1 = src + w + 1;
	pEnd_1 = src + h_Minus_1 * w;
	pGrad_1 = grad + w * 2 + 2;
	pEnd = pSource_1 + w - 2;
	//���´��뻹��Bug, ����ѭ���е����һ�л��ǻ���ֲȹ����������
	for (; pSource_1 < pEnd_1; pSource_1 += w, pGrad_1 += w_x2, pEnd += w)
	{
		pSource = pSource_1;
		pGrad = pGrad_1;

		//�м��е�һ��
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
			//gx = 0.5f * (src[+xo] - src[-xo]);	//����û�а�3x3ģ��㣬͵����������һ��һά��ģ��
			gx = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + 1), _mm512_loadu_ps(pSource - 1)));
			//gy = 0.5f * (src[+yo] - src[-yo]);	//ͬ��
			gy = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + w), _mm512_loadu_ps(pSource - w)));

			//sqrt(gx * gx + gy * gy)
			Value = _mm512_sqrt_ps(_mm512_add_ps(_mm512_mul_ps(gx, gx), _mm512_mul_ps(gy, gy)));
			_mm512_i32scatter_ps(pGrad, vIndex_Grad, Value, 1);

			//�˴��о����Լ� arctg���жϣ�ֵ��Ϊ(-PI/2,PI/2);
			Value = _mm512_atan_ps(_mm512_div_ps(gy, gx));
			iResult = _mm512_cmp_ps_mask(Value, _mm512_set1_ps(0), _CMP_LE_OS);
			Value = _mm512_mask_add_ps(Value, iResult, Value, _mm512_set1_ps(PI * 2));
			_mm512_i32scatter_ps(pGrad + 1, vIndex_Grad, Value, 1);

			pSource += 16;
			pGrad += 32;
		}

		//�м������һ��
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

	//���һ��
	pGrad = pGrad_1;
	pSource = pSource_1;
	//pEnd = pSource_1 + w - 2;	//ͷβ���㲻��
	iValue = (((w - 2) >> 4) << 4);
	pEnd = pSource + iValue;
	iValue =(1<<( w - 2 - iValue))-1;

	//���һ�е�һ��
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

		//�˴��о����Լ� arctg���жϣ�ֵ��Ϊ(-PI/2,PI/2);
		Value = _mm512_atan_ps(_mm512_div_ps(gy, gx));
		iResult = _mm512_cmp_ps_mask(Value, _mm512_set1_ps(0), _CMP_LT_OS);
		Value = _mm512_mask_add_ps(Value, iResult, Value, _mm512_set1_ps(PI * 2));
		_mm512_i32scatter_ps(pGrad + 1, vIndex_Grad, Value, 1);

		pSource += 16;
		pGrad += 32;
	}

	//���һ������
	gx = _mm512_mul_ps(_mm512_set1_ps(0.5f), _mm512_sub_ps(_mm512_loadu_ps(pSource + 1), _mm512_loadu_ps(pSource - 1)));
	gy = _mm512_sub_ps(_mm512_loadu_ps(pSource), _mm512_loadu_ps(pSource - w));

	Value = _mm512_sqrt_ps(_mm512_add_ps(_mm512_mul_ps(gx, gx), _mm512_mul_ps(gy, gy)));
	_mm512_mask_i32scatter_ps(pGrad,iValue, vIndex_Grad, Value, 1);

	//�˴��о����Լ� arctg���жϣ�ֵ��Ϊ(-PI/2,PI/2);
	Value = _mm512_atan_ps(_mm512_div_ps(gy, gx));
	iResult = _mm512_cmp_ps_mask(Value, _mm512_set1_ps(0), _CMP_LT_OS);
	Value = _mm512_mask_add_ps(Value, iResult, Value, _mm512_set1_ps(PI * 2));
	_mm512_mask_i32scatter_ps(pGrad + 1,iValue, vIndex_Grad, Value, 1);

	//���һ�����һ��
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

	////����
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
{//�о��˴��ǹؼ���
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

	//��һԽ��
	if (xi < 0 || xi > w - 1 || yi < 0 || yi > h - 1 ||
		si < poSift->s_min + 1 || si > poSift->s_max - 2)
		return 0;	//�������֣�Ϊʲô�����Խ��������о���α�ж�

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
			//Ԥ���Ѿ����������Update_Gradient�У�������ʲô����
			mod = *(pt + xs * xo + ys * yo);		//��Ȼ�����ݶȣ�ģ��Խ�������Խ��
			ang = *(pt + xs * xo + ys * yo + 1);	//����ǽǶȣ��õ���ݶȷ���
			fbin = nbins * ang / (2 * PI);			//һ��Բ��36�֣�fbin��ang����һ��

#if defined(VL_SIFT_BILINEAR_ORIENTATIONS)
			{
				int bin = (int)floor(fbin - 0.5f);
				float rbin = fbin - bin - 0.5f;
				//һ����ĽǶ�һ��������ݽǶȣ�����λ�þ����ķݶ���ķ��ٵ㣬Ȼ��
				//�ݶȵĹ���Ҳ�ֿ������֣����䵽���ڵ������Ƕ��ϡ����������Ǹ�Сѧ����
				//����ͼ�������������֮�䣬��Ȼ�Ǹ�ռ0.5��������Ҷ�㣬�����ھ��ٵ㡣
				//������һ������

				// �˴�ɵ�ƣ����˸����������ڸ�λ��
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
			//���Լ�Ϊ���ģ����������ھӣ�������ƽ����Ϊ��ֵ
			//Ȼ���������ڼ����Ѵﵽƽ��Ч��
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
		float hm = hist[(i - 1 + nbins) % nbins];	//h0�Ƕ��ԱߵĽǶ�
		float hp = hist[(i + 1 + nbins) % nbins];

		/* is this a peak? */
		if (h0 > 0.8 * maxh && h0 > hm && h0 > hp)
		{//����W�㹻�󣬱�ʾ���ڱ��ؼ����������Χ�㹻�󣬱���뾶�ﵽ8���أ����ʱ����ȫ�����Ҷ�����ܴ��·��
			//�붼�뵽����ɽ��·����ֹһ������ô���ʱ����ſ��ݶ��Ƿ��㹻�������һ���̶ȣ�����������ݶȵ�0.8����
			//�����γ���һ������,�������Ҳ���÷��������
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
{//�����Ͼ��������ĵ�λ��
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
{//�򵥼���
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

	int          xi = (int)(x + 0.5);	//�˴�xi�õ���������ֵ����������������Ϊ�������ƺ�ûʲô��
	int          yi = (int)(y + 0.5);
	int          si = k->is;

	float const st0 = (float)sin(angle0);
	float const ct0 = (float)cos(angle0);
	float const SBP = (float)(magnif * sigma + eps);
	//�������W����r
	int    const W = (int)(floor(sqrt(2.0) * SBP * (NBP + 1) / 2.0 + 0.5));

	int const binto = 1;          /* bin theta-stride */
	int const binyo = NBO * NBP;  /* bin y-stride */
	int const binxo = NBO;        /* bin x-stride */

	int bin, dxi, dyi;
	float* pt;
	float* dpt;

	//Խ����
	if (k->o != poSift->o_cur || xi < 0 || xi >= w ||
		yi < 0 || yi >= h - 1 || si    <  poSift->s_min + 1 ||
		si    >  poSift->s_max - 2)
		return;

	//8x4x4�飬128άDescriptor��Ҫ��һ�������仮��
	memset(descr, 0, sizeof(float) * NBO * NBP * NBP);

	//x0��ÿ��ķѶ��ٸ������� y0��ÿ�ж��ٸ������� ��Ϊÿ����ݶ���������������
	//1����2������  pt�ĺ����ǣ�point
	pt = &poSift->m_pGrad[xi * xo + yi * yo];

	//��������β²⣬һ��Descriptor�� Surfһ������4x4��sub block���ɣ�ÿ��
	//sub block�ĵĴ�СΪ2x2=4, ����һ����4x4lx4����Ϣ��ÿ����Ϣ�����������ֶ�
	//���ɣ� 1��ģ��������2������ ����һ��128ά
	//dpt�ĺ����� descriptor point������ֵ�ϣ�dpt��λ��descr[80], Ϊʲô��80��
	//������Ϊ���λ������4x4���м�飬 [2][2]�ϡ� 2*4*2*2 + 2*2*2=80�����
	dpt = descr + (NBP / 2) * binyo + (NBP / 2) * binxo;

#define atd(dbinx,dbiny,dbint) *(dpt + (dbint)*binto + (dbiny)*binyo + (dbinx)*binxo)

	//�����Ǽ��Ż���ǰ���ճ������沢�в������Ż�
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

	//�������鴰�ھ����ܴ�
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
						//һ��ָ������
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
{//��128άDescriptor������һ�ֹ�񻯡� ģΪһά����
	float fFactor;
	int i;
	for (i = 0, fFactor = 0; i < 128; i++)
		fFactor += Desc[i];
	fFactor = 1.f / fFactor;
	for (i = 0; i < 128; i++)
		Desc[i] = (float)sqrt(Desc[i] * fFactor);
}
void FeatureDescriptorsToUnsignedByte(float Desc[128], unsigned char Desc_i[128])
{//ֻ������һ�£����ܱ��	0, 7, 6, 5, 4, 3, 2, 1, 8, 15, 14, 13, 12, 11, 10, 9
	__m512i Re_Order = _mm512_set_epi32(9, 10, 11, 12, 13, 14, 15, 8, 1, 2, 3, 4, 5, 6, 7, 0);
	__m512 Value_16 = _mm512_set1_ps(512.f);

	for (int i = 0; i < 128; i += 16)
		*(__m128i*)& Desc_i[i] = _mm512_cvtepi32_epi8(_mm512_cvt_roundps_epi32(_mm512_mul_ps(_mm512_i32gather_ps(Re_Order, &Desc[i], 4), Value_16), _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}
static void Get_Feature(Sift* poSift, Sift_Feature_Level* poFeature_Level, int iDoG_Top, int iOctave_Top)
{//����������
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
	int iTemp,iOctave_Top_Index = iOctave_Top % 3;	//���ݵ�ǰ�������߲����Ƴ����ڴ������е�����
	//��һ���˹�˲�
	pOctave_0 = oSift.m_pGauss[iOctave_Top_Index];
	iTemp = (1 + iOctave_Top_Index) % 3;
	pOctave_1 = oSift.m_pGauss[iTemp];
	oSift.m_pTemp = oSift.m_pDoG[iOctave_Top_Index];	//���ñ���DoG
	sd = oSift.dsigma0 * (float)pow(oSift.sigmak, iOctave_Top);
	r = (int)max((float)ceil(4.0 * sd), 1.f);
	Gen_Gauss_Filter(r, sd, &pFilter);
	//������һ����������һ��
	Gauss_Filter_AVX512(pOctave_0, oSift.octave_width, oSift.octave_height, pOctave_1, oSift.m_pTemp, r, pFilter);
	free(pFilter);

	//��һ��DoG
	pt = oSift.m_pDoG[(iDoG_Top + 1) % 3];
	pEnd = pt + iSize;
	for (; pt < pEnd; Addr_3 = _mm256_add_epi64(Addr_3, _mm256_set1_epi64x(16 * 4))   /*pt += 16, pOctave_0 += 16, pOctave_1 += 16*/)
		*(__m512*)pt = _mm512_sub_ps(*(__m512*)pOctave_1, *(__m512*)pOctave_0);

	//���Ѿ����õ�DoG��һ��Keypoint���
	Get_Keypoint_1(&oSift, iDoG_Top, iOctave_Top - 2, &oFeature_Level);
	if (iOctave_Top == oSift.s_max)
	{//����ǰ�ĸ�˹���Ѿ�������߲㣬�򽫵�ǰ����һ���²�����������һ��
		Resize_Image_Half(oSift.m_pGauss[(iOctave_Top-1) % 3], oSift.octave_width, oSift.octave_height, oSift.m_pGauss[iOctave_Top%3]);
		//bSave_Image("c:\\tmp\\1.bmp", oSift.m_pGauss[iOctave_Top % 3], oSift.octave_width>>1, oSift.octave_height>>1);
	}
	//�����ݶ�, ��ʵӦ����Octave[iOctave_Top_Index-1]��֡�����ڼ���-1 ����ģ���и������⣬
	//���Խ� -1��ģ�ĳ� +2 %3���Ե�
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
{//��ʱҪ��Octave[iOctave_Reserve_Index]����Octave[0]��������Сһ��
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
{//Sift����ӿڣ���������ˬ�� ����������������
	Sift oSift;
	Sift_Feature_Level oFeature_Level,Feature_Level[4][3];
	int i,iFeature_Count = 0, iKeypoint_Count = 0;

	Init_Sift(&oSift, 4, 3,o_min, iWidth, iHeight, pBuffer);
	Get_w_h(oSift.m_iWidth, oSift.m_iHeight, oSift.octave_width, oSift.octave_height, oSift.o_cur);
	Arrange_Mem(&oSift, oSift.octave_width, oSift.octave_height, oSift.m_pBuffer_Start);
	oFeature_Level = { oSift.m_pFeature,oSift.m_pKey_Point,0,0 };

	//����o_min��ͼ���ŵ�Gauss[0]��
	if (oSift.o_min == -1)
		Resize_Image_x2(pImage, oSift.m_iWidth, oSift.m_iHeight, oSift.m_pGauss[0]);
	else if (oSift.o_min == 1)
		Resize_Image_Half(pImage, oSift.m_iWidth, oSift.m_iHeight, oSift.m_pGauss[0]);
	else
		memcpy(oSift.m_pGauss[0], pImage, oSift.m_iWidth * oSift.m_iHeight * sizeof(float));

	//bSave_Image("c:\\tmp\\1.bmp", oSift.m_pGauss[0], oSift.octave_width, oSift.octave_height);
	
	while (1)
	{//����ֱ�����Sift������w*2*h*2->w*h->w/2 * h/2 -> w/4 * h/4
		//�˴�Ҫһ��ǰ�˷�����ʡ���ڴ�
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
			{//�˴�������ȫ��Ҫ�������жϳ�������
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
		if (oSift.o_cur == 2)	//Ŀǰֻ�������㵽��2��octave����Ҫ��һ����֤
			break;
		//����һ��oSift
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
	//�����㹻�ĵ����ݣ�������ǰ�棬��Ϊ���ֻ��������������
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
{//���˽ӿ�1�����ú����Լ������ͷ��ڴ�
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
	//�����㹻�ĵ����ݣ�������ǰ�棬��Ϊ���ֻ��������������
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
{//��һ��ת�ã�16������һ�飬 һ��Ľṹ�� 128�� x 16 ���ֽڣ� ����������
	//�ٹ����������
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
	//�ٹ����������
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
{//����ŷ�Ͼ���
#pragma pack(16)
	register __m512i Value_32;
	register __m512i Sum;
	union {
		__m512i Sum_32;
		__m256i Sum_16[2];
	};
#pragma pack()

	//�˴���Ҫ��һ�ģ��ĳ�128�ֽڶ��룬���ܻ��Щ
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

	return _mm512_reduce_add_epi32(Sum);	//��Щ
}

void Get_Nearest_2_Point_1(unsigned char Pos[128], unsigned char(*Point)[128], int iPoint_Count, Neighbour_K* poNeighbour, int iIndex_A, Neighbour_K Neighbour_B[])
{//Neighbour_B ����ôһ������ÿһ���Neighbour��ʾ
	Neighbour_K oNeighbour;
	int i;
	unsigned int iDistance;
	unsigned char* pCur_Point;
	if (iPoint_Count < 2)
	{
		printf("Point Count less then 2\n");
		return;	//����Ҳ��
	}
	pCur_Point = Point[0];
	iDistance = iGet_Distance_AVX512(Pos, pCur_Point);
	oNeighbour.m_Buffer[0] = { 0, (unsigned int)iDistance };

	//�ٷ����Neighbour_B����ֵ
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
	//�ٷ����Neighbour_B����ֵ
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
		//Neighbour_B�ĵ�i�㣬ҲҪ�ж�һ���Ƿ�Ϊ�����
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
{//��Point�������ҵ���Posƥ��ĵ�
	Neighbour_K oNeighbour;
	int i, iDistance;
	unsigned char* pCur_Point;
	if (iPoint_Count < 2)
	{
		printf("Point Count less then 2\n");
		return;	//����Ҳ��
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
{//��B���ҵ�����ڣ�����B������ʵ�����
	Neighbour_K oNeighbour;
	const float kDistNorm = 1.0f / (512.0f * 512.0f);
	const float max_distance = 0.7f,
		max_ratio = 0.8f;
	float best_dist_normed, second_best_dist_normed;

	Get_Nearest_2_Point_1(Desc_A, pDesc_B, iCount_B, &oNeighbour,iIndex_A, Neighbour_B);
	
	//��ŷʽ�����Ϊ���
	Dot_1(Desc_A, pDesc_B[oNeighbour.m_Buffer[0].m_iIndex], oNeighbour.m_Buffer[0].m_iDistance);
	Dot_1(Desc_A, pDesc_B[oNeighbour.m_Buffer[1].m_iIndex], oNeighbour.m_Buffer[1].m_iDistance);
	//oNeighbour.m_Buffer[0].m_iDistance = iDot(Desc_A, pDesc_B[oNeighbour.m_Buffer[0].m_iIndex]);
	//oNeighbour.m_Buffer[1].m_iDistance = iDot(Desc_A, pDesc_B[oNeighbour.m_Buffer[1].m_iIndex]);

	//�ٽ�������߷���[0],��֮����[1]
	if (oNeighbour.m_Buffer[0].m_iDistance < oNeighbour.m_Buffer[1].m_iDistance)
		swap(oNeighbour.m_Buffer[0], oNeighbour.m_Buffer[1]);

	best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[0].m_iDistance, 1.0f));
	second_best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[1].m_iDistance, 1.0f));

	//�����ж����ŵ��Ƿ������ֵmax_distance,������˾�������ûë��
	if (best_dist_normed > max_distance)
		return -1;
	//���ж����ŵ��Ƿ�ȴ��ŵ��0.8�����󣬱�ʾ���ŵ�����ŵ㲻��̫Զ��
	//����ж���δ���ס����ŵ㲻�������ҵ���
	if (best_dist_normed >= max_ratio * second_best_dist_normed)
		return -1;

	return oNeighbour.m_Buffer[0].m_iIndex;
}

int iGet_Match(unsigned char Desc_A[128], unsigned char(*pDesc_B)[128], int iCount_B)
{//��B���ҵ�����ڣ�����B������ʵ�����
	Neighbour_K oNeighbour;
	const float kDistNorm = 1.0f / (512.0f * 512.0f);
	const float max_distance = 0.7f,
		max_ratio = 0.8f;
	float best_dist_normed, second_best_dist_normed;

	Get_Nearest_2_Point(Desc_A, pDesc_B, iCount_B, &oNeighbour);

	//��ŷʽ�����Ϊ���
	Dot(Desc_A, pDesc_B[oNeighbour.m_Buffer[0].m_iIndex], oNeighbour.m_Buffer[0].m_iDistance);
	Dot(Desc_A, pDesc_B[oNeighbour.m_Buffer[1].m_iIndex], oNeighbour.m_Buffer[1].m_iDistance);

	//�ٽ�������߷���[0],��֮����[1]
	if (oNeighbour.m_Buffer[0].m_iDistance < oNeighbour.m_Buffer[1].m_iDistance)
		swap(oNeighbour.m_Buffer[0], oNeighbour.m_Buffer[1]);

	best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[0].m_iDistance, 1.0f));
	second_best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[1].m_iDistance, 1.0f));

	//�����ж����ŵ��Ƿ������ֵmax_distance,������˾�������ûë��
	if (best_dist_normed > max_distance)
		return -1;
	//���ж����ŵ��Ƿ�ȴ��ŵ��0.8�����󣬱�ʾ���ŵ�����ŵ㲻��̫Զ��
	//����ж���δ���ס����ŵ㲻�������ҵ���
	if (best_dist_normed >= max_ratio * second_best_dist_normed)
		return -1;
	return oNeighbour.m_Buffer[0].m_iIndex;
}

void Sift_Match_2_Image_1(Sift_Image oImage_A, Sift_Image oImage_B, int iA_Index, int iB_Index, Sift_Match_Item* poMatch,Neighbour_K Neighbour_B[])
{//��������һ�ַ���
	int i, iMatch_B;
	Sift_Match_Item oMatch = { (short)iA_Index,(short)iB_Index,0,poMatch->m_Match };
	Neighbour_K oNeighbour;
	memset(Neighbour_B, 0xFF, oImage_B.m_iCount * sizeof(Neighbour_K));
	//����ֱ���ң����������ֲ���
	for (i = 0; i < oImage_A.m_iCount; i++)
	{//�˴�Ϊ�Ż��ؼ�
		if ((iMatch_B = iGet_Match_1(oImage_A.m_pDesc[i], oImage_B.m_pDesc, oImage_B.m_iCount,i, Neighbour_B)) != -1)
		{
			oMatch.m_Match[oMatch.m_iMatch_Count][0] = i;
			oMatch.m_Match[oMatch.m_iMatch_Count++][1] = iMatch_B;
		}
	}

	//����һ�δ�B��A�Ĳ���
	int j,iMatch_A;
	const float kDistNorm = 1.0f / (512.0f * 512.0f);
	const float max_distance = 0.7f,
		max_ratio = 0.8f;
	float best_dist_normed, second_best_dist_normed;

	for (i = j = 0; i < oMatch.m_iMatch_Count; i++)
	{
		iMatch_A = oMatch.m_Match[i][0];
		iMatch_B = oMatch.m_Match[i][1];

		//���ڵ㼯B�ĵ�iMatch_B�㣬��������Neighbour
		oNeighbour = Neighbour_B[iMatch_B];

		//��ŷʽ�����Ϊ���
		Dot_1(oImage_A.m_pDesc[oNeighbour.m_Buffer[0].m_iIndex], oImage_B.m_pDesc[iMatch_B], oNeighbour.m_Buffer[0].m_iDistance);
		Dot_1(oImage_A.m_pDesc[oNeighbour.m_Buffer[1].m_iIndex], oImage_B.m_pDesc[iMatch_B], oNeighbour.m_Buffer[1].m_iDistance);
		//oNeighbour.m_Buffer[0].m_iDistance = iDot(oImage_A.m_pDesc[oNeighbour.m_Buffer[0].m_iIndex], oImage_B.m_pDesc[iMatch_B]);
		//oNeighbour.m_Buffer[1].m_iDistance = iDot(oImage_A.m_pDesc[oNeighbour.m_Buffer[1].m_iIndex], oImage_B.m_pDesc[iMatch_B]);
		//�ٽ�������߷���[0],��֮����[1]
		if (oNeighbour.m_Buffer[0].m_iDistance < oNeighbour.m_Buffer[1].m_iDistance)
			swap(oNeighbour.m_Buffer[0], oNeighbour.m_Buffer[1]);

		if (oNeighbour.m_Buffer[0].m_iIndex != iMatch_A)
			continue;

		best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[0].m_iDistance, 1.0f));
		//�����ж����ŵ��Ƿ������ֵmax_distance,������˾�������ûë��
		if (best_dist_normed > max_distance)
			continue;
		second_best_dist_normed = (float)acos(std::min(kDistNorm * oNeighbour.m_Buffer[1].m_iDistance, 1.0f));
		//���ж����ŵ��Ƿ�ȴ��ŵ��0.8�����󣬱�ʾ���ŵ�����ŵ㲻��̫Զ��
		//����ж���δ���ס����ŵ㲻�������ҵ���
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
	//����ֱ���ң����������ֲ���
	for (i = 0; i < oImage_A.m_iCount; i++)
	{//�˴�Ϊ�Ż��ؼ�
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

	////��ʱ�洢����
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
{//�Ƚ�����ͼ�Ĺؼ��㣬�ҳ�ƥ���
	//return pPoint_1, pPoint_2�� ���ǵ��ú���ֻ���ͷ�pPoint_1����
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

	//����Sift_Image
	Sift_Image oSift_Image, * pSift_Image_Arr = (Sift_Image*)pMalloc(&oMem_Mgr, 2 * sizeof(Sift_Image));
	pSift_Image_Arr[0].m_pFile_Name = (char*)pcFile_1;
	pSift_Image_Arr[1].m_pFile_Name = (char*)pcFile_2;

	for (i = 0; i < 2; i++)
	{
		oSift_Image = pSift_Image_Arr[i];
		//�ȸ�Feature Point����Щ�ռ�
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

	//�����Ժ�OK�ˣ����Ը�ƥ�䣬��Ԥ�ȷ���ռ�
	iSift_Size = iGet_Sift_Match_Size(pSift_Image_Arr, 2);
	pBuffer = (unsigned char*)pMalloc(&oMem_Mgr, iSift_Size);
	if (!pBuffer)
		return;	//�����ڴ棬���ش���
	unsigned char* pStart = pBuffer;
	//����Match, һ�������� iImage_Count*iImage_Count*sizeof(Sift_Match_Item)
	for (i = 0; i < 2; i++)
	{
		pSift_Image_Arr[i].m_pMatch = (Sift_Match_Item*)pBuffer;
		memset(pBuffer, 0, 2 * sizeof(Sift_Match_Item));
		pBuffer += 2 * sizeof(Sift_Match_Item);
	}//�����Ժ��ڴ��Ƶ� pStart + iImage_Count*iImage_Count*sizeof(Sift_Match_Item)

	 //��pBuffer��ǰ�ƶ�����128�ֽ�
	pBuffer = (unsigned char*) (((unsigned long long)pBuffer / 128 + 1) * 128);

	//�ٷ���һ��Desc, ������Ա��ں�������
	for (i = 0; i < 2; i++)
	{
		pSift_Image_Arr[i].m_pDesc = (unsigned char(*)[128])pBuffer;
		pBuffer += pSift_Image_Arr[i].m_iCount * 128;
	}
	Copy_Desc(pSift_Image_Arr, 2, pSift_Image_Arr->m_pDesc[0]);

	Sift_Match_Item oMatch_Item;
	oMatch_Item = { (short)0,(short)1,0,(unsigned short(*)[2])pBuffer };
	if (pBuffer + pSift_Image_Arr[0].m_iCount * sizeof(unsigned short) * 2 - pStart > iSift_Size)
	{//�ж��Ƿ�Խ��
		printf("Insufficient memory\n");
		return;
	}
	iMax_Size = Max(pSift_Image_Arr[0].m_iCount, pSift_Image_Arr[1].m_iCount);
	//���˴�Ҫ����һ���ڴ�����ƥ�����ʱ�ռ�
	Neighbour_K* pNeighbour_B;	//�㼯A�������	
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
{//Ԥ����һ���㹻��Ŀռ䣬װ����ͼ�����������Ϣ
	int iSize,iUpper_Triangle_Size;
	//Image_Info����
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

	//�ļ���
	unsigned char* pCur;
	char* pFile_Name;
	pFile_Name = (char*)poMap->m_pBuffer + iImage_Count * sizeof(Image_Info);
	Get_All_File(File, pFile_Name);

	//�Ȱ�File_Name����poMap
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
{//������ƥ��׶�����
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
	//�ٸ�pSift_Image_Arr����File_Name�ڴ�
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

	//�˴���Sift_Image��Match
	//Disp_Mem(poMem_Mgr, 0);
	iSize = iGet_Sift_Match_Size(pSift_Image_Arr, iImage_Count);
	//Temp code
	iSize *= 2;

	pCur = pStart = (unsigned char*)pMalloc(poMem_Mgr, iSize);
	if (!pCur)
		return;	//�����ڴ棬���ش���

	//����Match, һ�������� iImage_Count*iImage_Count*sizeof(Sift_Match_Item)
	for (i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pMatch = (Sift_Match_Item*)pCur;
		memset(pCur, 0, iImage_Count * sizeof(Sift_Match_Item));
		pCur += iImage_Count * sizeof(Sift_Match_Item);
	}//�����Ժ��ڴ��Ƶ� pStart + iImage_Count*iImage_Count*sizeof(Sift_Match_Item)

	//��pBuffer��ǰ�ƶ�����128�ֽ�
	pCur = (unsigned char*)(((unsigned long long)pCur / 128 + 1) * 128);

	//�ٷ���һ��Desc, ������Ա��ں�������
	iMax_Size = 0;
	for (i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pDesc = (unsigned char(*)[128])pCur;
		pCur += pSift_Image_Arr[i].m_iCount * 128;

		if (pSift_Image_Arr[i].m_iCount > iMax_Size)//Ѱ����󿪱ٿռ�
			iMax_Size = pSift_Image_Arr[i].m_iCount;
	}
	iMax_Size *= sizeof(Neighbour_K);

	Copy_Desc(pSift_Image_Arr, iImage_Count, pSift_Image_Arr->m_pDesc[0]);

	//���˴�Ҫ����һ���ڴ�����ƥ�����ʱ�ռ�
	Neighbour_K* pNeighbour_B;	//�㼯A�������	
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
			{//�ж��Ƿ�Խ��
				printf("Insufficient memory\n");
				return;
			}
			Sift_Match_2_Image_1(pSift_Image_Arr[i], pSift_Image_Arr[j], i, j, &oMatch_Item, pNeighbour_B);
			pCur += ALIGN_SIZE_128(oMatch_Item.m_iMatch_Count * 2 * sizeof(unsigned short));
			pSift_Image_Arr[j].m_pMatch[i] = pSift_Image_Arr[i].m_pMatch[j] = oMatch_Item;
			printf("i:%d j:%d Match_Count:%d \n", i, j, oMatch_Item.m_iMatch_Count);
		}
	}

	//���Feature ���� Match_Map
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
	//�ٽ�Sift_Image_Arrɾ��

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
{//��ĳһ��Ŀ¼����ɨ�裬���е�bmp�ļ�������Sift
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
	//�ٸ�pSift_Image_Arr����File_Name�ڴ�
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

	//�˴���Sift_Image��Match
	//Disp_Mem(poMem_Mgr, 0);
	iSize = iGet_Sift_Match_Size(pSift_Image_Arr, iImage_Count);
	//Temp code
	iSize *= 2;

	pCur = pStart = (unsigned char*)pMalloc(poMem_Mgr, iSize);
	if (!pCur)
		return;	//�����ڴ棬���ش���

	//����Match, һ�������� iImage_Count*iImage_Count*sizeof(Sift_Match_Item)
	for (i = 0; i < iImage_Count; i++)
	{
		pSift_Image_Arr[i].m_pMatch = (Sift_Match_Item*)pCur;
		memset(pCur, 0, iImage_Count * sizeof(Sift_Match_Item));
		pCur += iImage_Count * sizeof(Sift_Match_Item);
	}//�����Ժ��ڴ��Ƶ� pStart + iImage_Count*iImage_Count*sizeof(Sift_Match_Item)

	//�ٷ���һ��Desc, ������Ա��ں�������
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
			{//�ж��Ƿ�Խ��
				printf("Insufficient memory\n");
				return;
			}
			Sift_Match_2_Image(pSift_Image_Arr[i], pSift_Image_Arr[j], i, j, &oMatch_Item);
			pCur += ALIGN_SIZE_128(oMatch_Item.m_iMatch_Count * 2 * sizeof(unsigned short));
			pSift_Image_Arr[j].m_pMatch[i] = pSift_Image_Arr[i].m_pMatch[j] = oMatch_Item;
			printf("i:%d j:%d Match_Count:%d \n", i, j, oMatch_Item.m_iMatch_Count);
		}
	}

	////�ٷ���һ��Desc, ������Ա��ں�������
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
	//		{//�ж��Ƿ�Խ��
	//			printf("Insufficient memory\n");
	//			return;
	//		}
	//		Sift_Match_2_Image_1(pSift_Image_Arr[i], pSift_Image_Arr[j], i, j, &oMatch_Item);
	//		pCur += ALIGN_SIZE_128(oMatch_Item.m_iMatch_Count * 2 * sizeof(unsigned short));
	//		pSift_Image_Arr[j].m_pMatch[i] = pSift_Image_Arr[i].m_pMatch[j] = oMatch_Item;
	//		printf("i:%d j:%d Match_Count:%d \n", i, j, oMatch_Item.m_iMatch_Count);
	//	}
	//}

	//���Feature ���� Match_Map
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
	//�ٽ�Sift_Image_Arrɾ��
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
{//�˴��ٶ�poDest�������
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
{//������ʵ�飬�ʴ˲����ռ��Ż������100������
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

	//��Լ������ȷ����
	Init_Mem_Mgr(&oMem_Mgr, (unsigned long long)(iSize * 1.2 + iImage_Count * MAX_KEY_POINT_COUNT * sizeof(Sift_Feature)), 1024, 997);
	Sift_Match_Path_1(pcPath, &oMatch_Map, &oMem_Mgr);

	Copy_Match_Map(oMatch_Map, poMap);

	Free_Mem_Mgr(&oMem_Mgr);
	return;
}

template<typename _T>void Shrink_Match_Point(_T(**ppPoint_1)[2], _T(**ppPoint_2)[2], unsigned char Mask[], int iCount)
{//��ƥ��ĵ����£������ϵ������ڴ�
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