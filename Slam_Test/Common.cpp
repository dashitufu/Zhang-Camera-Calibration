#include "Common.h"
#ifdef WIN32
	#include "io.h"
	#include "windows.h"
#endif

Mem_Mgr oMatrix_Mem = {};

//unsigned long long iGet_File_Length(char* pcFile)
//{//return: >-0 if success; -1 if fail
//	FILE* pFile = fopen(pcFile, "rb");
//	unsigned long long iLen;
//	if (!pFile)
//		return -1;
//
//#if���f WIN32
//	_fseeki64(pFile, 0, SEEK_END);
//	iLen = _ftelli64(pFile);
//#else
//	fpos64_t oPos;
//	if (fgetpos64(pFile, &oPos) == 0)
//		iLen = oPos.__pos;
//	else
//		iLen = 0;
//#endif
//	fclose(pFile);
//	return iLen;
//}

#ifdef WIN32

void Init_BitPtr(BitPtr* poBitPtr, unsigned char* pBuffer, int iSize)
{
	poBitPtr->m_iBitPtr = poBitPtr->m_iCur = 0;
	poBitPtr->m_iEnd = iSize;
	poBitPtr->m_pBuffer = pBuffer;
}

int iGetBits(BitPtr* poBitPtr, int iLen)
{
	unsigned int iValue, iCur = poBitPtr->m_iCur, iShiftBits;
	int iBitLeft = iLen;
	//�ȸ㶨��һ�ֽ�
	iValue = ((poBitPtr->m_pBuffer[poBitPtr->m_iCur] << poBitPtr->m_iBitPtr) & 0xFF) << 24;
	iBitLeft -= (8 - poBitPtr->m_iBitPtr);
	iCur++;
	iShiftBits = 16 + poBitPtr->m_iBitPtr;
	while (iBitLeft > 0)
	{
		if (iShiftBits > 0)
			iValue |= (poBitPtr->m_pBuffer[iCur] & 0xFF) << iShiftBits;
		iBitLeft -= 8;
		iShiftBits -= 8;
		iCur++;
	}
	iValue = iLen ? iValue >> (32 - iLen) : 0;
	poBitPtr->m_iCur += (poBitPtr->m_iBitPtr + iLen) >> 3;
	poBitPtr->m_iBitPtr = (iLen + poBitPtr->m_iBitPtr) & 7;
	return iValue;
}

void Get_All_File(const char* pcPath,char *pBuffer)
{//ȡһ��Ŀ¼�����ļ� ��: c:\\tmp\\*.bin
	intptr_t handle;
	_finddata_t findData;
	int iCount = 0;
	char* pCur = pBuffer;
	handle = _findfirst(pcPath, &findData);    // ����Ŀ¼�еĵ�һ���ļ�
	if (handle == -1)
	{
		printf("File not found\n");
		return;
	}
	do
	{
		if (findData.attrib & _A_ARCH)
		{
			strcpy(pCur, findData.name);
			pCur += strlen(pCur)+1;
			iCount++;
		}	
	} while (_findnext(handle, &findData) == 0);    // ����Ŀ¼�е���һ���ļ�
	_findclose(handle);    // �ر��������
}
int iGet_File_Count(const char* pcPath)
{//�����ļ�·�������·���µ��ļ�����
	intptr_t handle;
	_finddata_t findData;
	int iCount = 0;
	handle = _findfirst(pcPath, &findData);    // ����Ŀ¼�еĵ�һ���ļ�
	if (handle == -1)
	{
		printf("File not found\n");
		return 0;
	}
	do
	{
		if (findData.attrib & _A_ARCH)
			iCount++;
	} while (_findnext(handle, &findData) == 0);    // ����Ŀ¼�е���һ���ļ�
	_findclose(handle);    // �ر��������
	return iCount;
}
#endif

int bSave_Bin(const char* pcFile, float* pData, int iSize)
{
	FILE* pFile = fopen(pcFile, "wb");
	if (!pFile)
	{
		printf("Fail to save:%s\n", pcFile);
		return 0;
	}
	int iResult = (int)fwrite(pData, 1, iSize, pFile);
	if (iResult != iSize)
	{
		printf("Fail to save:%s\n", pcFile);
		iResult = 0;
	}
	else
		iResult = 1;
	return iResult;
}
int iGet_Upper_Triangle_Size(int w)
{
	int iSize;
	iSize = w * (w - 1) / 2;
	return iSize;
}

int iUpper_Triangle_Cord_2_Index(int x, int y, int w)
{//һ�������Ǿ��󣬸���(x,y)���꣬ת��Ϊ����ֵ, ע�⣬wΪ�����Ǿ��ε�����, ���������ǵ�һ����ЧԪ������
	int iPos;
	//���� w=4, y=2 x=3 Index=7
	if (x <= y || y >= w - 1)
		return -1;
	iPos = ((w - 1) + (w - y)) * y / 2 + x - y - 1;
	return iPos;
}
unsigned long long iGet_File_Length(char* pcFile)
{//return: >-0 if success; -1 if fail
	FILE* pFile = fopen(pcFile, "rb");
	__int64 iLen;
	if (!pFile)
	{
		int iResult = GetLastError();
		printf("Fail to get file length, error:%d\n", iResult);
		return -1;
	}
	_fseeki64(pFile, 0, SEEK_END);
	iLen = _ftelli64(pFile);
	fclose(pFile);
	return iLen;
}

unsigned long long iGet_Tick_Count()
{//��ǰ�ĺ��뼶Tick Count��֮���Բ���΢�뼶����Ϊ����ʱ��Ƭ���⣬��ʹ���뼶Ҳ��׼�����п���Round��16����һ��ʱ��Ƭ����ʤ����
	timeb tp;
	ftime(&tp);
	return (unsigned long long) ((unsigned long long)tp.time * 1000 + tp.millitm);
}

template<typename _T>void Get_Random_Norm_Vec(_T V[], int n)
{//����һ����һ������
	double fTotal=0;
	int i;
	for (i = 0; i < n; i++)
	{
		unsigned int iValue =  iGet_Random_No();
		V[i] = (_T)iValue;
		fTotal += (unsigned long long)iValue * iValue;
	}
	_T fMod = (_T)sqrt((double)fTotal);
	for (i = 0; i < n; i++)
		V[i] /= fMod;
}
unsigned int iGet_Random_No()
{//α���	(789, 123)	(389,621) ��ͨ���������Զ����Լ�������
#define b 389
#define c 621
	static unsigned int iNum = 0xFFFFFFF;	//GetTickCount();	//����
	return iNum = iNum * b + c;
#undef b
#undef c
}
int iRandom(int iStart, int iEnd)
{//��iStart��iEnd֮�������һ�����֣�����RandomInteger̫ɵ�ƣ�û�б�Ҫ��ʱ���˷���ɵ������
	return iStart + iGet_Random_No() % (iEnd - iStart + 1);
}
int iRandom()
{//���Ը�һ���޲������������ĺ���������취Ϊ iRandom_No= iRandom_No*a + c; ���ȡģ 0x7FFFFFFF
#define m 1999999973			//����ģΪ��������ɢ�и����ȵ�����
	static unsigned int a = (unsigned int)iGet_Tick_Count(); //1103515245;		//1103515245Ϊ�ܺõĳ�ʼֵ����ѡȡ��ֵ̬�����ȷ������
	static unsigned int c = 2347;	// 12345;
	static unsigned long long iRandom_No = 1;
	iRandom_No = (iRandom_No * a + c) % m;	//��Ͳ����������ģ
	return (int)iRandom_No;
#undef m
}
template<typename _T>int iQuick_Sort_Partition(_T pBuffer[], int left, int right)
{//С�����˳��
	//Real fRef;
	_T iValue, oTemp;
	int pos = right;

	//��һ�½��м�Ԫ�ؽ�����ͷ����һ����������˼��˲���̬
	oTemp = pBuffer[right];
	pBuffer[right] = pBuffer[(left + right) >> 1];
	pBuffer[(left + right) >> 1] = oTemp;

	right--;
	iValue = pBuffer[pos];
	while (left <= right)
	{
		while (left < pos && pBuffer[left] <= iValue)
			left++;
		while (right >= 0 && pBuffer[right] > iValue)
			right--;
		if (left >= right)
			break;
		oTemp = pBuffer[left];
		pBuffer[left] = pBuffer[right];
		pBuffer[right] = oTemp;
	}

	oTemp = pBuffer[left];
	pBuffer[left] = pBuffer[pos];
	pBuffer[pos] = oTemp;

	return left;
}
template<typename _T>int iAdjust_Left(_T* pStart, _T* pEnd)
{
	_T oTemp, * pCur_Left = pEnd - 1,
		* pCur_Right;
	_T oRef = *pEnd;

	//Ϊ�˼���һ���жϣ��˴���ɨ��ȥ
	while (pCur_Left >= pStart && *pCur_Left == oRef)
		pCur_Left--;
	pCur_Right = pCur_Left;
	pCur_Left--;

	while (pCur_Left >= pStart)
	{
		if (*pCur_Left == oRef)
		{
			oTemp = *pCur_Left;
			*pCur_Left = *pCur_Right;
			*pCur_Right = oTemp;
			pCur_Right--;
		}
		pCur_Left--;
	}
	return (int)(pCur_Right - pStart);
}

template<typename _T> _T oGet_Nth_Elem(_T Seq[],int iCount, int iStart, int iEnd, int iNth,int bAdjust_Pos=0)
{//ȡ��n��Ԫ�أ���Quick_Sort��Partition��
	int iPos;
	if (iStart < iEnd)
	{
		iPos = iQuick_Sort_Partition(Seq, iStart, iEnd);
		if (iPos > iNth)//��ô��n��Ԫ�����������
			return oGet_Nth_Elem(Seq,iCount, iStart, iPos - 1, iNth);
		else if (iPos < iNth)//��n��Ԫ�����ҷ�����
			return oGet_Nth_Elem(Seq,iCount, iPos + 1, iEnd, iNth);
		else //�ҵ��ˣ�������iPos��
			return Seq[iPos];
	}else
	{//��ʱ�ַ���ż�������
		if(iCount&1 || !bAdjust_Pos)
			return Seq[iStart];	//�����ð죬���ر���
		else //ż���Ļ�Ҫ��ǰ�����ֵ
		{
			_T fMax = Seq[iStart - 1];
			for (int i = iStart - 2; i >= 0; i--)
				if (Seq[i] > fMax)
					fMax = Seq[i];
			return (_T)((Seq[iStart] + fMax) / 2.f);
		}
	}	
}

template<typename _T> _T oGet_Nth_Elem(_T Seq[], int iCount, int iNth)
{
	return oGet_Nth_Elem(Seq, iCount, 0, iCount - 1, iNth);
}
template<typename _T> void Quick_Sort(_T Seq[], int iStart, int iEnd)
{
	int iPos, iLeft;
	if (iStart < iEnd)
	{
		iPos = iQuick_Sort_Partition(Seq, iStart, iEnd);
		//���iPos>=iEnd, �����ϼ�����̬�����ˣ���ʱ��һ���ƽ����ҳ�������Seq[iEnd]��ȵ���Ŀ���ϵ��ұ�
		if (iPos >= iEnd)
		{
			iLeft = iAdjust_Left(Seq + iStart, Seq + iPos);
			iLeft = iStart + iLeft;
		}
		else
			iLeft = iPos - 1;

		if (iStart < iLeft)
			Quick_Sort(Seq, iStart, iLeft);

		Quick_Sort(Seq, iPos + 1, iEnd);
	}
}
void SB_Common()
{//templateʵ������ֻ��vc��Ч
	Temp_Load_File_2(NULL, NULL, NULL, (Point_2D<double>**)NULL,(double(**)[3])NULL,(double(**)[9])NULL);
	Temp_Load_File_2(NULL, NULL, NULL, (Point_2D<float>**)NULL, (float(**)[3])NULL, (float(**)[9])NULL);

	bSave_PLY(NULL, (double(*)[3])NULL, 0);
	bSave_PLY(NULL, (float(*)[3])NULL, 0);

	oGet_Nth_Elem((double*)NULL, 0, 0);
	oGet_Nth_Elem((float*)NULL, 0, 0);
	oGet_Nth_Elem((int*)NULL, 0, 0);

	Quick_Sort((double*)NULL, 0, 0);
	Quick_Sort((float*)NULL, 0, 0);
	Quick_Sort((int*)NULL, 0, 0);

	Get_Random_Norm_Vec((double*)NULL, 0);
	Get_Random_Norm_Vec((float*)NULL, 0);
}
int bLoad_Text_File(const char* pcFile, char** ppBuffer, int* piSize)
{
	FILE* pFile = fopen(pcFile, "rb");
	int bRet = 0, iResult, iSize;
	char* pBuffer;
	iSize = (int)iGet_File_Length((char*)pcFile);
	pBuffer = (char*)malloc(iSize+1);

	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		goto END;
	}
	if (!pBuffer)
	{
		printf("Fail to allocate memory\n");
		goto END;
	}

	iResult = (int)fread(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		if (pBuffer)
			free(pBuffer);
		*ppBuffer = NULL;
		printf("Fail to read data\n");
		goto END;
	}
	pBuffer[iSize] = '\0';
	*ppBuffer = pBuffer;
	if (piSize)
		*piSize = iSize;
	bRet = 1;
END:
	if (pFile)
		fclose(pFile);
	if (!bRet)
	{
		if (pBuffer)
			free(pBuffer);
	}
	return bRet;
}
int bSave_Raw_Data(const char* pcFile, unsigned char* pBuffer, int iSize)
{
	FILE* pFile = fopen(pcFile, "wb");
	if (!pFile)
	{
		printf("Fail to save file:%s\n", pcFile);
		return 0;
	}
	int iResult=(int)fwrite(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		printf("Fail to save file:%s %d\n", pcFile,GetLastError());
		fclose(pFile);
		return 0;
	}
	fclose(pFile);
	return 1;
}
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer, int iSize, int bNeed_Malloc, int iFrame_No)
{
	FILE* pFile = fopen(pcFile, "rb");
	unsigned long long iPos;
	int bRet = 0, iResult;
	iPos = (unsigned long long)iSize * iFrame_No;
	unsigned char* pBuffer;

	if (!iSize)
		iSize = (int)iGet_File_Length((char*)pcFile);

	if (bNeed_Malloc)
		pBuffer = (unsigned char*)malloc(iSize);
	else
		pBuffer = *ppBuffer;

	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		goto END;
	}
	if (!pBuffer)
	{
		printf("Fail to allocate memory\n");
		goto END;
	}
#ifdef WIN32
	_fseeki64(pFile, iPos, SEEK_SET);
#else
	fseeko64(pFile, (unsigned long long)iPos, SEEK_SET);
#endif

	iResult = (int)fread(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		if (pBuffer)
			free(pBuffer);
		*ppBuffer = NULL;
		printf("Fail to read data\n");
		bRet = 0;
		goto END;
	}
	*ppBuffer = pBuffer;
	bRet = 1;
END:
	if (pFile)
		fclose(pFile);
	if (!bRet)
	{
		if (pBuffer && bNeed_Malloc)
			free(pBuffer);
	}
	return bRet;
}
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer,int *piSize)
{
	FILE* pFile = fopen(pcFile, "rb");
	int bRet = 0, iResult,iSize;
	unsigned char* pBuffer;
	iSize = (int)iGet_File_Length((char*)pcFile);
	pBuffer = (unsigned char*)malloc(iSize);

	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		goto END;
	}
	if (!pBuffer)
	{
		printf("Fail to allocate memory\n");
		goto END;
	}

	iResult = (int)fread(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		if (pBuffer)
			free(pBuffer);
		*ppBuffer = NULL;
		printf("Fail to read data\n");
		goto END;
	}
	*ppBuffer = pBuffer;
	if (piSize)
		*piSize = iSize;
	bRet = 1;
END:
	if (pFile)
		fclose(pFile);
	if (!bRet)
	{
		if (pBuffer)
			free(pBuffer);
	}
	return bRet;
}

template<typename _T>int bSave_PLY(const char* pcFile, _T Point[][3], int iPoint_Count, unsigned char Color[][3], int bText)
{//����ƣ������ʽ������ʵ�飬���ṹ����Ҫ
	FILE* pFile = fopen(pcFile, "wb");
	char Header[512];
	int i;
	_T* pPos;

	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		return 0;
	}
	if (!iPoint_Count)
	{
		printf("No point to save\n");
		return 0;
	}

	//��д��Header
	sprintf(Header, "ply\r\n");
	if (bText)
		sprintf(Header + strlen(Header), "format ascii 1.0\r\n");
	else
		sprintf(Header + strlen(Header), "format binary_little_endian 1.0\r\n");
	sprintf(Header + strlen(Header), "comment HQYT generated\r\n");
	sprintf(Header + strlen(Header), "element vertex %d\r\n", iPoint_Count);
	sprintf(Header + strlen(Header), "property float x\r\n");
	sprintf(Header + strlen(Header), "property float y\r\n");
	sprintf(Header + strlen(Header), "property float z\r\n");

	if (Color)
	{
		sprintf(Header + strlen(Header), "property uchar red\r\n");
		sprintf(Header + strlen(Header), "property uchar green\r\n");
		sprintf(Header + strlen(Header), "property uchar blue\r\n");
	}

	sprintf(Header + strlen(Header), "end_header\r\n");
	fwrite(Header, 1, strlen(Header), pFile);

	for (i = 0; i < iPoint_Count; i++)
	{
		pPos = Point[i];
		if (bText)
		{
			fprintf(pFile, "%f %f %f ", pPos[0], pPos[1], pPos[2]);
			if (pPos[0] >= 1000000.f || isnan(pPos[0]))
				printf("here");
			if (Color)
				fprintf(pFile, "%d %d %d\r\n", Color[i][0], Color[i][1], Color[i][2]);
			else
				fprintf(pFile, "\r\n");
		}
	}
	fclose(pFile);
	return 1;
}

template<typename _T>void Temp_Load_File_2(int* piCameta_Count, int* piPoint_Count, int* piObservation_Count, Point_2D<_T>** ppPoint_2D, _T(**ppPoint_3D)[3], _T(**ppCamera)[3 * 3])
{//���װ��problem-16-22106-pre.txt������õ�����ݽṹָ��ƥ���ϵ

	int i, iCamera_Count, iPoint_Count, iObservation_Count;
	_T(*pPoint_3D)[3], (*pCamera)[3 * 3];
	Point_2D<_T>* pPoint_2D;
	char* pcFile = (char*)"c:\\tmp\\temp\\problem-16-22106-pre.txt";	//"Sample\\problem-16-22106-pre.txt";
	FILE* pFile = fopen(pcFile, "rb");
	i = fscanf(pFile, "%d %d %d\n", &iCamera_Count, &iPoint_Count, &iObservation_Count);
	pPoint_2D = (Point_2D<_T>*)malloc(iObservation_Count * 2 * sizeof(Point_2D<_T>));
	pPoint_3D = (_T(*)[3])malloc(iPoint_Count * 3 * sizeof(_T));
	pCamera = (_T(*)[3 * 3])malloc(iCamera_Count * 16 * sizeof(_T));

	for (i = 0; i < iObservation_Count; i++)
	{
		float xy[2];
		fscanf(pFile, "%d %d %f %f", &pPoint_2D[i].m_iCamera_Index, &pPoint_2D[i].m_iPoint_Index, &xy[0], &xy[1]);
		pPoint_2D[i].m_Pos[0] = xy[0], pPoint_2D[i].m_Pos[1] = xy[1];
		//fscanf(pFile, "%d %d %f %f", &pPoint_2D[i].m_iCamera_Index, &pPoint_2D[i].m_iPoint_Index, &pPoint_2D[i].m_Pos[0], &pPoint_2D[i].m_Pos[1]);
	}

	int iResult;
	for (i = 0; i < iCamera_Count; i++)
		for (int j = 0; j < 9; j++)
			if(sizeof(_T)==4)
				iResult=fscanf(pFile, "%f", (float*)&pCamera[i][j]);
			else
				iResult=fscanf(pFile, "%lf", (double*)&pCamera[i][j]);
	for (i = 0; i < iPoint_Count; i++)
		if(sizeof(_T)==4)
			fscanf(pFile, "%f %f %f ",(float*)&pPoint_3D[i][0], (float*)&pPoint_3D[i][1], (float*)&pPoint_3D[i][2]);
		else
			fscanf(pFile, "%lf %lf %lf ", (double*)&pPoint_3D[i][0], (double*)&pPoint_3D[i][1], (double*)&pPoint_3D[i][2]);
	fclose(pFile);
	*piCameta_Count = iCamera_Count;
	*piPoint_Count = iPoint_Count;
	*piObservation_Count = iObservation_Count;
	*ppPoint_2D = pPoint_2D;
	*ppPoint_3D = pPoint_3D;
	*ppCamera = pCamera;
	//Disp((_T*)pCamera, 16, 9, "Camera");
	return;
}

void* pMalloc(Light_Ptr* poPtr, int iSize)
{//���������ڴ����
	unsigned char* pBuffer;
	Malloc(*poPtr, iSize, pBuffer);
	return (void*)pBuffer;
}
void *pMalloc(unsigned int iSize)
{//ԭ����pMalloc���˸�����̫�鷳���ɴ��һ�����
	return pMalloc(&oMatrix_Mem, iSize);
}
void Free(void* p)
{
	Free(&oMatrix_Mem, p);
}
void Shrink(void* p, unsigned int iSize)
{
	Shrink(&oMatrix_Mem, p, iSize);
	return;
}
void Disp_Mem()
{
	Disp_Mem(&oMatrix_Mem, 0);
}
