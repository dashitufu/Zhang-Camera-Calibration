#pragma once
#include "sift.h"
#include "Matrix.h"
#include "Image.h"
#include "Chess_Board_Detect.h"

typedef struct Plane {	//��ʾһ��ƽ�棿ûŪ��
	float a, b, d;
}Plane;

//������һ��������������ɾ����Ľṹ
template<typename _T> struct Point_Cloud {
	_T(*m_pPoint)[3];				//��¼���λ��(x,y,z)
	unsigned char (*m_pColor)[3];	//RGB
	unsigned char *m_pBuffer;		//�ڴ�����
	unsigned char m_bHas_Color : 1;	//��־�Ƿ�����ɫ
	int m_iMax_Count;				//m_pBuffer������ɶ��ٵ�
	int m_iCount;					//Ŀǰ�ж��ٵ�
};

typedef struct Recon_Image {	//�ֵ��ؽ��׶�һ��ͼ����Ϣ
	void* m_poFeature_Image;	//��ͼ��Ӧ��Featureƥ��ͼ���
	int m_iCorrespondence;		//������ͼƥ���֮��
	int m_iObservation;			//�۲��������
}Recon_Image;

typedef struct Ransac_Support {
	int m_iInlier_Count;			//����ô����ͷ���Եĸ��ָ�ľ����ж��ٵ�������
	float m_fResidual_Sum;			//���еľ���������
}Ransac_Support;

typedef struct Ransac_Report {	//�ýṹ��ʾRansac�������ʱֻ����Ransac����
	int m_bSuccess;
	int m_iSample_Count;
	int m_iTrial_Count;	//���Դ���
	int m_iFloat_Size;	//4�ֽڻ���8�ֽ�
	union {		//����������Modal
		double m_Modal_d[3 * 3];
		float m_Modal_f[3 * 3];
		unsigned char m_Modal[3 * 3 * 8];
	};
	//float s;	//ֻ����H����ʱ��Ч��p2 = sHp1

	Ransac_Support m_oSupport;
	unsigned char* m_pInlier_Mask;	//����Residual���ڷ�Χ����Ϊ1
}Ransac_Report;

typedef struct Two_View_Geometry {
	enum Config_Type {	//��־�����ճ�
		UNDEFINED = 0,
		// Degenerate configuration (e.g., no overlap or not enough inliers).
		DEGENERATE = 1,
		// Essential matrix.
		CALIBRATED = 2,
		// Fundamental matrix.
		UNCALIBRATED = 3,
		// Homography, planar scene with baseline.
		PLANAR = 4,
		// Homography, pure rotation without baseline.
		PANORAMIC = 5,
		// Homography, planar or panoramic.
		PLANAR_OR_PANORAMIC = 6,
		// Watermark, pure 2D translation in image borders.
		WATERMARK = 7,
		// Multi-model configuration, i.e. the inlier matches result from multiple
		// individual, non-degenerate configurations.
		MULTIPLE = 8,
	};
	Ransac_Report m_oReport_E, m_oReport_F, m_oReport_H;	//����������ڹ���
	Config_Type m_iConfig;									//���������ĸ�������
	int num_inliers;
}Two_View_Geometry;

template<typename _T> struct Schur_Camera_Data {	//ר����Slam��˹����е����ݼ�¼
	typedef struct One_Camera_Data {    //һ����������Ӧ�������
		typedef struct Point_Data {
			int m_iPoint_Index;
			_T m_Data_6x3[6 * 3];
			//_T Data_3x3[3 * 3];
		}Point_Data;
		_T m_Data_6x6[6 * 6];    //
		Point_Data* m_pPoint_Data;
		int m_iCur = 0; //��ǰ���õ�λ�ã���һ������λ��
		int m_iPoint_Count;
	}One_Camera_Data;
	_T(*m_pData_3x3)[3 * 3]; //3x3���������ۼ�������
	union {
		One_Camera_Data* m_pCamera_Data;
		unsigned char* m_pBuffer;
	};
	int m_iCamera_Count;
	int m_iPoint_Count;
	int m_iObservation_Count;
};
template<typename _T> struct Measurement {
	int m_Camera_Index[2];    //�۲��е�����λ��
	_T Delta_ksi[7];        //���ksi��ǰ�����൱��λ�ƣ���4��Ϊ4Ԫ��
};
template<typename _T> struct Pose_Graph_Sigma_H {
	typedef struct Camera_Data {
		_T m_Data_6x6[6 * 6];
		int m_iIndex;   //�����ݿ���������е�����
	}Camera_Data;
	typedef struct Camera_Line {
		Camera_Data* m_pCamera_Data;     //һ�������Sigma_Hռһ�У�����һ�������ڿ��еĿ�ʼλ��
		int m_iCount;   //ÿ����������������������ϵ�����ڷ��Է���ϵ,��<i,j>�Ͳ���Ҫ<j,i>
	}Camera_Line;

	Camera_Line* m_pLine;
	union {
		Camera_Data* m_pCamera_Data;
		unsigned char* m_pBuffer;
	};
	int m_iCamera_Data_Count;   //��Meaurement Count�� �������
	int m_iCamera_Count;        //һ���ж��ٸ����
};

template<typename _T>struct Image_Match_Param {
	_T(*m_pImage_Point_0)[2];
	_T(*m_pImage_Point_1)[2];	//����ƽ����

	union {
		_T(*m_pPoint_3D_0)[3];
		_T(*m_pPoint_3D)[3];
	};
	_T(*m_pPoint_3D_1)[3];
	_T(*m_pNorm_Point_0)[2];
	_T(*m_pNorm_Point_1)[2];	//��һ��ƽ����
	int m_iMatch_Count;
	_T K[3 * 3],				//�ڲ�
		T[4 * 4];				//��Σ�Image_0�����Image_1��λ��
	Ransac_Report m_oReport;
	Image m_oImage_0, m_oImage_1, m_oImage_2;
};

template<typename _T> struct LM_Param_g2o {
	unsigned short m_iIter;	//�ڼ��ֵĵ���
	unsigned char m_bStop;	//�ⲿֹͣ�ź�
	_T m_fLoss;			//�������
	_T Lamda;			//�ؼ�����
	_T _ni;
	_T* m_pOrg_X;			//ԭ���Ĳ���

	//ceres����
	_T radius_;
	_T accumulated_reference_model_cost_change_;
	_T decrease_factor_;

	void(*Get_J)(_T*, _T*, _T*, _T*, _T*, ...);
	_T* m_pSample_In;
	_T* Y;
	int m_iSample_Count;
	unsigned char m_iIn_Dim, m_iOut_Dim;
};

//template<typename _T> struct Point_2D_N_Cam {    //�������ID��2D��
//	_T m_Pos[2];    //λ��
//	unsigned short m_iPoint_Index;
//	unsigned short m_iCamera_Index;    
//};

//����E,F,H��������Ĺ��ƣ���һ��Report���˴�Ҫ���ڴ����
void Free_Report(Ransac_Report oReport, Mem_Mgr* poMem_Mgr = NULL);
void Disp_Report(Ransac_Report oReport);
void Disp_Ransac_Report(Ransac_Report oReport);

template<typename _T>void Normalize_Point(_T(*pPoint_1)[2], _T(*pPoint_2)[2], int iCount, _T(*pNorm_Point_1)[2], _T(*pNorm_Point_2)[2], float f, float c1, float c2);

template<typename _T> void Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T(*Norm_Point_1)[2] = NULL, _T(*Norm_Point_2)[2] = NULL, Light_Ptr oPtr = { 0 });
template<typename _T> void Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T(*Norm_Point_1)[2] = NULL, _T(*Norm_Point_2)[2] = NULL, Light_Ptr oPtr = { 0 });
template<typename _T> void Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T H[3 * 3], _T(*Norm_Point_1)[2] = NULL, _T(*Norm_Point_2)[2] = NULL, Light_Ptr oPtr = { 0 });

template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr = NULL, _T fMax_Residual = 4 * 4);
template<typename _T> void Ransac_Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, float f, float c1, float c2, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr = NULL);
template<typename _T> void Ransac_Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr = NULL);

template<typename _T>void Decompose_E(_T E[3 * 3], _T R1[3 * 3], _T R2[3 * 3], _T t1[3], _T t2[3], int bNormalize_t = 1);
template<typename _T>void Decompose_H(_T H[3 * 3], _T R1[3 * 3], _T R2[3 * 3], _T t1[3], _T t2[3], _T K1[], _T K2[]=NULL);

template<typename _T>void Check_Cheirality(_T Point_1[][2], _T Point_2[][2], int* piCount, _T R[], _T t[], _T Point_3d[][3], unsigned char* pInlier_Mask = NULL);
template<typename _T>void E_2_R_t(_T E[3 * 3], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3]=NULL, int* piCount=NULL, unsigned char *pInlier_Mask=NULL);
template<typename _T>void E_2_R_t_Pixel_Pos(_T E[3 * 3], _T Point_1[][2], _T Point_2[][2], _T K[], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3]);

template<typename _T> void Compute_Squared_Sampson_Error(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T Residual[]);

template<typename _T>void Triangulate_Point(_T Point_1[2], _T Point_2[2], _T P_1[], _T P_2[], _T New_Point[]);

template<typename _T>void Gen_Camera_Intrinsic(_T K[3 * 3], float fFocal, float a, float b, float cx, float cy);

template<typename _T>void Test_E(_T E[], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount);	//����һ��E

//���������ӿڲ��ã��Ժ�����������������ýӿ�
template<typename _T> void Estimate_Relative_Pose(Two_View_Geometry oGeo, float Camera_1[3], float Camera_2[2], _T Point_1[][2], _T Point_2[][2], int iCount, Mem_Mgr* poMem_Mgr);
template<typename _T> void Determine_Confg(Two_View_Geometry* poGeo, _T Point_1[][2], _T Point_2[][2], int iCount, _T(**ppNew_Point_1)[2], _T(**ppNew_Point_2)[2], Mem_Mgr* poMem_Mgr = NULL);

//��RGBDͼ�ָ��ռ��
template<typename _T> void RGBD_2_Point_3D(Image oImage, unsigned short* pDepth, _T K[][3], _T fDepth_Factor, _T Point_3D[][3], int* piPoint_Count, unsigned char Color[][3] = 0);
template<typename _T>void Image_Pos_2_3D(_T Image_Pos[][3], int iCount, _T K[], _T fDepth_Factor, _T Pos_3D[][3]);
template<typename _T>void Image_Pos_2_3D(_T Image_Pos[][2], unsigned short Depth[], int iCount, int iWidth, _T K[], _T fDepth_Factor, _T Pos_3D[][3]);

////����Ļ���ת��Ϊ��һ��ƽ������
template<typename _T>void Image_Pos_2_Norm(_T Image_Pos[][2], int iCount, _T K[], _T(*pNorm_Point)[2]);

//Bundle_Adjust�����ǹ��ƵĹؼ��������кܶ��ֱ���
template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_1[][3], _T Point_2D_2[][2], int iCount, _T K[], _T Pose[], int* piResult);
template<typename _T>void ICP_BA_2_Image_1(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[], int* piResult);
template<typename _T>void ICP_BA_2_Image_2(_T P1[][3], _T P2[][3], int iCount, _T Pose[], int* piResult);
template<typename _T>void ICP_SVD(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[], int* piResult);

//�˴�����Ҫ����һ����ſɱ�
template<typename _T>void Get_Deriv_E_P(_T K[3 * 3], _T Camera[4 * 4], _T TP[3], _T J[2 * 6]);
template<typename _T>void Get_Deriv_E_Ksi(_T K[3 * 3], _T TP[3], _T J[2 * 6]);
template<typename _T>void Get_Deriv_TP_Ksi_1(_T T[4 * 4], _T P[3], _T Deriv[4 * 6]);
template<typename _T>void Get_Deriv_TP_Ksi(_T T[4 * 4], _T TP[3], _T Deriv[4 * 6]);
template<typename _T>void Get_Drive_UV_P(_T fx, _T fy, _T P[3], _T J[2 * 3]);
template<typename _T>void Get_Drive_UV_P(_T K[3 * 3], _T P[3], _T J[2 * 3]);

template<typename _T>void Get_Delta_Pose(_T Pose_1[4 * 4], _T Pose_2[4 * 4], _T Delta_Pose[4 * 4]);
template<typename _T>void Get_T_Inv(_T T[4 * 4], _T T_Inv[4 * 4]);//��һ��λ�˽��п�������

//һ��ר���ڴ��������ϵ�ĺ���
template<typename _T>void Init_All_Camera_Data(Schur_Camera_Data<_T>* poData, int Point_Count_Each_Camera[], int iCamera_Count, int iPoint_Count);
template<typename _T>void Free(Schur_Camera_Data<_T>* poData);
template<typename _T>void Distribute_Data(Schur_Camera_Data<_T> oData, _T JJt[9 * 9], int iCamera_ID, int iPoint_ID);
template<typename _T>void Copy_Data_2_Sparse(Schur_Camera_Data<_T> oData, Sparse_Matrix<_T>* poA);

//����һ�麯��������λ��ͼ
template<typename _T>void TQ_2_Rt(_T TQ[7], _T Rt[4 * 4]);	//7���ɶ�se3ת��SE3�ϵ�Rt����
template<typename _T>void Init_Pose_Graph(Measurement<_T>* pMeasurement, int iMeasurement_Count, int iCamera_Count, Pose_Graph_Sigma_H<_T>* poPose_Graph);
template<typename _T>void Reset_Pose_Graph(Pose_Graph_Sigma_H<_T> oSigma_H);
template<typename _T>void Get_J_Inv(_T eij[6], _T J_Inv[6 * 6]);
template<typename _T>void Get_Adj(_T Rt[4 * 4], _T Adj[6 * 6]);
template<typename _T>void Distribute_Data(Pose_Graph_Sigma_H<_T> oSigma_H, _T JJt[12 * 12], int iCamera_i, int iCamera_j);
template<typename _T>void Copy_Data_2_Sparse(Pose_Graph_Sigma_H<_T> oPose_Graph, Sparse_Matrix<_T>* poA);
template<typename _T>void Undistort_uv(_T TP[3], _T K[3 * 3], _T Dist_Coeff[], int iCoeff_Count, _T uv[2]);

//Schur��Ԫ����Slam���Է���
template<typename _T>void Solve_Linear_Schur(Schur_Camera_Data<_T> oData, _T Sigma_JE[], _T X[], int* pbSuccess = NULL);

template<typename _T> void Optical_Flow_1(Image oImage_1, Image oImage_2, _T KP_1[][2], _T KP_2[][2], int iCount, int* piMatch_Count, int bHas_Initial = 0, int bInverse = false);

template<typename _T>void Disp_Error(_T P1[][3], _T P2[][3], int iCount, _T Pose[4 * 4]);
template<typename _T>void Disp_T(_T T[4 * 4]);

template<typename _T> void Gen_Cube(_T Cube[][4], float fScale, _T x_Center = 0, _T y_Center = 0, _T z_Center = 0);//����һ��Cube
template<typename _T>void Gen_Cube(_T(**ppCube)[3], int* piCount, float fScale, _T x_Center = 0, _T y_Center = 0, _T z_Center = 0);

template<typename _T>void Gen_Sphere(_T(**ppPoint_3D)[3], int* piCount, _T r = 1.f, int iStep_Count = 40);
template<typename _T>void Gen_Plane(_T(**ppPoint)[3], int* piCount, _T K[3 * 3] = NULL, _T T[4 * 4] = NULL);
template<typename _T>void Gen_Plane_z0(_T(**ppPoint_3D)[3], int* piCount);
template<typename _T>void Get_K_Inv(_T K[3 * 3], _T K_Inv[3 * 3]);	//���ٵ��ڲ������
template<typename _T>void Get_K_Inv_With_gama(_T K[], _T K_Inv[]);

template<typename _T>void Gen_Pose(_T T[4 * 4], _T v0, _T v1, _T v2, _T theta, _T t0, _T t1, _T t2, int* piResult = NULL);
//Temp Code
template<typename _T> int bTemp_Load_Data(const char* pcFile, _T(**ppT)[7], int* piPoint_Count,
	Measurement<_T>** ppMeasurement, int* piMeasure_Count);
template<typename _T>void Match_2_Image(const char* pcFile_0, const char* pDepth_File_0,
	const char* pcFile_1, const char* pDepth_File_1,
	_T fDepth_Factor, //�����������
	_T K[],    //�ڲΣ�������NULL
	int* piCount,  //�������
	_T(**ppImage_Point_0)[2], _T(**ppImage_Point_1)[2],   //����ƽ����
	_T(**ppNorm_Point_0)[2], _T(**ppNorm_Point_1)[2],     //��һ��ƽ����
	_T(**ppPoint_3D_0)[3], _T(**ppPoint_3D_1)[3],          //�ռ���
	Image* poImage_0, Image* poImage_1, Image* poImage_2);
template<typename _T>void Estimate_2_Image_T(const char* pcFile_0, const char* pcFile_1, _T K[3 * 3], Image_Match_Param<_T>* poParam);

template<typename _T>void Free_2_Image_Match(Image_Match_Param<_T>* poParam);

template<typename _T>void Test_T(_T Point_3D[][3], _T Norm_Point_0[][2], _T T0[4 * 4], int iCount);
template<typename _T>_T Test_T(_T Point_3D_0[][3], _T Point_3D_1[][3], _T T[], int iCount);
template<typename _T>void DLT_svd(_T Point_3D_0[][3], _T Point_3D_1[][3], _T Norm_Point_1[][2], int iCount, _T T[]);
template<typename _T>void DLT(_T Point_3D_0[][3], _T Point_3D_1[][3], int iCount, _T T[]);
template<typename _T>_T fGet_Error(_T A[], int m, int n, _T X[]);
template<typename _T>void Test_Triangulate(_T Point_3D[][3], _T Norm_Point_0[][2], _T Norm_Point_1[][2], int iCount, _T T[4 * 4], _T* pfError=NULL);

//���ڵ��Ƽ򵥻�ͼ
template<typename _T>void bSave_PLY(const char* pcFile, Point_Cloud<_T> oPC);
template<typename _T>void Init_Point_Cloud(Point_Cloud<_T>* poPC, int iMax_Count, int bHas_Color);
template<typename _T>void Free_Point_Cloud(Point_Cloud<_T>* poPC);
template<typename _T>void Draw_Point(Point_Cloud<_T>* poPC, _T x, _T y, _T z, int R = 255, int G = 255, int B = 255);
template<typename _T>void Draw_Sphere(Point_Cloud<_T>* poPC, _T x, _T y, _T z, _T r = 1.f, int iStep_Count = 40, int R = 255, int G = 255, int B = 255);
template<typename _T>void Draw_Line(Point_Cloud<_T>* poPC, _T x0, _T y0, _T z0, _T x1, _T y1, _T z1, int iCount = 50, int R = 255, int G = 255, int B = 255);
template<typename _T>void Draw_Camera(Point_Cloud<_T>* poPC, _T T[4 * 4], int R = 255, int G = 255, int B = 255);
template<typename _T>void Draw_Image(Point_Cloud<_T>* poPC, Image oImage, _T T[4 * 4], _T K[3 * 3], int iCount);
template<typename _T>void bSave_PLY(const char* pcFile, Point_Cloud<_T> oPC);


//����PnP
template<typename _T>void PnP_Pose_Point(_T Pose[][4 * 4], int iCamera_Count, _T Intrinsic[4], _T Point[][3], int iPoint_3D_Count, Point_2D<_T> Observation[], int iObservation_Count);


//ͳһ��BA�ӿڣ��Ѿ�ʧ��
template<typename _T>void Bundle_Adjust(_T Sample_In[], const int iIn_Dim, _T Y[], const int iOut_Dim, int iSample_Count, _T X[], int iParam_Count, _T eps,int iMax_Iter_Count,
	void(*Get_J)(_T*, _T*, _T*, _T*, _T*,...),...);
template<typename _T>void Init_LM_Param(LM_Param_g2o<_T>* poParam);
template<typename _T>void Solve_Linear_LM(_T A[], int iOrder, _T b[], _T x[], int* pbResult, LM_Param_g2o<_T>* poParam);


//��ʱ����
template<typename _T>int bTemp_Load_Data(char* pcFile, _T(**ppCamera)[4 * 4], int* piCamera_Count, _T** ppDepth);;
template<typename _T>_T NCC(Image oRef, Image oCur, int x1, int y1, _T x2, _T y2);
