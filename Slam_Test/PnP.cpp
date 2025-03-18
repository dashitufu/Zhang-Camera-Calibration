//���㵥��һ���ļ��������е�pnp�Ż�����ȫ��������
//����һ���滮����������������
//PnP_Pose_Point			//ֻ��λ����ռ���Ż�
//PnP_Pose_Point_f_cx_cy	//������ڲ�һ���Ż���ȫ���������һ���ڲ�
//PnP_Pose_Point_fx_fy_cx,cy		//������Ļ����Ͻ�f�ֳ�fx,fy
//PnP_Pose_Point_f_cx_cy_k1_k2		//������������Ż�
//PnP_Pose_Point_fx_fy_cx_cy_k1_k2	//�����ڲε��Ż�

#include "Reconstruct.h"

template<typename _T> struct LM_Param_Ceres {//�����ӵ�LM������Ҫ������ݿ��ܸ���
	unsigned char reuse_diagonal;	//��ʼ��Ϊʲô������
	unsigned char bIsStepSuccessful;	//��һ���Ƿ�����ɹ�
	unsigned char bStep_is_valid;
	unsigned short m_iIter;
	unsigned short num_consecutive_nonmonotonic_steps;
	_T radius;
	_T current_cost, reference_cost, candidate_cost, minimum_cost;

	_T decrease_factor;
	_T* m_pDiag;	//�Խ���Ԫ��
	_T accumulated_reference_model_cost_change;
	_T accumulated_candidate_model_cost_change;

	union {
		_T* J;
		_T (*J_9)[2][9];
	};
	
	_T (*Residual)[2];

	_T* H, * JtE;	//H����
};
void SB_PnP()
{//ʵ����ģ����ѣ�����Ӫ��
	PnP_Pose_Point((double(*)[4 * 4])NULL, 0, (double*)NULL, (double(*)[3])NULL, 0, (Point_2D<double>*)NULL, 0);
	PnP_Pose_Point((float(*)[4 * 4])NULL, 0, (float*)NULL, (float(*)[3])NULL, 0, (Point_2D<float>*)NULL, 0);
}
template<typename _T>void Sort_Observation(Point_2D<_T> Observation[], int iCamera_Count, int iObservation_Count)
{//����Ͱ��������
	int i;
	union {
		int * pPoint_Per_Cam ;
		int* pStart;
	};
	/*for (i = 0; i < iObservation_Count; i++)
	{
		Point_2D<_T>oPoint = Observation[i];
		printf("Camera Index:%d Point_Index:%d Pos:%f %f\n", oPoint.m_iCamera_Index, oPoint.m_iPoint_Index, oPoint.m_Pos[0], oPoint.m_Pos[1]);
	}*/
	pPoint_Per_Cam = (int*)pMalloc(iCamera_Count * sizeof(_T));
	memset(pPoint_Per_Cam, 0, iCamera_Count * sizeof(_T));
	for (i = 0; i < iObservation_Count; i++)
		pPoint_Per_Cam[Observation[i].m_iCamera_Index]++;

	int iCur = 0;
	for (i = 0; i < iObservation_Count; i++)
	{
		int iCount=pPoint_Per_Cam[i];	
		pStart[i] = iCur;
		iCur += iCount;
	}
	Point_2D<_T>* pObservation = (Point_2D<_T>*)pMalloc(iObservation_Count * sizeof(Point_2D<_T>));
	for (i = 0; i < iObservation_Count; i++)
	{
		Point_2D<_T> oPoint = Observation[i];
		pObservation[pStart[oPoint.m_iCamera_Index]++] = oPoint;
	}
	memcpy(Observation, pObservation, iObservation_Count * sizeof(Point_2D<_T>));
	Free(pObservation);
	Free(pPoint_Per_Cam);

	/*printf("\n");
	for (i = 0; i < iObservation_Count; i++)
	{
		Point_2D<_T>oPoint = Observation[i];
		printf("Camera Index:%d Point_Index:%d Pos:%f %f\n", oPoint.m_iCamera_Index, oPoint.m_iPoint_Index, oPoint.m_Pos[0], oPoint.m_Pos[1]);
	}*/
	return;
}
template<typename _T>static void Free_LM_Param_PnP_Pose_Point(LM_Param_Ceres<_T>* poParam)
{
	LM_Param_Ceres<_T> oParam = *poParam;
	if (oParam.J)Free(oParam.J);
	if (oParam.Residual)Free(oParam.Residual);
	if (oParam.m_pDiag)Free(oParam.m_pDiag);
	if (oParam.JtE)Free(oParam.JtE);
	if (oParam.H)Free(oParam.H);
	*poParam = {};
}
template<typename _T>static void Init_LM_Param_PnP_Pose_Point(LM_Param_Ceres<_T>* poParam, int iCamera_Count, int iObservation_Count)
{//��PnP_Pose_Point�����ʼ��LM
	LM_Param_Ceres<_T> oParam;
	//��ʾ��ʼ��ֵ����ö�һ����һ�������û
	oParam.reuse_diagonal = 0;		//��ʼ��Ϊʲô������
	oParam.bIsStepSuccessful = 0;	//�����Ƿ�ɹ�
	oParam.bStep_is_valid = 0;
	oParam.m_iIter = 0;
	oParam.num_consecutive_nonmonotonic_steps = 0;
	oParam.decrease_factor = 2.f;
	oParam.accumulated_reference_model_cost_change = 0;
	oParam.accumulated_candidate_model_cost_change = 0;
	oParam.radius = 10000.f;

	int iSize;
	oParam.J_9 = (_T(*)[2][9])pMalloc(iObservation_Count * 2 * 9 * sizeof(_T));
	oParam.Residual = (_T(*)[2])pMalloc(iObservation_Count * 2 * sizeof(_T));
	iSize = iCamera_Count * 6 + iObservation_Count * 3;
	oParam.m_pDiag = (_T*)pMalloc(iSize * sizeof(_T));
	oParam.JtE = (_T*)pMalloc(iSize * sizeof(_T));
	iSize *= iSize;
	oParam.H = (_T*)pMalloc(iSize * sizeof(_T));
	*poParam = oParam;
}
template<typename _T>static void PnP_Pose_Point_Get_J(_T Point_3D[3], _T Pose[4 * 4], _T Intrinsic[], _T Point_2D[2], _T J_E_Ksi[2 * 6], _T J_UV_P[2 * 3], _T E[2])
{
	_T TP[3];
	_T J_UV_TP[2 * 3];
	//_T Point_2D_1[3];
	
	TP[0] = Pose[0] * Point_3D[0] + Pose[1] * Point_3D[1] + Pose[2] * Point_3D[2] + Pose[3];
	TP[1] = Pose[1*4+0] * Point_3D[0] + Pose[1*4+1] * Point_3D[1] + Pose[1*4+2] * Point_3D[2] + Pose[1*4+3];
	TP[2] = Pose[2*4+0] * Point_3D[0] + Pose[2*4+1] * Point_3D[1] + Pose[2*4+2] * Point_3D[2] + Pose[2*4+3];
	
	//�ɴ಻���ã�ȫһ���Զ�����
	_T z_recip = 1.f / TP[2];
	//_T z_recip_sqr = z_recip * z_recip;

	//������fx/TPz
	J_E_Ksi[0] = Intrinsic[0] * z_recip;
	//fy/TPz
	J_E_Ksi[7] = Intrinsic[1] * z_recip;

	//fx*TPx/TPz
	E[0] = J_E_Ksi[0] * TP[0];
	//fy*TPy/TPz
	E[1] = J_E_Ksi[7] * TP[1];

	//J_E_Ksi[2]=-fx*TPx/(TPz*TPz)
	J_E_Ksi[2] = -E[0] * z_recip;
	//J_E_Ksi[8] = -fy*TPy/(TPz*TPz)
	J_E_Ksi[8] = -E[1] * z_recip;

	E[0]  += Intrinsic[2] - Point_2D[0];
	E[1]  += Intrinsic[3] - Point_2D[1];

	//Disp(Point_3D, 3, 1, "ԭ�㣺");
	//Disp(TP, 3, 1, "��λ��");
	//Disp(E, 1, 2, "Residual");
	//J_E_Ksi[3] -fx*TPx*TPy/(TPz*TPz)
	J_E_Ksi[3] = J_E_Ksi[2] * TP[1];
	//J_E_Ksi[10] = fy*TPx*TPy/(TPz*TPz)
	J_E_Ksi[10] = -J_E_Ksi[8] * TP[0];

	//J_E_Ksi[4] = fx + fx*TPx*TPx/(TPz*TPz)
	J_E_Ksi[4] = Intrinsic[0] - J_E_Ksi[2] * TP[0];
	//J_E_Ksi[9]= -fy - fy*TPy*TPy/)TPz*TPz)
	J_E_Ksi[9] = -Intrinsic[1] + J_E_Ksi[8] * TP[1];

	//J_E_Ksi[5] = -fx*TPy/TPz
	J_E_Ksi[5] = -J_E_Ksi[0] * TP[1];
	//J_E_Ksi[11] = fy*TPx/TPz
	J_E_Ksi[11] = J_E_Ksi[7] * TP[0];
	
	J_E_Ksi[1] = J_E_Ksi[6] = 0;

	J_UV_TP[0] = J_E_Ksi[0], J_UV_TP[1] = J_E_Ksi[1], J_UV_TP[2] = J_E_Ksi[2];
	J_UV_TP[3] = J_E_Ksi[6], J_UV_TP[4] = J_E_Ksi[7], J_UV_TP[5] = J_E_Ksi[8];

	_T R[3 * 3] = { Pose[0],Pose[1],Pose[2],
				Pose[4],Pose[5],Pose[6],
				Pose[8],Pose[9],Pose[10] };
	Matrix_Multiply(J_UV_TP, 2, 3, R, 3, J_UV_P);

	/*Disp(J_E_Ksi, 2, 6, "J_E_Ksi");
	Disp(J_UV_TP, 2, 3, "J_UV_TP");
	Disp(E, 1, 2, "E");*/
	return;
}
template<typename _T>static void Get_J_Residual_PnP_Pose_Point(_T Pose[][16], int iCamera_Count, _T Intrinsic[], _T Point[][3], int iPoint_3D_Count, Point_2D<_T> Observation[], int iObservation_Count,
	_T J[][2][9], _T Residual[][2], _T* pfSum_e)
{//�ȶ�����������������˺���������е�����
	int i;
	_T fSum_e = 0;
	for (i = 0; i < iObservation_Count; i++)
	{
		Point_2D<_T> oPoint_2D = Observation[i];
		_T J_E_Ksi[2 * 6],J_UV_P[2*3];
		PnP_Pose_Point_Get_J(Point[oPoint_2D.m_iPoint_Index], Pose[oPoint_2D.m_iCamera_Index], Intrinsic, oPoint_2D.m_Pos, 
			J_E_Ksi,J_UV_P, Residual[i]);

		fSum_e += Residual[i][0] * Residual[i][0] + Residual[i][1] * Residual[i][1];
		Copy_Matrix_Partial(J_E_Ksi, 2, 6, (_T*)J[i], 9, 0, 0);
		Copy_Matrix_Partial(J_UV_P, 2, 3, (_T*)J[i], 9, 6, 0);
	}
	*pfSum_e = fSum_e/2.f;
	return;
}
//template<typename _T>static void Get_H_Block(_T J[2][9], _T Pose[6 * 6], _T Pose_Point[6 * 3], _T Point[3 * 3])
//{//������������
//	int i, j, iDest_Pos;
//	//Pose�飬�����ۼ�
//	for (i = 0; i < 6; i++)
//	{
//		iDest_Pos = i * 6 + i;
//		for (j = i; j < 6; j++,iDest_Pos++)		//iDest_Pos = i * 6 + j;
//			Pose[iDest_Pos] += J[0][i] * J[0][j] + J[1][i] * J[1][j];
//	}
//	for (i = 0; i < 6; i++)
//		for (j = 0; j < 3; j++)
//			Pose[i * 3 + j] = J[0][i] * J[0][j + 6] + J[1][i] * J[1][j + 6];
//}
template<typename _T>static void Get_H_PnP_Pose_Point(_T J[][2][9], Point_2D<_T>Observation[], int iPose_Count,int iPoint_Count,int iObservation_Count, _T Sigma_H[])
{
	int i, iWidth_Pose=iPose_Count*6, 
		iWidth_H = iWidth_Pose + iPoint_Count * 3;
	memset(Sigma_H, 0, iWidth_H * iWidth_H * sizeof(_T));
	for (i = 0; i < iObservation_Count;)
	{
		int iPrevious_Pose_Index = Observation[i].m_iCamera_Index;
		int iPose_Start = iPrevious_Pose_Index* 6;
		for (; i < iObservation_Count; i++)
		{
			Point_2D<_T> oPoint = Observation[i];
			if (oPoint.m_iCamera_Index != iPrevious_Pose_Index)
				break;
			//������JtJ, JtE�ӵ�ϵ�������b������
			_T H[9 * 9];
			int iPoint_Start = iWidth_Pose + oPoint.m_iPoint_Index * 3;

			//Disp((_T*)J[j], 2, 9, "J");
			for (int k = 0; k < 2; k++)
			{
				Transpose_Multiply(J[i][k], 9, 1, H);
				//Disp(H, 9, 9, "H");
				Matrix_Add_Partial(H, 9, 0, 0, 6, 6, Sigma_H, iWidth_H, iPose_Start, iPose_Start);
				Matrix_Add_Partial(H, 9, 6, 0, 3, 6, Sigma_H, iWidth_H, iPoint_Start, iPose_Start);
				Matrix_Add_Partial(H, 9, 0, 6, 6, 3, Sigma_H, iWidth_H, iPose_Start, iPoint_Start);
				Matrix_Add_Partial(H, 9, 6, 6, 3, 3, Sigma_H, iWidth_H, iPoint_Start, iPoint_Start);
			}
			//Disp(Sigma_H, 9, 9, "Sigma_H");
		}
	}
	//Disp(Sigma_H, iWidth_H, iWidth_H, "Sigma_H");
	return;
}
template<typename _T>static void Get_JtE_PnP_Pose_Point(_T J[][2][9], _T Residual[][2], Point_2D<_T> Observation[], int iPose_Count, int iPoint_Count, int iObservation_Count, _T JtE[])
{//�ϸ��� JtJ x = -JtE
	int i, iPoint_Start = iPose_Count * 6;
	int iOrder = iPose_Count * 6 + iPoint_Count * 3;
	memset(JtE, 0, iOrder * sizeof(_T));
	for (i = 0; i < iObservation_Count; i++)
	{
		//Disp((_T*)J[i], 2, 9, "J");
		Point_2D<_T>oPoint = Observation[i];
		_T JtE_1[9];
		for (int j = 0; j < 2; j++)
		{
			Matrix_Multiply(J[i][j], 9, 1, &Residual[i][j], 1, JtE_1);
			//�ӵ�Ŀ��λ����
			Vector_Add(JtE_1, &JtE[oPoint.m_iCamera_Index * 6], 6,&JtE[oPoint.m_iCamera_Index * 6]);
			Vector_Add(&JtE_1[6], &JtE[iPoint_Start + oPoint.m_iPoint_Index * 3], 3, &JtE[iPoint_Start + oPoint.m_iPoint_Index * 3]);
		}
	}

	//�ǵó���-1����Ax = b�е�b
	for (i = 0; i < iOrder; i++)
		JtE[i] = -JtE[i];
	//Disp(JtE, iOrder, 1, "JtE");
	return;
}
template<typename _T>static void Get_Diag_PnP_Pose_Point(LM_Param_Ceres<_T> oParam, int iOrder, _T lm_diagonal[])
{
	int i;
	//�������Խ���
	/*if (oParam.m_iIter == 1)
		printf("here");*/
	if (!oParam.reuse_diagonal)
	{
		for (i = 0; i < iOrder; i++)
		{
			oParam.m_pDiag[i] = oParam.H[i * iOrder + i];	// sqrt(Abs(A[i * iOrder + i]) / oParam.radius);
			//�޸���
			oParam.m_pDiag[i] =(_T)Clip3(1e-6, 1e32, oParam.m_pDiag[i]); 
		}
	}
	//Disp(oParam.m_pDiag, iOrder,1,  "Diag");
	for (i = 0; i < iOrder; i++)
		lm_diagonal[i] = (_T)sqrt(oParam.m_pDiag[i] / oParam.radius);
}
template<typename _T>static _T fGet_model_cost_change_PnP_Pose_Point(Point_2D<_T> Observation[], int iObservation_Count,_T Residual[][2],_T J[][2][9], _T x[], _T JtE[],int iCamera_Count)
{
	int iPoint_Start = iCamera_Count * 6;
	_T model_cost_change = 0;
	for (int i = 0; i < iObservation_Count; i++)
	{
		//�˴����ش�Ķ����������ԭ����Residual����һ��Bug
		_T E1[] = { Residual[i][0],Residual[i][1] };
		Point_2D<_T> oPoint = Observation[i];

		_T E2[2], x1[9];
		memcpy(x1, &x[oPoint.m_iCamera_Index * 6], 6 * sizeof(_T));
		memcpy(&x1[6], &x[iPoint_Start + oPoint.m_iPoint_Index*3], 3 * sizeof(_T));
		Matrix_Multiply((_T*)J[i], 2, 9, x1, 1, E2);

		E1[0] = (E1[0] + E2[0] / 2.f);
		E1[1] = (E1[1] + E2[1] / 2.f);
		//Disp(E1, 2, 1, "E2");
		model_cost_change += -E2[0] * E1[0] + (-E2[1] * E1[1]);
	}
	return model_cost_change;
}

template<typename _T>static _T fGet_candidate_cost_PnP_Pose_Point(_T Pose[][4 * 4], _T Intrinsic[6], Point_2D<_T> Observation[], _T x[], _T Point[][3], 
	int iPose_Count, int iPoint_Count, int iObservation_Count, _T J[][2][9], _T Residual[][2])
{//����ceres���Ѹ����ɴ��Լ���һ������
 //xΪ��������Ŷ��������Ŷ�������
	_T candidate_cost = 0;
	int i;
	//�ȵ���λ��
	for (i = 0; i < iPose_Count; i++)
	{
		_T* pPose_Delta_6 = &x[i * 6];
		_T Pose_Delta_4x4[4 * 4]; 
		//Disp(pPose_Delta_6, 6, 1, "pPose_Delta_6");
		se3_2_SE3(pPose_Delta_6, Pose_Delta_4x4);
		Matrix_Multiply(Pose_Delta_4x4, 4, 4, Pose[i], 4, Pose[i]);
	}

	//�ٵ����ռ��
	int iPoint_Start = iPose_Count * 6;
	for (i = 0; i < iPoint_Count; i++,iPoint_Start+=3)
		Vector_Add(Point[i], &x[iPoint_Start], 3, Point[i]);
	

	Get_J_Residual_PnP_Pose_Point(Pose, iPose_Count, Intrinsic, Point, iPoint_Count, Observation, iObservation_Count,
		J, Residual,&candidate_cost);

	return candidate_cost;
}

template<typename _T>static void Update_Param_Ceres(LM_Param_Ceres<_T> *poParam,_T candidate_cost,_T model_cost_change)
{//����һ��Cerer��Param
	LM_Param_Ceres<_T> oParam = *poParam;
	_T relative_decrease;
	if (oParam.current_cost > MAX_FLOAT)
		relative_decrease = -MAX_FLOAT;
	/*if (model_cost_change == 0)
	printf("Error");*/
	relative_decrease = (oParam.current_cost - candidate_cost) / model_cost_change;
	//printf("%f\n", relative_decrease);
	const _T historical_relative_decrease =
		(oParam.reference_cost - candidate_cost) /
		(oParam.accumulated_reference_model_cost_change + model_cost_change);

	relative_decrease = Max(relative_decrease, historical_relative_decrease);
	oParam.bIsStepSuccessful = relative_decrease >	0.001;

	//ע�⣬relative_decreaseҲ��Step Quality
	if (oParam.bIsStepSuccessful)
	{
		oParam.radius=(_T)(oParam.radius/std::max(1.0 / 3.0, 1.0 - pow(2.f*relative_decrease - 1.0, 3)));
		oParam.radius = (_T)Min(10000000000000000., oParam.radius);
		//printf("radius:%f relative_decrease:%f\n", oParam.radius,relative_decrease);

		oParam.decrease_factor = 2.f;
		oParam.reuse_diagonal = 0;

		//candidate_cost_, model_cost_change_
		oParam.current_cost = candidate_cost;
		oParam.accumulated_candidate_model_cost_change += model_cost_change;
		oParam.accumulated_reference_model_cost_change += model_cost_change;
		//printf("minimum_cost:%f\n", oParam.minimum_cost);
		if (oParam.current_cost < oParam.minimum_cost)
		{
			oParam.minimum_cost = oParam.current_cost;
			oParam.candidate_cost = oParam.current_cost;
			oParam.accumulated_candidate_model_cost_change = 0;
		}
		else
		{	//������
			printf("here");
			oParam.num_consecutive_nonmonotonic_steps++;
			if (oParam.current_cost > candidate_cost)
			{
				oParam.candidate_cost = oParam.current_cost;
				oParam.accumulated_candidate_model_cost_change = 0.0;
			}
		}
		if (oParam.num_consecutive_nonmonotonic_steps == 0)
		{
			oParam.reference_cost = candidate_cost;
			oParam.accumulated_reference_model_cost_change =
				oParam.accumulated_candidate_model_cost_change;
		}
	}else
	{//ʧ��
		oParam.radius = oParam.radius / oParam.decrease_factor;
		oParam.decrease_factor *= 2.0;
		oParam.reuse_diagonal = 1;
	}
	//if (oParam.m_iIter > 39)
	//printf("current cost:%f candidate_cost:%f\n", oParam.current_cost, candidate_cost);
	*poParam = oParam;
}
template<typename _T>void BA_PnP_Pose_Point(LM_Param_Ceres<_T>* poParam, _T Pose[][16], int iPose_Count, _T Intrinsic[], _T Point[][3], int iPoint_Count, Point_2D<_T> Observation_2D[], int iObservation_Count, _T fLoss_eps = (_T)1e-10)
{//������һ��LM�����������·�������ì�ܷ��� JtJx = -JtE, �������Ϸ���
	LM_Param_Ceres<_T>oParam = *poParam;
	int i, bResult,iOrder = iPose_Count * 6 + iPoint_Count*3;
	//���ڴ�
	_T* lm_diagonal = (_T*)pMalloc(iOrder * sizeof(_T));
	_T* x = (_T*)pMalloc(iOrder * sizeof(_T));
	
	_T(*pPose_1)[4 * 4] = (_T(*)[4 * 4])pMalloc(iPose_Count * 16 * sizeof(_T));
	//_T Intrinsic_1[4];
	_T(*Point_1)[3] = (_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));

	_T(*J_1)[2][9] = (_T(*)[2][9])pMalloc(iObservation_Count * 2 * 9 * sizeof(_T));
	_T(*Residual_1)[2] = (_T(*)[2])pMalloc(iObservation_Count * 2 * sizeof(_T));
	
	if (!oParam.reuse_diagonal)
	{
		Get_H_PnP_Pose_Point(oParam.J_9, Observation_2D, iPose_Count,iPoint_Count, iObservation_Count, oParam.H);
		//Disp(oParam.H, iOrder, 1, "Sigma_H");
		Get_JtE_PnP_Pose_Point(oParam.J_9, oParam.Residual, Observation_2D, iPose_Count,iPoint_Count, iObservation_Count, oParam.JtE);
	}	
	//if(oParam.m_iIter==1)
	//Disp(oParam.J, iObservation_Count * 2, 9, "J");
	
	Get_Diag_PnP_Pose_Point(oParam, iOrder, lm_diagonal);
	//Disp(oParam.m_pDiag, iOrder, 1, "Diag");
	//if (oParam.m_iIter == 5)
		//Disp(lm_diagonal, 1, iOrder, "lm_diagonal");
	
	//��һ�����޸ĶԽ���Ԫ�أ���һ����g2o�и�������g2o�Խ��߼�һ��ͳһ��ֵ
	for (i = 0; i < iOrder; i++)
		oParam.H[i * iOrder + i] +=lm_diagonal[i]*lm_diagonal[i];   // *Diag[i];
	//Disp(oParam.H, iOrder, iOrder, "Sigma_H");
	//Disp(oParam.JtE, iOrder, 1, "JtE");
	//Solve_Linear_Gause(oParam.H, iOrder, oParam.JtE, x, &bResult);
	Solve_Linear_Gause_AAt(oParam.H, iOrder, oParam.JtE, x, &bResult);
	
	//if (oParam.m_iIter == 5)
	//{
	//	//Disp(x, iOrder, 1, "x");
	//	Disp((_T*)oParam.Residual, iObservation_Count, 2, "Residual");
	//}		

	memcpy(pPose_1, Pose, iPose_Count*16 * sizeof(_T));
	memcpy(Point_1, Point, iPoint_Count * 3 * sizeof(_T));

	//����ļ�����������˼·, ��ȻJx= -e�������Ѿ��⵽x,
	//��ô��x���ص���������������ϣ��Jx��ӽ� -e, ��Ȼ�������
	//�������Ϊmodel_cost_change
	_T model_cost_change = fGet_model_cost_change_PnP_Pose_Point(Observation_2D, iObservation_Count,oParam.Residual, oParam.J_9, x, oParam.JtE,iPose_Count);
	int bStep_is_valid = model_cost_change > 0;
	oParam.bStep_is_valid = bStep_is_valid;
	
	_T candidate_cost = fGet_candidate_cost_PnP_Pose_Point(pPose_1, Intrinsic, Observation_2D, 
		x,Point_1, iPose_Count, iPoint_Count, iObservation_Count,J_1,Residual_1);

	Update_Param_Ceres(&oParam, candidate_cost,model_cost_change);

	if (oParam.bIsStepSuccessful)
	{
		memcpy(Pose, pPose_1, iPose_Count * 4 * 4 * sizeof(_T));
		memcpy(Point,Point_1, iPoint_Count * 3 * sizeof(_T));
		memcpy(oParam.J, J_1, iObservation_Count * 2 * 9 * sizeof(_T));
		memcpy(oParam.Residual, Residual_1, iObservation_Count * 2 * sizeof(_T));
	}else
	{//һ��ʯ��Ҫ�ָ�����
		for (i = 0; i < iOrder; i++)
			oParam.H[i * iOrder + i] -= lm_diagonal[i]*lm_diagonal[i];   // *Diag[i];
	}
	if(x)Free(x);
	if (pPose_1)Free(pPose_1);
	if (Point_1)Free(Point_1);
	if (lm_diagonal)Free(lm_diagonal);
	if (J_1)Free(J_1);
	if (Residual_1)Free(Residual_1);
	*poParam = oParam;
	return;
}
template<typename _T>void PnP_Pose_Point(_T Pose[][4*4], int iCamera_Count,_T Intrinsic[4], _T Point[][3], int iPoint_3D_Count, Point_2D<_T> Observation[], int iObservation_Count)
{//����λ��6������Ż������Ż��ڲΣ��ο�ʼ�ĵط�
 //�Ƚ��������ŵĵ�����
	Sort_Observation(Observation,iCamera_Count, iObservation_Count);

	LM_Param_Ceres<_T> oParam;
	Init_LM_Param_PnP_Pose_Point(&oParam,iCamera_Count, iObservation_Count);
	Get_J_Residual_PnP_Pose_Point(Pose, iCamera_Count, Intrinsic, Point, iPoint_3D_Count, Observation, iObservation_Count, oParam.J_9, oParam.Residual, &oParam.current_cost);
	oParam.reference_cost = oParam.candidate_cost=oParam.minimum_cost= oParam.current_cost;
	printf("Iter:%d Error:%e bStep_is_valid:%d bIsStepSuccessful:%d\n", oParam.m_iIter, oParam.current_cost, (int)oParam.bStep_is_valid, (int)oParam.bIsStepSuccessful);
	//printf("radius:%f\n", oParam.radius);
	
	_T fPrevious_Cost = (_T)1e20;
	_T fLoss_Diff_eps = (_T)1e-10;
	fLoss_Diff_eps*= iObservation_Count;   //����֮��Ĳ������йأ�����Խ�࣬epsԽ��
	for (oParam.m_iIter = 1;oParam.m_iIter<50; oParam.m_iIter++)
	{
		BA_PnP_Pose_Point(&oParam, Pose, iCamera_Count, Intrinsic, Point, iPoint_3D_Count,
			Observation, iObservation_Count);
		if (oParam.bIsStepSuccessful)
		{
			printf("Iter:%d Error:%e bStep_is_valid:%d bIsStepSuccessful:%d\n", oParam.m_iIter, oParam.current_cost, (int)oParam.bStep_is_valid, (int)oParam.bIsStepSuccessful);
			printf("radius:%f\n", oParam.radius);
		}else
			printf("Fail\n");

		if (!oParam.bStep_is_valid && !oParam.bIsStepSuccessful)
			break;

		if (abs(oParam.current_cost - fPrevious_Cost) < fLoss_Diff_eps && oParam.bIsStepSuccessful)
		{
			//printf("Iter:%d Error:%f bStep_is_valid:%d bIsStepSuccessful:%d\n", oParam.m_iIter, oParam.current_cost / 2, (int)oParam.bStep_is_valid, (int)oParam.bIsStepSuccessful);
			break;
		}			
		fPrevious_Cost = oParam.candidate_cost;
	}
	Free_LM_Param_PnP_Pose_Point(&oParam);
	return;
}
