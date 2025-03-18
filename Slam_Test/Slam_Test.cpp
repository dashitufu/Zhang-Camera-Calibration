// Slam_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#pragma once
#include <iostream>
#include "Common.h"
#include "Image.h"
//#include "sift.h"
#include "Matrix.h"
#include "Reconstruct.h"

extern void Test_Main();
extern void IMU_Test_Main();
void Chess_Board_Test();

extern "C"
{
#include "Buddy_System.h"
}
using namespace std;

template<typename _T>int bTemp_Load_Data(char* pcFile, _T(**ppCamera)[4 * 4], int* piCamera_Count, _T** ppDepth);

#define Image_Pos_2_Norm_1(x,y,K,Point) \
{ Point[0] = ((x) - K[2]) / K[0];Point[1] = ((y) - K[5]) / K[4]; Point[2] = 1; }    \

//空间坐标投影到像素平面，x,y为像素平面位置
#define Point_Pos_2_Image(Point,K,x,y) \
{ x = Point[0]*K[0]/Point[2] + K[2]; y = Point[1]*K[4]/Point[2] + K[5]; }   \

#define Interpolate(oImage, x, y,fValue)    \
{   \
    int iPos = ((int)y) * oImage.m_iWidth + (int)x; \
    unsigned char* pd = &oImage.m_pChannel[0][iPos]; \
    _T xx = x - floor(x); \
    _T yy = y - floor(y); \
    fValue= _T(((1 - xx) * (1 - yy) * _T(pd[0]) + \
        xx * (1 - yy) * _T(pd[1]) + \
        (1 - xx) * yy * _T(pd[oImage.m_iWidth]) + \
        xx * yy * _T(pd[oImage.m_iWidth + 1])) / 255.f); \
}

template<typename _T>static _T NCC(Image oRef, Image oCur, int x1, int y1, _T x2, _T y2)
{
    const int ncc_window_size=3;    //窗口半径
    const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);     //窗口面积
    int i, x, y, iPos;
    _T fValue_Cur,
        mean_curr = 0;
    _T values_curr[ncc_area];
    unsigned char values_ref[ncc_area], fValue_Ref;
    union {
        unsigned int mean_ref_int = 0;
        _T mean_ref;
    };

    for (i = 0, y = -ncc_window_size; y <= ncc_window_size; y++)
    {
        //这就奇了怪了，参考区域用的是整像素，另一相机的搜索区域有必要用子像素吗？
        iPos = ((int)(y + y1)) * oRef.m_iWidth - ncc_window_size + (int)x1;
        for (x = -ncc_window_size; x <= ncc_window_size; x++, i++, iPos++)
        {
            //算出来的value_curr就是另一相机搜索区域对应的点的插值，个人认为此处用必须以参考点区域也用才有意义
            //可以尝试用整像素试一下
            Interpolate(oCur, x2 + x, y2 + y, fValue_Cur);
            
            mean_curr += fValue_Cur;

            fValue_Ref = oRef.m_pChannel[0][iPos];
            values_ref[i] = fValue_Ref;
            mean_ref_int += fValue_Ref;

            values_curr[i] = fValue_Cur;

        }
    }
    mean_ref = (_T)mean_ref_int / (ncc_area * 255.f);
    mean_curr /= (_T)ncc_area;
    
    //后面算个协方差与相关系数
    //协方差 cov(x,y) = E((x-Ex)*(y=ey))
    //相关系数  rho = cov(x,y)/sqrt(dx)*sqrt(dy)
    _T cov,     //协方差
        fDenom_Ref, fDenom_Mean;    //方差

    
    fDenom_Ref = cov = fDenom_Mean = 0;
    for (i = 0; i < ncc_area; i++)
    {
        _T fValue_ref = values_ref[i] / 255.f - mean_ref;
        cov += fValue_ref* (values_curr[i] - mean_curr);
        fDenom_Ref += fValue_ref * fValue_ref;
        fDenom_Mean += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
    }

    //由于n可以约掉，以下可以不算
    cov /= ncc_area;
    fDenom_Ref /= ncc_area;
    fDenom_Mean /= ncc_area;

    return cov/(sqrt(fDenom_Ref)* sqrt(fDenom_Mean) + 1e-10);
}
template<typename _T>static int epipolarSearch(Image oRef, Image oCur,_T K[3*3],
    int x, int y, _T fDepth, _T fDepth_Cov, _T Delta_T[4 * 4], _T pt_curr[2], _T epipolar_direction[2])
{//极线搜索，这里会比较复杂，
 //Delta_T：应该是oRef与oCur两图之间的位姿差
 //oRef: Image 0;   oCur: oImage_i
//K: 此处要有相机内参
    _T f_ref[3], P_ref[4], px_mean_curr[4];
    //将Image_0的像素位置转换为归一化平面位置
    Image_Pos_2_Norm_1(x, y,K, f_ref);
    //此处怀疑将平面点换算成空间点，统统缩会去，以0,0为中心的单位球面上，有什么营养？
    Normalize(f_ref, 3, f_ref);

    //顺着这个点向空间发射出去一条射线，长度为fDepth,还是范围为fDepth?
    Matrix_Multiply(f_ref, 3, 1, fDepth, P_ref);
    P_ref[3] = 1;
    //printf("%f\n", fGet_Mod(P_ref, 3)); //此处证明了射线长度为fDepth

    //这点做一个变换，称为另一个相机看到的点
    Matrix_Multiply(Delta_T, 4, 4, P_ref, 1, px_mean_curr);
    //printf("%f\n", fGet_Mod(px_mean_curr, 3)); //另一相机到点的射线长度也为fDepth其实也没毛病，极面组成个等腰三角形
    Point_Pos_2_Image(px_mean_curr, K, px_mean_curr[0], px_mean_curr[1]);
    
    //这个怀疑只是拍脑袋
    _T d_min = fDepth - 3 * fDepth_Cov, d_max = fDepth + 3 * fDepth_Cov;
    if (d_min < 0.1f)
        d_min = 0.1f;

    _T px_min_curr[4], px_max_curr[4];
    Matrix_Multiply(f_ref, 3, 1, d_min, px_min_curr);
    //对参考点射线上取个最小值px_min_curr
    px_min_curr[3] = 1;
    //将该点用另一个相机看，得到新的空间点px_min_curr
    Matrix_Multiply(Delta_T, 4, 4, px_min_curr, 1, px_min_curr);
    //投影回像素平面
    Point_Pos_2_Image(px_min_curr, K, px_min_curr[0], px_min_curr[1]);
    //至此，已经得到参考点射线最小位置在另一相机上的投影

    //参考点取个射线上最大点
    Matrix_Multiply(f_ref, 3, 1, d_max, px_max_curr);
    px_max_curr[3] = 1;
    //换另一个相机看，还是空间点
    Matrix_Multiply(Delta_T, 4, 4, px_max_curr, 1, px_max_curr);
    //投影到另一相机的像素平面上
    Point_Pos_2_Image(px_max_curr, K, px_max_curr[0], px_max_curr[1]);
    //至此，已经得到射线上最大位置在另一相机上的投影
    //回顾一下，参考点共算了三个值，最小值，中间值，最大值，都在另一相机上得到像素平面上的投影位置
    //该三点应该是共线，应为是同一条射线的投影。也就是要搜的极线

    _T epipolar_line[2], half_length;
    Vector_Minus(px_max_curr, px_min_curr, 2, epipolar_line);
    Normalize(epipolar_line, 2, epipolar_direction);    //算个极线方向，营养何在？
    //莫非以下才是搜索范围？
    half_length = 0.5f * fGet_Mod(epipolar_line, 2);
    if (half_length > 100)
        half_length = 100;

    _T best_ncc = -1.f, best_px_curr[2], px_curr[2], Temp[2], l,ncc;
    int i;
    
    for (i = 0, l = -half_length; l <= half_length; l += 0.7f, i++) //这句话是沿着极线爬
    {//感觉就是对参考点做一个范围为(-half_length,+half_length)沿极线搜索，Step为0.7
        //极线方向为epipolar_direction, 在这个方向上走l距离
        Matrix_Multiply(epipolar_direction, 2, 1, l, Temp);
        //从哪开始走？从mean开始走,走到px_curr
        Vector_Add(px_mean_curr, Temp, 2, px_curr);
        //判断Temp点是否还在图的内框内
        if (!(px_curr[0] >= 20 && px_curr[0] < oRef.m_iWidth - 20 &&
            px_curr[1] >= 20 && px_curr[1] < oRef.m_iHeight - 20))
            continue;

        ncc = 0;
        //用相关系数来搞，相关系数越大越相关
        ncc = NCC(oRef, oCur, x, y, px_curr[0], px_curr[1]);
        if (ncc > best_ncc)
        {
            best_ncc = ncc;
            best_px_curr[0] = px_curr[0];
            best_px_curr[1] = px_curr[1];
            //printf("rho:%f\n", ncc);
        }
    }
    if (best_ncc < 0.85f)      // 只相信 NCC 很高的匹配
        return 0;
    pt_curr[0] = best_px_curr[0];
    pt_curr[1] = best_px_curr[1];
    return 1;
}
template<typename _T>void Temp_Triangulate(_T x1, _T y1, _T x2, _T y2, _T K[3 * 3], _T T[4 * 4])
{

}
template<typename _T>static int updateDepthFilter(_T x1, _T y1, _T x2, _T y2, _T K[3 * 3], _T Delta_T[4 * 4], _T epipolar_direction[2], _T Depth[], _T Depth_Cov[], int iWidth, int iHeight)
{//(x1,y1)为参考点的位置   (x2,y2)是通过极线搜索到的对应点在另一相机像素平面上的投影
    _T T_Inv[4 * 4], f_ref[4], f_curr[4];
    int iResult;

    //此处先搁置争议，不管前面的Delta_T怎么来的，此处只管这个位姿回退到原位置，这是逆的原意
    Get_Inv_Matrix_Row_Op_2(Delta_T, T_Inv, 4, &iResult);
    Disp(Delta_T, 4, 4, "Delta_T");
    Disp(T_Inv, 4, 4, "Delta_Inv");

    //将像素平面点投影到归一化平面上，同样所有点都在单位求面上
    Image_Pos_2_Norm_1(x1, y1, K, f_ref);
    Normalize(f_ref, 3, f_ref);

    //另一个相机同样操作
    Image_Pos_2_Norm_1(x2, y2,K, f_curr);
    Normalize(f_curr, 3, f_curr);

    _T R[3 * 3], t[3], f2[3];
    union {
        _T A[2 * 2];
        _T A_Inv[2 * 2];
    };

    Get_R_t(T_Inv, R, t);
    //f2 = R*f_curr 想干嘛？
    Matrix_Multiply(R, 3, 3, f_curr, 1, f2);

    Temp_Triangulate(x1, y1, x2, y2, K, Delta_T);

    return 0;
}
template<typename _T>void Update(Image oRef, Image oCur, _T K[3*3], _T Depth[], _T Depth_Cov[], _T Delta_T[4 * 4])
{
    int x, y;
    _T pt_cur[2], epipolar_direction[2];
    //显然，以下搜索范围是去掉边缘20个像素，以免捣乱，这个边缘宽度应该可调
    for (x = 20; x < oRef.m_iWidth - 20; x++)
    {
        for (y = 20; y < oRef.m_iHeight - 20; y++)
        {
            //极线搜索没啥好说的，就是沿着极线搜就完
            if (!epipolarSearch(oRef, oCur,K, x, y, Depth[y * oRef.m_iWidth + x], sqrt(Depth_Cov[y * oRef.m_iWidth + x]), Delta_T, pt_cur, epipolar_direction))
                continue;

            //看来这个深度滤波器是关键
            updateDepthFilter((_T)x, (_T)y, pt_cur[0], pt_cur[1],K,Delta_T, epipolar_direction, Depth, Depth_Cov, oRef.m_iWidth, oRef.m_iHeight);
        }
    }
    return;
}

void Epipolor_Search_Test_1()
{
    typedef double _T;
    int i,iCamera_Count,iResult;
    _T(*pCamera)[4 * 4],    //第0帧作为参考帧
        * pRef_Depth,
        * pDepth, * pDepth_Cov,
        * pTi, * pT0;

    _T K[3 * 3] = { 481.2f,0,319.5f,0,-480.0f,239.5f,0,0,1 };   //显然，做了一次的上下颠倒
    char File[256];
    Image oImage_Ref, oImage_Cur;

    //装入所有位姿
    if (!bTemp_Load_Data((char*)"D:\\Software\\3rdparty\\slambook2\\ch12\\test_data\\first_200_frames_traj_over_table_input_sequence.txt", &pCamera, &iCamera_Count, &pRef_Depth))
        return;
    bLoad_Image("c:\\tmp\\temp\\000.bmp", &oImage_Ref);
    pDepth = (_T*)pMalloc(&oMatrix_Mem, oImage_Ref.m_iWidth * oImage_Ref.m_iHeight * sizeof(_T));
    pDepth_Cov = (_T*)pMalloc(&oMatrix_Mem, oImage_Ref.m_iWidth * oImage_Ref.m_iHeight * sizeof(_T));
    for (i = 0; i < oImage_Ref.m_iWidth * oImage_Ref.m_iHeight; i++)
        pDepth[i] = pDepth_Cov[i] = 3.f;
    pT0 = pCamera[0];    //第0个位姿作为参考

    for (i = 1; i < iCamera_Count; i++)
    {
        pTi = pCamera[i];
        
        _T Delta_T[4 * 4],T_Inv[4*4];
        Get_Inv_Matrix_Row_Op_2(pTi, T_Inv, 4, &iResult);
        Matrix_Multiply(T_Inv, 4, 4, pT0, 4, Delta_T);

        //Disp(pT0, 4, 4, "T0");
        //Disp(pTi, 4, 4, "Ti");
        Disp(Delta_T, 4, 4, "Delta_T");

        Get_Inv_Matrix_Row_Op_2(pT0, T_Inv, 4, &iResult);
        Matrix_Multiply(pTi, 4, 4, T_Inv, 4, Delta_T);
        Disp(Delta_T,4,4,"Delta_T");
        Get_Inv_Matrix_Row_Op_2(Delta_T, T_Inv, 4, &iResult);
        Disp(T_Inv, 4, 4, "Delta_Inv");

        sprintf(File, "c:\\tmp\\temp\\%03d.bmp", 0);
        if (!bLoad_Image(File, &oImage_Cur))
            continue;
        //Disp(Delta_T, 4, 4);
        Update<_T>(oImage_Ref, oImage_Cur,K, pDepth, pDepth_Cov, Delta_T);

        //Disp_T(pT0);
    }

    if (pCamera)Free(pCamera);
    if (pRef_Depth)Free(pRef_Depth);
    if (pDepth)Free(pDepth);
    if (pDepth_Cov)Free(pDepth_Cov);
    Free_Image(&oImage_Ref);
    return;
}


static void Test_1()
{
    typedef float _T;
    //_T K[] = { 481.2f,0,319.5f,0,480.0f,239.5f,0,0,1 };
    _T K[] = { 640, 10, 240,0,720, 320,0,0,1 };
    //_T K[] = { 1135, 0, 1135,0,1135, 640,0,0,1 };

    _T I[4 * 4];
    Image_Match_Param<_T> oParam = {};

    //Estimate_2_Image_T("c:\\tmp\\1.bmp", "c:\\tmp\\2.bmp", K, &oParam);
    Estimate_2_Image_T("C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Ajay\\A01.bmp",
        "C:\\Users\\admin\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Ajay\\A02.bmp", K, &oParam);
    Point_Cloud<_T> oPC;
    Init_Point_Cloud(&oPC, 10000, 1);

    //Draw_Sphere(&oPC, 0.f, 0.f, 0.f);
    //Draw_Line(&oPC, 0.f, 0.f, 0.f, 5.f, 5.f, 5.f,100, 255, 0, 0);
    Gen_I_Matrix(I, 4, 4);
    Draw_Camera(&oPC, oParam.T);
    Draw_Camera(&oPC, I);

    Draw_Image(&oPC, oParam.m_oImage_0, (_T*)NULL, K, 1000);
    Draw_Image(&oPC, oParam.m_oImage_1, oParam.T, K, 1000);

    _T R[3 * 3], Rotation_Vector[4];;
    Get_R_t(oParam.T, R, (_T*)NULL);
    Rotation_Matrix_2_Vector(R, Rotation_Vector);
    Disp(Rotation_Vector, 1, 4);
    Disp(oParam.T, 4, 4,"T");
    //bSave_PLY("c:\\tmp\\1.ply", oPC);


    _T fError = 0;
    Test_Triangulate(oParam.m_pPoint_3D, oParam.m_pNorm_Point_0, oParam.m_pNorm_Point_1, oParam.m_iMatch_Count,oParam.T, &fError);
    printf("Error:%f\n", fError);

    Free_2_Image_Match(&oParam);
    Free_Point_Cloud(&oPC);


    Disp(K, 3, 3, "K");
    _T Temp[3 * 3];
    Get_Inv_Matrix_Row_Op_2(K, Temp, 3);
    Disp(Temp, 3, 3, "K_Inv");
    Get_K_Inv(K, Temp);
    Disp(Temp, 3, 3, "K_Inv");
    Matrix_Multiply(K, 3, 3, Temp, 3, Temp);
    Disp(Temp, 3, 3, "I");
    return;
}
template<typename _T>void Test_H(_T P1[][2], _T P2[][2], _T H[3*3], int iCount,_T *pfError=NULL)
{//测试p2 = s*H*p1
    int i;
    _T fTotal = 0;
    for (i = 0; i < iCount; i++)
    {
        _T p1[] = { P1[i][0],P1[i][1],1.f }, x[3],
            y[3] = { P2[i][0],P2[i][1],1 };
        
        //以下求p2 = sHp1， 可见s只能针对没对点对，统一优化一个不靠谱
        Matrix_Multiply(H, 3, 3, p1, 1, x);
        _T s = (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]) / (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
        printf("s:%f s*Hp1=(%f %f %f)\n", s, s * x[0], s * x[1], s * x[2]);
        Matrix_Multiply(x, 1, 3, s, x);
        fTotal += fGet_Distance(x, y, 3);
        if (fGet_Distance(x, y, 3) > 12)
        {
            Disp(x, 1, 3, "p2'");
            Disp(P2[i], 1, 2, "p2");
            printf("Error:%f i:%d\n", fGet_Distance(x, y, 3), i);
        }
            
        
    }
    printf("Total:%f avg:%f\n", fTotal, fTotal/iCount);
    if (pfError)
        *pfError = fTotal / iCount;
    return;
}
template<typename _T>void Zhang_Get_Vij(_T H[3 * 3], _T Vij[6],int i,int j)
{//对张氏标定法搞Vij*b中的一条方程
    _T* a0 = &H[0], * a1 = &H[3], * a2 = &H[6];
    //a0i*a0j*b00
    Vij[0] = a0[i] * a0[j];
    //(a1i*a0j +a0i*a1j)*b01
    Vij[1] = a1[i] * a0[j] + a0[i] * a1[j];
    //a1i*a1j*b11
    Vij[2] = a1[i] * a1[j];
    //(a2i*a0j+a0i*a2j)*b02 
    Vij[3] = a2[i] * a0[j] + a0[i] * a2[j];
    //(a2i*a1j + a1i*a2j)*b12
    Vij[4] = a2[i] * a1[j] + a1[i] * a2[j];
    //a2i*a2j*b22
    Vij[5] = a2[i] * a2[j];
    return;
}
template<typename _T>void Zhang_Get_A_2_row(_T H[3*3], _T A[6*2])
{//给定一个H，搞出两行系数
    //v01
    Zhang_Get_Vij(H, &A[0], 0, 1);
    //v00 - v11
    _T Temp[6];
    //Disp(H, 3, 3, "H");
    Zhang_Get_Vij(H, &A[6], 0, 0);
    Zhang_Get_Vij(H, Temp, 1, 1);
    //Vector_Minus(Temp, &A[6], 6, &A[6]);
    Vector_Minus(&A[6],Temp, 6, &A[6]);
    return;
}
template<typename _T>void Test_Zhang_1(_T B[3 * 3], _T K[3 * 3])
{
    _T K_Inv[3 * 3], K_Inv_Transpose[3 * 3];
    _T B1[3 * 3];
    int iResult;
    Get_Inv_Matrix_Row_Op_2(K, K_Inv, 3, &iResult);
    Matrix_Transpose(K_Inv, 3, 3, K_Inv_Transpose);

    //K'(-1)* K(-1)
    Matrix_Multiply(K_Inv_Transpose, 3, 3, K_Inv, 3, B1);
    //Matrix_Multiply(B1, 3, 3, B[8] / B1[8], B1);
    Disp(B1,3,3,"K'(-1)* K(-1)");
    Disp(B,3,3,"B");
    return;
}
template<typename _T>void H_2_R_t(_T H[3 * 3], _T K[3*3], _T Point_1[][2], _T Point_2[][2], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3], int* piCount, unsigned char* pInlier_Mask)
{//从E中恢复R矩阵与 位移 t, 注意了，由于向量可行可列，在列向量前提下，
    //此处用归一化坐标
    //E= t^ * R 这才跟原来一直的计算对齐，先旋转后位移。否则天下大乱
    //并且，准确的表达是 E = a * t^ * R, 其中 a是E的特征值。否则数值不对
    _T R1[3 * 3], R2[3 * 3], t1[3], t2[3];
    _T* Comb[4][2] = { {R1,t1},{R2,t1},{R1,t2},{R2,t2} };
    _T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
    unsigned char* pInlier_Mask_1 = (unsigned char*)pMalloc(iCount);

    int i;
    Decompose_H(H, R1, R2, t1, t2,K); //E = a * t^ * R

    //组成4对 (R1,t1), (R2,t1),(R1,t2),(R2,t2),分别检验转换后结果的对错
    int Count_1[4], iMax_Count = 0, iMax_Index;;
    for (i = 0; i < 4; i++)
    {
        Count_1[i] = iCount;
        //_T v[4];
        //fsRotation_Matrix_2_Vector(Comb[i][0],v);
        //Disp(v, 1, 4,"V");
        Check_Cheirality(Point_1, Point_2, &Count_1[i], Comb[i][0], Comb[i][1], pPoint_3D, pInlier_Mask_1);

        if (Count_1[i] > iMax_Count)
        {
            iMax_Count = Count_1[i];
            iMax_Index = i;
            if (Point_3D)
                memcpy(Point_3D, pPoint_3D, iMax_Count * 3 * sizeof(_T));
            if (pInlier_Mask)
                memcpy(pInlier_Mask, pInlier_Mask_1, iCount);
        }
        if (Count_1[i] == iCount)
            break;  //找到了
    }

    if (iMax_Count)
    {
        memcpy(R, Comb[iMax_Index][0], 3 * 3 * sizeof(_T));
        memcpy(t, Comb[iMax_Index][1], 3 * sizeof(_T));
    }
    if (piCount)
        *piCount = iMax_Count;

    Free(pPoint_3D);
    Free(pInlier_Mask_1);
    return;
}

template<typename _T>void Zhang_Test_1(_T H[3 * 3],_T K[3*3], _T Point_1[][2], _T Point_2[][2],int iCount)
{//对张氏算法一顿验证
    _T h1[3] = { H[0],H[3],H[6] },
        h2[3] = { H[1],H[4],H[7] };
    _T K_Inv[3 * 3], K_Inv_Transpose[3 * 3], Temp[3 * 3], B1[3 * 3];
    int iResult;
    Get_Inv_Matrix_Row_Op_2(K, K_Inv, 3, &iResult);
    Matrix_Transpose(K_Inv, 3, 3, K_Inv_Transpose);
    Matrix_Multiply(K_Inv_Transpose, 3, 3, K_Inv, 3, B1);

    //先验证 h1' * K'(-1) * K(-1) * h2 = 0
    Matrix_Multiply(h1, 1, 3, B1, 3, Temp);
    Matrix_Multiply(Temp, 1, 3, h2, 1, Temp);
    printf("h1' * K'(-1) * K(-1) * h2=%f\n", Temp[0]);

    //验证h1' * K'(-1) * K(-1) * h1 = λ²， 若h[8]不做置1操作，则λ应该=1
    Matrix_Multiply(h1, 1, 3, B1, 3, Temp);
    Matrix_Multiply(Temp, 1, 3, h1, 1, Temp);
    printf("h1' * K'(-1) * K(-1) * h1=%f\n", Temp[0]);

    Matrix_Multiply(h2, 1, 3, B1, 3, Temp);
    Matrix_Multiply(Temp, 1, 3, h2, 1, Temp);
    printf("h2' * K'(-1) * K(-1) * h2=%f\n", Temp[0]);


    return;
}
template<typename _T>static void Get_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T H[])
{//通过一堆点对求解一个H矩阵
    _T *pCur,* A = (_T*)pMalloc(iCount * 2 * 9 * sizeof(_T));
    memset(A, 0, iCount * 2 * 9 * sizeof(_T));
    pCur = A;
    for (int i = 0; i < iCount; i++)
    {
        _T u1 = Point_1[i][0], v1 = Point_1[i][1],
            u2 = Point_2[i][0], v2 = Point_2[i][1];
        //u1   v1	1	0	0	0	-u1u2	-v1u2	-u2		*h =	0
        pCur[0] = u1;
        pCur[1] = v1;
        pCur[2] = 1;
        pCur[6] = -u1*u2;
        pCur[7] = -v1 * u2;
        pCur[8] = -u2;
        pCur += 9;

        //0	0	0	u1	v1	1	-u1v2	-v1v2	-v2			0
        pCur[3] = u1;
        pCur[4] = v1;
        pCur[5] = 1;
        pCur[6] = -u1 * v2;
        pCur[7] = -v1 * v2;
        pCur[8] = -v2;
        pCur += 9;
    }
    int iResult;
    Solve_Linear_Contradictory(A, iCount * 2, 9,(_T*)NULL, H, &iResult);

    return;
}
void Test_2()
{
    typedef double _T;
    _T K[] = { 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 };
    Image_Match_Param<_T> oParam_12 = {}, oParam_23 = {}, oParam_13 = {};
    int iResult,iCount;

    Match_2_Image("C:\\tmp\\temp\\001.bmp", (const char*)NULL,
        "C:\\tmp\\temp\\002.bmp", (const char*)NULL,
        (_T)5000.f, K, &iCount,
        &oParam_12.m_pImage_Point_0, &oParam_12.m_pImage_Point_1,
        &oParam_12.m_pNorm_Point_0, &oParam_12.m_pNorm_Point_1,
        &oParam_12.m_pPoint_3D_0, &oParam_12.m_pPoint_3D_1, &oParam_12.m_oImage_0, &oParam_12.m_oImage_1, &oParam_12.m_oImage_2);
    Ransac_Estimate_H(oParam_12.m_pImage_Point_0, oParam_12.m_pImage_Point_1, iCount, &oParam_12.m_oReport,NULL,(_T)4.f);
    Shrink_Match_Point(&oParam_12.m_pImage_Point_0, &oParam_12.m_pImage_Point_1, oParam_12.m_oReport.m_pInlier_Mask, iCount);

    Match_2_Image("c:\\tmp\\temp\\002.bmp", (const char*)NULL,
        "C:\\tmp\\temp\\003.bmp", (const char*)NULL,
        (_T)5000.f, K, &iCount,
        &oParam_23.m_pImage_Point_0, &oParam_23.m_pImage_Point_1,
        &oParam_23.m_pNorm_Point_0, &oParam_23.m_pNorm_Point_1,
        &oParam_23.m_pPoint_3D_0, &oParam_23.m_pPoint_3D_1, &oParam_23.m_oImage_0, &oParam_23.m_oImage_1, &oParam_23.m_oImage_2);
    Ransac_Estimate_H(oParam_23.m_pImage_Point_0, oParam_23.m_pImage_Point_1, iCount, &oParam_23.m_oReport,NULL, (_T)4.f);
    Shrink_Match_Point(&oParam_23.m_pImage_Point_0, &oParam_23.m_pImage_Point_1, oParam_23.m_oReport.m_pInlier_Mask, iCount);

    Match_2_Image("c:\\tmp\\temp\\001.bmp", (const char*)NULL,
        "C:\\tmp\\temp\\003.bmp", (const char*)NULL,
        (_T)5000.f, K, &iCount,
        &oParam_13.m_pImage_Point_0, &oParam_13.m_pImage_Point_1,
        &oParam_13.m_pNorm_Point_0, &oParam_13.m_pNorm_Point_1,
        &oParam_13.m_pPoint_3D_0, &oParam_13.m_pPoint_3D_1, &oParam_13.m_oImage_0, &oParam_13.m_oImage_1, &oParam_13.m_oImage_2);
    Ransac_Estimate_H(oParam_13.m_pImage_Point_0, oParam_13.m_pImage_Point_1, iCount, &oParam_13.m_oReport,NULL, (_T)4.f);
    Shrink_Match_Point(&oParam_13.m_pImage_Point_0, &oParam_13.m_pImage_Point_1, oParam_13.m_oReport.m_pInlier_Mask, iCount);

    //iCount = oParam_12.m_oReport.m_oSupport.m_iInlier_Count;
    _T* H = (_T*)oParam_12.m_oReport.m_Modal,fError;
    Test_H(oParam_12.m_pImage_Point_0, oParam_12.m_pImage_Point_1, H, oParam_12.m_oReport.m_oSupport.m_iInlier_Count);
    //Matrix_Multiply(H, 3, 3, 1.f / H[8], H);
    
    H = (_T*)oParam_13.m_oReport.m_Modal, fError;
    Test_H(oParam_13.m_pImage_Point_0, oParam_13.m_pImage_Point_1, H, oParam_13.m_oReport.m_oSupport.m_iInlier_Count);
    //Matrix_Multiply(H, 3, 3, 1.f / H[8], H);
    
    H = (_T*)oParam_23.m_oReport.m_Modal, fError;
    Test_H(oParam_23.m_pImage_Point_0, oParam_23.m_pImage_Point_1, H, oParam_23.m_oReport.m_oSupport.m_iInlier_Count);
    //Matrix_Multiply(H, 3, 3, 1.f / H[8], H);

    Disp((_T*)oParam_12.m_oReport.m_Modal, 3, 3, "H_12");
    //Disp((_T*)oParam_13.m_oReport.m_Modal, 3, 3, "H_13");
    //Disp((_T*)oParam_23.m_oReport.m_Modal, 3, 3, "H_23");
    //另一种方法求H矩阵，看看对不对
    Get_H(oParam_12.m_pImage_Point_0, oParam_12.m_pImage_Point_1, oParam_12.m_oReport.m_oSupport.m_iInlier_Count, (_T*)oParam_12.m_oReport.m_Modal);
    Get_H(oParam_23.m_pImage_Point_0, oParam_23.m_pImage_Point_1, oParam_23.m_oReport.m_oSupport.m_iInlier_Count, (_T*)oParam_23.m_oReport.m_Modal);
    Get_H(oParam_13.m_pImage_Point_0, oParam_13.m_pImage_Point_1, oParam_13.m_oReport.m_oSupport.m_iInlier_Count, (_T*)oParam_13.m_oReport.m_Modal);
    Matrix_Multiply((_T*)oParam_12.m_oReport.m_Modal, 3, 3, 1.f / ((_T*)oParam_12.m_oReport.m_Modal)[8], (_T*)oParam_12.m_oReport.m_Modal);
    Matrix_Multiply((_T*)oParam_23.m_oReport.m_Modal, 3, 3, 1.f / ((_T*)oParam_23.m_oReport.m_Modal)[8], (_T*)oParam_23.m_oReport.m_Modal);
    Matrix_Multiply((_T*)oParam_13.m_oReport.m_Modal, 3, 3, 1.f / ((_T*)oParam_13.m_oReport.m_Modal)[8], (_T*)oParam_13.m_oReport.m_Modal);
    /*Test_H(oParam_12.m_pImage_Point_0, oParam_12.m_pImage_Point_1, (_T*)oParam_12.m_oReport.m_Modal, oParam_12.m_oReport.m_oSupport.m_iInlier_Count);
    Test_H(oParam_23.m_pImage_Point_0, oParam_23.m_pImage_Point_1, (_T*)oParam_23.m_oReport.m_Modal, oParam_23.m_oReport.m_oSupport.m_iInlier_Count);
    Test_H(oParam_13.m_pImage_Point_0, oParam_13.m_pImage_Point_1, (_T*)oParam_13.m_oReport.m_Modal, oParam_13.m_oReport.m_oSupport.m_iInlier_Count);*/

    //Zhang_Test_1(H, K,oParam_12.m_pImage_Point_0,oParam_12.m_pImage_Point_1,oParam_12.m_oReport.m_oSupport.m_iInlier_Count);
    _T A[6 * 6],b[6*5];    //用于求解方程 A*b = 0
    Zhang_Get_A_2_row((_T*)oParam_12.m_oReport.m_Modal, &A[0]);
    Zhang_Get_A_2_row((_T*)oParam_23.m_oReport.m_Modal, &A[12]);
    Zhang_Get_A_2_row((_T*)oParam_13.m_oReport.m_Modal, &A[24]);
    Disp(A, 6, 6, "A");
    Solve_Linear_Contradictory(A, 6, 6, (_T*)NULL, b, &iResult);
    Test_Linear(A, 6, (_T*)NULL, b);
    //printf("Mod of b:%f\n", fGet_Mod(b, 6));

    _T fx, fy, cx, cy,ramda,gama;
    //cy = (b01*b02 - b00*b12)/(b00*b11 - b01*b01)
    _T B[3 * 3] = { b[0], b[1],b[3],
                b[1],b[2],b[4],
                b[3],b[4],b[5] };
    //cy = (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0] * B[1 * 3 + 2]) / (B[0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]);

    cy = (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0 * 3 + 0] * B[1 * 3 + 2]) / (B[0 * 3 + 0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]);
    //ramda = b22 - [b02² + cy * (b01 * b02 - b00 * b12)] / b00
    //ramda = B[2 * 3 + 2] - (B[0 * 3 + 2] * B[0 * 3 + 2] + cy * (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0 * 3 + 0] * B[1 * 3 + 2])) / B[0 * 3 + 0];
    ramda = 1.f;
    //fx = sqrt(ramda/b00)
    fx = sqrt(ramda / B[0 * 3 + 0]);
    //fy = sqrt(ramda * b00) / (b00 * b11 - b01 * b01)
    fy = sqrt( (ramda * B[0 * 3 + 0]) / (B[0 * 3 + 0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]));
    //gama= -b01*fx*fx*fy/ramda
    gama = -B[0*3+1] * fx * fx * fy / ramda;
    //cx	= gama*cy/fx - B02*fx*fx/ramda
    cx = gama * cy / fx - B[0 * 3 + 2] * fx * fx / ramda;
    memset(K, 0, 9 * sizeof(_T));
    K[0] = fx; K[4] = fy; K[2] = cx; K[5] = cy; K[1] = gama; K[8] = 1;

    Test_Zhang_1(B, K);


    Free_2_Image_Match(&oParam_12);
    Free_2_Image_Match(&oParam_23);
    Free_2_Image_Match(&oParam_13);
    return;
}
template<typename _T>void Gen_Plane(float a, float b, float c, float d, int iCount,_T(**ppPoint_3D)[3])
{
    _T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
    //ax + by +cz +d =0 => z = (-ax -by -d)/c
    int iPos,iRange = (int) ceil(sqrt(iCount)/2);
    float x, y, z;
    for (iPos=0,y =(float)(-iRange); y < iRange; y++)
    {
        for (x = (float)(-iRange); x < iRange && iPos<iCount; x++,iPos++)
        {
            z = (-a*x - b*y - d) / c;
            pPoint_3D[iPos][0] = x;
            pPoint_3D[iPos][1] = y;
            pPoint_3D[iPos][2] = z;
        }
    }
    
    //bSave_PLY("c:\\tmp\\1.ply", pPoint_3D,iPos);
    if (ppPoint_3D)
        *ppPoint_3D = pPoint_3D;
    return;
}
template<typename _T>void Add_Point(Point_Cloud<_T>* poPC, _T Point[][3],int iCount,int R=255,int G=255,int B=255)
{
    Point_Cloud<_T> oPC = *poPC;
    if (oPC.m_iCount + iCount >= oPC.m_iMax_Count)
    {
        printf("Insufficient space in Add_Point\n");
        return;
    }
    for (int i = 0; i < iCount; i++)
    {
        oPC.m_pPoint[oPC.m_iCount + i][0] = Point[i][0];
        oPC.m_pPoint[oPC.m_iCount + i][1] = Point[i][1];
        oPC.m_pPoint[oPC.m_iCount + i][2] = Point[i][2];
        oPC.m_pColor[oPC.m_iCount + i][0] = R;
        oPC.m_pColor[oPC.m_iCount + i][1] = G;
        oPC.m_pColor[oPC.m_iCount + i][2] = B;
    }
    oPC.m_iCount += iCount;
    *poPC = oPC;
    return;
}
void Test_3()
{
    typedef double _T;
    Point_Cloud<_T> oPC;
    _T (*pPoint_3D_1)[3], 
        (*pPoint_3D_2)[3],
        (*pPoint_3D_3)[3]; //空间点
    _T(*pPoint_2D_1)[2],
        (*pPoint_2D_2)[2],
        (*pPoint_2D_3)[2];  //像素平面点

    int i,iResult,iCount = 110;
    _T K[3 * 3] = { 640,0,640,0,640,360,0,0,1 };
    //_T K[3 * 3] = { 1,0,0,0,1,0,0,0,1 };
    _T I[4 * 4], T[4 * 4];

    pPoint_3D_2 = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
    pPoint_3D_3 = (_T(*)[3])pMalloc(iCount * 3 * sizeof(_T));
    pPoint_2D_1 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
    pPoint_2D_2 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
    pPoint_2D_3 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));

    Gen_I_Matrix(I, 4, 4);
    //先造一些点
    Gen_Plane(1.f, 2.f, 3.f, -20.f, iCount,&pPoint_3D_1);
    Init_Point_Cloud(&oPC, 10000, 1);
    Add_Point(&oPC, pPoint_3D_1, iCount,255,0,0);

    //投影到相机1上
    for (i = 0; i < iCount; i++)
    {
        _T Temp[3];
        Matrix_Multiply(K,3,3, pPoint_3D_1[i], 1,Temp);
        Temp[0] /= Temp[2], Temp[1] /= Temp[2], Temp[2] = 1.f;
        //Disp(Temp, 1, 3);
        pPoint_2D_1[i][0] = Temp[0];
        pPoint_2D_1[i][1] = Temp[1];

        Draw_Point(&oPC, Temp[0], Temp[1], Temp[2],255,0,0);
    }
    //Draw_Camera(&oPC, I,255,0,0);
    
    //搞个旋转向量
    {
        _T R[3 * 3], t[3] = { 0.3,0.4,640 };  //位移
        _T V[4] = { 1,1,1 };    //绕这个轴转
        Normalize(V, 3, V);
        V[3] = PI / 30;         //旋转 1/10 pi
        Rotation_Vector_4_2_Matrix(V, R);
        Gen_Homo_Matrix(R, t, T);
    }
    
    //将相机1像素平面上的点施以一个R,t,投影到相机2的像素平面上
    for (i = 0; i < iCount; i++)
    {
        _T Temp[3] = { pPoint_2D_1[i][0], pPoint_2D_1[i][1], 1 };
        _T T1[3 * 3] = { T[0],T[1],T[3],
                    T[4],T[5],T[7],
                    T[8],T[9],T[11] };
        Matrix_Multiply(T1, 3, 3, Temp, 1, Temp);
        Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
        Temp[0] /= Temp[2], Temp[1] /= Temp[2], Temp[2] = 1.f;
        pPoint_2D_2[i][0] = Temp[0];
        pPoint_2D_2[i][1] = Temp[1];
    }    
    //Disp((_T*)pPoint_2D_2, iCount, 2, "Point_2");

    //搞个旋转向量
    {
        _T R[3 * 3], t[3] = { 0.3,0.4,640 };  //位移
        _T V[4] = { 1,1,1 };    //绕这个轴转
        Normalize(V, 3, V);
        V[3] = (PI / 30) *1.2f;         //旋转 1/10 pi
        Rotation_Vector_4_2_Matrix(V, R);
        Gen_Homo_Matrix(R, t, T);
    }

    //将相机1像素平面上的点施以一个R,t,投影到相机3的像素平面上
    for (i = 0; i < iCount; i++)
    {
        _T Temp[3] = { pPoint_2D_1[i][0], pPoint_2D_1[i][1], 1 };
        _T T1[3 * 3] = { T[0],T[1],T[3],
                    T[4],T[5],T[7],
                    T[8],T[9],T[11] };
        Matrix_Multiply(T1, 3, 3, Temp, 1, Temp);
        Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
        Temp[0] /= Temp[2], Temp[1] /= Temp[2], Temp[2] = 1.f;
        pPoint_2D_3[i][0] = Temp[0];
        pPoint_2D_3[i][1] = Temp[1];
    }

    {//验算(u2,v2,1) = 1/z * KT*(u1,v1,0,1)
        _T Temp[4]= { pPoint_2D_1[0][0], pPoint_2D_1[0][1],0, 1 };
        Matrix_Multiply(T, 4, 4, Temp, 1, Temp);
        Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
        Temp[0] /= Temp[2], Temp[1] /= Temp[2], Temp[2] = 1.f;
        Disp(Temp, 1, 3, "1/z * KT*(u1,v1,0,1)");
        Disp(pPoint_2D_3[0], 1, 2, "(u2,v2,1) ");
    }

    //估计一个H矩阵
    _T H[3 * 3];
    Ransac_Report oReport_12, oReport_13, oReport_23;
    Ransac_Estimate_H(pPoint_2D_1, pPoint_2D_2, iCount, &oReport_12);
    Ransac_Estimate_H(pPoint_2D_1, pPoint_2D_3, iCount, &oReport_13);
    Ransac_Estimate_H(pPoint_2D_2, pPoint_2D_3, iCount, &oReport_23);
    {
        memcpy(H, oReport_12.m_Modal, 3 * 3 * sizeof(_T));
        //Test_H(pPoint_2D_1, pPoint_2D_2, H, iCount);
        Disp(H, 3, 3, "H");
        //验算h1' * K'(-1) * K(-1) * h1 = ramda
        _T h1[3] = { H[0],H[3],H[6] },
            h2[3] = { H[1],H[4],H[7] };

        _T K_Inv[3 * 3], K_Inv_Transpose[3 * 3];
        _T B1[3 * 3],Temp[4];
        Get_Inv_Matrix_Row_Op_2(K, K_Inv, 3,&iResult);
        Matrix_Transpose(K_Inv, 3, 3, K_Inv_Transpose);
        Matrix_Multiply(K_Inv_Transpose, 3, 3, K_Inv, 3,B1);
        //Disp(B1, 3, 3, "B1");
        Matrix_Multiply(h1, 1, 3, B1, 3, Temp);
        Matrix_Multiply(Temp, 1, 3, h1, 1, Temp);
        printf("ramda:%f\n", sqrt(Temp[0]));

        Matrix_Multiply(h2, 1, 3, B1, 3, Temp);
        Matrix_Multiply(Temp, 1, 3, h2, 1, Temp);
        //printf("ramda:%f\n", sqrt(Temp[0]));
    }

    {
        memcpy(H, oReport_13.m_Modal, 3 * 3 * sizeof(_T));
        //Test_H(pPoint_2D_1, pPoint_2D_2, H, iCount);
        //Disp(H, 3, 3, "H");
        //验算h1' * K'(-1) * K(-1) * h1 = ramda
        _T h1[3] = { H[0],H[3],H[6] },
            h2[3] = { H[1],H[4],H[7] };

        _T K_Inv[3 * 3], K_Inv_Transpose[3 * 3];
        _T B1[3 * 3], Temp[4];
        Get_Inv_Matrix_Row_Op_2(K, K_Inv, 3, &iResult);
        Matrix_Transpose(K_Inv, 3, 3, K_Inv_Transpose);
        Matrix_Multiply(K_Inv_Transpose, 3, 3, K_Inv, 3, B1);
        Disp(B1, 3, 3, "B1");
        Matrix_Multiply(h1, 1, 3, B1, 3, Temp);
        Matrix_Multiply(Temp, 1, 3, h1, 1, Temp);
        //printf("ramda:%f\n", sqrt(Temp[0]));

        Matrix_Multiply(h2, 1, 3, B1, 3, Temp);
        Matrix_Multiply(Temp, 1, 3, h2, 1, Temp);
        //printf("ramda:%f\n", sqrt(Temp[0]));
    }

    {
        memcpy(H, oReport_23.m_Modal, 3 * 3 * sizeof(_T));
        //Test_H(pPoint_2D_1, pPoint_2D_2, H, iCount);
        //Disp(H, 3, 3, "H");
        //验算h1' * K'(-1) * K(-1) * h1 = ramda
        _T h1[3] = { H[0],H[3],H[6] },
            h2[3] = { H[1],H[4],H[7] };

        _T K_Inv[3 * 3], K_Inv_Transpose[3 * 3];
        _T B1[3 * 3], Temp[4];
        Get_Inv_Matrix_Row_Op_2(K, K_Inv, 3, &iResult);
        Matrix_Transpose(K_Inv, 3, 3, K_Inv_Transpose);
        Matrix_Multiply(K_Inv_Transpose, 3, 3, K_Inv, 3, B1);
        //Disp(B1, 3, 3, "B1");
        Matrix_Multiply(h1, 1, 3, B1, 3, Temp);
        Matrix_Multiply(Temp, 1, 3, h1, 1, Temp);
        //printf("ramda:%f\n", sqrt(Temp[0]));

        Matrix_Multiply(h2, 1, 3, B1, 3, Temp);
        Matrix_Multiply(Temp, 1, 3, h2, 1, Temp);
        //printf("ramda:%f\n", sqrt(Temp[0]));
    }

    //解一个B出来看看
    {
        _T A[6 * 6], b[6 * 5];    //用于求解方程 A*b = 0
        Zhang_Get_A_2_row((_T*)oReport_12.m_Modal, &A[0]);
        Zhang_Get_A_2_row((_T*)oReport_13.m_Modal, &A[12]);
        Zhang_Get_A_2_row((_T*)oReport_23.m_Modal, &A[24]);
        Disp(A, 6, 6, "A");
        Solve_Linear_Contradictory(A, 6, 6, (_T*)NULL, b, &iResult);
        Test_Linear(A, 6, (_T*)NULL, b);
        _T B[3 * 3] = { b[0], b[1],b[3],
                b[1],b[2],b[4],
                b[3],b[4],b[5] };
        
        //Disp(B, 3, 3, "B");
        //printf("%f\n", fGet_Mod(b,6));
        //不妨验算以下B矩阵
        _T H1[3] = { H[0],H[3],H[6] },
            H2[3] = { H[1],H[4],H[7] };
        _T Temp[4];
        memcpy(H, oReport_12.m_Modal, 3 * 3 * sizeof(_T));
        Matrix_Multiply(B, 3, 3, H1, 1, Temp);
        Matrix_Multiply(H1, 1, 3, Temp, 1, Temp);
        printf("%0.20f\n", Temp[0]);

        Matrix_Multiply(B, 3, 3, H2, 1, Temp);
        Matrix_Multiply(H2, 1, 3, Temp, 1, Temp);
        printf("%0.20f\n", Temp[0]);

        memcpy(H, oReport_13.m_Modal, 3 * 3 * sizeof(_T));
        Matrix_Multiply(B, 3, 3, H1, 1, Temp);
        Matrix_Multiply(H1, 1, 3, Temp, 1, Temp);
        printf("%0.20f\n", Temp[0]);

        Matrix_Multiply(B, 3, 3, H2, 1, Temp);
        Matrix_Multiply(H2, 1, 3, Temp, 1, Temp);
        printf("%0.20f\n", Temp[0]);

        memcpy(H, oReport_23.m_Modal, 3 * 3 * sizeof(_T));
        Matrix_Multiply(B, 3, 3, H1, 1, Temp);
        Matrix_Multiply(H1, 1, 3, Temp, 1, Temp);
        printf("%0.20f\n", Temp[0]);

        Matrix_Multiply(B, 3, 3, H2, 1, Temp);
        Matrix_Multiply(H2, 1, 3, Temp, 1, Temp);
        printf("%0.20f\n", Temp[0]);

        _T cx, cy, fx, fy, ramda,gama;
        cy = (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0] * B[1 * 3 + 2]) / (B[0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]);
        ramda = B[2 * 3 + 2] - (B[0 * 3 + 2] * B[0 * 3 + 2] + cy * (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0] * B[1 * 3 + 2])) / B[0];
        fx = sqrt(ramda / B[0]);
        fy = sqrt((ramda * B[0]) / (B[0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]));
        gama = -B[0 * 3 + 1] * fx * fx * fy / ramda;
        cx = gama * cy / fx - B[0 * 3 + 2] * fx * fx / ramda;
        printf("%f\n", sqrt(1.f / B[0]));
    }

    bSave_PLY("c:\\tmp\\1.ply", oPC);
    Free_Point_Cloud(&oPC);
    Free(pPoint_3D_1);
    return;
}
template<typename _T>void Project(_T Point_3D[][3], int iCount, _T T[4 * 4], _T K[3 * 3], _T Point_2D[][2])
{//将空间点按照位姿，内参投影到像素平面上
    _T Temp[4] = { 0,0,0,1.f };
    _T R[3 * 3], t[3];
    Get_R_t(T, R, t);
    for (int i = 0; i < iCount; i++)
    {
        memcpy(Temp, Point_3D[i], 3 * sizeof(_T));
        Matrix_Multiply(R, 3, 3, Temp, 1, Temp);
        Vector_Add(Temp, t, 3, Temp);
        Matrix_Multiply(K, 3, 3, Temp, 1, Temp);
        Point_2D[i][0] = Temp[0] / Temp[2];
        Point_2D[i][1] = Temp[1] / Temp[2];
    }
    return;
}
void Test_4()
{//用更真实的空间点进行投影，上一个例子还是好像有问题
    typedef double _T;
    Point_Cloud<_T> oPC;
    _T(*pPoint_3D)[3];    //只有一组空间点
    _T(*pPoint_2D_1)[2],
        (*pPoint_2D_2)[2],
        (*pPoint_2D_3)[2];  //像素平面点

    int i, iResult, iCount = 110;
    _T K[3 * 3] = { 640,0,640,0,640,360,0,0,1 };
    _T T[4 * 4];

    pPoint_2D_1 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
    pPoint_2D_2 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));
    pPoint_2D_3 = (_T(*)[2])pMalloc(iCount * 2 * sizeof(_T));

    //生成空间面的所有点
    Gen_Plane(1.f, 2.f, 3.f, -20.f, iCount, &pPoint_3D);
    Init_Point_Cloud(&oPC, 10000, 1);
    Add_Point(&oPC, pPoint_3D, iCount, 255, 0, 0);

    //投影到相机1上
    for (i = 0; i < iCount; i++)
    {
        _T Temp[3];
        Matrix_Multiply(K, 3, 3, pPoint_3D[i], 1, Temp);
        Temp[0] /= Temp[2], Temp[1] /= Temp[2], Temp[2] = 1.f;
        
        pPoint_2D_1[i][0] = Temp[0];
        pPoint_2D_1[i][1] = Temp[1];
        Draw_Point(&oPC, Temp[0], Temp[1], Temp[2], 255, 0, 0);
    }

    //投影到相机2上
    {   //搞个旋转向量
        _T R[3 * 3], t[3] = { 0.3,0.5,10 };  //位移
        _T V[4] = { 1,1,1 };    //绕这个轴转
        Normalize(V, 3, V);
        V[3] = PI / 30;         //旋转 1/10 pi
        Rotation_Vector_4_2_Matrix(V, R);
        Gen_Homo_Matrix(R, t, T);
        Project(pPoint_3D, iCount, T, K, pPoint_2D_1);
    }
    
    //投影到相机2
    {   //搞个旋转向量
        _T R[3 * 3], t[3] = { 0.3,0.5,10 };  //位移
        _T V[4] = { 1,1,1 };    //绕这个轴转
        Normalize(V, 3, V);
        V[3] = PI / 30*2;         //旋转 1/10 pi
        Rotation_Vector_4_2_Matrix(V, R);
        Gen_Homo_Matrix(R, t, T);
        Project(pPoint_3D, iCount, T, K, pPoint_2D_2);
        //Disp((_T*)pPoint_2D_2, iCount, 2, "Point_2D_2");
    }

    //投影到相机2
    {   //搞个旋转向量
        _T R[3 * 3], t[3] = { 0.3,0.5,10 };  //位移
        _T V[4] = { 1,1,1 };    //绕这个轴转
        Normalize(V, 3, V);
        V[3] = PI / 30 * 3;         //旋转 1/10 pi
        Rotation_Vector_4_2_Matrix(V, R);
        Gen_Homo_Matrix(R, t, T);
        Project(pPoint_3D, iCount, T, K, pPoint_2D_3);
        Disp((_T*)pPoint_2D_3, iCount, 2, "Point_2D_3");
    }
    Ransac_Report oReport_12, oReport_13, oReport_23;
    Ransac_Estimate_H(pPoint_2D_1, pPoint_2D_2, iCount, &oReport_12);
    Ransac_Estimate_H(pPoint_2D_1, pPoint_2D_3, iCount, &oReport_13);
    Ransac_Estimate_H(pPoint_2D_2, pPoint_2D_3, iCount, &oReport_23);
      
    
    //Matrix_Multiply((_T*)oReport_12.m_Modal, 3, 3, 1.f / ((_T*)oReport_12.m_Modal)[8], (_T*)oReport_12.m_Modal);
    //Matrix_Multiply((_T*)oReport_13.m_Modal, 3, 3, 1.f / ((_T*)oReport_13.m_Modal)[8], (_T*)oReport_13.m_Modal);
    //Matrix_Multiply((_T*)oReport_23.m_Modal, 3, 3, 1.f / ((_T*)oReport_23.m_Modal)[8], (_T*)oReport_23.m_Modal);
    //Disp((_T*)oReport_12.m_Modal, 3, 3, "H_12");
    //Disp((_T*)oReport_13.m_Modal, 3, 3, "H_13");
    //Disp((_T*)oReport_23.m_Modal, 3, 3, "H_23");*/

    //解一个B出来看看
    {
        _T A[6 * 6], b[6 * 5];    //用于求解方程 A*b = 0
        Zhang_Get_A_2_row((_T*)oReport_12.m_Modal, &A[0]);
        Zhang_Get_A_2_row((_T*)oReport_13.m_Modal, &A[12]);
        Zhang_Get_A_2_row((_T*)oReport_23.m_Modal, &A[24]);
        Disp(A, 6, 6, "A");
        Solve_Linear_Contradictory(A, 6, 6, (_T*)NULL, b, &iResult);
        Test_Linear(A, 6, (_T*)NULL, b);
        _T B[3 * 3] = { b[0], b[1],b[3],
                b[1],b[2],b[4],
                b[3],b[4],b[5] };
        Disp(B, 3, 3, "B");
        _T cx, cy, fx, fy, ramda, gama;
        cy = (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0] * B[1 * 3 + 2]) / (B[0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]);
        ramda = B[2 * 3 + 2] - (B[0 * 3 + 2] * B[0 * 3 + 2] + cy * (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0] * B[1 * 3 + 2])) / B[0];
        fx = sqrt(ramda / B[0]);
        fy = sqrt((ramda * B[0]) / (B[0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]));
        gama = -B[0 * 3 + 1] * fx * fx * fy / ramda;
        cx = gama * cy / fx - B[0 * 3 + 2] * fx * fx / ramda;
        //printf("%f\n", sqrt(1.f / B[0]));
    }

    {
        _T K_Inv[3 * 3], K_Inv_Transpose[3 * 3];
        _T B1[3 * 3];
        Get_Inv_Matrix_Row_Op_2(K, K_Inv, 3, &iResult);
        Matrix_Transpose(K_Inv, 3, 3, K_Inv_Transpose);
        Matrix_Multiply(K_Inv_Transpose, 3, 3, K_Inv, 3, B1);
        Disp(B1, 3, 3, "B1");
    }
    return;
}

void Test_5()
{//屡败屡战，把example的数据抽出来搞搞, 此处发掘问题不在后面的求解，而在H矩阵的计算
    typedef double _T;
    _T H1[] = { -393.1960644063013, 227.6484258591809, -174.7833244352132,
                 166.6938928017319, 588.1836408105223, -226.5913216732196,
                 0.3788517596451904, 0.2468587951992746, -0.6306119624241887 };
    _T H2[] = { -376.2715135700373, 278.7691844835609, -142.4570273211084,
                 193.1239328397774, 574.4161108164174, -203.367138194009,
                 0.3865929625261305, 0.2857410571812361, -0.6317350285793172    };
    _T H3[] = { -579.7173352148884, 99.78270767088762, -121.2449611511642,
                 -81.63228242769272, 615.9653333335006, -114.5078510639517,
                 -0.02235097890813136, 0.5130743349722929, -0.6095681577276318  };
    _T A[6 * 6], b[6 * 5];    //用于求解方程 A*b = 0
    int iResult;
    Zhang_Get_A_2_row(H1, &A[0]);
    Zhang_Get_A_2_row(H2, &A[12]);
    Zhang_Get_A_2_row(H3, &A[24]);
    //Disp(A, 6, 6, "A");
    Solve_Linear_Contradictory(A, 6, 6, (_T*)NULL, b, &iResult);
    Test_Linear(A, 6, (_T*)NULL, b);
    _T B[3 * 3] = { b[0], b[1],b[3],
            b[1],b[2],b[4],
            b[3],b[4],b[5] };
    Disp(b, 1, 6, "b");
    //Disp(B, 3, 3, "B");
    _T cx, cy, fx, fy, ramda, gama;
    cy = (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0] * B[1 * 3 + 2]) / (B[0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]);
    ramda = B[2 * 3 + 2] - (B[0 * 3 + 2] * B[0 * 3 + 2] + cy * (B[0 * 3 + 1] * B[0 * 3 + 2] - B[0] * B[1 * 3 + 2])) / B[0];
    fx = sqrt(ramda / B[0]);
    fy = sqrt((ramda * B[0]) / (B[0] * B[1 * 3 + 1] - B[0 * 3 + 1] * B[0 * 3 + 1]));
    gama = -B[0 * 3 + 1] * fx * fx * fy / ramda;
    cx = gama * cy / fx - B[0 * 3 + 2] * fx * fx / ramda;
    return;
}
void Test_6()
{//这里搞明白H矩阵的scale怎么来的
    typedef double _T;
    FILE* pFile = fopen("c:\\tmp\\point_2d_12,bin", "rb");
    const int iCount = 88,iPoint_Count_h = 8,   //每列多少点
        iPoint_Count_w = 11;                    //每行多少点
    const _T sqr_size = 0.02;               //这个是自定义，是棋盘格子的边长
    _T Point_3D[iPoint_Count_h * iPoint_Count_w][2];

    int x, y, iPos;
    for (iPos=y = 0; y < iPoint_Count_h; y++)
    {
        for (x = 0; x < iPoint_Count_w; x++,iPos++)
        {
            Point_3D[iPos][0] = x * sqr_size;
            Point_3D[iPos][1] = y * sqr_size;
        }
    }
    Disp((_T*)Point_3D, iCount, 2);

    return;
}
void Long_Task_Test()
{
    unsigned long long iTotal = 0;
    for (;;)
        iTotal++;

}
void Test_7()
{//测一下多线程性能
#define THREAD_COUNT 8
    int i;
    thread* Thread_Arr[THREAD_COUNT];
    for (i = 0; i < THREAD_COUNT; i++)
        Thread_Arr[i] = new thread(Long_Task_Test);
    for (i = 0; i < THREAD_COUNT; i++)
    {
        Thread_Arr[i]->join();
        delete Thread_Arr[i];
    }
#undef THREAD_COUNT
}

void Distor_Test()
{//尝试观察一下
    typedef double _T;
    _T K[] = { 6359.84906057630, 0.499556244775606, 2470.20710871242,
        0, 6359.12264731417, 2550.99677756024,
        0, 0, 1 };

    //此处的排列和opencv不一样，注意数据顺序
    //_T Dist_Coeff[5] = {-6.26548780e-02, 1.33740651e-01, -6.15227977e-05,-6.27767225e-03, 2.92800336e-04 };
    _T Dist_Coeff[5] = {-6.26548780e-02, 1.33740651e-01,2.92800336e-04, -6.15227977e-05,-6.27767225e-03 };
    //造一个棋盘空间点
    const int iHeight = 5, iWidth = 5;
    _T Point_3D[iHeight *iWidth][4];
    double Grid_Width = 30, Grid_Height = 30;//
    int iCount = 0;
    for (int y = 0; y < iHeight; ++y)
    {
        for (int x = 0; x < iWidth; ++x,iCount++)
        {
            Point_3D[iCount][0] = x * Grid_Width;
            Point_3D[iCount][1]= y * Grid_Height;
            Point_3D[iCount][2] = 0;
            Point_3D[iCount][3] = 1;
        }
    }

    //造一个位姿，这个位姿在(0,0,t_z)处
    const int t_z =10000;
    _T T[4 * 4], R[3 * 3];
    _T V[4] = { 0,1,0,0 };    //不旋转
    _T t[3] = { 0,0,t_z };
    Rotation_Vector_4_2_Matrix(V, R);
    Gen_Homo_Matrix(R, t, T);

    _T uv[iWidth * iHeight][2];
    int i;

    for (i = 0; i < iCount; i++)
    //i = 6;
    {
        _T TP[4];   //, Point_2D[3];
        Matrix_Multiply(T,4,4, Point_3D[i],1, TP);

        //TP[0] /= TP[2], TP[1] /= TP[2];
        //Matrix_Multiply(K, 3, 3, TP,1,Point_2D);

        Undistort_uv(TP, K, Dist_Coeff, 5, uv[i]);
        //printf("i:%d\n", i);
        Disp(uv[i], 1, 2);
    }
    return;
}
void PnP_Pose_Point_Test()
{//Pose + Point一起优化试验。由于前面所有的LM都不能完全跟足Ceres，这次尝试一毛一样
    typedef double _T;   

    //第一部分，造数据
    const int iCamera_Count = 16,    //iCamera_Count=1, iPoint_Count=100时收敛奇怪，需要重搞LM
        iPoint_Count = 500,
        iObservation_Count = iPoint_Count*iCamera_Count;
    int i;

    _T(*pTrue_Point_3D)[3] = //真实点
        (_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));
    _T(*pNoisy_Point_3D)[3]=//真实点加上噪声
        (_T(*)[3])pMalloc(iPoint_Count * 3 * sizeof(_T));
    
    for (i = 0; i < iPoint_Count; i++)
    {
        const int Recip = 1234567;
        _T Point_3D[] = { ((_T)((int)iGet_Random_No() % Recip) / Recip - 0.5f) * 3,
            (_T)((int)iGet_Random_No() % Recip) / Recip - 0.5,
            (_T)((int)iGet_Random_No() % Recip) / Recip + 3 };
        memcpy(pTrue_Point_3D[i], Point_3D, 3 * sizeof(_T));
    }

    _T(*pCamera_4x4)[4 * 4] =   //相机外参的T矩阵
        (_T(*)[4 * 4])pMalloc(iCamera_Count * 4*4 * sizeof(_T));
    _T K[3 * 3] = { 1000,0,320, 0,1000,240,0,0,1 };  //相机内参
    for (i = 0; i < iCamera_Count; i++)
    {//造位姿
        _T Camera_6[] = { 0,0,0,i * 0.04 - 1.f,0,0 };
        _T Rotation_Vector[6], R[3 * 3];
        Rotation_Vector_3_2_4(Camera_6, Rotation_Vector);
        Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
        Gen_Homo_Matrix(R, &Camera_6[3], pCamera_4x4[i]);
        //注意，此处不能用se3_2_SE3，因为此Ksi不是彼ksi
    }

    //_T(*pPoint_2D_Ref)[iCamera_Count][2] = //空间点在每个相机像素平面上的投影
    //    (_T(*)[iCamera_Count][2])pMalloc(iPoint_Count * iCamera_Count * 2 * sizeof(_T));
    Point_2D<_T>* pObservation_2D = (Point_2D<_T>*)pMalloc(iPoint_Count * iCamera_Count * sizeof(Point_2D<_T>));
    for (i = 0; i < iPoint_Count; i++)
    {
        const int Recip = 1234567;
        pNoisy_Point_3D[i][0] = pTrue_Point_3D[i][0]+ (double)((int)iGet_Random_No() % Recip) / Recip;
        pNoisy_Point_3D[i][1] = pTrue_Point_3D[i][1]+(double)((int)iGet_Random_No() % Recip) / Recip;
        pNoisy_Point_3D[i][2] = pTrue_Point_3D[i][2]+(double)((int)iGet_Random_No() % Recip) / Recip;
        //Disp(pNoisy_Point_3D[i], 1, 3, "Point_3D");
        for (int j = 0; j < iCamera_Count; j++)
        {           
            _T Point_4D[] = { pTrue_Point_3D[i][0],pTrue_Point_3D[i][1],pTrue_Point_3D[i][2],1 };  
            _T Point_3D[4], Point_2D[2];

            Matrix_Multiply(pCamera_4x4[j], 4, 4, Point_4D, 1, Point_3D);
            Matrix_Multiply(K, 3, 3, Point_3D, 1, Point_3D);
            Point_2D[0] = Point_3D[0] / Point_3D[2];
            Point_2D[1] = Point_3D[1] / Point_3D[2];

            const int Recip = 1234567;
            Point_2D[0] += (double)((int)iGet_Random_No() % Recip) / Recip,
                Point_2D[1] += (double)((int)iGet_Random_No() % Recip) / Recip;
            //Disp(Point_2D, 1, 2, "Point_2D");
            pObservation_2D[i*iCamera_Count+j].m_iCamera_Index = j;
            pObservation_2D[i*iCamera_Count+j].m_iPoint_Index = i;
            memcpy(pObservation_2D[i*iCamera_Count+j].m_Pos, Point_2D, 2 * sizeof(_T));
        }
    }
    //至此，数据已经做完，造数据部分和纯位姿估计完全一致
    _T Intrinsic[4] = { K[0],K[4],K[2],K[5] };
    PnP_Pose_Point(pCamera_4x4, iCamera_Count,Intrinsic, pNoisy_Point_3D, iPoint_Count, pObservation_2D, iObservation_Count);
    
    if (pTrue_Point_3D)Free(pTrue_Point_3D);
    if (pNoisy_Point_3D)Free(pNoisy_Point_3D);
    if (pCamera_4x4)Free(pCamera_4x4);
    if (pObservation_2D)Free(pObservation_2D);
    return;
}



//int main()
//{
//    Init_Env();
//    //PnP_Pose_Point_Test();
//    //Distor_Test();
//    //Zhang_Test_1();
//    Chess_Board_Test();
//	//Test_Main();
//    IMU_Test_Main();    //IMU操作
//    Free_Env();
//#ifdef WIN32
//    _CrtDumpMemoryLeaks();
//#endif
//}
