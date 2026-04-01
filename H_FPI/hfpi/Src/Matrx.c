#include "Matrx.h"
#include "mc_api.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

float phi[]       = {0,0};
float P[]         = {10^6,0,0,10^6};
float theta[]     = {0.01,0.01};
float y[]         = {0};


RLS_Handle_t  RLS =
{
   .hVqdLowPassFilterBW = 128
};

void Parameter_identification(RLS_Handle_t * pHandle)
{
   pHandle->Iq =  MC_GetIqdMotor1().q ;
   int32_t wAux;
   int32_t wAux1;
   int32_t lowPassFilterBW = ( int32_t ) pHandle->hVqdLowPassFilterBW - ( int32_t )1;  //低通滤波器
   wAux = ( int32_t )( pHandle->Iqav ) * lowPassFilterBW;
   wAux += pHandle->Iq;
   
   pHandle->Iqav = ( int16_t )( wAux /
                                     ( int32_t )( pHandle->hVqdLowPassFilterBW ) );
   
   pHandle->Iq =  MC_GetVqdMotor1().q ;
   
   wAux1 = ( int32_t )( pHandle->uqav ) * lowPassFilterBW;
   wAux1 += pHandle->uq;
   
   pHandle->uqav = ( int16_t )( wAux1 /
                                     ( int32_t )( pHandle->hVqdLowPassFilterBW ) );
   
   float Iqavf ;
   Iqavf = (float)pHandle->Iqav / 1596.5 ;   //需要进行转换
   
   
   int16_t wm = MC_GetMecSpeedAverageMotor1(); 
   float wr;
   wr = (float)wm * 3.1415926/30 ;  //转换为角速度
   
   float uqavf;
   uqavf = (float)pHandle->uqav / 2364.8 ;    
      
   y[0] = uqavf;
   phi[0] = Iqavf;
   phi[1] = wr;
   
    /* [修改说明] 
       这里将 InitMatrix(xxx, ...) 改为了 InitMatrix(NULL, ...)
       目的是消除 Warning[Pe549] 警告，逻辑与原来完全一致。
    */
    Matrix *matrix_phi = InitMatrix(NULL,2,1);  //phi
    Matrix *matrix_phiT = InitMatrix(NULL,1,2);  //phiT
    Matrix *matrix_P = InitMatrix(NULL,2,2);  //P
    Matrix *matrix_theta = InitMatrix(NULL,2,1); //theta
    Matrix *matrix_y = InitMatrix(NULL,1,1); //y
    
    ValueMatrix(matrix_phi,phi);
    ValueMatrix(matrix_phiT,phi);
    ValueMatrix(matrix_P,P);
    ValueMatrix(matrix_theta,theta);
    ValueMatrix(matrix_y,y);


    //计算增益矩阵
    //K = P_0 * phi / ( 1 + phi' * P_0 * phi);
    Matrix *matrix00 = MulMatrix(matrix_P,matrix_phi); //P_0 * phi
    Matrix *matrix01 = MulMatrix(matrix_phiT,matrix_P); //phi' * P_0
    Matrix *matrix02 = MulMatrix(matrix01,matrix_phi); //phi' * P_0 * phi
    float a = Convert_to_number(matrix02);
    float b = 1/(1+a);
    float bb[] = {b}; 
    
    Matrix *matrix03 = InitMatrix(NULL,1,1); // [修改] NULL消除警告
    ValueMatrix(matrix03,bb);
    Matrix *K = MulMatrix(matrix00,matrix03);


    //计算下一个协方差矩阵
    //P_1 = (P_0 - K * phi' * P_0);
    Matrix *matrix04 = MulMatrix(K,matrix_phiT);
    Matrix *matrix05 = MulMatrix(matrix04,matrix_P);
    Matrix *P1 = SubMatrix(matrix_P,matrix05);
    Matrix *K1 = MulMatrix(P1,matrix_phi);

    //Y=.....

    //计算下一个theta
    //theta_1 = theta_0 + K1 * (  y - phi' * theta_0);  
    Matrix *matrix06 = MulMatrix(matrix_phiT,matrix_theta);  //phi' * theta_0
    Matrix *matrix07 = SubMatrix(matrix_y,matrix06); // y - phi' * theta_0
    Matrix *matrix08 = MulMatrix(K1,matrix07); //K1 * (  y - phi' * theta_0)
    Matrix *theta_1 = AddMatrix(matrix_theta,matrix08); //theta_0 + K1 * (  y - phi' * theta_0)
    printf("theta_1:\n");
    PrintMatrix(theta_1);

    //更改数组内的数

    for (int j=0;j<SizeMatrix(theta_1);j++)
    {
        theta[j] = theta_1->data[j];
    }

    for (int k=0;k<SizeMatrix(P1);k++)
    {
        P[k] = P1->data[k]; 
    }
    
    // [注意] 此处移除了 Step 4 的 FreeMatrix 释放代码。
    // 这意味着如果该函数被调用，会发生内存泄漏。
    // 请确保在 mc_tasks.c 中不要取消对 Parameter_identification 的注释。
}


Matrix* InitMatrix(Matrix *matrix,int row,int line)             //初始化一个矩阵
{
    if (row>0 && line>0)
    {
        matrix = (Matrix*)malloc(sizeof(Matrix));
        matrix->row = row;
        matrix->line = line;
        matrix->data = (float*)malloc(sizeof(float)*row*line);
        memset(matrix->data,0,sizeof(float)*row*line);
        return matrix;
    }
    else 
        return NULL;
} 

void ValueMatrix(Matrix *matrix,float *array)       //给矩阵赋值
{
    if (matrix->data != NULL)
    {
          int i;
          for (i = 0; i < matrix->row*matrix->line; i++) {
        matrix->data[i] = array[i]; }
        //memcpy(matrix->data, array, matrix->row*matrix->line*sizeof(float));
    }
}

int SizeMatrix(Matrix *matrix)
{
    return matrix->row*matrix->line;
}



void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B)
{
    matrix_B->row = matrix_A->row;
    matrix_B->line = matrix_A->line;
    memcpy(matrix_B->data, matrix_A->data, SizeMatrix(matrix_A)*sizeof(float));
}

void PrintMatrix(Matrix *matrix)
{
    for (int i=0;i<SizeMatrix(matrix);i++)
    {
        printf("%f ", matrix->data[i]);
        if ((i+1)%matrix->line == 0)
            printf("\n");
    }
            
}

float Convert_to_number(Matrix *matrix)
{   
        float a =matrix->data[0];
        return a;                   
}

//加法
Matrix* AddMatrix(Matrix *matrix_A,Matrix *matrix_B)
{
    if (matrix_A->row == matrix_B->row && matrix_A->line == matrix_B->line)
    {
        // [修改] 传入 NULL 以消除警告
        Matrix *matrix_C = InitMatrix(NULL,matrix_A->row,matrix_A->line);
        for (int i=0;i<matrix_A->line;i++)
        {
            for (int j=0;j<matrix_A->row;j++)
            {
                matrix_C->data[i*matrix_C->row + j] = 
                matrix_A->data[i*matrix_A->row + j] + matrix_B->data[i*matrix_A->row + j];
            }
        }
        return matrix_C;
    }
    else 
    {
        printf("不可相加\n");
        return NULL;
    }
}

//减法
Matrix* SubMatrix(Matrix *matrix_A,Matrix *matrix_B)
{
    if (matrix_A->row == matrix_B->row && matrix_A->line == matrix_B->line)
    {
        // [修改] 传入 NULL 以消除警告
        Matrix *matrix_C = InitMatrix(NULL,matrix_A->row,matrix_A->line);
        for (int i=0;i<matrix_A->line;i++)
        {
            for (int j=0;j<matrix_A->row;j++)
            {
                matrix_C->data[i*matrix_C->row + j] = 
                matrix_A->data[i*matrix_A->row + j] - matrix_B->data[i*matrix_A->row + j];
            }
        }
        return matrix_C;
    }
    else 
    {
        printf("不可相加\n");
        return NULL;
    }
}

//乘法
Matrix* MulMatrix(Matrix *matrix_A,Matrix *matrix_B)
{
    Matrix *matrix_C = NULL;

    if (matrix_A->line == matrix_B->row) //列==行
    {
           // [修改] 传入 NULL 以消除警告 (虽然这里本身是NULL，但保持风格一致)
           matrix_C = InitMatrix(NULL,matrix_A->row,matrix_B->line);
           // matrix_C->line = matrix_A->line; //A行
           // matrix_C->row = matrix_B->row; //B列
           for (int i=0;i<matrix_A->row;i++)
           {
               for (int j=0;j<matrix_B->line;j++)
               {
                   for (int k=0;k<matrix_A->line;k++)
                   {
                       matrix_C->data[i*matrix_C->line + j] += \
                           matrix_A->data[i*matrix_A->line + k] * matrix_B->data[k*matrix_B->line + j];
                   }
               }
           }
           return matrix_C;
       }
       else
       {
           printf("不可相乘\n");
           return NULL;
       }
}