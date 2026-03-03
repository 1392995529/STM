#include "Matrx.h"
#include "mc_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mc_tasks.h"


double phi[]       = {0,0};
double P[]         = {1000000,0,0,1000000};
double theta[]     = {0.01,0.01};
double y[]         = {0};

Matrix Mphi = {0};
Matrix *matrix_phi = &Mphi;
Matrix MphiT= {0};
Matrix *matrix_phiT = &MphiT;
Matrix Mp = {0};
Matrix *matrix_P = &Mp;
Matrix Mtheta= {0};
Matrix *matrix_theta = &Mtheta;
Matrix My = {0};
Matrix *matrix_y = &My;
  
Matrix M00 = {0};
Matrix *matrix00 = &M00;
Matrix M01 = {0};
Matrix *matrix01 = &M01;
Matrix M02 = {0};
Matrix *matrix02 = &M02;
Matrix M03 = {0};
Matrix *matrix03 = &M03;
Matrix Mk = {0};
Matrix *K = &Mk;
   
Matrix M04 = {0};
Matrix *matrix04 = &M04;
Matrix M05 = {0};
Matrix *matrix05 = &M05;
Matrix Mp1 = {0};
Matrix *P1 = &Mp1;
Matrix Mk1 = {0};
Matrix *K1 = &Mk1;

Matrix M06 = {0};
Matrix *matrix06 = &M06;
Matrix M07 = {0};
Matrix *matrix07 = &M07;
Matrix M08 = {0};
Matrix *matrix08 = &M08;
Matrix Mtheta_1= {0} ;
Matrix *theta_1 = &Mtheta_1;

extern double R ;
extern double Fluxlinkage ;

RLS_Handle_t  RLS =
{
   .hVqdLowPassFilterBW = 128
};

void Initialization_matrix(void)
{ 
  InitMatrix(matrix_phi,2,1);     //phi
  InitMatrix(matrix_phiT,1,2);    //phiT
  InitMatrix(matrix_P,2,2);       //P
  InitMatrix(matrix_theta,2,1);   //theta
  InitMatrix(matrix_y,1,1);       //y
  InitMatrix(matrix00,2,1);       //P_0 * phi
  InitMatrix(matrix01,1,2);       //phiT * P_0
  InitMatrix(matrix02,1,1);       //phiT * P_0* phi
  InitMatrix(matrix03,1,1);       //1/( 1 + phiT * P_0 * phi)
  InitMatrix(K,2,1);              //P_0 * phi / ( 1 + phi' * P_0 * phi);  
  InitMatrix(matrix04,2,2);       //K * phiT
  InitMatrix(matrix05,2,2);       //K * phiT * P_0
  InitMatrix(P1,2,2);             //P_0 - K * phi' * P_0 = P1
  InitMatrix(K1,2,1);             //P1 * phi
  InitMatrix(matrix06,1,1);       //phiT * theta_0
  InitMatrix(matrix07,1,1);       //y - phiT * theta_0
  InitMatrix(matrix08,2,1);       //K1 * (y - phi' * theta_0)
  InitMatrix(theta_1,2,1);        //theta_0 + K1 * (y - phi' * theta_0)  
}

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
   
   pHandle->uq =  MC_GetVqdMotor1().q ;
   
   wAux1 = ( int32_t )( pHandle->uqav ) * lowPassFilterBW;
   wAux1 += pHandle->uq;
   
   pHandle->uqav = ( int16_t )( wAux1 /
                                      ( int32_t )( pHandle->hVqdLowPassFilterBW ) );
   
   double Iqavf ;
   Iqavf = (double)pHandle->Iqav / 1596.5 ;   //需要进行转换
   
   
   int16_t wm = MC_GetMecSpeedAverageMotor1(); 
   double wr;
   wr = (double)wm * 3.1415926/30 ;  //转换为角速度
   
   double uqavf;
   uqavf = (double)pHandle->uqav / 2364.8 ;    
      
   y[0] = uqavf;
   phi[0] = Iqavf;
   phi[1] = wr;
   
   ValueMatrix(matrix_phi,phi);
   ValueMatrix(matrix_phiT,phi);
   ValueMatrix(matrix_P,P);
   ValueMatrix(matrix_theta,theta);
   ValueMatrix(matrix_y,y);
	
		
   //计算增益矩阵
   //K = P_0 * phi / ( 1 + phi' * P_0 * phi);

   MulMatrix(matrix00,matrix_P,matrix_phi); //P_0 * phi
   MulMatrix(matrix01,matrix_phiT,matrix_P); //phi' * P_0
   MulMatrix(matrix02,matrix01,matrix_phi); //phi' * P_0 * phi
   double a = Convert_to_number(matrix02);
   double b = 1/(1+a);
   double bb[] = {b};    
   ValueMatrix(matrix03,bb);
   MulMatrix(K,matrix00,matrix03);
   
   

   //计算下一个协方差矩阵
   //P_1 = (P_0 - K * phi' * P_0);
   MulMatrix(matrix04,K,matrix_phiT);
   MulMatrix(matrix05,matrix04,matrix_P);
   SubMatrix(P1,matrix_P,matrix05);
   MulMatrix(K1,P1,matrix_phi);
   
   //计算下一个theta
   //theta_1 = theta_0 + K1 * (  y - phi' * theta_0);  
   MulMatrix(matrix06,matrix_phiT,matrix_theta);  //phi' * theta_0
   SubMatrix(matrix07,matrix_y,matrix06); // y - phi' * theta_0
   MulMatrix(matrix08,K1,matrix07); //K1 * (  y - phi' * theta_0)
   AddMatrix(theta_1,matrix_theta,matrix08); //theta_0 + K1 * (  y - phi' * theta_0)  

   //更改数组内的数
   for (int j=0;j<SizeMatrix(theta_1);j++)
   {
     theta[j] = theta_1->data[j];
   }

   for (int k=0;k<SizeMatrix(P1);k++)
   {
     P[k] = P1->data[k];	
   }
   R = theta_1->data[0];
   Fluxlinkage = theta_1->data[1];
   
	
}




void InitMatrix(Matrix *matrix,int row,int line)				//初始化一个矩阵
{
  if (row>0 && line>0)
  {
    matrix->row = row;
    matrix->line = line;
    matrix->data = (double*)malloc(sizeof(double)*row*line);
    memset(matrix->data,0,sizeof(double)*row*line);
  }
} 

void ValueMatrix(Matrix *matrix,double *array) 		//给矩阵赋值
{
  
    {
	int i;
        for (i = 0; i < matrix->row*matrix->line; i++) {
          matrix->data[i] = array[i];}
    }
  
}

int SizeMatrix(Matrix *matrix)
{
  return matrix->row*matrix->line;
}




double Convert_to_number(Matrix *matrix)
{	
  double a =matrix->data[0];
  return a;					
}


//加法
void AddMatrix(Matrix *matrix_C,Matrix *matrix_A,Matrix *matrix_B)
{
  if (matrix_A->row == matrix_B->row && matrix_A->line == matrix_B->line)
  {
    //InitMatrix(matrix_C,matrix_A->row,matrix_A->line);
    matrix_C->row = matrix_A->row;
    matrix_C->line = matrix_A->line;
    for (int i=0;i<matrix_A->line;i++)
    {
      for (int j=0;j<matrix_A->row;j++)
      {
        matrix_C->data[i*matrix_C->row + j] = 
          matrix_A->data[i*matrix_A->row + j] + matrix_B->data[i*matrix_A->row + j];
      }
    }
  }

}

//减法
void SubMatrix(Matrix *matrix_C,Matrix *matrix_A,Matrix *matrix_B)
{
  if (matrix_A->row == matrix_B->row && matrix_A->line == matrix_B->line)
  {
    //InitMatrix(matrix_C,matrix_A->row,matrix_A->line);
    matrix_C->row = matrix_A->row;
    matrix_C->line = matrix_A->line;
    for (int i=0;i<matrix_A->line;i++)
    {
      for (int j=0;j<matrix_A->row;j++)
      {
        matrix_C->data[i*matrix_C->row + j] = 
          matrix_A->data[i*matrix_A->row + j] - matrix_B->data[i*matrix_A->row + j];
      }
    }
  }

}

//乘法
void MulMatrix(Matrix *matrix_C,Matrix *matrix_A,Matrix *matrix_B)
{
  if (matrix_A->line == matrix_B->row) //列==行
  {
    //InitMatrix(matrix_C,matrix_A->row,matrix_B->line);
    matrix_C->row = matrix_A->row;
    matrix_C->line = matrix_B->line;
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
  }

}


