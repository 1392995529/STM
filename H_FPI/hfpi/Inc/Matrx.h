#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mc_type.h"

typedef struct 
{
	int row,line;		//line为行,row为列
	float *data;
}Matrix;



typedef struct
{
  int16_t Iqav;
  int16_t Iq;
  int16_t uqav;
  int16_t uq;
  int16_t hVqdLowPassFilterBW; 
}RLS_Handle_t;

Matrix* InitMatrix(Matrix *matrix,int row,int line);		//初始化矩阵
void ValueMatrix(Matrix *matrix,float *array);				//给一个矩阵赋值
int SizeMatrix(Matrix *matrix);								//获得一个矩阵的大小
void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B);		//复制一个矩阵的值
void PrintMatrix(Matrix *matrix);							//打印一个矩阵
float Convert_to_number(Matrix *matrix);
void Parameter_identification(RLS_Handle_t * pHandle);

//矩阵的基本运算
Matrix* AddMatrix(Matrix *matrix_A,Matrix *matrix_B);		//矩阵的加法
Matrix* SubMatrix(Matrix *matrix_A,Matrix *matrix_B);       //矩阵的减法
Matrix* MulMatrix(Matrix *matrix_A,Matrix *matrix_B);		//矩阵的乘法

extern RLS_Handle_t  RLS;