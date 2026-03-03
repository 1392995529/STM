#include "mc_type.h"

typedef struct 
{
	int row,line;		//line为行,row为列
	double *data;
}Matrix;

typedef struct
{
  int16_t Iqav;
  int16_t Iq;
  int16_t uqav;
  int16_t uq;
  int16_t hVqdLowPassFilterBW; 
}RLS_Handle_t;

void InitMatrix(Matrix *matrix,int row,int line);		    //初始化矩阵
void ValueMatrix(Matrix *matrix,double *array);			    //给一个矩阵赋值
int SizeMatrix(Matrix *matrix);					    //获得一个矩阵的大小
double Convert_to_number(Matrix *matrix);                           //将单个元素矩阵转换为整数形式


//矩阵的基本运算
void AddMatrix(Matrix *matrix_C,Matrix *matrix_A,Matrix *matrix_B);		//矩阵的加法
void SubMatrix(Matrix *matrix_C,Matrix *matrix_A,Matrix *matrix_B);             //矩阵的减法
void MulMatrix(Matrix *matrix_C,Matrix *matrix_A,Matrix *matrix_B);		//矩阵的乘法

void Parameter_identification(RLS_Handle_t * pHandle);
void Initialization_matrix(void);

extern RLS_Handle_t  RLS;