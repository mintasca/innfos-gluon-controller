#ifndef MATH_FUN_H
#define MATH_FUN_H

int inverse_matrix(double *a, int n);
void mul_matrix(double *op1,double *op2,double *out,int m,int n,int l);
void sub_vector(double v1[],double v2[],double v[],int dim);
void add_vector(double v1[],double v2[],double v[],int dim);
double dot_product(double v1[],double v2[],int dim);
double norm(double v[],int dim);
void cross_3d(double v1[],double v2[],double v[]);
void scaling(double v[],double scale,int dim);
void MatrixToUnitQuaternion(double matrix[4][4],double pos[7]);
void UnitQuaternionToMatrix(double matrix[4][4],double pos[7]);

bool is_nan(double val);
void print_matrix_4x4(double matrix[4][4]);
void print_array(double pos[],int n);
void PrintPositionDegree(double pos[],int n);
void CalcTransmatrix(double matrix[4][4],int axis,double dertS);
void MulMatrix3x3(double out[][3],double op1[][3],double op2[][3]);
void MatrixLeftMul(double matOut[4][4],double matTransf[4][4],double matIn[4][4]);


#endif
