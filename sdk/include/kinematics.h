#ifndef KINEMATICS_H
#define KINEMATICS_H

void fk_ur(const double q[6], double A6[4][4],double dh[]);
int ik_ur(double q[],double matrix[4][4],double dh[]);
void calculate_jacobian_ur(const double in1[6], const double in2[3], const
  double in3[6], double JExpress[36]);


#endif
