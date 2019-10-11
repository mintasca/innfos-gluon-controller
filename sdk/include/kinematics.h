#ifndef KINEMATICS_H
#define KINEMATICS_H

void fk_ur(const double q[6], double A6[4][4],double dh[]);
int ik_ur(double q[],double matrix[4][4],double dh[]);


#endif
