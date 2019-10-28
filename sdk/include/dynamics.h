#ifndef DYNAMICS_H
#define DYNAMICS_H

void InverserDynamicsUr(double a[6],double d[6],double q[],double qd[],double qdd[],double tau[]);

void AddFrictionMoment(double qd[],double tau[]);
#endif
