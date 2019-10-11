#ifndef DYNAMICS_H
#define DYNAMICS_H

void InverserDynamicsNE30(double q[],double qd[],double qdd[],double tau[]);

void AddFrictionMoment(double qd[],double tau[]);
#endif
