/* Produced by CVXGEN, 2013-01-17 22:52:37 +0000.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Psi_k[0])-rhs[1]*(params.Psi_k[3])-rhs[2]*(params.Psi_k[6])-rhs[3]*(params.Psi_k[9])-rhs[4]*(params.Psi_k[12])-rhs[5]*(params.Psi_k[15])-rhs[6]*(-1);
  lhs[1] = -rhs[0]*(params.Psi_k[1])-rhs[1]*(params.Psi_k[4])-rhs[2]*(params.Psi_k[7])-rhs[3]*(params.Psi_k[10])-rhs[4]*(params.Psi_k[13])-rhs[5]*(params.Psi_k[16])-rhs[7]*(-1);
  lhs[2] = -rhs[0]*(params.Psi_k[2])-rhs[1]*(params.Psi_k[5])-rhs[2]*(params.Psi_k[8])-rhs[3]*(params.Psi_k[11])-rhs[4]*(params.Psi_k[14])-rhs[5]*(params.Psi_k[17])-rhs[8]*(-1);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Psi_k[0])-rhs[1]*(params.Psi_k[1])-rhs[2]*(params.Psi_k[2]);
  lhs[1] = -rhs[0]*(params.Psi_k[3])-rhs[1]*(params.Psi_k[4])-rhs[2]*(params.Psi_k[5]);
  lhs[2] = -rhs[0]*(params.Psi_k[6])-rhs[1]*(params.Psi_k[7])-rhs[2]*(params.Psi_k[8]);
  lhs[3] = -rhs[0]*(params.Psi_k[9])-rhs[1]*(params.Psi_k[10])-rhs[2]*(params.Psi_k[11]);
  lhs[4] = -rhs[0]*(params.Psi_k[12])-rhs[1]*(params.Psi_k[13])-rhs[2]*(params.Psi_k[14]);
  lhs[5] = -rhs[0]*(params.Psi_k[15])-rhs[1]*(params.Psi_k[16])-rhs[2]*(params.Psi_k[17]);
  lhs[6] = -rhs[0]*(-1);
  lhs[7] = -rhs[1]*(-1);
  lhs[8] = -rhs[2]*(-1);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[4]*(-1);
  lhs[5] = -rhs[5]*(-1);
  lhs[6] = -rhs[0]*(1);
  lhs[7] = -rhs[1]*(1);
  lhs[8] = -rhs[2]*(1);
  lhs[9] = -rhs[3]*(1);
  lhs[10] = -rhs[4]*(1);
  lhs[11] = -rhs[5]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[6]*(1);
  lhs[1] = -rhs[1]*(-1)-rhs[7]*(1);
  lhs[2] = -rhs[2]*(-1)-rhs[8]*(1);
  lhs[3] = -rhs[3]*(-1)-rhs[9]*(1);
  lhs[4] = -rhs[4]*(-1)-rhs[10]*(1);
  lhs[5] = -rhs[5]*(-1)-rhs[11]*(1);
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.epsilon[0]);
  lhs[1] = rhs[1]*(2*params.epsilon[1]);
  lhs[2] = rhs[2]*(2*params.epsilon[2]);
  lhs[3] = rhs[3]*(2*params.epsilon[3]);
  lhs[4] = rhs[4]*(2*params.epsilon[4]);
  lhs[5] = rhs[5]*(2*params.epsilon[5]);
  lhs[6] = rhs[6]*(2);
  lhs[7] = rhs[7]*(2);
  lhs[8] = rhs[8]*(2);
}
void fillq(void) {
  work.q[0] = -2*params.epsilon[0]*params.eta_d[0];
  work.q[1] = -2*params.epsilon[1]*params.eta_d[1];
  work.q[2] = -2*params.epsilon[2]*params.eta_d[2];
  work.q[3] = -2*params.epsilon[3]*params.eta_d[3];
  work.q[4] = -2*params.epsilon[4]*params.eta_d[4];
  work.q[5] = -2*params.epsilon[5]*params.eta_d[5];
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
}
void fillh(void) {
  work.h[0] = -params.etamin[0];
  work.h[1] = -params.etamin[1];
  work.h[2] = -params.etamin[2];
  work.h[3] = -params.etamin[3];
  work.h[4] = -params.etamin[4];
  work.h[5] = -params.etamin[5];
  work.h[6] = params.etamax[0];
  work.h[7] = params.etamax[1];
  work.h[8] = params.etamax[2];
  work.h[9] = params.etamax[3];
  work.h[10] = params.etamax[4];
  work.h[11] = params.etamax[5];
}
void fillb(void) {
  work.b[0] = params.kappa_k[0];
  work.b[1] = params.kappa_k[1];
  work.b[2] = params.kappa_k[2];
}
void pre_ops(void) {
  work.quad_153257975808[0] = params.eta_d[0]*params.epsilon[0]*params.eta_d[0]+params.eta_d[1]*params.epsilon[1]*params.eta_d[1]+params.eta_d[2]*params.epsilon[2]*params.eta_d[2]+params.eta_d[3]*params.epsilon[3]*params.eta_d[3]+params.eta_d[4]*params.epsilon[4]*params.eta_d[4]+params.eta_d[5]*params.epsilon[5]*params.eta_d[5];
}
