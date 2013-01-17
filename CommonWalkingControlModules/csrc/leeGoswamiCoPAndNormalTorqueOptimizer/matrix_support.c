/* Produced by CVXGEN, 2013-01-17 22:23:43 +0000.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
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
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*(work.quad_557401407488[0]+work.quad_544982749184[0]))+rhs[1]*(2*(work.quad_557401407488[3]+work.quad_544982749184[3]))+rhs[2]*(2*(work.quad_557401407488[6]+work.quad_544982749184[6]));
  lhs[1] = rhs[0]*(2*(work.quad_557401407488[1]+work.quad_544982749184[1]))+rhs[1]*(2*(work.quad_557401407488[4]+work.quad_544982749184[4]))+rhs[2]*(2*(work.quad_557401407488[7]+work.quad_544982749184[7]));
  lhs[2] = rhs[0]*(2*(work.quad_557401407488[2]+work.quad_544982749184[2]))+rhs[1]*(2*(work.quad_557401407488[5]+work.quad_544982749184[5]))+rhs[2]*(2*(work.quad_557401407488[8]+work.quad_544982749184[8]));
  lhs[3] = rhs[3]*(2*(work.quad_875231227904[0]+work.quad_544982749184[0]))+rhs[4]*(2*(work.quad_875231227904[3]+work.quad_544982749184[3]))+rhs[5]*(2*(work.quad_875231227904[6]+work.quad_544982749184[6]));
  lhs[4] = rhs[3]*(2*(work.quad_875231227904[1]+work.quad_544982749184[1]))+rhs[4]*(2*(work.quad_875231227904[4]+work.quad_544982749184[4]))+rhs[5]*(2*(work.quad_875231227904[7]+work.quad_544982749184[7]));
  lhs[5] = rhs[3]*(2*(work.quad_875231227904[2]+work.quad_544982749184[2]))+rhs[4]*(2*(work.quad_875231227904[5]+work.quad_544982749184[5]))+rhs[5]*(2*(work.quad_875231227904[8]+work.quad_544982749184[8]));
}
void fillq(void) {
  work.q[0] = -2*(params.Psi_k_1[0]*params.kappa_k_1[0]+params.Psi_k_1[1]*params.kappa_k_1[1]+params.Psi_k_1[2]*params.kappa_k_1[2])-2*(params.epsilon[0]*(params.epsilon[0]*params.eta_d_1[0]+params.epsilon[3]*params.eta_d_1[1]+params.epsilon[6]*params.eta_d_1[2])+params.epsilon[3]*(params.epsilon[1]*params.eta_d_1[0]+params.epsilon[4]*params.eta_d_1[1]+params.epsilon[7]*params.eta_d_1[2])+params.epsilon[6]*(params.epsilon[2]*params.eta_d_1[0]+params.epsilon[5]*params.eta_d_1[1]+params.epsilon[8]*params.eta_d_1[2]));
  work.q[1] = -2*(params.Psi_k_1[3]*params.kappa_k_1[0]+params.Psi_k_1[4]*params.kappa_k_1[1]+params.Psi_k_1[5]*params.kappa_k_1[2])-2*(params.epsilon[1]*(params.epsilon[0]*params.eta_d_1[0]+params.epsilon[3]*params.eta_d_1[1]+params.epsilon[6]*params.eta_d_1[2])+params.epsilon[4]*(params.epsilon[1]*params.eta_d_1[0]+params.epsilon[4]*params.eta_d_1[1]+params.epsilon[7]*params.eta_d_1[2])+params.epsilon[7]*(params.epsilon[2]*params.eta_d_1[0]+params.epsilon[5]*params.eta_d_1[1]+params.epsilon[8]*params.eta_d_1[2]));
  work.q[2] = -2*(params.Psi_k_1[6]*params.kappa_k_1[0]+params.Psi_k_1[7]*params.kappa_k_1[1]+params.Psi_k_1[8]*params.kappa_k_1[2])-2*(params.epsilon[2]*(params.epsilon[0]*params.eta_d_1[0]+params.epsilon[3]*params.eta_d_1[1]+params.epsilon[6]*params.eta_d_1[2])+params.epsilon[5]*(params.epsilon[1]*params.eta_d_1[0]+params.epsilon[4]*params.eta_d_1[1]+params.epsilon[7]*params.eta_d_1[2])+params.epsilon[8]*(params.epsilon[2]*params.eta_d_1[0]+params.epsilon[5]*params.eta_d_1[1]+params.epsilon[8]*params.eta_d_1[2]));
  work.q[3] = -2*(params.Psi_k_2[0]*params.kappa_k_2[0]+params.Psi_k_2[1]*params.kappa_k_2[1]+params.Psi_k_2[2]*params.kappa_k_2[2])-2*(params.epsilon[0]*(params.epsilon[0]*params.eta_d_2[0]+params.epsilon[3]*params.eta_d_2[1]+params.epsilon[6]*params.eta_d_2[2])+params.epsilon[3]*(params.epsilon[1]*params.eta_d_2[0]+params.epsilon[4]*params.eta_d_2[1]+params.epsilon[7]*params.eta_d_2[2])+params.epsilon[6]*(params.epsilon[2]*params.eta_d_2[0]+params.epsilon[5]*params.eta_d_2[1]+params.epsilon[8]*params.eta_d_2[2]));
  work.q[4] = -2*(params.Psi_k_2[3]*params.kappa_k_2[0]+params.Psi_k_2[4]*params.kappa_k_2[1]+params.Psi_k_2[5]*params.kappa_k_2[2])-2*(params.epsilon[1]*(params.epsilon[0]*params.eta_d_2[0]+params.epsilon[3]*params.eta_d_2[1]+params.epsilon[6]*params.eta_d_2[2])+params.epsilon[4]*(params.epsilon[1]*params.eta_d_2[0]+params.epsilon[4]*params.eta_d_2[1]+params.epsilon[7]*params.eta_d_2[2])+params.epsilon[7]*(params.epsilon[2]*params.eta_d_2[0]+params.epsilon[5]*params.eta_d_2[1]+params.epsilon[8]*params.eta_d_2[2]));
  work.q[5] = -2*(params.Psi_k_2[6]*params.kappa_k_2[0]+params.Psi_k_2[7]*params.kappa_k_2[1]+params.Psi_k_2[8]*params.kappa_k_2[2])-2*(params.epsilon[2]*(params.epsilon[0]*params.eta_d_2[0]+params.epsilon[3]*params.eta_d_2[1]+params.epsilon[6]*params.eta_d_2[2])+params.epsilon[5]*(params.epsilon[1]*params.eta_d_2[0]+params.epsilon[4]*params.eta_d_2[1]+params.epsilon[7]*params.eta_d_2[2])+params.epsilon[8]*(params.epsilon[2]*params.eta_d_2[0]+params.epsilon[5]*params.eta_d_2[1]+params.epsilon[8]*params.eta_d_2[2]));
}
void fillh(void) {
  work.h[0] = -params.etamin_1[0];
  work.h[1] = -params.etamin_1[1];
  work.h[2] = -params.etamin_1[2];
  work.h[3] = -params.etamin_2[0];
  work.h[4] = -params.etamin_2[1];
  work.h[5] = -params.etamin_2[2];
  work.h[6] = params.etamax_1[0];
  work.h[7] = params.etamax_1[1];
  work.h[8] = params.etamax_1[2];
  work.h[9] = params.etamax_2[0];
  work.h[10] = params.etamax_2[1];
  work.h[11] = params.etamax_2[2];
}
void fillb(void) {
}
void pre_ops(void) {
  work.quad_557401407488[0] = params.Psi_k_1[0]*params.Psi_k_1[0]+params.Psi_k_1[1]*params.Psi_k_1[1]+params.Psi_k_1[2]*params.Psi_k_1[2];
  work.quad_557401407488[3] = params.Psi_k_1[0]*params.Psi_k_1[3]+params.Psi_k_1[1]*params.Psi_k_1[4]+params.Psi_k_1[2]*params.Psi_k_1[5];
  work.quad_557401407488[6] = params.Psi_k_1[0]*params.Psi_k_1[6]+params.Psi_k_1[1]*params.Psi_k_1[7]+params.Psi_k_1[2]*params.Psi_k_1[8];
  work.quad_557401407488[1] = params.Psi_k_1[3]*params.Psi_k_1[0]+params.Psi_k_1[4]*params.Psi_k_1[1]+params.Psi_k_1[5]*params.Psi_k_1[2];
  work.quad_557401407488[4] = params.Psi_k_1[3]*params.Psi_k_1[3]+params.Psi_k_1[4]*params.Psi_k_1[4]+params.Psi_k_1[5]*params.Psi_k_1[5];
  work.quad_557401407488[7] = params.Psi_k_1[3]*params.Psi_k_1[6]+params.Psi_k_1[4]*params.Psi_k_1[7]+params.Psi_k_1[5]*params.Psi_k_1[8];
  work.quad_557401407488[2] = params.Psi_k_1[6]*params.Psi_k_1[0]+params.Psi_k_1[7]*params.Psi_k_1[1]+params.Psi_k_1[8]*params.Psi_k_1[2];
  work.quad_557401407488[5] = params.Psi_k_1[6]*params.Psi_k_1[3]+params.Psi_k_1[7]*params.Psi_k_1[4]+params.Psi_k_1[8]*params.Psi_k_1[5];
  work.quad_557401407488[8] = params.Psi_k_1[6]*params.Psi_k_1[6]+params.Psi_k_1[7]*params.Psi_k_1[7]+params.Psi_k_1[8]*params.Psi_k_1[8];
  work.quad_544982749184[0] = params.epsilon[0]*params.epsilon[0]+params.epsilon[3]*params.epsilon[1]+params.epsilon[6]*params.epsilon[2];
  work.quad_544982749184[3] = params.epsilon[0]*params.epsilon[3]+params.epsilon[3]*params.epsilon[4]+params.epsilon[6]*params.epsilon[5];
  work.quad_544982749184[6] = params.epsilon[0]*params.epsilon[6]+params.epsilon[3]*params.epsilon[7]+params.epsilon[6]*params.epsilon[8];
  work.quad_544982749184[1] = params.epsilon[1]*params.epsilon[0]+params.epsilon[4]*params.epsilon[1]+params.epsilon[7]*params.epsilon[2];
  work.quad_544982749184[4] = params.epsilon[1]*params.epsilon[3]+params.epsilon[4]*params.epsilon[4]+params.epsilon[7]*params.epsilon[5];
  work.quad_544982749184[7] = params.epsilon[1]*params.epsilon[6]+params.epsilon[4]*params.epsilon[7]+params.epsilon[7]*params.epsilon[8];
  work.quad_544982749184[2] = params.epsilon[2]*params.epsilon[0]+params.epsilon[5]*params.epsilon[1]+params.epsilon[8]*params.epsilon[2];
  work.quad_544982749184[5] = params.epsilon[2]*params.epsilon[3]+params.epsilon[5]*params.epsilon[4]+params.epsilon[8]*params.epsilon[5];
  work.quad_544982749184[8] = params.epsilon[2]*params.epsilon[6]+params.epsilon[5]*params.epsilon[7]+params.epsilon[8]*params.epsilon[8];
  work.quad_875231227904[0] = params.Psi_k_2[0]*params.Psi_k_2[0]+params.Psi_k_2[1]*params.Psi_k_2[1]+params.Psi_k_2[2]*params.Psi_k_2[2];
  work.quad_875231227904[3] = params.Psi_k_2[0]*params.Psi_k_2[3]+params.Psi_k_2[1]*params.Psi_k_2[4]+params.Psi_k_2[2]*params.Psi_k_2[5];
  work.quad_875231227904[6] = params.Psi_k_2[0]*params.Psi_k_2[6]+params.Psi_k_2[1]*params.Psi_k_2[7]+params.Psi_k_2[2]*params.Psi_k_2[8];
  work.quad_875231227904[1] = params.Psi_k_2[3]*params.Psi_k_2[0]+params.Psi_k_2[4]*params.Psi_k_2[1]+params.Psi_k_2[5]*params.Psi_k_2[2];
  work.quad_875231227904[4] = params.Psi_k_2[3]*params.Psi_k_2[3]+params.Psi_k_2[4]*params.Psi_k_2[4]+params.Psi_k_2[5]*params.Psi_k_2[5];
  work.quad_875231227904[7] = params.Psi_k_2[3]*params.Psi_k_2[6]+params.Psi_k_2[4]*params.Psi_k_2[7]+params.Psi_k_2[5]*params.Psi_k_2[8];
  work.quad_875231227904[2] = params.Psi_k_2[6]*params.Psi_k_2[0]+params.Psi_k_2[7]*params.Psi_k_2[1]+params.Psi_k_2[8]*params.Psi_k_2[2];
  work.quad_875231227904[5] = params.Psi_k_2[6]*params.Psi_k_2[3]+params.Psi_k_2[7]*params.Psi_k_2[4]+params.Psi_k_2[8]*params.Psi_k_2[5];
  work.quad_875231227904[8] = params.Psi_k_2[6]*params.Psi_k_2[6]+params.Psi_k_2[7]*params.Psi_k_2[7]+params.Psi_k_2[8]*params.Psi_k_2[8];
  work.quad_573953101824[0] = params.kappa_k_1[0]*params.kappa_k_1[0]+params.kappa_k_1[1]*params.kappa_k_1[1]+params.kappa_k_1[2]*params.kappa_k_1[2];
  work.quad_34828926976[0] = (params.eta_d_1[0]*(params.epsilon[0]*(params.epsilon[0]*params.eta_d_1[0]+params.epsilon[3]*params.eta_d_1[1]+params.epsilon[6]*params.eta_d_1[2])+params.epsilon[3]*(params.epsilon[1]*params.eta_d_1[0]+params.epsilon[4]*params.eta_d_1[1]+params.epsilon[7]*params.eta_d_1[2])+params.epsilon[6]*(params.epsilon[2]*params.eta_d_1[0]+params.epsilon[5]*params.eta_d_1[1]+params.epsilon[8]*params.eta_d_1[2]))+params.eta_d_1[1]*(params.epsilon[1]*(params.epsilon[0]*params.eta_d_1[0]+params.epsilon[3]*params.eta_d_1[1]+params.epsilon[6]*params.eta_d_1[2])+params.epsilon[4]*(params.epsilon[1]*params.eta_d_1[0]+params.epsilon[4]*params.eta_d_1[1]+params.epsilon[7]*params.eta_d_1[2])+params.epsilon[7]*(params.epsilon[2]*params.eta_d_1[0]+params.epsilon[5]*params.eta_d_1[1]+params.epsilon[8]*params.eta_d_1[2]))+params.eta_d_1[2]*(params.epsilon[2]*(params.epsilon[0]*params.eta_d_1[0]+params.epsilon[3]*params.eta_d_1[1]+params.epsilon[6]*params.eta_d_1[2])+params.epsilon[5]*(params.epsilon[1]*params.eta_d_1[0]+params.epsilon[4]*params.eta_d_1[1]+params.epsilon[7]*params.eta_d_1[2])+params.epsilon[8]*(params.epsilon[2]*params.eta_d_1[0]+params.epsilon[5]*params.eta_d_1[1]+params.epsilon[8]*params.eta_d_1[2])));
  work.quad_49710497792[0] = params.kappa_k_2[0]*params.kappa_k_2[0]+params.kappa_k_2[1]*params.kappa_k_2[1]+params.kappa_k_2[2]*params.kappa_k_2[2];
  work.quad_859417034752[0] = (params.eta_d_2[0]*(params.epsilon[0]*(params.epsilon[0]*params.eta_d_2[0]+params.epsilon[3]*params.eta_d_2[1]+params.epsilon[6]*params.eta_d_2[2])+params.epsilon[3]*(params.epsilon[1]*params.eta_d_2[0]+params.epsilon[4]*params.eta_d_2[1]+params.epsilon[7]*params.eta_d_2[2])+params.epsilon[6]*(params.epsilon[2]*params.eta_d_2[0]+params.epsilon[5]*params.eta_d_2[1]+params.epsilon[8]*params.eta_d_2[2]))+params.eta_d_2[1]*(params.epsilon[1]*(params.epsilon[0]*params.eta_d_2[0]+params.epsilon[3]*params.eta_d_2[1]+params.epsilon[6]*params.eta_d_2[2])+params.epsilon[4]*(params.epsilon[1]*params.eta_d_2[0]+params.epsilon[4]*params.eta_d_2[1]+params.epsilon[7]*params.eta_d_2[2])+params.epsilon[7]*(params.epsilon[2]*params.eta_d_2[0]+params.epsilon[5]*params.eta_d_2[1]+params.epsilon[8]*params.eta_d_2[2]))+params.eta_d_2[2]*(params.epsilon[2]*(params.epsilon[0]*params.eta_d_2[0]+params.epsilon[3]*params.eta_d_2[1]+params.epsilon[6]*params.eta_d_2[2])+params.epsilon[5]*(params.epsilon[1]*params.eta_d_2[0]+params.epsilon[4]*params.eta_d_2[1]+params.epsilon[7]*params.eta_d_2[2])+params.epsilon[8]*(params.epsilon[2]*params.eta_d_2[0]+params.epsilon[5]*params.eta_d_2[1]+params.epsilon[8]*params.eta_d_2[2])));
}
