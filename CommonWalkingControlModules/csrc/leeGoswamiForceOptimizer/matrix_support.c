/* Produced by CVXGEN, 2013-01-16 23:01:59 +0000.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Phi[0])-rhs[1]*(params.Phi[6])-rhs[2]*(params.Phi[12])-rhs[3]*(params.Phi[18])-rhs[4]*(params.Phi[24])-rhs[5]*(params.Phi[30])-rhs[6]*(params.Phi[36])-rhs[7]*(params.Phi[42])-rhs[8]*(params.Phi[48])-rhs[9]*(params.Phi[54])-rhs[10]*(params.Phi[60])-rhs[11]*(params.Phi[66])-rhs[12]*(params.Phi[72])-rhs[13]*(params.Phi[78])-rhs[14]*(params.Phi[84])-rhs[15]*(params.Phi[90])-rhs[16]*(-1);
  lhs[1] = -rhs[0]*(params.Phi[1])-rhs[1]*(params.Phi[7])-rhs[2]*(params.Phi[13])-rhs[3]*(params.Phi[19])-rhs[4]*(params.Phi[25])-rhs[5]*(params.Phi[31])-rhs[6]*(params.Phi[37])-rhs[7]*(params.Phi[43])-rhs[8]*(params.Phi[49])-rhs[9]*(params.Phi[55])-rhs[10]*(params.Phi[61])-rhs[11]*(params.Phi[67])-rhs[12]*(params.Phi[73])-rhs[13]*(params.Phi[79])-rhs[14]*(params.Phi[85])-rhs[15]*(params.Phi[91])-rhs[17]*(-1);
  lhs[2] = -rhs[0]*(params.Phi[2])-rhs[1]*(params.Phi[8])-rhs[2]*(params.Phi[14])-rhs[3]*(params.Phi[20])-rhs[4]*(params.Phi[26])-rhs[5]*(params.Phi[32])-rhs[6]*(params.Phi[38])-rhs[7]*(params.Phi[44])-rhs[8]*(params.Phi[50])-rhs[9]*(params.Phi[56])-rhs[10]*(params.Phi[62])-rhs[11]*(params.Phi[68])-rhs[12]*(params.Phi[74])-rhs[13]*(params.Phi[80])-rhs[14]*(params.Phi[86])-rhs[15]*(params.Phi[92])-rhs[18]*(-1);
  lhs[3] = -rhs[0]*(params.Phi[3])-rhs[1]*(params.Phi[9])-rhs[2]*(params.Phi[15])-rhs[3]*(params.Phi[21])-rhs[4]*(params.Phi[27])-rhs[5]*(params.Phi[33])-rhs[6]*(params.Phi[39])-rhs[7]*(params.Phi[45])-rhs[8]*(params.Phi[51])-rhs[9]*(params.Phi[57])-rhs[10]*(params.Phi[63])-rhs[11]*(params.Phi[69])-rhs[12]*(params.Phi[75])-rhs[13]*(params.Phi[81])-rhs[14]*(params.Phi[87])-rhs[15]*(params.Phi[93])-rhs[19]*(-1);
  lhs[4] = -rhs[0]*(params.Phi[4])-rhs[1]*(params.Phi[10])-rhs[2]*(params.Phi[16])-rhs[3]*(params.Phi[22])-rhs[4]*(params.Phi[28])-rhs[5]*(params.Phi[34])-rhs[6]*(params.Phi[40])-rhs[7]*(params.Phi[46])-rhs[8]*(params.Phi[52])-rhs[9]*(params.Phi[58])-rhs[10]*(params.Phi[64])-rhs[11]*(params.Phi[70])-rhs[12]*(params.Phi[76])-rhs[13]*(params.Phi[82])-rhs[14]*(params.Phi[88])-rhs[15]*(params.Phi[94])-rhs[20]*(-1);
  lhs[5] = -rhs[0]*(params.Phi[5])-rhs[1]*(params.Phi[11])-rhs[2]*(params.Phi[17])-rhs[3]*(params.Phi[23])-rhs[4]*(params.Phi[29])-rhs[5]*(params.Phi[35])-rhs[6]*(params.Phi[41])-rhs[7]*(params.Phi[47])-rhs[8]*(params.Phi[53])-rhs[9]*(params.Phi[59])-rhs[10]*(params.Phi[65])-rhs[11]*(params.Phi[71])-rhs[12]*(params.Phi[77])-rhs[13]*(params.Phi[83])-rhs[14]*(params.Phi[89])-rhs[15]*(params.Phi[95])-rhs[21]*(-1);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Phi[0])-rhs[1]*(params.Phi[1])-rhs[2]*(params.Phi[2])-rhs[3]*(params.Phi[3])-rhs[4]*(params.Phi[4])-rhs[5]*(params.Phi[5]);
  lhs[1] = -rhs[0]*(params.Phi[6])-rhs[1]*(params.Phi[7])-rhs[2]*(params.Phi[8])-rhs[3]*(params.Phi[9])-rhs[4]*(params.Phi[10])-rhs[5]*(params.Phi[11]);
  lhs[2] = -rhs[0]*(params.Phi[12])-rhs[1]*(params.Phi[13])-rhs[2]*(params.Phi[14])-rhs[3]*(params.Phi[15])-rhs[4]*(params.Phi[16])-rhs[5]*(params.Phi[17]);
  lhs[3] = -rhs[0]*(params.Phi[18])-rhs[1]*(params.Phi[19])-rhs[2]*(params.Phi[20])-rhs[3]*(params.Phi[21])-rhs[4]*(params.Phi[22])-rhs[5]*(params.Phi[23]);
  lhs[4] = -rhs[0]*(params.Phi[24])-rhs[1]*(params.Phi[25])-rhs[2]*(params.Phi[26])-rhs[3]*(params.Phi[27])-rhs[4]*(params.Phi[28])-rhs[5]*(params.Phi[29]);
  lhs[5] = -rhs[0]*(params.Phi[30])-rhs[1]*(params.Phi[31])-rhs[2]*(params.Phi[32])-rhs[3]*(params.Phi[33])-rhs[4]*(params.Phi[34])-rhs[5]*(params.Phi[35]);
  lhs[6] = -rhs[0]*(params.Phi[36])-rhs[1]*(params.Phi[37])-rhs[2]*(params.Phi[38])-rhs[3]*(params.Phi[39])-rhs[4]*(params.Phi[40])-rhs[5]*(params.Phi[41]);
  lhs[7] = -rhs[0]*(params.Phi[42])-rhs[1]*(params.Phi[43])-rhs[2]*(params.Phi[44])-rhs[3]*(params.Phi[45])-rhs[4]*(params.Phi[46])-rhs[5]*(params.Phi[47]);
  lhs[8] = -rhs[0]*(params.Phi[48])-rhs[1]*(params.Phi[49])-rhs[2]*(params.Phi[50])-rhs[3]*(params.Phi[51])-rhs[4]*(params.Phi[52])-rhs[5]*(params.Phi[53]);
  lhs[9] = -rhs[0]*(params.Phi[54])-rhs[1]*(params.Phi[55])-rhs[2]*(params.Phi[56])-rhs[3]*(params.Phi[57])-rhs[4]*(params.Phi[58])-rhs[5]*(params.Phi[59]);
  lhs[10] = -rhs[0]*(params.Phi[60])-rhs[1]*(params.Phi[61])-rhs[2]*(params.Phi[62])-rhs[3]*(params.Phi[63])-rhs[4]*(params.Phi[64])-rhs[5]*(params.Phi[65]);
  lhs[11] = -rhs[0]*(params.Phi[66])-rhs[1]*(params.Phi[67])-rhs[2]*(params.Phi[68])-rhs[3]*(params.Phi[69])-rhs[4]*(params.Phi[70])-rhs[5]*(params.Phi[71]);
  lhs[12] = -rhs[0]*(params.Phi[72])-rhs[1]*(params.Phi[73])-rhs[2]*(params.Phi[74])-rhs[3]*(params.Phi[75])-rhs[4]*(params.Phi[76])-rhs[5]*(params.Phi[77]);
  lhs[13] = -rhs[0]*(params.Phi[78])-rhs[1]*(params.Phi[79])-rhs[2]*(params.Phi[80])-rhs[3]*(params.Phi[81])-rhs[4]*(params.Phi[82])-rhs[5]*(params.Phi[83]);
  lhs[14] = -rhs[0]*(params.Phi[84])-rhs[1]*(params.Phi[85])-rhs[2]*(params.Phi[86])-rhs[3]*(params.Phi[87])-rhs[4]*(params.Phi[88])-rhs[5]*(params.Phi[89]);
  lhs[15] = -rhs[0]*(params.Phi[90])-rhs[1]*(params.Phi[91])-rhs[2]*(params.Phi[92])-rhs[3]*(params.Phi[93])-rhs[4]*(params.Phi[94])-rhs[5]*(params.Phi[95]);
  lhs[16] = -rhs[0]*(-1);
  lhs[17] = -rhs[1]*(-1);
  lhs[18] = -rhs[2]*(-1);
  lhs[19] = -rhs[3]*(-1);
  lhs[20] = -rhs[4]*(-1);
  lhs[21] = -rhs[5]*(-1);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[4]*(-1);
  lhs[5] = -rhs[5]*(-1);
  lhs[6] = -rhs[6]*(-1);
  lhs[7] = -rhs[7]*(-1);
  lhs[8] = -rhs[8]*(-1);
  lhs[9] = -rhs[9]*(-1);
  lhs[10] = -rhs[10]*(-1);
  lhs[11] = -rhs[11]*(-1);
  lhs[12] = -rhs[12]*(-1);
  lhs[13] = -rhs[13]*(-1);
  lhs[14] = -rhs[14]*(-1);
  lhs[15] = -rhs[15]*(-1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[4]*(-1);
  lhs[5] = -rhs[5]*(-1);
  lhs[6] = -rhs[6]*(-1);
  lhs[7] = -rhs[7]*(-1);
  lhs[8] = -rhs[8]*(-1);
  lhs[9] = -rhs[9]*(-1);
  lhs[10] = -rhs[10]*(-1);
  lhs[11] = -rhs[11]*(-1);
  lhs[12] = -rhs[12]*(-1);
  lhs[13] = -rhs[13]*(-1);
  lhs[14] = -rhs[14]*(-1);
  lhs[15] = -rhs[15]*(-1);
  lhs[16] = 0;
  lhs[17] = 0;
  lhs[18] = 0;
  lhs[19] = 0;
  lhs[20] = 0;
  lhs[21] = 0;
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.epsilon_f[0]);
  lhs[1] = rhs[1]*(2*params.epsilon_f[0]);
  lhs[2] = rhs[2]*(2*params.epsilon_f[0]);
  lhs[3] = rhs[3]*(2*params.epsilon_f[0]);
  lhs[4] = rhs[4]*(2*params.epsilon_f[0]);
  lhs[5] = rhs[5]*(2*params.epsilon_f[0]);
  lhs[6] = rhs[6]*(2*params.epsilon_f[0]);
  lhs[7] = rhs[7]*(2*params.epsilon_f[0]);
  lhs[8] = rhs[8]*(2*params.epsilon_f[0]);
  lhs[9] = rhs[9]*(2*params.epsilon_f[0]);
  lhs[10] = rhs[10]*(2*params.epsilon_f[0]);
  lhs[11] = rhs[11]*(2*params.epsilon_f[0]);
  lhs[12] = rhs[12]*(2*params.epsilon_f[0]);
  lhs[13] = rhs[13]*(2*params.epsilon_f[0]);
  lhs[14] = rhs[14]*(2*params.epsilon_f[0]);
  lhs[15] = rhs[15]*(2*params.epsilon_f[0]);
  lhs[16] = rhs[16]*(2);
  lhs[17] = rhs[17]*(2);
  lhs[18] = rhs[18]*(2);
  lhs[19] = rhs[19]*(2);
  lhs[20] = rhs[20]*(2);
  lhs[21] = rhs[21]*(2);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
  work.q[9] = 0;
  work.q[10] = 0;
  work.q[11] = 0;
  work.q[12] = 0;
  work.q[13] = 0;
  work.q[14] = 0;
  work.q[15] = 0;
  work.q[16] = 0;
  work.q[17] = 0;
  work.q[18] = 0;
  work.q[19] = 0;
  work.q[20] = 0;
  work.q[21] = 0;
}
void fillh(void) {
  work.h[0] = 0;
  work.h[1] = 0;
  work.h[2] = 0;
  work.h[3] = 0;
  work.h[4] = 0;
  work.h[5] = 0;
  work.h[6] = 0;
  work.h[7] = 0;
  work.h[8] = 0;
  work.h[9] = 0;
  work.h[10] = 0;
  work.h[11] = 0;
  work.h[12] = 0;
  work.h[13] = 0;
  work.h[14] = 0;
  work.h[15] = 0;
}
void fillb(void) {
  work.b[0] = params.xi[0];
  work.b[1] = params.xi[1];
  work.b[2] = params.xi[2];
  work.b[3] = params.xi[3];
  work.b[4] = params.xi[4];
  work.b[5] = params.xi[5];
}
void pre_ops(void) {
}
