/* Produced by CVXGEN, 2013-01-17 15:52:18 +0000.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void ldl_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */
  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  work.v[0] = target[14];
  work.v[1] = target[15];
  work.v[2] = target[16];
  work.v[3] = target[17];
  work.v[4] = target[18];
  work.v[5] = target[19];
  work.v[6] = target[20];
  work.v[7] = target[21];
  work.v[8] = target[22]-work.L[0]*work.v[0];
  work.v[9] = target[23]-work.L[1]*work.v[1];
  work.v[10] = target[24]-work.L[2]*work.v[2];
  work.v[11] = target[25]-work.L[3]*work.v[3];
  work.v[12] = target[26]-work.L[4]*work.v[4];
  work.v[13] = target[27]-work.L[5]*work.v[5];
  work.v[14] = target[28]-work.L[6]*work.v[6];
  work.v[15] = target[29]-work.L[7]*work.v[7];
  work.v[16] = target[8];
  work.v[17] = target[9];
  work.v[18] = target[10];
  work.v[19] = target[11];
  work.v[20] = target[12];
  work.v[21] = target[13];
  work.v[22] = target[0]-work.L[8]*work.v[8];
  work.v[23] = target[1]-work.L[9]*work.v[9];
  work.v[24] = target[2]-work.L[10]*work.v[10];
  work.v[25] = target[3]-work.L[11]*work.v[11];
  work.v[26] = target[4]-work.L[12]*work.v[12];
  work.v[27] = target[5]-work.L[13]*work.v[13];
  work.v[28] = target[6]-work.L[14]*work.v[14];
  work.v[29] = target[7]-work.L[15]*work.v[15];
  work.v[30] = target[30]-work.L[16]*work.v[16]-work.L[17]*work.v[22]-work.L[18]*work.v[23]-work.L[19]*work.v[24]-work.L[20]*work.v[25]-work.L[21]*work.v[26]-work.L[22]*work.v[27]-work.L[23]*work.v[28]-work.L[24]*work.v[29];
  work.v[31] = target[31]-work.L[25]*work.v[17]-work.L[26]*work.v[22]-work.L[27]*work.v[23]-work.L[28]*work.v[24]-work.L[29]*work.v[25]-work.L[30]*work.v[26]-work.L[31]*work.v[27]-work.L[32]*work.v[28]-work.L[33]*work.v[29]-work.L[34]*work.v[30];
  work.v[32] = target[32]-work.L[35]*work.v[18]-work.L[36]*work.v[22]-work.L[37]*work.v[23]-work.L[38]*work.v[24]-work.L[39]*work.v[25]-work.L[40]*work.v[26]-work.L[41]*work.v[27]-work.L[42]*work.v[28]-work.L[43]*work.v[29]-work.L[44]*work.v[30]-work.L[45]*work.v[31];
  work.v[33] = target[33]-work.L[46]*work.v[19]-work.L[47]*work.v[22]-work.L[48]*work.v[23]-work.L[49]*work.v[24]-work.L[50]*work.v[25]-work.L[51]*work.v[26]-work.L[52]*work.v[27]-work.L[53]*work.v[28]-work.L[54]*work.v[29]-work.L[55]*work.v[30]-work.L[56]*work.v[31]-work.L[57]*work.v[32];
  work.v[34] = target[34]-work.L[58]*work.v[20]-work.L[59]*work.v[22]-work.L[60]*work.v[23]-work.L[61]*work.v[24]-work.L[62]*work.v[25]-work.L[63]*work.v[26]-work.L[64]*work.v[27]-work.L[65]*work.v[28]-work.L[66]*work.v[29]-work.L[67]*work.v[30]-work.L[68]*work.v[31]-work.L[69]*work.v[32]-work.L[70]*work.v[33];
  work.v[35] = target[35]-work.L[71]*work.v[21]-work.L[72]*work.v[22]-work.L[73]*work.v[23]-work.L[74]*work.v[24]-work.L[75]*work.v[25]-work.L[76]*work.v[26]-work.L[77]*work.v[27]-work.L[78]*work.v[28]-work.L[79]*work.v[29]-work.L[80]*work.v[30]-work.L[81]*work.v[31]-work.L[82]*work.v[32]-work.L[83]*work.v[33]-work.L[84]*work.v[34];
  /* Diagonal scaling. Assume correctness of work.d_inv. */
  for (i = 0; i < 36; i++)
    work.v[i] *= work.d_inv[i];
  /* Back substitution */
  work.v[34] -= work.L[84]*work.v[35];
  work.v[33] -= work.L[70]*work.v[34]+work.L[83]*work.v[35];
  work.v[32] -= work.L[57]*work.v[33]+work.L[69]*work.v[34]+work.L[82]*work.v[35];
  work.v[31] -= work.L[45]*work.v[32]+work.L[56]*work.v[33]+work.L[68]*work.v[34]+work.L[81]*work.v[35];
  work.v[30] -= work.L[34]*work.v[31]+work.L[44]*work.v[32]+work.L[55]*work.v[33]+work.L[67]*work.v[34]+work.L[80]*work.v[35];
  work.v[29] -= work.L[24]*work.v[30]+work.L[33]*work.v[31]+work.L[43]*work.v[32]+work.L[54]*work.v[33]+work.L[66]*work.v[34]+work.L[79]*work.v[35];
  work.v[28] -= work.L[23]*work.v[30]+work.L[32]*work.v[31]+work.L[42]*work.v[32]+work.L[53]*work.v[33]+work.L[65]*work.v[34]+work.L[78]*work.v[35];
  work.v[27] -= work.L[22]*work.v[30]+work.L[31]*work.v[31]+work.L[41]*work.v[32]+work.L[52]*work.v[33]+work.L[64]*work.v[34]+work.L[77]*work.v[35];
  work.v[26] -= work.L[21]*work.v[30]+work.L[30]*work.v[31]+work.L[40]*work.v[32]+work.L[51]*work.v[33]+work.L[63]*work.v[34]+work.L[76]*work.v[35];
  work.v[25] -= work.L[20]*work.v[30]+work.L[29]*work.v[31]+work.L[39]*work.v[32]+work.L[50]*work.v[33]+work.L[62]*work.v[34]+work.L[75]*work.v[35];
  work.v[24] -= work.L[19]*work.v[30]+work.L[28]*work.v[31]+work.L[38]*work.v[32]+work.L[49]*work.v[33]+work.L[61]*work.v[34]+work.L[74]*work.v[35];
  work.v[23] -= work.L[18]*work.v[30]+work.L[27]*work.v[31]+work.L[37]*work.v[32]+work.L[48]*work.v[33]+work.L[60]*work.v[34]+work.L[73]*work.v[35];
  work.v[22] -= work.L[17]*work.v[30]+work.L[26]*work.v[31]+work.L[36]*work.v[32]+work.L[47]*work.v[33]+work.L[59]*work.v[34]+work.L[72]*work.v[35];
  work.v[21] -= work.L[71]*work.v[35];
  work.v[20] -= work.L[58]*work.v[34];
  work.v[19] -= work.L[46]*work.v[33];
  work.v[18] -= work.L[35]*work.v[32];
  work.v[17] -= work.L[25]*work.v[31];
  work.v[16] -= work.L[16]*work.v[30];
  work.v[15] -= work.L[15]*work.v[29];
  work.v[14] -= work.L[14]*work.v[28];
  work.v[13] -= work.L[13]*work.v[27];
  work.v[12] -= work.L[12]*work.v[26];
  work.v[11] -= work.L[11]*work.v[25];
  work.v[10] -= work.L[10]*work.v[24];
  work.v[9] -= work.L[9]*work.v[23];
  work.v[8] -= work.L[8]*work.v[22];
  work.v[7] -= work.L[7]*work.v[15];
  work.v[6] -= work.L[6]*work.v[14];
  work.v[5] -= work.L[5]*work.v[13];
  work.v[4] -= work.L[4]*work.v[12];
  work.v[3] -= work.L[3]*work.v[11];
  work.v[2] -= work.L[2]*work.v[10];
  work.v[1] -= work.L[1]*work.v[9];
  work.v[0] -= work.L[0]*work.v[8];
  /* Unpermute the result, from v to var. */
  var[0] = work.v[22];
  var[1] = work.v[23];
  var[2] = work.v[24];
  var[3] = work.v[25];
  var[4] = work.v[26];
  var[5] = work.v[27];
  var[6] = work.v[28];
  var[7] = work.v[29];
  var[8] = work.v[16];
  var[9] = work.v[17];
  var[10] = work.v[18];
  var[11] = work.v[19];
  var[12] = work.v[20];
  var[13] = work.v[21];
  var[14] = work.v[0];
  var[15] = work.v[1];
  var[16] = work.v[2];
  var[17] = work.v[3];
  var[18] = work.v[4];
  var[19] = work.v[5];
  var[20] = work.v[6];
  var[21] = work.v[7];
  var[22] = work.v[8];
  var[23] = work.v[9];
  var[24] = work.v[10];
  var[25] = work.v[11];
  var[26] = work.v[12];
  var[27] = work.v[13];
  var[28] = work.v[14];
  var[29] = work.v[15];
  var[30] = work.v[30];
  var[31] = work.v[31];
  var[32] = work.v[32];
  var[33] = work.v[33];
  var[34] = work.v[34];
  var[35] = work.v[35];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared norm for solution is %.8g.\n", check_residual(target, var));
  }
#endif
}
void ldl_factor(void) {
  work.d[0] = work.KKT[0];
  if (work.d[0] < 0)
    work.d[0] = settings.kkt_reg;
  else
    work.d[0] += settings.kkt_reg;
  work.d_inv[0] = 1/work.d[0];
  work.L[0] = work.KKT[1]*work.d_inv[0];
  work.v[1] = work.KKT[2];
  work.d[1] = work.v[1];
  if (work.d[1] < 0)
    work.d[1] = settings.kkt_reg;
  else
    work.d[1] += settings.kkt_reg;
  work.d_inv[1] = 1/work.d[1];
  work.L[1] = (work.KKT[3])*work.d_inv[1];
  work.v[2] = work.KKT[4];
  work.d[2] = work.v[2];
  if (work.d[2] < 0)
    work.d[2] = settings.kkt_reg;
  else
    work.d[2] += settings.kkt_reg;
  work.d_inv[2] = 1/work.d[2];
  work.L[2] = (work.KKT[5])*work.d_inv[2];
  work.v[3] = work.KKT[6];
  work.d[3] = work.v[3];
  if (work.d[3] < 0)
    work.d[3] = settings.kkt_reg;
  else
    work.d[3] += settings.kkt_reg;
  work.d_inv[3] = 1/work.d[3];
  work.L[3] = (work.KKT[7])*work.d_inv[3];
  work.v[4] = work.KKT[8];
  work.d[4] = work.v[4];
  if (work.d[4] < 0)
    work.d[4] = settings.kkt_reg;
  else
    work.d[4] += settings.kkt_reg;
  work.d_inv[4] = 1/work.d[4];
  work.L[4] = (work.KKT[9])*work.d_inv[4];
  work.v[5] = work.KKT[10];
  work.d[5] = work.v[5];
  if (work.d[5] < 0)
    work.d[5] = settings.kkt_reg;
  else
    work.d[5] += settings.kkt_reg;
  work.d_inv[5] = 1/work.d[5];
  work.L[5] = (work.KKT[11])*work.d_inv[5];
  work.v[6] = work.KKT[12];
  work.d[6] = work.v[6];
  if (work.d[6] < 0)
    work.d[6] = settings.kkt_reg;
  else
    work.d[6] += settings.kkt_reg;
  work.d_inv[6] = 1/work.d[6];
  work.L[6] = (work.KKT[13])*work.d_inv[6];
  work.v[7] = work.KKT[14];
  work.d[7] = work.v[7];
  if (work.d[7] < 0)
    work.d[7] = settings.kkt_reg;
  else
    work.d[7] += settings.kkt_reg;
  work.d_inv[7] = 1/work.d[7];
  work.L[7] = (work.KKT[15])*work.d_inv[7];
  work.v[0] = work.L[0]*work.d[0];
  work.v[8] = work.KKT[16]-work.L[0]*work.v[0];
  work.d[8] = work.v[8];
  if (work.d[8] > 0)
    work.d[8] = -settings.kkt_reg;
  else
    work.d[8] -= settings.kkt_reg;
  work.d_inv[8] = 1/work.d[8];
  work.L[8] = (work.KKT[17])*work.d_inv[8];
  work.v[1] = work.L[1]*work.d[1];
  work.v[9] = work.KKT[18]-work.L[1]*work.v[1];
  work.d[9] = work.v[9];
  if (work.d[9] > 0)
    work.d[9] = -settings.kkt_reg;
  else
    work.d[9] -= settings.kkt_reg;
  work.d_inv[9] = 1/work.d[9];
  work.L[9] = (work.KKT[19])*work.d_inv[9];
  work.v[2] = work.L[2]*work.d[2];
  work.v[10] = work.KKT[20]-work.L[2]*work.v[2];
  work.d[10] = work.v[10];
  if (work.d[10] > 0)
    work.d[10] = -settings.kkt_reg;
  else
    work.d[10] -= settings.kkt_reg;
  work.d_inv[10] = 1/work.d[10];
  work.L[10] = (work.KKT[21])*work.d_inv[10];
  work.v[3] = work.L[3]*work.d[3];
  work.v[11] = work.KKT[22]-work.L[3]*work.v[3];
  work.d[11] = work.v[11];
  if (work.d[11] > 0)
    work.d[11] = -settings.kkt_reg;
  else
    work.d[11] -= settings.kkt_reg;
  work.d_inv[11] = 1/work.d[11];
  work.L[11] = (work.KKT[23])*work.d_inv[11];
  work.v[4] = work.L[4]*work.d[4];
  work.v[12] = work.KKT[24]-work.L[4]*work.v[4];
  work.d[12] = work.v[12];
  if (work.d[12] > 0)
    work.d[12] = -settings.kkt_reg;
  else
    work.d[12] -= settings.kkt_reg;
  work.d_inv[12] = 1/work.d[12];
  work.L[12] = (work.KKT[25])*work.d_inv[12];
  work.v[5] = work.L[5]*work.d[5];
  work.v[13] = work.KKT[26]-work.L[5]*work.v[5];
  work.d[13] = work.v[13];
  if (work.d[13] > 0)
    work.d[13] = -settings.kkt_reg;
  else
    work.d[13] -= settings.kkt_reg;
  work.d_inv[13] = 1/work.d[13];
  work.L[13] = (work.KKT[27])*work.d_inv[13];
  work.v[6] = work.L[6]*work.d[6];
  work.v[14] = work.KKT[28]-work.L[6]*work.v[6];
  work.d[14] = work.v[14];
  if (work.d[14] > 0)
    work.d[14] = -settings.kkt_reg;
  else
    work.d[14] -= settings.kkt_reg;
  work.d_inv[14] = 1/work.d[14];
  work.L[14] = (work.KKT[29])*work.d_inv[14];
  work.v[7] = work.L[7]*work.d[7];
  work.v[15] = work.KKT[30]-work.L[7]*work.v[7];
  work.d[15] = work.v[15];
  if (work.d[15] > 0)
    work.d[15] = -settings.kkt_reg;
  else
    work.d[15] -= settings.kkt_reg;
  work.d_inv[15] = 1/work.d[15];
  work.L[15] = (work.KKT[31])*work.d_inv[15];
  work.v[16] = work.KKT[32];
  work.d[16] = work.v[16];
  if (work.d[16] < 0)
    work.d[16] = settings.kkt_reg;
  else
    work.d[16] += settings.kkt_reg;
  work.d_inv[16] = 1/work.d[16];
  work.L[16] = (work.KKT[33])*work.d_inv[16];
  work.v[17] = work.KKT[34];
  work.d[17] = work.v[17];
  if (work.d[17] < 0)
    work.d[17] = settings.kkt_reg;
  else
    work.d[17] += settings.kkt_reg;
  work.d_inv[17] = 1/work.d[17];
  work.L[25] = (work.KKT[35])*work.d_inv[17];
  work.v[18] = work.KKT[36];
  work.d[18] = work.v[18];
  if (work.d[18] < 0)
    work.d[18] = settings.kkt_reg;
  else
    work.d[18] += settings.kkt_reg;
  work.d_inv[18] = 1/work.d[18];
  work.L[35] = (work.KKT[37])*work.d_inv[18];
  work.v[19] = work.KKT[38];
  work.d[19] = work.v[19];
  if (work.d[19] < 0)
    work.d[19] = settings.kkt_reg;
  else
    work.d[19] += settings.kkt_reg;
  work.d_inv[19] = 1/work.d[19];
  work.L[46] = (work.KKT[39])*work.d_inv[19];
  work.v[20] = work.KKT[40];
  work.d[20] = work.v[20];
  if (work.d[20] < 0)
    work.d[20] = settings.kkt_reg;
  else
    work.d[20] += settings.kkt_reg;
  work.d_inv[20] = 1/work.d[20];
  work.L[58] = (work.KKT[41])*work.d_inv[20];
  work.v[21] = work.KKT[42];
  work.d[21] = work.v[21];
  if (work.d[21] < 0)
    work.d[21] = settings.kkt_reg;
  else
    work.d[21] += settings.kkt_reg;
  work.d_inv[21] = 1/work.d[21];
  work.L[71] = (work.KKT[43])*work.d_inv[21];
  work.v[8] = work.L[8]*work.d[8];
  work.v[22] = work.KKT[44]-work.L[8]*work.v[8];
  work.d[22] = work.v[22];
  if (work.d[22] < 0)
    work.d[22] = settings.kkt_reg;
  else
    work.d[22] += settings.kkt_reg;
  work.d_inv[22] = 1/work.d[22];
  work.L[17] = (work.KKT[45])*work.d_inv[22];
  work.L[26] = (work.KKT[46])*work.d_inv[22];
  work.L[36] = (work.KKT[47])*work.d_inv[22];
  work.L[47] = (work.KKT[48])*work.d_inv[22];
  work.L[59] = (work.KKT[49])*work.d_inv[22];
  work.L[72] = (work.KKT[50])*work.d_inv[22];
  work.v[9] = work.L[9]*work.d[9];
  work.v[23] = work.KKT[51]-work.L[9]*work.v[9];
  work.d[23] = work.v[23];
  if (work.d[23] < 0)
    work.d[23] = settings.kkt_reg;
  else
    work.d[23] += settings.kkt_reg;
  work.d_inv[23] = 1/work.d[23];
  work.L[18] = (work.KKT[52])*work.d_inv[23];
  work.L[27] = (work.KKT[53])*work.d_inv[23];
  work.L[37] = (work.KKT[54])*work.d_inv[23];
  work.L[48] = (work.KKT[55])*work.d_inv[23];
  work.L[60] = (work.KKT[56])*work.d_inv[23];
  work.L[73] = (work.KKT[57])*work.d_inv[23];
  work.v[10] = work.L[10]*work.d[10];
  work.v[24] = work.KKT[58]-work.L[10]*work.v[10];
  work.d[24] = work.v[24];
  if (work.d[24] < 0)
    work.d[24] = settings.kkt_reg;
  else
    work.d[24] += settings.kkt_reg;
  work.d_inv[24] = 1/work.d[24];
  work.L[19] = (work.KKT[59])*work.d_inv[24];
  work.L[28] = (work.KKT[60])*work.d_inv[24];
  work.L[38] = (work.KKT[61])*work.d_inv[24];
  work.L[49] = (work.KKT[62])*work.d_inv[24];
  work.L[61] = (work.KKT[63])*work.d_inv[24];
  work.L[74] = (work.KKT[64])*work.d_inv[24];
  work.v[11] = work.L[11]*work.d[11];
  work.v[25] = work.KKT[65]-work.L[11]*work.v[11];
  work.d[25] = work.v[25];
  if (work.d[25] < 0)
    work.d[25] = settings.kkt_reg;
  else
    work.d[25] += settings.kkt_reg;
  work.d_inv[25] = 1/work.d[25];
  work.L[20] = (work.KKT[66])*work.d_inv[25];
  work.L[29] = (work.KKT[67])*work.d_inv[25];
  work.L[39] = (work.KKT[68])*work.d_inv[25];
  work.L[50] = (work.KKT[69])*work.d_inv[25];
  work.L[62] = (work.KKT[70])*work.d_inv[25];
  work.L[75] = (work.KKT[71])*work.d_inv[25];
  work.v[12] = work.L[12]*work.d[12];
  work.v[26] = work.KKT[72]-work.L[12]*work.v[12];
  work.d[26] = work.v[26];
  if (work.d[26] < 0)
    work.d[26] = settings.kkt_reg;
  else
    work.d[26] += settings.kkt_reg;
  work.d_inv[26] = 1/work.d[26];
  work.L[21] = (work.KKT[73])*work.d_inv[26];
  work.L[30] = (work.KKT[74])*work.d_inv[26];
  work.L[40] = (work.KKT[75])*work.d_inv[26];
  work.L[51] = (work.KKT[76])*work.d_inv[26];
  work.L[63] = (work.KKT[77])*work.d_inv[26];
  work.L[76] = (work.KKT[78])*work.d_inv[26];
  work.v[13] = work.L[13]*work.d[13];
  work.v[27] = work.KKT[79]-work.L[13]*work.v[13];
  work.d[27] = work.v[27];
  if (work.d[27] < 0)
    work.d[27] = settings.kkt_reg;
  else
    work.d[27] += settings.kkt_reg;
  work.d_inv[27] = 1/work.d[27];
  work.L[22] = (work.KKT[80])*work.d_inv[27];
  work.L[31] = (work.KKT[81])*work.d_inv[27];
  work.L[41] = (work.KKT[82])*work.d_inv[27];
  work.L[52] = (work.KKT[83])*work.d_inv[27];
  work.L[64] = (work.KKT[84])*work.d_inv[27];
  work.L[77] = (work.KKT[85])*work.d_inv[27];
  work.v[14] = work.L[14]*work.d[14];
  work.v[28] = work.KKT[86]-work.L[14]*work.v[14];
  work.d[28] = work.v[28];
  if (work.d[28] < 0)
    work.d[28] = settings.kkt_reg;
  else
    work.d[28] += settings.kkt_reg;
  work.d_inv[28] = 1/work.d[28];
  work.L[23] = (work.KKT[87])*work.d_inv[28];
  work.L[32] = (work.KKT[88])*work.d_inv[28];
  work.L[42] = (work.KKT[89])*work.d_inv[28];
  work.L[53] = (work.KKT[90])*work.d_inv[28];
  work.L[65] = (work.KKT[91])*work.d_inv[28];
  work.L[78] = (work.KKT[92])*work.d_inv[28];
  work.v[15] = work.L[15]*work.d[15];
  work.v[29] = work.KKT[93]-work.L[15]*work.v[15];
  work.d[29] = work.v[29];
  if (work.d[29] < 0)
    work.d[29] = settings.kkt_reg;
  else
    work.d[29] += settings.kkt_reg;
  work.d_inv[29] = 1/work.d[29];
  work.L[24] = (work.KKT[94])*work.d_inv[29];
  work.L[33] = (work.KKT[95])*work.d_inv[29];
  work.L[43] = (work.KKT[96])*work.d_inv[29];
  work.L[54] = (work.KKT[97])*work.d_inv[29];
  work.L[66] = (work.KKT[98])*work.d_inv[29];
  work.L[79] = (work.KKT[99])*work.d_inv[29];
  work.v[16] = work.L[16]*work.d[16];
  work.v[22] = work.L[17]*work.d[22];
  work.v[23] = work.L[18]*work.d[23];
  work.v[24] = work.L[19]*work.d[24];
  work.v[25] = work.L[20]*work.d[25];
  work.v[26] = work.L[21]*work.d[26];
  work.v[27] = work.L[22]*work.d[27];
  work.v[28] = work.L[23]*work.d[28];
  work.v[29] = work.L[24]*work.d[29];
  work.v[30] = 0-work.L[16]*work.v[16]-work.L[17]*work.v[22]-work.L[18]*work.v[23]-work.L[19]*work.v[24]-work.L[20]*work.v[25]-work.L[21]*work.v[26]-work.L[22]*work.v[27]-work.L[23]*work.v[28]-work.L[24]*work.v[29];
  work.d[30] = work.v[30];
  if (work.d[30] > 0)
    work.d[30] = -settings.kkt_reg;
  else
    work.d[30] -= settings.kkt_reg;
  work.d_inv[30] = 1/work.d[30];
  work.L[34] = (-work.L[26]*work.v[22]-work.L[27]*work.v[23]-work.L[28]*work.v[24]-work.L[29]*work.v[25]-work.L[30]*work.v[26]-work.L[31]*work.v[27]-work.L[32]*work.v[28]-work.L[33]*work.v[29])*work.d_inv[30];
  work.L[44] = (-work.L[36]*work.v[22]-work.L[37]*work.v[23]-work.L[38]*work.v[24]-work.L[39]*work.v[25]-work.L[40]*work.v[26]-work.L[41]*work.v[27]-work.L[42]*work.v[28]-work.L[43]*work.v[29])*work.d_inv[30];
  work.L[55] = (-work.L[47]*work.v[22]-work.L[48]*work.v[23]-work.L[49]*work.v[24]-work.L[50]*work.v[25]-work.L[51]*work.v[26]-work.L[52]*work.v[27]-work.L[53]*work.v[28]-work.L[54]*work.v[29])*work.d_inv[30];
  work.L[67] = (-work.L[59]*work.v[22]-work.L[60]*work.v[23]-work.L[61]*work.v[24]-work.L[62]*work.v[25]-work.L[63]*work.v[26]-work.L[64]*work.v[27]-work.L[65]*work.v[28]-work.L[66]*work.v[29])*work.d_inv[30];
  work.L[80] = (-work.L[72]*work.v[22]-work.L[73]*work.v[23]-work.L[74]*work.v[24]-work.L[75]*work.v[25]-work.L[76]*work.v[26]-work.L[77]*work.v[27]-work.L[78]*work.v[28]-work.L[79]*work.v[29])*work.d_inv[30];
  work.v[17] = work.L[25]*work.d[17];
  work.v[22] = work.L[26]*work.d[22];
  work.v[23] = work.L[27]*work.d[23];
  work.v[24] = work.L[28]*work.d[24];
  work.v[25] = work.L[29]*work.d[25];
  work.v[26] = work.L[30]*work.d[26];
  work.v[27] = work.L[31]*work.d[27];
  work.v[28] = work.L[32]*work.d[28];
  work.v[29] = work.L[33]*work.d[29];
  work.v[30] = work.L[34]*work.d[30];
  work.v[31] = 0-work.L[25]*work.v[17]-work.L[26]*work.v[22]-work.L[27]*work.v[23]-work.L[28]*work.v[24]-work.L[29]*work.v[25]-work.L[30]*work.v[26]-work.L[31]*work.v[27]-work.L[32]*work.v[28]-work.L[33]*work.v[29]-work.L[34]*work.v[30];
  work.d[31] = work.v[31];
  if (work.d[31] > 0)
    work.d[31] = -settings.kkt_reg;
  else
    work.d[31] -= settings.kkt_reg;
  work.d_inv[31] = 1/work.d[31];
  work.L[45] = (-work.L[36]*work.v[22]-work.L[37]*work.v[23]-work.L[38]*work.v[24]-work.L[39]*work.v[25]-work.L[40]*work.v[26]-work.L[41]*work.v[27]-work.L[42]*work.v[28]-work.L[43]*work.v[29]-work.L[44]*work.v[30])*work.d_inv[31];
  work.L[56] = (-work.L[47]*work.v[22]-work.L[48]*work.v[23]-work.L[49]*work.v[24]-work.L[50]*work.v[25]-work.L[51]*work.v[26]-work.L[52]*work.v[27]-work.L[53]*work.v[28]-work.L[54]*work.v[29]-work.L[55]*work.v[30])*work.d_inv[31];
  work.L[68] = (-work.L[59]*work.v[22]-work.L[60]*work.v[23]-work.L[61]*work.v[24]-work.L[62]*work.v[25]-work.L[63]*work.v[26]-work.L[64]*work.v[27]-work.L[65]*work.v[28]-work.L[66]*work.v[29]-work.L[67]*work.v[30])*work.d_inv[31];
  work.L[81] = (-work.L[72]*work.v[22]-work.L[73]*work.v[23]-work.L[74]*work.v[24]-work.L[75]*work.v[25]-work.L[76]*work.v[26]-work.L[77]*work.v[27]-work.L[78]*work.v[28]-work.L[79]*work.v[29]-work.L[80]*work.v[30])*work.d_inv[31];
  work.v[18] = work.L[35]*work.d[18];
  work.v[22] = work.L[36]*work.d[22];
  work.v[23] = work.L[37]*work.d[23];
  work.v[24] = work.L[38]*work.d[24];
  work.v[25] = work.L[39]*work.d[25];
  work.v[26] = work.L[40]*work.d[26];
  work.v[27] = work.L[41]*work.d[27];
  work.v[28] = work.L[42]*work.d[28];
  work.v[29] = work.L[43]*work.d[29];
  work.v[30] = work.L[44]*work.d[30];
  work.v[31] = work.L[45]*work.d[31];
  work.v[32] = 0-work.L[35]*work.v[18]-work.L[36]*work.v[22]-work.L[37]*work.v[23]-work.L[38]*work.v[24]-work.L[39]*work.v[25]-work.L[40]*work.v[26]-work.L[41]*work.v[27]-work.L[42]*work.v[28]-work.L[43]*work.v[29]-work.L[44]*work.v[30]-work.L[45]*work.v[31];
  work.d[32] = work.v[32];
  if (work.d[32] > 0)
    work.d[32] = -settings.kkt_reg;
  else
    work.d[32] -= settings.kkt_reg;
  work.d_inv[32] = 1/work.d[32];
  work.L[57] = (-work.L[47]*work.v[22]-work.L[48]*work.v[23]-work.L[49]*work.v[24]-work.L[50]*work.v[25]-work.L[51]*work.v[26]-work.L[52]*work.v[27]-work.L[53]*work.v[28]-work.L[54]*work.v[29]-work.L[55]*work.v[30]-work.L[56]*work.v[31])*work.d_inv[32];
  work.L[69] = (-work.L[59]*work.v[22]-work.L[60]*work.v[23]-work.L[61]*work.v[24]-work.L[62]*work.v[25]-work.L[63]*work.v[26]-work.L[64]*work.v[27]-work.L[65]*work.v[28]-work.L[66]*work.v[29]-work.L[67]*work.v[30]-work.L[68]*work.v[31])*work.d_inv[32];
  work.L[82] = (-work.L[72]*work.v[22]-work.L[73]*work.v[23]-work.L[74]*work.v[24]-work.L[75]*work.v[25]-work.L[76]*work.v[26]-work.L[77]*work.v[27]-work.L[78]*work.v[28]-work.L[79]*work.v[29]-work.L[80]*work.v[30]-work.L[81]*work.v[31])*work.d_inv[32];
  work.v[19] = work.L[46]*work.d[19];
  work.v[22] = work.L[47]*work.d[22];
  work.v[23] = work.L[48]*work.d[23];
  work.v[24] = work.L[49]*work.d[24];
  work.v[25] = work.L[50]*work.d[25];
  work.v[26] = work.L[51]*work.d[26];
  work.v[27] = work.L[52]*work.d[27];
  work.v[28] = work.L[53]*work.d[28];
  work.v[29] = work.L[54]*work.d[29];
  work.v[30] = work.L[55]*work.d[30];
  work.v[31] = work.L[56]*work.d[31];
  work.v[32] = work.L[57]*work.d[32];
  work.v[33] = 0-work.L[46]*work.v[19]-work.L[47]*work.v[22]-work.L[48]*work.v[23]-work.L[49]*work.v[24]-work.L[50]*work.v[25]-work.L[51]*work.v[26]-work.L[52]*work.v[27]-work.L[53]*work.v[28]-work.L[54]*work.v[29]-work.L[55]*work.v[30]-work.L[56]*work.v[31]-work.L[57]*work.v[32];
  work.d[33] = work.v[33];
  if (work.d[33] > 0)
    work.d[33] = -settings.kkt_reg;
  else
    work.d[33] -= settings.kkt_reg;
  work.d_inv[33] = 1/work.d[33];
  work.L[70] = (-work.L[59]*work.v[22]-work.L[60]*work.v[23]-work.L[61]*work.v[24]-work.L[62]*work.v[25]-work.L[63]*work.v[26]-work.L[64]*work.v[27]-work.L[65]*work.v[28]-work.L[66]*work.v[29]-work.L[67]*work.v[30]-work.L[68]*work.v[31]-work.L[69]*work.v[32])*work.d_inv[33];
  work.L[83] = (-work.L[72]*work.v[22]-work.L[73]*work.v[23]-work.L[74]*work.v[24]-work.L[75]*work.v[25]-work.L[76]*work.v[26]-work.L[77]*work.v[27]-work.L[78]*work.v[28]-work.L[79]*work.v[29]-work.L[80]*work.v[30]-work.L[81]*work.v[31]-work.L[82]*work.v[32])*work.d_inv[33];
  work.v[20] = work.L[58]*work.d[20];
  work.v[22] = work.L[59]*work.d[22];
  work.v[23] = work.L[60]*work.d[23];
  work.v[24] = work.L[61]*work.d[24];
  work.v[25] = work.L[62]*work.d[25];
  work.v[26] = work.L[63]*work.d[26];
  work.v[27] = work.L[64]*work.d[27];
  work.v[28] = work.L[65]*work.d[28];
  work.v[29] = work.L[66]*work.d[29];
  work.v[30] = work.L[67]*work.d[30];
  work.v[31] = work.L[68]*work.d[31];
  work.v[32] = work.L[69]*work.d[32];
  work.v[33] = work.L[70]*work.d[33];
  work.v[34] = 0-work.L[58]*work.v[20]-work.L[59]*work.v[22]-work.L[60]*work.v[23]-work.L[61]*work.v[24]-work.L[62]*work.v[25]-work.L[63]*work.v[26]-work.L[64]*work.v[27]-work.L[65]*work.v[28]-work.L[66]*work.v[29]-work.L[67]*work.v[30]-work.L[68]*work.v[31]-work.L[69]*work.v[32]-work.L[70]*work.v[33];
  work.d[34] = work.v[34];
  if (work.d[34] > 0)
    work.d[34] = -settings.kkt_reg;
  else
    work.d[34] -= settings.kkt_reg;
  work.d_inv[34] = 1/work.d[34];
  work.L[84] = (-work.L[72]*work.v[22]-work.L[73]*work.v[23]-work.L[74]*work.v[24]-work.L[75]*work.v[25]-work.L[76]*work.v[26]-work.L[77]*work.v[27]-work.L[78]*work.v[28]-work.L[79]*work.v[29]-work.L[80]*work.v[30]-work.L[81]*work.v[31]-work.L[82]*work.v[32]-work.L[83]*work.v[33])*work.d_inv[34];
  work.v[21] = work.L[71]*work.d[21];
  work.v[22] = work.L[72]*work.d[22];
  work.v[23] = work.L[73]*work.d[23];
  work.v[24] = work.L[74]*work.d[24];
  work.v[25] = work.L[75]*work.d[25];
  work.v[26] = work.L[76]*work.d[26];
  work.v[27] = work.L[77]*work.d[27];
  work.v[28] = work.L[78]*work.d[28];
  work.v[29] = work.L[79]*work.d[29];
  work.v[30] = work.L[80]*work.d[30];
  work.v[31] = work.L[81]*work.d[31];
  work.v[32] = work.L[82]*work.d[32];
  work.v[33] = work.L[83]*work.d[33];
  work.v[34] = work.L[84]*work.d[34];
  work.v[35] = 0-work.L[71]*work.v[21]-work.L[72]*work.v[22]-work.L[73]*work.v[23]-work.L[74]*work.v[24]-work.L[75]*work.v[25]-work.L[76]*work.v[26]-work.L[77]*work.v[27]-work.L[78]*work.v[28]-work.L[79]*work.v[29]-work.L[80]*work.v[30]-work.L[81]*work.v[31]-work.L[82]*work.v[32]-work.L[83]*work.v[33]-work.L[84]*work.v[34];
  work.d[35] = work.v[35];
  if (work.d[35] > 0)
    work.d[35] = -settings.kkt_reg;
  else
    work.d[35] -= settings.kkt_reg;
  work.d_inv[35] = 1/work.d[35];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", check_factorization());
  }
#endif
}
double check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;
  /* Only check the lower triangle. */
  residual = 0;
  temp = work.KKT[44]-1*work.d[22]*1-work.L[8]*work.d[8]*work.L[8];
  residual += temp*temp;
  temp = work.KKT[51]-1*work.d[23]*1-work.L[9]*work.d[9]*work.L[9];
  residual += temp*temp;
  temp = work.KKT[58]-1*work.d[24]*1-work.L[10]*work.d[10]*work.L[10];
  residual += temp*temp;
  temp = work.KKT[65]-1*work.d[25]*1-work.L[11]*work.d[11]*work.L[11];
  residual += temp*temp;
  temp = work.KKT[72]-1*work.d[26]*1-work.L[12]*work.d[12]*work.L[12];
  residual += temp*temp;
  temp = work.KKT[79]-1*work.d[27]*1-work.L[13]*work.d[13]*work.L[13];
  residual += temp*temp;
  temp = work.KKT[86]-1*work.d[28]*1-work.L[14]*work.d[14]*work.L[14];
  residual += temp*temp;
  temp = work.KKT[93]-1*work.d[29]*1-work.L[15]*work.d[15]*work.L[15];
  residual += temp*temp;
  temp = work.KKT[32]-1*work.d[16]*1;
  residual += temp*temp;
  temp = work.KKT[34]-1*work.d[17]*1;
  residual += temp*temp;
  temp = work.KKT[36]-1*work.d[18]*1;
  residual += temp*temp;
  temp = work.KKT[38]-1*work.d[19]*1;
  residual += temp*temp;
  temp = work.KKT[40]-1*work.d[20]*1;
  residual += temp*temp;
  temp = work.KKT[42]-1*work.d[21]*1;
  residual += temp*temp;
  temp = work.KKT[0]-1*work.d[0]*1;
  residual += temp*temp;
  temp = work.KKT[2]-1*work.d[1]*1;
  residual += temp*temp;
  temp = work.KKT[4]-1*work.d[2]*1;
  residual += temp*temp;
  temp = work.KKT[6]-1*work.d[3]*1;
  residual += temp*temp;
  temp = work.KKT[8]-1*work.d[4]*1;
  residual += temp*temp;
  temp = work.KKT[10]-1*work.d[5]*1;
  residual += temp*temp;
  temp = work.KKT[12]-1*work.d[6]*1;
  residual += temp*temp;
  temp = work.KKT[14]-1*work.d[7]*1;
  residual += temp*temp;
  temp = work.KKT[1]-work.L[0]*work.d[0]*1;
  residual += temp*temp;
  temp = work.KKT[3]-work.L[1]*work.d[1]*1;
  residual += temp*temp;
  temp = work.KKT[5]-work.L[2]*work.d[2]*1;
  residual += temp*temp;
  temp = work.KKT[7]-work.L[3]*work.d[3]*1;
  residual += temp*temp;
  temp = work.KKT[9]-work.L[4]*work.d[4]*1;
  residual += temp*temp;
  temp = work.KKT[11]-work.L[5]*work.d[5]*1;
  residual += temp*temp;
  temp = work.KKT[13]-work.L[6]*work.d[6]*1;
  residual += temp*temp;
  temp = work.KKT[15]-work.L[7]*work.d[7]*1;
  residual += temp*temp;
  temp = work.KKT[16]-work.L[0]*work.d[0]*work.L[0]-1*work.d[8]*1;
  residual += temp*temp;
  temp = work.KKT[18]-work.L[1]*work.d[1]*work.L[1]-1*work.d[9]*1;
  residual += temp*temp;
  temp = work.KKT[20]-work.L[2]*work.d[2]*work.L[2]-1*work.d[10]*1;
  residual += temp*temp;
  temp = work.KKT[22]-work.L[3]*work.d[3]*work.L[3]-1*work.d[11]*1;
  residual += temp*temp;
  temp = work.KKT[24]-work.L[4]*work.d[4]*work.L[4]-1*work.d[12]*1;
  residual += temp*temp;
  temp = work.KKT[26]-work.L[5]*work.d[5]*work.L[5]-1*work.d[13]*1;
  residual += temp*temp;
  temp = work.KKT[28]-work.L[6]*work.d[6]*work.L[6]-1*work.d[14]*1;
  residual += temp*temp;
  temp = work.KKT[30]-work.L[7]*work.d[7]*work.L[7]-1*work.d[15]*1;
  residual += temp*temp;
  temp = work.KKT[17]-1*work.d[8]*work.L[8];
  residual += temp*temp;
  temp = work.KKT[19]-1*work.d[9]*work.L[9];
  residual += temp*temp;
  temp = work.KKT[21]-1*work.d[10]*work.L[10];
  residual += temp*temp;
  temp = work.KKT[23]-1*work.d[11]*work.L[11];
  residual += temp*temp;
  temp = work.KKT[25]-1*work.d[12]*work.L[12];
  residual += temp*temp;
  temp = work.KKT[27]-1*work.d[13]*work.L[13];
  residual += temp*temp;
  temp = work.KKT[29]-1*work.d[14]*work.L[14];
  residual += temp*temp;
  temp = work.KKT[31]-1*work.d[15]*work.L[15];
  residual += temp*temp;
  temp = work.KKT[45]-work.L[17]*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[52]-work.L[18]*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[59]-work.L[19]*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[66]-work.L[20]*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[73]-work.L[21]*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[80]-work.L[22]*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[87]-work.L[23]*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[94]-work.L[24]*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[46]-work.L[26]*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[53]-work.L[27]*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[60]-work.L[28]*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[67]-work.L[29]*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[74]-work.L[30]*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[81]-work.L[31]*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[88]-work.L[32]*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[95]-work.L[33]*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[47]-work.L[36]*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[54]-work.L[37]*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[61]-work.L[38]*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[68]-work.L[39]*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[75]-work.L[40]*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[82]-work.L[41]*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[89]-work.L[42]*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[96]-work.L[43]*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[48]-work.L[47]*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[55]-work.L[48]*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[62]-work.L[49]*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[69]-work.L[50]*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[76]-work.L[51]*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[83]-work.L[52]*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[90]-work.L[53]*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[97]-work.L[54]*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[49]-work.L[59]*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[56]-work.L[60]*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[63]-work.L[61]*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[70]-work.L[62]*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[77]-work.L[63]*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[84]-work.L[64]*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[91]-work.L[65]*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[98]-work.L[66]*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[50]-work.L[72]*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[57]-work.L[73]*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[64]-work.L[74]*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[71]-work.L[75]*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[78]-work.L[76]*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[85]-work.L[77]*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[92]-work.L[78]*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[99]-work.L[79]*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[33]-work.L[16]*work.d[16]*1;
  residual += temp*temp;
  temp = work.KKT[35]-work.L[25]*work.d[17]*1;
  residual += temp*temp;
  temp = work.KKT[37]-work.L[35]*work.d[18]*1;
  residual += temp*temp;
  temp = work.KKT[39]-work.L[46]*work.d[19]*1;
  residual += temp*temp;
  temp = work.KKT[41]-work.L[58]*work.d[20]*1;
  residual += temp*temp;
  temp = work.KKT[43]-work.L[71]*work.d[21]*1;
  residual += temp*temp;
  return residual;
}
void matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = work.KKT[44]*source[0]+work.KKT[17]*source[22]+work.KKT[45]*source[30]+work.KKT[46]*source[31]+work.KKT[47]*source[32]+work.KKT[48]*source[33]+work.KKT[49]*source[34]+work.KKT[50]*source[35];
  result[1] = work.KKT[51]*source[1]+work.KKT[19]*source[23]+work.KKT[52]*source[30]+work.KKT[53]*source[31]+work.KKT[54]*source[32]+work.KKT[55]*source[33]+work.KKT[56]*source[34]+work.KKT[57]*source[35];
  result[2] = work.KKT[58]*source[2]+work.KKT[21]*source[24]+work.KKT[59]*source[30]+work.KKT[60]*source[31]+work.KKT[61]*source[32]+work.KKT[62]*source[33]+work.KKT[63]*source[34]+work.KKT[64]*source[35];
  result[3] = work.KKT[65]*source[3]+work.KKT[23]*source[25]+work.KKT[66]*source[30]+work.KKT[67]*source[31]+work.KKT[68]*source[32]+work.KKT[69]*source[33]+work.KKT[70]*source[34]+work.KKT[71]*source[35];
  result[4] = work.KKT[72]*source[4]+work.KKT[25]*source[26]+work.KKT[73]*source[30]+work.KKT[74]*source[31]+work.KKT[75]*source[32]+work.KKT[76]*source[33]+work.KKT[77]*source[34]+work.KKT[78]*source[35];
  result[5] = work.KKT[79]*source[5]+work.KKT[27]*source[27]+work.KKT[80]*source[30]+work.KKT[81]*source[31]+work.KKT[82]*source[32]+work.KKT[83]*source[33]+work.KKT[84]*source[34]+work.KKT[85]*source[35];
  result[6] = work.KKT[86]*source[6]+work.KKT[29]*source[28]+work.KKT[87]*source[30]+work.KKT[88]*source[31]+work.KKT[89]*source[32]+work.KKT[90]*source[33]+work.KKT[91]*source[34]+work.KKT[92]*source[35];
  result[7] = work.KKT[93]*source[7]+work.KKT[31]*source[29]+work.KKT[94]*source[30]+work.KKT[95]*source[31]+work.KKT[96]*source[32]+work.KKT[97]*source[33]+work.KKT[98]*source[34]+work.KKT[99]*source[35];
  result[8] = work.KKT[32]*source[8]+work.KKT[33]*source[30];
  result[9] = work.KKT[34]*source[9]+work.KKT[35]*source[31];
  result[10] = work.KKT[36]*source[10]+work.KKT[37]*source[32];
  result[11] = work.KKT[38]*source[11]+work.KKT[39]*source[33];
  result[12] = work.KKT[40]*source[12]+work.KKT[41]*source[34];
  result[13] = work.KKT[42]*source[13]+work.KKT[43]*source[35];
  result[14] = work.KKT[0]*source[14]+work.KKT[1]*source[22];
  result[15] = work.KKT[2]*source[15]+work.KKT[3]*source[23];
  result[16] = work.KKT[4]*source[16]+work.KKT[5]*source[24];
  result[17] = work.KKT[6]*source[17]+work.KKT[7]*source[25];
  result[18] = work.KKT[8]*source[18]+work.KKT[9]*source[26];
  result[19] = work.KKT[10]*source[19]+work.KKT[11]*source[27];
  result[20] = work.KKT[12]*source[20]+work.KKT[13]*source[28];
  result[21] = work.KKT[14]*source[21]+work.KKT[15]*source[29];
  result[22] = work.KKT[1]*source[14]+work.KKT[16]*source[22]+work.KKT[17]*source[0];
  result[23] = work.KKT[3]*source[15]+work.KKT[18]*source[23]+work.KKT[19]*source[1];
  result[24] = work.KKT[5]*source[16]+work.KKT[20]*source[24]+work.KKT[21]*source[2];
  result[25] = work.KKT[7]*source[17]+work.KKT[22]*source[25]+work.KKT[23]*source[3];
  result[26] = work.KKT[9]*source[18]+work.KKT[24]*source[26]+work.KKT[25]*source[4];
  result[27] = work.KKT[11]*source[19]+work.KKT[26]*source[27]+work.KKT[27]*source[5];
  result[28] = work.KKT[13]*source[20]+work.KKT[28]*source[28]+work.KKT[29]*source[6];
  result[29] = work.KKT[15]*source[21]+work.KKT[30]*source[29]+work.KKT[31]*source[7];
  result[30] = work.KKT[45]*source[0]+work.KKT[52]*source[1]+work.KKT[59]*source[2]+work.KKT[66]*source[3]+work.KKT[73]*source[4]+work.KKT[80]*source[5]+work.KKT[87]*source[6]+work.KKT[94]*source[7]+work.KKT[33]*source[8];
  result[31] = work.KKT[46]*source[0]+work.KKT[53]*source[1]+work.KKT[60]*source[2]+work.KKT[67]*source[3]+work.KKT[74]*source[4]+work.KKT[81]*source[5]+work.KKT[88]*source[6]+work.KKT[95]*source[7]+work.KKT[35]*source[9];
  result[32] = work.KKT[47]*source[0]+work.KKT[54]*source[1]+work.KKT[61]*source[2]+work.KKT[68]*source[3]+work.KKT[75]*source[4]+work.KKT[82]*source[5]+work.KKT[89]*source[6]+work.KKT[96]*source[7]+work.KKT[37]*source[10];
  result[33] = work.KKT[48]*source[0]+work.KKT[55]*source[1]+work.KKT[62]*source[2]+work.KKT[69]*source[3]+work.KKT[76]*source[4]+work.KKT[83]*source[5]+work.KKT[90]*source[6]+work.KKT[97]*source[7]+work.KKT[39]*source[11];
  result[34] = work.KKT[49]*source[0]+work.KKT[56]*source[1]+work.KKT[63]*source[2]+work.KKT[70]*source[3]+work.KKT[77]*source[4]+work.KKT[84]*source[5]+work.KKT[91]*source[6]+work.KKT[98]*source[7]+work.KKT[41]*source[12];
  result[35] = work.KKT[50]*source[0]+work.KKT[57]*source[1]+work.KKT[64]*source[2]+work.KKT[71]*source[3]+work.KKT[78]*source[4]+work.KKT[85]*source[5]+work.KKT[92]*source[6]+work.KKT[99]*source[7]+work.KKT[43]*source[13];
}
double check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  matrix_multiply(work.v, multiplicand);
  for (i = 0; i < 14; i++) {
    residual += (target[i] - work.v[i])*(target[i] - work.v[i]);
  }
  return residual;
}
void fill_KKT(void) {
  work.KKT[44] = 2*params.epsilon_f[0]*params.epsilon_f[0];
  work.KKT[51] = 2*params.epsilon_f[0]*params.epsilon_f[0];
  work.KKT[58] = 2*params.epsilon_f[0]*params.epsilon_f[0];
  work.KKT[65] = 2*params.epsilon_f[0]*params.epsilon_f[0];
  work.KKT[72] = 2*params.epsilon_f[0]*params.epsilon_f[0];
  work.KKT[79] = 2*params.epsilon_f[0]*params.epsilon_f[0];
  work.KKT[86] = 2*params.epsilon_f[0]*params.epsilon_f[0];
  work.KKT[93] = 2*params.epsilon_f[0]*params.epsilon_f[0];
  work.KKT[32] = 2;
  work.KKT[34] = 2;
  work.KKT[36] = 2;
  work.KKT[38] = 2;
  work.KKT[40] = 2;
  work.KKT[42] = 2;
  work.KKT[0] = work.s_inv_z[0];
  work.KKT[2] = work.s_inv_z[1];
  work.KKT[4] = work.s_inv_z[2];
  work.KKT[6] = work.s_inv_z[3];
  work.KKT[8] = work.s_inv_z[4];
  work.KKT[10] = work.s_inv_z[5];
  work.KKT[12] = work.s_inv_z[6];
  work.KKT[14] = work.s_inv_z[7];
  work.KKT[1] = 1;
  work.KKT[3] = 1;
  work.KKT[5] = 1;
  work.KKT[7] = 1;
  work.KKT[9] = 1;
  work.KKT[11] = 1;
  work.KKT[13] = 1;
  work.KKT[15] = 1;
  work.KKT[16] = work.block_33[0];
  work.KKT[18] = work.block_33[0];
  work.KKT[20] = work.block_33[0];
  work.KKT[22] = work.block_33[0];
  work.KKT[24] = work.block_33[0];
  work.KKT[26] = work.block_33[0];
  work.KKT[28] = work.block_33[0];
  work.KKT[30] = work.block_33[0];
  work.KKT[17] = -1;
  work.KKT[19] = -1;
  work.KKT[21] = -1;
  work.KKT[23] = -1;
  work.KKT[25] = -1;
  work.KKT[27] = -1;
  work.KKT[29] = -1;
  work.KKT[31] = -1;
  work.KKT[45] = params.Phi[0];
  work.KKT[52] = params.Phi[6];
  work.KKT[59] = params.Phi[12];
  work.KKT[66] = params.Phi[18];
  work.KKT[73] = params.Phi[24];
  work.KKT[80] = params.Phi[30];
  work.KKT[87] = params.Phi[36];
  work.KKT[94] = params.Phi[42];
  work.KKT[46] = params.Phi[1];
  work.KKT[53] = params.Phi[7];
  work.KKT[60] = params.Phi[13];
  work.KKT[67] = params.Phi[19];
  work.KKT[74] = params.Phi[25];
  work.KKT[81] = params.Phi[31];
  work.KKT[88] = params.Phi[37];
  work.KKT[95] = params.Phi[43];
  work.KKT[47] = params.Phi[2];
  work.KKT[54] = params.Phi[8];
  work.KKT[61] = params.Phi[14];
  work.KKT[68] = params.Phi[20];
  work.KKT[75] = params.Phi[26];
  work.KKT[82] = params.Phi[32];
  work.KKT[89] = params.Phi[38];
  work.KKT[96] = params.Phi[44];
  work.KKT[48] = params.Phi[3];
  work.KKT[55] = params.Phi[9];
  work.KKT[62] = params.Phi[15];
  work.KKT[69] = params.Phi[21];
  work.KKT[76] = params.Phi[27];
  work.KKT[83] = params.Phi[33];
  work.KKT[90] = params.Phi[39];
  work.KKT[97] = params.Phi[45];
  work.KKT[49] = params.Phi[4];
  work.KKT[56] = params.Phi[10];
  work.KKT[63] = params.Phi[16];
  work.KKT[70] = params.Phi[22];
  work.KKT[77] = params.Phi[28];
  work.KKT[84] = params.Phi[34];
  work.KKT[91] = params.Phi[40];
  work.KKT[98] = params.Phi[46];
  work.KKT[50] = params.Phi[5];
  work.KKT[57] = params.Phi[11];
  work.KKT[64] = params.Phi[17];
  work.KKT[71] = params.Phi[23];
  work.KKT[78] = params.Phi[29];
  work.KKT[85] = params.Phi[35];
  work.KKT[92] = params.Phi[41];
  work.KKT[99] = params.Phi[47];
  work.KKT[33] = -1;
  work.KKT[35] = -1;
  work.KKT[37] = -1;
  work.KKT[39] = -1;
  work.KKT[41] = -1;
  work.KKT[43] = -1;
}
