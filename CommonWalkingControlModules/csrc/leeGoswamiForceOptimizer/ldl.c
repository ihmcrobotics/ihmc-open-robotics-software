/* Produced by CVXGEN, 2013-01-16 23:01:59 +0000.  */
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
  work.v[0] = target[22];
  work.v[1] = target[23];
  work.v[2] = target[24];
  work.v[3] = target[25];
  work.v[4] = target[26];
  work.v[5] = target[27];
  work.v[6] = target[28];
  work.v[7] = target[29];
  work.v[8] = target[30];
  work.v[9] = target[31];
  work.v[10] = target[32];
  work.v[11] = target[33];
  work.v[12] = target[34];
  work.v[13] = target[35];
  work.v[14] = target[36];
  work.v[15] = target[37];
  work.v[16] = target[38]-work.L[0]*work.v[0];
  work.v[17] = target[39]-work.L[1]*work.v[1];
  work.v[18] = target[40]-work.L[2]*work.v[2];
  work.v[19] = target[41]-work.L[3]*work.v[3];
  work.v[20] = target[42]-work.L[4]*work.v[4];
  work.v[21] = target[43]-work.L[5]*work.v[5];
  work.v[22] = target[44]-work.L[6]*work.v[6];
  work.v[23] = target[45]-work.L[7]*work.v[7];
  work.v[24] = target[46]-work.L[8]*work.v[8];
  work.v[25] = target[47]-work.L[9]*work.v[9];
  work.v[26] = target[48]-work.L[10]*work.v[10];
  work.v[27] = target[49]-work.L[11]*work.v[11];
  work.v[28] = target[50]-work.L[12]*work.v[12];
  work.v[29] = target[51]-work.L[13]*work.v[13];
  work.v[30] = target[52]-work.L[14]*work.v[14];
  work.v[31] = target[53]-work.L[15]*work.v[15];
  work.v[32] = target[16];
  work.v[33] = target[17];
  work.v[34] = target[18];
  work.v[35] = target[19];
  work.v[36] = target[20];
  work.v[37] = target[21];
  work.v[38] = target[0]-work.L[16]*work.v[16];
  work.v[39] = target[1]-work.L[17]*work.v[17];
  work.v[40] = target[2]-work.L[18]*work.v[18];
  work.v[41] = target[3]-work.L[19]*work.v[19];
  work.v[42] = target[4]-work.L[20]*work.v[20];
  work.v[43] = target[5]-work.L[21]*work.v[21];
  work.v[44] = target[6]-work.L[22]*work.v[22];
  work.v[45] = target[7]-work.L[23]*work.v[23];
  work.v[46] = target[8]-work.L[24]*work.v[24];
  work.v[47] = target[9]-work.L[25]*work.v[25];
  work.v[48] = target[10]-work.L[26]*work.v[26];
  work.v[49] = target[11]-work.L[27]*work.v[27];
  work.v[50] = target[12]-work.L[28]*work.v[28];
  work.v[51] = target[13]-work.L[29]*work.v[29];
  work.v[52] = target[14]-work.L[30]*work.v[30];
  work.v[53] = target[15]-work.L[31]*work.v[31];
  work.v[54] = target[54]-work.L[32]*work.v[32]-work.L[33]*work.v[38]-work.L[34]*work.v[39]-work.L[35]*work.v[40]-work.L[36]*work.v[41]-work.L[37]*work.v[42]-work.L[38]*work.v[43]-work.L[39]*work.v[44]-work.L[40]*work.v[45]-work.L[41]*work.v[46]-work.L[42]*work.v[47]-work.L[43]*work.v[48]-work.L[44]*work.v[49]-work.L[45]*work.v[50]-work.L[46]*work.v[51]-work.L[47]*work.v[52]-work.L[48]*work.v[53];
  work.v[55] = target[55]-work.L[49]*work.v[33]-work.L[50]*work.v[38]-work.L[51]*work.v[39]-work.L[52]*work.v[40]-work.L[53]*work.v[41]-work.L[54]*work.v[42]-work.L[55]*work.v[43]-work.L[56]*work.v[44]-work.L[57]*work.v[45]-work.L[58]*work.v[46]-work.L[59]*work.v[47]-work.L[60]*work.v[48]-work.L[61]*work.v[49]-work.L[62]*work.v[50]-work.L[63]*work.v[51]-work.L[64]*work.v[52]-work.L[65]*work.v[53]-work.L[66]*work.v[54];
  work.v[56] = target[56]-work.L[67]*work.v[34]-work.L[68]*work.v[38]-work.L[69]*work.v[39]-work.L[70]*work.v[40]-work.L[71]*work.v[41]-work.L[72]*work.v[42]-work.L[73]*work.v[43]-work.L[74]*work.v[44]-work.L[75]*work.v[45]-work.L[76]*work.v[46]-work.L[77]*work.v[47]-work.L[78]*work.v[48]-work.L[79]*work.v[49]-work.L[80]*work.v[50]-work.L[81]*work.v[51]-work.L[82]*work.v[52]-work.L[83]*work.v[53]-work.L[84]*work.v[54]-work.L[85]*work.v[55];
  work.v[57] = target[57]-work.L[86]*work.v[35]-work.L[87]*work.v[38]-work.L[88]*work.v[39]-work.L[89]*work.v[40]-work.L[90]*work.v[41]-work.L[91]*work.v[42]-work.L[92]*work.v[43]-work.L[93]*work.v[44]-work.L[94]*work.v[45]-work.L[95]*work.v[46]-work.L[96]*work.v[47]-work.L[97]*work.v[48]-work.L[98]*work.v[49]-work.L[99]*work.v[50]-work.L[100]*work.v[51]-work.L[101]*work.v[52]-work.L[102]*work.v[53]-work.L[103]*work.v[54]-work.L[104]*work.v[55]-work.L[105]*work.v[56];
  work.v[58] = target[58]-work.L[106]*work.v[36]-work.L[107]*work.v[38]-work.L[108]*work.v[39]-work.L[109]*work.v[40]-work.L[110]*work.v[41]-work.L[111]*work.v[42]-work.L[112]*work.v[43]-work.L[113]*work.v[44]-work.L[114]*work.v[45]-work.L[115]*work.v[46]-work.L[116]*work.v[47]-work.L[117]*work.v[48]-work.L[118]*work.v[49]-work.L[119]*work.v[50]-work.L[120]*work.v[51]-work.L[121]*work.v[52]-work.L[122]*work.v[53]-work.L[123]*work.v[54]-work.L[124]*work.v[55]-work.L[125]*work.v[56]-work.L[126]*work.v[57];
  work.v[59] = target[59]-work.L[127]*work.v[37]-work.L[128]*work.v[38]-work.L[129]*work.v[39]-work.L[130]*work.v[40]-work.L[131]*work.v[41]-work.L[132]*work.v[42]-work.L[133]*work.v[43]-work.L[134]*work.v[44]-work.L[135]*work.v[45]-work.L[136]*work.v[46]-work.L[137]*work.v[47]-work.L[138]*work.v[48]-work.L[139]*work.v[49]-work.L[140]*work.v[50]-work.L[141]*work.v[51]-work.L[142]*work.v[52]-work.L[143]*work.v[53]-work.L[144]*work.v[54]-work.L[145]*work.v[55]-work.L[146]*work.v[56]-work.L[147]*work.v[57]-work.L[148]*work.v[58];
  /* Diagonal scaling. Assume correctness of work.d_inv. */
  for (i = 0; i < 60; i++)
    work.v[i] *= work.d_inv[i];
  /* Back substitution */
  work.v[58] -= work.L[148]*work.v[59];
  work.v[57] -= work.L[126]*work.v[58]+work.L[147]*work.v[59];
  work.v[56] -= work.L[105]*work.v[57]+work.L[125]*work.v[58]+work.L[146]*work.v[59];
  work.v[55] -= work.L[85]*work.v[56]+work.L[104]*work.v[57]+work.L[124]*work.v[58]+work.L[145]*work.v[59];
  work.v[54] -= work.L[66]*work.v[55]+work.L[84]*work.v[56]+work.L[103]*work.v[57]+work.L[123]*work.v[58]+work.L[144]*work.v[59];
  work.v[53] -= work.L[48]*work.v[54]+work.L[65]*work.v[55]+work.L[83]*work.v[56]+work.L[102]*work.v[57]+work.L[122]*work.v[58]+work.L[143]*work.v[59];
  work.v[52] -= work.L[47]*work.v[54]+work.L[64]*work.v[55]+work.L[82]*work.v[56]+work.L[101]*work.v[57]+work.L[121]*work.v[58]+work.L[142]*work.v[59];
  work.v[51] -= work.L[46]*work.v[54]+work.L[63]*work.v[55]+work.L[81]*work.v[56]+work.L[100]*work.v[57]+work.L[120]*work.v[58]+work.L[141]*work.v[59];
  work.v[50] -= work.L[45]*work.v[54]+work.L[62]*work.v[55]+work.L[80]*work.v[56]+work.L[99]*work.v[57]+work.L[119]*work.v[58]+work.L[140]*work.v[59];
  work.v[49] -= work.L[44]*work.v[54]+work.L[61]*work.v[55]+work.L[79]*work.v[56]+work.L[98]*work.v[57]+work.L[118]*work.v[58]+work.L[139]*work.v[59];
  work.v[48] -= work.L[43]*work.v[54]+work.L[60]*work.v[55]+work.L[78]*work.v[56]+work.L[97]*work.v[57]+work.L[117]*work.v[58]+work.L[138]*work.v[59];
  work.v[47] -= work.L[42]*work.v[54]+work.L[59]*work.v[55]+work.L[77]*work.v[56]+work.L[96]*work.v[57]+work.L[116]*work.v[58]+work.L[137]*work.v[59];
  work.v[46] -= work.L[41]*work.v[54]+work.L[58]*work.v[55]+work.L[76]*work.v[56]+work.L[95]*work.v[57]+work.L[115]*work.v[58]+work.L[136]*work.v[59];
  work.v[45] -= work.L[40]*work.v[54]+work.L[57]*work.v[55]+work.L[75]*work.v[56]+work.L[94]*work.v[57]+work.L[114]*work.v[58]+work.L[135]*work.v[59];
  work.v[44] -= work.L[39]*work.v[54]+work.L[56]*work.v[55]+work.L[74]*work.v[56]+work.L[93]*work.v[57]+work.L[113]*work.v[58]+work.L[134]*work.v[59];
  work.v[43] -= work.L[38]*work.v[54]+work.L[55]*work.v[55]+work.L[73]*work.v[56]+work.L[92]*work.v[57]+work.L[112]*work.v[58]+work.L[133]*work.v[59];
  work.v[42] -= work.L[37]*work.v[54]+work.L[54]*work.v[55]+work.L[72]*work.v[56]+work.L[91]*work.v[57]+work.L[111]*work.v[58]+work.L[132]*work.v[59];
  work.v[41] -= work.L[36]*work.v[54]+work.L[53]*work.v[55]+work.L[71]*work.v[56]+work.L[90]*work.v[57]+work.L[110]*work.v[58]+work.L[131]*work.v[59];
  work.v[40] -= work.L[35]*work.v[54]+work.L[52]*work.v[55]+work.L[70]*work.v[56]+work.L[89]*work.v[57]+work.L[109]*work.v[58]+work.L[130]*work.v[59];
  work.v[39] -= work.L[34]*work.v[54]+work.L[51]*work.v[55]+work.L[69]*work.v[56]+work.L[88]*work.v[57]+work.L[108]*work.v[58]+work.L[129]*work.v[59];
  work.v[38] -= work.L[33]*work.v[54]+work.L[50]*work.v[55]+work.L[68]*work.v[56]+work.L[87]*work.v[57]+work.L[107]*work.v[58]+work.L[128]*work.v[59];
  work.v[37] -= work.L[127]*work.v[59];
  work.v[36] -= work.L[106]*work.v[58];
  work.v[35] -= work.L[86]*work.v[57];
  work.v[34] -= work.L[67]*work.v[56];
  work.v[33] -= work.L[49]*work.v[55];
  work.v[32] -= work.L[32]*work.v[54];
  work.v[31] -= work.L[31]*work.v[53];
  work.v[30] -= work.L[30]*work.v[52];
  work.v[29] -= work.L[29]*work.v[51];
  work.v[28] -= work.L[28]*work.v[50];
  work.v[27] -= work.L[27]*work.v[49];
  work.v[26] -= work.L[26]*work.v[48];
  work.v[25] -= work.L[25]*work.v[47];
  work.v[24] -= work.L[24]*work.v[46];
  work.v[23] -= work.L[23]*work.v[45];
  work.v[22] -= work.L[22]*work.v[44];
  work.v[21] -= work.L[21]*work.v[43];
  work.v[20] -= work.L[20]*work.v[42];
  work.v[19] -= work.L[19]*work.v[41];
  work.v[18] -= work.L[18]*work.v[40];
  work.v[17] -= work.L[17]*work.v[39];
  work.v[16] -= work.L[16]*work.v[38];
  work.v[15] -= work.L[15]*work.v[31];
  work.v[14] -= work.L[14]*work.v[30];
  work.v[13] -= work.L[13]*work.v[29];
  work.v[12] -= work.L[12]*work.v[28];
  work.v[11] -= work.L[11]*work.v[27];
  work.v[10] -= work.L[10]*work.v[26];
  work.v[9] -= work.L[9]*work.v[25];
  work.v[8] -= work.L[8]*work.v[24];
  work.v[7] -= work.L[7]*work.v[23];
  work.v[6] -= work.L[6]*work.v[22];
  work.v[5] -= work.L[5]*work.v[21];
  work.v[4] -= work.L[4]*work.v[20];
  work.v[3] -= work.L[3]*work.v[19];
  work.v[2] -= work.L[2]*work.v[18];
  work.v[1] -= work.L[1]*work.v[17];
  work.v[0] -= work.L[0]*work.v[16];
  /* Unpermute the result, from v to var. */
  var[0] = work.v[38];
  var[1] = work.v[39];
  var[2] = work.v[40];
  var[3] = work.v[41];
  var[4] = work.v[42];
  var[5] = work.v[43];
  var[6] = work.v[44];
  var[7] = work.v[45];
  var[8] = work.v[46];
  var[9] = work.v[47];
  var[10] = work.v[48];
  var[11] = work.v[49];
  var[12] = work.v[50];
  var[13] = work.v[51];
  var[14] = work.v[52];
  var[15] = work.v[53];
  var[16] = work.v[32];
  var[17] = work.v[33];
  var[18] = work.v[34];
  var[19] = work.v[35];
  var[20] = work.v[36];
  var[21] = work.v[37];
  var[22] = work.v[0];
  var[23] = work.v[1];
  var[24] = work.v[2];
  var[25] = work.v[3];
  var[26] = work.v[4];
  var[27] = work.v[5];
  var[28] = work.v[6];
  var[29] = work.v[7];
  var[30] = work.v[8];
  var[31] = work.v[9];
  var[32] = work.v[10];
  var[33] = work.v[11];
  var[34] = work.v[12];
  var[35] = work.v[13];
  var[36] = work.v[14];
  var[37] = work.v[15];
  var[38] = work.v[16];
  var[39] = work.v[17];
  var[40] = work.v[18];
  var[41] = work.v[19];
  var[42] = work.v[20];
  var[43] = work.v[21];
  var[44] = work.v[22];
  var[45] = work.v[23];
  var[46] = work.v[24];
  var[47] = work.v[25];
  var[48] = work.v[26];
  var[49] = work.v[27];
  var[50] = work.v[28];
  var[51] = work.v[29];
  var[52] = work.v[30];
  var[53] = work.v[31];
  var[54] = work.v[54];
  var[55] = work.v[55];
  var[56] = work.v[56];
  var[57] = work.v[57];
  var[58] = work.v[58];
  var[59] = work.v[59];
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
  work.v[8] = work.KKT[16];
  work.d[8] = work.v[8];
  if (work.d[8] < 0)
    work.d[8] = settings.kkt_reg;
  else
    work.d[8] += settings.kkt_reg;
  work.d_inv[8] = 1/work.d[8];
  work.L[8] = (work.KKT[17])*work.d_inv[8];
  work.v[9] = work.KKT[18];
  work.d[9] = work.v[9];
  if (work.d[9] < 0)
    work.d[9] = settings.kkt_reg;
  else
    work.d[9] += settings.kkt_reg;
  work.d_inv[9] = 1/work.d[9];
  work.L[9] = (work.KKT[19])*work.d_inv[9];
  work.v[10] = work.KKT[20];
  work.d[10] = work.v[10];
  if (work.d[10] < 0)
    work.d[10] = settings.kkt_reg;
  else
    work.d[10] += settings.kkt_reg;
  work.d_inv[10] = 1/work.d[10];
  work.L[10] = (work.KKT[21])*work.d_inv[10];
  work.v[11] = work.KKT[22];
  work.d[11] = work.v[11];
  if (work.d[11] < 0)
    work.d[11] = settings.kkt_reg;
  else
    work.d[11] += settings.kkt_reg;
  work.d_inv[11] = 1/work.d[11];
  work.L[11] = (work.KKT[23])*work.d_inv[11];
  work.v[12] = work.KKT[24];
  work.d[12] = work.v[12];
  if (work.d[12] < 0)
    work.d[12] = settings.kkt_reg;
  else
    work.d[12] += settings.kkt_reg;
  work.d_inv[12] = 1/work.d[12];
  work.L[12] = (work.KKT[25])*work.d_inv[12];
  work.v[13] = work.KKT[26];
  work.d[13] = work.v[13];
  if (work.d[13] < 0)
    work.d[13] = settings.kkt_reg;
  else
    work.d[13] += settings.kkt_reg;
  work.d_inv[13] = 1/work.d[13];
  work.L[13] = (work.KKT[27])*work.d_inv[13];
  work.v[14] = work.KKT[28];
  work.d[14] = work.v[14];
  if (work.d[14] < 0)
    work.d[14] = settings.kkt_reg;
  else
    work.d[14] += settings.kkt_reg;
  work.d_inv[14] = 1/work.d[14];
  work.L[14] = (work.KKT[29])*work.d_inv[14];
  work.v[15] = work.KKT[30];
  work.d[15] = work.v[15];
  if (work.d[15] < 0)
    work.d[15] = settings.kkt_reg;
  else
    work.d[15] += settings.kkt_reg;
  work.d_inv[15] = 1/work.d[15];
  work.L[15] = (work.KKT[31])*work.d_inv[15];
  work.v[0] = work.L[0]*work.d[0];
  work.v[16] = work.KKT[32]-work.L[0]*work.v[0];
  work.d[16] = work.v[16];
  if (work.d[16] > 0)
    work.d[16] = -settings.kkt_reg;
  else
    work.d[16] -= settings.kkt_reg;
  work.d_inv[16] = 1/work.d[16];
  work.L[16] = (work.KKT[33])*work.d_inv[16];
  work.v[1] = work.L[1]*work.d[1];
  work.v[17] = work.KKT[34]-work.L[1]*work.v[1];
  work.d[17] = work.v[17];
  if (work.d[17] > 0)
    work.d[17] = -settings.kkt_reg;
  else
    work.d[17] -= settings.kkt_reg;
  work.d_inv[17] = 1/work.d[17];
  work.L[17] = (work.KKT[35])*work.d_inv[17];
  work.v[2] = work.L[2]*work.d[2];
  work.v[18] = work.KKT[36]-work.L[2]*work.v[2];
  work.d[18] = work.v[18];
  if (work.d[18] > 0)
    work.d[18] = -settings.kkt_reg;
  else
    work.d[18] -= settings.kkt_reg;
  work.d_inv[18] = 1/work.d[18];
  work.L[18] = (work.KKT[37])*work.d_inv[18];
  work.v[3] = work.L[3]*work.d[3];
  work.v[19] = work.KKT[38]-work.L[3]*work.v[3];
  work.d[19] = work.v[19];
  if (work.d[19] > 0)
    work.d[19] = -settings.kkt_reg;
  else
    work.d[19] -= settings.kkt_reg;
  work.d_inv[19] = 1/work.d[19];
  work.L[19] = (work.KKT[39])*work.d_inv[19];
  work.v[4] = work.L[4]*work.d[4];
  work.v[20] = work.KKT[40]-work.L[4]*work.v[4];
  work.d[20] = work.v[20];
  if (work.d[20] > 0)
    work.d[20] = -settings.kkt_reg;
  else
    work.d[20] -= settings.kkt_reg;
  work.d_inv[20] = 1/work.d[20];
  work.L[20] = (work.KKT[41])*work.d_inv[20];
  work.v[5] = work.L[5]*work.d[5];
  work.v[21] = work.KKT[42]-work.L[5]*work.v[5];
  work.d[21] = work.v[21];
  if (work.d[21] > 0)
    work.d[21] = -settings.kkt_reg;
  else
    work.d[21] -= settings.kkt_reg;
  work.d_inv[21] = 1/work.d[21];
  work.L[21] = (work.KKT[43])*work.d_inv[21];
  work.v[6] = work.L[6]*work.d[6];
  work.v[22] = work.KKT[44]-work.L[6]*work.v[6];
  work.d[22] = work.v[22];
  if (work.d[22] > 0)
    work.d[22] = -settings.kkt_reg;
  else
    work.d[22] -= settings.kkt_reg;
  work.d_inv[22] = 1/work.d[22];
  work.L[22] = (work.KKT[45])*work.d_inv[22];
  work.v[7] = work.L[7]*work.d[7];
  work.v[23] = work.KKT[46]-work.L[7]*work.v[7];
  work.d[23] = work.v[23];
  if (work.d[23] > 0)
    work.d[23] = -settings.kkt_reg;
  else
    work.d[23] -= settings.kkt_reg;
  work.d_inv[23] = 1/work.d[23];
  work.L[23] = (work.KKT[47])*work.d_inv[23];
  work.v[8] = work.L[8]*work.d[8];
  work.v[24] = work.KKT[48]-work.L[8]*work.v[8];
  work.d[24] = work.v[24];
  if (work.d[24] > 0)
    work.d[24] = -settings.kkt_reg;
  else
    work.d[24] -= settings.kkt_reg;
  work.d_inv[24] = 1/work.d[24];
  work.L[24] = (work.KKT[49])*work.d_inv[24];
  work.v[9] = work.L[9]*work.d[9];
  work.v[25] = work.KKT[50]-work.L[9]*work.v[9];
  work.d[25] = work.v[25];
  if (work.d[25] > 0)
    work.d[25] = -settings.kkt_reg;
  else
    work.d[25] -= settings.kkt_reg;
  work.d_inv[25] = 1/work.d[25];
  work.L[25] = (work.KKT[51])*work.d_inv[25];
  work.v[10] = work.L[10]*work.d[10];
  work.v[26] = work.KKT[52]-work.L[10]*work.v[10];
  work.d[26] = work.v[26];
  if (work.d[26] > 0)
    work.d[26] = -settings.kkt_reg;
  else
    work.d[26] -= settings.kkt_reg;
  work.d_inv[26] = 1/work.d[26];
  work.L[26] = (work.KKT[53])*work.d_inv[26];
  work.v[11] = work.L[11]*work.d[11];
  work.v[27] = work.KKT[54]-work.L[11]*work.v[11];
  work.d[27] = work.v[27];
  if (work.d[27] > 0)
    work.d[27] = -settings.kkt_reg;
  else
    work.d[27] -= settings.kkt_reg;
  work.d_inv[27] = 1/work.d[27];
  work.L[27] = (work.KKT[55])*work.d_inv[27];
  work.v[12] = work.L[12]*work.d[12];
  work.v[28] = work.KKT[56]-work.L[12]*work.v[12];
  work.d[28] = work.v[28];
  if (work.d[28] > 0)
    work.d[28] = -settings.kkt_reg;
  else
    work.d[28] -= settings.kkt_reg;
  work.d_inv[28] = 1/work.d[28];
  work.L[28] = (work.KKT[57])*work.d_inv[28];
  work.v[13] = work.L[13]*work.d[13];
  work.v[29] = work.KKT[58]-work.L[13]*work.v[13];
  work.d[29] = work.v[29];
  if (work.d[29] > 0)
    work.d[29] = -settings.kkt_reg;
  else
    work.d[29] -= settings.kkt_reg;
  work.d_inv[29] = 1/work.d[29];
  work.L[29] = (work.KKT[59])*work.d_inv[29];
  work.v[14] = work.L[14]*work.d[14];
  work.v[30] = work.KKT[60]-work.L[14]*work.v[14];
  work.d[30] = work.v[30];
  if (work.d[30] > 0)
    work.d[30] = -settings.kkt_reg;
  else
    work.d[30] -= settings.kkt_reg;
  work.d_inv[30] = 1/work.d[30];
  work.L[30] = (work.KKT[61])*work.d_inv[30];
  work.v[15] = work.L[15]*work.d[15];
  work.v[31] = work.KKT[62]-work.L[15]*work.v[15];
  work.d[31] = work.v[31];
  if (work.d[31] > 0)
    work.d[31] = -settings.kkt_reg;
  else
    work.d[31] -= settings.kkt_reg;
  work.d_inv[31] = 1/work.d[31];
  work.L[31] = (work.KKT[63])*work.d_inv[31];
  work.v[32] = work.KKT[64];
  work.d[32] = work.v[32];
  if (work.d[32] < 0)
    work.d[32] = settings.kkt_reg;
  else
    work.d[32] += settings.kkt_reg;
  work.d_inv[32] = 1/work.d[32];
  work.L[32] = (work.KKT[65])*work.d_inv[32];
  work.v[33] = work.KKT[66];
  work.d[33] = work.v[33];
  if (work.d[33] < 0)
    work.d[33] = settings.kkt_reg;
  else
    work.d[33] += settings.kkt_reg;
  work.d_inv[33] = 1/work.d[33];
  work.L[49] = (work.KKT[67])*work.d_inv[33];
  work.v[34] = work.KKT[68];
  work.d[34] = work.v[34];
  if (work.d[34] < 0)
    work.d[34] = settings.kkt_reg;
  else
    work.d[34] += settings.kkt_reg;
  work.d_inv[34] = 1/work.d[34];
  work.L[67] = (work.KKT[69])*work.d_inv[34];
  work.v[35] = work.KKT[70];
  work.d[35] = work.v[35];
  if (work.d[35] < 0)
    work.d[35] = settings.kkt_reg;
  else
    work.d[35] += settings.kkt_reg;
  work.d_inv[35] = 1/work.d[35];
  work.L[86] = (work.KKT[71])*work.d_inv[35];
  work.v[36] = work.KKT[72];
  work.d[36] = work.v[36];
  if (work.d[36] < 0)
    work.d[36] = settings.kkt_reg;
  else
    work.d[36] += settings.kkt_reg;
  work.d_inv[36] = 1/work.d[36];
  work.L[106] = (work.KKT[73])*work.d_inv[36];
  work.v[37] = work.KKT[74];
  work.d[37] = work.v[37];
  if (work.d[37] < 0)
    work.d[37] = settings.kkt_reg;
  else
    work.d[37] += settings.kkt_reg;
  work.d_inv[37] = 1/work.d[37];
  work.L[127] = (work.KKT[75])*work.d_inv[37];
  work.v[16] = work.L[16]*work.d[16];
  work.v[38] = work.KKT[76]-work.L[16]*work.v[16];
  work.d[38] = work.v[38];
  if (work.d[38] < 0)
    work.d[38] = settings.kkt_reg;
  else
    work.d[38] += settings.kkt_reg;
  work.d_inv[38] = 1/work.d[38];
  work.L[33] = (work.KKT[77])*work.d_inv[38];
  work.L[50] = (work.KKT[78])*work.d_inv[38];
  work.L[68] = (work.KKT[79])*work.d_inv[38];
  work.L[87] = (work.KKT[80])*work.d_inv[38];
  work.L[107] = (work.KKT[81])*work.d_inv[38];
  work.L[128] = (work.KKT[82])*work.d_inv[38];
  work.v[17] = work.L[17]*work.d[17];
  work.v[39] = work.KKT[83]-work.L[17]*work.v[17];
  work.d[39] = work.v[39];
  if (work.d[39] < 0)
    work.d[39] = settings.kkt_reg;
  else
    work.d[39] += settings.kkt_reg;
  work.d_inv[39] = 1/work.d[39];
  work.L[34] = (work.KKT[84])*work.d_inv[39];
  work.L[51] = (work.KKT[85])*work.d_inv[39];
  work.L[69] = (work.KKT[86])*work.d_inv[39];
  work.L[88] = (work.KKT[87])*work.d_inv[39];
  work.L[108] = (work.KKT[88])*work.d_inv[39];
  work.L[129] = (work.KKT[89])*work.d_inv[39];
  work.v[18] = work.L[18]*work.d[18];
  work.v[40] = work.KKT[90]-work.L[18]*work.v[18];
  work.d[40] = work.v[40];
  if (work.d[40] < 0)
    work.d[40] = settings.kkt_reg;
  else
    work.d[40] += settings.kkt_reg;
  work.d_inv[40] = 1/work.d[40];
  work.L[35] = (work.KKT[91])*work.d_inv[40];
  work.L[52] = (work.KKT[92])*work.d_inv[40];
  work.L[70] = (work.KKT[93])*work.d_inv[40];
  work.L[89] = (work.KKT[94])*work.d_inv[40];
  work.L[109] = (work.KKT[95])*work.d_inv[40];
  work.L[130] = (work.KKT[96])*work.d_inv[40];
  work.v[19] = work.L[19]*work.d[19];
  work.v[41] = work.KKT[97]-work.L[19]*work.v[19];
  work.d[41] = work.v[41];
  if (work.d[41] < 0)
    work.d[41] = settings.kkt_reg;
  else
    work.d[41] += settings.kkt_reg;
  work.d_inv[41] = 1/work.d[41];
  work.L[36] = (work.KKT[98])*work.d_inv[41];
  work.L[53] = (work.KKT[99])*work.d_inv[41];
  work.L[71] = (work.KKT[100])*work.d_inv[41];
  work.L[90] = (work.KKT[101])*work.d_inv[41];
  work.L[110] = (work.KKT[102])*work.d_inv[41];
  work.L[131] = (work.KKT[103])*work.d_inv[41];
  work.v[20] = work.L[20]*work.d[20];
  work.v[42] = work.KKT[104]-work.L[20]*work.v[20];
  work.d[42] = work.v[42];
  if (work.d[42] < 0)
    work.d[42] = settings.kkt_reg;
  else
    work.d[42] += settings.kkt_reg;
  work.d_inv[42] = 1/work.d[42];
  work.L[37] = (work.KKT[105])*work.d_inv[42];
  work.L[54] = (work.KKT[106])*work.d_inv[42];
  work.L[72] = (work.KKT[107])*work.d_inv[42];
  work.L[91] = (work.KKT[108])*work.d_inv[42];
  work.L[111] = (work.KKT[109])*work.d_inv[42];
  work.L[132] = (work.KKT[110])*work.d_inv[42];
  work.v[21] = work.L[21]*work.d[21];
  work.v[43] = work.KKT[111]-work.L[21]*work.v[21];
  work.d[43] = work.v[43];
  if (work.d[43] < 0)
    work.d[43] = settings.kkt_reg;
  else
    work.d[43] += settings.kkt_reg;
  work.d_inv[43] = 1/work.d[43];
  work.L[38] = (work.KKT[112])*work.d_inv[43];
  work.L[55] = (work.KKT[113])*work.d_inv[43];
  work.L[73] = (work.KKT[114])*work.d_inv[43];
  work.L[92] = (work.KKT[115])*work.d_inv[43];
  work.L[112] = (work.KKT[116])*work.d_inv[43];
  work.L[133] = (work.KKT[117])*work.d_inv[43];
  work.v[22] = work.L[22]*work.d[22];
  work.v[44] = work.KKT[118]-work.L[22]*work.v[22];
  work.d[44] = work.v[44];
  if (work.d[44] < 0)
    work.d[44] = settings.kkt_reg;
  else
    work.d[44] += settings.kkt_reg;
  work.d_inv[44] = 1/work.d[44];
  work.L[39] = (work.KKT[119])*work.d_inv[44];
  work.L[56] = (work.KKT[120])*work.d_inv[44];
  work.L[74] = (work.KKT[121])*work.d_inv[44];
  work.L[93] = (work.KKT[122])*work.d_inv[44];
  work.L[113] = (work.KKT[123])*work.d_inv[44];
  work.L[134] = (work.KKT[124])*work.d_inv[44];
  work.v[23] = work.L[23]*work.d[23];
  work.v[45] = work.KKT[125]-work.L[23]*work.v[23];
  work.d[45] = work.v[45];
  if (work.d[45] < 0)
    work.d[45] = settings.kkt_reg;
  else
    work.d[45] += settings.kkt_reg;
  work.d_inv[45] = 1/work.d[45];
  work.L[40] = (work.KKT[126])*work.d_inv[45];
  work.L[57] = (work.KKT[127])*work.d_inv[45];
  work.L[75] = (work.KKT[128])*work.d_inv[45];
  work.L[94] = (work.KKT[129])*work.d_inv[45];
  work.L[114] = (work.KKT[130])*work.d_inv[45];
  work.L[135] = (work.KKT[131])*work.d_inv[45];
  work.v[24] = work.L[24]*work.d[24];
  work.v[46] = work.KKT[132]-work.L[24]*work.v[24];
  work.d[46] = work.v[46];
  if (work.d[46] < 0)
    work.d[46] = settings.kkt_reg;
  else
    work.d[46] += settings.kkt_reg;
  work.d_inv[46] = 1/work.d[46];
  work.L[41] = (work.KKT[133])*work.d_inv[46];
  work.L[58] = (work.KKT[134])*work.d_inv[46];
  work.L[76] = (work.KKT[135])*work.d_inv[46];
  work.L[95] = (work.KKT[136])*work.d_inv[46];
  work.L[115] = (work.KKT[137])*work.d_inv[46];
  work.L[136] = (work.KKT[138])*work.d_inv[46];
  work.v[25] = work.L[25]*work.d[25];
  work.v[47] = work.KKT[139]-work.L[25]*work.v[25];
  work.d[47] = work.v[47];
  if (work.d[47] < 0)
    work.d[47] = settings.kkt_reg;
  else
    work.d[47] += settings.kkt_reg;
  work.d_inv[47] = 1/work.d[47];
  work.L[42] = (work.KKT[140])*work.d_inv[47];
  work.L[59] = (work.KKT[141])*work.d_inv[47];
  work.L[77] = (work.KKT[142])*work.d_inv[47];
  work.L[96] = (work.KKT[143])*work.d_inv[47];
  work.L[116] = (work.KKT[144])*work.d_inv[47];
  work.L[137] = (work.KKT[145])*work.d_inv[47];
  work.v[26] = work.L[26]*work.d[26];
  work.v[48] = work.KKT[146]-work.L[26]*work.v[26];
  work.d[48] = work.v[48];
  if (work.d[48] < 0)
    work.d[48] = settings.kkt_reg;
  else
    work.d[48] += settings.kkt_reg;
  work.d_inv[48] = 1/work.d[48];
  work.L[43] = (work.KKT[147])*work.d_inv[48];
  work.L[60] = (work.KKT[148])*work.d_inv[48];
  work.L[78] = (work.KKT[149])*work.d_inv[48];
  work.L[97] = (work.KKT[150])*work.d_inv[48];
  work.L[117] = (work.KKT[151])*work.d_inv[48];
  work.L[138] = (work.KKT[152])*work.d_inv[48];
  work.v[27] = work.L[27]*work.d[27];
  work.v[49] = work.KKT[153]-work.L[27]*work.v[27];
  work.d[49] = work.v[49];
  if (work.d[49] < 0)
    work.d[49] = settings.kkt_reg;
  else
    work.d[49] += settings.kkt_reg;
  work.d_inv[49] = 1/work.d[49];
  work.L[44] = (work.KKT[154])*work.d_inv[49];
  work.L[61] = (work.KKT[155])*work.d_inv[49];
  work.L[79] = (work.KKT[156])*work.d_inv[49];
  work.L[98] = (work.KKT[157])*work.d_inv[49];
  work.L[118] = (work.KKT[158])*work.d_inv[49];
  work.L[139] = (work.KKT[159])*work.d_inv[49];
  work.v[28] = work.L[28]*work.d[28];
  work.v[50] = work.KKT[160]-work.L[28]*work.v[28];
  work.d[50] = work.v[50];
  if (work.d[50] < 0)
    work.d[50] = settings.kkt_reg;
  else
    work.d[50] += settings.kkt_reg;
  work.d_inv[50] = 1/work.d[50];
  work.L[45] = (work.KKT[161])*work.d_inv[50];
  work.L[62] = (work.KKT[162])*work.d_inv[50];
  work.L[80] = (work.KKT[163])*work.d_inv[50];
  work.L[99] = (work.KKT[164])*work.d_inv[50];
  work.L[119] = (work.KKT[165])*work.d_inv[50];
  work.L[140] = (work.KKT[166])*work.d_inv[50];
  work.v[29] = work.L[29]*work.d[29];
  work.v[51] = work.KKT[167]-work.L[29]*work.v[29];
  work.d[51] = work.v[51];
  if (work.d[51] < 0)
    work.d[51] = settings.kkt_reg;
  else
    work.d[51] += settings.kkt_reg;
  work.d_inv[51] = 1/work.d[51];
  work.L[46] = (work.KKT[168])*work.d_inv[51];
  work.L[63] = (work.KKT[169])*work.d_inv[51];
  work.L[81] = (work.KKT[170])*work.d_inv[51];
  work.L[100] = (work.KKT[171])*work.d_inv[51];
  work.L[120] = (work.KKT[172])*work.d_inv[51];
  work.L[141] = (work.KKT[173])*work.d_inv[51];
  work.v[30] = work.L[30]*work.d[30];
  work.v[52] = work.KKT[174]-work.L[30]*work.v[30];
  work.d[52] = work.v[52];
  if (work.d[52] < 0)
    work.d[52] = settings.kkt_reg;
  else
    work.d[52] += settings.kkt_reg;
  work.d_inv[52] = 1/work.d[52];
  work.L[47] = (work.KKT[175])*work.d_inv[52];
  work.L[64] = (work.KKT[176])*work.d_inv[52];
  work.L[82] = (work.KKT[177])*work.d_inv[52];
  work.L[101] = (work.KKT[178])*work.d_inv[52];
  work.L[121] = (work.KKT[179])*work.d_inv[52];
  work.L[142] = (work.KKT[180])*work.d_inv[52];
  work.v[31] = work.L[31]*work.d[31];
  work.v[53] = work.KKT[181]-work.L[31]*work.v[31];
  work.d[53] = work.v[53];
  if (work.d[53] < 0)
    work.d[53] = settings.kkt_reg;
  else
    work.d[53] += settings.kkt_reg;
  work.d_inv[53] = 1/work.d[53];
  work.L[48] = (work.KKT[182])*work.d_inv[53];
  work.L[65] = (work.KKT[183])*work.d_inv[53];
  work.L[83] = (work.KKT[184])*work.d_inv[53];
  work.L[102] = (work.KKT[185])*work.d_inv[53];
  work.L[122] = (work.KKT[186])*work.d_inv[53];
  work.L[143] = (work.KKT[187])*work.d_inv[53];
  work.v[32] = work.L[32]*work.d[32];
  work.v[38] = work.L[33]*work.d[38];
  work.v[39] = work.L[34]*work.d[39];
  work.v[40] = work.L[35]*work.d[40];
  work.v[41] = work.L[36]*work.d[41];
  work.v[42] = work.L[37]*work.d[42];
  work.v[43] = work.L[38]*work.d[43];
  work.v[44] = work.L[39]*work.d[44];
  work.v[45] = work.L[40]*work.d[45];
  work.v[46] = work.L[41]*work.d[46];
  work.v[47] = work.L[42]*work.d[47];
  work.v[48] = work.L[43]*work.d[48];
  work.v[49] = work.L[44]*work.d[49];
  work.v[50] = work.L[45]*work.d[50];
  work.v[51] = work.L[46]*work.d[51];
  work.v[52] = work.L[47]*work.d[52];
  work.v[53] = work.L[48]*work.d[53];
  work.v[54] = 0-work.L[32]*work.v[32]-work.L[33]*work.v[38]-work.L[34]*work.v[39]-work.L[35]*work.v[40]-work.L[36]*work.v[41]-work.L[37]*work.v[42]-work.L[38]*work.v[43]-work.L[39]*work.v[44]-work.L[40]*work.v[45]-work.L[41]*work.v[46]-work.L[42]*work.v[47]-work.L[43]*work.v[48]-work.L[44]*work.v[49]-work.L[45]*work.v[50]-work.L[46]*work.v[51]-work.L[47]*work.v[52]-work.L[48]*work.v[53];
  work.d[54] = work.v[54];
  if (work.d[54] > 0)
    work.d[54] = -settings.kkt_reg;
  else
    work.d[54] -= settings.kkt_reg;
  work.d_inv[54] = 1/work.d[54];
  work.L[66] = (-work.L[50]*work.v[38]-work.L[51]*work.v[39]-work.L[52]*work.v[40]-work.L[53]*work.v[41]-work.L[54]*work.v[42]-work.L[55]*work.v[43]-work.L[56]*work.v[44]-work.L[57]*work.v[45]-work.L[58]*work.v[46]-work.L[59]*work.v[47]-work.L[60]*work.v[48]-work.L[61]*work.v[49]-work.L[62]*work.v[50]-work.L[63]*work.v[51]-work.L[64]*work.v[52]-work.L[65]*work.v[53])*work.d_inv[54];
  work.L[84] = (-work.L[68]*work.v[38]-work.L[69]*work.v[39]-work.L[70]*work.v[40]-work.L[71]*work.v[41]-work.L[72]*work.v[42]-work.L[73]*work.v[43]-work.L[74]*work.v[44]-work.L[75]*work.v[45]-work.L[76]*work.v[46]-work.L[77]*work.v[47]-work.L[78]*work.v[48]-work.L[79]*work.v[49]-work.L[80]*work.v[50]-work.L[81]*work.v[51]-work.L[82]*work.v[52]-work.L[83]*work.v[53])*work.d_inv[54];
  work.L[103] = (-work.L[87]*work.v[38]-work.L[88]*work.v[39]-work.L[89]*work.v[40]-work.L[90]*work.v[41]-work.L[91]*work.v[42]-work.L[92]*work.v[43]-work.L[93]*work.v[44]-work.L[94]*work.v[45]-work.L[95]*work.v[46]-work.L[96]*work.v[47]-work.L[97]*work.v[48]-work.L[98]*work.v[49]-work.L[99]*work.v[50]-work.L[100]*work.v[51]-work.L[101]*work.v[52]-work.L[102]*work.v[53])*work.d_inv[54];
  work.L[123] = (-work.L[107]*work.v[38]-work.L[108]*work.v[39]-work.L[109]*work.v[40]-work.L[110]*work.v[41]-work.L[111]*work.v[42]-work.L[112]*work.v[43]-work.L[113]*work.v[44]-work.L[114]*work.v[45]-work.L[115]*work.v[46]-work.L[116]*work.v[47]-work.L[117]*work.v[48]-work.L[118]*work.v[49]-work.L[119]*work.v[50]-work.L[120]*work.v[51]-work.L[121]*work.v[52]-work.L[122]*work.v[53])*work.d_inv[54];
  work.L[144] = (-work.L[128]*work.v[38]-work.L[129]*work.v[39]-work.L[130]*work.v[40]-work.L[131]*work.v[41]-work.L[132]*work.v[42]-work.L[133]*work.v[43]-work.L[134]*work.v[44]-work.L[135]*work.v[45]-work.L[136]*work.v[46]-work.L[137]*work.v[47]-work.L[138]*work.v[48]-work.L[139]*work.v[49]-work.L[140]*work.v[50]-work.L[141]*work.v[51]-work.L[142]*work.v[52]-work.L[143]*work.v[53])*work.d_inv[54];
  work.v[33] = work.L[49]*work.d[33];
  work.v[38] = work.L[50]*work.d[38];
  work.v[39] = work.L[51]*work.d[39];
  work.v[40] = work.L[52]*work.d[40];
  work.v[41] = work.L[53]*work.d[41];
  work.v[42] = work.L[54]*work.d[42];
  work.v[43] = work.L[55]*work.d[43];
  work.v[44] = work.L[56]*work.d[44];
  work.v[45] = work.L[57]*work.d[45];
  work.v[46] = work.L[58]*work.d[46];
  work.v[47] = work.L[59]*work.d[47];
  work.v[48] = work.L[60]*work.d[48];
  work.v[49] = work.L[61]*work.d[49];
  work.v[50] = work.L[62]*work.d[50];
  work.v[51] = work.L[63]*work.d[51];
  work.v[52] = work.L[64]*work.d[52];
  work.v[53] = work.L[65]*work.d[53];
  work.v[54] = work.L[66]*work.d[54];
  work.v[55] = 0-work.L[49]*work.v[33]-work.L[50]*work.v[38]-work.L[51]*work.v[39]-work.L[52]*work.v[40]-work.L[53]*work.v[41]-work.L[54]*work.v[42]-work.L[55]*work.v[43]-work.L[56]*work.v[44]-work.L[57]*work.v[45]-work.L[58]*work.v[46]-work.L[59]*work.v[47]-work.L[60]*work.v[48]-work.L[61]*work.v[49]-work.L[62]*work.v[50]-work.L[63]*work.v[51]-work.L[64]*work.v[52]-work.L[65]*work.v[53]-work.L[66]*work.v[54];
  work.d[55] = work.v[55];
  if (work.d[55] > 0)
    work.d[55] = -settings.kkt_reg;
  else
    work.d[55] -= settings.kkt_reg;
  work.d_inv[55] = 1/work.d[55];
  work.L[85] = (-work.L[68]*work.v[38]-work.L[69]*work.v[39]-work.L[70]*work.v[40]-work.L[71]*work.v[41]-work.L[72]*work.v[42]-work.L[73]*work.v[43]-work.L[74]*work.v[44]-work.L[75]*work.v[45]-work.L[76]*work.v[46]-work.L[77]*work.v[47]-work.L[78]*work.v[48]-work.L[79]*work.v[49]-work.L[80]*work.v[50]-work.L[81]*work.v[51]-work.L[82]*work.v[52]-work.L[83]*work.v[53]-work.L[84]*work.v[54])*work.d_inv[55];
  work.L[104] = (-work.L[87]*work.v[38]-work.L[88]*work.v[39]-work.L[89]*work.v[40]-work.L[90]*work.v[41]-work.L[91]*work.v[42]-work.L[92]*work.v[43]-work.L[93]*work.v[44]-work.L[94]*work.v[45]-work.L[95]*work.v[46]-work.L[96]*work.v[47]-work.L[97]*work.v[48]-work.L[98]*work.v[49]-work.L[99]*work.v[50]-work.L[100]*work.v[51]-work.L[101]*work.v[52]-work.L[102]*work.v[53]-work.L[103]*work.v[54])*work.d_inv[55];
  work.L[124] = (-work.L[107]*work.v[38]-work.L[108]*work.v[39]-work.L[109]*work.v[40]-work.L[110]*work.v[41]-work.L[111]*work.v[42]-work.L[112]*work.v[43]-work.L[113]*work.v[44]-work.L[114]*work.v[45]-work.L[115]*work.v[46]-work.L[116]*work.v[47]-work.L[117]*work.v[48]-work.L[118]*work.v[49]-work.L[119]*work.v[50]-work.L[120]*work.v[51]-work.L[121]*work.v[52]-work.L[122]*work.v[53]-work.L[123]*work.v[54])*work.d_inv[55];
  work.L[145] = (-work.L[128]*work.v[38]-work.L[129]*work.v[39]-work.L[130]*work.v[40]-work.L[131]*work.v[41]-work.L[132]*work.v[42]-work.L[133]*work.v[43]-work.L[134]*work.v[44]-work.L[135]*work.v[45]-work.L[136]*work.v[46]-work.L[137]*work.v[47]-work.L[138]*work.v[48]-work.L[139]*work.v[49]-work.L[140]*work.v[50]-work.L[141]*work.v[51]-work.L[142]*work.v[52]-work.L[143]*work.v[53]-work.L[144]*work.v[54])*work.d_inv[55];
  work.v[34] = work.L[67]*work.d[34];
  work.v[38] = work.L[68]*work.d[38];
  work.v[39] = work.L[69]*work.d[39];
  work.v[40] = work.L[70]*work.d[40];
  work.v[41] = work.L[71]*work.d[41];
  work.v[42] = work.L[72]*work.d[42];
  work.v[43] = work.L[73]*work.d[43];
  work.v[44] = work.L[74]*work.d[44];
  work.v[45] = work.L[75]*work.d[45];
  work.v[46] = work.L[76]*work.d[46];
  work.v[47] = work.L[77]*work.d[47];
  work.v[48] = work.L[78]*work.d[48];
  work.v[49] = work.L[79]*work.d[49];
  work.v[50] = work.L[80]*work.d[50];
  work.v[51] = work.L[81]*work.d[51];
  work.v[52] = work.L[82]*work.d[52];
  work.v[53] = work.L[83]*work.d[53];
  work.v[54] = work.L[84]*work.d[54];
  work.v[55] = work.L[85]*work.d[55];
  work.v[56] = 0-work.L[67]*work.v[34]-work.L[68]*work.v[38]-work.L[69]*work.v[39]-work.L[70]*work.v[40]-work.L[71]*work.v[41]-work.L[72]*work.v[42]-work.L[73]*work.v[43]-work.L[74]*work.v[44]-work.L[75]*work.v[45]-work.L[76]*work.v[46]-work.L[77]*work.v[47]-work.L[78]*work.v[48]-work.L[79]*work.v[49]-work.L[80]*work.v[50]-work.L[81]*work.v[51]-work.L[82]*work.v[52]-work.L[83]*work.v[53]-work.L[84]*work.v[54]-work.L[85]*work.v[55];
  work.d[56] = work.v[56];
  if (work.d[56] > 0)
    work.d[56] = -settings.kkt_reg;
  else
    work.d[56] -= settings.kkt_reg;
  work.d_inv[56] = 1/work.d[56];
  work.L[105] = (-work.L[87]*work.v[38]-work.L[88]*work.v[39]-work.L[89]*work.v[40]-work.L[90]*work.v[41]-work.L[91]*work.v[42]-work.L[92]*work.v[43]-work.L[93]*work.v[44]-work.L[94]*work.v[45]-work.L[95]*work.v[46]-work.L[96]*work.v[47]-work.L[97]*work.v[48]-work.L[98]*work.v[49]-work.L[99]*work.v[50]-work.L[100]*work.v[51]-work.L[101]*work.v[52]-work.L[102]*work.v[53]-work.L[103]*work.v[54]-work.L[104]*work.v[55])*work.d_inv[56];
  work.L[125] = (-work.L[107]*work.v[38]-work.L[108]*work.v[39]-work.L[109]*work.v[40]-work.L[110]*work.v[41]-work.L[111]*work.v[42]-work.L[112]*work.v[43]-work.L[113]*work.v[44]-work.L[114]*work.v[45]-work.L[115]*work.v[46]-work.L[116]*work.v[47]-work.L[117]*work.v[48]-work.L[118]*work.v[49]-work.L[119]*work.v[50]-work.L[120]*work.v[51]-work.L[121]*work.v[52]-work.L[122]*work.v[53]-work.L[123]*work.v[54]-work.L[124]*work.v[55])*work.d_inv[56];
  work.L[146] = (-work.L[128]*work.v[38]-work.L[129]*work.v[39]-work.L[130]*work.v[40]-work.L[131]*work.v[41]-work.L[132]*work.v[42]-work.L[133]*work.v[43]-work.L[134]*work.v[44]-work.L[135]*work.v[45]-work.L[136]*work.v[46]-work.L[137]*work.v[47]-work.L[138]*work.v[48]-work.L[139]*work.v[49]-work.L[140]*work.v[50]-work.L[141]*work.v[51]-work.L[142]*work.v[52]-work.L[143]*work.v[53]-work.L[144]*work.v[54]-work.L[145]*work.v[55])*work.d_inv[56];
  work.v[35] = work.L[86]*work.d[35];
  work.v[38] = work.L[87]*work.d[38];
  work.v[39] = work.L[88]*work.d[39];
  work.v[40] = work.L[89]*work.d[40];
  work.v[41] = work.L[90]*work.d[41];
  work.v[42] = work.L[91]*work.d[42];
  work.v[43] = work.L[92]*work.d[43];
  work.v[44] = work.L[93]*work.d[44];
  work.v[45] = work.L[94]*work.d[45];
  work.v[46] = work.L[95]*work.d[46];
  work.v[47] = work.L[96]*work.d[47];
  work.v[48] = work.L[97]*work.d[48];
  work.v[49] = work.L[98]*work.d[49];
  work.v[50] = work.L[99]*work.d[50];
  work.v[51] = work.L[100]*work.d[51];
  work.v[52] = work.L[101]*work.d[52];
  work.v[53] = work.L[102]*work.d[53];
  work.v[54] = work.L[103]*work.d[54];
  work.v[55] = work.L[104]*work.d[55];
  work.v[56] = work.L[105]*work.d[56];
  work.v[57] = 0-work.L[86]*work.v[35]-work.L[87]*work.v[38]-work.L[88]*work.v[39]-work.L[89]*work.v[40]-work.L[90]*work.v[41]-work.L[91]*work.v[42]-work.L[92]*work.v[43]-work.L[93]*work.v[44]-work.L[94]*work.v[45]-work.L[95]*work.v[46]-work.L[96]*work.v[47]-work.L[97]*work.v[48]-work.L[98]*work.v[49]-work.L[99]*work.v[50]-work.L[100]*work.v[51]-work.L[101]*work.v[52]-work.L[102]*work.v[53]-work.L[103]*work.v[54]-work.L[104]*work.v[55]-work.L[105]*work.v[56];
  work.d[57] = work.v[57];
  if (work.d[57] > 0)
    work.d[57] = -settings.kkt_reg;
  else
    work.d[57] -= settings.kkt_reg;
  work.d_inv[57] = 1/work.d[57];
  work.L[126] = (-work.L[107]*work.v[38]-work.L[108]*work.v[39]-work.L[109]*work.v[40]-work.L[110]*work.v[41]-work.L[111]*work.v[42]-work.L[112]*work.v[43]-work.L[113]*work.v[44]-work.L[114]*work.v[45]-work.L[115]*work.v[46]-work.L[116]*work.v[47]-work.L[117]*work.v[48]-work.L[118]*work.v[49]-work.L[119]*work.v[50]-work.L[120]*work.v[51]-work.L[121]*work.v[52]-work.L[122]*work.v[53]-work.L[123]*work.v[54]-work.L[124]*work.v[55]-work.L[125]*work.v[56])*work.d_inv[57];
  work.L[147] = (-work.L[128]*work.v[38]-work.L[129]*work.v[39]-work.L[130]*work.v[40]-work.L[131]*work.v[41]-work.L[132]*work.v[42]-work.L[133]*work.v[43]-work.L[134]*work.v[44]-work.L[135]*work.v[45]-work.L[136]*work.v[46]-work.L[137]*work.v[47]-work.L[138]*work.v[48]-work.L[139]*work.v[49]-work.L[140]*work.v[50]-work.L[141]*work.v[51]-work.L[142]*work.v[52]-work.L[143]*work.v[53]-work.L[144]*work.v[54]-work.L[145]*work.v[55]-work.L[146]*work.v[56])*work.d_inv[57];
  work.v[36] = work.L[106]*work.d[36];
  work.v[38] = work.L[107]*work.d[38];
  work.v[39] = work.L[108]*work.d[39];
  work.v[40] = work.L[109]*work.d[40];
  work.v[41] = work.L[110]*work.d[41];
  work.v[42] = work.L[111]*work.d[42];
  work.v[43] = work.L[112]*work.d[43];
  work.v[44] = work.L[113]*work.d[44];
  work.v[45] = work.L[114]*work.d[45];
  work.v[46] = work.L[115]*work.d[46];
  work.v[47] = work.L[116]*work.d[47];
  work.v[48] = work.L[117]*work.d[48];
  work.v[49] = work.L[118]*work.d[49];
  work.v[50] = work.L[119]*work.d[50];
  work.v[51] = work.L[120]*work.d[51];
  work.v[52] = work.L[121]*work.d[52];
  work.v[53] = work.L[122]*work.d[53];
  work.v[54] = work.L[123]*work.d[54];
  work.v[55] = work.L[124]*work.d[55];
  work.v[56] = work.L[125]*work.d[56];
  work.v[57] = work.L[126]*work.d[57];
  work.v[58] = 0-work.L[106]*work.v[36]-work.L[107]*work.v[38]-work.L[108]*work.v[39]-work.L[109]*work.v[40]-work.L[110]*work.v[41]-work.L[111]*work.v[42]-work.L[112]*work.v[43]-work.L[113]*work.v[44]-work.L[114]*work.v[45]-work.L[115]*work.v[46]-work.L[116]*work.v[47]-work.L[117]*work.v[48]-work.L[118]*work.v[49]-work.L[119]*work.v[50]-work.L[120]*work.v[51]-work.L[121]*work.v[52]-work.L[122]*work.v[53]-work.L[123]*work.v[54]-work.L[124]*work.v[55]-work.L[125]*work.v[56]-work.L[126]*work.v[57];
  work.d[58] = work.v[58];
  if (work.d[58] > 0)
    work.d[58] = -settings.kkt_reg;
  else
    work.d[58] -= settings.kkt_reg;
  work.d_inv[58] = 1/work.d[58];
  work.L[148] = (-work.L[128]*work.v[38]-work.L[129]*work.v[39]-work.L[130]*work.v[40]-work.L[131]*work.v[41]-work.L[132]*work.v[42]-work.L[133]*work.v[43]-work.L[134]*work.v[44]-work.L[135]*work.v[45]-work.L[136]*work.v[46]-work.L[137]*work.v[47]-work.L[138]*work.v[48]-work.L[139]*work.v[49]-work.L[140]*work.v[50]-work.L[141]*work.v[51]-work.L[142]*work.v[52]-work.L[143]*work.v[53]-work.L[144]*work.v[54]-work.L[145]*work.v[55]-work.L[146]*work.v[56]-work.L[147]*work.v[57])*work.d_inv[58];
  work.v[37] = work.L[127]*work.d[37];
  work.v[38] = work.L[128]*work.d[38];
  work.v[39] = work.L[129]*work.d[39];
  work.v[40] = work.L[130]*work.d[40];
  work.v[41] = work.L[131]*work.d[41];
  work.v[42] = work.L[132]*work.d[42];
  work.v[43] = work.L[133]*work.d[43];
  work.v[44] = work.L[134]*work.d[44];
  work.v[45] = work.L[135]*work.d[45];
  work.v[46] = work.L[136]*work.d[46];
  work.v[47] = work.L[137]*work.d[47];
  work.v[48] = work.L[138]*work.d[48];
  work.v[49] = work.L[139]*work.d[49];
  work.v[50] = work.L[140]*work.d[50];
  work.v[51] = work.L[141]*work.d[51];
  work.v[52] = work.L[142]*work.d[52];
  work.v[53] = work.L[143]*work.d[53];
  work.v[54] = work.L[144]*work.d[54];
  work.v[55] = work.L[145]*work.d[55];
  work.v[56] = work.L[146]*work.d[56];
  work.v[57] = work.L[147]*work.d[57];
  work.v[58] = work.L[148]*work.d[58];
  work.v[59] = 0-work.L[127]*work.v[37]-work.L[128]*work.v[38]-work.L[129]*work.v[39]-work.L[130]*work.v[40]-work.L[131]*work.v[41]-work.L[132]*work.v[42]-work.L[133]*work.v[43]-work.L[134]*work.v[44]-work.L[135]*work.v[45]-work.L[136]*work.v[46]-work.L[137]*work.v[47]-work.L[138]*work.v[48]-work.L[139]*work.v[49]-work.L[140]*work.v[50]-work.L[141]*work.v[51]-work.L[142]*work.v[52]-work.L[143]*work.v[53]-work.L[144]*work.v[54]-work.L[145]*work.v[55]-work.L[146]*work.v[56]-work.L[147]*work.v[57]-work.L[148]*work.v[58];
  work.d[59] = work.v[59];
  if (work.d[59] > 0)
    work.d[59] = -settings.kkt_reg;
  else
    work.d[59] -= settings.kkt_reg;
  work.d_inv[59] = 1/work.d[59];
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
  temp = work.KKT[76]-1*work.d[38]*1-work.L[16]*work.d[16]*work.L[16];
  residual += temp*temp;
  temp = work.KKT[83]-1*work.d[39]*1-work.L[17]*work.d[17]*work.L[17];
  residual += temp*temp;
  temp = work.KKT[90]-1*work.d[40]*1-work.L[18]*work.d[18]*work.L[18];
  residual += temp*temp;
  temp = work.KKT[97]-1*work.d[41]*1-work.L[19]*work.d[19]*work.L[19];
  residual += temp*temp;
  temp = work.KKT[104]-1*work.d[42]*1-work.L[20]*work.d[20]*work.L[20];
  residual += temp*temp;
  temp = work.KKT[111]-1*work.d[43]*1-work.L[21]*work.d[21]*work.L[21];
  residual += temp*temp;
  temp = work.KKT[118]-1*work.d[44]*1-work.L[22]*work.d[22]*work.L[22];
  residual += temp*temp;
  temp = work.KKT[125]-1*work.d[45]*1-work.L[23]*work.d[23]*work.L[23];
  residual += temp*temp;
  temp = work.KKT[132]-1*work.d[46]*1-work.L[24]*work.d[24]*work.L[24];
  residual += temp*temp;
  temp = work.KKT[139]-1*work.d[47]*1-work.L[25]*work.d[25]*work.L[25];
  residual += temp*temp;
  temp = work.KKT[146]-1*work.d[48]*1-work.L[26]*work.d[26]*work.L[26];
  residual += temp*temp;
  temp = work.KKT[153]-1*work.d[49]*1-work.L[27]*work.d[27]*work.L[27];
  residual += temp*temp;
  temp = work.KKT[160]-1*work.d[50]*1-work.L[28]*work.d[28]*work.L[28];
  residual += temp*temp;
  temp = work.KKT[167]-1*work.d[51]*1-work.L[29]*work.d[29]*work.L[29];
  residual += temp*temp;
  temp = work.KKT[174]-1*work.d[52]*1-work.L[30]*work.d[30]*work.L[30];
  residual += temp*temp;
  temp = work.KKT[181]-1*work.d[53]*1-work.L[31]*work.d[31]*work.L[31];
  residual += temp*temp;
  temp = work.KKT[64]-1*work.d[32]*1;
  residual += temp*temp;
  temp = work.KKT[66]-1*work.d[33]*1;
  residual += temp*temp;
  temp = work.KKT[68]-1*work.d[34]*1;
  residual += temp*temp;
  temp = work.KKT[70]-1*work.d[35]*1;
  residual += temp*temp;
  temp = work.KKT[72]-1*work.d[36]*1;
  residual += temp*temp;
  temp = work.KKT[74]-1*work.d[37]*1;
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
  temp = work.KKT[16]-1*work.d[8]*1;
  residual += temp*temp;
  temp = work.KKT[18]-1*work.d[9]*1;
  residual += temp*temp;
  temp = work.KKT[20]-1*work.d[10]*1;
  residual += temp*temp;
  temp = work.KKT[22]-1*work.d[11]*1;
  residual += temp*temp;
  temp = work.KKT[24]-1*work.d[12]*1;
  residual += temp*temp;
  temp = work.KKT[26]-1*work.d[13]*1;
  residual += temp*temp;
  temp = work.KKT[28]-1*work.d[14]*1;
  residual += temp*temp;
  temp = work.KKT[30]-1*work.d[15]*1;
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
  temp = work.KKT[17]-work.L[8]*work.d[8]*1;
  residual += temp*temp;
  temp = work.KKT[19]-work.L[9]*work.d[9]*1;
  residual += temp*temp;
  temp = work.KKT[21]-work.L[10]*work.d[10]*1;
  residual += temp*temp;
  temp = work.KKT[23]-work.L[11]*work.d[11]*1;
  residual += temp*temp;
  temp = work.KKT[25]-work.L[12]*work.d[12]*1;
  residual += temp*temp;
  temp = work.KKT[27]-work.L[13]*work.d[13]*1;
  residual += temp*temp;
  temp = work.KKT[29]-work.L[14]*work.d[14]*1;
  residual += temp*temp;
  temp = work.KKT[31]-work.L[15]*work.d[15]*1;
  residual += temp*temp;
  temp = work.KKT[32]-work.L[0]*work.d[0]*work.L[0]-1*work.d[16]*1;
  residual += temp*temp;
  temp = work.KKT[34]-work.L[1]*work.d[1]*work.L[1]-1*work.d[17]*1;
  residual += temp*temp;
  temp = work.KKT[36]-work.L[2]*work.d[2]*work.L[2]-1*work.d[18]*1;
  residual += temp*temp;
  temp = work.KKT[38]-work.L[3]*work.d[3]*work.L[3]-1*work.d[19]*1;
  residual += temp*temp;
  temp = work.KKT[40]-work.L[4]*work.d[4]*work.L[4]-1*work.d[20]*1;
  residual += temp*temp;
  temp = work.KKT[42]-work.L[5]*work.d[5]*work.L[5]-1*work.d[21]*1;
  residual += temp*temp;
  temp = work.KKT[44]-work.L[6]*work.d[6]*work.L[6]-1*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[46]-work.L[7]*work.d[7]*work.L[7]-1*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[48]-work.L[8]*work.d[8]*work.L[8]-1*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[50]-work.L[9]*work.d[9]*work.L[9]-1*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[52]-work.L[10]*work.d[10]*work.L[10]-1*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[54]-work.L[11]*work.d[11]*work.L[11]-1*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[56]-work.L[12]*work.d[12]*work.L[12]-1*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[58]-work.L[13]*work.d[13]*work.L[13]-1*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[60]-work.L[14]*work.d[14]*work.L[14]-1*work.d[30]*1;
  residual += temp*temp;
  temp = work.KKT[62]-work.L[15]*work.d[15]*work.L[15]-1*work.d[31]*1;
  residual += temp*temp;
  temp = work.KKT[33]-1*work.d[16]*work.L[16];
  residual += temp*temp;
  temp = work.KKT[35]-1*work.d[17]*work.L[17];
  residual += temp*temp;
  temp = work.KKT[37]-1*work.d[18]*work.L[18];
  residual += temp*temp;
  temp = work.KKT[39]-1*work.d[19]*work.L[19];
  residual += temp*temp;
  temp = work.KKT[41]-1*work.d[20]*work.L[20];
  residual += temp*temp;
  temp = work.KKT[43]-1*work.d[21]*work.L[21];
  residual += temp*temp;
  temp = work.KKT[45]-1*work.d[22]*work.L[22];
  residual += temp*temp;
  temp = work.KKT[47]-1*work.d[23]*work.L[23];
  residual += temp*temp;
  temp = work.KKT[49]-1*work.d[24]*work.L[24];
  residual += temp*temp;
  temp = work.KKT[51]-1*work.d[25]*work.L[25];
  residual += temp*temp;
  temp = work.KKT[53]-1*work.d[26]*work.L[26];
  residual += temp*temp;
  temp = work.KKT[55]-1*work.d[27]*work.L[27];
  residual += temp*temp;
  temp = work.KKT[57]-1*work.d[28]*work.L[28];
  residual += temp*temp;
  temp = work.KKT[59]-1*work.d[29]*work.L[29];
  residual += temp*temp;
  temp = work.KKT[61]-1*work.d[30]*work.L[30];
  residual += temp*temp;
  temp = work.KKT[63]-1*work.d[31]*work.L[31];
  residual += temp*temp;
  temp = work.KKT[77]-work.L[33]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[84]-work.L[34]*work.d[39]*1;
  residual += temp*temp;
  temp = work.KKT[91]-work.L[35]*work.d[40]*1;
  residual += temp*temp;
  temp = work.KKT[98]-work.L[36]*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[105]-work.L[37]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[112]-work.L[38]*work.d[43]*1;
  residual += temp*temp;
  temp = work.KKT[119]-work.L[39]*work.d[44]*1;
  residual += temp*temp;
  temp = work.KKT[126]-work.L[40]*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[133]-work.L[41]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[140]-work.L[42]*work.d[47]*1;
  residual += temp*temp;
  temp = work.KKT[147]-work.L[43]*work.d[48]*1;
  residual += temp*temp;
  temp = work.KKT[154]-work.L[44]*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[161]-work.L[45]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[168]-work.L[46]*work.d[51]*1;
  residual += temp*temp;
  temp = work.KKT[175]-work.L[47]*work.d[52]*1;
  residual += temp*temp;
  temp = work.KKT[182]-work.L[48]*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[78]-work.L[50]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[85]-work.L[51]*work.d[39]*1;
  residual += temp*temp;
  temp = work.KKT[92]-work.L[52]*work.d[40]*1;
  residual += temp*temp;
  temp = work.KKT[99]-work.L[53]*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[106]-work.L[54]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[113]-work.L[55]*work.d[43]*1;
  residual += temp*temp;
  temp = work.KKT[120]-work.L[56]*work.d[44]*1;
  residual += temp*temp;
  temp = work.KKT[127]-work.L[57]*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[134]-work.L[58]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[141]-work.L[59]*work.d[47]*1;
  residual += temp*temp;
  temp = work.KKT[148]-work.L[60]*work.d[48]*1;
  residual += temp*temp;
  temp = work.KKT[155]-work.L[61]*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[162]-work.L[62]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[169]-work.L[63]*work.d[51]*1;
  residual += temp*temp;
  temp = work.KKT[176]-work.L[64]*work.d[52]*1;
  residual += temp*temp;
  temp = work.KKT[183]-work.L[65]*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[79]-work.L[68]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[86]-work.L[69]*work.d[39]*1;
  residual += temp*temp;
  temp = work.KKT[93]-work.L[70]*work.d[40]*1;
  residual += temp*temp;
  temp = work.KKT[100]-work.L[71]*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[107]-work.L[72]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[114]-work.L[73]*work.d[43]*1;
  residual += temp*temp;
  temp = work.KKT[121]-work.L[74]*work.d[44]*1;
  residual += temp*temp;
  temp = work.KKT[128]-work.L[75]*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[135]-work.L[76]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[142]-work.L[77]*work.d[47]*1;
  residual += temp*temp;
  temp = work.KKT[149]-work.L[78]*work.d[48]*1;
  residual += temp*temp;
  temp = work.KKT[156]-work.L[79]*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[163]-work.L[80]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[170]-work.L[81]*work.d[51]*1;
  residual += temp*temp;
  temp = work.KKT[177]-work.L[82]*work.d[52]*1;
  residual += temp*temp;
  temp = work.KKT[184]-work.L[83]*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[80]-work.L[87]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[87]-work.L[88]*work.d[39]*1;
  residual += temp*temp;
  temp = work.KKT[94]-work.L[89]*work.d[40]*1;
  residual += temp*temp;
  temp = work.KKT[101]-work.L[90]*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[108]-work.L[91]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[115]-work.L[92]*work.d[43]*1;
  residual += temp*temp;
  temp = work.KKT[122]-work.L[93]*work.d[44]*1;
  residual += temp*temp;
  temp = work.KKT[129]-work.L[94]*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[136]-work.L[95]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[143]-work.L[96]*work.d[47]*1;
  residual += temp*temp;
  temp = work.KKT[150]-work.L[97]*work.d[48]*1;
  residual += temp*temp;
  temp = work.KKT[157]-work.L[98]*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[164]-work.L[99]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[171]-work.L[100]*work.d[51]*1;
  residual += temp*temp;
  temp = work.KKT[178]-work.L[101]*work.d[52]*1;
  residual += temp*temp;
  temp = work.KKT[185]-work.L[102]*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[81]-work.L[107]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[88]-work.L[108]*work.d[39]*1;
  residual += temp*temp;
  temp = work.KKT[95]-work.L[109]*work.d[40]*1;
  residual += temp*temp;
  temp = work.KKT[102]-work.L[110]*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[109]-work.L[111]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[116]-work.L[112]*work.d[43]*1;
  residual += temp*temp;
  temp = work.KKT[123]-work.L[113]*work.d[44]*1;
  residual += temp*temp;
  temp = work.KKT[130]-work.L[114]*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[137]-work.L[115]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[144]-work.L[116]*work.d[47]*1;
  residual += temp*temp;
  temp = work.KKT[151]-work.L[117]*work.d[48]*1;
  residual += temp*temp;
  temp = work.KKT[158]-work.L[118]*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[165]-work.L[119]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[172]-work.L[120]*work.d[51]*1;
  residual += temp*temp;
  temp = work.KKT[179]-work.L[121]*work.d[52]*1;
  residual += temp*temp;
  temp = work.KKT[186]-work.L[122]*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[82]-work.L[128]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[89]-work.L[129]*work.d[39]*1;
  residual += temp*temp;
  temp = work.KKT[96]-work.L[130]*work.d[40]*1;
  residual += temp*temp;
  temp = work.KKT[103]-work.L[131]*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[110]-work.L[132]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[117]-work.L[133]*work.d[43]*1;
  residual += temp*temp;
  temp = work.KKT[124]-work.L[134]*work.d[44]*1;
  residual += temp*temp;
  temp = work.KKT[131]-work.L[135]*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[138]-work.L[136]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[145]-work.L[137]*work.d[47]*1;
  residual += temp*temp;
  temp = work.KKT[152]-work.L[138]*work.d[48]*1;
  residual += temp*temp;
  temp = work.KKT[159]-work.L[139]*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[166]-work.L[140]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[173]-work.L[141]*work.d[51]*1;
  residual += temp*temp;
  temp = work.KKT[180]-work.L[142]*work.d[52]*1;
  residual += temp*temp;
  temp = work.KKT[187]-work.L[143]*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[65]-work.L[32]*work.d[32]*1;
  residual += temp*temp;
  temp = work.KKT[67]-work.L[49]*work.d[33]*1;
  residual += temp*temp;
  temp = work.KKT[69]-work.L[67]*work.d[34]*1;
  residual += temp*temp;
  temp = work.KKT[71]-work.L[86]*work.d[35]*1;
  residual += temp*temp;
  temp = work.KKT[73]-work.L[106]*work.d[36]*1;
  residual += temp*temp;
  temp = work.KKT[75]-work.L[127]*work.d[37]*1;
  residual += temp*temp;
  return residual;
}
void matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = work.KKT[76]*source[0]+work.KKT[33]*source[38]+work.KKT[77]*source[54]+work.KKT[78]*source[55]+work.KKT[79]*source[56]+work.KKT[80]*source[57]+work.KKT[81]*source[58]+work.KKT[82]*source[59];
  result[1] = work.KKT[83]*source[1]+work.KKT[35]*source[39]+work.KKT[84]*source[54]+work.KKT[85]*source[55]+work.KKT[86]*source[56]+work.KKT[87]*source[57]+work.KKT[88]*source[58]+work.KKT[89]*source[59];
  result[2] = work.KKT[90]*source[2]+work.KKT[37]*source[40]+work.KKT[91]*source[54]+work.KKT[92]*source[55]+work.KKT[93]*source[56]+work.KKT[94]*source[57]+work.KKT[95]*source[58]+work.KKT[96]*source[59];
  result[3] = work.KKT[97]*source[3]+work.KKT[39]*source[41]+work.KKT[98]*source[54]+work.KKT[99]*source[55]+work.KKT[100]*source[56]+work.KKT[101]*source[57]+work.KKT[102]*source[58]+work.KKT[103]*source[59];
  result[4] = work.KKT[104]*source[4]+work.KKT[41]*source[42]+work.KKT[105]*source[54]+work.KKT[106]*source[55]+work.KKT[107]*source[56]+work.KKT[108]*source[57]+work.KKT[109]*source[58]+work.KKT[110]*source[59];
  result[5] = work.KKT[111]*source[5]+work.KKT[43]*source[43]+work.KKT[112]*source[54]+work.KKT[113]*source[55]+work.KKT[114]*source[56]+work.KKT[115]*source[57]+work.KKT[116]*source[58]+work.KKT[117]*source[59];
  result[6] = work.KKT[118]*source[6]+work.KKT[45]*source[44]+work.KKT[119]*source[54]+work.KKT[120]*source[55]+work.KKT[121]*source[56]+work.KKT[122]*source[57]+work.KKT[123]*source[58]+work.KKT[124]*source[59];
  result[7] = work.KKT[125]*source[7]+work.KKT[47]*source[45]+work.KKT[126]*source[54]+work.KKT[127]*source[55]+work.KKT[128]*source[56]+work.KKT[129]*source[57]+work.KKT[130]*source[58]+work.KKT[131]*source[59];
  result[8] = work.KKT[132]*source[8]+work.KKT[49]*source[46]+work.KKT[133]*source[54]+work.KKT[134]*source[55]+work.KKT[135]*source[56]+work.KKT[136]*source[57]+work.KKT[137]*source[58]+work.KKT[138]*source[59];
  result[9] = work.KKT[139]*source[9]+work.KKT[51]*source[47]+work.KKT[140]*source[54]+work.KKT[141]*source[55]+work.KKT[142]*source[56]+work.KKT[143]*source[57]+work.KKT[144]*source[58]+work.KKT[145]*source[59];
  result[10] = work.KKT[146]*source[10]+work.KKT[53]*source[48]+work.KKT[147]*source[54]+work.KKT[148]*source[55]+work.KKT[149]*source[56]+work.KKT[150]*source[57]+work.KKT[151]*source[58]+work.KKT[152]*source[59];
  result[11] = work.KKT[153]*source[11]+work.KKT[55]*source[49]+work.KKT[154]*source[54]+work.KKT[155]*source[55]+work.KKT[156]*source[56]+work.KKT[157]*source[57]+work.KKT[158]*source[58]+work.KKT[159]*source[59];
  result[12] = work.KKT[160]*source[12]+work.KKT[57]*source[50]+work.KKT[161]*source[54]+work.KKT[162]*source[55]+work.KKT[163]*source[56]+work.KKT[164]*source[57]+work.KKT[165]*source[58]+work.KKT[166]*source[59];
  result[13] = work.KKT[167]*source[13]+work.KKT[59]*source[51]+work.KKT[168]*source[54]+work.KKT[169]*source[55]+work.KKT[170]*source[56]+work.KKT[171]*source[57]+work.KKT[172]*source[58]+work.KKT[173]*source[59];
  result[14] = work.KKT[174]*source[14]+work.KKT[61]*source[52]+work.KKT[175]*source[54]+work.KKT[176]*source[55]+work.KKT[177]*source[56]+work.KKT[178]*source[57]+work.KKT[179]*source[58]+work.KKT[180]*source[59];
  result[15] = work.KKT[181]*source[15]+work.KKT[63]*source[53]+work.KKT[182]*source[54]+work.KKT[183]*source[55]+work.KKT[184]*source[56]+work.KKT[185]*source[57]+work.KKT[186]*source[58]+work.KKT[187]*source[59];
  result[16] = work.KKT[64]*source[16]+work.KKT[65]*source[54];
  result[17] = work.KKT[66]*source[17]+work.KKT[67]*source[55];
  result[18] = work.KKT[68]*source[18]+work.KKT[69]*source[56];
  result[19] = work.KKT[70]*source[19]+work.KKT[71]*source[57];
  result[20] = work.KKT[72]*source[20]+work.KKT[73]*source[58];
  result[21] = work.KKT[74]*source[21]+work.KKT[75]*source[59];
  result[22] = work.KKT[0]*source[22]+work.KKT[1]*source[38];
  result[23] = work.KKT[2]*source[23]+work.KKT[3]*source[39];
  result[24] = work.KKT[4]*source[24]+work.KKT[5]*source[40];
  result[25] = work.KKT[6]*source[25]+work.KKT[7]*source[41];
  result[26] = work.KKT[8]*source[26]+work.KKT[9]*source[42];
  result[27] = work.KKT[10]*source[27]+work.KKT[11]*source[43];
  result[28] = work.KKT[12]*source[28]+work.KKT[13]*source[44];
  result[29] = work.KKT[14]*source[29]+work.KKT[15]*source[45];
  result[30] = work.KKT[16]*source[30]+work.KKT[17]*source[46];
  result[31] = work.KKT[18]*source[31]+work.KKT[19]*source[47];
  result[32] = work.KKT[20]*source[32]+work.KKT[21]*source[48];
  result[33] = work.KKT[22]*source[33]+work.KKT[23]*source[49];
  result[34] = work.KKT[24]*source[34]+work.KKT[25]*source[50];
  result[35] = work.KKT[26]*source[35]+work.KKT[27]*source[51];
  result[36] = work.KKT[28]*source[36]+work.KKT[29]*source[52];
  result[37] = work.KKT[30]*source[37]+work.KKT[31]*source[53];
  result[38] = work.KKT[1]*source[22]+work.KKT[32]*source[38]+work.KKT[33]*source[0];
  result[39] = work.KKT[3]*source[23]+work.KKT[34]*source[39]+work.KKT[35]*source[1];
  result[40] = work.KKT[5]*source[24]+work.KKT[36]*source[40]+work.KKT[37]*source[2];
  result[41] = work.KKT[7]*source[25]+work.KKT[38]*source[41]+work.KKT[39]*source[3];
  result[42] = work.KKT[9]*source[26]+work.KKT[40]*source[42]+work.KKT[41]*source[4];
  result[43] = work.KKT[11]*source[27]+work.KKT[42]*source[43]+work.KKT[43]*source[5];
  result[44] = work.KKT[13]*source[28]+work.KKT[44]*source[44]+work.KKT[45]*source[6];
  result[45] = work.KKT[15]*source[29]+work.KKT[46]*source[45]+work.KKT[47]*source[7];
  result[46] = work.KKT[17]*source[30]+work.KKT[48]*source[46]+work.KKT[49]*source[8];
  result[47] = work.KKT[19]*source[31]+work.KKT[50]*source[47]+work.KKT[51]*source[9];
  result[48] = work.KKT[21]*source[32]+work.KKT[52]*source[48]+work.KKT[53]*source[10];
  result[49] = work.KKT[23]*source[33]+work.KKT[54]*source[49]+work.KKT[55]*source[11];
  result[50] = work.KKT[25]*source[34]+work.KKT[56]*source[50]+work.KKT[57]*source[12];
  result[51] = work.KKT[27]*source[35]+work.KKT[58]*source[51]+work.KKT[59]*source[13];
  result[52] = work.KKT[29]*source[36]+work.KKT[60]*source[52]+work.KKT[61]*source[14];
  result[53] = work.KKT[31]*source[37]+work.KKT[62]*source[53]+work.KKT[63]*source[15];
  result[54] = work.KKT[77]*source[0]+work.KKT[84]*source[1]+work.KKT[91]*source[2]+work.KKT[98]*source[3]+work.KKT[105]*source[4]+work.KKT[112]*source[5]+work.KKT[119]*source[6]+work.KKT[126]*source[7]+work.KKT[133]*source[8]+work.KKT[140]*source[9]+work.KKT[147]*source[10]+work.KKT[154]*source[11]+work.KKT[161]*source[12]+work.KKT[168]*source[13]+work.KKT[175]*source[14]+work.KKT[182]*source[15]+work.KKT[65]*source[16];
  result[55] = work.KKT[78]*source[0]+work.KKT[85]*source[1]+work.KKT[92]*source[2]+work.KKT[99]*source[3]+work.KKT[106]*source[4]+work.KKT[113]*source[5]+work.KKT[120]*source[6]+work.KKT[127]*source[7]+work.KKT[134]*source[8]+work.KKT[141]*source[9]+work.KKT[148]*source[10]+work.KKT[155]*source[11]+work.KKT[162]*source[12]+work.KKT[169]*source[13]+work.KKT[176]*source[14]+work.KKT[183]*source[15]+work.KKT[67]*source[17];
  result[56] = work.KKT[79]*source[0]+work.KKT[86]*source[1]+work.KKT[93]*source[2]+work.KKT[100]*source[3]+work.KKT[107]*source[4]+work.KKT[114]*source[5]+work.KKT[121]*source[6]+work.KKT[128]*source[7]+work.KKT[135]*source[8]+work.KKT[142]*source[9]+work.KKT[149]*source[10]+work.KKT[156]*source[11]+work.KKT[163]*source[12]+work.KKT[170]*source[13]+work.KKT[177]*source[14]+work.KKT[184]*source[15]+work.KKT[69]*source[18];
  result[57] = work.KKT[80]*source[0]+work.KKT[87]*source[1]+work.KKT[94]*source[2]+work.KKT[101]*source[3]+work.KKT[108]*source[4]+work.KKT[115]*source[5]+work.KKT[122]*source[6]+work.KKT[129]*source[7]+work.KKT[136]*source[8]+work.KKT[143]*source[9]+work.KKT[150]*source[10]+work.KKT[157]*source[11]+work.KKT[164]*source[12]+work.KKT[171]*source[13]+work.KKT[178]*source[14]+work.KKT[185]*source[15]+work.KKT[71]*source[19];
  result[58] = work.KKT[81]*source[0]+work.KKT[88]*source[1]+work.KKT[95]*source[2]+work.KKT[102]*source[3]+work.KKT[109]*source[4]+work.KKT[116]*source[5]+work.KKT[123]*source[6]+work.KKT[130]*source[7]+work.KKT[137]*source[8]+work.KKT[144]*source[9]+work.KKT[151]*source[10]+work.KKT[158]*source[11]+work.KKT[165]*source[12]+work.KKT[172]*source[13]+work.KKT[179]*source[14]+work.KKT[186]*source[15]+work.KKT[73]*source[20];
  result[59] = work.KKT[82]*source[0]+work.KKT[89]*source[1]+work.KKT[96]*source[2]+work.KKT[103]*source[3]+work.KKT[110]*source[4]+work.KKT[117]*source[5]+work.KKT[124]*source[6]+work.KKT[131]*source[7]+work.KKT[138]*source[8]+work.KKT[145]*source[9]+work.KKT[152]*source[10]+work.KKT[159]*source[11]+work.KKT[166]*source[12]+work.KKT[173]*source[13]+work.KKT[180]*source[14]+work.KKT[187]*source[15]+work.KKT[75]*source[21];
}
double check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  matrix_multiply(work.v, multiplicand);
  for (i = 0; i < 22; i++) {
    residual += (target[i] - work.v[i])*(target[i] - work.v[i]);
  }
  return residual;
}
void fill_KKT(void) {
  work.KKT[76] = 2*params.epsilon_f[0];
  work.KKT[83] = 2*params.epsilon_f[0];
  work.KKT[90] = 2*params.epsilon_f[0];
  work.KKT[97] = 2*params.epsilon_f[0];
  work.KKT[104] = 2*params.epsilon_f[0];
  work.KKT[111] = 2*params.epsilon_f[0];
  work.KKT[118] = 2*params.epsilon_f[0];
  work.KKT[125] = 2*params.epsilon_f[0];
  work.KKT[132] = 2*params.epsilon_f[0];
  work.KKT[139] = 2*params.epsilon_f[0];
  work.KKT[146] = 2*params.epsilon_f[0];
  work.KKT[153] = 2*params.epsilon_f[0];
  work.KKT[160] = 2*params.epsilon_f[0];
  work.KKT[167] = 2*params.epsilon_f[0];
  work.KKT[174] = 2*params.epsilon_f[0];
  work.KKT[181] = 2*params.epsilon_f[0];
  work.KKT[64] = 2;
  work.KKT[66] = 2;
  work.KKT[68] = 2;
  work.KKT[70] = 2;
  work.KKT[72] = 2;
  work.KKT[74] = 2;
  work.KKT[0] = work.s_inv_z[0];
  work.KKT[2] = work.s_inv_z[1];
  work.KKT[4] = work.s_inv_z[2];
  work.KKT[6] = work.s_inv_z[3];
  work.KKT[8] = work.s_inv_z[4];
  work.KKT[10] = work.s_inv_z[5];
  work.KKT[12] = work.s_inv_z[6];
  work.KKT[14] = work.s_inv_z[7];
  work.KKT[16] = work.s_inv_z[8];
  work.KKT[18] = work.s_inv_z[9];
  work.KKT[20] = work.s_inv_z[10];
  work.KKT[22] = work.s_inv_z[11];
  work.KKT[24] = work.s_inv_z[12];
  work.KKT[26] = work.s_inv_z[13];
  work.KKT[28] = work.s_inv_z[14];
  work.KKT[30] = work.s_inv_z[15];
  work.KKT[1] = 1;
  work.KKT[3] = 1;
  work.KKT[5] = 1;
  work.KKT[7] = 1;
  work.KKT[9] = 1;
  work.KKT[11] = 1;
  work.KKT[13] = 1;
  work.KKT[15] = 1;
  work.KKT[17] = 1;
  work.KKT[19] = 1;
  work.KKT[21] = 1;
  work.KKT[23] = 1;
  work.KKT[25] = 1;
  work.KKT[27] = 1;
  work.KKT[29] = 1;
  work.KKT[31] = 1;
  work.KKT[32] = work.block_33[0];
  work.KKT[34] = work.block_33[0];
  work.KKT[36] = work.block_33[0];
  work.KKT[38] = work.block_33[0];
  work.KKT[40] = work.block_33[0];
  work.KKT[42] = work.block_33[0];
  work.KKT[44] = work.block_33[0];
  work.KKT[46] = work.block_33[0];
  work.KKT[48] = work.block_33[0];
  work.KKT[50] = work.block_33[0];
  work.KKT[52] = work.block_33[0];
  work.KKT[54] = work.block_33[0];
  work.KKT[56] = work.block_33[0];
  work.KKT[58] = work.block_33[0];
  work.KKT[60] = work.block_33[0];
  work.KKT[62] = work.block_33[0];
  work.KKT[33] = -1;
  work.KKT[35] = -1;
  work.KKT[37] = -1;
  work.KKT[39] = -1;
  work.KKT[41] = -1;
  work.KKT[43] = -1;
  work.KKT[45] = -1;
  work.KKT[47] = -1;
  work.KKT[49] = -1;
  work.KKT[51] = -1;
  work.KKT[53] = -1;
  work.KKT[55] = -1;
  work.KKT[57] = -1;
  work.KKT[59] = -1;
  work.KKT[61] = -1;
  work.KKT[63] = -1;
  work.KKT[77] = params.Phi[0];
  work.KKT[84] = params.Phi[6];
  work.KKT[91] = params.Phi[12];
  work.KKT[98] = params.Phi[18];
  work.KKT[105] = params.Phi[24];
  work.KKT[112] = params.Phi[30];
  work.KKT[119] = params.Phi[36];
  work.KKT[126] = params.Phi[42];
  work.KKT[133] = params.Phi[48];
  work.KKT[140] = params.Phi[54];
  work.KKT[147] = params.Phi[60];
  work.KKT[154] = params.Phi[66];
  work.KKT[161] = params.Phi[72];
  work.KKT[168] = params.Phi[78];
  work.KKT[175] = params.Phi[84];
  work.KKT[182] = params.Phi[90];
  work.KKT[78] = params.Phi[1];
  work.KKT[85] = params.Phi[7];
  work.KKT[92] = params.Phi[13];
  work.KKT[99] = params.Phi[19];
  work.KKT[106] = params.Phi[25];
  work.KKT[113] = params.Phi[31];
  work.KKT[120] = params.Phi[37];
  work.KKT[127] = params.Phi[43];
  work.KKT[134] = params.Phi[49];
  work.KKT[141] = params.Phi[55];
  work.KKT[148] = params.Phi[61];
  work.KKT[155] = params.Phi[67];
  work.KKT[162] = params.Phi[73];
  work.KKT[169] = params.Phi[79];
  work.KKT[176] = params.Phi[85];
  work.KKT[183] = params.Phi[91];
  work.KKT[79] = params.Phi[2];
  work.KKT[86] = params.Phi[8];
  work.KKT[93] = params.Phi[14];
  work.KKT[100] = params.Phi[20];
  work.KKT[107] = params.Phi[26];
  work.KKT[114] = params.Phi[32];
  work.KKT[121] = params.Phi[38];
  work.KKT[128] = params.Phi[44];
  work.KKT[135] = params.Phi[50];
  work.KKT[142] = params.Phi[56];
  work.KKT[149] = params.Phi[62];
  work.KKT[156] = params.Phi[68];
  work.KKT[163] = params.Phi[74];
  work.KKT[170] = params.Phi[80];
  work.KKT[177] = params.Phi[86];
  work.KKT[184] = params.Phi[92];
  work.KKT[80] = params.Phi[3];
  work.KKT[87] = params.Phi[9];
  work.KKT[94] = params.Phi[15];
  work.KKT[101] = params.Phi[21];
  work.KKT[108] = params.Phi[27];
  work.KKT[115] = params.Phi[33];
  work.KKT[122] = params.Phi[39];
  work.KKT[129] = params.Phi[45];
  work.KKT[136] = params.Phi[51];
  work.KKT[143] = params.Phi[57];
  work.KKT[150] = params.Phi[63];
  work.KKT[157] = params.Phi[69];
  work.KKT[164] = params.Phi[75];
  work.KKT[171] = params.Phi[81];
  work.KKT[178] = params.Phi[87];
  work.KKT[185] = params.Phi[93];
  work.KKT[81] = params.Phi[4];
  work.KKT[88] = params.Phi[10];
  work.KKT[95] = params.Phi[16];
  work.KKT[102] = params.Phi[22];
  work.KKT[109] = params.Phi[28];
  work.KKT[116] = params.Phi[34];
  work.KKT[123] = params.Phi[40];
  work.KKT[130] = params.Phi[46];
  work.KKT[137] = params.Phi[52];
  work.KKT[144] = params.Phi[58];
  work.KKT[151] = params.Phi[64];
  work.KKT[158] = params.Phi[70];
  work.KKT[165] = params.Phi[76];
  work.KKT[172] = params.Phi[82];
  work.KKT[179] = params.Phi[88];
  work.KKT[186] = params.Phi[94];
  work.KKT[82] = params.Phi[5];
  work.KKT[89] = params.Phi[11];
  work.KKT[96] = params.Phi[17];
  work.KKT[103] = params.Phi[23];
  work.KKT[110] = params.Phi[29];
  work.KKT[117] = params.Phi[35];
  work.KKT[124] = params.Phi[41];
  work.KKT[131] = params.Phi[47];
  work.KKT[138] = params.Phi[53];
  work.KKT[145] = params.Phi[59];
  work.KKT[152] = params.Phi[65];
  work.KKT[159] = params.Phi[71];
  work.KKT[166] = params.Phi[77];
  work.KKT[173] = params.Phi[83];
  work.KKT[180] = params.Phi[89];
  work.KKT[187] = params.Phi[95];
  work.KKT[65] = -1;
  work.KKT[67] = -1;
  work.KKT[69] = -1;
  work.KKT[71] = -1;
  work.KKT[73] = -1;
  work.KKT[75] = -1;
}
