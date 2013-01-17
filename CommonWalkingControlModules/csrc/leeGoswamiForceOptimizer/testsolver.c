/* Produced by CVXGEN, 2013-01-16 23:02:00 +0000.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 10000
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.Phi[0] = 0.20319161029830202;
  params.Phi[1] = 0.8325912904724193;
  params.Phi[2] = -0.8363810443482227;
  params.Phi[3] = 0.04331042079065206;
  params.Phi[4] = 1.5717878173906188;
  params.Phi[5] = 1.5851723557337523;
  params.Phi[6] = -1.497658758144655;
  params.Phi[7] = -1.171028487447253;
  params.Phi[8] = -1.7941311867966805;
  params.Phi[9] = -0.23676062539745413;
  params.Phi[10] = -1.8804951564857322;
  params.Phi[11] = -0.17266710242115568;
  params.Phi[12] = 0.596576190459043;
  params.Phi[13] = -0.8860508694080989;
  params.Phi[14] = 0.7050196079205251;
  params.Phi[15] = 0.3634512696654033;
  params.Phi[16] = -1.9040724704913385;
  params.Phi[17] = 0.23541635196352795;
  params.Phi[18] = -0.9629902123701384;
  params.Phi[19] = -0.3395952119597214;
  params.Phi[20] = -0.865899672914725;
  params.Phi[21] = 0.7725516732519853;
  params.Phi[22] = -0.23818512931704205;
  params.Phi[23] = -1.372529046100147;
  params.Phi[24] = 0.17859607212737894;
  params.Phi[25] = 1.1212590580454682;
  params.Phi[26] = -0.774545870495281;
  params.Phi[27] = -1.1121684642712744;
  params.Phi[28] = -0.44811496977740495;
  params.Phi[29] = 1.7455345994417217;
  params.Phi[30] = 1.9039816898917352;
  params.Phi[31] = 0.6895347036512547;
  params.Phi[32] = 1.6113364341535923;
  params.Phi[33] = 1.383003485172717;
  params.Phi[34] = -0.48802383468444344;
  params.Phi[35] = -1.631131964513103;
  params.Phi[36] = 0.6136436100941447;
  params.Phi[37] = 0.2313630495538037;
  params.Phi[38] = -0.5537409477496875;
  params.Phi[39] = -1.0997819806406723;
  params.Phi[40] = -0.3739203344950055;
  params.Phi[41] = -0.12423900520332376;
  params.Phi[42] = -0.923057686995755;
  params.Phi[43] = -0.8328289030982696;
  params.Phi[44] = -0.16925440270808823;
  params.Phi[45] = 1.442135651787706;
  params.Phi[46] = 0.34501161787128565;
  params.Phi[47] = -0.8660485502711608;
  params.Phi[48] = -0.8880899735055947;
  params.Phi[49] = -0.1815116979122129;
  params.Phi[50] = -1.17835862158005;
  params.Phi[51] = -1.1944851558277074;
  params.Phi[52] = 0.05614023926976763;
  params.Phi[53] = -1.6510825248767813;
  params.Phi[54] = -0.06565787059365391;
  params.Phi[55] = -0.5512951504486665;
  params.Phi[56] = 0.8307464872626844;
  params.Phi[57] = 0.9869848924080182;
  params.Phi[58] = 0.7643716874230573;
  params.Phi[59] = 0.7567216550196565;
  params.Phi[60] = -0.5055995034042868;
  params.Phi[61] = 0.6725392189410702;
  params.Phi[62] = -0.6406053441727284;
  params.Phi[63] = 0.29117547947550015;
  params.Phi[64] = -0.6967713677405021;
  params.Phi[65] = -0.21941980294587182;
  params.Phi[66] = -1.753884276680243;
  params.Phi[67] = -1.0292983112626475;
  params.Phi[68] = 1.8864104246942706;
  params.Phi[69] = -1.077663182579704;
  params.Phi[70] = 0.7659100437893209;
  params.Phi[71] = 0.6019074328549583;
  params.Phi[72] = 0.8957565577499285;
  params.Phi[73] = -0.09964555746227477;
  params.Phi[74] = 0.38665509840745127;
  params.Phi[75] = -1.7321223042686946;
  params.Phi[76] = -1.7097514487110663;
  params.Phi[77] = -1.2040958948116867;
  params.Phi[78] = -1.3925560119658358;
  params.Phi[79] = -1.5995826216742213;
  params.Phi[80] = -1.4828245415645833;
  params.Phi[81] = 0.21311092723061398;
  params.Phi[82] = -1.248740700304487;
  params.Phi[83] = 1.808404972124833;
  params.Phi[84] = 0.7264471152297065;
  params.Phi[85] = 0.16407869343908477;
  params.Phi[86] = 0.8287224032315907;
  params.Phi[87] = -0.9444533161899464;
  params.Phi[88] = 1.7069027370149112;
  params.Phi[89] = 1.3567722311998827;
  params.Phi[90] = 0.9052779937121489;
  params.Phi[91] = -0.07904017565835986;
  params.Phi[92] = 1.3684127435065871;
  params.Phi[93] = 0.979009293697437;
  params.Phi[94] = 0.6413036255984501;
  params.Phi[95] = 1.6559010680237511;
  params.xi[0] = 0.5346622551502991;
  params.xi[1] = -0.5362376605895625;
  params.xi[2] = 0.2113782926017822;
  params.xi[3] = -1.2144776931994525;
  params.xi[4] = -1.2317108144255875;
  params.xi[5] = 0.9026784957312834;
  params.epsilon_f[0] = 1.5698734068622622;
}
