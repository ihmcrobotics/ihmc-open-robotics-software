/* Produced by CVXGEN, 2013-01-17 15:52:18 +0000.  */
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
#define NUMTESTS 1
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

  printf("rho:\n");
  for (i = 0; i < 8; i++)
  {
	  printf("%.5f\n", vars.rho[i]);
  }

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
  params.xi[0] = -0.8880899735055947;
  params.xi[1] = -0.1815116979122129;
  params.xi[2] = -1.17835862158005;
  params.xi[3] = -1.1944851558277074;
  params.xi[4] = 0.05614023926976763;
  params.xi[5] = -1.6510825248767813;
  params.epsilon_f[0] = 0.967171064703173;
}
