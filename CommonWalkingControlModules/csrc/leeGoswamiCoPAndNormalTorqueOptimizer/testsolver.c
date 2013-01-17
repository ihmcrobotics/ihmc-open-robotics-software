/* Produced by CVXGEN, 2013-01-17 22:52:37 +0000.  */
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
  params.Psi_k[0] = 0.20319161029830202;
  params.Psi_k[1] = 0.8325912904724193;
  params.Psi_k[2] = -0.8363810443482227;
  params.Psi_k[3] = 0.04331042079065206;
  params.Psi_k[4] = 1.5717878173906188;
  params.Psi_k[5] = 1.5851723557337523;
  params.Psi_k[6] = -1.497658758144655;
  params.Psi_k[7] = -1.171028487447253;
  params.Psi_k[8] = -1.7941311867966805;
  params.Psi_k[9] = -0.23676062539745413;
  params.Psi_k[10] = -1.8804951564857322;
  params.Psi_k[11] = -0.17266710242115568;
  params.Psi_k[12] = 0.596576190459043;
  params.Psi_k[13] = -0.8860508694080989;
  params.Psi_k[14] = 0.7050196079205251;
  params.Psi_k[15] = 0.3634512696654033;
  params.Psi_k[16] = -1.9040724704913385;
  params.Psi_k[17] = 0.23541635196352795;
  params.kappa_k[0] = -0.9629902123701384;
  params.kappa_k[1] = -0.3395952119597214;
  params.kappa_k[2] = -0.865899672914725;
  params.eta_d[0] = 0.7725516732519853;
  params.eta_d[1] = -0.23818512931704205;
  params.eta_d[2] = -1.372529046100147;
  params.eta_d[3] = 0.17859607212737894;
  params.eta_d[4] = 1.1212590580454682;
  params.eta_d[5] = -0.774545870495281;
  params.epsilon[0] = 1.2219578839321814;
  params.epsilon[1] = 1.3879712575556487;
  params.epsilon[2] = 1.9363836498604305;
  params.epsilon[3] = 1.9759954224729337;
  params.epsilon[4] = 1.6723836759128137;
  params.epsilon[5] = 1.9028341085383982;
  params.etamin[0] = 1.383003485172717;
  params.etamin[1] = -0.48802383468444344;
  params.etamin[2] = -1.631131964513103;
  params.etamin[3] = 0.6136436100941447;
  params.etamin[4] = 0.2313630495538037;
  params.etamin[5] = -0.5537409477496875;
  params.etamax[0] = -1.0997819806406723;
  params.etamax[1] = -0.3739203344950055;
  params.etamax[2] = -0.12423900520332376;
  params.etamax[3] = -0.923057686995755;
  params.etamax[4] = -0.8328289030982696;
  params.etamax[5] = -0.16925440270808823;
}
