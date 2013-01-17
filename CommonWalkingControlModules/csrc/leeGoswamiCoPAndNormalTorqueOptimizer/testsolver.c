/* Produced by CVXGEN, 2013-01-17 22:23:43 +0000.  */
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
#define NUMTESTS 0
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
  params.Psi_k_1[0] = 0.20319161029830202;
  params.Psi_k_1[1] = 0.8325912904724193;
  params.Psi_k_1[2] = -0.8363810443482227;
  params.Psi_k_1[3] = 0.04331042079065206;
  params.Psi_k_1[4] = 1.5717878173906188;
  params.Psi_k_1[5] = 1.5851723557337523;
  params.Psi_k_1[6] = -1.497658758144655;
  params.Psi_k_1[7] = -1.171028487447253;
  params.Psi_k_1[8] = -1.7941311867966805;
  params.kappa_k_1[0] = -0.23676062539745413;
  params.kappa_k_1[1] = -1.8804951564857322;
  params.kappa_k_1[2] = -0.17266710242115568;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.epsilon[0] = 1.6491440476147607;
  params.epsilon[3] = 0;
  params.epsilon[6] = 0;
  params.epsilon[1] = 0;
  params.epsilon[4] = 1.2784872826479754;
  params.epsilon[7] = 0;
  params.epsilon[2] = 0;
  params.epsilon[5] = 0;
  params.epsilon[8] = 1.6762549019801312;
  params.eta_d_1[0] = 0.3634512696654033;
  params.eta_d_1[1] = -1.9040724704913385;
  params.eta_d_1[2] = 0.23541635196352795;
  params.Psi_k_2[0] = -0.9629902123701384;
  params.Psi_k_2[1] = -0.3395952119597214;
  params.Psi_k_2[2] = -0.865899672914725;
  params.Psi_k_2[3] = 0.7725516732519853;
  params.Psi_k_2[4] = -0.23818512931704205;
  params.Psi_k_2[5] = -1.372529046100147;
  params.Psi_k_2[6] = 0.17859607212737894;
  params.Psi_k_2[7] = 1.1212590580454682;
  params.Psi_k_2[8] = -0.774545870495281;
  params.kappa_k_2[0] = -1.1121684642712744;
  params.kappa_k_2[1] = -0.44811496977740495;
  params.kappa_k_2[2] = 1.7455345994417217;
  params.eta_d_2[0] = 1.9039816898917352;
  params.eta_d_2[1] = 0.6895347036512547;
  params.eta_d_2[2] = 1.6113364341535923;
  params.etamin_1[0] = 1.383003485172717;
  params.etamin_1[1] = -0.48802383468444344;
  params.etamin_1[2] = -1.631131964513103;
  params.etamin_2[0] = 0.6136436100941447;
  params.etamin_2[1] = 0.2313630495538037;
  params.etamin_2[2] = -0.5537409477496875;
  params.etamax_1[0] = -1.0997819806406723;
  params.etamax_1[1] = -0.3739203344950055;
  params.etamax_1[2] = -0.12423900520332376;
  params.etamax_2[0] = -0.923057686995755;
  params.etamax_2[1] = -0.8328289030982696;
  params.etamax_2[2] = -0.16925440270808823;
}
