/* Produced by CVXGEN, 2013-11-07 20:49:04 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"phi", "rho", "vd"};
  const int num_var_names = 3;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A");
  if (xm == NULL) {
    printf("could not find params.A.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 34))) {
      printf("A must be size (6,34), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A;
      src = mxGetPr(xm);
      for (i = 0; i < 204; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "C");
  if (xm == NULL) {
    printf("could not find params.C.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 6))) {
      printf("C must be size (6,6), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter C must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter C must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter C must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.C;
      src = mxGetPr(xm);
      warned_diags = 0;
      for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
          if (i == j) {
            *dest++ = *src;
          } else if (!warned_diags && (*src != 0)) {
            printf("\n!!! Warning: ignoring off-diagonal elements in C !!!\n\n");
            warned_diags = 1;
          }
          src++;
        }
      }
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Js");
  if (xm == NULL) {
    printf("could not find params.Js.\n");
  } else {
    if (!((mxGetM(xm) == 34) && (mxGetN(xm) == 34))) {
      printf("Js must be size (34,34), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Js must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Js must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Js must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Js;
      src = mxGetPr(xm);
      for (i = 0; i < 1156; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Lambda");
  if (xm == NULL) {
    printf("could not find params.Lambda.\n");
  } else {
    if (!((mxGetM(xm) == 34) && (mxGetN(xm) == 34))) {
      printf("Lambda must be size (34,34), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Lambda must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Lambda must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Lambda must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Lambda;
      src = mxGetPr(xm);
      warned_diags = 0;
      for (i = 0; i < 34; i++) {
        for (j = 0; j < 34; j++) {
          if (i == j) {
            *dest++ = *src;
          } else if (!warned_diags && (*src != 0)) {
            printf("\n!!! Warning: ignoring off-diagonal elements in Lambda !!!\n\n");
            warned_diags = 1;
          }
          src++;
        }
      }
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Qphi");
  if (xm == NULL) {
    printf("could not find params.Qphi.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 10))) {
      printf("Qphi must be size (6,10), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Qphi must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Qphi must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Qphi must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Qphi;
      src = mxGetPr(xm);
      for (i = 0; i < 60; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Qrho");
  if (xm == NULL) {
    printf("could not find params.Qrho.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 64))) {
      printf("Qrho must be size (6,64), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Qrho must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Qrho must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Qrho must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Qrho;
      src = mxGetPr(xm);
      for (i = 0; i < 384; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "WPhi");
  if (xm == NULL) {
    printf("could not find params.WPhi.\n");
  } else {
    if (!((mxGetM(xm) == 10) && (mxGetN(xm) == 10))) {
      printf("WPhi must be size (10,10), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter WPhi must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter WPhi must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter WPhi must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.WPhi;
      src = mxGetPr(xm);
      warned_diags = 0;
      for (i = 0; i < 10; i++) {
        for (j = 0; j < 10; j++) {
          if (i == j) {
            *dest++ = *src;
          } else if (!warned_diags && (*src != 0)) {
            printf("\n!!! Warning: ignoring off-diagonal elements in WPhi !!!\n\n");
            warned_diags = 1;
          }
          src++;
        }
      }
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "WRho");
  if (xm == NULL) {
    printf("could not find params.WRho.\n");
  } else {
    if (!((mxGetM(xm) == 64) && (mxGetN(xm) == 64))) {
      printf("WRho must be size (64,64), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter WRho must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter WRho must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter WRho must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.WRho;
      src = mxGetPr(xm);
      warned_diags = 0;
      for (i = 0; i < 64; i++) {
        for (j = 0; j < 64; j++) {
          if (i == j) {
            *dest++ = *src;
          } else if (!warned_diags && (*src != 0)) {
            printf("\n!!! Warning: ignoring off-diagonal elements in WRho !!!\n\n");
            warned_diags = 1;
          }
          src++;
        }
      }
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "WRhoSmoother");
  if (xm == NULL) {
    printf("could not find params.WRhoSmoother.\n");
  } else {
    if (!((mxGetM(xm) == 64) && (mxGetN(xm) == 64))) {
      printf("WRhoSmoother must be size (64,64), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter WRhoSmoother must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter WRhoSmoother must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter WRhoSmoother must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.WRhoSmoother;
      src = mxGetPr(xm);
      warned_diags = 0;
      for (i = 0; i < 64; i++) {
        for (j = 0; j < 64; j++) {
          if (i == j) {
            *dest++ = *src;
          } else if (!warned_diags && (*src != 0)) {
            printf("\n!!! Warning: ignoring off-diagonal elements in WRhoSmoother !!!\n\n");
            warned_diags = 1;
          }
          src++;
        }
      }
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Ws");
  if (xm == NULL) {
    printf("could not find params.Ws.\n");
  } else {
    if (!((mxGetM(xm) == 34) && (mxGetN(xm) == 34))) {
      printf("Ws must be size (34,34), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Ws must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Ws must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Ws must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Ws;
      src = mxGetPr(xm);
      warned_diags = 0;
      for (i = 0; i < 34; i++) {
        for (j = 0; j < 34; j++) {
          if (i == j) {
            *dest++ = *src;
          } else if (!warned_diags && (*src != 0)) {
            printf("\n!!! Warning: ignoring off-diagonal elements in Ws !!!\n\n");
            warned_diags = 1;
          }
          src++;
        }
      }
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "b");
  if (xm == NULL) {
    printf("could not find params.b.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("b must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter b must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter b must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter b must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.b;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "c");
  if (xm == NULL) {
    printf("could not find params.c.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("c must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter c must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter c must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter c must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.c;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "phiMax");
  if (xm == NULL) {
    printf("could not find params.phiMax.\n");
  } else {
    if (!((mxGetM(xm) == 10) && (mxGetN(xm) == 1))) {
      printf("phiMax must be size (10,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter phiMax must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter phiMax must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter phiMax must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.phiMax;
      src = mxGetPr(xm);
      for (i = 0; i < 10; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "phiMin");
  if (xm == NULL) {
    printf("could not find params.phiMin.\n");
  } else {
    if (!((mxGetM(xm) == 10) && (mxGetN(xm) == 1))) {
      printf("phiMin must be size (10,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter phiMin must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter phiMin must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter phiMin must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.phiMin;
      src = mxGetPr(xm);
      for (i = 0; i < 10; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ps");
  if (xm == NULL) {
    printf("could not find params.ps.\n");
  } else {
    if (!((mxGetM(xm) == 34) && (mxGetN(xm) == 1))) {
      printf("ps must be size (34,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ps must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ps must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ps must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ps;
      src = mxGetPr(xm);
      for (i = 0; i < 34; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "rhoMin");
  if (xm == NULL) {
    printf("could not find params.rhoMin.\n");
  } else {
    if (!((mxGetM(xm) == 64) && (mxGetN(xm) == 1))) {
      printf("rhoMin must be size (64,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter rhoMin must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter rhoMin must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter rhoMin must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.rhoMin;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "rhoPrevious");
  if (xm == NULL) {
    printf("could not find params.rhoPrevious.\n");
  } else {
    if (!((mxGetM(xm) == 64) && (mxGetN(xm) == 1))) {
      printf("rhoPrevious must be size (64,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter rhoPrevious must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter rhoPrevious must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter rhoPrevious must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.rhoPrevious;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 17) {
    printf("Error: %d parameters are invalid.\n", 17 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 204; i++)
      printf("  params.A[%d] = %.6g;\n", i, params.A[i]);
    for (i = 0; i < 6; i++)
      printf("  params.b[%d] = %.6g;\n", i, params.b[i]);
    for (i = 0; i < 6; i++)
      printf("  params.C[%d] = %.6g;\n", i, params.C[i]);
    for (i = 0; i < 1156; i++)
      printf("  params.Js[%d] = %.6g;\n", i, params.Js[i]);
    for (i = 0; i < 34; i++)
      printf("  params.ps[%d] = %.6g;\n", i, params.ps[i]);
    for (i = 0; i < 34; i++)
      printf("  params.Ws[%d] = %.6g;\n", i, params.Ws[i]);
    for (i = 0; i < 64; i++)
      printf("  params.WRho[%d] = %.6g;\n", i, params.WRho[i]);
    for (i = 0; i < 10; i++)
      printf("  params.WPhi[%d] = %.6g;\n", i, params.WPhi[i]);
    for (i = 0; i < 34; i++)
      printf("  params.Lambda[%d] = %.6g;\n", i, params.Lambda[i]);
    for (i = 0; i < 64; i++)
      printf("  params.rhoPrevious[%d] = %.6g;\n", i, params.rhoPrevious[i]);
    for (i = 0; i < 64; i++)
      printf("  params.WRhoSmoother[%d] = %.6g;\n", i, params.WRhoSmoother[i]);
    for (i = 0; i < 384; i++)
      printf("  params.Qrho[%d] = %.6g;\n", i, params.Qrho[i]);
    for (i = 0; i < 60; i++)
      printf("  params.Qphi[%d] = %.6g;\n", i, params.Qphi[i]);
    for (i = 0; i < 6; i++)
      printf("  params.c[%d] = %.6g;\n", i, params.c[i]);
    for (i = 0; i < 64; i++)
      printf("  params.rhoMin[%d] = %.6g;\n", i, params.rhoMin[i]);
    for (i = 0; i < 10; i++)
      printf("  params.phiMin[%d] = %.6g;\n", i, params.phiMin[i]);
    for (i = 0; i < 10; i++)
      printf("  params.phiMax[%d] = %.6g;\n", i, params.phiMax[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  xm = mxCreateDoubleMatrix(10, 1, mxREAL);
  mxSetField(plhs[0], 0, "phi", xm);
  dest = mxGetPr(xm);
  src = vars.phi;
  for (i = 0; i < 10; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(64, 1, mxREAL);
  mxSetField(plhs[0], 0, "rho", xm);
  dest = mxGetPr(xm);
  src = vars.rho;
  for (i = 0; i < 64; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(34, 1, mxREAL);
  mxSetField(plhs[0], 0, "vd", xm);
  dest = mxGetPr(xm);
  src = vars.vd;
  for (i = 0; i < 34; i++) {
    *dest++ = *src++;
  }
}
