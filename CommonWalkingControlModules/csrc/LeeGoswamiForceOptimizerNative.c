/*
 * LeeGoswamiForceOptimizer.c
 *
 *  Created on: Jan 17, 2013
 *      Author: jesper
 */

#include <stdio.h>
#include "LeeGoswamiForceOptimizerNative.h"
#include "leeGoswamiForceOptimizer/solver.h"

#define phiSize us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_n * us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_m
#define xiSize us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_n
#define rhoSize us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_m

Vars vars;
Params params;
Workspace work;
Settings settings;

jobject phiByteBuffer;
jobject xiByteBuffer;
jobject rhoByteBuffer;


JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_initialize
  (JNIEnv* env, jclass jClass)
{
	set_defaults();
	setup_indexing();
	settings.verbose = 0;

	phiByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Phi, sizeof(double) * phiSize));
	xiByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.xi, sizeof(double) * xiSize));
	rhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.rho, sizeof(double) * rhoSize));
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_getPhiBuffer
  (JNIEnv* env, jclass jClass)
{
	return phiByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_getXiBuffer
  (JNIEnv* env, jclass jClass)
{
	return xiByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_getRhoBuffer
  (JNIEnv* env, jclass jClass)
{
	return rhoByteBuffer;
}

JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_solveNative
  (JNIEnv* env, jclass jClass, jdouble epsilonF)
{
	int numberOfIterations;


	params.epsilon_f[0] = epsilonF;
	numberOfIterations = solve();

	if(work.converged == 1)
	{
		return numberOfIterations;
	}
	else
	{
		return -1;
	}
}

JNIEXPORT jdouble JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiForceOptimizerNative_getOptValNative
  (
		JNIEnv* env, jclass jClass) {
	return work.optval;
}
