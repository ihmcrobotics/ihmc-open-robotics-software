/*
 * ContactPointWrenchOptimizerNative.c
 *
 *  Created on: Jan 22, 2013
 *      Author: twan
 */
#include <stdio.h>
#include "ContactPointWrenchOptimizerNative.h"
#include "contactPointWrenchOptimizer/solver.h"

#define nPointsPerContact us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_NUMBER_OF_POINTS_PER_CONTACT
#define nSupportVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_NUMBER_OF_SUPPORT_VECTORS
#define nContacts us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_MAX_NUMBER_OF_CONTACTS
#define wrenchLength us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_WRENCH_LENGTH

#define aSize wrenchLength * nSupportVectors * nPointsPerContact * nContacts
#define wSize wrenchLength
#define cSize wrenchLength
#define bSize nContacts * nSupportVectors * nPointsPerContact * nContacts
#define fMinSize nContacts
#define rhoSize nSupportVectors * nPointsPerContact * nContacts


Vars vars;
Params params;
Workspace work;
Settings settings;

jobject aByteBuffer;
jobject wByteBuffer;
jobject cByteBuffer;
jobject rhoByteBuffer;
jobject rhoMinByteBuffer;

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_initialize
  (JNIEnv * env, jclass jClass)
{
	set_defaults();
	setup_indexing();
	settings.verbose = 0;

	aByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.A, sizeof(double) * aSize));
	wByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.W, sizeof(double) * wSize));
	cByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.C, sizeof(double) * cSize));
	rhoMinByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.rhomin, sizeof(double) * rhoSize));
	rhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.rho, sizeof(double) * rhoSize));
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getABuffer
  (JNIEnv * env, jclass jClass)
{
	return aByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getWBuffer
  (JNIEnv * env, jclass jClass)
{
	return wByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getCBuffer
  (JNIEnv * env, jclass jClass)
{
	return cByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getRhoMinBuffer
  (JNIEnv * env, jclass jClass)
{
	return rhoMinByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getRhoBuffer
  (JNIEnv * env, jclass jClass)
{
	return rhoByteBuffer;
}

JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_solveNative
  (JNIEnv * env, jclass jClass, jdouble epsilon)
{
	int numberOfIterations;

	params.epsilon[0] = epsilon;
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

JNIEXPORT jdouble JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getOptValNative
  (JNIEnv * env, jclass jClass)
{
	return work.optval;
}
