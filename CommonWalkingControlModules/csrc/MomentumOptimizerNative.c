/*
 * MomentumOptimizerNative.c
 *
 *  Created on: Apr 23, 2013
 *      Author: twan
 */
#include <stdio.h>
#include "MomentumOptimizerNative.h"
#include "momentumOptimizer/solver.h"

#define nSupportVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_nSupportVectors
#define nPointsPerContact us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_nPointsPerContact
#define nContacts us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_nContacts
#define wrenchLength us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_wrenchLength
#define nDoF us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_nDoF
#define nNull us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_nNull



#define ASize wrenchLength * nDoF
#define bSize wrenchLength
#define CSize wrenchLength
#define JsSize nDoF * nDoF
#define psSize nDoF
#define WsSize nDoF
#define LambdaSize nDoF
#define QSize wrenchLength * nContacts * nSupportVectors * nPointsPerContact
#define cSize wrenchLength
#define rhoMinSize nSupportVectors * nPointsPerContact * nContacts
#define NSize nDoF * nNull
#define zSize nNull

#define rhoSize nPointsPerContact * nSupportVectors * nContacts
#define vdSize nDoF

Vars vars;
Params params;
Workspace work;
Settings settings;


jobject AByteBuffer;
jobject bByteBuffer;
jobject CByteBuffer;
jobject JsByteBuffer;
jobject psByteBuffer;
jobject WsByteBuffer;
jobject LambdaByteBuffer;
jobject QByteBuffer;
jobject cByteBuffer;
jobject NByteBuffer;
jobject zByteBuffer;
jobject rhoMinByteBuffer;
jobject rhoByteBuffer;
jobject vdByteBuffer;

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_initialize
  (JNIEnv * env, jclass jClass)
{
	set_defaults();
	setup_indexing();
	settings.verbose = 0;

	AByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.A, sizeof(double) * ASize));
    bByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.b, sizeof(double) * bSize));
    CByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.C, sizeof(double) * CSize));
    JsByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Js, sizeof(double) * JsSize));
    psByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.ps, sizeof(double) * psSize));
    WsByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Ws, sizeof(double) * WsSize));
    LambdaByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Lambda, sizeof(double) * LambdaSize));
    QByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Q, sizeof(double) * QSize));
    cByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.c, sizeof(double) * cSize));
    NByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.N, sizeof(double) * NSize));
    zByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.z, sizeof(double) * zSize));
    rhoMinByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.rhoMin, sizeof(double) * rhoMinSize));
    rhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.rho, sizeof(double) * rhoSize));
    vdByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.vd, sizeof(double) * vdSize));
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getABuffer
  (JNIEnv * env, jclass jClass)
{
    return AByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getbBuffer
  (JNIEnv * env, jclass jClass)
{
    return bByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getCBuffer
  (JNIEnv * env, jclass jClass)
{
    return CByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getJsBuffer
  (JNIEnv * env, jclass jClass)
{
    return JsByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getpsBuffer
  (JNIEnv * env, jclass jClass)
{
    return psByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getWsBuffer
  (JNIEnv * env, jclass jClass)
{
    return WsByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getLambdaBuffer
  (JNIEnv * env, jclass jClass)
{
    return LambdaByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getQBuffer
  (JNIEnv * env, jclass jClass)
{
    return QByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getcBuffer
  (JNIEnv * env, jclass jClass)
{
    return cByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getNBuffer
  (JNIEnv * env, jclass jClass)
{
    return NByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getzBuffer
  (JNIEnv * env, jclass jClass)
{
    return zByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getrhoMinBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoMinByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getrhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getvdBuffer
  (JNIEnv * env, jclass jClass)
{
    return vdByteBuffer;
}

JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_solveNative
  (JNIEnv * env, jclass jClass, jdouble wRho)
{
	int numberOfIterations;

	params.wRho[0] = wRho;
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

JNIEXPORT jdouble JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_MomentumOptimizerNative_getOptValNative
  (JNIEnv * env, jclass jClass)
{
	return work.optval;
}
