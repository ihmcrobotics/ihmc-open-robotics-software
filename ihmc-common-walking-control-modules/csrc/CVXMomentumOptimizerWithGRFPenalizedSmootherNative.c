/*
 * CVXMomentumOptimizerWithGRFPenalizedSmootherNative.c
 *
 *  Created on: November 7, 2013
 *      Author: Tomas
 *      based on work by Twan.
 */

#include <stdio.h>
#include "CVXMomentumOptimizerWithGRFPenalizedSmootherNative.h"
#include "CVXMomentumOptimizerWithGRFPenalizedSmoother/solver.h"

#define nSupportVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_nSupportVectors
#define nPointsPerPlane us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_nPointsPerPlane
#define nPlanes us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_nPlanes
#define wrenchLength us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_wrenchLength
#define nDoF us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_nDoF

#define rhoSize (nSupportVectors * nPointsPerPlane * nPlanes)
#define vdSize nDoF

#define ASize (wrenchLength * nDoF)
#define bSize wrenchLength
#define CSize wrenchLength
#define JsSize (nDoF * nDoF)
#define psSize nDoF
#define WsSize nDoF
#define LambdaSize nDoF
#define QrhoSize (wrenchLength * rhoSize)
#define cSize wrenchLength
#define rhoMinSize rhoSize
#define WRhoSize rhoSize
#define rhoPreviousSize rhoSize
#define WRhoSmootherSize rhoSize
#define rhoPreviousMeanSize rhoSize
#define WRhoCoPPenaltySize rhoSize



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
jobject QrhoByteBuffer;
jobject cByteBuffer;
jobject rhoMinByteBuffer;
jobject rhoByteBuffer;
jobject vdByteBuffer;
jobject WRhoByteBuffer;
jobject rhoPreviousByteBuffer;
jobject WRhoSmootherByteBuffer;
jobject rhoPreviousMeanByteBuffer;
jobject WRhoCoPPenaltyByteBuffer;

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_initialize
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
    QrhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Qrho, sizeof(double) * QrhoSize));
    cByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.c, sizeof(double) * cSize));
    rhoMinByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.rhoMin, sizeof(double) * rhoMinSize));
    WRhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.WRho, sizeof(double) * WRhoSize));

    rhoPreviousByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.rhoPrevious, sizeof(double) * rhoPreviousSize));
    WRhoSmootherByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.WRhoSmoother, sizeof(double) * WRhoSmootherSize));

    rhoPreviousMeanByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.rhoPreviousMean, sizeof(double) * rhoPreviousMeanSize));
    WRhoCoPPenaltyByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.WRhoCoPPenalty, sizeof(double) * WRhoCoPPenaltySize));

    rhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.rho, sizeof(double) * rhoSize));
    vdByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.vd, sizeof(double) * vdSize));
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getABuffer
  (JNIEnv * env, jclass jClass)
{
    return AByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getbBuffer
  (JNIEnv * env, jclass jClass)
{
    return bByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getCBuffer
  (JNIEnv * env, jclass jClass)
{
    return CByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getJsBuffer
  (JNIEnv * env, jclass jClass)
{
    return JsByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getpsBuffer
  (JNIEnv * env, jclass jClass)
{
    return psByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getWsBuffer
  (JNIEnv * env, jclass jClass)
{
    return WsByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getLambdaBuffer
  (JNIEnv * env, jclass jClass)
{
    return LambdaByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getQrhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return QrhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getcBuffer
  (JNIEnv * env, jclass jClass)
{
    return cByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getrhoMinBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoMinByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getrhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getvdBuffer
  (JNIEnv * env, jclass jClass)
{
    return vdByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getWRhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return WRhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getrhoPreviousBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoPreviousByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getWRhoSmootherBuffer
  (JNIEnv * env, jclass jClass)
{
    return WRhoSmootherByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getrhoPreviousMeanBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoPreviousMeanByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getWRhoCoPPenaltyBuffer
  (JNIEnv * env, jclass jClass)
{
    return WRhoCoPPenaltyByteBuffer;
}

JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_solveNative
  (JNIEnv * env, jclass jClass)
{
	int numberOfIterations;

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

JNIEXPORT jdouble JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFPenalizedSmootherNative_getOptValNative
  (JNIEnv * env, jclass jClass)
{
	return work.optval;
}




