/*
 * CVXMomentumOptimizerWithGRFSmootherNative.c
 *
 *  Created on: November 7, 2013
 *      Author: Sylvain
 *      based on work by Twan.
 */

#include <stdio.h>
#include "CVXMomentumOptimizerWithGRFSmootherNative.h"
#include "CVXMomentumOptimizerWithGRFSmoother/solver.h"

#define nSupportVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_nSupportVectors
#define nPointsPerPlane us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_nPointsPerPlane
#define nPlanes us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_nPlanes
#define nCylinders us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_nCylinders
#define nCylinderVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_nCylinderVectors
#define nCylinderBoundedVariables us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_nCylinderBoundedVariables
#define wrenchLength us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_wrenchLength
#define nDoF us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_nDoF

#define rhoSize (nSupportVectors * nPointsPerPlane * nPlanes+nCylinderVectors*nCylinders)
#define phiSize (nCylinders * nCylinderBoundedVariables)
#define vdSize nDoF

#define ASize (wrenchLength * nDoF)
#define bSize wrenchLength
#define CSize wrenchLength
#define JsSize (nDoF * nDoF)
#define psSize nDoF
#define WsSize nDoF
#define LambdaSize nDoF
#define QrhoSize (wrenchLength * rhoSize)
#define QphiSize (wrenchLength * phiSize)
#define cSize wrenchLength
#define rhoMinSize rhoSize
#define phiMinSize phiSize
#define phiMaxSize phiSize
#define WRhoSize rhoSize
#define WPhiSize phiSize
#define rhoPreviousSize rhoSize
#define WRhoSmootherSize rhoSize



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
jobject QphiByteBuffer;
jobject cByteBuffer;
jobject rhoMinByteBuffer;
jobject phiMinByteBuffer;
jobject phiMaxByteBuffer;
jobject rhoByteBuffer;
jobject phiByteBuffer;
jobject vdByteBuffer;
jobject WRhoByteBuffer;
jobject WPhiByteBuffer;
jobject rhoPreviousByteBuffer;
jobject WRhoSmootherByteBuffer;

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_initialize
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
    QphiByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Qphi, sizeof(double) * QphiSize));
    cByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.c, sizeof(double) * cSize));
    rhoMinByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.rhoMin, sizeof(double) * rhoMinSize));
    phiMinByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.phiMin, sizeof(double) * phiMinSize));
    phiMaxByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.phiMax, sizeof(double) * phiMaxSize));
    WRhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.WRho, sizeof(double) * WRhoSize));
    WPhiByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.WPhi, sizeof(double) * WPhiSize));

    rhoPreviousByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.rhoPrevious, sizeof(double) * rhoPreviousSize));
    WRhoSmootherByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.WRhoSmoother, sizeof(double) * WRhoSmootherSize));

    rhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.rho, sizeof(double) * rhoSize));
    phiByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.phi, sizeof(double) * phiSize));
    vdByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.vd, sizeof(double) * vdSize));
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getABuffer
  (JNIEnv * env, jclass jClass)
{
    return AByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getbBuffer
  (JNIEnv * env, jclass jClass)
{
    return bByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getCBuffer
  (JNIEnv * env, jclass jClass)
{
    return CByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getJsBuffer
  (JNIEnv * env, jclass jClass)
{
    return JsByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getpsBuffer
  (JNIEnv * env, jclass jClass)
{
    return psByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getWsBuffer
  (JNIEnv * env, jclass jClass)
{
    return WsByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getLambdaBuffer
  (JNIEnv * env, jclass jClass)
{
    return LambdaByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getQrhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return QrhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getQphiBuffer
	(JNIEnv * env, jclass jClass)
{
	return QphiByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getcBuffer
  (JNIEnv * env, jclass jClass)
{
    return cByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getrhoMinBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoMinByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getphiMinBuffer
  (JNIEnv * env, jclass jClass)
{
	return phiMinByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getphiMaxBuffer
  (JNIEnv * env, jclass jClass)
{
	return phiMaxByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getrhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getphiBuffer
  (JNIEnv * env, jclass jClass)
{
	return phiByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getvdBuffer
  (JNIEnv * env, jclass jClass)
{
    return vdByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getWRhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return WRhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getWPhiBuffer
  (JNIEnv * env, jclass jClass)
{
    return WPhiByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getrhoPreviousBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoPreviousByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getWRhoSmootherBuffer
  (JNIEnv * env, jclass jClass)
{
    return WRhoSmootherByteBuffer;
}

JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_solveNative
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

JNIEXPORT jdouble JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CVXMomentumOptimizerWithGRFSmootherNative_getOptValNative
  (JNIEnv * env, jclass jClass)
{
	return work.optval;
}




