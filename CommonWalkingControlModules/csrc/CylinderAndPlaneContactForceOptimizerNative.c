/*
 * CylinderAndPlaneContactForceOptimizerNative.c
 *
 *  Created on: May 8, 2013
 *      Author: graythomas
 *      based on work by twan.
 */

#include <stdio.h>
#include "CylinderAndPlaneContactForceOptimizerNative.h"
#include "cylinderAndPlaneContactForceOptimizer/solver.h"

#define nSupportVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_nSupportVectors
#define nPointsPerPlane us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_nPointsPerPlane
#define nPlanes us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_nPlanes
#define nCylinders us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_nCylinders
#define nCylinderVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_nCylinderVectors
#define nCylinderBoundedVariables us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_nCylinderBoundedVariables
#define wrenchLength us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_wrenchLength

#define rhoSize (nSupportVectors * nPointsPerPlane * nPlanes+nCylinderVectors*nCylinders)
#define phiSize (nCylinders * nCylinderBoundedVariables)


#define CSize wrenchLength
#define QrhoSize (wrenchLength * rhoSize)
#define QphiSize (wrenchLength * phiSize)
#define cSize wrenchLength
#define rhoMinSize rhoSize
#define phiMinSize phiSize
#define phiMaxSize phiSize

Vars vars;
Params params;
Workspace work;
Settings settings;


jobject CByteBuffer;
jobject QrhoByteBuffer;
jobject QphiByteBuffer;
jobject cByteBuffer;
jobject rhoMinByteBuffer;
jobject phiMinByteBuffer;
jobject phiMaxByteBuffer;
jobject rhoByteBuffer;
jobject phiByteBuffer;

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_initialize
  (JNIEnv * env, jclass jClass)
{
	set_defaults();
	setup_indexing();
	settings.verbose = 0;

    CByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.C, sizeof(double) * CSize));
    QrhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Qrho, sizeof(double) * QrhoSize));
    QphiByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Qphi, sizeof(double) * QphiSize));
    cByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.c, sizeof(double) * cSize));
    rhoMinByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.rhoMin, sizeof(double) * rhoMinSize));
    phiMinByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.phiMin, sizeof(double) * phiMinSize));
    phiMaxByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.phiMax, sizeof(double) * phiMaxSize));
    rhoByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.rho, sizeof(double) * rhoSize));
    phiByteBuffer = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.phi, sizeof(double) * phiSize));
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getCBuffer
  (JNIEnv * env, jclass jClass)
{
    return CByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getQrhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return QrhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getQphiBuffer
	(JNIEnv * env, jclass jClass)
{
	return QphiByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getcBuffer
  (JNIEnv * env, jclass jClass)
{
    return cByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getrhoMinBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoMinByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getphiMinBuffer
  (JNIEnv * env, jclass jClass)
{
	return phiMinByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getphiMaxBuffer
  (JNIEnv * env, jclass jClass)
{
	return phiMaxByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getrhoBuffer
  (JNIEnv * env, jclass jClass)
{
    return rhoByteBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getphiBuffer
  (JNIEnv * env, jclass jClass)
{
	return phiByteBuffer;
}

JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_solveNative
  (JNIEnv * env, jclass jClass, jdouble wRho, jdouble wPhi)
{
	int numberOfIterations;

	params.wRho[0] = wRho;
  params.wPhi[0] = wPhi;
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

JNIEXPORT jdouble JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_CylinderAndPlaneContactForceOptimizerNative_getOptValNative
  (JNIEnv * env, jclass jClass)
{
	return work.optval;
}




