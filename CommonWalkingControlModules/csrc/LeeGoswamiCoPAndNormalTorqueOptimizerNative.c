#include <stdio.h>
#include "LeeGoswamiCoPAndNormalTorqueOptimizerNative.h"
#include "leeGoswamiCoPAndNormalTorqueOptimizer/solver.h"

#define nFeet us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_MAX_NUMBER_OF_CONTACTS
#define vectorLength us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_vectorLength

#define psiKSize	(vectorLength * nFeet * vectorLength)
#define kappaKSize vectorLength
#define etaMinSize (nFeet * vectorLength)
#define etaMaxSize (nFeet * vectorLength)
#define etaDSize (nFeet * vectorLength)
#define epsilonSize (nFeet * vectorLength)

#define etaSize (nFeet * vectorLength)



Vars vars;
Params params;
Workspace work;
Settings settings;

jobject psiKBuffer;
jobject kappaKBuffer;
jobject etaMinBuffer;
jobject etaMaxBuffer;
jobject etaDBuffer;
jobject epsilonBuffer;

jobject etaBuffer;


JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_initialize
  (JNIEnv* env, jclass jThis)
{
	set_defaults();
	setup_indexing();
	settings.verbose = 0;

	psiKBuffer      = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.Psi_k, sizeof(double) * psiKSize));
	kappaKBuffer    = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.kappa_k, sizeof(double) * kappaKSize));
	etaMinBuffer    = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.etamin, sizeof(double) * etaMinSize));
	etaMaxBuffer    = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.etamax, sizeof(double) * etaMaxSize));
	etaDBuffer      = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.eta_d, sizeof(double) * etaDSize));
	epsilonBuffer   = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, params.epsilon, sizeof(double) * epsilonSize));

	etaBuffer       = (*env)->NewGlobalRef(env, (*env)->NewDirectByteBuffer(env, vars.eta, sizeof(double) * etaSize));
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_getPsiKBuffer
  (JNIEnv* env, jclass jThis)
{
	return psiKBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_getKappaKBuffer
  (JNIEnv* env, jclass jThis)
{
	return kappaKBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_getEtaMinBuffer
  (JNIEnv* env, jclass jThis)
{
	return etaMinBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_getEtaMaxBuffer
  (JNIEnv* env, jclass jThis)
{
	return etaMaxBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_getEtaDBuffer
  (JNIEnv* env, jclass jThis)
{
	return etaDBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_getEpsilonBuffer
  (JNIEnv* env, jclass jThis)
{
	return epsilonBuffer;
}

JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_getEtaBuffer
  (JNIEnv* env, jclass jThis)
{
	return etaBuffer;
}

JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_solveNative
  (JNIEnv* env, jclass jThis)
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

JNIEXPORT jdouble JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_LeeGoswamiCoPAndNormalTorqueOptimizerNative_getOptValNative
  (JNIEnv* env, jclass jThis)
{
	return work.optval;
}


