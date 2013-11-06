#include <jni.h>
#include "HQPNative.h"
#include <iostream>
#include <sstream>

#include "soth/HCOD.hpp"
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace soth;
using std::endl;
using std::cout;
using std::cerr;
using std::vector;

#define nSupportVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_nSupportVectors
#define nPointsPerContact us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_nPointsPerContact
#define nPlanes us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_nPlanes
#define wrenchLength us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_wrenchLength
#define nDoF us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_nDoF
#define nCylinders us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_nCylinders
#define nCylinderVectors us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_nCylinderVectors
#define nCylinderBoundedVariables us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_nCylinderBoundedVariables
#define vdSize us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_vdSize
#define rhoSize us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_rhoSize
#define phiSize us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_phiSize

jobject AByteBuffer, bByteBuffer, CByteBuffer, JsByteBuffer, psByteBuffer,
		WsByteBuffer, WRhoByteBuffer, WPhiByteBuffer, LambdaByteBuffer,
		QrhoByteBuffer, QphiByteBuffer, cByteBuffer, rhoMinByteBuffer,
		phiMinByteBuffer, phiMaxByteBuffer;

jobject xByteBuffer;

std::vector<cstref_vector_t> res;

typedef struct Params_t {
	double A[wrenchLength * nDoF];
	double b[wrenchLength];
	double C[wrenchLength * wrenchLength];
	double Js[nDoF * nDoF];
	double ps[nDoF];
	double Ws[nDoF * nDoF];
	double WRho[rhoSize * rhoSize];
	double WPhi[phiSize * phiSize];
	double Lambda[nDoF * nDoF];
	double Qrho[wrenchLength * rhoSize];
	double Qphi[wrenchLength * phiSize];
	double c[wrenchLength];
	double rhoMin[rhoSize];
	double phiMin[phiSize];
	double phiMax[phiSize];
	double x[vdSize+rhoSize+phiSize];
} Params;

Params params;

unsigned int NB_HIERARCHICAL_STAGES, NB_VARIABLES;
vector<unsigned int> NB_CONSTRAINTS;
vector<Eigen::MatrixXd> Constraints;
vector<soth::VectorBound> Parameters;
Eigen::MatrixXd dataA, dataJs, dataContactForce, dataWeights;

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_initialize(JNIEnv *env, jclass jClass)
{
	// TODO: dont define here.
	NB_HIERARCHICAL_STAGES = 4;
	NB_CONSTRAINTS += wrenchLength+rhoSize+phiSize,nDoF, wrenchLength, nDoF+rhoSize+phiSize;

	NB_VARIABLES = vdSize+rhoSize+phiSize;

	Constraints.resize(NB_HIERARCHICAL_STAGES);
	Parameters.resize(NB_HIERARCHICAL_STAGES);

	for(int i = 0; i< NB_HIERARCHICAL_STAGES; i++)
	{
		Constraints[i].resize(NB_CONSTRAINTS[i], NB_VARIABLES);
		Constraints[i].setZero();
		Parameters[i].resize(NB_CONSTRAINTS[i]);
	}

	dataA.resize(wrenchLength, nDoF + rhoSize+phiSize);
	dataA.setZero();
	dataJs.resize(nDoF, nDoF + rhoSize+phiSize);
	dataJs.setZero();
	dataContactForce.resize(wrenchLength + rhoSize + phiSize, nDoF + rhoSize + phiSize);
	dataContactForce.setZero();
	dataWeights.resize(nDoF + rhoSize + phiSize,nDoF + rhoSize + phiSize);
	dataWeights.setZero();

	AByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.A, sizeof(double) * wrenchLength * nDoF));
	bByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.b, sizeof(double) * wrenchLength));
	CByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.C, sizeof(double) * wrenchLength * wrenchLength));
	JsByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.Js, sizeof(double) * nDoF * nDoF));

	psByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.ps, sizeof(double) * nDoF));
	WsByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.Ws, sizeof(double) * nDoF * nDoF));
	WRhoByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.WRho, sizeof(double) * rhoSize * rhoSize));
	WPhiByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.WPhi, sizeof(double) * phiSize * phiSize));

	LambdaByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.Lambda, sizeof(double) * nDoF * nDoF));
	QrhoByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.Qrho, sizeof(double) * wrenchLength * rhoSize));
	QphiByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.Qphi, sizeof(double) * wrenchLength * phiSize));
	cByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.c, sizeof(double) * wrenchLength));

	rhoMinByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.rhoMin, sizeof(double) * rhoSize));
	phiMinByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.phiMin, sizeof(double) * phiSize));
	phiMaxByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.phiMax, sizeof(double) * phiSize));

	xByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(params.x, sizeof(double) * (vdSize+rhoSize+phiSize)));
}

JNIEXPORT void JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_solveInCPP(JNIEnv *env, jclass jClass)
{

	double *dBufA 		= (double*)env->GetDirectBufferAddress(AByteBuffer);
	double *dBufb 		= (double*)env->GetDirectBufferAddress(bByteBuffer);
	double *dBufJs 		= (double*)env->GetDirectBufferAddress(JsByteBuffer);
	double *dBufps 		= (double*)env->GetDirectBufferAddress(psByteBuffer);
	double *dBufx 		= (double*)env->GetDirectBufferAddress(xByteBuffer);
	double *dBufQrho 	= (double*)env->GetDirectBufferAddress(QrhoByteBuffer);
	double *dBufQphi 	= (double*)env->GetDirectBufferAddress(QphiByteBuffer);
	double *dBufc 		= (double*)env->GetDirectBufferAddress(cByteBuffer);
	double *dBufrhoMin 	= (double*)env->GetDirectBufferAddress(rhoMinByteBuffer);
	double *dBufphiMin 	= (double*)env->GetDirectBufferAddress(phiMinByteBuffer);
	double *dBufphiMax 	= (double*)env->GetDirectBufferAddress(phiMaxByteBuffer);
	double *dBufLambda 	= (double*)env->GetDirectBufferAddress(LambdaByteBuffer);
	double *dBufWRho 	= (double*)env->GetDirectBufferAddress(WRhoByteBuffer);
	double *dBufWPhi 	= (double*)env->GetDirectBufferAddress(WPhiByteBuffer);

	double *dBufC 		= (double*)env->GetDirectBufferAddress(CByteBuffer);
	double *dBufWs 		= (double*)env->GetDirectBufferAddress(WsByteBuffer);

	int k = 0;
	int l = 0;
	int m = 0;
	int n = 0;
//						| -A	Qrho	Qphi|	|vd	|		|	c			|
//	dataContactForce = 	|  0	I64		0	| * |rho|	= 	|>rhomin		|
//						|  0	0		I10	|	|phi|		|phimin><phiMax	|
	for(int i = 0; i < wrenchLength; i++)
	{
		for(int j = 0; j < nDoF; j++)
		{
			dataContactForce(i,j) = -dBufA[k];
			dataA(i,j) = dBufA[k++];// * dBufC[n];
		}
		for(int j = nDoF; j < nDoF + rhoSize; j++)
		{
			dataContactForce(i,j) = dBufQrho[l++];
		}
		for(int j = nDoF + rhoSize; j < nDoF + rhoSize + phiSize; j++)
		{
			dataContactForce(i,j) = dBufQphi[m++];
		}
		Parameters[0][i] = soth::Bound(dBufc[i]);
		Parameters[2][i] = soth::Bound(dBufb[i]);// * dBufC[n++]);
	}

	for(int i = wrenchLength; i < (wrenchLength + rhoSize + phiSize); i++)
	{
		dataContactForce(i,i+nDoF-wrenchLength) = 1;
	}

	k = 0;
	l = 0;
	n = 0;
	for(int i = 0; i < nDoF; i++)
	{
		for(int j = 0; j < nDoF; j++)
		{
			dataJs(i,j) = dBufJs[k++];// * dBufWs[n];
		}
	dataWeights(i,i) = dBufLambda[l++];
	Parameters[1][i] = soth::Bound(dBufps[i]);// * dBufWs[n++]);
	}
	for(int i = wrenchLength; i < wrenchLength + rhoSize; i++)
	{
		Parameters[0][i] = soth::Bound(dBufrhoMin[i-wrenchLength],soth::Bound::BOUND_INF);
	}
	for(int i = wrenchLength + rhoSize; i < wrenchLength + rhoSize + phiSize; i++)
	{
		Parameters[0][i] = soth::Bound(dBufphiMin[i-(wrenchLength + rhoSize)],dBufphiMax[i-(wrenchLength + rhoSize)]);
	}
	k = 0;
	for(int j = nDoF; j < nDoF + rhoSize; j++)
	{
		dataWeights(j,j) = dBufWRho[k++];
	}
	k = 0;
	for(int j = nDoF + rhoSize; j < nDoF + rhoSize + phiSize; j++)
	{
		dataWeights(j,j) = dBufWPhi[k++];
	}
	for(int i = 0; i < nDoF + rhoSize + phiSize; i++)
	{
		Parameters[3][i] = soth::Bound(0);
	}
	Constraints[0] = dataContactForce.operator *=(1);
	Constraints[1] = dataJs.operator *=(1);
	Constraints[2] = dataA.operator *=(1);
	Constraints[3] = dataWeights.operator *=(1);

	HCOD hsolver(108,1);
	hsolver.pushBackStage(Constraints[0], Parameters[0]);
	hsolver.pushBackStage(Constraints[1], Parameters[1]);
	hsolver.pushBackStage(Constraints[2], Parameters[2]);
	hsolver.pushBackStage(Constraints[3], Parameters[3]);

	if(res.size()>0)
	{
		hsolver.setInitialActiveSet(res);
	}
	else
	{
		hsolver.setInitialActiveSet();
	}

	VectorXd solution(vdSize+rhoSize+phiSize); //0.13 ms
	hsolver.activeSearch(solution);
	res = hsolver.getOptimalActiveSet();

	for(int i = 0; i < vdSize+rhoSize+phiSize; i++)
	{
		dBufx[i] = solution[i];
	} // 4.57 ms
//	for(int i = 98; i < 108; i++)
//	{
//		cout<< solution[i]<<endl;
//	}
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getABuffer(JNIEnv *env, jclass jClass)
{
	return AByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getbBuffer(JNIEnv *env, jclass jClass)
{
	return bByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getCBuffer(JNIEnv *env, jclass jClass)
{
	return CByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getJsBuffer(JNIEnv *env, jclass jClass)
{
	return JsByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getpsBuffer(JNIEnv *env, jclass jClass)
{
	return psByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getWsBuffer(JNIEnv *env, jclass jClass)
{
	return WsByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getQrhoBuffer(JNIEnv *env, jclass jClass)
{
	return QrhoByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getQphiBuffer(JNIEnv *env, jclass jClass)
{
	return QphiByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getcBuffer(JNIEnv *env, jclass jClass)
{
	return cByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getrhoMinBuffer(JNIEnv *env, jclass jClass)
{
	return rhoMinByteBuffer;
}
JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getphiMinBuffer(JNIEnv *env, jclass jClass)
{
	return phiMinByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getphiMaxBuffer(JNIEnv *env, jclass jClass)
{
	return phiMaxByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getLambdaBuffer(JNIEnv *env, jclass jClass)
{
	return LambdaByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getWRhoBuffer(JNIEnv *env, jclass jClass)
{
	return WRhoByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getWPhiBuffer(JNIEnv *env, jclass jClass)
{
	return WPhiByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_HQPNative_getxBuffer(JNIEnv *env, jclass jClass)
{
	return xByteBuffer;
}

