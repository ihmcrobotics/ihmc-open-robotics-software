/*solve a simple HQP problem up to four stages:
	min ||x^2||
	subject to
	Ax=b
	Cx=d (optional)
	Ex=f (optional)
	Gx=h (optional)
*/

/*
TODO: ADD constraintTypes e.g. bType=1 is equality bType=2 is smaller than etc
*/

#include <jni.h>
#include "SimpleHQPNative.h"
#include <iostream>
#include <sstream>
#include <fstream>

#include "soth/HCOD.hpp"
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace soth;
using std::endl;
using std::cout;
using std::cerr;
using std::vector;

#define nParams us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_nParams

int nStages, ASize, bSize, AInit, CSize, dSize, CInit, ESize, fSize, EInit, GSize, hSize, GInit, xSize;

jobject AByteBuffer, bByteBuffer, bTypeByteBuffer, CByteBuffer, dByteBuffer, dTypeByteBuffer, EByteBuffer,
		fByteBuffer, fTypeByteBuffer, GByteBuffer, hByteBuffer, hTypeByteBuffer, xByteBuffer, ParamsByteBuffer;

// Temporarily assign an address that will be freed and overwritten in initialize
double * AAddress = new double[1];
double * bAddress = new double[1];
double * bTypeAddress = new double[1];
double * CAddress = new double[1];
double * dAddress = new double[1];
double * dTypeAddress = new double[1];
double * EAddress = new double[1];
double * fAddress = new double[1];
double * fTypeAddress = new double[1];
double * GAddress = new double[1];
double * hAddress = new double[1];
double * hTypeAddress = new double[1];
double * xAddress = new double[1];
double * ParAddress = new double[1];

std::vector<cstref_vector_t> res;

unsigned int NB_HIERARCHICAL_STAGES, NB_VARIABLES;
vector<unsigned int> NB_CONSTRAINTS;
vector<Eigen::MatrixXd> Constraints;
vector<soth::VectorBound> Parameters;
Eigen::MatrixXd dataA, dataC, dataE, dataG;

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_initParams(JNIEnv *env, jclass jClass)
{
	double * Par = new double[nParams];

	delete ParAddress;
	ParAddress = Par;
	ParamsByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(ParAddress, sizeof(double) * nParams));
}

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_initialize(JNIEnv *env, jclass jClass)
{
	double *dBufParams = (double*)env->GetDirectBufferAddress(ParamsByteBuffer);
	nStages=dBufParams[0];
	ASize=dBufParams[1];
	bSize=dBufParams[2];
	AInit=dBufParams[3];
	CSize=dBufParams[4];
	dSize=dBufParams[5];
	CInit=dBufParams[6];
	ESize=dBufParams[7];
	fSize=dBufParams[8];
	EInit=dBufParams[9];
	GSize=dBufParams[10];
	hSize=dBufParams[11];
	GInit=dBufParams[12];
	xSize=dBufParams[13];

	NB_HIERARCHICAL_STAGES = nStages;
	NB_CONSTRAINTS += bSize,dSize, fSize, hSize;

	NB_VARIABLES = xSize;

	Constraints.resize(NB_HIERARCHICAL_STAGES);
	Parameters.resize(NB_HIERARCHICAL_STAGES);

	for(int i = 0; i< NB_HIERARCHICAL_STAGES; i++)
	{
		Constraints[i].resize(NB_CONSTRAINTS[i], NB_VARIABLES);
		Constraints[i].setZero();
		Parameters[i].resize(NB_CONSTRAINTS[i]);
	}

	dataA.resize(bSize, xSize);
	dataA.setZero();
	dataC.resize(dSize, xSize);
	dataC.setZero();
	dataE.resize(fSize, xSize);
	dataE.setZero();
	dataG.resize(hSize,xSize);
	dataG.setZero();

	double * A = new double[bSize*ASize];
	double * b = new double[bSize];
	double * bType = new double[bSize];
	double * C = new double[dSize*CSize];
	double * d = new double[dSize];
	double * dType = new double[dSize];
	double * E = new double[fSize*ESize];
	double * f = new double[fSize];
	double * fType = new double[fSize];
	double * G = new double[hSize*GSize];
	double * h = new double[hSize];
	double * hType = new double[hSize];
	double * x = new double[xSize];

	delete AAddress, bAddress, bTypeAddress, CAddress, dAddress, dTypeAddress, EAddress, fAddress, fTypeAddress, GAddress, hAddress, hTypeAddress, xAddress;
	AAddress = A;
	bAddress = b;
	bTypeAddress = bType;
	CAddress = C;
	dAddress = d;
	dTypeAddress = dType;
	EAddress = E;
	fAddress = f;
	fTypeAddress = fType;
	GAddress = G;
	hAddress = h;
	hTypeAddress = hType;
	xAddress = x;

	AByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(AAddress, sizeof(double) * bSize*ASize));
	bByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(bAddress, sizeof(double) * bSize));
	bTypeByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(bTypeAddress, sizeof(double) * bSize));	
	CByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(CAddress, sizeof(double) * dSize*CSize));
	dByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(dAddress, sizeof(double) * dSize));
	dTypeByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(dTypeAddress, sizeof(double) * dSize));
	EByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(EAddress, sizeof(double) * fSize*ESize));
	fByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(fAddress, sizeof(double) * fSize));
	fTypeByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(fTypeAddress, sizeof(double) * fSize));
	GByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(GAddress, sizeof(double) * hSize*GSize));
	hByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(hAddress, sizeof(double) * hSize));
	hTypeByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(hTypeAddress, sizeof(double) * hSize));

	xByteBuffer = env->NewGlobalRef(env->NewDirectByteBuffer(x, sizeof(double) * xSize));
}

JNIEXPORT void JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_solveInCPP(JNIEnv *env, jclass jClass)
{

	double *dBufA = (double*)env->GetDirectBufferAddress(AByteBuffer);
	double *dBufb = (double*)env->GetDirectBufferAddress(bByteBuffer);
	double *dBufbType = (double*)env->GetDirectBufferAddress(bTypeByteBuffer);
	double *dBufC = (double*)env->GetDirectBufferAddress(CByteBuffer);
	double *dBufd = (double*)env->GetDirectBufferAddress(dByteBuffer);
	double *dBufdType = (double*)env->GetDirectBufferAddress(dTypeByteBuffer);
	double *dBufE = (double*)env->GetDirectBufferAddress(EByteBuffer);
	double *dBuff = (double*)env->GetDirectBufferAddress(fByteBuffer);
	double *dBuffType = (double*)env->GetDirectBufferAddress(fTypeByteBuffer);
	double *dBufG = (double*)env->GetDirectBufferAddress(GByteBuffer);
	double *dBufh = (double*)env->GetDirectBufferAddress(hByteBuffer);
	double *dBufhType = (double*)env->GetDirectBufferAddress(hTypeByteBuffer);
	double *dBufx = (double*)env->GetDirectBufferAddress(xByteBuffer);

	int k=0;
	for(int i = 0; i < bSize; i++)
	{
		for(int j = AInit; j < AInit+ASize; j++)
		{
			dataA(i,j) = dBufA[k++];
		}
	}

	k=0;
	for(int i = 0; i < dSize; i++)
	{
		for(int j = CInit; j < CInit+CSize; j++)
		{
			dataC(i,j) = dBufC[k++];
		}
	}

	k=0;
	for(int i = 0; i < fSize; i++)
	{
		for(int j = EInit; j < EInit+ESize; j++)
		{
			dataE(i,j) = dBufE[k++];
		}
	}

	k=0;
	for(int i = 0; i < hSize; i++)
	{
		for(int j = GInit; j < GInit+GSize; j++)
		{
			dataG(i,j) = dBufG[k++];
		}
	}

	for(int i = 0; i < bSize; i++)
	{
		switch ((int)dBufbType[i])
		{
			case 0:		
				Parameters[0][i] = soth::Bound(dBufb[i]);
				break;
			case 1:		
				Parameters[0][i] = soth::Bound(dBufb[i],soth::Bound::BOUND_INF);
				break;
			case 2:		
				Parameters[0][i] = soth::Bound(dBufb[i],soth::Bound::BOUND_SUP);
				break;
			default:
				Parameters[0][i] = soth::Bound(dBufb[i]);
				std::cout << "Using equality constraint as default, please give a proper bound type" << std::endl;
				break;
		}
	}

	for(int i = 0; i < dSize; i++)
	{
		switch ((int)dBufdType[i])
		{
			case 0:		
				Parameters[1][i] = soth::Bound(dBufd[i]);
				break;
			case 1:		
				Parameters[1][i] = soth::Bound(dBufd[i],soth::Bound::BOUND_INF);
				break;
			case 2:		
				Parameters[1][i] = soth::Bound(dBufd[i],soth::Bound::BOUND_SUP);
				break;
			default:
				Parameters[1][i] = soth::Bound(dBufd[i]);
				std::cout << "Using equality constraint as default, please give a proper bound type" << std::endl;
				break;
		}
	}

	for(int i = 0; i < fSize; i++)
	{
		switch ((int)dBuffType[i])
		{
			case 0:		
				Parameters[2][i] = soth::Bound(dBuff[i]);
				break;
			case 1:		
				Parameters[2][i] = soth::Bound(dBuff[i],soth::Bound::BOUND_INF);
				break;
			case 2:		
				Parameters[2][i] = soth::Bound(dBuff[i],soth::Bound::BOUND_SUP);
				break;
			default:
				Parameters[2][i] = soth::Bound(dBuff[i]);
				std::cout << "Using equality constraint as default, please give a proper bound type" << std::endl;
				break;
		}
	}

	for(int i = 0; i < hSize; i++)
	{
		switch ((int)dBufhType[i])
		{
			case 0:		
				Parameters[3][i] = soth::Bound(dBufh[i]);
				break;
			case 1:		
				Parameters[3][i] = soth::Bound(dBufh[i],soth::Bound::BOUND_INF);
				break;
			case 2:		
				Parameters[3][i] = soth::Bound(dBufh[i],soth::Bound::BOUND_SUP);
				break;
			default:
				Parameters[3][i] = soth::Bound(dBufh[i]);
				std::cout << "Using equality constraint as default, please give a proper bound type" << std::endl;
				break;
		}
	}

	if(nStages>0)
	{
		Constraints[0] = dataA.operator *=(1);
		if(nStages>1)
		{
			Constraints[1] = dataC.operator *=(1);
			if(nStages>2)
			{
				Constraints[2] = dataE.operator *=(1);
				if(nStages>3)
				{
				Constraints[3] = dataG.operator *=(1);
				}
			}
		}
	}

	HCOD hsolver(xSize,1);
	for(int i=0;i<nStages;i++)
	{
		hsolver.pushBackStage(Constraints[i], Parameters[i]);
	}

	if(res.size()>0)
	{
		hsolver.setInitialActiveSet(res);
	}
	else
	{
		hsolver.setInitialActiveSet();
	}

	VectorXd solution(xSize); 
	hsolver.activeSearch(solution);
	res = hsolver.getOptimalActiveSet();

	for(int i = 0; i < xSize; i++)
	{
		dBufx[i] = solution[i];
	} 

	//logging
	const std::string name = "logfileOfProblem";
	std::ofstream fout(name.c_str());
	fout << "variable size " << NB_VARIABLES << endl << endl;

	for( unsigned int s=0;s<NB_HIERARCHICAL_STAGES;++s )
	{
		fout << "level" << endl << endl;

		int nbEqualities = 0;
		for( unsigned int r=0;r<NB_CONSTRAINTS[s];++r )
		{
			if( Parameters[s][r].getType() == Bound::BOUND_TWIN ) nbEqualities++;
		}

		fout << "equalities " << nbEqualities << endl << endl;
		for( unsigned int r=0;r<NB_CONSTRAINTS[s];++r )
		{
			if( Parameters[s][r].getType() == Bound::BOUND_TWIN )
			{
				fout << Constraints[s].row(r) << "   " << Parameters[s][r].getBound( Bound::BOUND_TWIN ) << endl;
			}
		}
		fout << endl << "inequalities " << NB_CONSTRAINTS[s]-nbEqualities << endl << endl;
		for( unsigned int r=0;r<NB_CONSTRAINTS[s];++r )
		{
			switch(Parameters[s][r].getType())
			{
			case Bound::BOUND_TWIN:
				break;
			case Bound::BOUND_INF:
				fout << Constraints[s].row(r) << "   " << Parameters[s][r].getBound( Bound::BOUND_INF ) << " 1e25" << endl;
				break;
			case Bound::BOUND_SUP:
				fout << Constraints[s].row(r) << "   " << -1e25 << " " << Parameters[s][r].getBound( Bound::BOUND_SUP ) << endl;
				break;
			case Bound::BOUND_DOUBLE:
				fout << Constraints[s].row(r) << "   " <<  Parameters[s][r].getBound( Bound::BOUND_INF ) << " " << Parameters[s][r].getBound( Bound::BOUND_SUP ) << endl;
				break;
			case Bound::BOUND_NONE:
				assert( Parameters[s][r].getType()!= Bound::BOUND_NONE );
			}
		}
		fout << endl;
	}

	fout << endl << "end" << endl << endl;
	//end of logging
}

JNIEXPORT void JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_clearMemory(JNIEnv *env, jclass jClass)
{
	delete AAddress, bAddress, bTypeAddress, CAddress, dAddress, dTypeAddress, EAddress, fAddress, fTypeAddress, GAddress, hAddress, hTypeAddress, xAddress, ParAddress;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getABuffer(JNIEnv *env, jclass jClass)
{
	return AByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getbBuffer(JNIEnv *env, jclass jClass)
{
	return bByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getbTypeBuffer(JNIEnv *env, jclass jClass)
{
	return bTypeByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getCBuffer(JNIEnv *env, jclass jClass)
{
	return CByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getdBuffer(JNIEnv *env, jclass jClass)
{
	return dByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getdTypeBuffer(JNIEnv *env, jclass jClass)
{
	return dTypeByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getEBuffer(JNIEnv *env, jclass jClass)
{
	return EByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getfBuffer(JNIEnv *env, jclass jClass)
{
	return fByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getfTypeBuffer(JNIEnv *env, jclass jClass)
{
	return fTypeByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getGBuffer(JNIEnv *env, jclass jClass)
{
	return GByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_gethBuffer(JNIEnv *env, jclass jClass)
{
	return hByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_gethTypeBuffer(JNIEnv *env, jclass jClass)
{
	return hTypeByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getxBuffer(JNIEnv *env, jclass jClass)
{
	return xByteBuffer;
}

JNIEXPORT jobject JNICALL
Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_SimpleHQPNative_getParamsBuffer(JNIEnv *env, jclass jClass)

{
	return ParamsByteBuffer;
}

