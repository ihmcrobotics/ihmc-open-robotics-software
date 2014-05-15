#include "ActiveSetQPMomentumOptimizer.h"

#define USE_ACTIVE_SET
//#define DUMP_TO_FILE

#ifdef USE_ACTIVE_SET
#include "ActiveSetQP/fastQP.h"
#else
#include "ActiveSetQP/eiquadprog.hpp"
#endif

#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <set>
#include <iostream>
#include <fstream>
#define nRho us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ActiveSetQPMomentumOptimizer_nRho
#define nWrench us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ActiveSetQPMomentumOptimizer_nWrench 

typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXdR;

#define J2E(X, R, C)  double * c ## X = (double*)env->GetPrimitiveArrayCritical(X, &isCopy);   \
				Eigen::Map<MatrixXdR > (e ## X)(c ## X, R, C);

#define J2E_FREE(X) env->ReleasePrimitiveArrayCritical(X, c ## X, 0);

#define JNA2E(X, R, C)  Eigen::Map<MatrixXdR> (e ## X)(X, R, C);

int nDoF=-1;
jboolean isCopy=JNI_FALSE;
using namespace Eigen;

extern "C"
{
	extern int MAX_ITER; //instance in QP.cpp
	JNIEXPORT void initializeNative (int _nDoF, int _max_iter)
	{
		nDoF = _nDoF;
		MAX_ITER = _max_iter;
		std::cerr << "ActiveSetQPMomentumOptimizer Library initialized nDoF = " << nDoF << " MAX_ITER="<< MAX_ITER <<std::endl;
		std::cerr << "Compiled at " __TIME__ << std::endl;
	}
}

JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ActiveSetQPMomentumOptimizer_initializeNative (JNIEnv *env, jobject obj, jint _nDoF, jint _max_iter)
{
//	std::cout << "use JNI" << std::endl;
	initializeNative(_nDoF, _max_iter);
}




bool myIsNAN(double x)
{
	return !(x==x);
}

int solveEigen(
		MatrixXd A, MatrixXd  b, MatrixXd  C, 
		MatrixXd Jp, MatrixXd  pp,
		MatrixXd Js, MatrixXd  ps, MatrixXd  Ws, 
		MatrixXd WRho, MatrixXd  Lambda, 
		MatrixXd WRhoSmoother, 
		MatrixXd rhoPrevMean, MatrixXd  WRhoCoPPenalty, 
		MatrixXd QRho, MatrixXd  c, MatrixXd  rhoMin, 
		Eigen::Map<MatrixXdR>&  vd, Eigen::Map<MatrixXdR>& rho, 
		int* activeSet)
{

	/*
	 *       [vd  ]'[ A' C A + Js' Ws Js + Lambda     0               ] [vd  ]  + 2 [vd  ]' [ -A' C b - Js' Ws ps          ]    +  b' C b + ps' Ws ps + Pprev' Wpsm Pprev + Ppavg' Wpcop Ppavg 
	 *       [rho ] [              0                Wp + Wpsm + Wpcop ] [rho ]      [rho ]  [-Wpsm Pprev - Wpcop Ppavg     ] 
	 *
	 *       min_x 0.5 x'Q x  + f'x  + g
	 *       st [-A QRho ] [vd  ] = [c ]
	 *          [Jp      ] [rho ] = [pp]
	 *        [0 -I] rho <= -rhoMin
	 */

	//Objective
	//Q
	std::vector<MatrixXd *> QblkDiag;
	MatrixXd Qblk1 = A.transpose()*C.diagonal().asDiagonal()*A + Js.transpose()*Ws.diagonal().asDiagonal()*Js + Lambda;
	MatrixXd Qblk2 = WRho.diagonal() + WRhoSmoother.diagonal() + WRhoCoPPenalty.diagonal();
	QblkDiag.push_back(&Qblk1);
	QblkDiag.push_back(&Qblk2);

#ifdef DUMP_TO_FILE
	MatrixXd Q(nDoF+nRho,nDoF+nRho);
	Q << Qblk1, MatrixXd::Zero(nDoF,nRho), MatrixXd::Zero(nRho,nDoF), MatrixXd(Qblk2.asDiagonal()); //Qblk2;
#endif
	//f
	VectorXd f(nRho+nDoF);
	f << (- A.transpose() * C.diagonal().asDiagonal() * b - Js.transpose()*Ws.diagonal().asDiagonal()*ps),
	  	 (- MatrixXd(WRhoSmoother.diagonal().asDiagonal())*rho - WRhoCoPPenalty.diagonal().asDiagonal()*rhoPrevMean);

	MatrixXd g = b.transpose()*C.diagonal().asDiagonal()*b + ps.transpose()*Ws.diagonal().asDiagonal()*ps 
		+ rho.transpose()*WRhoSmoother.diagonal().asDiagonal()*rho + rhoPrevMean.transpose()*WRhoCoPPenalty*rhoPrevMean;

	//Equality
	MatrixXd Aeq,beq;
	if(myIsNAN(Jp(0,0)))
	{
		Aeq.resize(nWrench, nDoF+nRho);
		Aeq << -A, QRho;
		beq.resize(nWrench,nDoF+nRho);
		beq = c;
	}
	else
	{
		/*  Aeq - 
		 *    nDoF  nRho
		 *  ----------------
		 *  | -A  |  QRho |  nWrench
		 *  ----------------
		 *  | Jp     Zero |  nDoF
		 *  ----------------
		 */
		Aeq.resize(nWrench+nDoF, nDoF+nRho);
		Aeq << -A, QRho, Jp, MatrixXd::Zero(nDoF, nRho) ;
		beq.resize(nWrench+nDoF,1);
		beq << c, pp;
	}

	//Inequality
	MatrixXd Ain(nRho, nDoF+nRho);
	Ain << MatrixXd::Zero(nRho,nDoF), -MatrixXd::Identity(nRho,nRho);
	VectorXd bin = -rhoMin;


	//final Call
	VectorXd x(nRho+nDoF);
	x.head(nDoF) =  vd;
	x.tail(nRho) = rho;


#define REG  1e-9
#ifdef  USE_ACTIVE_SET
	for(int i=0;i<QblkDiag.size();i++)
	{
		if(QblkDiag[i]->cols()==1)
			QblkDiag[i]->operator+=(VectorXd::Constant(QblkDiag[i]->rows(),REG));
		else
			QblkDiag[i]->diagonal() += VectorXd::Ones(QblkDiag[i]->rows())*REG;
	}
#ifdef DUMP_TO_FILE
	//use blkDiagQ in ActiveSet
	Q.diagonal() += VectorXd::Ones(nRho+nDoF)*REG;
#endif //DUMP_TO_FILE

#else
	Aeq = MatrixXd(Aeq.transpose());
	beq = -beq;
	Ain = -MatrixXd(Ain.transpose());
	Q.diagonal() += VectorXd::Ones(nRho+nDoF)*REG;
#endif //USE_ACTIVE_SET

#ifdef DUMP_TO_FILE
	std::ofstream ofP("/tmp/P.txt");
	ofP << "Q" << std::endl   << Q << std::endl;
	ofP << "f" << std::endl << f << std::endl;
	ofP << "Aeq" << std::endl << Aeq << std::endl;
	ofP << "beq" << std::endl << beq << std::endl;
	ofP << "Ain" << std::endl << Ain << std::endl;
	ofP << "bin" << std::endl << bin << std::endl;
	ofP << "x0" << std::endl << x << std::endl;
	ofP.close();
#endif

#ifdef USE_ACTIVE_SET
	std::set<int> stlActiveSet;
	for(int i=0;i<Aeq.rows();i++)
	{
		if(activeSet[i])
			stlActiveSet.insert(i);
	}

	int ret=fastQP(QblkDiag, f,  Aeq, beq, Ain, bin, stlActiveSet, x);

	std::fill(activeSet, activeSet+Aeq.rows(),0);
	for(std::set<int>::iterator p=stlActiveSet.begin();p!=stlActiveSet.end();p++)
	{
		activeSet[*p]=1;
	}
#else
	int ret=solve_quadprog(Q, f,  Aeq, beq, Ain, bin, x);
#endif

	vd = x.head(nDoF);
	rho = x.tail(nRho);

#ifdef DUMP_TO_FILE
	std::ofstream of("/tmp/Q.txt");
	of << "A" << std::endl    << A << std::endl;
	of << "b" << std::endl    << b << std::endl;
	of << "C" << std::endl    << C << std::endl;

	of << "Jp" << std::endl   << Jp << std::endl;
	of << "pp" << std::endl   << pp << std::endl;

	of << "Js" << std::endl   << Js << std::endl;
	of << "ps" << std::endl   << ps << std::endl;
	of << "Ws" << std::endl   << Ws << std::endl;

	of << "WRho" << std::endl << WRho.diagonal()<< std::endl;
	of << "Lambda" << std::endl   << Lambda.diagonal() << std::endl;

	of << "Wpsm" << std::endl << WRhoSmoother.diagonal() << std::endl;

	of << "rhoPrevMean" << std::endl << rhoPrevMean << std::endl;
	of << "Wpcop" << std::endl << WRhoCoPPenalty.diagonal()<< std::endl;
	
	of << "Qrho" << std::endl << QRho << std::endl;
	of << "c" << std::endl << c << std::endl;
	of << "rhoMin" << std::endl << rhoMin << std::endl;


	of << "vd" << std::endl << vd << std::endl;
	of << "rho" << std::endl << rho << std::endl;

	of << "optVal" << std::endl << g(0,0)+2*ret << std::endl;
	of.close();
#endif
	return ret;
}

JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ActiveSetQPMomentumOptimizer_solveNative (JNIEnv *env, jobject obj, 
		jdoubleArray A, jdoubleArray b, jdoubleArray C, 
		jdoubleArray Jp, jdoubleArray pp, 
		jdoubleArray Js, jdoubleArray ps, jdoubleArray Ws, 
		jdoubleArray WRho, jdoubleArray Lambda, 
		jdoubleArray WRhoSmoother, 
		jdoubleArray rhoPrevMean, jdoubleArray WRhoCoPPenalty, 
		jdoubleArray QRho, jdoubleArray c, jdoubleArray rhoMin, 
		jdoubleArray vd, jdoubleArray rho, jintArray activeSet)
{
	int ret;
	J2E(A, nWrench, nDoF);  J2E(b,nWrench,1);  J2E(C,nWrench,nWrench); 
	J2E(Jp,nDoF,nDoF);  J2E(pp,nDoF,1); 
	J2E(Js,nDoF,nDoF);  J2E(ps,nDoF,1);  J2E(Ws,nDoF,nDoF); 
	J2E(WRho,nRho,nRho);  J2E(Lambda,nDoF,nDoF); 
	J2E(WRhoSmoother,nRho,nRho); 
	J2E(rhoPrevMean,nRho,1);  J2E(WRhoCoPPenalty,nRho,nRho); 
	J2E(QRho,nWrench,nRho);  J2E(c,nWrench,1);  J2E(rhoMin,nRho,1); 
	J2E(vd,nDoF,1); J2E(rho,nRho,1); 
	int* cActiveSet = (int*)env->GetPrimitiveArrayCritical(activeSet, &isCopy);

	ret = solveEigen(eA,  eb,  eC, eJp, epp, eJs,  eps,  eWs, eWRho,  eLambda, eWRhoSmoother, erhoPrevMean,  eWRhoCoPPenalty, eQRho,  ec,  erhoMin, evd, erho, cActiveSet);

	J2E_FREE(A);  J2E_FREE(b);  J2E_FREE(C); 
	J2E_FREE(Jp);  J2E_FREE(pp); 
	J2E_FREE(Js);  J2E_FREE(ps);  J2E_FREE(Ws); 
	J2E_FREE(WRho);  J2E_FREE(Lambda); 
	J2E_FREE(WRhoSmoother); 
	J2E_FREE(rhoPrevMean);  J2E_FREE(WRhoCoPPenalty); 
	J2E_FREE(QRho);  J2E_FREE(c);  J2E_FREE(rhoMin); 
	J2E_FREE(vd); J2E_FREE(rho);  
	env->ReleasePrimitiveArrayCritical(activeSet, cActiveSet, 0);

	return ret;
}

extern "C" {
	JNIEXPORT int solveNative(
			double* A, double* b, double* C,
			double* Jp, double* pp,
			double* Js, double* ps, double* Ws,
			double* WRho, double* Lambda,
			double* WRhoSmoother,
			double* rhoPrevMean, double* WRhoCoPPenalty,
			double* QRho, double* c, double* rhoMin,
			double* vd, double* rho,
			int* activeSet)
	{
		int ret;
		JNA2E(A, nWrench, nDoF);  JNA2E(b,nWrench,1);  JNA2E(C,nWrench,nWrench);
		JNA2E(Jp,nDoF,nDoF);  JNA2E(pp,nDoF,1);
		JNA2E(Js,nDoF,nDoF);  JNA2E(ps,nDoF,1);  JNA2E(Ws,nDoF,nDoF);
		JNA2E(WRho,nRho,nRho);  JNA2E(Lambda,nDoF,nDoF);
		JNA2E(WRhoSmoother,nRho,nRho);
		JNA2E(rhoPrevMean,nRho,1);  JNA2E(WRhoCoPPenalty,nRho,nRho);
		JNA2E(QRho,nWrench,nRho);  JNA2E(c,nWrench,1);  JNA2E(rhoMin,nRho,1);
		JNA2E(vd,nDoF,1); JNA2E(rho,nRho,1);

		ret = solveEigen(eA,  eb,  eC, eJp, epp, eJs,  eps,  eWs, eWRho,  eLambda, eWRhoSmoother, erhoPrevMean,  eWRhoCoPPenalty, eQRho,  ec,  erhoMin, evd, erho, activeSet);
		return ret;
	}
}
