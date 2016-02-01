#include "Test.h"
#include <Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdR;
JNIEXPORT void JNICALL Java_Test_inverse (JNIEnv *env, jclass cls,jdoubleArray jm1,  jdoubleArray jm2, jdoubleArray jr, jint nrow, jint ncol)
{
	jboolean isCopy=JNI_TRUE;
	double* pm1=(double*)env->GetPrimitiveArrayCritical(jm1, &isCopy);
	double* pm2=(double*)env->GetPrimitiveArrayCritical(jm2, &isCopy);
	double* pr=(double*)env->GetPrimitiveArrayCritical(jr, &isCopy);
	Eigen::Map<MatrixXdR> em1(pm1, nrow, ncol);
	Eigen::Map<MatrixXdR> em2(pm2, nrow, ncol);
	Eigen::Map<MatrixXdR> er(pr, nrow, ncol);
	er=em1*em2; //.inverse();

	/*
	std::cout << "----" << std::endl;
	std::cout << em1 << std::endl;
	std::cout << em2 << std::endl;
	std::cout << er << std::endl;
	*/
	env->ReleasePrimitiveArrayCritical(jm1, pm1, 0);
	env->ReleasePrimitiveArrayCritical(jm2, pm2, 0);
	env->ReleasePrimitiveArrayCritical(jr, pr, 0);
}


JNIEXPORT void inverse(double* m1, double* m2, double* r, int nrow, int ncol)
{
	Eigen::Map<MatrixXdR> em1(m1, nrow, ncol);
	Eigen::Map<MatrixXdR> em2(m2, nrow, ncol);
	Eigen::Map<MatrixXdR> er(r, nrow, ncol);
	//er=em1*em2; //.inverse();
}
