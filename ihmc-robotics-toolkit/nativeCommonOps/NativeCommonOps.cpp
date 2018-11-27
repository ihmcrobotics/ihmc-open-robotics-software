/*
 * NativeCommonOps.cpp
 *
 *  Created on: Nov 27, 2018
 *      Author: Georg Wiedebach
 */

#include <jni.h>
#include <Eigen/Dense>
#include "us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper.h"

using Eigen::MatrixXd;

JNIEXPORT void JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_computeAB(JNIEnv *env, jobject thisObj, jdoubleArray result,
		jdoubleArray aData, jdoubleArray bData, jint aRows, jint aCols, jint bCols)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aCols);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, aCols, bCols);

	MatrixXd AB = A * B;

	jdouble *resultDataArray = new jdouble[aRows * bCols];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aRows, bCols) = AB;
	env->SetDoubleArrayRegion(result, 0, aRows * bCols, resultDataArray);

	delete aDataArray;
	delete bDataArray;
	delete resultDataArray;
}
