/*
 * NativeCommonOps.cpp
 *
 *  Created on: Nov 27, 2018
 *      Author: Georg Wiedebach
 */

#include <jni.h>
#include <Eigen/Dense>
#include <iostream>
#include "us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper.h"

using Eigen::MatrixXd;

JNIEXPORT void JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_mult(JNIEnv *env, jobject thisObj,
		jdoubleArray result, jdoubleArray aData, jdoubleArray bData, jint aRows, jint aCols, jint bCols)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aCols);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, aCols, bCols);

	MatrixXd AB = A * B;

	jdouble *resultDataArray = new jdouble[aRows * bCols];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aRows, bCols) = AB;
	env->SetDoubleArrayRegion(result, 0, aRows * bCols, resultDataArray);

	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_multQuad(JNIEnv *env, jobject thisObj,
		jdoubleArray result, jdoubleArray aData, jdoubleArray bData, jint aRows, jint aCols)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aCols);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, aRows, aRows);

	MatrixXd AtBA = A.transpose() * B * A;

	jdouble *resultDataArray = new jdouble[aCols * aCols];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aCols, aCols) = AtBA;
	env->SetDoubleArrayRegion(result, 0, aCols * aCols, resultDataArray);

	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_invert(JNIEnv *env, jobject thisObj,
		jdoubleArray result, jdoubleArray aData, jint aRows)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aRows);

	MatrixXd x = A.lu().inverse();

	jdouble *resultDataArray = new jdouble[aRows * aRows];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aRows, aRows) = x;
	env->SetDoubleArrayRegion(result, 0, aRows * aRows, resultDataArray);

	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_solve(JNIEnv *env, jobject thisObj,
		jdoubleArray result, jdoubleArray aData, jdoubleArray bData, jint aRows)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aRows);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, aRows, 1);

	MatrixXd x = A.lu().solve(B);

	jdouble *resultDataArray = new jdouble[aRows];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aRows, 1) = x;
	env->SetDoubleArrayRegion(result, 0, aRows, resultDataArray);

	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
}

JNIEXPORT jboolean JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_solveCheck(JNIEnv *env, jobject thisObj,
		jdoubleArray result, jdoubleArray aData, jdoubleArray bData, jint aRows)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aRows);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, aRows, 1);

	const Eigen::FullPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > fullPivLu = A.fullPivLu();
	if (fullPivLu.isInvertible())
	{
		MatrixXd x = fullPivLu.solve(B);

		jdouble *resultDataArray = new jdouble[aRows];
		Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aRows, 1) = x;
		env->SetDoubleArrayRegion(result, 0, aRows, resultDataArray);

		delete resultDataArray;
		env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
		env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
		return true;
	}
	else
	{
		env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
		env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
		return false;
	}
}

JNIEXPORT void JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_solveRobust(JNIEnv *env, jobject thisObj,
		jdoubleArray result, jdoubleArray aData, jdoubleArray bData, jint aRows, jint aCols)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aCols);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, aRows, 1);

	MatrixXd x = A.householderQr().solve(B);

	jdouble *resultDataArray = new jdouble[aCols];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aCols, 1) = x;
	env->SetDoubleArrayRegion(result, 0, aCols, resultDataArray);

	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_solveDamped(JNIEnv *env, jobject thisObj,
		jdoubleArray result, jdoubleArray aData, jdoubleArray bData, jint aRows, jint aCols, jdouble alpha)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aCols);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, aRows, 1);

	MatrixXd outer = A * A.transpose() + MatrixXd::Identity(aRows, aRows) * alpha * alpha;
	MatrixXd x = A.transpose() * outer.llt().solve(B);

	jdouble *resultDataArray = new jdouble[aCols];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aCols, 1) = x;
	env->SetDoubleArrayRegion(result, 0, aCols, resultDataArray);

	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_robotics_linearAlgebra_commonOps_NativeCommonOpsWrapper_projectOnNullspace(JNIEnv *env, jobject thisObj,
		jdoubleArray result, jdoubleArray aData, jdoubleArray bData, jint aRows, jint aCols, jint bRows, jdouble alpha)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, aRows, aCols);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, bRows, aCols);

	MatrixXd BtB = B.transpose() * B;
	MatrixXd outer = BtB + MatrixXd::Identity(aCols, aCols) * alpha * alpha;
	MatrixXd x = A * (MatrixXd::Identity(aCols, aCols) - outer.llt().solve(BtB));

	jdouble *resultDataArray = new jdouble[aRows * aCols];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, aRows, aCols) = x;
	env->SetDoubleArrayRegion(result, 0, aRows * aCols, resultDataArray);

	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
	delete resultDataArray;
}
