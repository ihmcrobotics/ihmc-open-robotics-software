package us.ihmc.robotics.linearAlgebra;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import org.ejml.data.DenseMatrix64F;

/**
 * This class provides a storeless computation for a 3D covariance matrix.
 * Implementation from the algorithm described on Wikipedia:
 * <a href="https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance"> Algorithms for calculating variance </a>.
 * <p>
 * It seems that there is a debate on whether this should be named the covariance or variance matrix, see
 * <a href="https://en.wikipedia.org/wiki/Covariance_matrix"> Covariance matrix </a>
 * <p>
 * This class is a tool to compute the two following versions of the covariance matrix:
 * <ul>
 *   <li> Standard Covariance matrix: CoVar(X) = Y<sup>T</sup> * Y / n
 *   <li> Covariance matrix with <a href="https://en.wikipedia.org/wiki/Bessel%27s_correction"> Bessel's Correction </a>: CoVar(X) = Y<sup>T</sup> * Y / (n-1)
 *   <li>Where:
 *   <ul>
 *     <li> Y = X - <b>1</b> mean(X)
 *     <li> n is the size of the dataset
 *     <li> X represents the n-by-3 dataset
 *     <li> <b>1</b> is a n-by-1 vector filled with 1.
 *     <li> mean(X) is the 1-by-3 average vector of the dataset X.
 *   </ul>
 * </ul>
 * @author Sylvain
 *
 */
public class IncrementalCovariance3D
{
   private int sampleSize = 0;
   private final Point3d predictedMean = new Point3d();
   private final Point3d sum = new Point3d();
   private final DenseMatrix64F sumOfOuterProducts = new DenseMatrix64F(3, 3);

   public IncrementalCovariance3D()
   {
   }

   /**
    * Clear the current data.
    * If the mean of the next dataset is somewhat known, it is preferable to use {@link #clearAndSetPredictedMean(Tuple3d)}.
    */
   public void clear()
   {
      clearAndSetPredictedMean(0.0, 0.0, 0.0);
   }

   /**
    * Clear the current data and set the predicted mean to use afterwards.
    * @param predictedMean an estimate of the mean of the data points that will be added.
    * It is not required but improves the accuracy of the covariance matrix to be calculated.
    * It also does not have to be exact or even precise, as the covariance does not depend on it theoretically.
    * It should be within the dataset range and as close to the actual mean as possible.
    */
   public void clearAndSetPredictedMean(Tuple3d predictedMean)
   {
      clearAndSetPredictedMean(predictedMean.getX(), predictedMean.getY(), predictedMean.getZ());
   }

   /**
    * Clear the current data and set the predicted mean to use afterwards.
    * @param predictedMean an estimate of the mean of the data points that will be added.
    * It is not required but improves the accuracy of the covariance matrix to be calculated.
    */
   public void clearAndSetPredictedMean(Tuple3f predictedMean)
   {
      clearAndSetPredictedMean(predictedMean.getX(), predictedMean.getY(), predictedMean.getZ());
   }

   /**
    * Clear the current data and set the predicted mean to use afterwards.
    * The tuple (x, y, z) represents an estimate of the mean of the data points that will be added.
    * It is not required but improves the accuracy of the covariance matrix to be calculated.
    * @param x the x-coordinate of the predicted mean.
    * @param y the y-coordinate of the predicted mean.
    * @param z the z-coordinate of the predicted mean.
    */
   public void clearAndSetPredictedMean(double x, double y, double z)
   {
      predictedMean.set(x, y, z);
      sum.set(0.0, 0.0, 0.0);
      sampleSize = 0;
      sumOfOuterProducts.zero();
   }

   /**
    * Inserts a list of data points and updates the covariance matrix.
    * @param tuples the list of data points to insert.
    */
   public void addAllDataPoints(List<? extends Tuple3d> tuples)
   {
      for (int i = 0; i < tuples.size(); i++)
         addDataPoint(tuples.get(i));
   }

   public void addAllDataPoint(DenseMatrix64F dataPoints)
   {
      if (dataPoints.getNumRows() == 3)
      {
         for (int col = 0; col < dataPoints.getNumCols(); col++)
         {
            double x = dataPoints.get(0, col);
            double y = dataPoints.get(1, col);
            double z = dataPoints.get(2, col);
            addDataPoint(x, y, z);
         }
      }
      else if (dataPoints.getNumCols() == 3)
      {
         for (int row = 0; row < dataPoints.getNumRows(); row++)
         {
            double x = dataPoints.get(row, 0);
            double y = dataPoints.get(row, 1);
            double z = dataPoints.get(row, 2);
            addDataPoint(x, y, z);
         }
      }
      else
      {
         throw new RuntimeException("Unexpected matrix size: [nRows = " + dataPoints.getNumRows() + ", nCols = " + dataPoints.getNumCols() + "]");
      }
   }

   /**
    * Inserts a new data point and updates the covariance matrix.
    * @param tuple the new data point.
    */
   public void addDataPoint(Tuple3d tuple)
   {
      addDataPoint(tuple.getX(), tuple.getY(), tuple.getZ());
   }

   /**
    * Inserts a new data point and updates the covariance matrix.
    * @param tuple the new data point.
    */
   public void addDataPoint(Tuple3f tuple)
   {
      addDataPoint(tuple.getX(), tuple.getY(), tuple.getZ());
   }

   /**
    * Inserts a new data point (x, y, z) and updates the covariance matrix.
    * @param x the x-coordinate of the new data point.
    * @param y the y-coordinate of the new data point.
    * @param z the z-coordinate of the new data point.
    */
   public void addDataPoint(double x, double y, double z)
   {
      sampleSize++;
      x -= predictedMean.getX();
      y -= predictedMean.getY();
      z -= predictedMean.getZ();
      addToTuple(sum, x, y, z);

      double xx = x * x;
      double xy = x * y;
      double xz = x * z;
      double yy = y * y;
      double yz = y * z;
      double zz = z * z;

      sumOfOuterProducts.add(0, 0, xx);
      sumOfOuterProducts.add(0, 1, xy);
      sumOfOuterProducts.add(0, 2, xz);
      sumOfOuterProducts.add(1, 0, xy);
      sumOfOuterProducts.add(1, 1, yy);
      sumOfOuterProducts.add(1, 2, yz);
      sumOfOuterProducts.add(2, 0, xz);
      sumOfOuterProducts.add(2, 1, yz);
      sumOfOuterProducts.add(2, 2, zz);
   }

   /**
    * Removes a data point and updates the covariance matrix.
    * @param tuple the data point to remove.
    */
   public void removeDataPoint(Tuple3d tuple)
   {
      removeDataPoint(tuple.getX(), tuple.getY(), tuple.getZ());
   }

   /**
    * Removes a data point and updates the covariance matrix.
    * @param tuple the data point to remove.
    */
   public void removeDataPoint(Tuple3f tuple)
   {
      removeDataPoint(tuple.getX(), tuple.getY(), tuple.getZ());
   }

   /**
    * Removes a data point (x, y, z) and updates the covariance matrix.
    * @param x the x-coordinate of the data point to remove.
    * @param y the y-coordinate of the data point to remove.
    * @param z the z-coordinate of the data point to remove.
    */
   public void removeDataPoint(double x, double y, double z)
   {
      sampleSize--;
      x -= predictedMean.getX();
      y -= predictedMean.getY();
      z -= predictedMean.getZ();
      addToTuple(sum, -x, -y, -z);

      double xx = x * x;
      double xy = x * y;
      double xz = x * z;
      double yy = y * y;
      double yz = y * z;
      double zz = z * z;

      sumOfOuterProducts.add(0, 0, -xx);
      sumOfOuterProducts.add(0, 1, -xy);
      sumOfOuterProducts.add(0, 2, -xz);
      sumOfOuterProducts.add(1, 0, -xy);
      sumOfOuterProducts.add(1, 1, -yy);
      sumOfOuterProducts.add(1, 2, -yz);
      sumOfOuterProducts.add(2, 0, -xz);
      sumOfOuterProducts.add(2, 1, -yz);
      sumOfOuterProducts.add(2, 2, -zz);
   }

   /**
    * Get the covariance matrix corresponding to the dataset added beforehand.
    * @param covarianceToPack the 3-by-3 covariance matrix.
    */
   public void getCovariance(DenseMatrix64F covarianceToPack)
   {
      double div = 1.0 / (double) (sampleSize * sampleSize);
      computeCovariance(covarianceToPack, div);
   }

   /**
    * Get the covariance matrix corresponding to the dataset added beforehand using <a href="https://en.wikipedia.org/wiki/Bessel%27s_correction"> Bessel's Correction </a>.
    * @param covarianceToPack the 3-by-3 covariance matrix.
    */
   public void getCovarianceCorrected(DenseMatrix64F covarianceToPack)
   {
      double div = 1.0 / (double) (sampleSize * (sampleSize - 1.0));
      computeCovariance(covarianceToPack, div);
   }

   private void computeCovariance(DenseMatrix64F covarianceToPack, double div)
   {
      covarianceToPack.reshape(3, 3);

      double xx = - sum.getX() * sum.getX();
      double xy = - sum.getX() * sum.getY();
      double xz = - sum.getX() * sum.getZ();
      double yy = - sum.getY() * sum.getY();
      double yz = - sum.getY() * sum.getZ();
      double zz = - sum.getZ() * sum.getZ();

      xx += sampleSize * sumOfOuterProducts.get(0, 0);
      xy += sampleSize * sumOfOuterProducts.get(0, 1);
      xz += sampleSize * sumOfOuterProducts.get(0, 2);
      yy += sampleSize * sumOfOuterProducts.get(1, 1);
      yz += sampleSize * sumOfOuterProducts.get(1, 2);
      zz += sampleSize * sumOfOuterProducts.get(2, 2);

      xx *= div;
      xy *= div;
      xz *= div;
      yy *= div;
      yz *= div;
      zz *= div;

      covarianceToPack.set(0, 0, xx);
      covarianceToPack.set(0, 1, xy);
      covarianceToPack.set(0, 2, xz);
      covarianceToPack.set(1, 0, xy);
      covarianceToPack.set(1, 1, yy);
      covarianceToPack.set(1, 2, yz);
      covarianceToPack.set(2, 0, xz);
      covarianceToPack.set(2, 1, yz);
      covarianceToPack.set(2, 2, zz);
   }

   private void addToTuple(Tuple3d tupleToModify, double x, double y, double z)
   {
      tupleToModify.setX(tupleToModify.getX() + x);
      tupleToModify.setY(tupleToModify.getY() + y);
      tupleToModify.setZ(tupleToModify.getZ() + z);
   }
}
