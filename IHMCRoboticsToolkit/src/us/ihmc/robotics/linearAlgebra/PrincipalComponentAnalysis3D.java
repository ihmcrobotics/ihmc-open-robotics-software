package us.ihmc.robotics.linearAlgebra;

import java.util.List;

import org.ejml.alg.dense.decomposition.svd.SvdImplicitQrDecompose_D64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.SingularOps;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

/**
 * Compute the singular value decomposition of a data matrix to find the three principal axes (called: principal axis, secondary axis, and third axis) and the associated variance.
 * <li> The principal axis is defined as the axis along which the provided data has the highest variance. </li>
 * <li> The secondary axis is defined as the axis orthogonal to the principal axis and along which the provided data has the maximum variance after the principal axis. </li>
 * <li> The third axis is defined as the axis orthogonal to the principal axis and to the secondary axis. It also along this axis that the provided data has the lowest variance. </li>
 * <p>
 * These three axes are provided as unit vectors forming a direct coordinate system. Methods are provided to also obtain the variance and/or the standard variation along each of these axes.
 * </p>
 * <p>
 * The algorithm is inspired from <a href="https://en.wikipedia.org/wiki/Principal_component_analysis"> Principal Component Analysis</a>.
 *
 */
public class PrincipalComponentAnalysis3D
{
   private static final boolean DEBUG = false;

   private final DenseMatrix64F W = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F V = new DenseMatrix64F(3, 3);

   /** Axis along which the data has the highest variance. It is a unit vector. */
   private final Vector3D principalAxis = new Vector3D();
   /** Axis along which the data has the highest variance after the {@link #principalAxis}. It is orthogonal to the {@link #principalAxis}. It is a unit vector. */
   private final Vector3D secondaryAxis = new Vector3D();
   /** Axis along which the data has the lowest variance. It is orthogonal to the {@link #principalAxis} and {@link #secondaryAxis}. It is a unit vector. */
   private final Vector3D thirdAxis = new Vector3D();

   private final Vector3D variance = new Vector3D();

   private final IncrementalCovariance3D covarianceCalculator = new IncrementalCovariance3D();
   private final DenseMatrix64F covariance = new DenseMatrix64F(3, 3);

   private final SingularValueDecomposition<DenseMatrix64F> svd;

   public PrincipalComponentAnalysis3D()
   {
      svd = new SvdImplicitQrDecompose_D64(true, false, true, false);
   }

   public void clear()
   {
      covarianceCalculator.clear();
   }

   public void addPoint(double x, double y, double z)
   {
      covarianceCalculator.addDataPoint(x, y, z);
   }

   /**
    * This method clears the current data held by this and add all the points from the given point cloud.
    * Use this method before {@link #compute()} to provide the point cloud to be analyzed.
    * @param pointCloud
    */
   public void setPointCloud(List<? extends Tuple3DBasics> pointCloud)
   {
      clear();

      covarianceCalculator.clear();
      covarianceCalculator.addAllDataPoints(pointCloud);

      if (DEBUG)
         System.out.println("PointCloud: \n" + pointCloud);
   }

   /**
    * This method clears the current data held by this and add all the points from the given point cloud.
    * Use this method before {@link #compute()} to provide the point cloud to be analyzed.
    * @param pointCloud matrix holding the point cloud data. Its size has to be either n-by-3 or 3-by-n, where n is the number of points.
    */
   public void setPointCloud(DenseMatrix64F pointCloud)
   {
      covarianceCalculator.clear();
      covarianceCalculator.addAllDataPoint(pointCloud);

      if (DEBUG)
         System.out.println("PointCloud: \n" + pointCloud);
   }

   /**
    * Performs the singular value decomposition to get the principal axes and the variance of the given dataset.
    * The point cloud needs to be provided before being able to call {@link #compute()}.
    */
   public void compute()
   {
      if (covarianceCalculator.getSampleSize() < 3)
      {
         principalAxis.set(Double.NaN, Double.NaN, Double.NaN);
         secondaryAxis.set(Double.NaN, Double.NaN, Double.NaN);
         thirdAxis.set(Double.NaN, Double.NaN, Double.NaN);
         variance.set(0.0, 0.0, 0.0);
         return;
      }

      covarianceCalculator.getCovariance(covariance);
      svd.decompose(covariance);

      svd.getW(W);
      svd.getV(V, false);
      SingularOps.descendingOrder(null, false, W, V, false);

      if (DEBUG)
         System.out.println("V: \n" + V);

      principalAxis.set(V.get(0, 0), V.get(1, 0), V.get(2, 0));
      secondaryAxis.set(V.get(0, 1), V.get(1, 1), V.get(2, 1));
      thirdAxis.cross(principalAxis, secondaryAxis);

      if (DEBUG)
         System.out.println("W: \n" + W);

      variance.setX(W.get(0, 0));
      variance.setY(W.get(1, 1));
      variance.setZ(W.get(2, 2));
   }

   /**
    * Pack the average of the provided point cloud.
    * @param meanToPack
    */
   public void getMean(Point3D meanToPack)
   {
      covarianceCalculator.getMean(meanToPack);
   }

   /**
    * Stores the three principal axes in the given Matrix3d such that it can be used as the rotation matrix describing the rotation from the principal frame to the parent coordinate system.
    * @param rotationMatrixToPack
    */
   public void getPrincipalFrameRotationMatrix(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.setColumns(principalAxis, secondaryAxis, thirdAxis);
   }

   /**
    * Pack the variance along each principal axis in the given Vector3d.
    * @param principalVarianceToPack The variance is stored in the Vector3d as follows: x is the variance on the principal axis, y on the secondary axis, and z on the third axis.
    */
   public void getVariance(Vector3D principalVarianceToPack)
   {
      principalVarianceToPack.setX(variance.getX());
      principalVarianceToPack.setY(variance.getY());
      principalVarianceToPack.setZ(variance.getZ());
   }

   /**
    * Pack the standard deviation along each principal axis in the Vector3d.
    * @param principalStandardDeviationToPack The standard deviation is stored in the Vector3d as follows: x is the standard deviation on the principal axis, y on the secondary axis, and z on the third axis.
    */
   public void getStandardDeviation(Vector3D principalStandardDeviationToPack)
   {
      principalStandardDeviationToPack.setX(Math.sqrt(variance.getX()));
      principalStandardDeviationToPack.setY(Math.sqrt(variance.getY()));
      principalStandardDeviationToPack.setZ(Math.sqrt(variance.getZ()));
   }

   /**
    * Pack the the three principal vectors in the three given vectors as follows:
    * @param principalAxisToPack is set to the principal axis of unit length, it is the axis along which the variance is the greatest.
    * @param secondaryAxisToPack is set to the secondary axis of unit length, it is the axis along which the variance is the greatest after the principal axis.
    * @param thirdAxisToPack is set to the third axis of unit length, it is the axis along which the variance is the least.
    */
   public void getPrincipalVectors(Vector3D principalAxisToPack, Vector3D secondaryAxisToPack, Vector3D thirdAxisToPack)
   {
      principalAxisToPack.set(principalAxis);
      secondaryAxisToPack.set(secondaryAxis);
      thirdAxisToPack.set(thirdAxis);
   }

   /**
    * Get the axis along which the variance is the greatest.
    * @param principalVectorToPack
    */
   public void getPrincipalVector(Vector3D principalVectorToPack)
   {
      principalVectorToPack.set(principalAxis);
   }

   /**
    * Get the axis along which the variance is the greatest after the principal axis.
    * @param secondaryVectorToPack
    */
   public void getSecondaryVector(Vector3D secondaryVectorToPack)
   {
      secondaryVectorToPack.set(secondaryAxis);
   }

   /**
    * Get the axis along which the variance is the least.
    * @param thirdVectorToPack
    */
   public void getThirdVector(Vector3D thirdVectorToPack)
   {
      thirdVectorToPack.set(thirdAxis);
   }

   /**
    * Pack the the three principal vectors in the three given vectors as follows:
    * @param principalVectorToPack is set to the principal axis scaled by the variance along this axis.
    * @param secondaryVectorToPack is set to the secondary axis scaled by the variance along this axis.
    * @param thirdVectorToPack is set to the third axis scaled by the third variance along this axis.
    */
   public void getScaledPrincipalVectors(Vector3D principalVectorToPack, Vector3D secondaryVectorToPack, Vector3D thirdVectorToPack)
   {
      principalVectorToPack.set(principalAxis);
      principalVectorToPack.scale(variance.getX());
      secondaryVectorToPack.set(secondaryAxis);
      secondaryVectorToPack.scale(variance.getY());
      thirdVectorToPack.set(thirdAxis);
      thirdVectorToPack.scale(variance.getZ());
   }
}
