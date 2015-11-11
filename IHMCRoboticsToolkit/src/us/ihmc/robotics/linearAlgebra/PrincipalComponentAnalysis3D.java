package us.ihmc.robotics.linearAlgebra;

import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.ejml.alg.dense.decomposition.svd.SvdImplicitQrDecompose_D64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;

/**
 * Compute the singular value decomposition of a data matrix to find the three principal axes (called: principal axis, secondary axis, and third axis) and the associated variance.
 * <li> The principal axis is defined as the axis along which the provided data has the highest variance. </li>
 * <li> The secondary axis is defined as the axis orthogonal to the principal axis and along which the provided data has the maximum variance after the principal axis. </li>
 * <li> The third axis is defined as the axis orthogonal to the principal axis and to the secondary axis. It also along this axis that the provided data has the lowest variance. </li>
 * <p>
 * These three axes are provided as unit vectors forming a direct coordinate system. However, methods are provided to obtain the variance and/or the standard variation along each of these axes.
 * </p>
 *
 */
public class PrincipalComponentAnalysis3D
{
   private static final boolean DEBUG = false;

   private final DenseMatrix64F pointCloud = new DenseMatrix64F(1, 3);
   private final DenseMatrix64F V = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F W = new DenseMatrix64F(3, 3);

   private int numberOfPoints;

   /** Axis along which the data has the highest variance. It is a unit vector. */
   private final Vector3d principalAxis = new Vector3d();
   /** Axis along which the data has the highest variance after the {@link #principalAxis}. It is orthogonal to the {@link #principalAxis}. It is a unit vector. */
   private final Vector3d secondaryAxis = new Vector3d();
   /** Axis along which the data has the lowest variance. It is orthogonal to the {@link #principalAxis} and {@link #secondaryAxis}. It is a unit vector. */
   private final Vector3d thirdAxis = new Vector3d();

   private final Point3d mean = new Point3d();

   private final Vector3d principalValues = new Vector3d();

   private final SingularValueDecomposition<DenseMatrix64F> svd;

   public PrincipalComponentAnalysis3D()
   {
      svd = new SvdImplicitQrDecompose_D64(true, false, true, false);
   }

   /**
    * Use this method before {@link #compute()} to provide the point cloud to be analyzed.
    * @param pointCloud
    */
   public void setPointCloud(List<? extends Tuple3d> pointCloud)
   {
      numberOfPoints = pointCloud.size();

      this.pointCloud.reshape(numberOfPoints, 3);

      double scale = 1.0 / numberOfPoints;
      mean.set(0.0, 0.0, 0.0);
      for (int row = 0; row < numberOfPoints; row++)
      {
         mean.x += pointCloud.get(row).x * scale;
         mean.y += pointCloud.get(row).y * scale;
         mean.z += pointCloud.get(row).z * scale;
      }

      for (int row = 0; row < numberOfPoints; row++)
      {
         this.pointCloud.set(row, 0, pointCloud.get(row).x - mean.x);
         this.pointCloud.set(row, 1, pointCloud.get(row).y - mean.y);
         this.pointCloud.set(row, 2, pointCloud.get(row).z - mean.z);
      }

      if (DEBUG)
         System.out.println("PointCloud: \n" + pointCloud);
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(1, 1);

   public void setPointCloud(DenseMatrix64F pointCloud)
   {
      if (pointCloud.getNumRows() == 3)
      {
         numberOfPoints = pointCloud.getNumCols();
         tempMatrix.reshape(pointCloud.getNumCols(), pointCloud.getNumRows());
         CommonOps.transpose(pointCloud, tempMatrix);
      }
      else if (pointCloud.getNumCols() == 3)
      {
         numberOfPoints = pointCloud.getNumRows();
         tempMatrix.set(pointCloud);
      }
      else
         throw new RuntimeException("Unexpected size");

      this.pointCloud.reshape(numberOfPoints, 3);

      double scale = 1.0 / numberOfPoints;
      mean.set(0.0, 0.0, 0.0);
      for (int row = 0; row < numberOfPoints; row++)
      {
         mean.x += tempMatrix.get(row, 0) * scale;
         mean.y += tempMatrix.get(row, 1) * scale;
         mean.z += tempMatrix.get(row, 2) * scale;
      }

      for (int row = 0; row < numberOfPoints; row++)
      {
         this.pointCloud.set(row, 0, tempMatrix.get(row, 0) - mean.x);
         this.pointCloud.set(row, 1, tempMatrix.get(row, 1) - mean.y);
         this.pointCloud.set(row, 2, tempMatrix.get(row, 2) - mean.z);
      }

      if (DEBUG)
         System.out.println("PointCloud: \n" + pointCloud);
   }

   /**
    * Performs the singular value decomposition to get the principal axes and necessary to get information, such as the variance, on the data.
    * The point cloud needs to be provided before being able to call {@link #compute()}.
    */
   public void compute()
   {
      svd.decompose(pointCloud);

      svd.getV(V, false);

      if (DEBUG)
         System.out.println("V: \n" + V);

      principalAxis.set(V.get(0, 0), V.get(1, 0), V.get(2, 0));
      secondaryAxis.set(V.get(0, 1), V.get(1, 1), V.get(2, 1));
      thirdAxis.cross(principalAxis, secondaryAxis);

      svd.getW(W);

      if (DEBUG)
         System.out.println("W: \n" + W);

      principalValues.x = W.get(0, 0);
      principalValues.y = W.get(1, 1);
      principalValues.z = W.get(2, 2);
   }

   /**
    * Pack the average point of the provided point cloud.
    * @param mean
    */
   public void getMean(Point3d mean)
   {
      mean.set(this.mean);
   }

   /**
    * Stores the three principal axes in the given Matrix3d such that it can be used as the rotation matrix describing the rotation from the principal frame to the parent coordinate system.
    * @param rotationMatrixToPack
    */
   public void getPrincipalFrameRotationMatrix(Matrix3d rotationMatrixToPack)
   {
      rotationMatrixToPack.setColumn(0, principalAxis);
      rotationMatrixToPack.setColumn(1, secondaryAxis);
      rotationMatrixToPack.setColumn(2, thirdAxis);
   }

   /**
    * Pack the variance along each principal axis in the given Vector3d.
    * @param principalVarianceToPack The variance is stored in the Vector3d as follows: x is the variance on the principal axis, y on the secondary axis, and z on the third axis.
    */
   public void getVariance(Vector3d principalVarianceToPack)
   {
      principalVarianceToPack.x = principalValues.x * principalValues.x;
      principalVarianceToPack.y = principalValues.y * principalValues.y;
      principalVarianceToPack.z = principalValues.z * principalValues.z;
      principalVarianceToPack.scale(1.0 / numberOfPoints);
   }

   /**
    * Pack the standard deviation along each principal axis in the Vector3d.
    * @param principalStandardDeviationToPack The standard deviation is stored in the Vector3d as follows: x is the standard deviation on the principal axis, y on the secondary axis, and z on the third axis.
    */
   public void getStandardDeviation(Vector3d principalStandardDeviationToPack)
   {
      principalStandardDeviationToPack.x = Math.abs(principalValues.x);
      principalStandardDeviationToPack.y = Math.abs(principalValues.y);
      principalStandardDeviationToPack.z = Math.abs(principalValues.z);
      principalStandardDeviationToPack.scale(1.0 / Math.sqrt(numberOfPoints));
   }

   /**
    * Pack the the three principal vectors in the three given vectors as follows:
    * @param principalVectorToPack is set to the principal axis of unit length, computed by the singular value decomposition on the point cloud matrix.
    * @param secondaryVectorToPack is set to the secondary axis of unit length, computed by the singular value decomposition on the point cloud matrix.
    * @param thirdVectorToPack is set to the third axis of unit length, computed by the singular value decomposition on the point cloud matrix.
    */
   public void getPrincipalVectors(Vector3d principalVectorToPack, Vector3d secondaryVectorToPack, Vector3d thirdVectorToPack)
   {
      principalVectorToPack.set(principalAxis);
      secondaryVectorToPack.set(secondaryAxis);
      thirdVectorToPack.set(thirdAxis);
   }

   public void getPrincipalVector(Vector3d principalVectorToPack)
   {
      principalVectorToPack.set(principalVectorToPack);
   }

   /**
    * Pack the the three principal vectors in the three given vectors as follows:
    * @param principalVectorToPack is set to the principal axis scaled by the first singular value, computed by the singular value decomposition on the point cloud matrix.
    * @param secondaryVectorToPack is set to the secondary axis scaled by the second singular value, computed by the singular value decomposition on the point cloud matrix.
    * @param thirdVectorToPack is set to the third axis scaled by the third singular value, computed by the singular value decomposition on the point cloud matrix.
    */
   public void getScaledPrincipalVectors(Vector3d principalVectorToPack, Vector3d secondaryVectorToPack, Vector3d thirdVectorToPack)
   {
      principalVectorToPack.set(principalAxis);
      principalVectorToPack.scale(principalValues.x);
      secondaryVectorToPack.set(secondaryAxis);
      secondaryVectorToPack.scale(principalValues.y);
      thirdVectorToPack.set(thirdAxis);
      thirdVectorToPack.scale(principalValues.z);
   }
}
