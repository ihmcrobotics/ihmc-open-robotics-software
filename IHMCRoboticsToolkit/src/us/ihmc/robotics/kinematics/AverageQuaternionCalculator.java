package us.ihmc.robotics.kinematics;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.SymmetricQRAlgorithmDecomposition_D64GCFree;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Algorithm designed based on this <a href="http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf"> paper</a>.
 * @author Sylvain
 *
 */
public class AverageQuaternionCalculator
{
   private final SymmetricQRAlgorithmDecomposition_D64GCFree eigenDecomposition;
   private final DenseMatrix64F quaternions = new DenseMatrix64F(0, 4);
   private final DenseMatrix64F outerProduct = new DenseMatrix64F(0, 0);
   private final Quat4d averageQuaternion = new Quat4d();
   private final Quat4d tempQuaternion = new Quat4d();

   private ReferenceFrame referenceFrame = null;

   public AverageQuaternionCalculator()
   {
      eigenDecomposition = new SymmetricQRAlgorithmDecomposition_D64GCFree(true);
   }

   public void reset()
   {
      quaternions.reshape(0, 4);
      outerProduct.reshape(0, 0);
      referenceFrame = null;
   }

   public void queueQuaternion(Quat4d quaternion)
   {
      quaternions.reshape(quaternions.getNumRows() + 1, 4, true);
      int lastIndex = quaternions.getNumRows() - 1;
      quaternions.set(lastIndex, 0, quaternion.getX());
      quaternions.set(lastIndex, 1, quaternion.getY());
      quaternions.set(lastIndex, 2, quaternion.getZ());
      quaternions.set(lastIndex, 3, quaternion.getW());
   }

   public void queueFrameOrientation(FrameOrientation frameOrientation)
   {
      if (referenceFrame != null)
         referenceFrame.checkReferenceFrameMatch(frameOrientation);
      frameOrientation.getQuaternion(tempQuaternion);
      queueQuaternion(tempQuaternion);
   }

   public void queueAxisAngle(AxisAngle4d axisAngle)
   {
      tempQuaternion.set(axisAngle);
      queueQuaternion(tempQuaternion);
   }

   public void queueMatrix(Matrix3d rotationMatrix)
   {
      tempQuaternion.set(rotationMatrix);
      queueQuaternion(tempQuaternion);
   }

   public void compute()
   {
      double weight = 1.0 / quaternions.getNumRows();
      outerProduct.reshape(4, 4);
      CommonOps.multAddTransA(weight, quaternions, quaternions, outerProduct);
      eigenDecomposition.decompose(outerProduct);
      
      double maxEigenValue = Double.NEGATIVE_INFINITY;
      DenseMatrix64F eigenVectorAssociatedWithMaxEigenValue = null;

      for (int i = 0; i < eigenDecomposition.getNumberOfEigenvalues(); i++)
      {
         double eigenValue = eigenDecomposition.getEigenValueAsDouble(i);
         if (eigenValue > maxEigenValue)
         {
            maxEigenValue = eigenValue;
            eigenVectorAssociatedWithMaxEigenValue = eigenDecomposition.getEigenVector(i);
         }
      }

      averageQuaternion.setX(eigenVectorAssociatedWithMaxEigenValue.get(0, 0));
      averageQuaternion.setY(eigenVectorAssociatedWithMaxEigenValue.get(1, 0));
      averageQuaternion.setZ(eigenVectorAssociatedWithMaxEigenValue.get(2, 0));
      averageQuaternion.setW(eigenVectorAssociatedWithMaxEigenValue.get(3, 0));
   }

   public void getAverageQuaternion(Quat4d averageQuaternionToPack)
   {
      averageQuaternionToPack.set(averageQuaternion);
   }

   public void getAverageFrameOrientation(FrameOrientation averageFrameOrientationToPack)
   {
      if (referenceFrame != null)
         referenceFrame.checkReferenceFrameMatch(averageFrameOrientationToPack);
      averageFrameOrientationToPack.set(averageQuaternion);
   }

   public void getAverageAxisAngle(AxisAngle4d averageAxisAngleToPack)
   {
      averageAxisAngleToPack.set(averageQuaternion);
   }

   public void getAverageMatrix(Matrix3d averageRotationMatrixToPack)
   {
      averageRotationMatrixToPack.set(averageQuaternion);
   }
}
