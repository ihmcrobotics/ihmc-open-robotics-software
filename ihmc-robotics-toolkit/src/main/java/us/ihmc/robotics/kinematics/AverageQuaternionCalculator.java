package us.ihmc.robotics.kinematics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.math.linearAlgebra.SymmetricQRAlgorithmDecomposition_D64GCFree;

/**
 * Algorithm designed based on this <a href="http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf"> paper</a>.
 * @author Sylvain
 *
 */
public class AverageQuaternionCalculator
{
   private final SymmetricQRAlgorithmDecomposition_D64GCFree eigenDecomposition;
   private final DMatrixRMaj quaternions = new DMatrixRMaj(0, 4);
   private final DMatrixRMaj outerProduct = new DMatrixRMaj(0, 0);
   private final Quaternion averageQuaternion = new Quaternion();
   private final Quaternion tempQuaternion = new Quaternion();

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

   public void queueQuaternion(Quaternion quaternion)
   {
      quaternions.reshape(quaternions.getNumRows() + 1, 4, true);
      int lastIndex = quaternions.getNumRows() - 1;
      quaternions.set(lastIndex, 0, quaternion.getX());
      quaternions.set(lastIndex, 1, quaternion.getY());
      quaternions.set(lastIndex, 2, quaternion.getZ());
      quaternions.set(lastIndex, 3, quaternion.getS());
   }

   public void queueFrameOrientation(FrameQuaternion frameOrientation)
   {
      if (referenceFrame != null)
         referenceFrame.checkReferenceFrameMatch(frameOrientation);
      tempQuaternion.set(frameOrientation);
      queueQuaternion(tempQuaternion);
   }

   public void queueAxisAngle(AxisAngle axisAngle)
   {
      tempQuaternion.set(axisAngle);
      queueQuaternion(tempQuaternion);
   }

   public void queueMatrix(RotationMatrix rotationMatrix)
   {
      tempQuaternion.set(rotationMatrix);
      queueQuaternion(tempQuaternion);
   }

   public void compute()
   {
      double weight = 1.0 / quaternions.getNumRows();
      outerProduct.reshape(4, 4);
      CommonOps_DDRM.multAddTransA(weight, quaternions, quaternions, outerProduct);
      eigenDecomposition.decompose(outerProduct);
      
      double maxEigenValue = Double.NEGATIVE_INFINITY;
      DMatrixRMaj eigenVectorAssociatedWithMaxEigenValue = null;

      for (int i = 0; i < eigenDecomposition.getNumberOfEigenvalues(); i++)
      {
         double eigenValue = eigenDecomposition.getEigenValueAsDouble(i);
         if (eigenValue > maxEigenValue)
         {
            maxEigenValue = eigenValue;
            eigenVectorAssociatedWithMaxEigenValue = eigenDecomposition.getEigenVector(i);
         }
      }

      double x = eigenVectorAssociatedWithMaxEigenValue.get(0, 0);
      double y = eigenVectorAssociatedWithMaxEigenValue.get(1, 0);
      double z = eigenVectorAssociatedWithMaxEigenValue.get(2, 0);
      double s = eigenVectorAssociatedWithMaxEigenValue.get(3, 0);
      averageQuaternion.set(x, y, z, s);
   }

   public void getAverageQuaternion(Quaternion averageQuaternionToPack)
   {
      averageQuaternionToPack.set(averageQuaternion);
   }

   public void getAverageFrameOrientation(FrameQuaternion averageFrameOrientationToPack)
   {
      if (referenceFrame != null)
         referenceFrame.checkReferenceFrameMatch(averageFrameOrientationToPack);
      averageFrameOrientationToPack.set(averageQuaternion);
   }

   public void getAverageAxisAngle(AxisAngle averageAxisAngleToPack)
   {
      averageAxisAngleToPack.set(averageQuaternion);
   }

   public void getAverageMatrix(RotationMatrix averageRotationMatrixToPack)
   {
      averageRotationMatrixToPack.set(averageQuaternion);
   }
}
