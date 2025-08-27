package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;


import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;

import java.util.Random;

public class OdometryKalmanFilterTest
{
   @Test
   public void testToRotationMatrix()
   {
      Random random = new Random(1738L);

      for (int i = 0; i < 1000; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);

         OdometryKalmanFilter.toRotationMatrix(quaternion, rotationMatrix);

         RotationMatrix mat = new RotationMatrix(quaternion);
         DMatrixRMaj expected = new DMatrixRMaj(3, 3);
         mat.get(expected);

         EjmlUnitTests.assertEquals(expected, rotationMatrix, 1e-8);
      }
   }

   @Test
   public void testToRotationMatrixInverse()
   {
      Random random = new Random(1738L);

      for (int i = 0; i < 1000; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         DMatrixRMaj rotationMatrixInverse = new DMatrixRMaj(3, 3);

         OdometryKalmanFilter.toRotationMatrixInverse(quaternion, rotationMatrixInverse);

         RotationMatrix mat = new RotationMatrix(quaternion);
         DMatrixRMaj rotationExpected = new DMatrixRMaj(3, 3);
         mat.get(rotationExpected);

         CommonOps_DDRM.invert(rotationExpected);

         EjmlUnitTests.assertEquals(rotationExpected, rotationMatrixInverse, 1e-8);
      }
   }
}
