package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;


import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Random;

public class OdometryToolsTest
{
   @Test
   public void testToSkewSymmetric()
   {
      Random random = new Random(1738L);

      for (int i = 0; i < 1000; i++)
      {
         Vector3D a = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D b = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D cExpected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D c = EuclidCoreRandomTools.nextVector3D(random);

         cExpected.cross(a, b);

         DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);

         DMatrixRMaj bVector = new DMatrixRMaj(3, 1);
         b.get(bVector);
         DMatrixRMaj cVector = new DMatrixRMaj(3, 1);
         OdometryTools.toSkewSymmetricMatrix(a, skewMatrix);

         CommonOps_DDRM.mult(skewMatrix, bVector, cVector);
         c.set(cVector);

         EuclidCoreTestTools.assertEquals(cExpected, c, 1e-8);
      }
   }

   @Test
   public void testTransform()
   {
      Random random = new Random(1738L);

      for (int i = 0; i < 1000; i++)
      {
         Quaternion a = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D b = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D cExpected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D c = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D cInverseExpected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D cInverse = EuclidCoreRandomTools.nextVector3D(random);

         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
         DMatrixRMaj rotationMatrixTranspose = new DMatrixRMaj(3, 3);
         DMatrixRMaj bVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj cVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj cInverseVector = new DMatrixRMaj(3, 1);
         OdometryTools.toRotationMatrix(a, rotationMatrix);
         CommonOps_DDRM.transpose(rotationMatrix, rotationMatrixTranspose);
         OdometryTools.toRotationMatrix(a, rotationMatrix);
         b.get(bVector);

         CommonOps_DDRM.mult(rotationMatrix, bVector, cVector);
         CommonOps_DDRM.mult(rotationMatrixTranspose, bVector, cInverseVector);
         c.set(cVector);
         cInverse.set(cInverseVector);

         a.transform(b, cExpected);
         a.inverseTransform(b, cInverseExpected);

         EuclidCoreTestTools.assertEquals(cExpected, c, 1e-5);
         EuclidCoreTestTools.assertEquals(cInverseExpected, cInverse, 1e-5);
      }
   }

   @Test
   public void testToRotationMatrix()
   {
      Random random = new Random(1738L);

      for (int i = 0; i < 1000; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);

         OdometryTools.toRotationMatrix(quaternion, rotationMatrix);

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

         OdometryTools.toRotationMatrixInverse(quaternion, rotationMatrixInverse);

         RotationMatrix mat = new RotationMatrix(quaternion);
         DMatrixRMaj rotationExpected = new DMatrixRMaj(3, 3);
         mat.get(rotationExpected);

         CommonOps_DDRM.invert(rotationExpected);

         EjmlUnitTests.assertEquals(rotationExpected, rotationMatrixInverse, 1e-8);
      }
   }
}
