package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.Random;

public class VariationalDynamicsCalculatorTest
{
   @Test
   public void testFormulation()
   {
      VariationalDynamicsCalculator dynamicsCalculator = new VariationalDynamicsCalculator();
      Random random = new Random(1738L);
      int iters = 1000;
      for (int iter = 0; iter < iters; iter++)
      {
         QuaternionReadOnly desiredRotation = EuclidCoreRandomTools.nextQuaternion(random, Math.toRadians(20.0));
         Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame = EuclidCoreRandomTools.nextVector3D(random, 10.0);
         Vector3DReadOnly desiredTorque = EuclidCoreRandomTools.nextVector3D(random, 10.0);

         DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
         DMatrixRMaj inertiaInverse = new DMatrixRMaj(3, 3);
         for (int i = 0; i < 3; i++)
            inertia.set(i, i, RandomNumbers.nextDouble(random, 0.1, 10.0));
         UnrolledInverseFromMinor_DDRM.inv3(inertia, inertiaInverse, 1.0);

         dynamicsCalculator.compute(desiredRotation,
                                    desiredBodyAngularVelocityInBodyFrame,
                                    desiredTorque,
                                    inertia,
                                    inertiaInverse);

         DMatrixRMaj skewDesiredAngularVelocity = new DMatrixRMaj(3, 3);

         MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocityInBodyFrame, skewDesiredAngularVelocity);

         DMatrixRMaj A = new DMatrixRMaj(6, 6);
         MatrixTools.setMatrixBlock(A, 0, 0, skewDesiredAngularVelocity, 0, 0, 3, 3, -1.0);
         MatrixTools.setMatrixBlock(A, 0, 3, CommonOps_DDRM.identity(3), 0, 0, 3, 3, 1.0);

         RotationMatrix rotation = new RotationMatrix();
         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
         desiredRotation.get(rotation);
         rotation.get(rotationMatrix);

         DMatrixRMaj IR = new DMatrixRMaj(3, 3);
         CommonOps_DDRM.multTransB(inertiaInverse, rotationMatrix, IR);

         DMatrixRMaj tauVector = new DMatrixRMaj(3, 1);
         DMatrixRMaj skewTau = new DMatrixRMaj(3, 3);
         desiredTorque.get(tauVector);
         MatrixMissingTools.toSkewSymmetricMatrix(tauVector, skewTau);

         DMatrixRMaj wVector = new DMatrixRMaj(3, 1);
         desiredBodyAngularVelocityInBodyFrame.get(wVector);
         DMatrixRMaj wI = new DMatrixRMaj(3, 3);
         DMatrixRMaj Iw = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(skewDesiredAngularVelocity, inertia, wI);
         CommonOps_DDRM.mult(inertia, wVector, Iw);

         DMatrixRMaj IwSkew = new DMatrixRMaj(3, 3);
         MatrixMissingTools.toSkewSymmetricMatrix(Iw, IwSkew);


         DMatrixRMaj temp = new DMatrixRMaj(3, 3);
         CommonOps_DDRM.subtract(IwSkew, wI, temp);

         DMatrixRMaj A21 = new DMatrixRMaj(3, 3);
         DMatrixRMaj A22 = new DMatrixRMaj(3, 3);

         CommonOps_DDRM.mult(IR, skewTau, A21);
         CommonOps_DDRM.mult(inertiaInverse, temp, A22);

         MatrixTools.setMatrixBlock(A, 3, 0, A21, 0, 0, 3, 3, 1.0);
         MatrixTools.setMatrixBlock(A, 3, 3, A22, 0, 0, 3, 3, 1.0);

         MatrixTestTools.assertMatrixEquals(A, dynamicsCalculator.getA(), 1e-5);

         DMatrixRMaj B = new DMatrixRMaj(6, 3);
         MatrixTools.setMatrixBlock(B, 3, 0, IR, 0, 0, 3, 3, 1.0);
         MatrixTestTools.assertMatrixEquals(B, dynamicsCalculator.getB(), 1e-5);
      }
   }
}
