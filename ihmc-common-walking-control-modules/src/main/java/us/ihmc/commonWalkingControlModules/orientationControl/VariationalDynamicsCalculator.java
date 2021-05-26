package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.MatrixMissingTools;

public class VariationalDynamicsCalculator
{
   private final DMatrixRMaj A21 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj A22 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj B2 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj identity = CommonOps_DDRM.identity(3);
   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 3);

   private final DMatrixRMaj desiredTorque = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj tauHat = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RBd = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj wBd = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wBdHat = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj skewAngularMomentum = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj wdI = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj temp = new DMatrixRMaj(3, 3);

   private final RotationMatrix rotationMatrix = new RotationMatrix();

   public void compute(QuaternionReadOnly desiredRotation,
                       Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame,
                       Vector3DReadOnly desiredNetAngularMomentum,
                       DMatrixRMaj inertia,
                       DMatrixRMaj inertiaInverse)
   {
      desiredRotation.get(rotationMatrix);
      rotationMatrix.get(RBd);
      desiredBodyAngularVelocityInBodyFrame.get(wBd);
      desiredNetAngularMomentum.get(desiredTorque);

      CommonOps_DDRM.multTransB(inertiaInverse, RBd, B2);

      MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocityInBodyFrame, wBdHat);
      MatrixMissingTools.toSkewSymmetricMatrix(desiredTorque, tauHat);

      CommonOps_DDRM.mult(B2, tauHat, A21);

      CommonOps_DDRM.mult(inertia, wBd, angularMomentum);
      MatrixMissingTools.toSkewSymmetricMatrix(angularMomentum, skewAngularMomentum);
      CommonOps_DDRM.mult(wBdHat, inertia, wdI);

      CommonOps_DDRM.subtract(skewAngularMomentum, wdI, temp);
      CommonOps_DDRM.mult(inertiaInverse, temp, A22);

      assembleAMatrix();
      assembleBMatrix();
   }

   private void assembleAMatrix()
   {
      MatrixMissingTools.setMatrixBlock(A, 0, 0, wBdHat, 0, 0, 3, 3, -1.0);
      MatrixMissingTools.setMatrixBlock(A, 0, 3, identity, 0, 0, 3, 3, 1.0);
      MatrixMissingTools.setMatrixBlock(A, 3, 0, A21, 0, 0, 3, 3, 1.0);
      MatrixMissingTools.setMatrixBlock(A, 3, 3, A22, 0, 0, 3, 3, 1.0);
   }

   private void assembleBMatrix()
   {
      MatrixMissingTools.setMatrixBlock(B, 3, 0, B2, 0, 0, 3, 3, 1.0);
   }

   public DMatrixRMaj getA()
   {
      return A;
   }

   public DMatrixRMaj getB()
   {
      return B;
   }
}
