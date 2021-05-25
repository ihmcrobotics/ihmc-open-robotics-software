package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;

public class AlgebraicVariationalFunction
{
   private final DMatrixRMaj A21 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj A22 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj B2 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj identity = CommonOps_DDRM.identity(3);
   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 3);

   private final DMatrixRMaj desiredTorque = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj RBd = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj wBd = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wBdHat = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj RBdTtau = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj RBdTtauHat = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj skewAngularMomentum = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj wdI = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj temp = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj P = new DMatrixRMaj(6, 6);

   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   private final RotationMatrix rotationMatrix = new RotationMatrix();

   public void setDesired(QuaternionReadOnly desiredRotation,
                          Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame,
                          Vector3DReadOnly desiredNetAngularMomentum,
                          DMatrixRMaj Q,
                          DMatrixRMaj R,
                          DMatrixRMaj inertia,
                          DMatrixRMaj inertiaInverse)
   {
      desiredRotation.get(rotationMatrix);
      rotationMatrix.get(RBd);
      desiredBodyAngularVelocityInBodyFrame.get(wBd);
      desiredNetAngularMomentum.get(desiredTorque);

      MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocityInBodyFrame, wBdHat);

      CommonOps_DDRM.multTransA(RBd, desiredTorque, RBdTtau);
      MatrixMissingTools.toSkewSymmetricMatrix(RBdTtau, RBdTtauHat);

      CommonOps_DDRM.mult(inertiaInverse, RBdTtauHat, A21);

      CommonOps_DDRM.mult(inertia, wBd, angularMomentum);
      MatrixMissingTools.toSkewSymmetricMatrix(angularMomentum, skewAngularMomentum);
      CommonOps_DDRM.mult(wBdHat, inertia, wdI);

      CommonOps_DDRM.subtract(skewAngularMomentum, wdI, temp);
      CommonOps_DDRM.mult(inertiaInverse, temp, A22);

      CommonOps_DDRM.multTransB(inertiaInverse, RBd, B2);

      assembleAMatrix();
      assembleBMatrix();

      careSolver.setMatrices(A, B, null, Q, R);
      P.set(careSolver.getP());
   }

   public DMatrixRMaj getP()
   {
      return P;
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
}
