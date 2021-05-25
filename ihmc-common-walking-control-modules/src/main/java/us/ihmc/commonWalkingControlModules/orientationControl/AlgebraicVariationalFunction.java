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
   private final DMatrixRMaj eye = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 3);

   private final DMatrixRMaj tauD = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj RBd = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RB = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj wBd = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wBdHat = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj RBdTtau = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj RBdTtauHat = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj Iwd = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj IwdHat = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj wdI = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj temp = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj P = new DMatrixRMaj(6, 6);

   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   private final RotationMatrix rotationMatrix = new RotationMatrix();

   public void setDesired(QuaternionReadOnly desiredRotation,
                          Vector3DReadOnly desiredAngularVelocity,
                          Vector3DReadOnly desiredTorque,
                          DMatrixRMaj Q,
                          DMatrixRMaj R,
                          DMatrixRMaj inertia,
                          DMatrixRMaj inertiaInverse)
   {
      desiredRotation.get(rotationMatrix);
      rotationMatrix.get(RBd);
      desiredAngularVelocity.get(wBd);
      desiredTorque.get(tauD);

      MatrixMissingTools.toSkewSymmetricMatrix(wBd, wBdHat);

      CommonOps_DDRM.multTransA(RBd, tauD, RBdTtau);
      MatrixMissingTools.toSkewSymmetricMatrix(RBdTtau, RBdTtauHat);

      CommonOps_DDRM.mult(inertiaInverse, RBdTtauHat, A21);

      CommonOps_DDRM.mult(inertia, wBd, Iwd);
      MatrixMissingTools.toSkewSymmetricMatrix(Iwd, IwdHat);
      CommonOps_DDRM.mult(wBdHat, inertia, wdI);

      CommonOps_DDRM.subtract(IwdHat, wdI, temp);
      CommonOps_DDRM.mult(inertiaInverse, temp, A22);

      CommonOps_DDRM.multTransB(inertiaInverse, RBd, B2);

      assembleAMatrix();
      assembleBMatrix();

      careSolver.setMatrices(A, B, null, Q, R);
      P.set(careSolver.getP());
   }

   private void assembleAMatrix()
   {
      MatrixMissingTools.setMatrixBlock(A, 0, 0, wBdHat, 0, 0, 3, 3, -1.0);
      MatrixMissingTools.setMatrixBlock(A, 0, 3, eye, 0, 0, 3, 3, 1.0);
      MatrixMissingTools.setMatrixBlock(A, 3, 0, A21, 0, 0, 3, 3, 1.0);
      MatrixMissingTools.setMatrixBlock(A, 3, 3, A22, 0, 0, 3, 3, 1.0);
   }

   private void assembleBMatrix()
   {
      MatrixMissingTools.setMatrixBlock(B, 3, 0, B2, 0, 0, 3, 3, 1.0);
   }
}
