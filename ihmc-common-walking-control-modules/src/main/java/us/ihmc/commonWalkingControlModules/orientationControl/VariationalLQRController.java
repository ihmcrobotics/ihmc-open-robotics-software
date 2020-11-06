package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ojalgo.matrix.transformation.Rotation;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;

public class VariationalLQRController
{
   private static final double defaultQR = 1100;
   private static final double defaultQw = 5;
   private static final double defaultR = 1.25;

   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   private final DMatrixRMaj A21 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj A22 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj B2 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj eye = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 3);

   private final DMatrixRMaj QR = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj Qw = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj Q = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj R = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RInverse = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj BTP = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj K = new DMatrixRMaj(3, 6);

   private final DMatrixRMaj wBd = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wBdHat = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RBd = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RB = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj wB = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj RBerror = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RBerrorVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wBerror = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj state = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj RBdTtau = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj RBdTtauHat = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj Iwd = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj IwdHat = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj wdI = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj temp = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj tauD = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj tau = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj deltaTau = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj inertiaInverse = new DMatrixRMaj(3, 3);

   public VariationalLQRController()
   {
      CommonOps_DDRM.setIdentity(eye);
      CommonOps_DDRM.setIdentity(inertia);

      MatrixTools.setDiagonal(QR, defaultQR);
      MatrixTools.setDiagonal(Qw, defaultQw);
      MatrixTools.setDiagonal(R, defaultR);
   }

   public void setInertia(SpatialInertiaReadOnly inertia)
   {
      inertia.getMomentOfInertia().get(this.inertia);
   }

   private final RotationMatrix rotationMatrix = new RotationMatrix();
   public void setDesired(QuaternionReadOnly desiredRotation, Vector3DReadOnly desiredAngularVelocity, Vector3DReadOnly desiredTorque)
   {
      // FIXME check these frames
      desiredRotation.get(rotationMatrix);
      rotationMatrix.get(RBd);
      desiredAngularVelocity.get(wBd);
      desiredTorque.get(tauD);

      makeSkewSymmetric(wBd, wBdHat);

      fast3x3Inverse(inertia, inertiaInverse);

      CommonOps_DDRM.multTransA(RBd, tauD, RBdTtau);
      makeSkewSymmetric(RBdTtau, RBdTtauHat);

      CommonOps_DDRM.mult(inertiaInverse, RBdTtauHat, A21);

      CommonOps_DDRM.mult(inertia, wBd, Iwd);
      makeSkewSymmetric(Iwd, IwdHat);
      CommonOps_DDRM.mult(wBdHat, inertia, wdI);

      CommonOps_DDRM.subtract(IwdHat, wdI, temp);
      CommonOps_DDRM.mult(inertiaInverse, temp, A22);

      CommonOps_DDRM.multTransB(inertiaInverse, RBd, B2);

      assembleAMatrix();
      assembleBMatrix();
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

   public void compute(QuaternionReadOnly currentRotation, Vector3DReadOnly currentAngularVelocity)
   {
      assembleQ();

      careSolver.setMatrices(A, B, null, Q, R);

      CommonOps_DDRM.multTransA(B, careSolver.getP(), BTP);
      CommonOps_DDRM.mult(RInverse, BTP, K);

      currentRotation.get(rotationMatrix);
      rotationMatrix.get(RB);
      currentAngularVelocity.get(wB);

      CommonOps_DDRM.multTransA(-1.0, RB, RBd, RBerror);
      CommonOps_DDRM.mult(RBerror, wBd, wBerror);
      CommonOps_DDRM.addEquals(wBerror, wB);

      CommonOps_DDRM.multAddTransA(RBd, RB, RBerror);
      CommonOps_DDRM.scale(0.5, RBerror);

      fromSkewSymmetric(RBerror, RBerrorVector);

      MatrixTools.setMatrixBlock(state, 0, 0, wBerror, 0, 0, 3, 1, 1.0);
      MatrixTools.setMatrixBlock(state, 3, 0, RBerrorVector, 0, 0, 3, 1, 1.0);

      CommonOps_DDRM.mult(-1.0, K, state, deltaTau);
      CommonOps_DDRM.add(deltaTau, tauD, tau);
   }

   public void getDesiredTorque(Vector3DBasics tau)
   {
      tau.set(this.tau);
   }

   private void assembleQ()
   {
      MatrixMissingTools.setMatrixBlock(Q, 0, 0, QR, 0, 0, 3, 3, 1.0);
      MatrixMissingTools.setMatrixBlock(Q, 3, 3, Qw, 0, 0, 3, 3, 1.0);
   }

   private static void makeSkewSymmetric(DMatrixRMaj m, DMatrixRMaj mHatToPack)
   {
      if (m.getNumRows() != 3)
         throw new IllegalArgumentException("Not the right number of rows.");
      if (m.getNumCols() != 1)
         throw new IllegalArgumentException("m is not a column vector.");
      if (mHatToPack.getNumRows() != 3 || mHatToPack.getNumCols() != 3)
         throw new IllegalArgumentException("Matrix to pack is the wrong size.");

      mHatToPack.zero();
      mHatToPack.set(0, 1, -m.get(2));
      mHatToPack.set(0, 2, m.get(1));
      mHatToPack.set(1, 0, m.get(2));
      mHatToPack.set(1, 2, -m.get(0));
      mHatToPack.set(2, 0, -m.get(1));
      mHatToPack.set(2, 1, m.get(0));
   }

   private static void fromSkewSymmetric(DMatrixRMaj mHat, DMatrixRMaj mToPack)
   {
      if (mToPack.getNumRows() != 3)
         throw new IllegalArgumentException("Not the right number of rows.");
      if (mToPack.getNumCols() != 1)
         throw new IllegalArgumentException("m is not a column vector.");
      if (mHat.getNumRows() != 3 || mHat.getNumCols() != 3)
         throw new IllegalArgumentException("Matrix to pack is the wrong size.");

      mToPack.set(0, mHat.get(2, 1));
      mToPack.set(1, mHat.get(0, 2));
      mToPack.set(2, mHat.get(1, 0));
   }

   private static void fast3x3Inverse(DMatrixRMaj matrix, DMatrixRMaj matrixInverseToPack)
   {
      double det = CommonOps_DDRM.det(matrix);
      fast3x3Adjugate(matrix, matrixInverseToPack);
      CommonOps_DDRM.scale(1.0 / det, matrixInverseToPack);
   }

   private static void fast3x3Adjugate(DMatrixRMaj matrix, DMatrixRMaj matrixAdjugateToPack)
   {
      double a11 = matrix.get(0, 0);
      double a22 = matrix.get(1, 1);
      double a33 = matrix.get(2, 2);
      double a12 = matrix.get(0, 1);
      double a13 = matrix.get(0, 2);
      double a21 = matrix.get(1, 0);
      double a23 = matrix.get(1, 2);
      double a31 = matrix.get(2, 0);
      double a32 = matrix.get(2, 1);

      matrixAdjugateToPack.set(0, 0, a22 * a33 - a23 * a32);
      matrixAdjugateToPack.set(0, 1, a13 * a32 - a12 * a33);
      matrixAdjugateToPack.set(0, 2, a12 * a23 - a13 * a22);
      matrixAdjugateToPack.set(1, 0, a23 * a31 - a21 - a33);
      matrixAdjugateToPack.set(1, 1, a11 * a33 - a13 * a31);
      matrixAdjugateToPack.set(1, 2, a13 * a21 - a11 * a23);
      matrixAdjugateToPack.set(2, 0, a21 * a32 - a22 * a31);
      matrixAdjugateToPack.set(2, 1, a12 * a31 - a11 * a32);
      matrixAdjugateToPack.set(2, 2, a11 * a22 - a12 * a21);
   }
}
