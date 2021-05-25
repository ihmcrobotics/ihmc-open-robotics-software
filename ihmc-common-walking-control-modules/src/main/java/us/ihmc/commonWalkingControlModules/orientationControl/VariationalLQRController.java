package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.MatrixMissingTools;

public class VariationalLQRController
{
   private static final double defaultQR = 1100;
   private static final double defaultQw = 5;
   private static final double defaultR = 1.25;

   private final DMatrixRMaj B = new DMatrixRMaj(6, 3);

   private final DMatrixRMaj QR = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj Qw = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj Q = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj R = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RInverse = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj BtransposeP = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj K = new DMatrixRMaj(3, 6);

   private final DMatrixRMaj wBd = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj desiredRotationMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj currentRotationMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj wB = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj RBerror = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj RBerrorVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wBerror = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj state = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj desiredTorque = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj feedbackTorque = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj deltaTau = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj inertiaInverse = new DMatrixRMaj(3, 3);

   private final AlgebraicVariationalFunction variationalFunction = new AlgebraicVariationalFunction();

   public VariationalLQRController()
   {
      CommonOps_DDRM.setIdentity(inertia);

      MatrixTools.setDiagonal(QR, defaultQR);
      MatrixTools.setDiagonal(Qw, defaultQw);
      MatrixTools.setDiagonal(R, defaultR);
   }

   public void setInertia(SpatialInertiaReadOnly inertia)
   {
      inertia.getMomentOfInertia().get(this.inertia);
   }

   private final RotationMatrix tempRotation = new RotationMatrix();

   public void setDesired(QuaternionReadOnly desiredRotation, Vector3DReadOnly desiredAngularVelocityInBodyFrame, Vector3DReadOnly desiredNetAngularMomentum)
   {
      assembleQ();
      UnrolledInverseFromMinor_DDRM.inv3(inertia, inertiaInverse, 1.0);

      desiredRotation.get(tempRotation);
      tempRotation.get(desiredRotationMatrix);
      desiredAngularVelocityInBodyFrame.get(wBd);
      desiredNetAngularMomentum.get(desiredTorque);

      variationalFunction.setDesired(desiredRotation, desiredAngularVelocityInBodyFrame, desiredNetAngularMomentum, Q, R, inertia, inertiaInverse);
   }

   public void compute(QuaternionReadOnly currentRotation, Vector3DReadOnly currentAngularVelocityInBodyFrame)
   {
      CommonOps_DDRM.multTransA(B, variationalFunction.getP(), BtransposeP);
      CommonOps_DDRM.mult(RInverse, BtransposeP, K);

      currentRotation.get(tempRotation);
      tempRotation.get(currentRotationMatrix);
      currentAngularVelocityInBodyFrame.get(wB);

      CommonOps_DDRM.multTransA(-1.0, currentRotationMatrix, desiredRotationMatrix, RBerror);
      CommonOps_DDRM.mult(RBerror, wBd, wBerror);
      CommonOps_DDRM.addEquals(wBerror, wB);

      CommonOps_DDRM.multAddTransA(desiredRotationMatrix, currentRotationMatrix, RBerror);
      CommonOps_DDRM.scale(0.5, RBerror);

      fromSkewSymmetric(RBerror, RBerrorVector);

      MatrixTools.setMatrixBlock(state, 0, 0, wBerror, 0, 0, 3, 1, 1.0);
      MatrixTools.setMatrixBlock(state, 3, 0, RBerrorVector, 0, 0, 3, 1, 1.0);

      CommonOps_DDRM.mult(-1.0, K, state, deltaTau);
      CommonOps_DDRM.add(deltaTau, desiredTorque, feedbackTorque);
   }

   public void getDesiredTorque(Vector3DBasics tau)
   {
      tau.set(this.feedbackTorque);
   }

   private void assembleQ()
   {
      MatrixMissingTools.setMatrixBlock(Q, 0, 0, QR, 0, 0, 3, 3, 1.0);
      MatrixMissingTools.setMatrixBlock(Q, 3, 3, Qw, 0, 0, 3, 3, 1.0);
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
}
