package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;

public class VariationalLQRController
{
   private static final double defaultQp = 20;
   private static final double defaultQpdot = 1.0;
   private static final double defaultQR = 1100;
   private static final double defaultQw = 5;
   private static final double defaultR = 1.25;

   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   private final DMatrix3x3 A41 = new DMatrix3x3();
   private final DMatrix3x3 A43 = new DMatrix3x3();
   private final DMatrix3x3 A44 = new DMatrix3x3();
   private final DMatrix3x3 B4 = new DMatrix3x3();
   private final DMatrix3x3 eye = new DMatrix3x3();
   private final DMatrixRMaj A = new DMatrixRMaj(12, 12);
   private final DMatrixRMaj B = new DMatrixRMaj(12, 3);

   private final DMatrix3x3 Qp = new DMatrix3x3();
   private final DMatrix3x3 QpDot = new DMatrix3x3();
   private final DMatrix3x3 QR = new DMatrix3x3();
   private final DMatrix3x3 Qw = new DMatrix3x3();
   private final DMatrixRMaj Q = new DMatrixRMaj(12, 12);
   private final DMatrix3x3 R = new DMatrix3x3();

   private final DMatrix3 wBd = new DMatrix3();
   private final DMatrix3x3 wBdHat = new DMatrix3x3();
   private final DMatrix3x3 RBd = new DMatrix3x3();
   private final DMatrix3 wB = new DMatrix3();

   private final DMatrix3x3 inertia = new DMatrix3x3();
   private final DMatrix3x3 inertiaInverse = new DMatrix3x3();
   private double mass = 1.0;

   public VariationalLQRController(ReferenceFrame bodyFrame)
   {
      CommonOps_DDF3.setIdentity(eye);
      CommonOps_DDF3.setIdentity(inertia);

      MatrixMissingTools.setDiagonal(Qp, defaultQp);
      MatrixMissingTools.setDiagonal(QpDot, defaultQpdot);
      MatrixMissingTools.setDiagonal(QR, defaultQR);
      MatrixMissingTools.setDiagonal(Qw, defaultQw);
      MatrixMissingTools.setDiagonal(R, defaultR);
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public void setInertia(SpatialInertiaReadOnly inertia)
   {
      inertia.getMomentOfInertia().get(this.inertia);
   }

   public void setDesired(FrameQuaternion desiredRotation, Vector3DReadOnly desiredAngularVelocity)
   {
      // FIXME check these frames
      desiredRotation.get(RBd);
      desiredAngularVelocity.get(wBd);

      // fixme set up skew version of desired value
      // todo assemble A submatrices
      // todo assemble B submatrices

      assembleAMatrix();
      assembleBMatrix();
   }

   private void assembleAMatrix()
   {
      MatrixMissingTools.setMatrixBlock(A, 0, 3, eye, 1.0);
      MatrixMissingTools.setMatrixBlock(A, 6, 6, wBdHat, -1.0);
      MatrixMissingTools.setMatrixBlock(A, 6, 9, eye, 1.0);
      MatrixMissingTools.setMatrixBlock(A, 9, 0, A41, 1.0);
      MatrixMissingTools.setMatrixBlock(A, 9, 6, A43, 1.0);
      MatrixMissingTools.setMatrixBlock(A, 9, 9, A44, 1.0);
   }

   private void assembleBMatrix()
   {
      MatrixMissingTools.setMatrixBlock(B, 3, 0, eye, 1.0 / mass);
      MatrixMissingTools.setMatrixBlock(B, 9, 0, B4, 1.0);
   }

   public void compute(FrameQuaternion current, Vector3DReadOnly currentAngularVelocity)
   {
      assembleQ();

      careSolver.setMatrices(A, B, null, Q, R);
   }

   private void assembleQ()
   {
      MatrixMissingTools.setMatrixBlock(Q, 0, 0, Qp, 1.0);
      MatrixMissingTools.setMatrixBlock(Q, 3, 3, QpDot, 1.0);
      MatrixMissingTools.setMatrixBlock(Q, 6, 6, QR, 1.0);
      MatrixMissingTools.setMatrixBlock(Q, 9, 9, Qw, 1.0);
   }
}
