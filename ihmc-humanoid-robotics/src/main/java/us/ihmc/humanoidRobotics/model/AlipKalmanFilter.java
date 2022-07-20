package us.ihmc.humanoidRobotics.model;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

import org.ejml.simple.ops.SimpleOperations_DDRM;

/// Helpful reference: http://ejml.org/javadoc/org/ejml/simple/SimpleMatrix.html

/// Parameters for the hybrid LTI system
/// \dot{x} = Ax + w
/// y = Cx + v
/// With reset map x^+ = x^- + Bu
/// w is guassian white noise w/ covariance Q
/// v is guassian white noise w/ covariance R
public class AlipKalmanFilter
{
   private SimpleMatrix x;
   private SimpleMatrix P;

   private Boolean initialized = false;

   private final SimpleMatrix A;
   private final SimpleMatrix B;
   private final SimpleMatrix C;
   private final SimpleMatrix Q;
   private final SimpleMatrix R;

   private final SimpleMatrix Ix;

   public AlipKalmanFilter(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix Q, SimpleMatrix R, double dt)
   {
      int nx = A.getDDRM().getNumRows();
      this.Ix = SimpleMatrix.identity(nx);

      MatrixExponentialCalculator expCalculator = new MatrixExponentialCalculator(nx);
      DMatrixRMaj result = new DMatrixRMaj(nx, nx);
      expCalculator.compute(result, A.scale(dt).getDDRM());
      this.A = new SimpleMatrix(result);

      //      this.A = Ix.plus(A.scale(dt));
      this.B = new SimpleMatrix(B);
      this.C = new SimpleMatrix(C);
      this.Q = new SimpleMatrix(Q);
      this.R = new SimpleMatrix(R);

      System.out.println(this.A);
      System.out.println(this.B);
      System.out.println(this.C);
      System.out.println(this.Q);
      System.out.println(this.R);

      // Check the matrix dimension
      if (A.getDDRM().getNumRows() != A.getDDRM().getNumCols())
         LogTools.error("A has wrong dimension");
      if (A.getDDRM().getNumRows() != B.getDDRM().getNumRows())
         LogTools.error("A and B doesn't share the same number of rows");
      if (A.getDDRM().getNumCols() != C.getDDRM().getNumCols())
         LogTools.error("A and C doesn't share the same number of columns");
      if (A.getDDRM().getNumRows() != Q.getDDRM().getNumRows())
         LogTools.error("A and Q doesn't share the same number of rows");
      if (A.getDDRM().getNumCols() != Q.getDDRM().getNumCols())
         LogTools.error("A and Q doesn't share the same number of columns");
      if (R.getDDRM().getNumRows() != R.getDDRM().getNumCols())
         LogTools.error("R has wrong dimension");
      if (C.getDDRM().getNumRows() != R.getDDRM().getNumRows())
         LogTools.error("C and R doesn't share the same number of rows");
   }

   public Boolean hasBeenInitialized()
   {
      return initialized;
   }

   public void initialize(SimpleMatrix x)
   {
      initialize(x, this.Q);
   }
   
   public void initialize(SimpleMatrix x, SimpleMatrix P)
   {
      this.x = x;
      this.P = P;
      initialized = true;
   }

   public SimpleMatrix getEstimatedState()
   {
      return x;
   }

   public void Update(SimpleMatrix u, SimpleMatrix y)
   {
      Predict(u);
      Correct(y);
   }

   private void Predict(SimpleMatrix u)
   {
      x = A.mult(x.plus(B.mult(u)));
      P = Q.plus(A.mult(P.mult(A.transpose())));
   }

   private void Correct(SimpleMatrix y)
   {
      // Calculate Kalman Gain
      SimpleMatrix K = P.mult(C.transpose().mult((R.plus(C.mult(P.mult(C.transpose())))).invert()));
      // Update state estimate
      x = x.plus(K.mult(y.minus(C.mult(x))));
      // Update covariance estimate
      SimpleMatrix H = Ix.minus(K.mult(C));
      P = H.mult(P);
      //      P = H.mult(P.mult(H.transpose())).plus(K.mult(R.mult(K.transpose())));  // same result
   }
}
