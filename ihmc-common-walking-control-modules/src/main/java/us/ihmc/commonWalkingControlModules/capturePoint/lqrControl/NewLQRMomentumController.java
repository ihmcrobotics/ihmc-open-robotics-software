package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import com.jme3.audio.openal.AL;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * This LQR controller tracks the CoM dynamics of the robot, using a VRP output.
 * This is just a 3D extension of http://groups.csail.mit.edu/robotics-center/public_papers/Tedrake15.pdf
 *
 * The equations of motion are as follows:
 *
 * <p> x = [x<sub>com</sub>; xDot<sub>com</sub>]</p>
 * <p> u = [xDdot<sub>com</sub>] </p>
 * <p> y = [x<sub>vrp</sub>] </p>
 *
 * <p> A = [0 I; 0 0]</p>
 * <p> B = [0; I]</p>
 * <p> C = </p>
 */
public class NewLQRMomentumController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoFrameVector3D yoK2 = new YoFrameVector3D("k2", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D feedbackForce = new YoFrameVector3D("feedbackForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D relativeCoMPosition = new YoFramePoint3D("relativeCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D relativeCoMVelocity = new YoFrameVector3D("relativeCoMVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D finalVRPPosition = new YoFramePoint3D("finalVRPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D referenceVRPPosition = new YoFramePoint3D("referenceVRPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D feedbackVRPPosition = new YoFramePoint3D("feedbackVRPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble omega = new YoDouble("omega", registry);

   static final double defaultVrpTrackingWeight = 1e2;
   static final double defaultMomentumRateWeight = 1e-4;

   private double vrpTrackingWeight = defaultVrpTrackingWeight;
   private double momentumRateWeight = defaultMomentumRateWeight;

   private final AlgebraicS1Function s1Function = new AlgebraicS1Function();
   private final AlgebraicS2Function s2Function = new AlgebraicS2Function();

   private final DMatrixRMaj Q = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 3);
   private final DMatrixRMaj BTranspose = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj C = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj D = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj A2 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj A2Inverse = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B2 = new DMatrixRMaj(6, 3);

   private final DMatrixRMaj Q1 = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj R1 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R1Inverse = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj N = new DMatrixRMaj(6, 3);
   private final DMatrixRMaj NTranspose = new DMatrixRMaj(3, 6);

   private final DMatrixRMaj NB = new DMatrixRMaj(3, 6);

   private final DMatrixRMaj S1 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj s2 = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj K1 = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj k2 = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj u = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);
   private final DMatrixRMaj DQ = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R1InverseDQ = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R1InverseBTranspose = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj finalVRPState = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj relativeState = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj relativeDesiredVRP = new DMatrixRMaj(3, 1);

   private final RecyclingArrayList<Trajectory3D> relativeVRPTrajectories = new RecyclingArrayList<>(() -> new Trajectory3D(4));

   private boolean shouldUpdateS1 = true;

   public NewLQRMomentumController(DoubleProvider omega)
   {
      this(omega, null);
   }

   public NewLQRMomentumController(DoubleProvider omega, YoRegistry parentRegistry)
   {
      this(omega.getValue(), parentRegistry);
   }

   public NewLQRMomentumController(double omega, YoRegistry parentRegistry)
   {
      this.omega.set(omega);
      computeDynamicsMatrix(this.omega.getDoubleValue());
                            
      this.omega.addListener(v -> {
         computeDynamicsMatrix(this.omega.getDoubleValue());
      });

      computeS1();
      

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setVRPTrackingWeight(double vrpTrackingWeight)
   {
      this.vrpTrackingWeight = vrpTrackingWeight;

      shouldUpdateS1 = true;
   }

   public void setMomentumRateWeight(double momentumRateWeight)
   {
      this.momentumRateWeight = momentumRateWeight;

      shouldUpdateS1 = true;
   }

   public void computeDynamicsMatrix(double omega)
   {
      MatrixTools.setMatrixBlock(A, 0, 3, CommonOps_DDRM.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(B, 3, 0, CommonOps_DDRM.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(C, 0, 0, CommonOps_DDRM.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(D, 0, 0, CommonOps_DDRM.identity(3, 3), 0, 0, 3, 3, -1.0 / MathTools.square(omega));

      CommonOps_DDRM.transpose(B, BTranspose);

      shouldUpdateS1 = true;
   }

   /**
    * Sets the desired VRP trajectory for the LQR control to track.
    */
   public void setVRPTrajectory(List<Trajectory3D> vrpTrajectory)
   {
      relativeVRPTrajectories.clear();

      Trajectory3D lastTrajectory = vrpTrajectory.get(vrpTrajectory.size() - 1);
      lastTrajectory.compute(lastTrajectory.getFinalTime());
      finalVRPPosition.set(lastTrajectory.getPosition());
      finalVRPPosition.get(finalVRPState);

      for (int i = 0; i < vrpTrajectory.size(); i++)
      {
         Trajectory3D trajectory = vrpTrajectory.get(i);
         Trajectory3D relativeTrajectory = relativeVRPTrajectories.add();

         relativeTrajectory.set(trajectory);
         relativeTrajectory.offsetTrajectoryPosition(-finalVRPState.get(0, 0), -finalVRPState.get(1, 0), -finalVRPState.get(2, 0));
      }
   }

   void computeS1()
   {
      MatrixTools.setDiagonal(Q, vrpTrackingWeight);
      MatrixTools.setDiagonal(R, momentumRateWeight);

      NativeCommonOps.multQuad(C, Q, Q1);
      NativeCommonOps.multQuad(D, Q, R1);
      CommonOps_DDRM.addEquals(R1, R);

      NativeCommonOps.invert(R1, R1Inverse);

      CommonOps_DDRM.mult(D, Q, DQ);

      tempMatrix.reshape(3, 3);
      CommonOps_DDRM.mult(Q, D, tempMatrix);
      CommonOps_DDRM.multTransA(C, tempMatrix, N);
      CommonOps_DDRM.transpose(N, NTranspose);

      /*
        A' S1 + S1 A - Nb' R1inv Nb + Q1 = S1dot = 0
        or
        A1 S1 + S1 A - S1' B R1inv B' S1 - N R1inv N' + Q1
        If the standard CARE is formed as
        A' P + P A - P B R^-1 B' P + Q = 0
        then we can rewrite this as
                     S1 = P
         A - B R1inv N' = A
                      B = B
        Q1 - N R1inv N' = Q
      */



      s1Function.set(Q1, R1, NTranspose, A, B);
      s1Function.compute(0.0, S1);

      // all the below stuff is constant
      // Nb = N' + B' S1
      CommonOps_DDRM.transpose(N, NB);
      CommonOps_DDRM.multAddTransA(B, S1, NB);

      // K1 = -R1inv NB
      CommonOps_DDRM.mult(-1.0, R1Inverse, NB, K1);

      // A2 = Nb' R1inv B' - A'
      tempMatrix.reshape(3, 6);
      MatrixTools.scaleTranspose(-1.0, A, A2);
      CommonOps_DDRM.multTransB(R1Inverse, B, tempMatrix);
      CommonOps_DDRM.multAddTransA(NB, tempMatrix, A2);

      // B2 = 2 (C' - Nb' R1inv D) Q
      CommonOps_DDRM.mult(R1Inverse, DQ, R1InverseDQ);
      CommonOps_DDRM.multTransA(-2.0, NB, R1InverseDQ, B2);
      CommonOps_DDRM.multAddTransA(2.0, C, Q, B2);

      NativeCommonOps.invert(A2, A2Inverse);
      CommonOps_DDRM.mult(-1.0, A2Inverse, B2, A2InverseB2);

      CommonOps_DDRM.multTransB(-0.5, R1Inverse, B, R1InverseBTranspose);

      shouldUpdateS1 = false;
   }

   private final DMatrixRMaj zeroVector = new DMatrixRMaj(6, 1);
   void computeS2Parameters()
   {
      s2Function.set(zeroVector, relativeVRPTrajectories, A2, B2);
   }

   void computeS2(double time)
   {
      int j = getSegmentNumber(time);
      double timeInSegment = computeTimeInSegment(time, j);

      relativeVRPTrajectories.get(j).compute(timeInSegment);
      referenceVRPPosition.set(relativeVRPTrajectories.get(j).getPosition());
      referenceVRPPosition.get(relativeDesiredVRP);
      referenceVRPPosition.add(finalVRPPosition);

      // Fix this to account for the change in segment
      s2Function.compute(j, timeInSegment, s2);

      CommonOps_DDRM.mult(R1InverseDQ, relativeDesiredVRP, k2);
      CommonOps_DDRM.multAdd(R1InverseBTranspose, s2, k2);

      yoK2.set(k2);
   }

   public AlgebraicS2Segment getS2Segment(int segmentNumber)
   {
      return s2Function.getSegment(segmentNumber);
   }

   /**
    * The current state is a stacked vector of the current CoM state and current CoM velocity, such that entries (0:2) are occupied by the position,
    * and entries (3:5) are occupied by the velocity
    */
   public void computeControlInput(DMatrixRMaj currentState, double time)
   {
      if (shouldUpdateS1)
         computeS1();

      computeS2Parameters();
      computeS2(time);

      relativeState.set(currentState);
      for (int i = 0; i < 3; i++)
         relativeState.add(i, 0, -finalVRPState.get(i));

      relativeCoMPosition.set(relativeState);
      relativeCoMVelocity.set(3, relativeState);

      // u = K1 relativeX + k2
      CommonOps_DDRM.mult(K1, relativeState, u);
      feedbackForce.set(u);

      CommonOps_DDRM.addEquals(u, k2);

      CommonOps_DDRM.mult(C, relativeState, relativeDesiredVRP);
      CommonOps_DDRM.multAdd(D, u, relativeDesiredVRP);

      feedbackVRPPosition.set(relativeDesiredVRP);
      feedbackVRPPosition.add(finalVRPPosition);
   }

   /**
    * Returns the unconstrained optimal control input, calculated by {@link #computeControlInput(DMatrixRMaj, double)}.
    */
   public DMatrixRMaj getU()
   {
      return u;
   }

   /**
    * If the optimal cost to go is assumed be x<sup>T</sup>(t) S<sub>1</sub>(t) x(t) + s<sub>2</sub><sup>T</sup>(t) x(t) + s<sub>3</sub>(t),
    * this method returns S<sub>1</sub>(t), where t is defined by {@link #computeControlInput(DMatrixRMaj, double)}.
    *
    * <p>
    *    Note: this is ONLY the optimal solution if u(t) is completely defined by x(t), meaning u(t) is completely unconstrained.
    * </p>
    */
   public DMatrixRMaj getCostHessian()
   {
      return S1;
   }

   /**
    * If the optimal cost to go is assumed be x<sup>T</sup>(t) S<sub>1</sub>(t) x(t) + s<sub>2</sub><sup>T</sup>(t) x(t) + s<sub>3</sub>(t),
    * this method returns s<sub>21</sub>(t), where t is defined by {@link #computeControlInput(DMatrixRMaj, double)}.
    *
    * <p>
    *    Note: this is ONLY the optimal solution if u(t) is completely defined by x(t), meaning u(t) is completely unconstrained.
    * </p>
    */
   public DMatrixRMaj getCostJacobian()
   {
      return s2;
   }

   private int getSegmentNumber(double time)
   {
      double timeToStart = 0.0;
      for (int i = 0; i < relativeVRPTrajectories.size(); i++)
      {
         double segmentDuration = relativeVRPTrajectories.get(i).getDuration();
         if (time - timeToStart <= segmentDuration)
            return i;
         timeToStart += segmentDuration;
      }

      return -1;
   }

   private double computeTimeInSegment(double time, int segment)
   {
      double timeOffset = 0.0;
      for (int i = 0; i < segment; i++)
         timeOffset += relativeVRPTrajectories.get(i).getDuration();
      return time - timeOffset;
   }
   
   public void setOmega(double omega)
   {
      this.omega.set(omega);
   }

   DMatrixRMaj getA()
   {
      return A;
   }

   DMatrixRMaj getB()
   {
      return B;
   }

   DMatrixRMaj getC()
   {
      return C;
   }

   DMatrixRMaj getD()
   {
      return D;
   }

   DMatrixRMaj getQ()
   {
      return Q;
   }

   DMatrixRMaj getR()
   {
      return R;
   }

   DMatrixRMaj getK1()
   {
      return K1;
   }

   DMatrixRMaj getK2()
   {
      return k2;
   }
}
