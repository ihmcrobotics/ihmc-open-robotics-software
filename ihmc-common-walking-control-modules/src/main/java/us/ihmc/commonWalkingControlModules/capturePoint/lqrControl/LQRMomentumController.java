package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
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
public class LQRMomentumController
{
   private final static double sufficientlyLarge = 1e3;

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

   private final LQRCommonValues lqrCommonValues = new LQRCommonValues();

   private final DMatrixRMaj S1 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj s2 = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj K1 = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj k2 = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj u = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj R1InverseDQ = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R1InverseBTranspose = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj finalVRPState = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj relativeState = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj relativeDesiredVRP = new DMatrixRMaj(3, 1);

   private final RecyclingArrayList<Trajectory3D> relativeVRPTrajectories = new RecyclingArrayList<>(() -> new Trajectory3D(4));

   private boolean shouldUpdateS1 = true;

   public LQRMomentumController(DoubleProvider omega)
   {
      this(omega, null);
   }

   public LQRMomentumController(DoubleProvider omega, YoRegistry parentRegistry)
   {
      this(omega.getValue(), parentRegistry);
   }

   public LQRMomentumController(double omega, YoRegistry parentRegistry)
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
      lqrCommonValues.computeDynamicsMatrix(omega);

      shouldUpdateS1 = true;
   }

   /**
    * Sets the desired VRP trajectory for the LQR control to track.
    */
   public void setVRPTrajectory(List<Trajectory3D> vrpTrajectory)
   {
      relativeVRPTrajectories.clear();

      Trajectory3D lastTrajectory = vrpTrajectory.get(vrpTrajectory.size() - 1);
      lastTrajectory.compute(Math.min(sufficientlyLarge, lastTrajectory.getFinalTime()));
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
      lqrCommonValues.computeEquivalentCostValues(momentumRateWeight, vrpTrackingWeight);


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

      s1Function.set(lqrCommonValues);
      s1Function.compute(0.0, S1);

      // all the below stuff is constant

      lqrCommonValues.computeS2ConstantStateMatrices(S1);


      // K1 = -R1inv NB
      CommonOps_DDRM.mult(-1.0, lqrCommonValues.getR1Inverse(), lqrCommonValues.getNb(), K1);

      CommonOps_DDRM.mult(lqrCommonValues.getR1Inverse(), lqrCommonValues.getDQ(), R1InverseDQ);
      CommonOps_DDRM.multTransB(-0.5, lqrCommonValues.getR1Inverse(), lqrCommonValues.getB(), R1InverseBTranspose);

      shouldUpdateS1 = false;
   }

   private final DMatrixRMaj zeroVector = new DMatrixRMaj(6, 1);
   void computeS2Parameters()
   {
      s2Function.set(zeroVector, relativeVRPTrajectories, lqrCommonValues);
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

      CommonOps_DDRM.mult(lqrCommonValues.getC(), relativeState, relativeDesiredVRP);
      CommonOps_DDRM.multAdd(lqrCommonValues.getD(), u, relativeDesiredVRP);

      feedbackVRPPosition.set(relativeDesiredVRP);
      feedbackVRPPosition.add(finalVRPPosition);
   }

   public FramePoint3DReadOnly getFeedbackVRPPosition()
   {
      return feedbackVRPPosition;
   }

   public FramePoint3DReadOnly getReferenceVRPPosition()
   {
      return referenceVRPPosition;
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

   public DMatrixRMaj getControlHessian()
   {
      return lqrCommonValues.getR1();
   }

   public DMatrixRMaj getControlJacobian()
   {
      return k2;
   }

   public DMatrixRMaj getStateDependentControlJacobian()
   {
      return lqrCommonValues.getNb();
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
      return lqrCommonValues.getA();
   }

   DMatrixRMaj getB()
   {
      return lqrCommonValues.getB();
   }

   DMatrixRMaj getC()
   {
      return lqrCommonValues.getC();
   }

   DMatrixRMaj getD()
   {
      return lqrCommonValues.getD();
   }

   DMatrixRMaj getQ()
   {
      return lqrCommonValues.getQ();
   }

   DMatrixRMaj getR()
   {
      return lqrCommonValues.getR();
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
