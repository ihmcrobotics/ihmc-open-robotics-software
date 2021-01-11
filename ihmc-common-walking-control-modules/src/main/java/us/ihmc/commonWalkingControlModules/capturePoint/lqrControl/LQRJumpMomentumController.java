package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.LinearMomentumRateCostCommand;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

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
public class LQRJumpMomentumController
{
   private static final double discreteDt = 5e-3;
   private static final double gravityZ = -9.81;

   private final double totalMass;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoFrameVector3D yoK2 = new YoFrameVector3D("k2", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D feedbackForce = new YoFrameVector3D("feedbackForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D relativeCoMPosition = new YoFramePoint3D("relativeCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D relativeCoMVelocity = new YoFrameVector3D("relativeCoMVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D finalVRPPosition = new YoFramePoint3D("finalVRPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D referenceVRPPosition = new YoFramePoint3D("referenceVRPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D feedbackVRPPosition = new YoFramePoint3D("feedbackVRPPosition", ReferenceFrame.getWorldFrame(), registry);

   static final double defaultVrpTrackingWeight = 1e6;
   static final double defaultMomentumRateWeight = 1e-5;

   private double vrpTrackingWeight = defaultVrpTrackingWeight;
   private double momentumRateWeight = defaultMomentumRateWeight;

   private final LQRCommonValues lqrCommonValues = new LQRCommonValues();

   private final AlgebraicS1Function finalS1Function = new AlgebraicS1Function();

   private final DMatrixRMaj Nb = new DMatrixRMaj(3, 6);

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

   private final DMatrixRMaj linearMomentumRateGradient = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj linearMomentumRateHessian = new DMatrixRMaj(3, 3);

   final RecyclingArrayList<Trajectory3D> relativeVRPTrajectories = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private boolean shouldUpdateP = true;

   private final HashMap<Trajectory3D, S1Function> s1Functions = new HashMap<>();
   private final List<S1Function> reversedS1FunctionList = new ArrayList<>();
   private final List<S1Function> s1FunctionList = new ArrayList<>();
   private final List<S2Segment> reversedS2FunctionList = new ArrayList<>();
   private final List<S2Segment> s2FunctionList = new ArrayList<>();

   private final LinearMomentumRateCostCommand momentumRateCostCommand = new LinearMomentumRateCostCommand();

   public LQRJumpMomentumController(DoubleProvider omega, double totalMass)
   {
      this(omega, totalMass, null);
   }

   public LQRJumpMomentumController(DoubleProvider omega, double totalMass, YoRegistry parentRegistry)
   {
      this.totalMass = totalMass;

      computeDynamicsMatrix(omega.getValue());

      computeP();

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setVRPTrackingWeight(double vrpTrackingWeight)
   {
      this.vrpTrackingWeight = vrpTrackingWeight;

      shouldUpdateP = true;
   }

   public void setMomentumRateWeight(double momentumRateWeight)
   {
      this.momentumRateWeight = momentumRateWeight;

      shouldUpdateP = true;
   }

   private void computeDynamicsMatrix(double omega)
   {
      lqrCommonValues.computeDynamicsMatrix(omega);

      shouldUpdateP = true;
   }

   public void setVRPTrajectory(List<Trajectory3D> vrpTrajectory, List<? extends ContactStateProvider> contactStateProviders)
   {
      relativeVRPTrajectories.clear();
      this.contactStateProviders.clear();

      Trajectory3D lastTrajectory = vrpTrajectory.get(vrpTrajectory.size() - 1);
      lastTrajectory.compute(Math.min(sufficientlyLongTime, lastTrajectory.getFinalTime()));
      finalVRPPosition.set(lastTrajectory.getPosition());
      finalVRPPosition.get(finalVRPState);

      for (int i = 0; i < vrpTrajectory.size(); i++)
      {
         Trajectory3D trajectory = vrpTrajectory.get(i);
         Trajectory3D relativeTrajectory = relativeVRPTrajectories.add();

         relativeTrajectory.set(trajectory);
         relativeTrajectory.offsetTrajectoryPosition(-finalVRPPosition.getX(), -finalVRPPosition.getY(), -finalVRPPosition.getZ());

         this.contactStateProviders.add().set(contactStateProviders.get(i));
      }
   }

   void computeP()
   {
      lqrCommonValues.computeEquivalentCostValues(momentumRateWeight, vrpTrackingWeight);

      finalS1Function.set(lqrCommonValues);

      shouldUpdateP = false;
   }

   void computeS1Segments()
   {
      s1Functions.clear();
      reversedS1FunctionList.clear();
      s1FunctionList.clear();

      int numberOfSegments = relativeVRPTrajectories.size() - 1;

      if (numberOfSegments < 0)
      {
         reversedS1FunctionList.add(finalS1Function);
      }
      else
      {
         Trajectory3D nextVRPTrajectory = relativeVRPTrajectories.get(numberOfSegments);
         finalS1Function.compute(0.0, S1);
         s1Functions.put(nextVRPTrajectory, finalS1Function);
         reversedS1FunctionList.add(finalS1Function);

         boolean hasHadSwitch = false;

         for (int j = numberOfSegments - 1; j >= 0; j--)
         {
            Trajectory3D thisVRPTrajectory = relativeVRPTrajectories.get(j);

            if (contactStateProviders.get(j).getContactState().isLoadBearing())
            {
               S1Function thisS1Trajectory;
               if (hasHadSwitch)
               {
                  DifferentialS1Segment s1Segment = new DifferentialS1Segment(discreteDt);
                  s1Segment.set(lqrCommonValues, S1, thisVRPTrajectory.getDuration());
                  thisS1Trajectory = s1Segment;
               }
               else
               {
                  thisS1Trajectory = finalS1Function;
               }

               thisS1Trajectory.compute(0.0, S1);
               s1Functions.put(thisVRPTrajectory, thisS1Trajectory);
               reversedS1FunctionList.add(thisS1Trajectory);
            }
            else
            {
               hasHadSwitch = true;
               FlightS1Function s1Function = new FlightS1Function();
               s1Function.set(S1, thisVRPTrajectory.getDuration());
               s1Function.compute(0.0, S1);

               s1Functions.put(thisVRPTrajectory, s1Function);
               reversedS1FunctionList.add(s1Function);
            }
         }
      }

      for (int i = reversedS1FunctionList.size() - 1; i >= 0; i--)
         s1FunctionList.add(reversedS1FunctionList.get(i));
   }

   void computeS2Segments()
   {
      reversedS2FunctionList.clear();
      s2FunctionList.clear();

      int numberOfSegments = relativeVRPTrajectories.size();
      int numberOfEndingContactSegments = 0;
      int j = numberOfSegments - 1;
      while (j >= 0 && contactStateProviders.get(j).getContactState().isLoadBearing())
      {
         numberOfEndingContactSegments++;
         j--;
      }

      List<Trajectory3D> endingContactVRPs = new ArrayList<>();
      for (j = numberOfSegments - numberOfEndingContactSegments; j < numberOfSegments; j++)
         endingContactVRPs.add(relativeVRPTrajectories.get(j));

      s2.zero();

      AlgebraicS2Function endingS2Function = new AlgebraicS2Function();
      finalS1Function.compute(0.0, S1);
      lqrCommonValues.computeS2ConstantStateMatrices(S1);
      
      endingS2Function.set(s2, endingContactVRPs, lqrCommonValues);
      endingS2Function.compute(0, 0.0, s2);

      for (j = numberOfEndingContactSegments - 1; j >= 0; j--)
      {
         reversedS2FunctionList.add(endingS2Function.getSegment(j));
      }

      for (j = numberOfSegments - numberOfEndingContactSegments - 1; j >= 0; j--)
      {
         Trajectory3D trajectorySegment = relativeVRPTrajectories.get(j);

         if (contactStateProviders.get(j).getContactState().isLoadBearing())
         {
            DifferentialS2Segment s2Segment = new DifferentialS2Segment(discreteDt);
            s2Segment.set(s1Functions.get(trajectorySegment), trajectorySegment, lqrCommonValues, s2);
            s2Segment.compute(0.0, s2);

            reversedS2FunctionList.add(s2Segment);
         }
         else
         {
            s1Functions.get(trajectorySegment).compute(trajectorySegment.getDuration(), S1);
            FlightS2Function s2Function = new FlightS2Function(gravityZ);
            s2Function.set(S1, s2, trajectorySegment.getDuration());

            s2Function.compute(0.0, s2);
            reversedS2FunctionList.add(s2Function);
         }
      }

      for (int i = reversedS2FunctionList.size() - 1; i >= 0; i--)
         s2FunctionList.add(reversedS2FunctionList.get(i));
   }

   void computeS1AndK1(double time)
   {
      int segmentNumber = getSegmentNumber(time);
      double timeInState = computeTimeInSegment(time, segmentNumber);

      s1FunctionList.get(segmentNumber).compute(timeInState, S1);

      // Nb = N' + B' S1
      Nb.set(lqrCommonValues.getNTranspose());
      CommonOps_DDRM.multAddTransA(lqrCommonValues.getB(), S1, Nb);

      // K1 = -R1inv NB
      CommonOps_DDRM.mult(-1.0, lqrCommonValues.getR1Inverse(), Nb, K1);
   }

   void computeS2AndK2(double time)
   {
      int j = getSegmentNumber(time);
      double timeInSegment = Math.min(sufficientlyLongTime, computeTimeInSegment(time, j));

      relativeVRPTrajectories.get(j).compute(timeInSegment);
      referenceVRPPosition.set(relativeVRPTrajectories.get(j).getPosition());
      referenceVRPPosition.get(relativeDesiredVRP);
      referenceVRPPosition.add(finalVRPPosition);

      s2FunctionList.get(j).compute(timeInSegment, s2);

      CommonOps_DDRM.mult(lqrCommonValues.getR1Inverse(), lqrCommonValues.getDQ(), R1InverseDQ);
      CommonOps_DDRM.multTransB(-0.5, lqrCommonValues.getR1Inverse(), lqrCommonValues.getB(), R1InverseBTranspose);

      CommonOps_DDRM.mult(R1InverseDQ, relativeDesiredVRP, k2);
      CommonOps_DDRM.multAdd(R1InverseBTranspose, s2, k2);

      yoK2.set(k2);
   }

   public void computeControlInput(DMatrixRMaj currentState, double time)
   {
      if (shouldUpdateP)
         computeP();

      computeS1Segments();
      computeS2Segments();

      computeS1AndK1(time);
      computeS2AndK2(time);

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

      double massInverse = 1.0 / totalMass;

      CommonOps_DDRM.multTransA(-2.0 * massInverse, k2, lqrCommonValues.getR1(), linearMomentumRateGradient);
      CommonOps_DDRM.multAddTransAB(2.0 * massInverse, relativeState, Nb, linearMomentumRateGradient);
      CommonOps_DDRM.scale(2.0 * massInverse * massInverse, lqrCommonValues.getR1(), linearMomentumRateHessian);

      momentumRateCostCommand.setLinearMomentumRateGradient(linearMomentumRateGradient);
      momentumRateCostCommand.setLinearMomentumRateHessian(linearMomentumRateHessian);
   }

   public DMatrixRMaj getU()
   {
      return u;
   }

   public DMatrixRMaj getCostHessian()
   {
      return S1;
   }

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

   S1Function getS1Segment(int segment)
   {
      return s1FunctionList.get(segment);
   }

   S2Segment getS2Segment(int segment)
   {
      return s2FunctionList.get(segment);
   }

   public FramePoint3DReadOnly getFeedbackVRPPosition()
   {
      return feedbackVRPPosition;
   }

   public LinearMomentumRateCostCommand getMomentumRateCostCommand()
   {
      return momentumRateCostCommand;
   }
}
