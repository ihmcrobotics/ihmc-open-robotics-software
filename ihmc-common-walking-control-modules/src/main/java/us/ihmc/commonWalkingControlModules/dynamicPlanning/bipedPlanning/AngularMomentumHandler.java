package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateBasics;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingCoPTrajectoryGeneratorState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.FixedFramePolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.function.Supplier;

public class AngularMomentumHandler<T extends ContactStateBasics<T>>
{
   private final ECMPTrajectoryCalculator<T> ecmpTrajectoryCalculator;
   private final ThreePotatoAngularMomentumCalculator angularMomentumCalculator;
   private final YoFrameVector2D desiredECMPOffset;

   public AngularMomentumHandler(double totalMass,
                                 double gravity,
                                 CenterOfMassJacobian centerOfMassJacobian,
                                 SideDependentList<MovingReferenceFrame> soleFrames,
                                 Supplier<T> contactSupplier,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry graphicsListRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      DoubleProvider potatoMassFraction = new DoubleParameter("potatoMassFraction", registry, 0.05);
      desiredECMPOffset = new YoFrameVector2D("desiredCMPOffset", ReferenceFrame.getWorldFrame(), registry);

      ecmpTrajectoryCalculator = new ECMPTrajectoryCalculator<>(totalMass, gravity, contactSupplier, registry);
      angularMomentumCalculator = new ThreePotatoAngularMomentumCalculator(totalMass,
                                                                           potatoMassFraction,
                                                                           gravity,
                                                                           centerOfMassJacobian,
                                                                           soleFrames,
                                                                           registry,
                                                                           graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void setSwingFootTrajectory(RobotSide swingSide, MultipleWaypointsPoseTrajectoryGenerator swingFootTrajectory)
   {
      angularMomentumCalculator.setSwingTrajectory(swingSide, swingFootTrajectory);
   }

   public void clearSwingFootTrajectory()
   {
      angularMomentumCalculator.clearSwingTrajectory();
   }

   public void resetAngularMomentum()
   {
      angularMomentumCalculator.reset();
   }

   public void computeAngularMomentum(double time)
   {
      angularMomentumCalculator.computeAngularMomentum(time);
      FrameVector3DReadOnly desiredAngularMomentumRate = angularMomentumCalculator.useHeightScaledAngularMomentum() ?
            angularMomentumCalculator.getDesiredHeightScaledAngularMomentumRate() :
            angularMomentumCalculator.getDesiredAngularMomentumRate();
      ecmpTrajectoryCalculator.computeECMPOffset(desiredAngularMomentumRate, desiredECMPOffset);
   }

   public void solveForAngularMomentumTrajectory(CoPTrajectoryGeneratorState state,
                                                 List<? extends TimeIntervalProvider> timeIntervals,
                                                 MultipleSegmentPositionTrajectoryGenerator<?> comTrajectory)
   {
      angularMomentumCalculator.predictFootTrajectories(state);
      angularMomentumCalculator.computeAngularMomentumTrajectories(timeIntervals, comTrajectory);
   }

   public void solveForAngularMomentumTrajectory(JumpingCoPTrajectoryGeneratorState state,
                                                 List<? extends TimeIntervalProvider> timeIntervals,
                                                 MultipleSegmentPositionTrajectoryGenerator<?> comTrajectory)
   {
      angularMomentumCalculator.predictFootTrajectories(state);
      angularMomentumCalculator.computeAngularMomentumTrajectories(timeIntervals, comTrajectory);
   }

   public List<T> computeECMPTrajectory(List<T> copTrajectories)
   {
      MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> trajectory = angularMomentumCalculator.useHeightScaledAngularMomentum() ?
            angularMomentumCalculator.getHeightScaledAngularMomentumTrajectories() :
            angularMomentumCalculator.getAngularMomentumTrajectories();

      return ecmpTrajectoryCalculator.computeECMPTrajectory(copTrajectories, trajectory);
   }

   public void computeCoPPosition(FramePoint3DReadOnly desiredECMPPosition, FixedFramePoint3DBasics copPositionToPack)
   {
      copPositionToPack.set(desiredECMPPosition);
      copPositionToPack.subX(desiredECMPOffset.getX());
      copPositionToPack.subY(desiredECMPOffset.getY());
   }

   public MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> getAngularMomentumTrajectories()
   {
      return angularMomentumCalculator.getAngularMomentumTrajectories();
   }

   public MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> getHeightScaledAngularMomentumTrajectories()
   {
      return angularMomentumCalculator.getHeightScaledAngularMomentumTrajectories();
   }

   public FrameVector3DReadOnly getDesiredAngularMomentum()
   {
      return angularMomentumCalculator.getDesiredAngularMomentum();
   }

   public FrameVector3DReadOnly getHeightScaledDesiredAngularMomentum()
   {
      return angularMomentumCalculator.getDesiredHeightScaledAngularMomentum();
   }

   public FrameVector3DReadOnly getDesiredAngularMomentumRate()
   {
      return angularMomentumCalculator.getDesiredAngularMomentumRate();
   }

   public FrameVector3DReadOnly getDesiredHeightScaledAngularMomentumRate()
   {
      return angularMomentumCalculator.getDesiredHeightScaledAngularMomentumRate();
   }
}
