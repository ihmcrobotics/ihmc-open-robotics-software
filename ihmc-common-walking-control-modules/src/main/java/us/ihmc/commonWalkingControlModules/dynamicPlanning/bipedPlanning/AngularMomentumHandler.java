package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class AngularMomentumHandler
{
   private final ECMPTrajectoryCalculator ecmpTrajectoryCalculator;
   private final ThreePotatoAngularMomentumCalculator angularMomentumCalculator;

   public AngularMomentumHandler(double mass, double potatoMass, double gravity,
                                 CenterOfMassJacobian centerOfMassJacobian,
                                 SideDependentList<MovingReferenceFrame> soleFrames,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry graphicsListRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      ecmpTrajectoryCalculator = new ECMPTrajectoryCalculator(mass, gravity, registry);
      angularMomentumCalculator = new ThreePotatoAngularMomentumCalculator(gravity, potatoMass, centerOfMassJacobian, soleFrames, registry, graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void computeAngularMomentum(double time)
   {
      angularMomentumCalculator.computeAngularMomentum(time);
   }

   public void computeAngularMomentumTrajectories(CoPTrajectoryGeneratorState state,
                                                  List<? extends TimeIntervalProvider> timeIntervals,
                                                  MultipleSegmentPositionTrajectoryGenerator<?> comTrajectories,
                                                  MultipleWaypointsPoseTrajectoryGenerator swingTrajectory)
   {
      angularMomentumCalculator.setSwingTrajectory(swingTrajectory);
      angularMomentumCalculator.predictFootTrajectories(state);
      angularMomentumCalculator.computeAngularMomentumTrajectories(timeIntervals, comTrajectories);
   }

   public List<? extends ContactStateProvider> computeECMPTrajectory(List<? extends ContactStateProvider> copTrajectories)
   {
      return ecmpTrajectoryCalculator.computeECMPTrajectory(copTrajectories, angularMomentumCalculator.getHeightScaledAngularMomentumTrajectories());
   }

   public void computeCoPPosition(FramePoint3DReadOnly desiredECMPPosition, FixedFramePoint3DBasics copPositionToPack)
   {
      ecmpTrajectoryCalculator.computeCoPPosition(desiredECMPPosition, angularMomentumCalculator.getDesiredAngularMomentumRate(), copPositionToPack);
   }

}
