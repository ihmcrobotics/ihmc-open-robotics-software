package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class AngularMomentumHandler
{
   private final ECMPTrajectoryCalculator ecmpTrajectoryCalculator;
   private final ThreePotatoAngularMomentumCalculator angularMomentumCalculator;
   private final YoFrameVector2D desiredECMPOffset;

   public AngularMomentumHandler(double totalMass,
                                 double gravity,
                                 CenterOfMassJacobian centerOfMassJacobian,
                                 SideDependentList<MovingReferenceFrame> soleFrames,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry graphicsListRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      DoubleProvider potatoMassFraction = new DoubleParameter("potatoMassFraction", registry, 0.05);
      desiredECMPOffset = new YoFrameVector2D("desiredCMPOffset", ReferenceFrame.getWorldFrame(), registry);

      ecmpTrajectoryCalculator = new ECMPTrajectoryCalculator(totalMass, gravity, registry);
      angularMomentumCalculator = new ThreePotatoAngularMomentumCalculator(totalMass,
                                                                           potatoMassFraction,
                                                                           gravity,
                                                                           centerOfMassJacobian,
                                                                           soleFrames,
                                                                           registry,
                                                                           graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void computeAngularMomentum(double time)
   {
      angularMomentumCalculator.computeAngularMomentum(time);
      ecmpTrajectoryCalculator.computeECMPOffset(angularMomentumCalculator.getDesiredAngularMomentumRate(), desiredECMPOffset);
   }

   public void solveForAngularMomentumTrajectory(CoPTrajectoryGeneratorState state,
                                                 List<? extends TimeIntervalProvider> timeIntervals,
                                                 MultipleSegmentPositionTrajectoryGenerator<?> comTrajectory,
                                                 MultipleWaypointsPoseTrajectoryGenerator swingTrajectory)
   {
      angularMomentumCalculator.setSwingTrajectory(swingTrajectory);
      angularMomentumCalculator.predictFootTrajectories(state);
      angularMomentumCalculator.computeAngularMomentumTrajectories(timeIntervals, comTrajectory);
   }

   public List<? extends ContactStateProvider> computeECMPTrajectory(List<? extends ContactStateProvider> copTrajectories)
   {
      return ecmpTrajectoryCalculator.computeECMPTrajectory(copTrajectories, angularMomentumCalculator.getAngularMomentumTrajectories());
   }

   public void computeCoPPosition(FramePoint3DReadOnly desiredECMPPosition, FixedFramePoint3DBasics copPositionToPack)
   {
      copPositionToPack.set(desiredECMPPosition);
      copPositionToPack.subX(desiredECMPOffset.getX());
      copPositionToPack.subY(desiredECMPOffset.getY());
   }
}
