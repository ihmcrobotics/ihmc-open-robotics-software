package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner.sufficientlyLong;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLarge;

public class ECMPTrajectoryCalculator
{
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);
   private final double mass;
   private final double gravity;
   private final double weight;

   public ECMPTrajectoryCalculator(double mass, double gravity, YoRegistry parentRegistry)
   {
      this.mass = mass;
      this.gravity = Math.abs(gravity);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      parentRegistry.addChild(registry);

      weight = mass * gravity;
   }

   private final FramePoint3D ecmpPosition = new FramePoint3D();
   private final FrameVector3D ecmpVelocity = new FrameVector3D();

   public void computeECMPTrajectory(List<? extends ContactStateProvider> copTrajectories, MultipleSegmentPositionTrajectoryGenerator<?> desiredAngularMomentumTrajectories)
   {
      contactStateProviders.clear();
      for (int i = 0; i < copTrajectories.size(); i++)
      {
         contactStateProviders.add().set(copTrajectories.get(i));
      }

      for (int i = 0; i < copTrajectories.size(); i++)
      {
         ContactStateProvider copTrajectory = copTrajectories.get(i);
         double startTime = Math.min(copTrajectory.getTimeInterval().getStartTime(), sufficientlyLong);
         double endTime = Math.min(copTrajectory.getTimeInterval().getEndTime(), sufficientlyLong);

         if (startTime > desiredAngularMomentumTrajectories.getEndTime() || endTime > desiredAngularMomentumTrajectories.getEndTime())
            return;

         SettableContactStateProvider eCMPTrajectory = contactStateProviders.get(i);

         desiredAngularMomentumTrajectories.compute(startTime);

         computeECMPPosition(copTrajectory.getECMPStartPosition(), desiredAngularMomentumTrajectories.getVelocity(), ecmpPosition);
         computeECMPVelocity(copTrajectory.getECMPStartVelocity(), desiredAngularMomentumTrajectories.getAcceleration(), ecmpVelocity);

         eCMPTrajectory.setStartCopPosition(ecmpPosition);
         eCMPTrajectory.setStartCopVelocity(ecmpVelocity);

         desiredAngularMomentumTrajectories.compute(endTime);

         computeECMPPosition(copTrajectory.getECMPEndPosition(), desiredAngularMomentumTrajectories.getVelocity(), ecmpPosition);
         computeECMPVelocity(copTrajectory.getECMPEndVelocity(), desiredAngularMomentumTrajectories.getAcceleration(), ecmpVelocity);

         eCMPTrajectory.setEndCopPosition(ecmpPosition);
         eCMPTrajectory.setEndCopVelocity(ecmpVelocity);
      }
   }

   public void computeCoPPosition(FramePoint3DReadOnly desiredECMPPosition, FrameVector3DReadOnly desiredAngularMomentumRate, FixedFramePoint3DBasics copPositionToPack)
   {
      copPositionToPack.setX(desiredAngularMomentumRate.getY());
      copPositionToPack.setY(-desiredAngularMomentumRate.getX());
      copPositionToPack.scale(-1.0 / weight);
      copPositionToPack.add(desiredECMPPosition);
   }

   public void computeECMPPosition(FramePoint3DReadOnly desiredCopPosition, FrameVector3DReadOnly desiredAngularMomentumRate, FixedFramePoint3DBasics ecmpPositionToPack)
   {
      ecmpPositionToPack.setX(desiredAngularMomentumRate.getY());
      ecmpPositionToPack.setY(-desiredAngularMomentumRate.getX());
      ecmpPositionToPack.scale(1.0 / weight);
      ecmpPositionToPack.add(desiredCopPosition);
   }

   public void computeECMPVelocity(FrameVector3DReadOnly desiredCopVelocity, FrameVector3DReadOnly desiredAngularMomentumAcceleration, FixedFrameVector3DBasics ecmpVelocityToPack)
   {
      ecmpVelocityToPack.setX(desiredAngularMomentumAcceleration.getY());
      ecmpVelocityToPack.setY(-desiredAngularMomentumAcceleration.getX());
      ecmpVelocityToPack.scale(1.0 / weight);
      ecmpVelocityToPack.add(desiredCopVelocity);
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }
}
