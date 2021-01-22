package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class ECMPTrajectoryCalculator
{
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);
   private final double mass;
   private final double gravity;
   private final double weight;

   private final MultipleWaypointsPositionTrajectoryGenerator desiredNetAngularMomentumTrajectory;

   private final FrameVector3D desiredReactionAngularMomentumRate = new FrameVector3D();
   private final FrameVector3D desiredReactionAngularMomentumAcceleration = new FrameVector3D();

   public ECMPTrajectoryCalculator(double mass, double gravity, YoRegistry parentRegistry)
   {
      this.mass = mass;
      this.gravity = Math.abs(gravity);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      desiredNetAngularMomentumTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("DesiredNetAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
      desiredNetAngularMomentumTrajectory.appendWaypoint(0.0, new FramePoint3D(), new FrameVector3D());
      desiredNetAngularMomentumTrajectory.initialize();

      parentRegistry.addChild(registry);

      weight = mass * gravity;
   }

   private final FramePoint3D ecmpPosition = new FramePoint3D();
   private final FrameVector3D ecmpVelocity = new FrameVector3D();

   // TODO in its current form, this assumes that the desired net angular momentum rate is zero.
   public void computeECMPTrajectory(List<? extends ContactStateProvider> copTrajectories, MultipleSegmentPositionTrajectoryGenerator<?> internalAngularMomentumTrajectories)
   {
      contactStateProviders.clear();
      for (int i = 0; i < copTrajectories.size(); i++)
      {
         contactStateProviders.add().set(copTrajectories.get(i));
      }

      for (int i = 0; i < copTrajectories.size(); i++)
      {
         ContactStateProvider copTrajectory = copTrajectories.get(i);
         double startTime = copTrajectory.getTimeInterval().getStartTime();
         double endTime = copTrajectory.getTimeInterval().getEndTime();

         if (startTime > internalAngularMomentumTrajectories.getEndTime() || endTime > internalAngularMomentumTrajectories.getEndTime())
            return;

         SettableContactStateProvider eCMPTrajectory = contactStateProviders.get(i);

         internalAngularMomentumTrajectories.compute(startTime);
         desiredNetAngularMomentumTrajectory.compute(startTime);

         desiredReactionAngularMomentumRate.sub(desiredNetAngularMomentumTrajectory.getVelocity(), internalAngularMomentumTrajectories.getVelocity());
         desiredReactionAngularMomentumAcceleration.sub(desiredNetAngularMomentumTrajectory.getAcceleration(), internalAngularMomentumTrajectories.getAcceleration());

         computeECMPPosition(copTrajectory.getECMPStartPosition(), desiredReactionAngularMomentumRate, ecmpPosition);
         computeECMPVelocity(copTrajectory.getECMPStartVelocity(), desiredReactionAngularMomentumAcceleration, ecmpVelocity);

         eCMPTrajectory.setStartCopPosition(ecmpPosition);
         eCMPTrajectory.setStartCopVelocity(ecmpVelocity);

         // Make sure to do this separately, as they may be the same trajectory objects
         internalAngularMomentumTrajectories.compute(endTime);
         desiredNetAngularMomentumTrajectory.compute(endTime);

         desiredReactionAngularMomentumRate.sub(desiredNetAngularMomentumTrajectory.getVelocity(), internalAngularMomentumTrajectories.getVelocity());
         desiredReactionAngularMomentumAcceleration.sub(desiredNetAngularMomentumTrajectory.getAcceleration(), internalAngularMomentumTrajectories.getAcceleration());

         computeECMPPosition(copTrajectory.getECMPEndPosition(), desiredReactionAngularMomentumRate, ecmpPosition);
         computeECMPVelocity(copTrajectory.getECMPEndVelocity(), desiredReactionAngularMomentumAcceleration, ecmpVelocity);

         eCMPTrajectory.setEndCopPosition(ecmpPosition);
         eCMPTrajectory.setEndCopVelocity(ecmpVelocity);
      }
   }

   public void computeECMPPosition(FramePoint3DReadOnly desiredCopPosition, FrameVector3DReadOnly desiredReactionAngularMomentumRate, FramePoint3DBasics ecmpPositionToPack)
   {
      ecmpPositionToPack.setX(desiredReactionAngularMomentumRate.getY());
      ecmpPositionToPack.setY(-desiredReactionAngularMomentumRate.getX());
      ecmpPositionToPack.scale(1.0 / weight);
      ecmpPositionToPack.add(desiredCopPosition);
   }

   public void computeECMPVelocity(FrameVector3DReadOnly desiredCopVelocity, FrameVector3DReadOnly desiredReactionAngularMomentumAcceleration, FrameVector3DBasics ecmpVelocityToPack)
   {
      ecmpVelocityToPack.setX(desiredReactionAngularMomentumAcceleration.getY());
      ecmpVelocityToPack.setY(-desiredReactionAngularMomentumAcceleration.getX());
      ecmpVelocityToPack.scale(1.0 / weight);
      ecmpVelocityToPack.add(desiredCopVelocity);
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }
}
