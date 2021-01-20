package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.PositionTrajectoryGenerator;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

import java.util.List;

public class ECMPTrajectoryCalculator
{
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);
   private final double mass;
   private final double gravity;
   private final double weight;

   public ECMPTrajectoryCalculator(double mass, double gravity)
   {
      this.mass = mass;
      this.gravity = Math.abs(gravity);

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

         Vector3DReadOnly initialAngularRate = internalAngularMomentumTrajectories.getVelocity();
         Vector3DReadOnly initialAngularAcceleration = internalAngularMomentumTrajectories.getAcceleration();

         // These are scaled by a negative one because we want the total angular momentum to be zero
         ecmpPosition.setX(initialAngularRate.getY());
         ecmpPosition.setY(-initialAngularRate.getX());
         ecmpPosition.scale(-1.0 / weight);
         ecmpPosition.add(copTrajectory.getECMPStartPosition());

         ecmpVelocity.setX(initialAngularAcceleration.getY());
         ecmpVelocity.setY(-initialAngularAcceleration.getX());
         ecmpVelocity.scale(-1.0 / weight);
         ecmpVelocity.add(copTrajectory.getECMPStartVelocity());

         eCMPTrajectory.setStartCopPosition(ecmpPosition);
         eCMPTrajectory.setStartCopVelocity(ecmpVelocity);

         // Make sure to do this separately, as they may be the same trajectory objects
         internalAngularMomentumTrajectories.compute(endTime);

         Vector3DReadOnly finalAngularRate = internalAngularMomentumTrajectories.getVelocity();
         Vector3DReadOnly finalAngularAcceleration = internalAngularMomentumTrajectories.getAcceleration();

         ecmpPosition.setX(finalAngularRate.getY());
         ecmpPosition.setY(-finalAngularRate.getX());
         ecmpPosition.scale(-1.0 / weight);
         ecmpPosition.add(copTrajectory.getECMPEndPosition());

         ecmpVelocity.setX(finalAngularAcceleration.getY());
         ecmpVelocity.setY(-finalAngularAcceleration.getX());
         ecmpVelocity.scale(-1.0 / weight);
         ecmpVelocity.add(copTrajectory.getECMPEndVelocity());

         eCMPTrajectory.setEndCopPosition(ecmpPosition);
         eCMPTrajectory.setEndCopVelocity(ecmpVelocity);
      }
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }
}
