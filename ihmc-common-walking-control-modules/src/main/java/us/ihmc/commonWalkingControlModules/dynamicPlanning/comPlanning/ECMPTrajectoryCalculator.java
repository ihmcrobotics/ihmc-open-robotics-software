package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory3DReadOnly;
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
   public void computeECMPTrajectory(List<? extends ContactStateProvider> copTrajectories, List<? extends Trajectory3DReadOnly> internalAngularMomentumTrajectories)
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
         int amStartSegmentId = getSegmentNumber(startTime, internalAngularMomentumTrajectories);
         double amStartTime = getTimeInSegment(amStartSegmentId, startTime, internalAngularMomentumTrajectories);

         double endTime = copTrajectory.getTimeInterval().getEndTime();
         int amEndSegmentId = getSegmentNumber(endTime, internalAngularMomentumTrajectories);
         double amEndTime = getTimeInSegment(amEndSegmentId, endTime, internalAngularMomentumTrajectories);

         if (amStartSegmentId < 0 || amEndSegmentId < 0)
            return;

         SettableContactStateProvider eCMPTrajectory = contactStateProviders.get(i);

         Trajectory3DReadOnly angularMomentumTrajectoryAtStart = internalAngularMomentumTrajectories.get(amStartSegmentId);
         angularMomentumTrajectoryAtStart.compute(amStartTime);

         Vector3DReadOnly initialAngularRate = angularMomentumTrajectoryAtStart.getVelocity();
         Vector3DReadOnly initialAngularAcceleration = angularMomentumTrajectoryAtStart.getAcceleration();

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
         Trajectory3DReadOnly angularMomentumTrajectoryAtEnd = internalAngularMomentumTrajectories.get(amEndSegmentId);
         angularMomentumTrajectoryAtEnd.compute(amEndTime);

         Vector3DReadOnly finalAngularRate = angularMomentumTrajectoryAtEnd.getVelocity();
         Vector3DReadOnly finalAngularAcceleration = angularMomentumTrajectoryAtEnd.getAcceleration();

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

   public int getSegmentNumber(double time, List<? extends TimeIntervalReadOnly> segments)
   {
      double startTime = 0.0;
      for (int i = 0; i < segments.size(); i++)
      {
         if (segments.get(i).intervalContains(time - startTime))
            return i;

         startTime += segments.get(i).getDuration();
      }

      return -1;
   }

   public double getTimeInSegment(int segmentNumber, double time, List<? extends TimeIntervalReadOnly> segments)
   {
      for (int i = 0; i < segmentNumber; i++)
         time -= segments.get(i).getDuration();

      return time;
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }
}
