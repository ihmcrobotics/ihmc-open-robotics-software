package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;

public interface FixedFrameSE3TrajectoryPointBasics extends FrameSE3TrajectoryPointReadOnly, SE3TrajectoryPointBasics, FixedFrameSE3WaypointBasics,
      FixedFrameEuclideanTrajectoryPointBasics, FixedFrameSO3TrajectoryPointBasics
{
   default void set(double time,
                    FramePoint3DReadOnly position,
                    FrameOrientation3DReadOnly orientation,
                    FrameVector3DReadOnly linearVelocity,
                    FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(position, orientation, linearVelocity, angularVelocity);
   }

   default void set(double time, FrameSE3WaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(double time, FrameEuclideanWaypointReadOnly euclideanWaypoint, FrameSO3WaypointReadOnly so3Waypoint)
   {
      setTime(time);
      set(euclideanWaypoint);
      set(so3Waypoint);
   }

   default void set(FrameSE3TrajectoryPointReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, SE3TrajectoryPointReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Creates a new frame trajectory point which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame trajectory point.
    * @return the new linked frame trajectory point.
    */
   public static FixedFrameSE3TrajectoryPointBasics newFixedFrameSE3TrajectoryPointBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameSE3TrajectoryPointBasics(referenceFrameHolder, new SE3TrajectoryPoint());
   }

   /**
    * Creates a new frame trajectory point which is linked to the given frameless trajectory point and
    * {@code referenceFrameHolder}.
    * <p>
    * This can essentially be used to wrap a frameless trajectory point into a frame trajectory point.
    * Modifications on the returned object will affect the frameless trajectory point.
    * </p>
    *
    * @param referenceFrameHolder    the reference frame holder to link to the new frame trajectory
    *                                point.
    * @param originalTrajectoryPoint the trajectory point to link to the new frame trajectory point.
    *                                Modifications on either the {@code originalTrajectoryPoint} or the
    *                                new frame trajectory point will be propagated to the other.
    * @return the new linked frame trajectory point.
    */
   public static FixedFrameSE3TrajectoryPointBasics newLinkedFixedFrameSE3TrajectoryPointBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                                SE3TrajectoryPointBasics originalTrajectoryPoint)
   {
      return new FixedFrameSE3TrajectoryPointBasics()
      {
         private final FixedFrameEuclideanWaypointBasics euclideanWaypoint = FixedFrameEuclideanWaypointBasics.newFixedFrameEuclideanWaypointBasics(this);
         private final FixedFrameSO3WaypointBasics so3Waypoint = FixedFrameSO3WaypointBasics.newFixedFrameSO3WaypointBasics(this);

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public FixedFrameEuclideanWaypointBasics getEuclideanWaypoint()
         {
            return euclideanWaypoint;
         }

         @Override
         public FixedFrameSO3WaypointBasics getSO3Waypoint()
         {
            return so3Waypoint;
         }

         @Override
         public void setTime(double time)
         {
            originalTrajectoryPoint.setTime(time);
         }

         @Override
         public double getTime()
         {
            return originalTrajectoryPoint.getTime();
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(getTime(), getEuclideanWaypoint(), getSO3Waypoint());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameSE3TrajectoryPointReadOnly)
               return equals((FrameSE3TrajectoryPointReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }
      };
   }
}