package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSO3WaypointBasics;

public class FrameSE3TrajectoryPoint implements FrameSE3TrajectoryPointBasics
{
   private final FrameSE3Waypoint se3Waypoint = new FrameSE3Waypoint();
   private double time;

   public FrameSE3TrajectoryPoint()
   {
   }

   public FrameSE3TrajectoryPoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameSE3TrajectoryPoint(double time,
                                  FramePoint3DReadOnly position,
                                  FrameOrientation3DReadOnly orientation,
                                  FrameVector3DReadOnly linearVelocity,
                                  FrameVector3DReadOnly angularVelocity)
   {
      setIncludingFrame(time, position, orientation, linearVelocity, angularVelocity);
   }

   public FrameSE3TrajectoryPoint(FrameSE3TrajectoryPointBasics other)
   {
      setIncludingFrame(other);
   }

   public FrameSE3TrajectoryPoint(ReferenceFrame referenceFrame, SE3TrajectoryPointReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   @Override
   public FixedFrameEuclideanWaypointBasics getEuclideanWaypoint()
   {
      return se3Waypoint.getEuclideanWaypoint();
   }

   @Override
   public FixedFrameSO3WaypointBasics getSO3Waypoint()
   {
      return se3Waypoint.getSO3Waypoint();
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      se3Waypoint.setReferenceFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return se3Waypoint.getReferenceFrame();
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public double getTime()
   {
      return time;
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
}
