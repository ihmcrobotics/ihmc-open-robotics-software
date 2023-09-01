package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointReadOnly;

public class FrameSE3Waypoint implements FrameSE3WaypointBasics
{
   private ReferenceFrame referenceFrame;
   private final FixedFrameEuclideanWaypointBasics euclideanWaypoint = FixedFrameEuclideanWaypointBasics.newFixedFrameEuclideanWaypointBasics(this);
   private final FixedFrameSO3WaypointBasics so3Waypoint = FixedFrameSO3WaypointBasics.newFixedFrameSO3WaypointBasics(this);

   public FrameSE3Waypoint()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameSE3Waypoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameSE3Waypoint(FramePoint3DReadOnly position,
                           FrameOrientation3DReadOnly orientation,
                           FrameVector3DReadOnly linearVelocity,
                           FrameVector3DReadOnly angularVelocity)
   {
      setIncludingFrame(position, orientation, linearVelocity, angularVelocity);
   }

   public FrameSE3Waypoint(ReferenceFrame referenceFrame, SE3WaypointReadOnly waypoint)
   {
      setIncludingFrame(referenceFrame, waypoint);
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
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getEuclideanWaypoint(), getSO3Waypoint());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameSE3WaypointReadOnly)
         return equals((FrameSE3WaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
