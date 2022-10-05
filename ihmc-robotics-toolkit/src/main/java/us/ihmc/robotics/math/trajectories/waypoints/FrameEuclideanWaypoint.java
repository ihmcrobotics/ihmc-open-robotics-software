package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;

public class FrameEuclideanWaypoint implements FrameEuclideanWaypointBasics
{
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint3DBasics position = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   private final FixedFrameVector3DBasics linearVelocity = EuclidFrameFactories.newFixedFrameVector3DBasics(this);

   public FrameEuclideanWaypoint()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameEuclideanWaypoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameEuclideanWaypoint(FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setIncludingFrame(position, linearVelocity);
   }

   public FrameEuclideanWaypoint(ReferenceFrame referenceFrame, EuclideanWaypointReadOnly waypoint)
   {
      setIncludingFrame(referenceFrame, waypoint);
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
   public FixedFramePoint3DBasics getPosition()
   {
      return position;
   }

   @Override
   public FixedFrameVector3DBasics getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getPosition(), getLinearVelocity());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameEuclideanWaypointReadOnly)
         return equals((FrameEuclideanWaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
