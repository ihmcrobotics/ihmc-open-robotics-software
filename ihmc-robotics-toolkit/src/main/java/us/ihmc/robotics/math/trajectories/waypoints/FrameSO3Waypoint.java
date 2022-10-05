package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;

public class FrameSO3Waypoint implements FrameSO3WaypointBasics
{
   private ReferenceFrame referenceFrame;
   private final FixedFrameQuaternionBasics orientation = EuclidFrameFactories.newFixedFrameQuaternionBasics(this);
   private final FixedFrameVector3DBasics angularVelocity = EuclidFrameFactories.newFixedFrameVector3DBasics(this);

   public FrameSO3Waypoint()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameSO3Waypoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
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
   public FixedFrameQuaternionBasics getOrientation()
   {
      return orientation;
   }

   @Override
   public FixedFrameVector3DBasics getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getOrientation(), getAngularVelocity());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameSO3WaypointReadOnly)
         return equals((FrameSO3WaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
