package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSO3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3Waypoint;

public class FrameSO3TrajectoryPoint implements FrameSO3TrajectoryPointBasics
{
   private final FrameSO3Waypoint so3Waypoint = new FrameSO3Waypoint();
   private double time;

   public FrameSO3TrajectoryPoint()
   {
   }

   public FrameSO3TrajectoryPoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameSO3TrajectoryPoint(double time, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setIncludingFrame(time, orientation, angularVelocity);
   }

   public FrameSO3TrajectoryPoint(FrameSO3TrajectoryPointReadOnly other)
   {
      setIncludingFrame(other);
   }

   public FrameSO3TrajectoryPoint(ReferenceFrame referenceFrame, SO3TrajectoryPointReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   @Override
   public FixedFrameQuaternionBasics getOrientation()
   {
      return so3Waypoint.getOrientation();
   }

   @Override
   public FixedFrameVector3DBasics getAngularVelocity()
   {
      return so3Waypoint.getAngularVelocity();
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      so3Waypoint.setReferenceFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return so3Waypoint.getReferenceFrame();
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
      return EuclidHashCodeTools.toIntHashCode(getTime(), getOrientation(), getAngularVelocity());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameSO3TrajectoryPointReadOnly)
         return equals((FrameSO3TrajectoryPointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
