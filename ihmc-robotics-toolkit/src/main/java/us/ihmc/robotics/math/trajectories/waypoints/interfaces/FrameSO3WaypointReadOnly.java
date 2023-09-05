package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface FrameSO3WaypointReadOnly extends SO3WaypointReadOnly, EuclidFrameGeometry
{
   @Override
   FrameQuaternionReadOnly getOrientation();

   @Override
   FrameVector3DReadOnly getAngularVelocity();

   default double orientationDistance(FrameSO3WaypointReadOnly other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   @Override
   default String toString(String format)
   {
      return SO3WaypointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}