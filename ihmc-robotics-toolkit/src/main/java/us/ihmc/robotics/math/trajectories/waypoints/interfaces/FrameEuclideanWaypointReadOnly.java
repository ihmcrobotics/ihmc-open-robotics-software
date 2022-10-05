package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface FrameEuclideanWaypointReadOnly extends EuclideanWaypointReadOnly, EuclidFrameGeometry
{
   @Override
   FramePoint3DReadOnly getPosition();

   @Override
   FrameVector3DReadOnly getLinearVelocity();

   default double positionDistance(FrameEuclideanWaypointReadOnly other)
   {
      return getPosition().distance(other.getPosition());
   }

   @Override
   default String toString(String format)
   {
      return EuclideanWaypointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}