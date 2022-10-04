package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface FixedFrameEuclideanWaypointBasics extends FrameEuclideanWaypointReadOnly, EuclideanWaypointBasics
{
   default void setPosition(FramePoint3DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   default void setLinearVelocity(FrameVector3DReadOnly linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      setLinearVelocity(linearVelocity.getX(), linearVelocity.getY(), linearVelocity.getZ());
   }

   default double positionDistance(FrameEuclideanWaypointReadOnly other)
   {
      return getPosition().distance(other.getPosition());
   }

   default void set(FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setPosition(position);
      setLinearVelocity(linearVelocity);
   }

   default void set(FrameEuclideanWaypointReadOnly other)
   {
      setPosition(other.getPosition());
      setLinearVelocity(other.getLinearVelocity());
   }

}