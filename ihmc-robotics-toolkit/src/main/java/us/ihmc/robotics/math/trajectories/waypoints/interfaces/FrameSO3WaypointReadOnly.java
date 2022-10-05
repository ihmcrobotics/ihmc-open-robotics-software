package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
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

   default void get(FixedFrameOrientation3DBasics orientationToPack, FixedFrameVector3DBasics angularVelocityToPack)
   {
      orientationToPack.set(getOrientation());
      angularVelocityToPack.set(getAngularVelocity());
   }

   default void getIncludingFrame(FrameOrientation3DBasics orientationToPack, FrameVector3DBasics angularVelocityToPack)
   {
      orientationToPack.setIncludingFrame(getOrientation());
      angularVelocityToPack.setIncludingFrame(getAngularVelocity());
   }

   @Override
   default String toString(String format)
   {
      return SO3WaypointReadOnly.super.toString(format) + " - " + getReferenceFrame();
   }
}