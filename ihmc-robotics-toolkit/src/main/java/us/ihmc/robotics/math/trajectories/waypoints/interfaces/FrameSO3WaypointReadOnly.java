package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface FrameSO3WaypointReadOnly extends SO3WaypointReadOnly, EuclidFrameGeometry
{
   @Override
   FrameQuaternionReadOnly getOrientation();

   @Override
   FrameVector3DReadOnly getAngularVelocity();

   @Deprecated
   @Override
   default FrameQuaternionBasics getOrientationCopy()
   {
      return new FrameQuaternion(getOrientation());
   }

   @Deprecated
   @Override
   default FrameVector3DBasics getAngularVelocityCopy()
   {
      return new FrameVector3D(getAngularVelocity());
   }

   default double orientationDistance(FrameSO3WaypointReadOnly other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   @Deprecated
   default void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   @Deprecated
   default void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(getAngularVelocity());
   }

   @Deprecated
   default void getOrientationIncludingFrame(FrameOrientation3DBasics orientationToPack)
   {
      orientationToPack.setIncludingFrame(getOrientation());
   }

   @Deprecated
   default void getAngularVelocityIncludingFrame(FrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(getAngularVelocity());
   }

   @Deprecated
   default void get(FixedFrameSO3WaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   @Deprecated
   default void getIncludingFrame(FrameSO3WaypointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
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