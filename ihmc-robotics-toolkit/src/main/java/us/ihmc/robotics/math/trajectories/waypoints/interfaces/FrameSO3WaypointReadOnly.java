package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface FrameSO3WaypointReadOnly extends SO3WaypointReadOnly, ReferenceFrameHolder
{
   @Override
   FrameQuaternionReadOnly getOrientation();

   @Override
   FrameVector3DReadOnly getAngularVelocity();

   @Override
   default FrameQuaternionBasics getOrientationCopy()
   {
      return new FrameQuaternion(getOrientation());
   }

   @Override
   default FrameVector3DBasics getAngularVelocityCopy()
   {
      return new FrameVector3D(getAngularVelocity());
   }

   default double orientationDistance(FrameSO3WaypointReadOnly other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   default void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      orientationToPack.set(getOrientation());
   }

   default void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      angularVelocityToPack.set(getAngularVelocity());
   }

   default void getOrientationIncludingFrame(FrameQuaternionBasics orientationToPack)
   {
      orientationToPack.setIncludingFrame(getOrientation());
   }

   default void getAngularVelocityIncludingFrame(FrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(getAngularVelocity());
   }

   default void get(FixedFrameSO3WaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void getIncludingFrame(FrameSO3WaypointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void get(FixedFrameQuaternionBasics orientationToPack, FixedFrameVector3DBasics angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   default void getIncludingFrame(FrameQuaternionBasics orientationToPack, FrameVector3DBasics angularVelocityToPack)
   {
      getOrientationIncludingFrame(orientationToPack);
      getAngularVelocityIncludingFrame(angularVelocityToPack);
   }

   default boolean epsilonEquals(FrameSO3WaypointReadOnly other, double epsilon)
   {
      boolean orientationMatches = getOrientation().epsilonEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().epsilonEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   default boolean geometricallyEquals(FrameSO3WaypointReadOnly other, double epsilon)
   {
      boolean orientationMatches = getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().geometricallyEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }
}