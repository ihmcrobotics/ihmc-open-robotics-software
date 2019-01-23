package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface FrameSO3WaypointInterface extends SO3WaypointInterface, FrameChangeable
{
   @Override
   abstract FrameQuaternionReadOnly getOrientation();

   @Override
   abstract FrameVector3DReadOnly getAngularVelocity();

   default void setOrientation(FrameQuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      setOrientation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
   }

   default void setAngularVelocity(FrameVector3DReadOnly angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      setAngularVelocity(angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ());
   }

   default double orientationDistance(FrameSO3WaypointInterface other)
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

   default void set(FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setOrientation(orientation);
      setAngularVelocity(angularVelocity);
   }

   default void setIncludingFrame(FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setReferenceFrame(orientation.getReferenceFrame());
      set(orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setReferenceFrame(referenceFrame);
      set(orientation, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SO3WaypointInterface other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSO3WaypointInterface other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set(other);
   }

   default void get(FrameSO3WaypointInterface otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void getIncludingFrame(FrameSO3WaypointInterface otherToPack)
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

   default boolean epsilonEquals(FrameSO3WaypointInterface other, double epsilon)
   {
      boolean orientationMatches = getOrientation().epsilonEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().epsilonEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   default boolean geometricallyEquals(FrameSO3WaypointInterface other, double epsilon)
   {
      boolean orientationMatches = getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().geometricallyEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   default void set(FrameSO3WaypointInterface other)
   {
      setOrientation(other.getOrientation());
      setAngularVelocity(other.getAngularVelocity());
   }

   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

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
}
