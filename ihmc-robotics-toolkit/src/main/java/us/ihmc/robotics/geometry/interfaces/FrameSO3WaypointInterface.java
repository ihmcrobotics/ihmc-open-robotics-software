package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface FrameSO3WaypointInterface<T extends FrameSO3WaypointInterface<T>> extends SO3WaypointInterface<T>, FrameChangeable
{
   @Override
   public abstract FrameQuaternionReadOnly getOrientation();

   @Override
   public abstract FrameVector3DReadOnly getAngularVelocity();

   public default void setOrientation(FrameQuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      setOrientation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
   }

   public default void setAngularVelocity(FrameVector3DReadOnly angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      setAngularVelocity(angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ());
   }

   @Override
   public default double orientationDistance(T other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   public default void getOrientation(FrameQuaternionBasics orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      orientationToPack.set(getOrientation());
   }

   public default void getAngularVelocity(FrameVector3DBasics angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      angularVelocityToPack.set(getAngularVelocity());
   }

   public default void getOrientationIncludingFrame(FrameQuaternionBasics orientationToPack)
   {
      orientationToPack.setIncludingFrame(getOrientation());
   }

   public default void getAngularVelocityIncludingFrame(FrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(getAngularVelocity());
   }

   public default void set(FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setOrientation(orientation);
      setAngularVelocity(angularVelocity);
   }

   public default void setIncludingFrame(FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setReferenceFrame(orientation.getReferenceFrame());
      set(orientation, angularVelocity);
   }

   public default void setIncludingFrame(ReferenceFrame referenceFrame, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setReferenceFrame(referenceFrame);
      set(orientation, angularVelocity);
   }

   public default void get(FrameQuaternionBasics orientationToPack, FrameVector3DBasics angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   public default void getIncludingFrame(FrameQuaternionBasics orientationToPack, FrameVector3DBasics angularVelocityToPack)
   {
      getOrientationIncludingFrame(orientationToPack);
      getAngularVelocityIncludingFrame(angularVelocityToPack);
   }

   @Override
   public default boolean epsilonEquals(T other, double epsilon)
   {
      boolean orientationMatches = getOrientation().epsilonEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().epsilonEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   @Override
   public default boolean geometricallyEquals(T other, double epsilon)
   {
      boolean orientationMatches = getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
      boolean angularVelocityMatches = getAngularVelocity().geometricallyEquals(other.getAngularVelocity(), epsilon);
      return orientationMatches && angularVelocityMatches;
   }

   @Override
   public default void set(T other)
   {
      setOrientation(other.getOrientation());
      setAngularVelocity(other.getAngularVelocity());
   }

   public default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   public default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }
}
