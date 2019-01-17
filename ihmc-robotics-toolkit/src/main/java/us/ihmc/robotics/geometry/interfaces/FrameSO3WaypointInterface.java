package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface FrameSO3WaypointInterface<T extends FrameSO3WaypointInterface<T>> extends SO3WaypointInterface<T>, FrameChangeable
{
   @Override
   public abstract FrameQuaternionReadOnly getOrientation();

   @Override
   public abstract FrameVector3DReadOnly getAngularVelocity();

   @Override
   public default void setOrientation(QuaternionReadOnly orientation)
   {
      if (orientation instanceof ReferenceFrameHolder)
      {
         checkReferenceFrameMatch((ReferenceFrameHolder) orientation);
      }
      setOrientation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
   }

   @Override
   public default void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      if (angularVelocity instanceof ReferenceFrameHolder)
      {
         checkReferenceFrameMatch((ReferenceFrameHolder) angularVelocity);
      }
      setAngularVelocity(angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ());
   }

   @Override
   public default double orientationDistance(T other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   @Override
   public default void getOrientation(QuaternionBasics orientationToPack)
   {
      if (orientationToPack instanceof ReferenceFrameHolder)
      {
         checkReferenceFrameMatch((ReferenceFrameHolder) orientationToPack);
      }
      orientationToPack.set(getOrientation());
   }

   @Override
   public default void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      if (angularVelocityToPack instanceof ReferenceFrameHolder)
      {
         checkReferenceFrameMatch((ReferenceFrameHolder) angularVelocityToPack);
      }
      angularVelocityToPack.set(getAngularVelocity());
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
