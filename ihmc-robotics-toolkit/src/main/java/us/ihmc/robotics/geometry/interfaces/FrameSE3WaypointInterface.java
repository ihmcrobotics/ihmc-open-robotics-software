package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface FrameSE3WaypointInterface<T extends FrameSE3WaypointInterface<T>> extends FrameEuclideanWaypointInterface<T>, FrameSO3WaypointInterface<T>
{
   public default void set(FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                           FrameVector3DReadOnly angularVelocity)
   {
      setPosition(position);
      setOrientation(orientation);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
   }

   public default void setIncludingFrame(FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                                         FrameVector3DReadOnly angularVelocity)
   {
      setReferenceFrame(position.getReferenceFrame());
      setPosition(position);
      setOrientation(orientation);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
   }

   public default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, QuaternionReadOnly orientation,
                                         Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setReferenceFrame(referenceFrame);
      setPosition(position);
      setOrientation(orientation);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
   }

   public default void get(FramePoint3DBasics positionToPack, FrameQuaternionBasics orientationToPack, FrameVector3DBasics linearVelocityToPack,
                           FrameVector3DBasics angularVelocityToPack)
   {
      getPosition(positionToPack);
      getOrientation(orientationToPack);
      getLinearVelocity(linearVelocityToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   public default void getIncludingFrame(FramePoint3DBasics positionToPack, FrameQuaternionBasics orientationToPack, FrameVector3DBasics linearVelocityToPack,
                                         FrameVector3DBasics angularVelocityToPack)
   {
      getPositionIncludingFrame(positionToPack);
      getOrientationIncludingFrame(orientationToPack);
      getLinearVelocityIncludingFrame(linearVelocityToPack);
      getAngularVelocityIncludingFrame(angularVelocityToPack);
   }

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      boolean euclideanMatch = FrameEuclideanWaypointInterface.super.epsilonEquals(other, epsilon);
      boolean so3Match = FrameSO3WaypointInterface.super.epsilonEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   @Override
   default boolean geometricallyEquals(T other, double epsilon)
   {
      boolean euclideanMatch = FrameEuclideanWaypointInterface.super.geometricallyEquals(other, epsilon);
      boolean so3Match = FrameSO3WaypointInterface.super.geometricallyEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   @Override
   default void set(T other)
   {
      FrameEuclideanWaypointInterface.super.set(other);
      FrameSO3WaypointInterface.super.set(other);
   }

   @Override
   default void setToNaN()
   {
      FrameEuclideanWaypointInterface.super.setToNaN();
      FrameSO3WaypointInterface.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      FrameEuclideanWaypointInterface.super.setToZero();
      FrameSO3WaypointInterface.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return FrameEuclideanWaypointInterface.super.containsNaN() || FrameSO3WaypointInterface.super.containsNaN();
   }

   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      FrameEuclideanWaypointInterface.super.setToNaN(referenceFrame);
      FrameSO3WaypointInterface.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      FrameEuclideanWaypointInterface.super.setToZero(referenceFrame);
      FrameSO3WaypointInterface.super.setToZero(referenceFrame);
   }
}
