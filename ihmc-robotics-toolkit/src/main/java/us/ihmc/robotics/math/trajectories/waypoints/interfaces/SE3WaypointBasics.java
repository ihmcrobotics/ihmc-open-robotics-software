package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SE3WaypointBasics extends EuclideanWaypointBasics, SO3WaypointBasics
{
   default void set(Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setPosition(position);
      setOrientation(orientation);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
   }

   default void get(Point3DBasics positionToPack, QuaternionBasics orientationToPack, Vector3DBasics linearVelocityToPack, Vector3DBasics angularVelocityToPack)
   {
      getPosition(positionToPack);
      getOrientation(orientationToPack);
      getLinearVelocity(linearVelocityToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   default void get(SE3WaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void get(EuclideanWaypointBasics euclideanWaypointToPack, SO3WaypointBasics so3WaypointToPack)
   {
      get(euclideanWaypointToPack);
      get(so3WaypointToPack);
   }

   default void getPose(Pose3DBasics poseToPack)
   {
      poseToPack.set(getPosition(), getOrientation());
   }

   default boolean epsilonEquals(SE3WaypointBasics other, double epsilon)
   {
      boolean euclideanMatch = EuclideanWaypointBasics.super.epsilonEquals(other, epsilon);
      boolean so3Match = SO3WaypointBasics.super.epsilonEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   default boolean geometricallyEquals(SE3WaypointBasics other, double epsilon)
   {
      boolean euclideanMatch = EuclideanWaypointBasics.super.geometricallyEquals(other, epsilon);
      boolean so3Match = SO3WaypointBasics.super.geometricallyEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   default void set(SE3WaypointBasics other)
   {
      EuclideanWaypointBasics.super.set(other);
      SO3WaypointBasics.super.set(other);
   }

   @Override
   default void setToNaN()
   {
      EuclideanWaypointBasics.super.setToNaN();
      SO3WaypointBasics.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      EuclideanWaypointBasics.super.setToZero();
      SO3WaypointBasics.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return EuclideanWaypointBasics.super.containsNaN() || SO3WaypointBasics.super.containsNaN();
   }
}
