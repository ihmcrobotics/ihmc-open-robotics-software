package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface SE3WaypointInterface extends EuclideanWaypointInterface, SO3WaypointInterface
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

   default void get(SE3WaypointInterface otherToPack)
   {
      otherToPack.set(this);
   }

   default void get(EuclideanWaypointInterface euclideanWaypointToPack, SO3WaypointInterface so3WaypointToPack)
   {
      get(euclideanWaypointToPack);
      get(so3WaypointToPack);
   }

   default void getPose(Pose3DBasics poseToPack)
   {
      poseToPack.set(getPosition(), getOrientation());
   }

   default boolean epsilonEquals(SE3WaypointInterface other, double epsilon)
   {
      boolean euclideanMatch = EuclideanWaypointInterface.super.epsilonEquals(other, epsilon);
      boolean so3Match = SO3WaypointInterface.super.epsilonEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   default boolean geometricallyEquals(SE3WaypointInterface other, double epsilon)
   {
      boolean euclideanMatch = EuclideanWaypointInterface.super.geometricallyEquals(other, epsilon);
      boolean so3Match = SO3WaypointInterface.super.geometricallyEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   default void set(SE3WaypointInterface other)
   {
      EuclideanWaypointInterface.super.set(other);
      SO3WaypointInterface.super.set(other);
   }

   @Override
   default void setToNaN()
   {
      EuclideanWaypointInterface.super.setToNaN();
      SO3WaypointInterface.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      EuclideanWaypointInterface.super.setToZero();
      SO3WaypointInterface.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return EuclideanWaypointInterface.super.containsNaN() || SO3WaypointInterface.super.containsNaN();
   }
}
