package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;

public interface SE3WaypointReadOnly extends EuclideanWaypointReadOnly, SO3WaypointReadOnly
{
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

   default boolean epsilonEquals(SE3WaypointReadOnly other, double epsilon)
   {
      boolean euclideanMatch = EuclideanWaypointReadOnly.super.epsilonEquals(other, epsilon);
      boolean so3Match = SO3WaypointReadOnly.super.epsilonEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   default boolean geometricallyEquals(SE3WaypointReadOnly other, double epsilon)
   {
      boolean euclideanMatch = EuclideanWaypointReadOnly.super.geometricallyEquals(other, epsilon);
      boolean so3Match = SO3WaypointReadOnly.super.geometricallyEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   @Override
   default boolean containsNaN()
   {
      return EuclideanWaypointReadOnly.super.containsNaN() || SO3WaypointReadOnly.super.containsNaN();
   }
}