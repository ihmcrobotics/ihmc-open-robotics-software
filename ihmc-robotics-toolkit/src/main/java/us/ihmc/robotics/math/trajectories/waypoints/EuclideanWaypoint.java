package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointReadOnly;

public class EuclideanWaypoint implements EuclideanWaypointBasics
{
   private final Point3D position = new Point3D();
   private final Vector3D linearVelocity = new Vector3D();

   public EuclideanWaypoint()
   {
   }

   public EuclideanWaypoint(Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      set(position, linearVelocity);
   }

   @Override
   public Point3D getPosition()
   {
      return position;
   }

   @Override
   public Vector3D getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getPosition(), getLinearVelocity());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof EuclideanWaypointReadOnly)
         return equals((EuclideanWaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
