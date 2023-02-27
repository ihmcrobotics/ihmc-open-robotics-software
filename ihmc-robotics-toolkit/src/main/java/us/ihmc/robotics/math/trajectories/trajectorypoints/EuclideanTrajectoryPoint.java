package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanWaypoint;

public class EuclideanTrajectoryPoint implements EuclideanTrajectoryPointBasics
{
   private final EuclideanWaypoint euclideanWaypoint = new EuclideanWaypoint();
   private double time;

   public EuclideanTrajectoryPoint()
   {
   }

   public EuclideanTrajectoryPoint(EuclideanTrajectoryPointBasics other)
   {
      set(other);
   }

   public EuclideanTrajectoryPoint(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      set(time, position, linearVelocity);
   }

   @Override
   public Point3D getPosition()
   {
      return euclideanWaypoint.getPosition();
   }

   @Override
   public Vector3D getLinearVelocity()
   {
      return euclideanWaypoint.getLinearVelocity();
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getTime(), getPosition(), getLinearVelocity());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof EuclideanTrajectoryPointReadOnly)
         return equals((EuclideanTrajectoryPointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
