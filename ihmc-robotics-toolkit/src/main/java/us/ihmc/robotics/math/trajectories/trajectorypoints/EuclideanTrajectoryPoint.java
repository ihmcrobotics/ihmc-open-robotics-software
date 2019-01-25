package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;

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
   public Point3DReadOnly getPosition()
   {
      return euclideanWaypoint.getPosition();
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      euclideanWaypoint.setPosition(x, y, z);
   }

   @Override
   public Vector3DReadOnly getLinearVelocity()
   {
      return euclideanWaypoint.getLinearVelocity();
   }

   @Override
   public void setLinearVelocity(double x, double y, double z)
   {
      euclideanWaypoint.setLinearVelocity(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      euclideanWaypoint.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      euclideanWaypoint.applyInverseTransform(transform);
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
   public String toString()
   {
      return "Euclidean trajectory point: (time = " + WaypointToStringTools.formatTime(getTime()) + ", " + WaypointToStringTools.waypointToString(euclideanWaypoint)
      + ")";
   }
}
