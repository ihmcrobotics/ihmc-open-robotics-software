package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.SimpleEuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointInterface;

public class SimpleEuclideanTrajectoryPoint extends SimpleTrajectoryPoint<SimpleEuclideanWaypoint, SimpleEuclideanTrajectoryPoint>
      implements EuclideanTrajectoryPointInterface<SimpleEuclideanTrajectoryPoint>
{
   public SimpleEuclideanTrajectoryPoint()
   {
      super(new SimpleEuclideanWaypoint());
   }

   @Override
   public void setPosition(Point3d position)
   {
      waypointData.setPosition(position);
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      waypointData.setLinearVelocity(linearVelocity);
   }

   public void set(double time, Point3d position, Vector3d linearVelocity)
   {
      setTime(time);
      waypointData.set(position, linearVelocity);
   }

   public void set(double time, EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      setTime(time);
      waypointData.set(euclideanWaypoint);
   }

   public void set(EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      setTime(euclideanTrajectoryPoint.getTime());
      waypointData.set(euclideanTrajectoryPoint);
   }

   @Override
   public void setPositionToZero()
   {
      waypointData.setPositionToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      waypointData.setLinearVelocityToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      waypointData.setPositionToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      waypointData.setLinearVelocityToNaN();
   }

   public double distance(SimpleEuclideanWaypoint euclideanWaypoint)
   {
      return waypointData.positionDistance(euclideanWaypoint);
   }

   public double positionDistance(SimpleEuclideanTrajectoryPoint euclideanTrajectoryPoint)
   {
      return waypointData.positionDistance(euclideanTrajectoryPoint.waypointData);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      waypointData.getPosition(positionToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      waypointData.getLinearVelocity(linearVelocityToPack);
   }

   public double get(Point3d positionToPack, Vector3d linearVelocityToPack)
   {
      waypointData.get(positionToPack, linearVelocityToPack);
      return getTime();
   }

   public double get(EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      euclideanWaypoint.setPosition(waypointData.getPosition());
      euclideanWaypoint.setLinearVelocity(waypointData.getLinearVelocity());
      return getTime();
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      waypointData.applyTransform(transform);
   }

   Point3d getPosition()
   {
      return waypointData.getPosition();
   }

   Vector3d getLinearVelocity()
   {
      return waypointData.getLinearVelocity();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");

      String timeToString = "time = " + doubleFormat.format(getTime());

      return "Euclidean trajectory point: (" + timeToString + ", " + waypointData + ")";
   }
}
