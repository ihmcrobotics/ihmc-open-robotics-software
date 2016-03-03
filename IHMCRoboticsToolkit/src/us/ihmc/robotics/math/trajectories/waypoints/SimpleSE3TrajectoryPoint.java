package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.SimpleEuclideanWaypoint;
import us.ihmc.robotics.geometry.transformables.SimpleSE3Waypoint;
import us.ihmc.robotics.geometry.transformables.SimpleSO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointInterface;

public class SimpleSE3TrajectoryPoint extends SimpleTrajectoryPoint<SimpleSE3Waypoint, SimpleSE3TrajectoryPoint>
      implements SE3TrajectoryPointInterface<SimpleSE3TrajectoryPoint>
{
   public SimpleSE3TrajectoryPoint()
   {
      super(new SimpleSE3Waypoint());
   }

   @Override
   public void setPosition(Point3d position)
   {
      waypointData.setPosition(position);
   }

   @Override
   public void setOrientation(Quat4d orientation)
   {
      waypointData.setOrientation(orientation);
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      waypointData.setLinearVelocity(linearVelocity);
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      waypointData.setAngularVelocity(angularVelocity);
   }

   public void set(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      setTime(time);
      waypointData.set(position, orientation, linearVelocity, angularVelocity);
   }

   public void set(double time, EuclideanWaypointInterface<?> euclideanWaypoint, SO3WaypointInterface<?> so3Waypoint)
   {
      setTime(time);
      waypointData.set(euclideanWaypoint, so3Waypoint);
   }
   
   public void set(double time, SE3WaypointInterface<?> se3Waypoint)
   {
      setTime(time);
      waypointData.set(se3Waypoint);
   }

   public void set(SE3TrajectoryPointInterface<?> se3TrajectoryPoint)
   {
      setTime(se3TrajectoryPoint.getTime());
      waypointData.set(se3TrajectoryPoint);
   }

   @Override
   public void setPositionToZero()
   {
      waypointData.setPositionToZero();
   }

   @Override
   public void setOrientationToZero()
   {
      waypointData.setOrientationToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      waypointData.setLinearVelocityToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      waypointData.setAngularVelocityToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      waypointData.setPositionToNaN();
   }

   @Override
   public void setOrientationToNaN()
   {
      waypointData.setOrientationToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      waypointData.setLinearVelocityToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      waypointData.setAngularVelocityToNaN();
   }

   @Override
   public double positionDistance(SimpleSE3TrajectoryPoint other)
   {
      return waypointData.positionDistance(other.waypointData);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      waypointData.getPosition(positionToPack);
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      waypointData.getOrientation(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      waypointData.getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      waypointData.getAngularVelocity(angularVelocityToPack);
   }

   public double get(Point3d positionToPack, Quat4d orientationToPack, Vector3d linearVelocityToPack, Vector3d angularVelocityToPack)
   {
      waypointData.get(positionToPack, orientationToPack, linearVelocityToPack, angularVelocityToPack);
      return getTime();
   }

   public double get(EuclideanWaypointInterface<?> euclideanWaypointToPack, SO3WaypointInterface<?> so3WaypointToPack)
   {
      waypointData.get(euclideanWaypointToPack, so3WaypointToPack);
      return getTime();
   }

   public double get(SE3WaypointInterface<?> se3WaypointToPack)
   {
      SimpleEuclideanWaypoint euclideanWaypoint = waypointData.getEuclideanWaypoint();
      SimpleSO3Waypoint so3Waypoint = waypointData.getSO3Waypoint();

      se3WaypointToPack.setPosition(euclideanWaypoint.getPosition());
      se3WaypointToPack.setLinearVelocity(euclideanWaypoint.getLinearVelocity());
      se3WaypointToPack.setOrientation(so3Waypoint.getOrientation());
      se3WaypointToPack.setAngularVelocity(so3Waypoint.getAngularVelocity());
      return getTime();
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      waypointData.applyTransform(transform);
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());

      return "SE3 trajectory point: (" + timeToString + ", " + waypointData + ")";
   }

   SimpleEuclideanWaypoint getEuclideanWaypoint()
   {
      return waypointData.getEuclideanWaypoint();
   }

   SimpleSO3Waypoint getSO3Waypoint()
   {
      return waypointData.getSO3Waypoint();
   }
}
