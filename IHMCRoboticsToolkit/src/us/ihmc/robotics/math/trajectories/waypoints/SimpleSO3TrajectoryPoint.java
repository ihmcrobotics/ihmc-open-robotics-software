package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.SimpleSO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointInterface;

public class SimpleSO3TrajectoryPoint extends SimpleTrajectoryPoint<SimpleSO3Waypoint, SimpleSO3TrajectoryPoint>
      implements SO3TrajectoryPointInterface<SimpleSO3TrajectoryPoint>
{
   public SimpleSO3TrajectoryPoint()
   {
      super(new SimpleSO3Waypoint());
   }

   @Override
   public void setOrientation(Quat4d orientation)
   {
      waypointData.setOrientation(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      waypointData.setAngularVelocity(angularVelocity);
   }

   public void set(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      setTime(time);
      waypointData.set(orientation, angularVelocity);
   }

   public void set(double time, SO3WaypointInterface<?> so3Waypoint)
   {
      setTime(time);
      waypointData.set(so3Waypoint);
   }

   public void set(SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      setTime(so3TrajectoryPoint.getTime());
      waypointData.set(so3TrajectoryPoint);
   }

   @Override
   public void setOrientationToZero()
   {
      waypointData.setOrientationToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      waypointData.setAngularVelocityToZero();
   }

   @Override
   public void setOrientationToNaN()
   {
      waypointData.setOrientationToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      waypointData.setAngularVelocityToNaN();
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      waypointData.getOrientation(orientationToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      waypointData.getAngularVelocity(angularVelocityToPack);
   }

   public double get(Quat4d orientationToPack, Vector3d angularVelocityToPack)
   {
      waypointData.get(orientationToPack, angularVelocityToPack);
      return getTime();
   }

   public double get(SO3WaypointInterface<?> so3Waypoint)
   {
      so3Waypoint.setOrientation(waypointData.getOrientation());
      so3Waypoint.setAngularVelocity(waypointData.getAngularVelocity());
      return getTime();
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      waypointData.applyTransform(transform);
   }

   Quat4d getOrientation()
   {
      return waypointData.getOrientation();
   }

   Vector3d getAngularVelocity()
   {
      return waypointData.getAngularVelocity();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");

      String timeToString = "time = " + doubleFormat.format(getTime());

      return "SO3 trajectory point: (" + timeToString + ", " + waypointData + ")";
   }
}
