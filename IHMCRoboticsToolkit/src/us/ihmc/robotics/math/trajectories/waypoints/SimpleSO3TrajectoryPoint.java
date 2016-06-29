package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.geometry.transformables.TransformableQuat4d;
import us.ihmc.robotics.geometry.transformables.TransformableVector3d;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;

public class SimpleSO3TrajectoryPoint extends SimpleTrajectoryPoint<SO3Waypoint, SimpleSO3TrajectoryPoint>
      implements SO3TrajectoryPointInterface<SimpleSO3TrajectoryPoint>
{
   public SimpleSO3TrajectoryPoint()
   {
      super(new SO3Waypoint());
   }

   public SimpleSO3TrajectoryPoint(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      this();
      set(time, orientation, angularVelocity);
   }

   public SimpleSO3TrajectoryPoint(SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint)
   {
      this();
      set(simpleSO3TrajectoryPoint);
   }

   public SO3Waypoint getSO3Waypoint()
   {
      return waypointData;
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

   public TransformableQuat4d getOrientationCopy()
   {
      TransformableQuat4d orientationCopy = new TransformableQuat4d();
      getOrientation(orientationCopy);
      return orientationCopy;
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      waypointData.getAngularVelocity(angularVelocityToPack);
   }
   
   public TransformableVector3d getAngularVelocityCopy()
   {
      TransformableVector3d angularVelocityCopy = new TransformableVector3d();
      getAngularVelocity(angularVelocityCopy);
      return angularVelocityCopy;
   }

   public double get(Quat4d orientationToPack, Vector3d angularVelocityToPack)
   {
      waypointData.get(orientationToPack, angularVelocityToPack);
      return getTime();
   }

   public void get(SimpleSO3TrajectoryPoint simpleSO3TrajectoryPointToPack)
   {
      simpleSO3TrajectoryPointToPack.setTime(getTime());
      simpleSO3TrajectoryPointToPack.setOrientation(waypointData.getOrientation());
      simpleSO3TrajectoryPointToPack.setAngularVelocity(waypointData.getAngularVelocity());
   }

   public double getOrientationQx()
   {
      return waypointData.getOrientationQx();
   }

   public double getOrientationQy()
   {
      return waypointData.getOrientationQy();
   }

   public double getOrientationQz()
   {
      return waypointData.getOrientationQz();
   }

   public double getOrientationQs()
   {
      return waypointData.getOrientationQs();
   }

   public double getAngularVelocityX()
   {
      return waypointData.getAngularVelocityX();
   }

   public double getAngularVelocityY()
   {
      return waypointData.getAngularVelocityY();
   }

   public double getAngularVelocityZ()
   {
      return waypointData.getAngularVelocityZ();
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
