package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;

public class SimpleSO3TrajectoryPoint extends SimpleTrajectoryPoint<SO3Waypoint, SimpleSO3TrajectoryPoint>
      implements SO3TrajectoryPointInterface<SimpleSO3TrajectoryPoint>
{
   public SimpleSO3TrajectoryPoint()
   {
      super(new SO3Waypoint());
   }

   public SimpleSO3TrajectoryPoint(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
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
   public void setOrientation(QuaternionReadOnly orientation)
   {
      waypointData.setOrientation(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      waypointData.setAngularVelocity(angularVelocity);
   }

   public void set(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
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
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      waypointData.getOrientation(orientationToPack);
   }

   public Quaternion getOrientationCopy()
   {
      Quaternion orientationCopy = new Quaternion();
      getOrientation(orientationCopy);
      return orientationCopy;
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      waypointData.getAngularVelocity(angularVelocityToPack);
   }
   
   public Vector3D getAngularVelocityCopy()
   {
      Vector3D angularVelocityCopy = new Vector3D();
      getAngularVelocity(angularVelocityCopy);
      return angularVelocityCopy;
   }

   public double get(QuaternionBasics orientationToPack, Vector3DBasics angularVelocityToPack)
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
   public void applyTransform(Transform transform)
   {
      waypointData.applyTransform(transform);
   }

   QuaternionReadOnly getOrientation()
   {
      return waypointData.getOrientation();
   }

   Vector3DReadOnly getAngularVelocity()
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
