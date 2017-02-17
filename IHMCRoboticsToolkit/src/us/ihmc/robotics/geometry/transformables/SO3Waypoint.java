package us.ihmc.robotics.geometry.transformables;

import java.text.NumberFormat;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;

public class SO3Waypoint implements GeometryObject<SO3Waypoint>, SO3WaypointInterface<SO3Waypoint>
{
   private final Quaternion orientation = new Quaternion();
   private final Vector3D angularVelocity = new Vector3D();

   public SO3Waypoint()
   {
      setToZero();
   }

   @Override
   public void setOrientation(QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void set(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setOrientation(orientation);
      setAngularVelocity(angularVelocity);
   }

   public void set(SO3WaypointInterface<?> other)
   {
      other.getOrientation(orientation);
      other.getAngularVelocity(angularVelocity);
   }

   @Override
   public void set(SO3Waypoint other)
   {
      orientation.set(other.orientation);
      angularVelocity.set(other.angularVelocity);
   }

   @Override
   public void setOrientationToZero()
   {
      orientation.set(0.0, 0.0, 0.0, 1.0);
   }

   @Override
   public void setAngularVelocityToZero()
   {
      angularVelocity.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setToZero()
   {
      setOrientationToZero();
      setAngularVelocityToZero();
   }

   @Override
   public void setOrientationToNaN()
   {
      orientation.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      angularVelocity.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public void setToNaN()
   {
      setOrientationToNaN();
      setAngularVelocityToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      if (Double.isNaN(orientation.getX()) || Double.isNaN(orientation.getY()) || Double.isNaN(orientation.getZ()) || Double.isNaN(orientation.getS()))
         return true;
      if (Double.isNaN(angularVelocity.getX()) || Double.isNaN(angularVelocity.getY()) || Double.isNaN(angularVelocity.getZ()))
         return true;
      return false;
   }

   @Override
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   public void get(QuaternionBasics orientationToPack, Vector3DBasics angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      orientation.applyTransform(transform);
      angularVelocity.applyTransform(transform);
   }

   @Override
   public boolean epsilonEquals(SO3Waypoint other, double epsilon)
   {
      if (!orientation.epsilonEquals(other.orientation, epsilon))
         return false;
      if (!angularVelocity.epsilonEquals(other.angularVelocity, epsilon))
         return false;
      return true;
   }

   public QuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   public Vector3DReadOnly getAngularVelocity()
   {
      return angularVelocity;
   }

   public double getOrientationQx()
   {
      return orientation.getX();
   }

   public double getOrientationQy()
   {
      return orientation.getY();
   }

   public double getOrientationQz()
   {
      return orientation.getZ();
   }

   public double getOrientationQs()
   {
      return orientation.getS();
   }

   public double getAngularVelocityX()
   {
      return angularVelocity.getX();
   }

   public double getAngularVelocityY()
   {
      return angularVelocity.getY();
   }

   public double getAngularVelocityZ()
   {
      return angularVelocity.getZ();
   }

   private NumberFormat numberFormat;

   public NumberFormat getNumberFormat()
   {
      return numberFormat;
   }

   public void setNumberFormat(NumberFormat numberFormat)
   {
      this.numberFormat = numberFormat;
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
