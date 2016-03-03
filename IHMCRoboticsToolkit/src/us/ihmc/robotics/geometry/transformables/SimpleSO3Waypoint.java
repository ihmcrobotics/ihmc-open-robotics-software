package us.ihmc.robotics.geometry.transformables;

import java.text.NumberFormat;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointInterface;

public class SimpleSO3Waypoint implements SO3WaypointInterface<SimpleSO3Waypoint>
{
   private final Quat4d orientation = new Quat4d();
   private final Vector3d angularVelocity = new Vector3d();

   public SimpleSO3Waypoint()
   {
      setToZero();
   }

   @Override
   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void set(Quat4d orientation, Vector3d angularVelocity)
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
   public void set(SimpleSO3Waypoint other)
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
      if (Double.isNaN(orientation.getX()) || Double.isNaN(orientation.getY()) || Double.isNaN(orientation.getZ()) || Double.isNaN(orientation.getW()))
         return true;
      if (Double.isNaN(angularVelocity.getX()) || Double.isNaN(angularVelocity.getY()) || Double.isNaN(angularVelocity.getZ()))
         return true;
      return false;
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   public void get(Quat4d orientationToPack, Vector3d angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   private final Quat4d tempQuaternionForTransform = new Quat4d();

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.get(tempQuaternionForTransform);
      orientation.mul(tempQuaternionForTransform, orientation);
      transform.transform(angularVelocity);
   }

   @Override
   public boolean epsilonEquals(SimpleSO3Waypoint other, double epsilon)
   {
      if (!orientation.epsilonEquals(other.orientation, epsilon))
         return false;
      if (!angularVelocity.epsilonEquals(other.angularVelocity, epsilon))
         return false;
      return true;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public Vector3d getAngularVelocity()
   {
      return angularVelocity;
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
