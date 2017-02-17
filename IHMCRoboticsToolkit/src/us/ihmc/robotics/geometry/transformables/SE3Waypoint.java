package us.ihmc.robotics.geometry.transformables;

import java.text.NumberFormat;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;

public class SE3Waypoint implements GeometryObject<SE3Waypoint>, SE3WaypointInterface<SE3Waypoint>
{
   private final EuclideanWaypoint euclideanWaypoint = new EuclideanWaypoint();
   private final SO3Waypoint so3Waypoint = new SO3Waypoint();

   public SE3Waypoint()
   {
      setToZero();
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      euclideanWaypoint.setPosition(position);
   }

   @Override
   public void setOrientation(QuaternionReadOnly orientation)
   {
      so3Waypoint.setOrientation(orientation);
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      euclideanWaypoint.setLinearVelocity(linearVelocity);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      so3Waypoint.setAngularVelocity(angularVelocity);
   }

   public void set(Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      euclideanWaypoint.set(position, linearVelocity);
      so3Waypoint.set(orientation, angularVelocity);
   }

   public void set(EuclideanWaypointInterface<?> euclideanWaypoint, SO3WaypointInterface<?> so3Waypoint)
   {
      this.euclideanWaypoint.set(euclideanWaypoint);
      this.so3Waypoint.set(so3Waypoint);
   }

   public void set(SE3WaypointInterface<?> se3Waypoint)
   {
      this.euclideanWaypoint.set(se3Waypoint);
      this.so3Waypoint.set(se3Waypoint);
   }

   @Override
   public void set(SE3Waypoint other)
   {
      euclideanWaypoint.set(other);
      so3Waypoint.set(other);
   }

   @Override
   public void setPositionToZero()
   {
      euclideanWaypoint.setPositionToZero();
   }

   @Override
   public void setOrientationToZero()
   {
      so3Waypoint.setOrientationToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      euclideanWaypoint.setLinearVelocityToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      so3Waypoint.setAngularVelocityToZero();
   }

   @Override
   public void setToZero()
   {
      euclideanWaypoint.setToZero();
      so3Waypoint.setToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      euclideanWaypoint.setPositionToNaN();
   }

   @Override
   public void setOrientationToNaN()
   {
      so3Waypoint.setOrientationToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      euclideanWaypoint.setLinearVelocityToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      so3Waypoint.setAngularVelocityToNaN();
   }

   @Override
   public void setToNaN()
   {
      euclideanWaypoint.setToNaN();
      so3Waypoint.setToNaN();
   }

   @Override
   public double positionDistance(SE3Waypoint other)
   {
      return euclideanWaypoint.positionDistance(other.euclideanWaypoint);
   }

   @Override
   public boolean containsNaN()
   {
      return euclideanWaypoint.containsNaN() || so3Waypoint.containsNaN();
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      euclideanWaypoint.getPosition(positionToPack);
   }

   @Override
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      so3Waypoint.getOrientation(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      euclideanWaypoint.getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      so3Waypoint.getAngularVelocity(angularVelocityToPack);
   }

   public void get(Point3DBasics positionToPack, QuaternionBasics orientationToPack, Vector3DBasics linearVelocityToPack, Vector3DBasics angularVelocityToPack)
   {
      getPosition(positionToPack);
      getOrientation(orientationToPack);
      getLinearVelocity(linearVelocityToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   public void get(EuclideanWaypointInterface<?> euclideanWaypointToPack, SO3WaypointInterface<?> so3WaypointToPack)
   {
      euclideanWaypointToPack.setPosition(euclideanWaypoint.getPosition());
      euclideanWaypointToPack.setLinearVelocity(euclideanWaypoint.getLinearVelocity());
      so3WaypointToPack.setOrientation(so3Waypoint.getOrientation());
      so3WaypointToPack.setAngularVelocity(so3Waypoint.getAngularVelocity());
   }

   @Override
   public void applyTransform(Transform transform)
   {
      euclideanWaypoint.applyTransform(transform);
      so3Waypoint.applyTransform(transform);
   }

   @Override
   public boolean epsilonEquals(SE3Waypoint other, double epsilon)
   {
      if (!euclideanWaypoint.epsilonEquals(other.euclideanWaypoint, epsilon))
         return false;
      if (!so3Waypoint.epsilonEquals(other.so3Waypoint, epsilon))
         return false;
      return true;
   }

   public EuclideanWaypoint getEuclideanWaypoint()
   {
      return euclideanWaypoint;
   }

   public SO3Waypoint getSO3Waypoint()
   {
      return so3Waypoint;
   }

   public double getPositionX()
   {
      return euclideanWaypoint.getPositionX();
   }

   public double getPositionY()
   {
      return euclideanWaypoint.getPositionY();
   }

   public double getPositionZ()
   {
      return euclideanWaypoint.getPositionZ();
   }

   public double getOrientationQx()
   {
      return so3Waypoint.getOrientationQx();
   }

   public double getOrientationQy()
   {
      return so3Waypoint.getOrientationQy();
   }

   public double getOrientationQz()
   {
      return so3Waypoint.getOrientationQz();
   }

   public double getOrientationQs()
   {
      return so3Waypoint.getOrientationQs();
   }

   public double getLinearVelocityX()
   {
      return euclideanWaypoint.getLinearVelocityX();
   }

   public double getLinearVelocityY()
   {
      return euclideanWaypoint.getLinearVelocityY();
   }

   public double getLinearVelocityZ()
   {
      return euclideanWaypoint.getLinearVelocityZ();
   }

   public double getAngularVelocityX()
   {
      return so3Waypoint.getAngularVelocityX();
   }

   public double getAngularVelocityY()
   {
      return so3Waypoint.getAngularVelocityY();
   }

   public double getAngularVelocityZ()
   {
      return so3Waypoint.getAngularVelocityZ();
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
