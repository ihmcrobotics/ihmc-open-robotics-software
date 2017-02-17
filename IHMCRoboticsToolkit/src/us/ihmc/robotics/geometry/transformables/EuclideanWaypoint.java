package us.ihmc.robotics.geometry.transformables;

import java.text.NumberFormat;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;

public class EuclideanWaypoint implements GeometryObject<EuclideanWaypoint>, EuclideanWaypointInterface<EuclideanWaypoint>
{
   private final Point3D position = new Point3D();
   private final Vector3D linearVelocity = new Vector3D();

   public EuclideanWaypoint()
   {
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      this.position.set(position);
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void set(Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setPosition(position);
      setLinearVelocity(linearVelocity);
   }

   public void set(EuclideanWaypointInterface<?> other)
   {
      other.getPosition(position);
      other.getLinearVelocity(linearVelocity);
   }

   @Override
   public void set(EuclideanWaypoint other)
   {
      position.set(other.position);
      linearVelocity.set(other.linearVelocity);
   }

   @Override
   public void setPositionToZero()
   {
      position.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setLinearVelocityToZero()
   {
      linearVelocity.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setToZero()
   {
      setPositionToZero();
      setLinearVelocityToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      position.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      linearVelocity.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public void setToNaN()
   {
      setPositionToNaN();
      setLinearVelocityToNaN();
   }

   public double positionDistance(EuclideanWaypoint euclideanWaypoint)
   {
      return position.distance(euclideanWaypoint.position);
   }

   @Override
   public boolean containsNaN()
   {
      if (Double.isNaN(position.getX()) || Double.isNaN(position.getY()) || Double.isNaN(position.getZ()))
         return true;
      if (Double.isNaN(linearVelocity.getX()) || Double.isNaN(linearVelocity.getY()) || Double.isNaN(linearVelocity.getZ()))
         return true;
      return false;
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      positionToPack.set(position);
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void get(Point3DBasics positionToPack, Vector3DBasics linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(position);
      transform.transform(linearVelocity);
   }

   @Override
   public boolean epsilonEquals(EuclideanWaypoint other, double epsilon)
   {
      if (!position.epsilonEquals(other.position, epsilon))
         return false;
      if (!linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;
      return true;
   }

   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public Vector3DReadOnly getLinearVelocity()
   {
      return linearVelocity;
   }

   public double getPositionX()
   {
      return position.getX();
   }

   public double getPositionY()
   {
      return position.getY();
   }

   public double getPositionZ()
   {
      return position.getZ();
   }

   public double getLinearVelocityX()
   {
      return linearVelocity.getX();
   }

   public double getLinearVelocityY()
   {
      return linearVelocity.getY();
   }

   public double getLinearVelocityZ()
   {
      return linearVelocity.getZ();
   }

   NumberFormat numberFormat;

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
