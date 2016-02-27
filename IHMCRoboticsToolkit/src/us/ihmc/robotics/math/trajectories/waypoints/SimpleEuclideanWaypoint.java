package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointInterface;

public class SimpleEuclideanWaypoint implements EuclideanWaypointInterface<SimpleEuclideanWaypoint>
{
   private final Point3d position = new Point3d();
   private final Vector3d linearVelocity = new Vector3d();

   public SimpleEuclideanWaypoint()
   {
   }

   @Override
   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void set(Point3d position, Vector3d linearVelocity)
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
   public void set(SimpleEuclideanWaypoint other)
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

   public double positionDistance(SimpleEuclideanWaypoint euclideanWaypoint)
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
   public void getPosition(Point3d positionToPack)
   {
      positionToPack.set(position);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void get(Point3d positionToPack, Vector3d linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(position);
      transform.transform(linearVelocity);
   }

   @Override
   public boolean epsilonEquals(SimpleEuclideanWaypoint other, double epsilon)
   {
      if (!position.epsilonEquals(other.position, epsilon))
         return false;
      if (!linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;
      return true;
   }

   Point3d getPosition()
   {
      return position;
   }

   Vector3d getLinearVelocity()
   {
      return linearVelocity;
   }

   NumberFormat numberFormat;

   NumberFormat getNumberFormat()
   {
      return numberFormat;
   }

   void setNumberFormat(NumberFormat numberFormat)
   {
      this.numberFormat = numberFormat;
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
