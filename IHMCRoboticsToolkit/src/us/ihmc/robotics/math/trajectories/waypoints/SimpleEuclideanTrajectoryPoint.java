package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
import us.ihmc.robotics.geometry.transformables.TransformableVector3d;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;

public class SimpleEuclideanTrajectoryPoint extends SimpleTrajectoryPoint<EuclideanWaypoint, SimpleEuclideanTrajectoryPoint>
      implements EuclideanTrajectoryPointInterface<SimpleEuclideanTrajectoryPoint>
{
   public SimpleEuclideanTrajectoryPoint()
   {
      super(new EuclideanWaypoint());
   }

   public SimpleEuclideanTrajectoryPoint(double time, Point3d position, Vector3d linearVelocity)
   {
      this();
      set(time, position, linearVelocity);
   }

   public SimpleEuclideanTrajectoryPoint(SimpleEuclideanTrajectoryPoint simpleEuclideanTrajectoryPoint)
   {
      this();
      set(simpleEuclideanTrajectoryPoint);
   }

   public EuclideanWaypoint getEuclideanWaypoint()
   {
      return waypointData;
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

   public double positionDistance(EuclideanWaypoint euclideanWaypoint)
   {
      return waypointData.positionDistance(euclideanWaypoint);
   }

   public double positionDistance(SimpleEuclideanTrajectoryPoint euclideanTrajectoryPoint)
   {
      return positionDistance(euclideanTrajectoryPoint.waypointData);
   }
   
   @Override
   public void getPosition(Point3d positionToPack)
   {
      waypointData.getPosition(positionToPack);
   }

   public TransformablePoint3d getPositionCopy()
   {
      TransformablePoint3d positionCopy = new TransformablePoint3d();
      getPosition(positionCopy);
      return positionCopy;
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      waypointData.getLinearVelocity(linearVelocityToPack);
   }

   public TransformableVector3d getLinearVelocityCopy()
   {
      TransformableVector3d linearVelocityCopy = new TransformableVector3d();
      getLinearVelocity(linearVelocityCopy);
      return linearVelocityCopy;
   }

   public double get(Point3d positionToPack, Vector3d linearVelocityToPack)
   {
      waypointData.get(positionToPack, linearVelocityToPack);
      return getTime();
   }

   public void get(SimpleEuclideanTrajectoryPoint simpleEuclideanTrajectoryPointToPack)
   {
      simpleEuclideanTrajectoryPointToPack.setTime(getTime());
      simpleEuclideanTrajectoryPointToPack.setPosition(waypointData.getPosition());
      simpleEuclideanTrajectoryPointToPack.setLinearVelocity(waypointData.getLinearVelocity());
   }

   public double getPositionX()
   {
      return waypointData.getPositionX();
   }

   public double getPositionY()
   {
      return waypointData.getPositionY();
   }

   public double getPositionZ()
   {
      return waypointData.getPositionZ();
   }

   public double getLinearVelocityX()
   {
      return waypointData.getLinearVelocityX();
   }

   public double getLinearVelocityY()
   {
      return waypointData.getLinearVelocityY();
   }

   public double getLinearVelocityZ()
   {
      return waypointData.getLinearVelocityZ();
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
