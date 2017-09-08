package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;

public class SimpleEuclideanTrajectoryPoint extends SimpleTrajectoryPoint<EuclideanWaypoint, SimpleEuclideanTrajectoryPoint>
      implements EuclideanTrajectoryPointInterface<SimpleEuclideanTrajectoryPoint>
{
   public SimpleEuclideanTrajectoryPoint()
   {
      super(new EuclideanWaypoint());
   }

   public SimpleEuclideanTrajectoryPoint(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
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
   public void setPosition(Point3DReadOnly position)
   {
      waypointData.setPosition(position);
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      waypointData.setLinearVelocity(linearVelocity);
   }

   public void set(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
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
   public void getPosition(Point3DBasics positionToPack)
   {
      waypointData.getPosition(positionToPack);
   }

   public Point3D getPositionCopy()
   {
      Point3D positionCopy = new Point3D();
      getPosition(positionCopy);
      return positionCopy;
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      waypointData.getLinearVelocity(linearVelocityToPack);
   }

   public Vector3D getLinearVelocityCopy()
   {
      Vector3D linearVelocityCopy = new Vector3D();
      getLinearVelocity(linearVelocityCopy);
      return linearVelocityCopy;
   }

   public double get(Point3DBasics positionToPack, Vector3DBasics linearVelocityToPack)
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
   public void applyTransform(Transform transform)
   {
      waypointData.applyTransform(transform);
   }

   Point3DReadOnly getPosition()
   {
      return waypointData.getPosition();
   }

   Vector3DReadOnly getLinearVelocity()
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
