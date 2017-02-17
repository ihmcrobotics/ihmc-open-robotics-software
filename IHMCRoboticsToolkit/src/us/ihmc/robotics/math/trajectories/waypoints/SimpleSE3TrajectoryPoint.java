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
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.geometry.transformables.SE3Waypoint;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;

public class SimpleSE3TrajectoryPoint extends SimpleTrajectoryPoint<SE3Waypoint, SimpleSE3TrajectoryPoint>
      implements SE3TrajectoryPointInterface<SimpleSE3TrajectoryPoint>
{
   public SimpleSE3TrajectoryPoint()
   {
      super(new SE3Waypoint());
   }

   public SimpleSE3TrajectoryPoint(double time, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity,
         Vector3DReadOnly angularVelocity)
   {
      this();
      this.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public SimpleSE3TrajectoryPoint(SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint)
   {
      this();
      this.set(simpleSE3TrajectoryPoint);
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      waypointData.setPosition(position);
   }

   @Override
   public void setOrientation(QuaternionReadOnly orientation)
   {
      waypointData.setOrientation(orientation);
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      waypointData.setLinearVelocity(linearVelocity);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      waypointData.setAngularVelocity(angularVelocity);
   }

   public void set(double time, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      waypointData.set(position, orientation, linearVelocity, angularVelocity);
   }

   public void set(double time, EuclideanWaypointInterface<?> euclideanWaypoint, SO3WaypointInterface<?> so3Waypoint)
   {
      setTime(time);
      waypointData.set(euclideanWaypoint, so3Waypoint);
   }
   
   public void set(double time, SE3WaypointInterface<?> se3Waypoint)
   {
      setTime(time);
      waypointData.set(se3Waypoint);
   }

   public void set(SE3TrajectoryPointInterface<?> se3TrajectoryPoint)
   {
      setTime(se3TrajectoryPoint.getTime());
      waypointData.set(se3TrajectoryPoint);
   }

   @Override
   public void setPositionToZero()
   {
      waypointData.setPositionToZero();
   }

   @Override
   public void setOrientationToZero()
   {
      waypointData.setOrientationToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      waypointData.setLinearVelocityToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      waypointData.setAngularVelocityToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      waypointData.setPositionToNaN();
   }

   @Override
   public void setOrientationToNaN()
   {
      waypointData.setOrientationToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      waypointData.setLinearVelocityToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      waypointData.setAngularVelocityToNaN();
   }

   @Override
   public double positionDistance(SimpleSE3TrajectoryPoint other)
   {
      return waypointData.positionDistance(other.waypointData);
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      waypointData.getPosition(positionToPack);
   }

   @Override
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      waypointData.getOrientation(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      waypointData.getLinearVelocity(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      waypointData.getAngularVelocity(angularVelocityToPack);
   }

   public double get(Point3DBasics positionToPack, QuaternionBasics orientationToPack, Vector3DBasics linearVelocityToPack, Vector3DBasics angularVelocityToPack)
   {
      waypointData.get(positionToPack, orientationToPack, linearVelocityToPack, angularVelocityToPack);
      return getTime();
   }

   public double get(EuclideanWaypoint euclideanWaypointToPack, SO3Waypoint so3WaypointToPack)
   {
      waypointData.get(euclideanWaypointToPack, so3WaypointToPack);
      return getTime();
   }

   public void get(SimpleSE3TrajectoryPoint se3TrajectoryPointToPack)
   {
      se3TrajectoryPointToPack.setTime(getTime());
      
      EuclideanWaypoint euclideanWaypoint = waypointData.getEuclideanWaypoint();
      SO3Waypoint so3Waypoint = waypointData.getSO3Waypoint();

      se3TrajectoryPointToPack.setPosition(euclideanWaypoint.getPosition());
      se3TrajectoryPointToPack.setLinearVelocity(euclideanWaypoint.getLinearVelocity());
      se3TrajectoryPointToPack.setOrientation(so3Waypoint.getOrientation());
      se3TrajectoryPointToPack.setAngularVelocity(so3Waypoint.getAngularVelocity());
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

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());

      return "SE3 trajectory point: (" + timeToString + ", " + waypointData + ")";
   }

   EuclideanWaypoint getEuclideanWaypoint()
   {
      return waypointData.getEuclideanWaypoint();
   }

   SO3Waypoint getSO3Waypoint()
   {
      return waypointData.getSO3Waypoint();
   }

   public Point3D getPositionCopy()
   {
      Point3D positionCopy = new Point3D();
      getPosition(positionCopy);
      return positionCopy;
   }

   public Quaternion getOrientationCopy()
   {
      Quaternion orientationCopy = new Quaternion();
      getOrientation(orientationCopy);
      return orientationCopy;
   }

   public Vector3D getLinearVelocityCopy()
   {
      Vector3D linearVelocityCopy = new Vector3D();
      getLinearVelocity(linearVelocityCopy);
      return linearVelocityCopy;
   }

   public Vector3D getAngularVelocityCopy()
   {
      Vector3D getAngularVelocityCopy = new Vector3D();
      getAngularVelocity(getAngularVelocityCopy);
      return getAngularVelocityCopy;
   }
}
