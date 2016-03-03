package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSE3TrajectoryPoint extends YoFrameTrajectoryPoint<YoFrameSE3TrajectoryPoint, FrameSE3TrajectoryPoint, SimpleSE3TrajectoryPoint>
      implements SE3TrajectoryPointInterface<YoFrameSE3TrajectoryPoint>
{
   private final YoFramePoint position;
   private final YoFrameQuaternion orientation;
   private final YoFrameVector linearVelocity;
   private final YoFrameVector angularVelocity;

   public YoFrameSE3TrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(new FrameSE3TrajectoryPoint(), namePrefix, nameSuffix, registry, referenceFrames);

      position = YoFrameEuclideanWaypoint.createYoPosition(this, namePrefix, nameSuffix, registry);
      orientation = YoFrameSO3Waypoint.createYoOrientation(this, namePrefix, nameSuffix, registry);
      linearVelocity = YoFrameEuclideanWaypoint.createYoLinearVelocity(this, namePrefix, nameSuffix, registry);
      angularVelocity = YoFrameSO3Waypoint.createYoAngularVelocity(this, namePrefix, nameSuffix, registry);
   }

   @Override
   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   @Override
   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void set(SE3TrajectoryPointInterface<?> se3TrajectoryPoint)
   {
      frameWaypoint.setToZero(getReferenceFrame());
      frameWaypoint.set(se3TrajectoryPoint);
      getYoValuesFromFrameWaypoint();
   }

   public void set(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.orientation.set(orientation);
      this.linearVelocity.set(linearVelocity);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.orientation.set(orientation);
      this.linearVelocity.set(linearVelocity);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(DoubleYoVariable time, YoFramePoint position, YoFrameQuaternion orientation, YoFrameVector linearVelocity, YoFrameVector angularVelocity)
   {
      this.time.set(time.getDoubleValue());
      this.position.set(position);
      this.orientation.set(orientation);
      this.linearVelocity.set(linearVelocity);
      this.angularVelocity.set(angularVelocity);
   }

   @Override
   public void setPositionToZero()
   {
      position.setToZero();
   }

   @Override
   public void setOrientationToZero()
   {
      orientation.setToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      linearVelocity.setToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      angularVelocity.setToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      position.setToNaN();
   }

   @Override
   public void setOrientationToNaN()
   {
      orientation.setToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      linearVelocity.setToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      angularVelocity.setToNaN();
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      setTimeToNaN();
      setPositionToNaN();
      setOrientationToNaN();
      setLinearVelocityToNaN();
      setAngularVelocityToNaN();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      super.setToNaN(referenceFrame);
      setToNaN();
   }

   @Override
   public double positionDistance(YoFrameSE3TrajectoryPoint other)
   {
      putYoValuesIntoFrameWaypoint();
      other.putYoValuesIntoFrameWaypoint();
      return frameWaypoint.positionDistance(other.frameWaypoint);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      position.get(positionToPack);
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      orientation.get(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocity.get(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      position.getFrameTuple(positionToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientation(orientationToPack);
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      linearVelocity.getFrameTuple(linearVelocityToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocity.getFrameTuple(angularVelocityToPack);
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocity.getFrameTupleIncludingFrame(linearVelocityToPack);
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocity.getFrameTupleIncludingFrame(angularVelocityToPack);
   }

   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(position);
   }

   public void getOrientation(YoFrameQuaternion orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void getLinearVelocity(YoFrameVector linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void getAngularVelocity(YoFrameVector angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   /**
    * Return the original position held by this trajectory point.
    */
   public YoFramePoint getPosition()
   {
      return position;
   }

   /**
    * Return the original orientation held by this trajectory point.
    */
   public YoFrameQuaternion getOrientation()
   {
      return orientation;
   }

   /**
    * Return the original linearVelocity held by this trajectory point.
    */
   public YoFrameVector getLinearVelocity()
   {
      return linearVelocity;
   }

   /**
    * Return the original angularVelocity held by this trajectory point.
    */
   public YoFrameVector getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   protected void getYoValuesFromFrameWaypoint()
   {
      SimpleSE3TrajectoryPoint simpleTrajectoryPoint = frameWaypoint.getSimpleWaypoint();
      SimpleEuclideanWaypoint euclideanWaypoint = simpleTrajectoryPoint.getEuclideanWaypoint();
      SimpleSO3Waypoint so3Waypoint = simpleTrajectoryPoint.getSO3Waypoint();

      time.set(simpleTrajectoryPoint.getTime());
      position.set(euclideanWaypoint.getPosition());
      orientation.set(so3Waypoint.getOrientation());
      linearVelocity.set(euclideanWaypoint.getLinearVelocity());
      angularVelocity.set(so3Waypoint.getAngularVelocity());
   }

   @Override
   protected void putYoValuesIntoFrameWaypoint()
   {
      frameWaypoint.setToZero(getReferenceFrame());
      frameWaypoint.set(this);
   }

   @Override
   public String toString()
   {
      putYoValuesIntoFrameWaypoint();
      return frameWaypoint.toString();
   }
}
