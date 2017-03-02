package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.geometry.yoFrameObjects.YoFrameSO3Waypoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSO3TrajectoryPoint extends YoFrameTrajectoryPoint<YoFrameSO3TrajectoryPoint, FrameSO3TrajectoryPoint, SimpleSO3TrajectoryPoint>
      implements SO3TrajectoryPointInterface<YoFrameSO3TrajectoryPoint>
{
   private final YoFrameQuaternion orientation;
   private final YoFrameVector angularVelocity;

   public YoFrameSO3TrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(new FrameSO3TrajectoryPoint(), namePrefix, nameSuffix, registry, referenceFrames);

      orientation = YoFrameSO3Waypoint.createYoOrientation(this, namePrefix, nameSuffix, registry);
      angularVelocity = YoFrameSO3Waypoint.createYoAngularVelocity(this, namePrefix, nameSuffix, registry);
   }

   @Override
   public void setOrientation(QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      this.orientation.set(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void setAngularVelocity(FrameVector angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }
   public void set(SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      frameWaypoint.setToZero(getReferenceFrame());
      frameWaypoint.set(so3TrajectoryPoint);
      getYoValuesFromFrameWaypoint();
   }

   public void set(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      this.time.set(time);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      this.time.set(time);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, YoFrameQuaternion orientation, YoFrameVector angularVelocity)
   {
      this.time.set(time);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }
   
   @Override
   public void setOrientationToZero()
   {
      orientation.setToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      angularVelocity.setToZero();
   }

   @Override
   public void setOrientationToNaN()
   {
      orientation.setToNaN();
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
      setOrientationToNaN();
      setAngularVelocityToNaN();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      super.setToNaN(referenceFrame);
      setToNaN();
   }

   @Override
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      orientation.get(orientationToPack);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientation(orientationToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocity.getFrameTuple(angularVelocityToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocity.getFrameTupleIncludingFrame(angularVelocityToPack);
   }

   public void getOrientation(YoFrameQuaternion orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void getAngularVelocity(YoFrameVector angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   /**
    * Return the original orientation held by this trajectory point.
    */
   public YoFrameQuaternion getOrientation()
   {
      return orientation;
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
      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = frameWaypoint.getGeometryObject();
      SO3Waypoint so3Waypoint = simpleTrajectoryPoint.getSO3Waypoint();
      
      time.set(simpleTrajectoryPoint.getTime());
      orientation.set(so3Waypoint.getOrientation());
      angularVelocity.set(so3Waypoint.getAngularVelocity());
   }

   @Override
   protected void putYoValuesIntoFrameWaypoint()
   {
      frameWaypoint.setToZero(getReferenceFrame());

      frameWaypoint.setTime(time.getDoubleValue());
      frameWaypoint.setOrientation(orientation.getFrameOrientation());
      frameWaypoint.setAngularVelocity(angularVelocity.getFrameTuple());
   }

   @Override
   public String toString()
   {
      putYoValuesIntoFrameWaypoint();
      return frameWaypoint.toString();
   }
}
