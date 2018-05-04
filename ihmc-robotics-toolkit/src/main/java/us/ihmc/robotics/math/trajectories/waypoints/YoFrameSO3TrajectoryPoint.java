package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.geometry.yoFrameObjects.YoFrameSO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;

public class YoFrameSO3TrajectoryPoint extends YoFrameTrajectoryPoint<YoFrameSO3TrajectoryPoint, FrameSO3TrajectoryPoint, SimpleSO3TrajectoryPoint>
      implements SO3TrajectoryPointInterface<YoFrameSO3TrajectoryPoint>
{
   private final YoFrameQuaternion orientation;
   private final YoFrameVector3D angularVelocity;

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

   public void setOrientation(FrameQuaternion orientation)
   {
      this.orientation.set(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void setAngularVelocity(FrameVector3D angularVelocity)
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

   public void set(double time, FrameQuaternion orientation, FrameVector3D angularVelocity)
   {
      this.time.set(time);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, YoFrameQuaternion orientation, YoFrameVector3D angularVelocity)
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
      orientationToPack.set(orientation);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   public void getOrientationIncludingFrame(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }

   public void getAngularVelocityIncludingFrame(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(angularVelocity);
   }

   public void getOrientation(YoFrameQuaternion orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void getAngularVelocity(YoFrameVector3D angularVelocityToPack)
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
   public YoFrameVector3D getAngularVelocity()
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
      frameWaypoint.setOrientation(orientation);
      frameWaypoint.setAngularVelocity(angularVelocity);
   }

   @Override
   public String toString()
   {
      putYoValuesIntoFrameWaypoint();
      return frameWaypoint.toString();
   }
}
