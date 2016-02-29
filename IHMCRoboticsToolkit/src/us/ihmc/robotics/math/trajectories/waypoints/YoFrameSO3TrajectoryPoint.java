package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSO3TrajectoryPoint extends YoFrameTrajectoryPoint<SimpleSO3TrajectoryPoint, FrameSO3TrajectoryPoint, YoFrameSO3TrajectoryPoint>
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
   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void set(SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      setTime(so3TrajectoryPoint.getTime());
      frameWaypoint.set(so3TrajectoryPoint);
      getYoValuesFromFrameTrajectoryPoint();
   }

   public void set(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      setTime(time);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      setTime(time);
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(angularVelocity);
      this.orientation.set(orientation.getQuaternion());
      this.angularVelocity.set(angularVelocity.getVector());
   }

   public void set(DoubleYoVariable time, YoFrameQuaternion orientation, YoFrameVector angularVelocity)
   {
      setTime(time.getDoubleValue());
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
      angularVelocity.setToNaN();
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
   public void getOrientation(Quat4d orientationToPack)
   {
      orientation.get(orientationToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
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
   protected void putYoValuesIntoFrameTrajectoryPoint()
   {
      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = frameWaypoint.getSimpleWaypoint();
      orientation.get(simpleTrajectoryPoint.getOrientation());
      angularVelocity.get(simpleTrajectoryPoint.getAngularVelocity());
   }

   @Override
   protected void getYoValuesFromFrameTrajectoryPoint()
   {
      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = frameWaypoint.getSimpleWaypoint();
      orientation.set(simpleTrajectoryPoint.getOrientation());
      angularVelocity.set(simpleTrajectoryPoint.getAngularVelocity());
   }
}
