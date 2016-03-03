package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSO3TrajectoryPoint extends FrameTrajectoryPoint<FrameSO3TrajectoryPoint, SimpleSO3TrajectoryPoint>
      implements SO3TrajectoryPointInterface<FrameSO3TrajectoryPoint>
{
   private final SimpleSO3TrajectoryPoint geometryObject;
   
   public FrameSO3TrajectoryPoint()
   {
      super(new SimpleSO3TrajectoryPoint());
      geometryObject = getGeometryObject();
   }

   public FrameSO3TrajectoryPoint(ReferenceFrame referenceFrame)
   {
      this();
      setToZero(referenceFrame);
   }

   public FrameSO3TrajectoryPoint(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      this();
      setIncludingFrame(time, orientation, angularVelocity);
   }

   public FrameSO3TrajectoryPoint(ReferenceFrame referenceFrame, SO3TrajectoryPointInterface<?> so3TrajectoryPointInterface)
   {
      this();
      setIncludingFrame(referenceFrame, so3TrajectoryPointInterface);
   }

   public FrameSO3TrajectoryPoint(FrameSO3TrajectoryPoint frameSO3TrajectoryPoint)
   {
      this();
      setIncludingFrame(frameSO3TrajectoryPoint);
   }

   @Override
   public void setOrientation(Quat4d orientation)
   {
      geometryObject.setOrientation(orientation);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      checkReferenceFrameMatch(orientation);
      geometryObject.setOrientation(orientation.getQuaternion());
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      geometryObject.setAngularVelocity(angularVelocity);
   }

   public void setAngularVelocity(FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.setAngularVelocity(angularVelocity.getVector());
   }

   public void set(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      geometryObject.set(time, orientation, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Quat4d orientation, Vector3d angularVelocity)
   {
      setToZero(referenceFrame);
      geometryObject.set(time, orientation, angularVelocity);
   }

   public void set(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.set(time, orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void setIncludingFrame(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      orientation.checkReferenceFrameMatch(angularVelocity);
      setToZero(orientation.getReferenceFrame());
      geometryObject.set(time, orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void set(double time, SO3WaypointInterface<?> so3Waypoint)
   {
      geometryObject.set(time, so3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, SO3WaypointInterface<?> so3Waypoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(time, so3Waypoint);
   }

   public void set(SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      geometryObject.set(so3TrajectoryPoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(so3TrajectoryPoint);
   }

   public void set(double time, FrameSO3Waypoint frameSO3Waypoint)
   {
      checkReferenceFrameMatch(frameSO3Waypoint);
      frameSO3Waypoint.get(geometryObject);
   }

   public void setIncludingFrame(double time, FrameSO3Waypoint frameSO3Waypoint)
   {
      setToZero(getReferenceFrame());
      frameSO3Waypoint.get(geometryObject);
   }

   @Override
   public void setOrientationToZero()
   {
      geometryObject.setOrientationToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      geometryObject.setAngularVelocityToZero();
   }

   @Override
   public void setOrientationToNaN()
   {
      geometryObject.setOrientationToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      geometryObject.setAngularVelocityToNaN();
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      geometryObject.getOrientation(orientationToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      geometryObject.getOrientation(orientationToPack.getQuaternion());
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      geometryObject.getOrientation(orientationToPack.getQuaternion());
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      geometryObject.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(getReferenceFrame());
      geometryObject.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public double get(Quat4d orientationToPack, Vector3d angularVelocityToPack)
   {
      return geometryObject.get(orientationToPack, angularVelocityToPack);
   }

   public double get(FrameOrientation orientationToPack, FrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      checkReferenceFrameMatch(angularVelocityToPack);
      return geometryObject.get(orientationToPack.getQuaternion(), angularVelocityToPack.getVector());
   }

   public double getIncludingFrame(FrameOrientation orientationToPack, FrameVector angularVelocityToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      angularVelocityToPack.setToZero(getReferenceFrame());
      return geometryObject.get(orientationToPack.getQuaternion(), angularVelocityToPack.getVector());
   }

   public double get(SO3WaypointInterface<?> so3Waypoint)
   {
      return geometryObject.get(so3Waypoint);
   }

   public double get(SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      so3TrajectoryPoint.setOrientation(geometryObject.getOrientation());
      so3TrajectoryPoint.setAngularVelocity(geometryObject.getAngularVelocity());
      return getTime();
   }

   public double get(YoFrameQuaternion orientationToPack, YoFrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      checkReferenceFrameMatch(angularVelocityToPack);
      orientationToPack.set(geometryObject.getOrientation());
      angularVelocityToPack.set(geometryObject.getAngularVelocity());
      return getTime();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());

      return "SO3 trajectory point: (" + timeToString + ", " + geometryObject + getReferenceFrame() + ")";
   }
}
