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
   public FrameSO3TrajectoryPoint()
   {
      super(new SimpleSO3TrajectoryPoint());
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
      simpleWaypoint.setOrientation(orientation);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      checkReferenceFrameMatch(orientation);
      simpleWaypoint.setOrientation(orientation.getQuaternion());
   }

   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      simpleWaypoint.setAngularVelocity(angularVelocity);
   }

   public void setAngularVelocity(FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      simpleWaypoint.setAngularVelocity(angularVelocity.getVector());
   }

   public void set(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      simpleWaypoint.set(time, orientation, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Quat4d orientation, Vector3d angularVelocity)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(time, orientation, angularVelocity);
   }

   public void set(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(angularVelocity);
      simpleWaypoint.set(time, orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void setIncludingFrame(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      orientation.checkReferenceFrameMatch(angularVelocity);
      setToZero(orientation.getReferenceFrame());
      simpleWaypoint.set(time, orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void set(double time, SO3WaypointInterface<?> so3Waypoint)
   {
      simpleWaypoint.set(time, so3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, SO3WaypointInterface<?> so3Waypoint)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(time, so3Waypoint);
   }

   public void set(SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      simpleWaypoint.set(so3TrajectoryPoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(so3TrajectoryPoint);
   }

   public void set(double time, FrameSO3Waypoint frameSO3Waypoint)
   {
      checkReferenceFrameMatch(frameSO3Waypoint);
      frameSO3Waypoint.get(simpleWaypoint);
   }

   public void setIncludingFrame(double time, FrameSO3Waypoint frameSO3Waypoint)
   {
      setToZero(getReferenceFrame());
      frameSO3Waypoint.get(simpleWaypoint);
   }

   @Override
   public void setOrientationToZero()
   {
      simpleWaypoint.setOrientationToZero();
   }

   @Override
   public void setAngularVelocityToZero()
   {
      simpleWaypoint.setAngularVelocityToZero();
   }

   @Override
   public void setOrientationToNaN()
   {
      simpleWaypoint.setOrientationToNaN();
   }

   @Override
   public void setAngularVelocityToNaN()
   {
      simpleWaypoint.setAngularVelocityToNaN();
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      simpleWaypoint.getOrientation(orientationToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      simpleWaypoint.getOrientation(orientationToPack.getQuaternion());
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getOrientation(orientationToPack.getQuaternion());
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      simpleWaypoint.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      simpleWaypoint.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public double get(Quat4d orientationToPack, Vector3d angularVelocityToPack)
   {
      return simpleWaypoint.get(orientationToPack, angularVelocityToPack);
   }

   public double get(FrameOrientation orientationToPack, FrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      checkReferenceFrameMatch(angularVelocityToPack);
      return simpleWaypoint.get(orientationToPack.getQuaternion(), angularVelocityToPack.getVector());
   }

   public double getIncludingFrame(FrameOrientation orientationToPack, FrameVector angularVelocityToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      angularVelocityToPack.setToZero(getReferenceFrame());
      return simpleWaypoint.get(orientationToPack.getQuaternion(), angularVelocityToPack.getVector());
   }

   public double get(SO3WaypointInterface<?> so3Waypoint)
   {
      return simpleWaypoint.get(so3Waypoint);
   }

   public double get(SO3TrajectoryPointInterface<?> so3TrajectoryPoint)
   {
      so3TrajectoryPoint.setOrientation(simpleWaypoint.getOrientation());
      so3TrajectoryPoint.setAngularVelocity(simpleWaypoint.getAngularVelocity());
      return getTime();
   }

   public double get(YoFrameQuaternion orientationToPack, YoFrameVector angularVelocityToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      checkReferenceFrameMatch(angularVelocityToPack);
      orientationToPack.set(simpleWaypoint.getOrientation());
      angularVelocityToPack.set(simpleWaypoint.getAngularVelocity());
      return getTime();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());

      return "SO3 trajectory point: (" + timeToString + ", " + simpleWaypoint + getReferenceFrame() + ")";
   }
}
