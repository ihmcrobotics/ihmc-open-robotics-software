package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSO3Waypoint extends FrameWaypoint<FrameSO3Waypoint, SimpleSO3Waypoint> implements SO3WaypointInterface<FrameSO3Waypoint>
{
   public FrameSO3Waypoint()
   {
      super(new SimpleSO3Waypoint());
   }

   public void set(Quat4d orientation, Vector3d angularVelocity)
   {
      simpleWaypoint.set(orientation, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Quat4d orientation, Vector3d angularVelocity)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(orientation, angularVelocity);
   }

   public void set(FrameOrientation orientation, FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(angularVelocity);
      simpleWaypoint.set(orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void setIncludingFrame(FrameOrientation orientation, FrameVector angularVelocity)
   {
      orientation.checkReferenceFrameMatch(angularVelocity);
      setToZero(orientation.getReferenceFrame());
      simpleWaypoint.set(orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void set(SO3WaypointInterface<?> so3Waypoint)
   {
      simpleWaypoint.set(so3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SO3WaypointInterface<?> so3Waypoint)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(so3Waypoint);
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
      setToZero(getReferenceFrame());
      simpleWaypoint.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public void get(Quat4d orientationToPack, Vector3d angularVelocityToPack)
   {
      simpleWaypoint.get(orientationToPack, angularVelocityToPack);
   }

   public void get(FrameOrientation orientationToPack, FrameVector angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   public void getIncludingFrame(FrameOrientation orientationToPack, FrameVector angularVelocityToPack)
   {
      getOrientationIncludingFrame(orientationToPack);
      getAngularVelocityIncludingFrame(angularVelocityToPack);
   }

   public void get(SO3WaypointInterface<?> so3Waypoint)
   {
      so3Waypoint.setOrientation(simpleWaypoint.getOrientation());
      so3Waypoint.setAngularVelocity(simpleWaypoint.getAngularVelocity());
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
