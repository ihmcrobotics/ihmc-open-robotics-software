package us.ihmc.robotics.geometry.frameObjects;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.AbstractFrameObject;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSO3Waypoint extends AbstractFrameObject<FrameSO3Waypoint, SO3Waypoint> implements SO3WaypointInterface<FrameSO3Waypoint>
{
   private final SO3Waypoint geometryObject;
   
   public FrameSO3Waypoint()
   {
      super(new SO3Waypoint());
      geometryObject = getGeometryObject();
   }

   public FrameSO3Waypoint(ReferenceFrame referenceFrame)
   {
     this();
     this.referenceFrame = referenceFrame;
   }

   public void set(Quat4d orientation, Vector3d angularVelocity)
   {
      geometryObject.set(orientation, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Quat4d orientation, Vector3d angularVelocity)
   {
      setToZero(referenceFrame);
      geometryObject.set(orientation, angularVelocity);
   }

   public void set(FrameOrientation orientation, FrameVector angularVelocity)
   {
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.set(orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void setIncludingFrame(FrameOrientation orientation, FrameVector angularVelocity)
   {
      orientation.checkReferenceFrameMatch(angularVelocity);
      setToZero(orientation.getReferenceFrame());
      geometryObject.set(orientation.getQuaternion(), angularVelocity.getVector());
   }

   public void set(FrameSO3Waypoint so3Waypoint)
   {
      checkReferenceFrameMatch(so3Waypoint);
      geometryObject.set(so3Waypoint);
   }
   
   public void set(SO3Waypoint so3Waypoint)
   {
      geometryObject.set(so3Waypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SO3WaypointInterface<?> so3Waypoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(so3Waypoint);
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
      setToZero(getReferenceFrame());
      geometryObject.getAngularVelocity(angularVelocityToPack.getVector());
   }

   public void get(Quat4d orientationToPack, Vector3d angularVelocityToPack)
   {
      geometryObject.get(orientationToPack, angularVelocityToPack);
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
      so3Waypoint.setOrientation(geometryObject.getOrientation());
      so3Waypoint.setAngularVelocity(geometryObject.getAngularVelocity());
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
