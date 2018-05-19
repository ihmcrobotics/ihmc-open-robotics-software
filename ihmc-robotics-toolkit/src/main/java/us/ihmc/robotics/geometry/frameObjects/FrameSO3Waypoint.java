package us.ihmc.robotics.geometry.frameObjects;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;

public class FrameSO3Waypoint extends FrameGeometryObject<FrameSO3Waypoint, SO3Waypoint> implements SO3WaypointInterface<FrameSO3Waypoint>
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

   public void set(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      geometryObject.set(orientation, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      setToZero(referenceFrame);
      geometryObject.set(orientation, angularVelocity);
   }

   public void set(FrameQuaternion orientation, FrameVector3D angularVelocity)
   {
      checkReferenceFrameMatch(orientation);
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.set(orientation, angularVelocity);
   }

   public void setIncludingFrame(FrameQuaternion orientation, FrameVector3D angularVelocity)
   {
      orientation.checkReferenceFrameMatch(angularVelocity);
      setToZero(orientation.getReferenceFrame());
      geometryObject.set(orientation, angularVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, SO3WaypointInterface<?> so3Waypoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(so3Waypoint);
   }

   @Override
   public void setOrientation(QuaternionReadOnly orientation)
   {
      geometryObject.setOrientation(orientation);
   }

   public void setOrientation(FrameQuaternion orientation)
   {
      checkReferenceFrameMatch(orientation);
      geometryObject.setOrientation(orientation);
   }

   @Override
   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      geometryObject.setAngularVelocity(angularVelocity);
   }

   public void setAngularVelocity(FrameVector3D angularVelocity)
   {
      checkReferenceFrameMatch(angularVelocity);
      geometryObject.setAngularVelocity(angularVelocity);
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
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      geometryObject.getOrientation(orientationToPack);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      checkReferenceFrameMatch(orientationToPack);
      geometryObject.getOrientation(orientationToPack);
   }

   public void getOrientationIncludingFrame(FrameQuaternion orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      geometryObject.getOrientation(orientationToPack);
   }

   @Override
   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      checkReferenceFrameMatch(angularVelocityToPack);
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularVelocityIncludingFrame(FrameVector3D angularVelocityToPack)
   {
      setToZero(getReferenceFrame());
      geometryObject.getAngularVelocity(angularVelocityToPack);
   }

   public void get(QuaternionBasics orientationToPack, Vector3DBasics angularVelocityToPack)
   {
      geometryObject.get(orientationToPack, angularVelocityToPack);
   }

   public void get(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   public void getIncludingFrame(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack)
   {
      getOrientationIncludingFrame(orientationToPack);
      getAngularVelocityIncludingFrame(angularVelocityToPack);
   }

   public void get(SO3WaypointInterface<?> so3Waypoint)
   {
      so3Waypoint.setOrientation(geometryObject.getOrientation());
      so3Waypoint.setAngularVelocity(geometryObject.getAngularVelocity());
   }
}
