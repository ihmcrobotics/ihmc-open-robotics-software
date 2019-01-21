package us.ihmc.robotics.geometry.yoWaypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.geometry.interfaces.FrameSO3WaypointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameQuaternion;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameVector3D;

public class YoFrameSO3Waypoint implements FrameSO3WaypointInterface
{
   private final FrameQuaternionBasics orientation;
   private final FrameVector3DBasics angularVelocity;

   public YoFrameSO3Waypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      orientation = new YoMutableFrameQuaternion(namePrefix + "Orientation", nameSuffix, registry);
      angularVelocity = new YoMutableFrameVector3D(namePrefix + "AngularVelocity", nameSuffix, registry);
   }

   @Override
   public void setOrientation(double x, double y, double z, double s)
   {
      orientation.set(x, y, z, s);
   }

   @Override
   public void setAngularVelocity(double x, double y, double z)
   {
      angularVelocity.set(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      orientation.applyTransform(transform);
      angularVelocity.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      orientation.applyInverseTransform(transform);
      angularVelocity.applyInverseTransform(transform);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      orientation.setReferenceFrame(referenceFrame);
      angularVelocity.setReferenceFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      orientation.checkReferenceFrameMatch(angularVelocity);
      return orientation.getReferenceFrame();
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
