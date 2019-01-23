package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointInterface;

public class FrameSO3Waypoint implements FrameSO3WaypointInterface
{
   private final FrameQuaternion orientation = new FrameQuaternion();
   private final FrameVector3D angularVelocity = new FrameVector3D();

   public FrameSO3Waypoint()
   {
   }

   public FrameSO3Waypoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
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
}
