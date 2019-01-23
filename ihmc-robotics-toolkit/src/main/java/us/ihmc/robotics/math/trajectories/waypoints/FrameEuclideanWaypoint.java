package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;

public class FrameEuclideanWaypoint implements FrameEuclideanWaypointBasics
{
   private final FramePoint3D position = new FramePoint3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();

   public FrameEuclideanWaypoint()
   {
   }

   public FrameEuclideanWaypoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   @Override
   public void setLinearVelocity(double x, double y, double z)
   {
      linearVelocity.set(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      position.applyTransform(transform);
      linearVelocity.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      position.applyInverseTransform(transform);
      linearVelocity.applyInverseTransform(transform);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      position.setReferenceFrame(referenceFrame);
      linearVelocity.setReferenceFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      position.checkReferenceFrameMatch(linearVelocity);
      return position.getReferenceFrame();
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return position;
   }

   @Override
   public FrameVector3DReadOnly getLinearVelocity()
   {
      return linearVelocity;
   }
}
