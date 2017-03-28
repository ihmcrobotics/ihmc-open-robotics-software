package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface ContactablePlaneBody extends ContactableBody
{
   public abstract void setSoleFrameTransformFromParentJoint(RigidBodyTransform transform);

   public abstract ReferenceFrame getSoleFrame();

   public abstract List<FramePoint2d> getContactPoints2d();
}
