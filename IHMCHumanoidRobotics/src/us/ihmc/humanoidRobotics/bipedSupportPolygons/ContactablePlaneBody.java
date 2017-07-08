package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FramePoint2D;

public interface ContactablePlaneBody extends ContactableBody
{
   public abstract void setSoleFrameTransformFromParentJoint(RigidBodyTransform transform);

   public abstract ReferenceFrame getSoleFrame();

   public abstract List<FramePoint2D> getContactPoints2d();
}
