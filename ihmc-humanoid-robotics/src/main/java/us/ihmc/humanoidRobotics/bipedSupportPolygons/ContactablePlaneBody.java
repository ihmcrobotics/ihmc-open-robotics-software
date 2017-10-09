package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

public interface ContactablePlaneBody extends ContactableBody
{
   public abstract void setSoleFrameTransformFromParentJoint(RigidBodyTransform transform);

   public abstract ReferenceFrame getSoleFrame();

   public abstract List<FramePoint2D> getContactPoints2d();
}
