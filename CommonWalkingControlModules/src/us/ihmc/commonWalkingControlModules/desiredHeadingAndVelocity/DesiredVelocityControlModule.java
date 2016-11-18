package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface DesiredVelocityControlModule
{
   public abstract ReferenceFrame getReferenceFrame();

   public abstract void getDesiredVelocity(FrameVector2d desiredVelocityToPack);

   public abstract void updateDesiredVelocity();
}
