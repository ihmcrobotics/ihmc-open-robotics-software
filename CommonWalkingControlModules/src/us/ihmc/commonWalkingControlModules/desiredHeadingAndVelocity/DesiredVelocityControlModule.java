package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.robotics.geometry.FrameVector2D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface DesiredVelocityControlModule
{
   public abstract ReferenceFrame getReferenceFrame();

   public abstract void getDesiredVelocity(FrameVector2D desiredVelocityToPack);

   public abstract void updateDesiredVelocity();
}
