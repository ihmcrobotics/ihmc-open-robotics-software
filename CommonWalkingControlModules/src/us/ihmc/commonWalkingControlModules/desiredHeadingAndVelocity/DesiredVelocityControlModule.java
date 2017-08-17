package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface DesiredVelocityControlModule
{
   public abstract ReferenceFrame getReferenceFrame();

   public abstract void getDesiredVelocity(FrameVector2D desiredVelocityToPack);

   public abstract void updateDesiredVelocity();
}
