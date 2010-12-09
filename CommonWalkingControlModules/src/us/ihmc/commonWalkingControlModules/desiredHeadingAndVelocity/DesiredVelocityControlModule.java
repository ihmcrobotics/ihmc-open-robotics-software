package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface DesiredVelocityControlModule
{
   public abstract FrameVector2d getDesiredVelocity();

   public abstract FrameVector2d getVelocityErrorInFrame(ReferenceFrame referenceFrame);

   public abstract void updateDesiredVelocity();
}
