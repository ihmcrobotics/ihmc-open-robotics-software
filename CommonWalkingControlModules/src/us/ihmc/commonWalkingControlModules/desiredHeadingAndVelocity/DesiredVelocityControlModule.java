package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.utilities.math.geometry.FrameVector2d;

public interface DesiredVelocityControlModule
{
   public abstract FrameVector2d getDesiredVelocity();

   public abstract void updateDesiredVelocity();
}
