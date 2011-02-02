package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface DesiredVelocityControlModule
{
   public abstract FrameVector2d getDesiredVelocity();

   public abstract FrameVector2d getVelocityErrorInFrame(ReferenceFrame referenceFrame, RobotSide legToTrustForCoMVelocity);

   public abstract void updateDesiredVelocity();
}
