package us.ihmc.commonWalkingControlModules.controlModules.desiredPelvisOrientation;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredPelvisOrientationControlModule;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ZeroDesiredPelvisOrientationControlModule implements DesiredPelvisOrientationControlModule
{
   private final ReferenceFrame desiredHeadingFrame;

   public ZeroDesiredPelvisOrientationControlModule(ReferenceFrame desiredHeadingFrame)
   {
      this.desiredHeadingFrame = desiredHeadingFrame;
   }

   public FrameOrientation getDesiredPelvisOrientationSingleSupport(RobotSide robotSide)
   {
      return new FrameOrientation(desiredHeadingFrame);
   }

   public FrameOrientation getDesiredPelvisOrientationDoubleSupport()
   {
      return new FrameOrientation(desiredHeadingFrame);
   }

   public void setDesiredPelvisOrientation(FrameOrientation orientation)
   {
      // empty
   }

   public FrameOrientation getEstimatedOrientationAtEndOfStep(RobotSide robotSide, FramePoint desiredFootLocation)
   {
      return new FrameOrientation(desiredHeadingFrame);
   }

   public void useTwistScale(boolean useTwistScale)
   {
   }
}
