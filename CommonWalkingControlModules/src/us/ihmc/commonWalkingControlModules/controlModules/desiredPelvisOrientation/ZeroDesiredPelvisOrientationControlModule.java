package us.ihmc.commonWalkingControlModules.controlModules.desiredPelvisOrientation;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredPelvisOrientationControlModule;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class ZeroDesiredPelvisOrientationControlModule implements DesiredPelvisOrientationControlModule
{
   private final ReferenceFrame desiredHeadingFrame;

   public ZeroDesiredPelvisOrientationControlModule(ReferenceFrame desiredHeadingFrame)
   {
      this.desiredHeadingFrame = desiredHeadingFrame;
   }

   public FrameOrientation getDesiredPelvisOrientationSingleSupportCopy(RobotSide robotSide)
   {
      return new FrameOrientation(desiredHeadingFrame);
   }

   public FrameOrientation getDesiredPelvisOrientationDoubleSupportCopy()
   {
      return new FrameOrientation(desiredHeadingFrame);
   }

   public void setDesiredPelvisOrientation(FrameOrientation orientation)
   {
      // empty
   }

   public FrameOrientation getEstimatedOrientationAtEndOfStepCopy(RobotSide robotSide, FramePoint desiredFootLocation)
   {
      return new FrameOrientation(desiredHeadingFrame);
   }

   public void useTwistScale(boolean useTwistScale)
   {
   }
}
