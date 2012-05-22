package us.ihmc.commonWalkingControlModules.controlModules.desiredPelvisOrientation;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredPelvisOrientationControlModule;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ZeroDesiredPelvisOrientationControlModule implements DesiredPelvisOrientationControlModule
{
   private final ReferenceFrame desiredHeadingFrame;

   public ZeroDesiredPelvisOrientationControlModule(ReferenceFrame desiredHeadingFrame)
   {
      this.desiredHeadingFrame = desiredHeadingFrame;
   }

   public Orientation getDesiredPelvisOrientationSingleSupport(RobotSide robotSide)
   {
      return new Orientation(desiredHeadingFrame);
   }

   public Orientation getDesiredPelvisOrientationDoubleSupport()
   {
      return new Orientation(desiredHeadingFrame);
   }

   public void setDesiredPelvisOrientation(Orientation orientation)
   {
      // empty
   }

   public Orientation getEstimatedOrientationAtEndOfStep(RobotSide robotSide, FramePoint desiredFootLocation)
   {
      return new Orientation(desiredHeadingFrame);
   }

   public void useTwistScale(boolean useTwistScale)
   {
   }
}
