package us.ihmc.commonWalkingControlModules.controlModules.pelvisOrientation;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisOrientationControlModule;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;

public class DoNothingPelvisOrientationControlModule implements PelvisOrientationControlModule
{
   public DoNothingPelvisOrientationControlModule(ReferenceFrame pelvisFrame)
   {
      this.pelvisFrame = pelvisFrame;
   }

   private final ReferenceFrame pelvisFrame;

   public FrameVector computePelvisTorque(RobotSide supportLeg, FrameOrientation desiredPelvisOrientation)
   {
      return new FrameVector(pelvisFrame);
   }

}
