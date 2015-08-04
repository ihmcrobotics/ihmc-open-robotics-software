package us.ihmc.commonWalkingControlModules.controlModules.pelvisOrientation;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisOrientationControlModule;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

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
