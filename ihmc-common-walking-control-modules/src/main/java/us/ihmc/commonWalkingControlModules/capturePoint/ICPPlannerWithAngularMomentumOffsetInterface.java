package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ICPPlannerWithAngularMomentumOffsetInterface extends ICPPlannerWithTimeFreezerInterface
{
   void modifyDesiredICPForAngularMomentum(FramePoint3D copEstimate, RobotSide supportSide);
}
