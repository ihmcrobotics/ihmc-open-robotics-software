package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ICPPlannerWithAngularMomentumOffsetInterface extends ICPPlannerWithTimeFreezerInterface
{
   void modifyDesiredICPForAngularMomentum(FramePoint3D copEstimate, RobotSide supportSide);

   void initializeParameters(ICPPlannerParameters plannerParameters, ICPAngularMomentumModifierParameters angularMomentumModifierParameters);
}
