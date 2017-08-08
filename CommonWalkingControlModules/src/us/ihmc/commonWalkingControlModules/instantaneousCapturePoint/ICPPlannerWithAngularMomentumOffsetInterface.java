package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ICPPlannerWithAngularMomentumOffsetInterface extends ICPPlannerWithTimeFreezerInterface
{
   void modifyDesiredICPForAngularMomentum(FramePoint copEstimate, RobotSide supportSide);

   void initializeParameters(ICPPlannerParameters plannerParameters, ICPAngularMomentumModifierParameters angularMomentumModifierParameters);
}
