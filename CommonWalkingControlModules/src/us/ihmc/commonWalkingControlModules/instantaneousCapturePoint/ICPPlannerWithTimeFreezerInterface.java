package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.robotics.geometry.FramePoint2d;

public interface ICPPlannerWithTimeFreezerInterface extends ICPPlannerInterface
{
   void compute(FramePoint2d currentCapturePointPosition, double time);

   boolean getIsTimeBeingFrozen();

   void initializeParameters(ICPWithTimeFreezingPlannerParameters plannerParameters);
}
