package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;

public interface ICPPlannerWithTimeFreezerInterface extends ICPPlannerInterface
{
   void compute(FramePoint2D currentCapturePointPosition, double time);

   boolean getIsTimeBeingFrozen();

   void initializeParameters(ICPWithTimeFreezingPlannerParameters plannerParameters);
}
