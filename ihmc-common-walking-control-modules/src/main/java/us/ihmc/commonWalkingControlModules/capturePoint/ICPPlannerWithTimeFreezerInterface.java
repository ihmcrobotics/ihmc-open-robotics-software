package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;

public interface ICPPlannerWithTimeFreezerInterface extends ICPPlannerInterface
{
   void compute(FramePoint2DReadOnly currentCapturePointPosition, double time);

   boolean getIsTimeBeingFrozen();

   void initializeParameters(ICPWithTimeFreezingPlannerParameters plannerParameters);
}
