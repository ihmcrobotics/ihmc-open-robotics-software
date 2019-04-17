package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;

public interface ICPPlannerWithTimeFreezerInterface extends ICPPlannerInterface
{
   void compute(FramePoint2DReadOnly currentCapturePointPosition, double time);

   boolean getIsTimeBeingFrozen();
}
