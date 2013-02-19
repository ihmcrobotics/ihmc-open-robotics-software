package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.utilities.math.geometry.FramePoint2d;

public interface FinalDesiredICPCalculator
{
   public abstract FramePoint2d getFinalDesiredICPForWalking(TransferToAndNextFootstepsData transferToAndNextFootstepsData);
}
