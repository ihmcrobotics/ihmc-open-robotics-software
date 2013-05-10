package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public interface InstantaneousCapturePointPlanner
{
   public void initializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime);

   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point2d initialICPPosition,
           double initialTime);

   public void initializeDoubleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime);

   public void getICPPositionAndVelocity(FramePoint2d icpPostionToPack, FrameVector2d icpVelocityToPack, FramePoint2d ecmpToPack, double time);

}
