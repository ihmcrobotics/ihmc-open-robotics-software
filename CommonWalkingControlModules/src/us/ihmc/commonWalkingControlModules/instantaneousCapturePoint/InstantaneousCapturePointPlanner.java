package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;

public interface InstantaneousCapturePointPlanner
{
   public abstract void initializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime);

   public abstract void reInitializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double currentTime);

   public abstract void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point2d initialICPPosition,
           double initialTime);

   public abstract void initializeDoubleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime);

   public abstract void getICPPositionAndVelocity(FramePoint2d icpPostionToPack, FrameVector2d icpVelocityToPack, FramePoint2d ecmpToPack, 
         FramePoint2d actualICP, double time);

   public abstract void reset(double time);

   public abstract boolean isDone(double time);
   
   public abstract double getEstimatedTimeRemainingForState(double time);
   
   public abstract boolean isPerformingICPDoubleSupport();

   public abstract FramePoint2d getFinalDesiredICP();
   
   public abstract FramePoint2d getConstantCenterOfPressure();
   
   public abstract FramePoint2d getSingleSupportStartICP();

   public abstract double getTimeInState(double time);

   public abstract void setDoHeelToToeTransfer(boolean doHeelToToeTransfer);
   
   public abstract void updatePlanForSingleSupportPush(TransferToAndNextFootstepsData transferToAndNextFootstepsData, FramePoint actualCapturePointPosition, double time);
   
   public abstract void updatePlanForDoubleSupportPush(TransferToAndNextFootstepsData transferToAndNextFootstepsData, FramePoint actualCapturePointPosition,
         double time);
}
