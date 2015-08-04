package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class InstantaneousCapturePointPlannerWithTimeFreezer implements InstantaneousCapturePointPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable timeDelay = new DoubleYoVariable("timeDelay", registry);
   private final DoubleYoVariable icpError = new DoubleYoVariable("icpError", registry);
   private final DoubleYoVariable maxICPErrorForStartingSwing = new DoubleYoVariable("maxICPErrorForStartingSwing", registry);
   
   private final DoubleYoVariable icpDistanceToFreezeLine = new DoubleYoVariable("icpDistanceToFreezeLine", registry);
   
   private final DoubleYoVariable previousTime = new DoubleYoVariable("previousTime", registry);
   private final DoubleYoVariable freezeTimeFactor = new DoubleYoVariable("freezeTimeFactor", "Set to 0.0 to turn off, 1.0 to completely freeze time", registry);
   private final double maxFreezeLineICPErrorWithoutTimeFreeze = 0.03; 
   private final BooleanYoVariable isTimeBeingFrozen = new BooleanYoVariable("oldIcpPlannerIsTimeBeingFrozen", registry);

   private final FrameVector2d normalizedVelocityVector;
   private final FrameVector2d vectorFromDesiredToActualICP;
   private final FrameVector2d deltaICP;
   
   private final InstantaneousCapturePointPlanner instantaneousCapturePointPlanner;
   
   public InstantaneousCapturePointPlannerWithTimeFreezer(InstantaneousCapturePointPlanner instantaneousCapturePointPlanner, YoVariableRegistry parentRegistry)
   {
      this.instantaneousCapturePointPlanner = instantaneousCapturePointPlanner;
      
      normalizedVelocityVector = new FrameVector2d(ReferenceFrame.getWorldFrame());
      vectorFromDesiredToActualICP = new FrameVector2d(ReferenceFrame.getWorldFrame());
      deltaICP = new FrameVector2d(ReferenceFrame.getWorldFrame());
      
      parentRegistry.addChild(registry);
      isTimeBeingFrozen.set(false);
      timeDelay.set(0.0);
      freezeTimeFactor.set(0.9); 
      maxICPErrorForStartingSwing.set(0.035); 
   }
   
   public void initializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      instantaneousCapturePointPlanner.initializeSingleSupport(transferToAndNextFootstepsData, initialTime);
      resetTiming(initialTime);
   }

   public void reInitializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double currentTime)
   {
      instantaneousCapturePointPlanner.reInitializeSingleSupport(transferToAndNextFootstepsData, currentTime);
   }

   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point2d initialICPPosition,
         double initialTime)
   {
      instantaneousCapturePointPlanner.initializeDoubleSupportInitialTransfer(transferToAndNextFootstepsData, initialICPPosition, initialTime);
      resetTiming(initialTime);
   }

   public void initializeDoubleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      instantaneousCapturePointPlanner.initializeDoubleSupport(transferToAndNextFootstepsData, initialTime);
      resetTiming(initialTime);
   }

   
   public void getICPPositionAndVelocity(FramePoint2d icpPostionToPack, FrameVector2d icpVelocityToPack, FramePoint2d ecmpToPack, FramePoint2d actualICP,
         double time)
   {
      instantaneousCapturePointPlanner.getICPPositionAndVelocity(icpPostionToPack, icpVelocityToPack, ecmpToPack, actualICP, getTimeWithDelay(time));    

      icpError.set(icpPostionToPack.distance(actualICP));
      icpDistanceToFreezeLine.set(computeDistanceFromFreezeLine(icpPostionToPack, icpVelocityToPack, actualICP));

      if (this.isDone(time))
      {
         freezeTime(time, 1.0);
         isTimeBeingFrozen.set(true);
      }
      else if ((getEstimatedTimeRemainingForState(time) < 0.1) && 
            (instantaneousCapturePointPlanner.isPerformingICPDoubleSupport()) && 
            (icpDistanceToFreezeLine.getDoubleValue() > maxICPErrorForStartingSwing.getDoubleValue()))
      {
         freezeTime(time, 1.0);
         isTimeBeingFrozen.set(true);
      }

      else if ((icpDistanceToFreezeLine.getDoubleValue() > maxFreezeLineICPErrorWithoutTimeFreeze))
      {
         freezeTime(time, freezeTimeFactor.getDoubleValue());
         isTimeBeingFrozen.set(true);
      }
      else
      {
         isTimeBeingFrozen.set(false);
      }
      
      previousTime.set(time);
   }
   
   private double computeDistanceFromFreezeLine(FramePoint2d icpPostionToPack, FrameVector2d icpVelocityToPack, FramePoint2d actualICP)
   {
      normalizedVelocityVector.setIncludingFrame(icpVelocityToPack);
      //If the icp velocity is zero, this normalize will return NaN. In this 
      //case, the comparison with another double will return false.
      normalizedVelocityVector.normalize();
            
      vectorFromDesiredToActualICP.setIncludingFrame(actualICP);
      vectorFromDesiredToActualICP.sub(icpPostionToPack);
      
      double distance = vectorFromDesiredToActualICP.dot(normalizedVelocityVector);
      
      deltaICP.setIncludingFrame(normalizedVelocityVector);
      deltaICP.scale(distance);
      
      return -distance;
   }

   private void freezeTime(double time, double freezeTimeFactor)
   {      
      double timeInState = instantaneousCapturePointPlanner.getTimeInState(getTimeWithDelay(time));
      if (timeInState < 0.0) return;
      
      timeDelay.add(freezeTimeFactor * (time - previousTime.getDoubleValue()));
   }

   public void reset(double time)
   {
      instantaneousCapturePointPlanner.reset(time);  
      resetTiming(time);
   }

   public boolean isDone(double time)
   {      
      if (instantaneousCapturePointPlanner.isPerformingICPDoubleSupport())
      {
         return instantaneousCapturePointPlanner.isDone(getTimeWithDelay(time));
      }
      else
      {
         return instantaneousCapturePointPlanner.isDone(getTimeWithDelay(time));
      }
   }

   public double getEstimatedTimeRemainingForState(double time)
   {
      return instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(getTimeWithDelay(time));
   }

   public boolean isPerformingICPDoubleSupport()
   {
      return instantaneousCapturePointPlanner.isPerformingICPDoubleSupport();
   }

   public FramePoint2d getFinalDesiredICP()
   {
      return instantaneousCapturePointPlanner.getFinalDesiredICP();
   }
   
   public double getTimeInState(double time)
   {
      return instantaneousCapturePointPlanner.getTimeInState(getTimeWithDelay(time));
   }
   
   private void resetTiming(double initialTime)
   {
      timeDelay.set(0.0);
      previousTime.set(initialTime);
   }
   
   private double getTimeWithDelay(double time)
   {
      return time - timeDelay.getDoubleValue();
   }

   public void setDoHeelToToeTransfer(boolean doHeelToToeTransfer)
   {
      instantaneousCapturePointPlanner.setDoHeelToToeTransfer(doHeelToToeTransfer);
   }

   @Override
   public FramePoint2d getConstantCenterOfPressure()
   {
      // TODO Auto-generated method stub
      return instantaneousCapturePointPlanner.getConstantCenterOfPressure();
   }

   @Override
   public FramePoint2d getSingleSupportStartICP()
   {
      // TODO Auto-generated method stub
      return instantaneousCapturePointPlanner.getSingleSupportStartICP();
   }

   @Override
   public void updatePlanForSingleSupportPush(TransferToAndNextFootstepsData transferToAndNextFootstepsData, FramePoint actualCapturePointPosition,double time)
   {
      throw new RuntimeException("Not implemented.");      
   }

   @Override
   public void updatePlanForDoubleSupportPush(TransferToAndNextFootstepsData transferToAndNextFootstepsData, FramePoint actualCapturePointPosition, double time)
   {
      throw new RuntimeException("Not implemented.");
   }
}
