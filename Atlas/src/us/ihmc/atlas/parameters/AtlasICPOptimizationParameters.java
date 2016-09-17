package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;

public class AtlasICPOptimizationParameters implements ICPOptimizationParameters
{
   @Override public int getMaximumNumberOfFootstepsToConsider()
   {
      return 5;
   }

   @Override public int numberOfFootstepsToConsider()
   {
      return 1;
   }

   @Override public double getFootstepWeight()
   {
      return 15.0;
   }

   @Override public double getFootstepRegularizationWeight()
   {
      return 0.0001;
   }

   @Override public double getFeedbackWeight()
   {
      return 1.0;
   }

   @Override public double getFeedbackRegularizationWeight()
   {
      return 0.00005;
   }

   @Override public double getFeedbackParallelGain()
   {
      return 10.0;
   }

   @Override public double getFeedbackOrthogonalGain()
   {
      return 10.0;
   }

   @Override public double getDynamicRelaxationWeight()
   {
      return 500.0;
   }

   @Override public double getDynamicRelaxationDoubleSupportWeightModifier()
   {
      return 5.0;
   }

   @Override public boolean scaleStepRegularizationWeightWithTime()
   {
      return true;
   }

   @Override public boolean scaleFeedbackWeightWithGain()
   {
      return true;
   }

   @Override public boolean scaleUpcomingStepWeights()
   {
      return true;
   }

   @Override public boolean useFeedback()
   {
      return true;
   }

   @Override public boolean useFeedbackRegularization()
   {
      return true;
   }

   @Override public boolean useStepAdjustment()
   {
      return true;
   }

   @Override public boolean useFootstepRegularization()
   {
      return true;
   }

   @Override public boolean useFeedbackWeightHardening()
   {
      return false;
   }

   @Override public boolean useICPFromBeginningOfState()
   {
      return true;
   }

   @Override public double getMinimumFootstepWeight()
   {
      return 0.0001;
   }

   @Override public double getMinimumFeedbackWeight()
   {
      return 0.0001;
   }

   @Override public double getMinimumTimeRemaining()
   {
      return 0.001;
   }

   @Override public double getFeedbackWeightHardeningMultiplier()
   {
      return 20.0;
   }

   @Override public double getMaxCMPForwardExit()
   {
      return 0.01;
   }

   @Override public double getMaxCMPLateralExit()
   {
      return 0.015;
   }

   @Override public double getForwardAdjustmentDeadband()
   {
      return 0.02;
   }

   @Override public double getLateralAdjustmentDeadband()
   {
      return 0.02;
   }

   @Override public double getRemainingTimeToStopAdjusting()
   {
      return 0.05;
   }
}
