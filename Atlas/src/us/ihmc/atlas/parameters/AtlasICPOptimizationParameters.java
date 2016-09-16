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
      return 2.0;
   }

   @Override public double getFootstepRegularizationWeight()
   {
      return 0.01;
   }

   @Override public double getFeedbackWeight()
   {
      return 5.0;
   }

   @Override public double getFeedbackRegularizationWeight()
   {
      return 0.0001;
   }

   @Override public double getFeedbackParallelGain()
   {
      return 20.0;
   }

   @Override public double getFeedbackOrthogonalGain()
   {
      return 20.0;
   }

   @Override public double getDynamicRelaxationWeight()
   {
      return 10000000.0;
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
      return false;
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

   @Override public double getMaxCMPExitForward()
   {
      return 10.0;
   }

   @Override public double getMaxCMPExitSideways()
   {
      return 10.0;
   }
}
