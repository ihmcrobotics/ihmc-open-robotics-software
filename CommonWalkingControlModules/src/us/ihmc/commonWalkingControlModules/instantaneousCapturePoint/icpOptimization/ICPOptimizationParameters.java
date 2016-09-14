package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

public interface ICPOptimizationParameters
{
   public int getMaximumNumberOfFootstepsToConsider();

   public int numberOfFootstepsToConsider();

   public double getFootstepWeight();

   public double getFootstepRegularizationWeight();

   public double getFeedbackWeight();

   public double getFeedbackRegularizationWeight();

   public double getFeedbackGain();

   public double getDynamicRelaxationWeight();

   public boolean scaleStepRegularizationWeightWithTime();

   public boolean scaleFeedbackWeightWithGain();

   public boolean scaleUpcomingStepWeights();

   public boolean useFeedback();

   public boolean useFeedbackRegularization();

   public boolean useStepAdjustment();

   public boolean useFootstepRegularization();

   public boolean useFeedbackWeightHardening();

   public double getMinimumFootstepWeight();

   public double getMinimumFeedbackWeight();

   public double getMinimumTimeRemaining();

   public double getFeedbackWeightHardeningMultiplier();

   public double getMaxCMPExitForward();

   public double getMaxCMPExitSideways();
}
