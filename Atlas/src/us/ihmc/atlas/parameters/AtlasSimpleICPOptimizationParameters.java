package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;

/** {@inheritDoc} */
public class AtlasSimpleICPOptimizationParameters extends ICPOptimizationParameters
{
   private final boolean runningOnRealRobot;
   private final boolean useAngularMomentum = false;

   public AtlasSimpleICPOptimizationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   public boolean useSimpleOptimization()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public int numberOfFootstepsToConsider()
   {
      return 1;
   }

   /** {@inheritDoc} */
   @Override
   public double getForwardFootstepWeight()
   {
      return runningOnRealRobot ? 20.0 : 20.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getLateralFootstepWeight()
   {
      return runningOnRealRobot ? 20.0 : 20.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getFootstepRegularizationWeight()
   {
      return runningOnRealRobot ? 0.001 : 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackLateralWeight()
   {
      return runningOnRealRobot ? 0.5 : 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackForwardWeight()
   {
      return runningOnRealRobot ? 0.5 : 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackRegularizationWeight()
   {
      return runningOnRealRobot ? 0.00001 : 0.00001;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackParallelGain()
   {
      return runningOnRealRobot ? 3.0 : 3.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackOrthogonalGain()
   {
      return runningOnRealRobot ? 2.5 : 2.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getDynamicRelaxationWeight()
   {
      return runningOnRealRobot ? 10000.0 : (useAngularMomentum ? 100000.0 : 1000.0);
   }

   /** {@inheritDoc} */
   @Override
   public double getDynamicRelaxationDoubleSupportWeightModifier()
   {
      if (useAngularMomentum)
         return runningOnRealRobot ? 50.0 : 100.0;
      else
         return runningOnRealRobot ? 1.0 : 4.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getAngularMomentumMinimizationWeight()
   {
      return 10.0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean scaleStepRegularizationWeightWithTime()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean scaleFeedbackWeightWithGain()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean scaleUpcomingStepWeights()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useFeedbackRegularization()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useStepAdjustment()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useAngularMomentum()
   {
      return useAngularMomentum;
   }

   /** {@inheritDoc} */
   @Override
   public double getSafeCoPDistanceToEdge()
   {
      return 0.002;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useTimingOptimization()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useFootstepRegularization()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFootstepWeight()
   {
      return 0.0001;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFeedbackWeight()
   {
      return 0.0001;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumTimeRemaining()
   {
      return 0.0001;
   }

   /** {@inheritDoc} */
   @Override
   public double getAdjustmentDeadband()
   {
      return 0.02;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportSplitFractionForBigAdjustment()
   {
      return runningOnRealRobot ? 0.25 : 0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumTimeOnInitialCMPForBigAdjustment()
   {
      return runningOnRealRobot ? 0.15 : 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getLateralReachabilityOuterLimit()
   {
      return runningOnRealRobot ? 0.5 : 0.85;
   }

   /** {@inheritDoc} */
   @Override
   public double getLateralReachabilityInnerLimit()
   {
      return 0.18;
   }

   /** {@inheritDoc} */
   @Override
   public double getForwardReachabilityLimit()
   {
      return runningOnRealRobot ? 0.65 : 0.9;
   }

   /** {@inheritDoc} */
   @Override
   public double getBackwardReachabilityLimit()
   {
      return runningOnRealRobot ? -0.3 : -0.5;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getLimitReachabilityFromAdjustment()
   {
      return false;
   }
}
