package us.ihmc.steppr.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

/** {@inheritDoc} */
public class BonoCapturePointPlannerParameters implements CapturePointPlannerParameters
{
   private final boolean runningOnRealRobot;
   // TODO Try using new ICP planner with two CMPs.
   private final boolean useTwoCMPsPerSupport;

   public BonoCapturePointPlannerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      useTwoCMPsPerSupport = false;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportInitialTransferDuration()
   {
      return 1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportDuration()
   {
      return runningOnRealRobot ? 0.5 : 0.25;
   }

   /** {@inheritDoc} */
   @Override
   public double getAdditionalTimeForSingleSupport()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getSingleSupportDuration()
   {
      return runningOnRealRobot ? 1.0 : 0.7;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportSplitFraction()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getFreezeTimeFactor()
   {
      return 0.9;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return 0.02;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getDoTimeFreezing()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxAllowedErrorWithoutPartialTimeFreeze()
   {
      return 0.03;
   }

   /** {@inheritDoc} */
   @Override
   public double getEntryCMPInsideOffset()
   {
      return 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getExitCMPInsideOffset()
   {
      return 0.025;
   }

   /** {@inheritDoc} */
   @Override
   public double getEntryCMPForwardOffset()
   {
      return -0.00;//0.00;//0.02-.025/2;
   }

   /** {@inheritDoc} */
   @Override
   public double getExitCMPForwardOffset()
   {
      return -0.00;//0.00;//0.02-.025/2;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useTwoCMPsPerSupport()
   {
      return useTwoCMPsPerSupport;
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeSpentOnExitCMPInPercentOfStepTime()
   {
      return 0.50;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxEntryCMPForwardOffset()
   {
      return 0.03;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinEntryCMPForwardOffset()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxExitCMPForwardOffset()
   {
      return 0.08;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinExitCMPForwardOffset()
   {
      return -0.04;
   }

   /** {@inheritDoc} */
   @Override
   public double getCMPSafeDistanceAwayFromSupportEdges()
   {
      return 0.03;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxDurationForSmoothingEntryToExitCMPSwitch()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthToCMPOffsetFactor()
   {
      return 1.0 / 3.0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useExitCMPOnToesForSteppingDown()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthThresholdForExitCMPOnToesWhenSteppingDown()
   {
      return 0.15;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepHeightThresholdForExitCMPOnToesWhenSteppingDown()
   {
      return 0.10;
   }

   /** {@inheritDoc} */
   @Override
   public double getCMPSafeDistanceAwayFromToesWhenSteppingDown()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinTimeToSpendOnExitCMPInSingleSupport()
   {
      return 0.0;
   }
}
