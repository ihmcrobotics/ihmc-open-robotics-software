package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

public class AtlasCapturePointPlannerParameters implements CapturePointPlannerParameters
{
   private boolean runningOnRealRobot;
   private final boolean useTwoCMPsPerSupport;
   private final boolean useNewICPPlanner;

   public AtlasCapturePointPlannerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      useTwoCMPsPerSupport = runningOnRealRobot;
      useNewICPPlanner = runningOnRealRobot;
   }

   @Override
   public double getDoubleSupportInitialTransferDuration()
   {
      return runningOnRealRobot ? 2.0 : 1.0;
   }

   @Override
   public double getDoubleSupportDuration()
   {
      return runningOnRealRobot ? 1.5 : 0.25;
   }

   @Override
   public double getAdditionalTimeForSingleSupport()
   {
      return 0.1;
   }

   @Override
   public double getSingleSupportDuration()
   {
      return runningOnRealRobot ? 1.5 : 0.7;
   }

   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   @Override
   public int getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory()
   {
      // Using higher order for the trajectory degrade the pseudo CMP trajectory (going a lot outside the support polygon) when using two CMPs per support.
      return useTwoCMPsPerSupport ? 4 : 5;
   }

   @Override
   public int getNumberOfFootstepsToStop()
   {
      return 2;
   }

   @Override
   public double getIsDoneTimeThreshold()
   {
      return -1e-4;
   }

   @Override
   public double getDoubleSupportSplitFraction()
   {
      return 0.5;
   }

   @Override
   public double getFreezeTimeFactor()
   {
      return 0.9;
   }

   @Override
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return 0.025; //0.035;
   }

   @Override
   public boolean getDoTimeFreezing()
   {
      return true;
   }

   @Override
   public boolean getDoFootSlipCompensation()
   {
      return true;
   }

   @Override
   public double getAlphaDeltaFootPositionForFootslipCompensation()
   {
      return 0.65;
   }

   @Override
   public double getEntryCMPInsideOffset()
   {
      return -0.005; //0.006;
   }

   @Override
   public double getExitCMPInsideOffset()
   {
      return 0.025;
   }

   @Override
   public double getEntryCMPForwardOffset()
   {
      return 0.0;
   }

   @Override
   public double getExitCMPForwardOffset()
   {
      return 0.0;
   }

   @Override
   public double getMaxAllowedErrorWithoutPartialTimeFreeze()
   {
      return 0.03;
   }

   @Override
   public boolean useTerribleHackToReduceICPVelocityAtTheEndOfTransfer()
   {
      return false;
   }

   @Override
   public boolean useNewICPPlanner()
   {
      return useNewICPPlanner;
   }

   @Override
   public boolean useTwoCMPsPerSupport()
   {
      return useTwoCMPsPerSupport;
   }

   @Override
   public double getTimeSpentOnExitCMPInPercentOfStepTime()
   {
      return 0.50;
   }

   @Override
   public double getMaxEntryCMPForwardOffset()
   {
      return 0.03;
   }

   @Override
   public double getMinEntryCMPForwardOffset()
   {
      return 0.0;
   }

   @Override
   public double getMaxExitCMPForwardOffset()
   {
      return 0.06;
   }

   @Override
   public double getMinExitCMPForwardOffset()
   {
      return -0.04;
   }

   @Override
   public double getCMPSafeDistanceAwayFromSupportEdges()
   {
      return 0.01;
   }
}
