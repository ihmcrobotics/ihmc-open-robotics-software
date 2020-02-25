package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;

/** {@inheritDoc} */
public class AtlasICPOptimizationParameters extends ICPOptimizationParameters
{
   private final boolean runningOnRealRobot;
   private final boolean useAngularMomentum = true;
   private final boolean useStepAdjustment = true;

   public AtlasICPOptimizationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
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
   public double getFootstepRateWeight()
   {
      return runningOnRealRobot ? 4e-9 : 4e-7;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackLateralWeight()
   {
      return runningOnRealRobot ? 1.5 : 1.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackForwardWeight()
   {
      return runningOnRealRobot ? 1.5 : 1.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackRateWeight()
   {
      return 5e-8;
   }

   /** {@inheritDoc} */
   @Override
   public double getCoPCMPFeedbackRateWeight()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public ICPControlGainsReadOnly getICPFeedbackGains()
   {
      ICPControlGains gains = new ICPControlGains();
      gains.setKpOrthogonalToMotion(2.0);
      gains.setKpParallelToMotion(2.5);

      gains.setIntegralLeakRatio(0.97);
      gains.setMaxIntegralError(0.05);
      gains.setKi(1.0);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public double getDynamicsObjectiveWeight()
   {
      return 10000.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDynamicsObjectiveDoubleSupportWeightModifier()
   {
      return 1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getAngularMomentumMinimizationWeight()
   {
      return 10.0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean scaleStepRateWeightWithTime()
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
   public boolean useFeedbackRate()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean allowStepAdjustment()
   {
      return useStepAdjustment;
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
      return 0.001;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useFootstepRate()
   {
      return true;
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
   public boolean getLimitReachabilityFromAdjustment()
   {
      return false;
   }

   @Override
   public double getTransferSplitFraction()
   {
      return 0.2;
   }

   @Override
   public double getMinimumFootstepMultiplier()
   {
      return 0.25;
   }

   @Override
   public boolean useSmartICPIntegrator()
   {
      return true;
   }
}
