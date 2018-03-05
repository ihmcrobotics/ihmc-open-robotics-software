package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;

/** {@inheritDoc} */
public class AtlasICPOptimizationParameters extends ICPOptimizationParameters
{
   private final boolean runningOnRealRobot;
   private final boolean useAngularMomentum = false;
   private final boolean useStepAdjustment = false;

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
      return runningOnRealRobot ? 0.001 : 0.001;
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
   public double getFeedbackRateWeight()
   {
      return runningOnRealRobot ? 1e-5 : 1e-9;
   }

   /** {@inheritDoc} */
   @Override
   public ICPControlGainsReadOnly getICPFeedbackGains()
   {
      ICPControlGains gains = new ICPControlGains();
      gains.setKpOrthogonalToMotion(1.5);
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
      if (runningOnRealRobot)
         return 10000.0;
      else if (useAngularMomentum)
         return 100000.0;
      else if (useStepAdjustment)
         return 1000.0;
      else
         return 10000.0;
      //return runningOnRealRobot ? 10000.0 : (useAngularMomentum ? 100000.0 : 1000.0);
   }

   /** {@inheritDoc} */
   @Override
   public double getDynamicsObjectiveDoubleSupportWeightModifier()
   {
      if (useAngularMomentum)
         return runningOnRealRobot ? 50.0 : 100.0;
      else if (useStepAdjustment)
         return runningOnRealRobot ? 1.0 : 4.0;
      else
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
      return !runningOnRealRobot;
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
