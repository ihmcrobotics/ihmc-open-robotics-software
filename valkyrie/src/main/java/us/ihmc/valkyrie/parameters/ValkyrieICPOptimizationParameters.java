package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;

/** {@inheritDoc} */
public class ValkyrieICPOptimizationParameters extends ICPOptimizationParameters
{
   private final boolean useAngularMomentum;
   private final boolean useStepAdjustment;
   private final RobotTarget target;

   public ValkyrieICPOptimizationParameters(RobotTarget target)
   {
      this.target = target;
      useAngularMomentum = (target == RobotTarget.REAL_ROBOT) ? false : true;
      useStepAdjustment = (target == RobotTarget.REAL_ROBOT) ? false : true;
   }

   /** {@inheritDoc} */
   @Override
   public double getForwardFootstepWeight()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 7.5 : 20.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getLateralFootstepWeight()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 7.5 : 20.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getFootstepRateWeight()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 4e-7 : 4e-7;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackLateralWeight()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 0.5 : 1.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackForwardWeight()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 0.5 : 1.5;
   }
   
   /**
    * Specifies the amount of ICP error (the 2D distance in XY from desired to current) that is required for the controller to consider step adjustment.
    */
   public double getMinICPErrorForStepAdjustment()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 0.04 : super.getMinICPErrorForStepAdjustment();
   }

   /** {@inheritDoc} */
   @Override
   public double getFeedbackRateWeight()
   {
      return 1e-8;
   }

   /** {@inheritDoc} */
   @Override
   public ICPControlGainsReadOnly getICPFeedbackGains()
   {
      ICPControlGains gains = new ICPControlGains();
      gains.setKpOrthogonalToMotion(1.9);
      gains.setKpParallelToMotion(2.0);

      gains.setIntegralLeakRatio(0.97);
      gains.setMaxIntegralError(0.05);
      gains.setKi(1.0);

      if (target == RobotTarget.REAL_ROBOT)
         gains.setFeedbackPartMaxRate(1.5);

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
   public boolean getUseAngularMomentumIntegrator()
   {
      return (target == RobotTarget.REAL_ROBOT) ? false : true;
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

   public boolean allowUsePlanarRegionConstraints()
   {
      return false;
   }
}
