package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;

/** {@inheritDoc} */
public class ValkyrieICPControllerParameters extends ICPControllerParameters
{
   private final boolean useAngularMomentum;
   private final RobotTarget target;

   public ValkyrieICPControllerParameters(RobotTarget target)
   {
      this.target = target;
      useAngularMomentum = (target == RobotTarget.REAL_ROBOT) ? false : true;
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
   public double getAngularMomentumMinimizationWeight()
   {
      return 10.0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean scaleFeedbackWeightWithGain()
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
      return 0.001;
   }
}
