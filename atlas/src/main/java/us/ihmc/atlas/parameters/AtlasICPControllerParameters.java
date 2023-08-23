package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.CoPProjectionTowardsMidpoint;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

/** {@inheritDoc} */
public class AtlasICPControllerParameters extends ICPControllerParameters
{
   private final boolean runningOnRealRobot;
   private final boolean useAngularMomentum = true;

   private FeedbackProjectionOperator feedbackProjectionOperator;

   public AtlasICPControllerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxAllowedDistanceCMPSupport()
   {
      return 0.04;
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

   @Override
   public double getFeedbackDirectionWeight()
   {
      return 1e6;
   }


   @Override
   public boolean useSmartICPIntegrator()
   {
      return true;
   }

   @Override
   public void createFeedbackProjectionOperator(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      feedbackProjectionOperator = new CoPProjectionTowardsMidpoint(registry, yoGraphicsListRegistry);
   }

   @Override
   public FeedbackProjectionOperator getFeedbackProjectionOperator()
   {
      return feedbackProjectionOperator;
   }
}
