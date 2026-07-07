package us.ihmc.zulu.parameters.controller;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;

public class ZuluICPControllerParameters extends ICPControllerParameters
{
   private FeedbackAlphaCalculator feedbackAlphaCalculator = null;
   private FeedForwardAlphaCalculator feedForwardAlphaCalculator = null;

   @Override
   public ICPControlGainsReadOnly getHighlyDampedICPFeedbackGains()
   {
      ICPControlGains gains = new ICPControlGains();
      gains.setKpOrthogonalToMotion(2.0);
      gains.setFeedbackPartMaxRate(4.0);
      gains.setKpParallelToMotion(2.5);
      gains.setKi(2.0);
      return gains;
   }

   @Override
   public double getFeedbackForwardWeight()
   {
      // TODO Needs tune up.
      return 0.5;
   }

   @Override
   public double getFeedbackLateralWeight()
   {
      // TODO Needs tune up.
      return 0.5;
   }

   @Override
   public double getFeedbackRateWeight()
   {
      // TODO Needs tune up.
      return 1e-8;
   }

   @Override
   public ICPControlGainsReadOnly getICPFeedbackGains()
   {
      ICPControlGains gains = new ICPControlGains();
      gains.setKpOrthogonalToMotion(2.0);
      gains.setKpParallelToMotion(2.5);
      gains.setKi(2.0);
      return gains;
   }

   @Override
   public double getDynamicsObjectiveWeight()
   {
      return 10000.0;
   }

   @Override
   public double getAngularMomentumMinimizationWeight()
   {
      // TODO Needs tune up.
      return 10.0;
   }

   @Override
   public boolean scaleFeedbackWeightWithGain()
   {
      return true;
   }

   @Override
   public boolean getUseICPControlPolygons()
   {
      return false;
   }

   @Override
   public boolean useAngularMomentum()
   {
      return true;
   }

   @Override
   public boolean getUseHeuristicICPController()
   {
      return false;
   }

   @Override
   public double getPureFeedbackErrorThreshold()
   {
      return 0.06;
   }

   @Override
   public boolean useSmartICPIntegrator()
   {
      return true;
   }

   @Override
   public double getICPVelocityThresholdForStuck()
   {
      return 0.06;
   }

   @Override
   public double getFeedbackDirectionWeight()
   {
      return 1e6;
   }

   @Override
   public FeedForwardAlphaCalculator getFeedForwardAlphaCalculator()
   {
      return feedForwardAlphaCalculator;
   }

   @Override
   public FeedbackAlphaCalculator getFeedbackAlphaCalculator()
   {
      return feedbackAlphaCalculator;
   }
}
