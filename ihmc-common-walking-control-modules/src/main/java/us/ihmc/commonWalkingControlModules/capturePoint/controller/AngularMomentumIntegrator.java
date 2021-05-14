package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AngularMomentumIntegrator
{

   private final BooleanProvider useSmartICPIntegrator;
   private final YoFrameVector2D feedbackCMPIntegral;
   private final YoDouble currentCoMVelocityMagnitude;
   private final YoDouble desiredICPVelocityMagnitude;


   private final ICPControlGainsReadOnly feedbackGains;

   private final FrameVector2D tempVector2d = new FrameVector2D();

   private final GlitchFilteredYoBoolean isICPStuck;
   private final DoubleProvider thresholdForStuck;


   private final double controlDT;

   public AngularMomentumIntegrator(String prefix, ICPOptimizationParameters icpOptimizationParameters, ICPControlGainsReadOnly feedbackGains, double controlDT, YoRegistry registry)
   {
      this.controlDT = controlDT;
      this.feedbackGains = feedbackGains;

      isICPStuck = new GlitchFilteredYoBoolean(prefix + "IsICPStuck", registry, (int) (0.03 / controlDT));
      currentCoMVelocityMagnitude = new YoDouble(prefix + "CurrentCoMVelocityMagnitude", registry);
      desiredICPVelocityMagnitude = new YoDouble(prefix + "DesiredICPVelocityMagnitude", registry);


      useSmartICPIntegrator = new BooleanParameter(prefix + "UseSmartICPIntegrator", registry, icpOptimizationParameters.useSmartICPIntegrator());
      thresholdForStuck = new DoubleParameter(prefix + "ThresholdForStuck", registry, icpOptimizationParameters.getICPVelocityThresholdForStuck());
      feedbackCMPIntegral = new YoFrameVector2D(prefix + "FeedbackCMPIntegral", ReferenceFrame.getWorldFrame(), registry);
   }

   public void reset()
   {
      isICPStuck.set(false);
   }

   public void update(FrameVector2DReadOnly desiredICPVelocity, FrameVector2DReadOnly currentCoMVelocity, FrameVector2DReadOnly icpError)
   {
      desiredICPVelocityMagnitude.set(desiredICPVelocity.length());
      if (desiredICPVelocityMagnitude.getDoubleValue() > thresholdForStuck.getValue())
      {
         isICPStuck.set(false);
         feedbackCMPIntegral.setToZero();
         return;
      }

      currentCoMVelocityMagnitude.set(currentCoMVelocity.length());
      if (currentCoMVelocityMagnitude.getDoubleValue() < thresholdForStuck.getValue())
         isICPStuck.set(true);

      if (useSmartICPIntegrator.getValue() && isICPStuck.getBooleanValue())
      {
         tempVector2d.set(icpError);
         tempVector2d.scale(controlDT * feedbackGains.getKi());

         feedbackCMPIntegral.scale(Math.pow(feedbackGains.getIntegralLeakRatio(), controlDT));
         feedbackCMPIntegral.add(tempVector2d);

         double length = feedbackCMPIntegral.length();
         double maxLength = feedbackGains.getMaxIntegralError();
         if (length > maxLength)
            feedbackCMPIntegral.scale(maxLength / length);
         if (Math.abs(feedbackGains.getKi()) < 1e-10)
            feedbackCMPIntegral.setToZero();
      }
      else
      {
         feedbackCMPIntegral.setToZero();
      }
   }


   public FrameVector2DReadOnly getFeedbackCMPIntegral()
   {
      return feedbackCMPIntegral;
   }
}
