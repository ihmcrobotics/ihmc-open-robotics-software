package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings.FilterDouble1D;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings.FilterVector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FeedbackControllerFilterFactory
{
   public static FilterDouble1D createVelocity1DErrorHPRLFilter(String jointName, YoHPRLParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistry(registry);
      return new YoHPRLFilterDouble1D(jointName + "VelocityErrorHPRL", parameters, dt, registry);
   }

   public static FilterVector3D createAngularVelocityErrorHPRLFilter(String endEffectorName, YoHPRLParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistry(registry);
      return new YoHPRLFilterVector3D(endEffectorName + "AngularVelocityErrorHPRL", parameters, dt, registry);
   }

   public static FilterVector3D createLinearVelocityErrorHPRLFilter(String endEffectorName, YoHPRLParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistry(registry);
      return new YoHPRLFilterVector3D(endEffectorName + "LinearVelocityErrorHPRL", parameters, dt, registry);
   }

   public static FilterDouble1D createVelocity1DErrorPIOFilter(String jointName, YoPIOParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistry(registry);
      return new YoPIOFilterDouble1D(jointName + "VelocityErrorPIO", parameters, dt, registry);
   }

   public static FilterVector3D createAngularVelocityErrorPIOFilter(String endEffectorName, YoPIOParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistry(registry);
      return new YoPIOFilterVector3D(endEffectorName + "AngularVelocityErrorPIO", parameters, dt, registry);
   }

   public static FilterVector3D createLinearVelocityErrorPIOFilter(String endEffectorName, YoPIOParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistry(registry);
      return new YoPIOFilterVector3D(endEffectorName + "LinearVelocityErrorPIO", parameters, dt, registry);
   }

   public static class YoHPRLParameters
   {
      private final YoBoolean enableFilter;
      private final YoDouble breakFrequency;
      private final YoDouble maxOutputRate;
      private final YoDouble updateScale;
      private final YoDouble updateReferenceRatio;

      public YoHPRLParameters(String namePrefix,
                              boolean enableFilter,
                              double breakFrequency,
                              double maxOutputRate,
                              double updateScale,
                              double updateReferenceRatio)
      {
         this.enableFilter = new YoBoolean(namePrefix + "EnableFilter", null);
         this.breakFrequency = new YoDouble(namePrefix + "BreakFrequency", null);
         this.maxOutputRate = new YoDouble(namePrefix + "MaxOutputRate", null);
         this.updateScale = new YoDouble(namePrefix + "UpdateScale", null);
         this.updateReferenceRatio = new YoDouble(namePrefix + "UpdateReferenceRatio", null);
         this.enableFilter.set(enableFilter);
         this.breakFrequency.set(breakFrequency);
         this.maxOutputRate.set(maxOutputRate);
         this.updateScale.set(updateScale);
         this.updateReferenceRatio.set(updateReferenceRatio);
      }

      public void setRegistry(YoRegistry registry)
      {
         if (enableFilter.getRegistry() != null)
            return;
         enableFilter.setRegistry(registry);
         breakFrequency.setRegistry(registry);
         maxOutputRate.setRegistry(registry);
         updateScale.setRegistry(registry);
         updateReferenceRatio.setRegistry(registry);
      }

      public void setEnableFilter(boolean enableFilter)
      {
         this.enableFilter.set(enableFilter);
      }

      public void setBreakFrequency(double breakFrequency)
      {
         this.breakFrequency.set(breakFrequency);
      }

      public void setMaxOutputRate(double maxOutputRate)
      {
         this.maxOutputRate.set(maxOutputRate);
      }

      public void setUpdateScale(double updateScale)
      {
         this.updateScale.set(updateScale);
      }

      public void setUpdateReferenceRatio(double updateReferenceRatio)
      {
         this.updateReferenceRatio.set(updateReferenceRatio);
      }

      public boolean getEnableFilter()
      {
         return enableFilter.getValue();
      }

      public double getBreakFrequency()
      {
         return breakFrequency.getValue();
      }

      public double getMaxOutputRate()
      {
         return maxOutputRate.getValue();
      }

      public double getUpdateScale()
      {
         return updateScale.getValue();
      }

      public double getUpdateReferenceRatio()
      {
         return updateReferenceRatio.getValue();
      }
   }

   public static class YoPIOParameters
   {
      private final YoBoolean enableFilter;
      private final YoDouble maxInputRate;
      private final YoDouble maxInput;
      private final YoDouble leadFrequency;
      private final YoDouble lagFrequency;
      private final YoDouble updateStepBreakFrequency;
      private final YoDouble maxOutputRate;

      public YoPIOParameters(String namePrefix,
                             boolean enableFilter,
                             double maxInputRate,
                             double maxInput,
                             double leadFrequency,
                             double lagFrequency,
                             double updateStepBreakFrequency,
                             double maxOutputRate)
      {
         this.enableFilter = new YoBoolean(namePrefix + "EnableFilter", null);
         this.maxInputRate = new YoDouble(namePrefix + "MaxInputRate", null);
         this.maxInput = new YoDouble(namePrefix + "MaxInput", null);
         this.leadFrequency = new YoDouble(namePrefix + "LeadFrequency", null);
         this.lagFrequency = new YoDouble(namePrefix + "LagFrequency", null);
         this.updateStepBreakFrequency = new YoDouble(namePrefix + "UpdateStepBreakFrequency", null);
         this.maxOutputRate = new YoDouble(namePrefix + "MaxOutputRate", null);

         this.enableFilter.set(enableFilter);
         this.maxInputRate.set(maxInputRate);
         this.maxInput.set(maxInput);
         this.leadFrequency.set(leadFrequency);
         this.lagFrequency.set(lagFrequency);
         this.updateStepBreakFrequency.set(updateStepBreakFrequency);
         this.maxOutputRate.set(maxOutputRate);
      }

      public void setRegistry(YoRegistry registry)
      {
         if (enableFilter.getRegistry() != null)
            return;

         enableFilter.setRegistry(registry);
         maxInputRate.setRegistry(registry);
         maxInput.setRegistry(registry);
         leadFrequency.setRegistry(registry);
         lagFrequency.setRegistry(registry);
         updateStepBreakFrequency.setRegistry(registry);
         maxOutputRate.setRegistry(registry);
      }

      public void setEnableFilter(boolean enableFilter)
      {
         this.enableFilter.set(enableFilter);
      }

      public void setMaxInputRate(double maxInputRate)
      {
         this.maxInputRate.set(maxInputRate);
      }

      public void setMaxInput(double maxInput)
      {
         this.maxInput.set(maxInput);
      }

      public void setLeadFrequency(double leadFrequency)
      {
         this.leadFrequency.set(leadFrequency);
      }

      public void setLagFrequency(double lagFrequency)
      {
         this.lagFrequency.set(lagFrequency);
      }

      public void setUpdateStepBreakFrequency(double updateStepBreakFrequency)
      {
         this.updateStepBreakFrequency.set(updateStepBreakFrequency);
      }

      public void setMaxOutputRate(double maxOutputRate)
      {
         this.maxOutputRate.set(maxOutputRate);
      }

      public boolean getEnableFilter()
      {
         return enableFilter.getValue();
      }

      public double getMaxInputRate()
      {
         return maxInputRate.getValue();
      }

      public double getMaxInput()
      {
         return maxInput.getValue();
      }

      public double getLeadFrequency()
      {
         return leadFrequency.getValue();
      }

      public double getLagFrequency()
      {
         return lagFrequency.getValue();
      }

      public double getUpdateStepFrequency()
      {
         return updateStepBreakFrequency.getValue();
      }

      public double getMaxOutputRate()
      {
         return maxOutputRate.getValue();
      }
   }

   private static class YoHPRLFilterDouble1D implements FilterDouble1D
   {
      private final YoBoolean initialize;
      private final YoDouble inputLPF;
      private final YoDouble inputLPF2;
      private final YoDouble inputHPF;
      private final YoDouble inputHPRL;
      private final YoDouble output;

      private final BooleanProvider enableFilter;
      private final DoubleProvider breakFrequency;
      private final DoubleProvider maxOutputRate;
      private final DoubleProvider updateScale;
      private final DoubleProvider updateReferenceRatio;

      private final double dt;

      public YoHPRLFilterDouble1D(String namePrefix, YoHPRLParameters parameters, double dt, YoRegistry registry)
      {
         this(namePrefix,
              parameters.enableFilter,
              parameters.breakFrequency,
              parameters.maxOutputRate,
              parameters.updateScale,
              parameters.updateReferenceRatio,
              dt,
              registry);
      }

      public YoHPRLFilterDouble1D(String namePrefix,
                                  BooleanProvider enableFilter,
                                  DoubleProvider breakFrequency,
                                  DoubleProvider maxOutputRate,
                                  DoubleProvider updateScale,
                                  DoubleProvider updateReferenceRatio,
                                  double dt,
                                  YoRegistry registry)
      {
         this.enableFilter = enableFilter;
         this.breakFrequency = breakFrequency;
         this.maxOutputRate = maxOutputRate;
         this.updateScale = updateScale;
         this.updateReferenceRatio = updateReferenceRatio;
         this.dt = dt;

         initialize = new YoBoolean(namePrefix + "Initialize", registry);
         inputLPF = new YoDouble(namePrefix + "InputLPF", registry);
         inputLPF2 = new YoDouble(namePrefix + "InputLPF2", registry);
         inputHPF = new YoDouble(namePrefix + "InputHPF", registry);
         inputHPRL = new YoDouble(namePrefix + "InputHPRL", registry);
         output = new YoDouble(namePrefix + "Output", registry);
      }

      @Override
      public void reset()
      {
         initialize.set(true);
      }

      @Override
      public double apply(double rawInput)
      {
         if (initialize.getValue())
         {
            inputLPF.set(rawInput);
            inputHPF.set(rawInput);
            output.set(rawInput);
            initialize.set(false);
         }

         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getValue(), dt);
         inputLPF.set(EuclidCoreTools.interpolate(rawInput, inputLPF.getValue(), alpha));
         inputLPF2.set(EuclidCoreTools.interpolate(inputLPF.getValue(), inputLPF2.getValue(), alpha));
         double inputHPFPrevious = inputHPF.getValue();
         inputHPF.set(rawInput - inputLPF2.getValue());
         if (updateScale.getValue() <= 0.0)
         {
            inputHPRL.set(0.0);
            output.set(inputLPF2.getValue());
         }
         else
         {
            double reference = EuclidCoreTools.interpolate(inputHPFPrevious, inputHPRL.getValue(), updateReferenceRatio.getValue());
            double update = updateScale.getValue() * (inputHPF.getValue() - reference);
            inputHPRL.add(MathTools.clamp(update, maxOutputRate.getValue() * dt));
            output.set(inputLPF2.getValue() + inputHPRL.getValue());
         }

         // Checking at the end allows to visualize the filter state when disabled.
         if (enableFilter.getValue())
            return output.getValue();
         else
            return rawInput;
      }
   }

   /**
    * Pilot-Induced Oscillation (PIO) filter for double values as implemented in the ISA spool controller.
    */
   public static class YoPIOFilterDouble1D implements FilterDouble1D
   {
      private final YoBoolean initialize;
      private final YoDouble rateLimitedInput;
      private final YoDouble updateStepLeadLag_q1;
      private final YoDouble updateStepLeadLag_q2;
      private final YoDouble updateStepLeadLagOutput;
      private final YoDouble updateStepLPF;
      private final YoDouble outputPreRateLimit;
      private final YoDouble output;

      private final BooleanProvider enableFilter;
      private final DoubleProvider maxInputRate;
      private final DoubleProvider maxInput;
      private final DoubleProvider leadFreq;
      private final DoubleProvider lagFreq;
      private final DoubleProvider updateStepBreakFrequency;
      private final DoubleProvider maxOutputRate;
      private final double dt;

      public YoPIOFilterDouble1D(String namePrefix, YoPIOParameters parameters, double dt, YoRegistry registry)
      {
         this(namePrefix,
              parameters.enableFilter,
              parameters.maxInputRate,
              parameters.maxInput,
              parameters.leadFrequency,
              parameters.lagFrequency,
              parameters.updateStepBreakFrequency,
              parameters.maxOutputRate,
              dt,
              registry);
      }

      public YoPIOFilterDouble1D(String namePrefix,
                                 BooleanProvider enableFilter,
                                 DoubleProvider maxInputRate,
                                 DoubleProvider maxInput,
                                 DoubleProvider leadFreq,
                                 DoubleProvider lagFreq,
                                 DoubleProvider updateStepBreakFrequency,
                                 DoubleProvider maxOutputRate,
                                 double dt,
                                 YoRegistry registry)
      {
         this.enableFilter = enableFilter;
         this.maxInputRate = maxInputRate;
         this.maxInput = maxInput;
         this.leadFreq = leadFreq;
         this.lagFreq = lagFreq;
         this.updateStepBreakFrequency = updateStepBreakFrequency;
         this.maxOutputRate = maxOutputRate;
         this.dt = dt;

         initialize = new YoBoolean(namePrefix + "Initialize", registry);
         if (maxInputRate != null)
            rateLimitedInput = new YoDouble(namePrefix + "RateLimitedInput", registry);
         else
            rateLimitedInput = null;

         updateStepLeadLag_q1 = new YoDouble(namePrefix + "UpdateStepLeadLag_q1", registry);
         updateStepLeadLag_q2 = new YoDouble(namePrefix + "UpdateStepLeadLag_q2", registry);
         updateStepLeadLagOutput = new YoDouble(namePrefix + "UpdateStepLeadLagOutput", registry);

         if (updateStepBreakFrequency != null)
            updateStepLPF = new YoDouble(namePrefix + "UpdateStepLPF", registry);
         else
            updateStepLPF = null;

         if (maxOutputRate != null)
            outputPreRateLimit = new YoDouble(namePrefix + "PIOOutputPreRateLimit", registry);
         else
            outputPreRateLimit = null;

         output = new YoDouble(namePrefix + "Output", registry);
      }

      @Override
      public void reset()
      {
         initialize.set(true);
      }

      @Override
      public double apply(double rawInput)
      {
         if (initialize.getValue())
         {
            initialize.set(false);

            if (rateLimitedInput != null)
               rateLimitedInput.set(rawInput);

            updateStepLeadLag_q1.set(0.0);
            updateStepLeadLag_q2.set(0.0);

            if (updateStepLPF != null)
               updateStepLPF.set(0.0);

            if (outputPreRateLimit != null)
               outputPreRateLimit.set(0.0);

            output.set(0.0);
            return output.getValue();
         }

         double processedInput = rawInput;
         if (rateLimitedInput != null)
         {
            // First, rate limit pass. maxInputRate is set pretty high and is tuned for the PIO
            rateLimitedInput.set(computeRateLimit(rateLimitedInput.getValue(), rawInput, maxInputRate.getValue(), dt));
            processedInput = rateLimitedInput.getValue();
         }

         if (maxInput != null)
         { // Then clamp the input
            processedInput = MathTools.clamp(processedInput, maxInput.getValue());
         }

         // Pilot Induced Oscillation filter described in Alcala paper.
         // Lead-Lag Filter:
         double pioDiff = processedInput - output.getValue();

         double alpha_lead = 2.0 * Math.PI * leadFreq.getValue();
         double alpha_lag = 2.0 * Math.PI * lagFreq.getValue();
         double alpha2_lead = MathTools.square(alpha_lead);
         double alpha2_lag = MathTools.square(alpha_lag);

         double q1 = updateStepLeadLag_q1.getValue();
         double q2 = updateStepLeadLag_q2.getValue();

         q1 = q1 + dt * q2;
         q2 = q2 + dt * (pioDiff - alpha2_lag * q1 - 2.0 * alpha_lag * q2);
         double pioDiffLeadLagOutput = (alpha2_lead - alpha2_lag) * q1 + 2.0 * (alpha_lead - alpha_lag) * q2 + pioDiff;
         updateStepLeadLagOutput.set(pioDiffLeadLagOutput);

         updateStepLeadLag_q1.set(q1);
         updateStepLeadLag_q2.set(q2);

         double preprocessedOutput;

         if (updateStepLPF != null)
         { // Low Pass Filter:
            double diffLPFAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(updateStepBreakFrequency.getValue(), dt);
            updateStepLPF.set(EuclidCoreTools.interpolate(pioDiffLeadLagOutput, updateStepLPF.getValue(), diffLPFAlpha));
            preprocessedOutput = updateStepLPF.getValue() + output.getValue();
         }
         else
         {
            preprocessedOutput = pioDiffLeadLagOutput + output.getValue();
         }

         if (outputPreRateLimit != null)
         { // PIO Rate Limit:
            outputPreRateLimit.set(preprocessedOutput);
            output.set(computeRateLimit(output.getValue(), outputPreRateLimit.getValue(), maxOutputRate.getValue(), dt));
         }
         else
         {
            output.set(preprocessedOutput);
         }

         // Checking at the end allows to visualize the filter state when disabled.
         if (enableFilter.getValue())
            return output.getValue();
         else
            return rawInput;
      }

      public YoBoolean getInitialize()
      {
         return initialize;
      }

      public YoDouble getRateLimitedInput()
      {
         return rateLimitedInput;
      }

      public YoDouble getUpdateStepLeadLag_q1()
      {
         return updateStepLeadLag_q1;
      }

      public YoDouble getUpdateStepLeadLag_q2()
      {
         return updateStepLeadLag_q2;
      }

      public YoDouble getUpdateStepLeadLagOutput()
      {
         return updateStepLeadLagOutput;
      }

      public YoDouble getUpdateStepLPF()
      {
         return updateStepLPF;
      }

      public YoDouble getOutputPreRateLimit()
      {
         return outputPreRateLimit;
      }

      public YoDouble getOutput()
      {
         return output;
      }
   }

   private static class YoHPRLFilterVector3D implements FilterVector3D
   {
      private final YoBoolean initialize;
      private final YoMutableFrameVector3D inputLPF;
      private final YoMutableFrameVector3D inputLPF2;
      private final YoMutableFrameVector3D inputHPF;
      private final YoMutableFrameVector3D inputHPRL;
      private final YoMutableFrameVector3D output;

      private final BooleanProvider enableFilter;
      private final DoubleProvider breakFrequency;
      private final DoubleProvider maxOutputRate;
      private final DoubleProvider updateScale;
      private final DoubleProvider updateReferenceRatio;

      private final double dt;

      public YoHPRLFilterVector3D(String namePrefix, YoHPRLParameters parameters, double dt, YoRegistry registry)
      {
         this(namePrefix,
              parameters.enableFilter,
              parameters.breakFrequency,
              parameters.maxOutputRate,
              parameters.updateScale,
              parameters.updateReferenceRatio,
              dt,
              registry);
      }

      public YoHPRLFilterVector3D(String namePrefix,
                                  BooleanProvider enableFilter,
                                  DoubleProvider breakFrequency,
                                  DoubleProvider maxOutputRate,
                                  DoubleProvider updateScale,
                                  DoubleProvider updateReferenceRatio,
                                  double dt,
                                  YoRegistry registry)
      {
         this.enableFilter = enableFilter;
         this.breakFrequency = breakFrequency;
         this.maxOutputRate = maxOutputRate;
         this.updateScale = updateScale;
         this.updateReferenceRatio = updateReferenceRatio;
         this.dt = dt;

         initialize = new YoBoolean(namePrefix + "Initialize", registry);
         inputLPF = new YoMutableFrameVector3D(namePrefix + "InputLPF", "", registry);
         inputLPF2 = new YoMutableFrameVector3D(namePrefix + "InputLPF2", "", registry);
         inputHPF = new YoMutableFrameVector3D(namePrefix + "InputHPF", "", registry);
         inputHPRL = new YoMutableFrameVector3D(namePrefix + "InputHPRL", "", registry);
         output = new YoMutableFrameVector3D(namePrefix + "Output", "", registry);
      }

      @Override
      public void reset()
      {
         initialize.set(true);
      }

      private final Vector3D inputHPFPrevious = new Vector3D();
      private final Vector3D reference = new Vector3D();
      private final Vector3D update = new Vector3D();

      @Override
      public void apply(FrameVector3DReadOnly rawInput, FixedFrameVector3DBasics filteredOutput)
      {
         if (initialize.getValue())
         {
            inputLPF.set(rawInput);
            inputHPF.set(rawInput);
            output.set(rawInput);
            initialize.set(false);
         }

         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getValue(), dt);
         inputLPF.interpolate(rawInput, inputLPF, alpha);
         inputLPF2.interpolate(inputLPF, inputLPF2, alpha);
         inputHPFPrevious.set(inputHPF);
         inputHPF.sub(rawInput, inputLPF2);

         if (updateScale.getValue() <= 0.0)
         {
            reference.interpolate(inputHPFPrevious, inputHPRL, updateReferenceRatio.getValue());
            update.sub(inputHPF, reference);
            update.scale(updateScale.getValue());
            update.clipToMaxNorm(maxOutputRate.getValue() * dt);
            inputHPRL.add(update);

            output.add(inputLPF2, inputHPRL);
         }
         else
         {
            inputHPRL.setToZero();
            output.set(inputLPF2);
         }

         // Checking at the end allows to visualize the filter state when disabled.
         if (enableFilter.getValue())
            filteredOutput.set(output);
         else
            filteredOutput.set(rawInput);
      }
   }

   private static class YoPIOFilterVector3D implements FilterVector3D
   {
      private final YoBoolean initialize;
      private final YoMutableFrameVector3D rateLimitedInput;
      private final YoMutableFrameVector3D limitedInput;
      private final YoMutableFrameVector3D updateStepLeadLag_q1;
      private final YoMutableFrameVector3D updateStepLeadLag_q2;
      private final YoMutableFrameVector3D updateStepLeadLagOutput;
      private final YoMutableFrameVector3D updateStepLPF;
      private final YoMutableFrameVector3D outputPreRateLimit;
      private final YoMutableFrameVector3D output;

      private final BooleanProvider enableFilter;
      private final DoubleProvider maxInputRate;
      private final DoubleProvider maxInput;
      private final DoubleProvider leadFreq;
      private final DoubleProvider lagFreq;
      private final DoubleProvider updateStepBreakFrequency;
      private final DoubleProvider maxOutputRate;
      private final double dt;

      public YoPIOFilterVector3D(String namePrefix, YoPIOParameters parameters, double dt, YoRegistry registry)
      {
         this(namePrefix,
              parameters.enableFilter,
              parameters.maxInputRate,
              parameters.maxInput,
              parameters.leadFrequency,
              parameters.lagFrequency,
              parameters.updateStepBreakFrequency,
              parameters.maxOutputRate,
              dt,
              registry);
      }

      public YoPIOFilterVector3D(String namePrefix,
                                 BooleanProvider enableFilter,
                                 DoubleProvider maxInputRate,
                                 DoubleProvider maxInput,
                                 DoubleProvider leadFreq,
                                 DoubleProvider lagFreq,
                                 DoubleProvider updateStepBreakFrequency,
                                 DoubleProvider maxOutputRate,
                                 double dt,
                                 YoRegistry registry)
      {
         this.enableFilter = enableFilter;
         this.maxInputRate = maxInputRate;
         this.maxInput = maxInput;
         this.leadFreq = leadFreq;
         this.lagFreq = lagFreq;
         this.updateStepBreakFrequency = updateStepBreakFrequency;
         this.maxOutputRate = maxOutputRate;
         this.dt = dt;

         initialize = new YoBoolean(namePrefix + "Initialize", registry);
         if (maxInputRate != null)
            rateLimitedInput = new YoMutableFrameVector3D(namePrefix + "RateLimitedInput", "", registry);
         else
            rateLimitedInput = null;

         if (maxInput != null)
            limitedInput = new YoMutableFrameVector3D(namePrefix + "LimitedInput", "", registry);
         else
            limitedInput = null;

         updateStepLeadLag_q1 = new YoMutableFrameVector3D(namePrefix + "UpdateStepLeadLag_q1", "", registry);
         updateStepLeadLag_q2 = new YoMutableFrameVector3D(namePrefix + "UpdateStepLeadLag_q2", "", registry);
         updateStepLeadLagOutput = new YoMutableFrameVector3D(namePrefix + "UpdateStepLeadLagOutput", "", registry);

         if (updateStepBreakFrequency != null)
            updateStepLPF = new YoMutableFrameVector3D(namePrefix + "UpdateStepLPF", "", registry);
         else
            updateStepLPF = null;

         if (maxOutputRate != null)
            outputPreRateLimit = new YoMutableFrameVector3D(namePrefix + "PIOOutputPreRateLimit", "", registry);
         else
            outputPreRateLimit = null;

         output = new YoMutableFrameVector3D(namePrefix + "Output", "", registry);
      }

      @Override
      public void reset()
      {
         initialize.set(true);
      }

      private final Vector3D rateLimitTemp = new Vector3D();
      private final Vector3D pioDiff = new Vector3D();
      private final Vector3D q1 = new Vector3D();
      private final Vector3D q2 = new Vector3D();
      private final Vector3D preprocessedOutput = new Vector3D();

      @Override
      public void apply(FrameVector3DReadOnly rawInput, FixedFrameVector3DBasics filteredOutput)
      {
         if (output.getReferenceFrame() != rawInput.getReferenceFrame())
            reset();

         if (initialize.getValue())
         {
            initialize.set(false);

            if (rateLimitedInput != null)
               rateLimitedInput.setIncludingFrame(rawInput);

            if (limitedInput != null)
               limitedInput.setToZero(rawInput.getReferenceFrame());

            updateStepLeadLag_q1.setToZero(rawInput.getReferenceFrame());
            updateStepLeadLag_q2.setToZero(rawInput.getReferenceFrame());

            if (updateStepLPF != null)
               updateStepLPF.setToZero(rawInput.getReferenceFrame());

            if (outputPreRateLimit != null)
               outputPreRateLimit.setToZero(rawInput.getReferenceFrame());

            output.setToZero(rawInput.getReferenceFrame());
            filteredOutput.setMatchingFrame(output);
         }

         FrameVector3DReadOnly processedInput = rawInput;
         if (rateLimitedInput != null)
         {
            // First, rate limit pass. maxInputRate is set pretty high and is tuned for the PIO
            rateLimitTemp.sub(rawInput, rateLimitedInput);
            rateLimitTemp.clipToMaxNorm(maxInputRate.getValue() * dt);
            rateLimitedInput.add(rateLimitTemp);
            processedInput = rateLimitedInput;
         }

         if (limitedInput != null)
         { // Then clamp the input
            limitedInput.set(processedInput);
            limitedInput.clipToMaxNorm(maxInput.getValue());
            processedInput = limitedInput;
         }

         // Pilot Induced Oscillation filter described in Alcala paper.
         // Lead-Lag Filter:
         pioDiff.sub(processedInput, output);

         double alpha_lead = 2.0 * Math.PI * leadFreq.getValue();
         double alpha_lag = 2.0 * Math.PI * lagFreq.getValue();
         double alpha2_lead = MathTools.square(alpha_lead);
         double alpha2_lag = MathTools.square(alpha_lag);

         // q1 = q1 + dt * q2
         q1.scaleAdd(dt, q2, updateStepLeadLag_q1);
         // q2 = q2 + dt * (pioDiff - alpha2_lag * q1 - 2.0 * alpha_lag * q2)
         q2.setAndScale(-2.0 * alpha_lag, updateStepLeadLag_q2);
         q2.scaleAdd(-alpha2_lag, q1, q2);
         q2.add(pioDiff);
         q2.scaleAdd(dt, updateStepLeadLag_q2);
         // pioDiffLeadLagOutput = (alpha2_lead - alpha2_lag) * q1 + 2.0 * (alpha_lead - alpha_lag) * q2 + pioDiff;
         updateStepLeadLagOutput.setAndScale(alpha2_lead - alpha2_lag, q1);
         updateStepLeadLagOutput.scaleAdd(2.0 * (alpha_lead - alpha_lag), q2, updateStepLeadLagOutput);
         updateStepLeadLagOutput.add(pioDiff);

         updateStepLeadLag_q1.set(q1);
         updateStepLeadLag_q2.set(q2);

         if (updateStepLPF != null)
         { // Low Pass Filter:
            double diffLPFAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(updateStepBreakFrequency.getValue(), dt);
            updateStepLPF.interpolate(updateStepLeadLagOutput, updateStepLPF, diffLPFAlpha);
            preprocessedOutput.add(updateStepLPF, output);
         }
         else
         {
            preprocessedOutput.add(updateStepLeadLagOutput, output);
         }

         if (outputPreRateLimit != null)
         { // PIO Rate Limit:
            outputPreRateLimit.set(preprocessedOutput);
            rateLimitTemp.sub(outputPreRateLimit, output);
            rateLimitTemp.clipToMaxNorm(maxOutputRate.getValue() * dt);
            output.add(rateLimitTemp);
         }
         else
         {
            output.set(preprocessedOutput);
         }

         // Checking at the end allows to visualize the filter state when disabled.
         if (enableFilter.getValue())
            filteredOutput.setMatchingFrame(output);
         else
            filteredOutput.setMatchingFrame(rawInput);
      }
   }

   private static double computeRateLimit(double previousValue, double currentValue, double maxRate, double dt)
   {
      return previousValue + MathTools.clamp(currentValue - previousValue, maxRate * dt);
   }
}
