package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings.FilterDouble1D;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings.FilterVector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMutableFrameVector3D;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FeedbackControllerFilterFactory
{
   /**
    * Creates a low-pass filter for a 1-DoF joint velocity error.
    *
    * @param jointName      the name of the joint to which the filter is to be applied.
    * @param breakFrequency the break frequency of the low-pass filter.
    * @param dt             the time step of the controller.
    * @param registry       the registry to which the filter variables are to be added.
    * @return the low-pass filter to apply to the error velocity.
    */
   public static FilterDouble1D createVelocity1DErrorLPFFilter(String jointName, DoubleProvider breakFrequency, double dt, YoRegistry registry)
   {
      return new YoLPFilterDouble1D(jointName + "VelocityErrorLPF", breakFrequency, dt, registry);
   }

   /**
    * Creates a low-pass filter for the error in angular velocity for an end-effector.
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    * @param breakFrequency  the break frequency of the low-pass filter.
    * @param dt              the time step of the controller.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the low-pass filter to apply to the error velocity.
    */
   public static FilterVector3D createAngularVelocityErrorLPFFilter(String endEffectorName, DoubleProvider breakFrequency, double dt, YoRegistry registry)
   {
      return new YoLPFilterVector3D(endEffectorName + "AngularVelocityErrorLPF", breakFrequency, dt, registry);
   }

   /**
    * Creates a low-pass filter for the error in linear velocity for an end-effector.
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    * @param breakFrequency  the break frequency of the low-pass filter.
    * @param dt              the time step of the controller.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the low-pass filter to apply to the error velocity.
    */
   public static FilterVector3D createLinearVelocityErrorLPFFilter(String endEffectorName, DoubleProvider breakFrequency, double dt, YoRegistry registry)
   {
      return new YoLPFilterVector3D(endEffectorName + "LinearVelocityErrorLPF", breakFrequency, dt, registry);
   }

   /**
    * Creates a High-Pass Rate-Limited (HPRL) filter for a 1-DoF joint velocity error.
    * <p>
    * The filter is based on the following paper: "Torque-Based Dynamic Walking - A Long Way from Simulation to Experiment" by Englsberger et al.
    * </p>
    *
    * @param jointName  the name of the joint to which the filter is to be applied.
    * @param parameters the parameters for the filter.
    * @param dt         the time step of the controller.
    * @param registry   the registry to which the filter variables are to be added.
    * @return the HPRL filter to apply to the error velocity.
    */
   public static FilterDouble1D createVelocity1DErrorHPRLFilter(String jointName, YoHPRLParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistryLazy(registry);
      return new YoHPRLFilterDouble1D(jointName + "VelocityErrorHPRL", parameters, dt, registry);
   }

   /**
    * Creates a High-Pass Rate-Limited (HPRL) filter for the error in angular velocity for an end-effector.
    * <p>
    * The filter is based on the following paper: "Torque-Based Dynamic Walking - A Long Way from Simulation to Experiment" by Englsberger et al.
    * </p>
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    * @param parameters      the parameters for the filter.
    * @param dt              the time step of the controller.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the HPRL filter to apply to the error velocity.
    */
   public static FilterVector3D createAngularVelocityErrorHPRLFilter(String endEffectorName, YoHPRLParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistryLazy(registry);
      return new YoHPRLFilterVector3D(endEffectorName + "AngularVelocityErrorHPRL", parameters, dt, registry);
   }

   /**
    * Creates a High-Pass Rate-Limited (HPRL) filter for the error in linear velocity for an end-effector.
    * <p>
    * The filter is based on the following paper: "Torque-Based Dynamic Walking - A Long Way from Simulation to Experiment" by Englsberger et al.
    * </p>
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    * @param parameters      the parameters for the filter.
    * @param dt              the time step of the controller.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the HPRL filter to apply to the error velocity.
    */
   public static FilterVector3D createLinearVelocityErrorHPRLFilter(String endEffectorName, YoHPRLParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistryLazy(registry);
      return new YoHPRLFilterVector3D(endEffectorName + "LinearVelocityErrorHPRL", parameters, dt, registry);
   }

   /**
    * Creates a Pilot-Induced Oscillation (PIO) filter for a 1-DoF joint velocity error.
    * <p>
    * The filter is based on the following paper: "Phase compensation design for prevention of PIO due to actuator rate saturation" by Alcala et al.
    * </p>
    *
    * @param jointName  the name of the joint to which the filter is to be applied.
    * @param parameters the parameters for the filter.
    * @param dt         the time step of the controller.
    * @param registry   the registry to which the filter variables are to be added.
    * @return the PIO filter to apply to the error velocity.
    */
   public static FilterDouble1D createVelocity1DErrorPIOFilter(String jointName, YoPIOParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistryLazy(registry);
      return new YoPIOFilterDouble1D(jointName + "VelocityErrorPIO", parameters, dt, registry);
   }

   /**
    * Creates a Pilot-Induced Oscillation (PIO) filter for the error in angular velocity for an end-effector.
    * <p>
    * The filter is based on the following paper: "Phase compensation design for prevention of PIO due to actuator rate saturation" by Alcala et al.
    * </p>
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    * @param parameters      the parameters for the filter.
    * @param dt              the time step of the controller.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the PIO filter to apply to the error velocity.
    */
   public static FilterVector3D createAngularVelocityErrorPIOFilter(String endEffectorName, YoPIOParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistryLazy(registry);
      return new YoPIOFilterVector3D(endEffectorName + "AngularVelocityErrorPIO", parameters, dt, registry);
   }

   /**
    * Creates a Pilot-Induced Oscillation (PIO) filter for the error in linear velocity for an end-effector.
    * <p>
    * The filter is based on the following paper: "Phase compensation design for prevention of PIO due to actuator rate saturation" by Alcala et al.
    * </p>
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    * @param parameters      the parameters for the filter.
    * @param dt              the time step of the controller.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the PIO filter to apply to the error velocity.
    */
   public static FilterVector3D createLinearVelocityErrorPIOFilter(String endEffectorName, YoPIOParameters parameters, double dt, YoRegistry registry)
   {
      parameters.setRegistryLazy(registry);
      return new YoPIOFilterVector3D(endEffectorName + "LinearVelocityErrorPIO", parameters, dt, registry);
   }

   /**
    * Parameters for the High-Pass Rate-Limited (HPRL) filter.
    */
   public static class YoHPRLParameters
   {
      private final YoBoolean enableFilter;
      private final YoDouble breakFrequency;
      private final YoDouble maxOutputRate;
      private final YoDouble updateReferenceRatio;

      /**
       * Creates a new set of parameters for the High-Pass Rate-Limited (HPRL) filter.
       *
       * @param namePrefix           the prefix to use for the names of the parameters.
       * @param enableFilter         whether the filter is enabled or not.
       * @param breakFrequency       the break frequency of the high-pass filter. The rate-limiter is applied after the high-pass filter.
       * @param maxOutputRate        the maximum rate applied to apply on the high-pass filtered signal.
       * @param updateReferenceRatio ratio in [0, 1] used to compute the reference for the rate limiter.
       *                             A value of 0.0 means the reference is the previous high-pass filtered signal, while this reduces the phase lag, the
       *                             resulting output may drift away when the maximum rate is reached.
       *                             A value 1.0 means the last value of the rate limiter is used as the reference, while this reduces the drift, the resulting
       *                             output may have a phase lag up to about 90 degrees which can affect the damping authority.
       */
      public YoHPRLParameters(String namePrefix, boolean enableFilter, double breakFrequency, double maxOutputRate, double updateReferenceRatio)
      {
         this.enableFilter = new YoBoolean(namePrefix + "EnableFilter", null);
         this.breakFrequency = new YoDouble(namePrefix + "BreakFrequency", null);
         this.maxOutputRate = new YoDouble(namePrefix + "MaxOutputRate", null);
         this.updateReferenceRatio = new YoDouble(namePrefix + "UpdateReferenceRatio", null);
         this.enableFilter.set(enableFilter);
         this.breakFrequency.set(breakFrequency);
         this.maxOutputRate.set(maxOutputRate);
         this.updateReferenceRatio.set(updateReferenceRatio);
      }

      public void setRegistryLazy(YoRegistry registry)
      {
         if (enableFilter.getRegistry() != null)
            return;
         setRegistry(registry);
      }

      private void setRegistry(YoRegistry registry)
      {
         enableFilter.setRegistry(registry);
         breakFrequency.setRegistry(registry);
         maxOutputRate.setRegistry(registry);
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

      public double getUpdateReferenceRatio()
      {
         return updateReferenceRatio.getValue();
      }
   }

   /**
    * Parameters for the Pilot-Induced Oscillation (PIO) filter.
    */
   public static class YoPIOParameters
   {
      private final YoBoolean enableFilter;
      private final YoDouble maxInputRate;
      private final YoDouble maxInput;
      private final YoDouble leadFrequency;
      private final YoDouble lagFrequency;
      private final YoDouble updateStepBreakFrequency;
      private final YoDouble maxOutputRate;

      /**
       * Creates a new set of parameters for the Pilot-Induced Oscillation (PIO) filter.
       *
       * @param namePrefix               the prefix to use for the names of the parameters.
       * @param enableFilter             whether the filter is enabled or not.
       * @param maxInputRate             the maximum rate of change of the input signal.
       * @param maxInput                 the maximum value of the input signal.
       * @param leadFrequency            the frequency of the lead filter.
       * @param lagFrequency             the frequency of the lag filter.
       * @param updateStepBreakFrequency the break frequency of the low-pass filter used to update the step in the PIO filter.
       * @param maxOutputRate            the maximum rate of change of the output signal.
       */
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

      public void setRegistryLazy(YoRegistry registry)
      {
         if (enableFilter.getRegistry() != null)
            return;

         setRegistry(registry);
      }

      private void setRegistry(YoRegistry registry)
      {
         enableFilter.setRegistry(registry);
         maxInputRate.setRegistry(registry);
         maxInput.setRegistry(registry);
         leadFrequency.setRegistry(registry);
         lagFrequency.setRegistry(registry);
         updateStepBreakFrequency.setRegistry(registry);
         maxOutputRate.setRegistry(registry);
      }
   }

   private static class YoLPFilterDouble1D implements FilterDouble1D
   {
      private final AlphaFilteredYoVariable output;

      public YoLPFilterDouble1D(String namePrefix, DoubleProvider breakFrequency, double dt, YoRegistry registry)
      {
         DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getValue(), dt);
         output = new AlphaFilteredYoVariable(namePrefix + "Output", registry, alpha);
      }

      @Override
      public void reset()
      {
         output.reset();
      }

      @Override
      public double apply(double rawInput)
      {
         output.update(rawInput);
         return output.getDoubleValue();
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
      private final DoubleProvider updateReferenceRatio;

      private final double dt;

      public YoHPRLFilterDouble1D(String namePrefix, YoHPRLParameters parameters, double dt, YoRegistry registry)
      {
         this(namePrefix, parameters.enableFilter, parameters.breakFrequency, parameters.maxOutputRate, parameters.updateReferenceRatio, dt, registry);
      }

      public YoHPRLFilterDouble1D(String namePrefix,
                                  BooleanProvider enableFilter,
                                  DoubleProvider breakFrequency,
                                  DoubleProvider maxOutputRate,
                                  DoubleProvider updateReferenceRatio,
                                  double dt,
                                  YoRegistry registry)
      {
         this.enableFilter = enableFilter;
         this.breakFrequency = breakFrequency;
         this.maxOutputRate = maxOutputRate;
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

         double reference = EuclidCoreTools.interpolate(inputHPFPrevious, inputHPRL.getValue(), MathTools.clamp(updateReferenceRatio.getValue(), 0.0, 1.0));
         double update = inputHPF.getValue() - reference;
         inputHPRL.add(MathTools.clamp(update, maxOutputRate.getValue() * dt));
         output.set(inputLPF2.getValue() + inputHPRL.getValue());

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
   }

   private static class YoLPFilterVector3D implements FilterVector3D
   {
      private final AlphaFilteredYoMutableFrameVector3D output;

      public YoLPFilterVector3D(String namePrefix, DoubleProvider breakFrequency, double dt, YoRegistry registry)
      {
         DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getValue(), dt);
         output = new AlphaFilteredYoMutableFrameVector3D(namePrefix + "Output", "", registry, alpha, ReferenceFrame.getWorldFrame());
      }

      @Override
      public void reset()
      {
         output.reset();
      }

      @Override
      public void apply(FrameVector3DReadOnly rawInput, FixedFrameVector3DBasics filteredOutput)
      {
         if (output.getReferenceFrame() != rawInput)
         {
            reset();
            output.setReferenceFrame(rawInput.getReferenceFrame());
         }

         output.update(rawInput);
         filteredOutput.setMatchingFrame(output);
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
      private final DoubleProvider updateReferenceRatio;

      private final double dt;

      public YoHPRLFilterVector3D(String namePrefix, YoHPRLParameters parameters, double dt, YoRegistry registry)
      {
         this(namePrefix, parameters.enableFilter, parameters.breakFrequency, parameters.maxOutputRate, parameters.updateReferenceRatio, dt, registry);
      }

      public YoHPRLFilterVector3D(String namePrefix,
                                  BooleanProvider enableFilter,
                                  DoubleProvider breakFrequency,
                                  DoubleProvider maxOutputRate,
                                  DoubleProvider updateReferenceRatio,
                                  double dt,
                                  YoRegistry registry)
      {
         this.enableFilter = enableFilter;
         this.breakFrequency = breakFrequency;
         this.maxOutputRate = maxOutputRate;
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

         double alpha = MathTools.clamp(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getValue(), dt), 0.0, 1.0);
         inputLPF.interpolate(rawInput, inputLPF, alpha);
         inputLPF2.interpolate(inputLPF, inputLPF2, alpha);
         inputHPFPrevious.set(inputHPF);
         inputHPF.sub(rawInput, inputLPF2);

         reference.interpolate(inputHPFPrevious, inputHPRL, MathTools.clamp(updateReferenceRatio.getValue(), 0.0, 1.0));
         update.sub(inputHPF, reference);
         update.clipToMaxNorm(maxOutputRate.getValue() * dt);
         inputHPRL.add(update);

         output.add(inputLPF2, inputHPRL);

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
