package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JerkLimitedYoVariable extends YoDouble
{
   private final double dt;

   private final YoBoolean hasBeenInitialized;

   private final YoDouble smoothedRate;
   private final YoDouble smoothedAcceleration;
   private final YoDouble smoothedJerk;

   private final YoDouble positionGain;
   private final YoDouble velocityGain;
   private final YoDouble accelerationGain;

   private final YoDouble maximumJerk;
   private final YoDouble maximumAcceleration;

   private final YoDouble inputPosition;
   private final YoDouble inputVelocity;
   private final YoDouble inputAcceleration;

   public JerkLimitedYoVariable(String name, YoVariableRegistry registry, YoDouble maxAcceleration, YoDouble maxJerk, double dt)
   {
      this(name, registry, maxAcceleration, maxJerk, null, null, null, dt);
   }

   public JerkLimitedYoVariable(String name, YoVariableRegistry registry, YoDouble maxAcceleration, YoDouble maxJerk,
         YoDouble inputPosition, double dt)
   {
      this(name, registry, maxAcceleration, maxJerk, inputPosition, null, null, dt);
   }

   public JerkLimitedYoVariable(String name, YoVariableRegistry registry, YoDouble maxAcceleration, YoDouble maxJerk,
         YoDouble inputPosition, YoDouble inputVelocity, double dt)
   {
      this(name, registry, maxAcceleration, maxJerk, inputPosition, inputVelocity, null, dt);
   }

   public JerkLimitedYoVariable(String name, YoVariableRegistry registry, YoDouble maxAcceleration, YoDouble maxJerk,
         YoDouble inputPosition, YoDouble inputVelocity, YoDouble inputAcceleration, double dt)
   {
      super(name, registry);

      this.inputPosition = inputPosition;
      this.inputVelocity = inputVelocity;
      this.inputAcceleration = inputAcceleration;

      this.maximumJerk = maxJerk;
      this.maximumAcceleration = maxAcceleration;

      this.dt = dt;

      hasBeenInitialized = new YoBoolean(name + "HasBeenInitialized", registry);

      smoothedRate = new YoDouble(name + "SmoothedRate", registry);
      smoothedAcceleration = new YoDouble(name + "SmoothedAcceleration", registry);
      smoothedJerk = new YoDouble(name + "SmoothedJerk", registry);

      positionGain = new YoDouble(name + "PositionGain", registry);
      velocityGain = new YoDouble(name + "VelocityGain", registry);
      accelerationGain = new YoDouble(name + "AccelerationGain", registry);

      double w0 = 2.0 * Math.PI * 16.0;
      double w1 = 2.0 * Math.PI * 16.0;
      double zeta = 1.0;

      setGainsByPolePlacement(w0, w1, zeta);
      hasBeenInitialized.set(false);
   }

   public void setMaximumAcceleration(double maximumAcceleration)
   {
      this.maximumAcceleration.set(maximumAcceleration);
   }

   public void setMaximumJerk(double maximumJerk)
   {
      this.maximumJerk.set(maximumJerk);
   }

   public void setGainsByPolePlacement(double w0, double w1, double zeta)
   {
      positionGain.set(w0 * w1 * w1);
      velocityGain.set(w1 * w1 + 2.0 * zeta * w1 * w0);
      accelerationGain.set(w0 + 2.0 * zeta * w1);
   }

   public void update()
   {
      double inputPosition = this.inputPosition == null ? 0.0 : this.inputPosition.getDoubleValue();
      double inputVelocity = this.inputVelocity == null ? 0.0 : this.inputVelocity.getDoubleValue();
      double inputAcceleration = this.inputAcceleration == null ? 0.0 : this.inputAcceleration.getDoubleValue();

      update(inputPosition, inputVelocity, inputAcceleration);
   }

   public void update(double inputPosition)
   {
      update(inputPosition, 0.0, 0.0);
   }

   public void update(double inputPosition, double inputVelocity)
   {
      update(inputPosition, inputVelocity, 0.0);
   }

   public void update(double inputPosition, double inputVelocity, double inputAcceleration)
   {
      if (!hasBeenInitialized.getBooleanValue())
         initialize(inputPosition, inputVelocity, inputAcceleration);

      double positionError = inputPosition - this.getDoubleValue();
      double velocityError = inputVelocity - smoothedRate.getDoubleValue();
      double accelerationError = inputAcceleration - smoothedAcceleration.getDoubleValue();
      double jerk = accelerationGain.getDoubleValue() * accelerationError + velocityGain.getDoubleValue() * velocityError + positionGain.getDoubleValue()
            * positionError;
      jerk = MathTools.clamp(jerk, -maximumJerk.getDoubleValue(), maximumJerk.getDoubleValue());

      smoothedJerk.set(jerk);
      smoothedAcceleration.add(smoothedJerk.getDoubleValue() * dt);
      smoothedAcceleration.set(MathTools.clamp(smoothedAcceleration.getDoubleValue(), maximumJerk.getDoubleValue()));
      smoothedRate.add(smoothedAcceleration.getDoubleValue() * dt);
      this.add(smoothedRate.getDoubleValue() * dt);
   }

   public void initialize(double inputPosition, double inputVelocity, double inputAcceleration)
   {
      this.set(inputPosition);
      smoothedRate.set(inputVelocity);
      smoothedAcceleration.set(inputAcceleration);
      smoothedJerk.set(0.0);

      this.hasBeenInitialized.set(true);
   }

   public void reset()
   {
      this.hasBeenInitialized.set(false);
      smoothedRate.set(0.0);
      smoothedAcceleration.set(0.0);
      smoothedJerk.set(0.0);
   }

}
