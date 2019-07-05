package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class AccelerationLimitedYoVariable extends YoDouble
{
   private final double dt;

   private final YoBoolean hasBeenInitialized;

   private final YoDouble smoothedRate;
   private final YoDouble smoothedAcceleration;

   private final YoDouble positionGain;
   private final YoDouble velocityGain;

   private DoubleProvider maximumRate;
   private DoubleProvider maximumAcceleration;

   private final DoubleProvider inputVariable;

   public AccelerationLimitedYoVariable(String name, YoVariableRegistry registry, DoubleProvider maxRate, DoubleProvider maxAcceleration, double dt)
   {
      this(name, registry, maxRate, maxAcceleration, null, dt);
   }

   public AccelerationLimitedYoVariable(String name, YoVariableRegistry registry, DoubleProvider maxRate, DoubleProvider maxAcceleration,
                                        DoubleProvider inputVariable, double dt)
   {
      super(name, registry);

      if (maxRate != null && maxAcceleration != null)
      {
         this.maximumRate = maxRate;
         this.maximumAcceleration = maxAcceleration;
      }

      this.dt = dt;

      hasBeenInitialized = new YoBoolean(name + "HasBeenInitialized", registry);

      smoothedRate = new YoDouble(name + "SmoothedRate", registry);
      smoothedAcceleration = new YoDouble(name + "SmoothedAcceleration", registry);

      positionGain = new YoDouble(name + "PositionGain", registry);
      velocityGain = new YoDouble(name + "VelocityGain", registry);

      double w0 = 2.0 * Math.PI * 16.0;
      double zeta = 1.0;

      setGainsByPolePlacement(w0, zeta);
      hasBeenInitialized.set(false);

      this.inputVariable = inputVariable;
   }


   public void setGainsByPolePlacement(double w0, double zeta)
   {
      positionGain.set(w0 * w0);
      velocityGain.set(2.0 * zeta * w0);
   }
   
   public YoDouble getPositionGain()
   {
      return positionGain;
   }
   
   public YoDouble getVelocityGain()
   {
      return velocityGain;
   }

   public void update()
   {
      update(inputVariable.getValue());
   }

   public void update(double input)
   {
      if (!hasBeenInitialized.getBooleanValue())
         initialize(input);

      double positionError = input - this.getDoubleValue();
      double acceleration = -velocityGain.getDoubleValue() * smoothedRate.getDoubleValue() + positionGain.getDoubleValue() * positionError;
      acceleration = MathTools.clamp(acceleration, -maximumAcceleration.getValue(), maximumAcceleration.getValue());

      smoothedAcceleration.set(acceleration);
      smoothedRate.add(smoothedAcceleration.getDoubleValue() * dt);
      smoothedRate.set(MathTools.clamp(smoothedRate.getDoubleValue(), maximumRate.getValue()));
      this.add(smoothedRate.getDoubleValue() * dt);
   }

   public void initialize(double input)
   {
      this.set(input);
      smoothedRate.set(0.0);
      smoothedAcceleration.set(0.0);

      this.hasBeenInitialized.set(true);
   }

   public void reset()
   {
      this.hasBeenInitialized.set(false);
      smoothedRate.set(0.0);
      smoothedAcceleration.set(0.0);
   }

   public YoDouble getSmoothedRate()
   {
      return smoothedRate;
   }

   public YoDouble getSmoothedAcceleration()
   {
      return smoothedAcceleration;
   }

   public boolean hasBeenInitialized()
   {
      return hasBeenInitialized.getBooleanValue();
   }
   
   public double getMaximumRate()
   {
      return maximumRate.getValue();
   }
   
   public double getMaximumAcceleration()
   {
      return maximumAcceleration.getValue();
   }
}