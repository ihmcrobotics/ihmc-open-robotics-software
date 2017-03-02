package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class AccelerationLimitedYoVariable extends DoubleYoVariable
{
   private final double dt;

   private final BooleanYoVariable hasBeenInitialized;

   private final DoubleYoVariable smoothedRate;
   private final DoubleYoVariable smoothedAcceleration;

   private final DoubleYoVariable positionGain;
   private final DoubleYoVariable velocityGain;

   private DoubleYoVariable maximumRate;
   private DoubleYoVariable maximumAcceleration;

   private final DoubleYoVariable inputVariable;

   public AccelerationLimitedYoVariable(String name, YoVariableRegistry registry, double maxRate, double maxAcceleration, DoubleYoVariable inputVariable, double dt)
   {
      this(name, registry, null, null, inputVariable, dt);

      maximumRate = new DoubleYoVariable(name + "MaxRate", registry);
      maximumAcceleration = new DoubleYoVariable(name + "MaxAcceleration", registry);

      maximumRate.set(maxRate);
      maximumAcceleration.set(maxAcceleration);
   }

   public AccelerationLimitedYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable maxRate, DoubleYoVariable maxAcceleration, double dt)
   {
      this(name, registry, maxRate, maxAcceleration, null, dt);
   }

   public AccelerationLimitedYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable maxRate, DoubleYoVariable maxAcceleration,
         DoubleYoVariable inputVariable, double dt)
   {
      super(name, registry);

      if (maxRate != null && maxAcceleration != null)
      {
         this.maximumRate = maxRate;
         this.maximumAcceleration = maxAcceleration;
      }

      this.dt = dt;

      hasBeenInitialized = new BooleanYoVariable(name + "HasBeenInitialized", registry);

      smoothedRate = new DoubleYoVariable(name + "SmoothedRate", registry);
      smoothedAcceleration = new DoubleYoVariable(name + "SmoothedAcceleration", registry);

      positionGain = new DoubleYoVariable(name + "PositionGain", registry);
      velocityGain = new DoubleYoVariable(name + "VelocityGain", registry);

      double w0 = 2.0 * Math.PI * 16.0;
      double zeta = 1.0;

      setGainsByPolePlacement(w0, zeta);
      hasBeenInitialized.set(false);

      this.inputVariable = inputVariable;
   }

   public void setMaximumAcceleration(double maximumAcceleration)
   {
      this.maximumAcceleration.set(maximumAcceleration);
   }

   public void setMaximumRate(double maximumRate)
   {
      this.maximumRate.set(maximumRate);
   }

   public void setGainsByPolePlacement(double w0, double zeta)
   {
      positionGain.set(w0 * w0);
      velocityGain.set(2.0 * zeta * w0);
   }
   
   public DoubleYoVariable getPositionGain()
   {
      return positionGain;
   }
   
   public DoubleYoVariable getVelocityGain()
   {
      return velocityGain;
   }

   public void update()
   {
      update(inputVariable.getDoubleValue());
   }

   public void update(double input)
   {
      if (!hasBeenInitialized.getBooleanValue())
         initialize(input);

      double positionError = input - this.getDoubleValue();
      double acceleration = -velocityGain.getDoubleValue() * smoothedRate.getDoubleValue() + positionGain.getDoubleValue() * positionError;
      acceleration = MathTools.clamp(acceleration, -maximumAcceleration.getDoubleValue(), maximumAcceleration.getDoubleValue());

      smoothedAcceleration.set(acceleration);
      smoothedRate.add(smoothedAcceleration.getDoubleValue() * dt);
      smoothedRate.set(MathTools.clamp(smoothedRate.getDoubleValue(), maximumRate.getDoubleValue()));
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

   public DoubleYoVariable getSmoothedRate()
   {
      return smoothedRate;
   }

   public DoubleYoVariable getSmoothedAcceleration()
   {
      return smoothedAcceleration;
   }

   public boolean hasBeenInitialized()
   {
      return hasBeenInitialized.getBooleanValue();
   }
   
   public double getMaximumRate()
   {
      return maximumRate.getDoubleValue();
   }
   
   public double getMaximumAcceleration()
   {
      return maximumAcceleration.getDoubleValue();
   }
}