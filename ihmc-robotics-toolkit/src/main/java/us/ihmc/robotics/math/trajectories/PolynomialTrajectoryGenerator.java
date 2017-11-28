package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class PolynomialTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   protected final DoubleProvider initialPositionProvider;
   protected final DoubleProvider finalPositionProvider;
   protected final YoPolynomial polynomial;
   protected final YoDouble trajectoryTime;
   protected final DoubleProvider trajectoryTimeProvider;
   protected final YoDouble currentTime;
   protected final YoDouble currentValue;
   protected final YoDouble currentVelocity;
   protected final YoDouble currentAcceleration;

   public PolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider finalPositionProvider,
         DoubleProvider trajectoryTimeProvider, int numberOfCoefficients, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.initialPositionProvider = initialPositionProvider;
      this.finalPositionProvider = finalPositionProvider;
      this.polynomial = new YoPolynomial(namePrefix + "Polynomial", numberOfCoefficients, registry);
      this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      this.currentValue = new YoDouble(namePrefix + "CurrentValue", registry);
      this.currentVelocity = new YoDouble(namePrefix + "CurrentVelocity", registry);
      this.currentAcceleration = new YoDouble(namePrefix + "CurrentAcceleration", registry);
      this.trajectoryTimeProvider = trajectoryTimeProvider;
      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      currentTime.set(0.0);
      this.trajectoryTime.set(trajectoryTimeProvider.getValue());
      setPolynomial();
      compute(0.0);
   }

   protected abstract void setPolynomial();

   @Override
   public void compute(double time)
   {
      if (Double.isNaN(time))
      {
         throw new RuntimeException("Can not call compute on trajectory generator with time NaN.");
      }

      this.currentTime.set(time);

      if (time < 0.0)
      {
         currentValue.set(initialPositionProvider.getValue());
         currentVelocity.set(0.0);
         currentAcceleration.set(0.0);
         return;
      }
      if (time > trajectoryTime.getDoubleValue())
      {
         currentValue.set(finalPositionProvider.getValue());
         currentVelocity.set(0.0);
         currentAcceleration.set(0.0);
         return;
      }

      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      polynomial.compute(time);
      currentValue.set(polynomial.getPosition());
      currentVelocity.set(polynomial.getVelocity());
      currentAcceleration.set(polynomial.getAcceleration());
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public double getValue()
   {
      return currentValue.getDoubleValue();
   }

   @Override
   public double getVelocity()
   {
      return currentVelocity.getDoubleValue();
   }

   @Override
   public double getAcceleration()
   {
      return currentAcceleration.getDoubleValue();
   }
}
