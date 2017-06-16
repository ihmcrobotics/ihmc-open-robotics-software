package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

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

   public void initialize()
   {
      currentTime.set(0.0);
      this.trajectoryTime.set(trajectoryTimeProvider.getValue());
      setPolynomial();
      compute(0.0);
   }

   protected abstract void setPolynomial();

   public void compute(double time)
   {
      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      polynomial.compute(time);
      currentValue.set(polynomial.getPosition());
      currentVelocity.set(polynomial.getVelocity());
      currentAcceleration.set(polynomial.getAcceleration());
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public double getValue()
   {
      return currentValue.getDoubleValue();
   }

   public double getVelocity()
   {
      return currentVelocity.getDoubleValue();
   }

   public double getAcceleration()
   {
      return currentAcceleration.getDoubleValue();
   }
}
