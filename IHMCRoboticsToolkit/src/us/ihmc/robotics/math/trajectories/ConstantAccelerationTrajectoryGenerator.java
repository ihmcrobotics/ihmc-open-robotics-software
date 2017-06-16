package us.ihmc.robotics.math.trajectories;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class ConstantAccelerationTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleProvider initialPositionProvider;
   private final DoubleProvider finalPositionProvider;
   private final DoubleProvider initialVelocityProvider;
   private final YoPolynomial polynomial;
   private final YoDouble trajectoryTime;
   private final DoubleProvider trajectoryTimeProvider;
   private final YoDouble currentTime;
   private final int numberOfCoefficients = 3;

   public ConstantAccelerationTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider finalPositionProvider,
         DoubleProvider initialVelocityProvider, DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.initialPositionProvider = initialPositionProvider;
      this.finalPositionProvider = finalPositionProvider;
      this.initialVelocityProvider = initialVelocityProvider;
      this.polynomial = new YoPolynomial(namePrefix + "Polynomial", numberOfCoefficients, registry);
      this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      this.trajectoryTimeProvider = trajectoryTimeProvider;
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      currentTime.set(0.0);
      this.trajectoryTime.set(trajectoryTimeProvider.getValue());
      polynomial.setQuadratic(0.0, trajectoryTime.getDoubleValue(), initialPositionProvider.getValue(), initialVelocityProvider.getValue(),
            finalPositionProvider.getValue());
   }

   public void compute(double time)
   {
      this.currentTime.set(time);
      polynomial.compute(time);
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() > trajectoryTime.getDoubleValue();
   }

   public double getValue()
   {
      return polynomial.getPosition();
   }

   public double getVelocity()
   {
      return polynomial.getVelocity();
   }

   public double getAcceleration()
   {
      return polynomial.getAcceleration();
   }
}