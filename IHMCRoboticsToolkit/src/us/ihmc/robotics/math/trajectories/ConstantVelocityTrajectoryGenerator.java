package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;


public class ConstantVelocityTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleProvider initialPositionProvider;
   private final DoubleProvider velocityProvider;
   private final YoPolynomial polynomial;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleProvider trajectoryTimeProvider;
   private final DoubleYoVariable currentTime;
   private final int numberOfCoefficients = 2;

   public ConstantVelocityTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider,
           DoubleProvider velocityProvider, DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.initialPositionProvider = initialPositionProvider;
      this.velocityProvider = velocityProvider;
      this.polynomial = new YoPolynomial(namePrefix + "Polynomial", numberOfCoefficients, registry);
      this.trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new DoubleYoVariable(namePrefix + "CurrentTime", registry);
      this.trajectoryTimeProvider = trajectoryTimeProvider;
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      currentTime.set(0.0);
      this.trajectoryTime.set(trajectoryTimeProvider.getValue());
      polynomial.setLinear(0.0, initialPositionProvider.getValue(), velocityProvider.getValue());
   }

   public void compute(double time)
   {
      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
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
      return 0.0;
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": x = " + getValue() + ", xdot = " + getVelocity() + "xddot = " + getAcceleration();
   }
}
