package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public abstract class PolynomialTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   protected final DoubleProvider initialPositionProvider;
   protected final DoubleProvider finalPositionProvider;
   protected final YoPolynomial polynomial;
   protected final DoubleYoVariable trajectoryTime;
   protected final DoubleProvider trajectoryTimeProvider;
   protected final DoubleYoVariable currentTime;
   protected final DoubleYoVariable currentValue;
   protected final DoubleYoVariable currentVelocity;
   protected final DoubleYoVariable currentAcceleration;

   public PolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider finalPositionProvider,
         DoubleProvider trajectoryTimeProvider, int numberOfCoefficients, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.initialPositionProvider = initialPositionProvider;
      this.finalPositionProvider = finalPositionProvider;
      this.polynomial = new YoPolynomial(namePrefix + "Polynomial", numberOfCoefficients, registry);
      this.trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new DoubleYoVariable(namePrefix + "CurrentTime", registry);
      this.currentValue = new DoubleYoVariable(namePrefix + "CurrentValue", registry);
      this.currentVelocity = new DoubleYoVariable(namePrefix + "CurrentVelocity", registry);
      this.currentAcceleration = new DoubleYoVariable(namePrefix + "CurrentAcceleration", registry);
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
