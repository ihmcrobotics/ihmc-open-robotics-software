package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class CubicPolynomialTrajectoryGenerator extends PolynomialTrajectoryGenerator
{
   private static final int numberOfCoefficients = 4;

   private final DoubleProvider initialVelocityProvider;
   private final DoubleProvider finalVelocityProvider;

   public CubicPolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider finalPositionProvider,
         DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, initialPositionProvider, new ConstantDoubleProvider(0.0), finalPositionProvider, new ConstantDoubleProvider(0.0), trajectoryTimeProvider,
            parentRegistry);
   }

   public CubicPolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider initialVelocityProvider,
         DoubleProvider finalPositionProvider, DoubleProvider finalVelocityProvider, DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, initialPositionProvider, finalPositionProvider, trajectoryTimeProvider, numberOfCoefficients, parentRegistry);

      this.initialVelocityProvider = initialVelocityProvider;
      this.finalVelocityProvider = finalVelocityProvider;
   }

   protected void setPolynomial()
   {
      polynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), initialPositionProvider.getValue(), initialVelocityProvider.getValue(),
            finalPositionProvider.getValue(), finalVelocityProvider.getValue());
   }

   @Override
   public void compute(double time)
   {
      super.compute(time);

      if (isDone())
      {
         currentValue.set(finalPositionProvider.getValue());
         currentVelocity.set(finalVelocityProvider.getValue());
         currentAcceleration.set(0.0);
      }
      else if (time <= 0.0)
      {
         currentValue.set(initialPositionProvider.getValue());
         currentVelocity.set(initialVelocityProvider.getValue());
         currentAcceleration.set(0.0);
      }
   }
}
