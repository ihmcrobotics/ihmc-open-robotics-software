package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class CubicPolynomialTrajectoryGenerator extends PolynomialTrajectoryGenerator
{
   private static final int numberOfCoefficients = 4;

   public CubicPolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider,
           DoubleProvider finalPositionProvider, DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, initialPositionProvider, finalPositionProvider, trajectoryTimeProvider, numberOfCoefficients, parentRegistry);
   }

   protected void setPolynomial()
   {
      polynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), initialPositionProvider.getValue(), 0.0, finalPositionProvider.getValue(), 0.0);
   }

   @Override
   public void compute(double time)
   {
      super.compute(time);

      if (isDone())
      {
         currentValue.set(finalPositionProvider.getValue());
         currentVelocity.set(0.0);
         currentAcceleration.set(0.0);
      }
   }
}

