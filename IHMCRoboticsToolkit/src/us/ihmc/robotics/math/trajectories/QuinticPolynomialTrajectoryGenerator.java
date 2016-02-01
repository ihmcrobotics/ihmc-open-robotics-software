package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class QuinticPolynomialTrajectoryGenerator extends PolynomialTrajectoryGenerator
{
   private static final int numberOfCoefficients = 6;
   private final DoubleProvider initialVelocityProvider;
   private final DoubleProvider finalVelocityProvider;

   public QuinticPolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider initialVelocityProvider,
         DoubleProvider finalPositionProvider, DoubleProvider finalVelocityProvider, DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, initialPositionProvider, finalPositionProvider, trajectoryTimeProvider, numberOfCoefficients, parentRegistry);
      this.initialVelocityProvider = initialVelocityProvider;
      this.finalVelocityProvider = finalVelocityProvider;
   }

   protected void setPolynomial()
   {
      this.polynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPositionProvider.getValue(), initialVelocityProvider.getValue(), 0.0,
            finalPositionProvider.getValue(), finalVelocityProvider.getValue(), 0.0);
   }
}
