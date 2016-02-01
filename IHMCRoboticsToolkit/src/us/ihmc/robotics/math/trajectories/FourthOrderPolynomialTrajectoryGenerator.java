package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class FourthOrderPolynomialTrajectoryGenerator extends PolynomialTrajectoryGenerator
{
   private static final int numberOfCoefficients = 5;
   private final DoubleProvider initialVelocityProvider;
   private final DoubleProvider finalVelocityProvider;

   public FourthOrderPolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider initialVelocityProvider, 
         DoubleProvider finalPositionProvider, DoubleProvider finalVelocityProvider, DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, initialPositionProvider, finalPositionProvider, trajectoryTimeProvider, numberOfCoefficients, parentRegistry);
      this.initialVelocityProvider = initialVelocityProvider;
      this.finalVelocityProvider = finalVelocityProvider;
   }

   protected void setPolynomial()
   {
      polynomial.setQuartic(0.0, trajectoryTime.getDoubleValue(), initialPositionProvider.getValue(), initialVelocityProvider.getValue(), 0.0, finalPositionProvider.getValue(), finalVelocityProvider.getValue());
   }
}