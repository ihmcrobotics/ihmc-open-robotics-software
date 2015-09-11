package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class ThirdOrderPolynomialTrajectoryGenerator extends PolynomialTrajectoryGenerator
{
   private static final int numberOfCoefficients = 4;
   private final DoubleProvider initialVelocityProvider;

   public ThirdOrderPolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider initialVelocityProvider, 
         DoubleProvider finalPositionProvider, DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, initialPositionProvider, finalPositionProvider, trajectoryTimeProvider, numberOfCoefficients, parentRegistry);
      this.initialVelocityProvider = initialVelocityProvider;
   }

   protected void setPolynomial()
   {
      polynomial.setCubicThreeInitialConditionsFinalPosition(0.0, trajectoryTime.getDoubleValue(), initialPositionProvider.getValue(), 
            initialVelocityProvider.getValue(), 0.0, finalPositionProvider.getValue());
   }

}