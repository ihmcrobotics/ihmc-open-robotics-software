package us.ihmc.robotics.math.trajectories;

import org.apache.commons.math3.util.Precision;

import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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

   @Override
   protected void setPolynomial()
   {
      if (Precision.equals(0.0, trajectoryTime.getDoubleValue()))
      {
         polynomial.setLinear(0.0, initialPositionProvider.getValue(), initialVelocityProvider.getValue());
      }
      else
      {
         polynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPositionProvider.getValue(), initialVelocityProvider.getValue(), 0.0,
                                    finalPositionProvider.getValue(), finalVelocityProvider.getValue(), 0.0);
      }
   }
}
