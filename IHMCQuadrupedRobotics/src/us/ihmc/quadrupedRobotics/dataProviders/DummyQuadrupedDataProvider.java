package us.ihmc.quadrupedRobotics.dataProviders;

import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class DummyQuadrupedDataProvider implements QuadrupedDataProvider
{

   public DummyQuadrupedDataProvider()
   {
   }

   @Override
   public VectorProvider getDesiredVelocityProvider()
   {
      return null;
   }

   @Override
   public DoubleProvider getDesiredYawRateProvider()
   {
      return null;
   }
}
