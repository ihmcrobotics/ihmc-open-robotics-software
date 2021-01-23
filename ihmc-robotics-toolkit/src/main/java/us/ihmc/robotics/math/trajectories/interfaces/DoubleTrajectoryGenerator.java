package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.yoVariables.providers.DoubleProvider;

public interface DoubleTrajectoryGenerator extends TrajectoryGenerator, DoubleProvider
{
   double getVelocity();
   double getAcceleration();

   /**
    * Use the {@link #getValue()} method instead.
    */
   @Deprecated
   default double getPosition()
   {
      return getValue();
   }
}
