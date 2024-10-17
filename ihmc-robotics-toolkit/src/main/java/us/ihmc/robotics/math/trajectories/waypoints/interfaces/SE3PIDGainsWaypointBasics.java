package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ZeroablePID3DGains;

public interface SE3PIDGainsWaypointBasics extends SE3PIDGainsWaypointReadOnly, Clearable
{
   void setAngular(PID3DGains angular);

   void setLinear(PID3DGains linear);

   default void set(PID3DGains angular, PID3DGains linear)
   {
      setAngular(angular);
      setLinear(linear);
   }

   default void set(SE3PIDGainsWaypointReadOnly other)
   {
      setAngular(other.getAngular());
      setLinear(other.getLinear());
   }

   @Override
   default void setToNaN()
   {
      setAngular(new DefaultPID3DGains());
      setLinear(new DefaultPID3DGains());

      getAngular().setProportionalGains(Double.NaN);
      getLinear().setProportionalGains(Double.NaN);
      getAngular().setDerivativeGains(Double.NaN);
      getLinear().setDerivativeGains(Double.NaN);

   }

   @Override
   default void setToZero()
   {
      setAngular(new ZeroablePID3DGains());
      setLinear(new ZeroablePID3DGains());
   }

   @Override
   default boolean containsNaN()
   {
      return SE3PIDGainsWaypointReadOnly.super.containsNaN();
   }
}
