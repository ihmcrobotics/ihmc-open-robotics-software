package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;

/**
 * Provides a default implementation for PID gains for a SE3 PID controller.
 * <p>
 * This class uses two {@link DefaultPID3DGains}, one for position and one for orientation control,
 * internally.
 * </p>
 */
public class ZeroablePIDSE3Gains implements PIDSE3Gains, Settable<ZeroablePIDSE3Gains>
{
   private final ZeroablePID3DGains positionGains;
   private final ZeroablePID3DGains orientationGains;

   public ZeroablePIDSE3Gains()
   {
      positionGains = new ZeroablePID3DGains();
      orientationGains = new ZeroablePID3DGains();
   }

   @Override
   public void set(ZeroablePIDSE3Gains other)
   {
      PIDSE3Gains.super.set(other);
   }

   @Override
   public ZeroablePID3DGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public ZeroablePID3DGains getOrientationGains()
   {
      return orientationGains;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PIDSE3GainsReadOnly)
         return PIDSE3Gains.super.equals((PIDSE3GainsReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return "Position: " + getPositionGains().toString() + "; Orientation: " + getOrientationGains().toString();
   }
}
