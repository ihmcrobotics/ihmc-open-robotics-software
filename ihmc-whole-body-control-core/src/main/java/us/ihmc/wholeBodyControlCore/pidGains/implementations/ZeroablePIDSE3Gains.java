package us.ihmc.wholeBodyControlCore.pidGains.implementations;

import us.ihmc.wholeBodyControlCore.pidGains.PIDSE3GainsBasics;
import us.ihmc.wholeBodyControlCore.pidGains.PIDSE3GainsReadOnly;

/**
 * Provides a default implementation for PID gains for a SE3 PID controller.
 * <p>
 * This class uses two {@link PID3DGains}, one for position and one for orientation control,
 * internally.
 * </p>
 */
public class ZeroablePIDSE3Gains implements PIDSE3GainsBasics
{
   private final ZeroablePID3DGains positionGains;
   private final ZeroablePID3DGains orientationGains;

   public ZeroablePIDSE3Gains()
   {
      positionGains = new ZeroablePID3DGains();
      orientationGains = new ZeroablePID3DGains();
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
      if (object instanceof PIDSE3GainsReadOnly other)
         return PIDSE3GainsBasics.super.equals(other);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return "Position: " + getPositionGains().toString() + "; Orientation: " + getOrientationGains().toString();
   }
}
