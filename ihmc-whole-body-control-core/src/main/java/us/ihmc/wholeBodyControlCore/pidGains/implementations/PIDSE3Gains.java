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
public class PIDSE3Gains implements PIDSE3GainsBasics
{
   private final PID3DGains positionGains;
   private final PID3DGains orientationGains;

   public PIDSE3Gains()
   {
      positionGains = new PID3DGains();
      orientationGains = new PID3DGains();
   }

   @Override
   public PID3DGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public PID3DGains getOrientationGains()
   {
      return orientationGains;
   }

   public void setPositionDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      getPositionGains().setDampingRatios(dampingRatioX, dampingRatioY, dampingRatioZ);
   }

   public void setOrientationDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      getOrientationGains().setDampingRatios(dampingRatioX, dampingRatioY, dampingRatioZ);
   }

   public void setPositionDampingRatios(double dampingRatio)
   {
      getPositionGains().setDampingRatios(dampingRatio);
   }

   public void setOrientationDampingRatios(double dampingRatio)
   {
      getOrientationGains().setDampingRatios(dampingRatio);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PIDSE3GainsReadOnly)
         return PIDSE3GainsBasics.super.equals((PIDSE3GainsReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return "Position: " + getPositionGains().toString() + "; Orientation: " + getOrientationGains().toString();
   }
}
