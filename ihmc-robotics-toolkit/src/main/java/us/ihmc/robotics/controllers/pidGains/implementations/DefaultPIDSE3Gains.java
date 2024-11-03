package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;

/**
 * Provides a default implementation for PID gains for a SE3 PID controller.
 * <p>
 * This class uses two {@link DefaultPID3DGains}, one for position and one for orientation control,
 * internally.
 * </p>
 */
public class DefaultPIDSE3Gains implements PIDSE3Gains, Settable<DefaultPIDSE3Gains>
{
   private final DefaultPID3DGains positionGains;
   private final DefaultPID3DGains orientationGains;

   public DefaultPIDSE3Gains()
   {
      positionGains = new DefaultPID3DGains();
      orientationGains = new DefaultPID3DGains();
   }

   @Override
   public void set(DefaultPIDSE3Gains other)
   {
      PIDSE3Gains.super.set(other);
   }

   @Override
   public DefaultPID3DGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public DefaultPID3DGains getOrientationGains()
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
