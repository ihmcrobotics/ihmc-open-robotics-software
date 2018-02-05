package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;

/**
 * Provides a default implementation for PID gains for a SE3 PID controller.
 * <p>
 * This class uses two {@link DefaultPID3DGains}, one for position and one for
 * orientation control, internally.
 * </p>
 */
public class DefaultPIDSE3Gains implements PIDSE3Gains
{
   private final DefaultPID3DGains positionGains;
   private final DefaultPID3DGains orientationGains;

   public DefaultPIDSE3Gains()
   {
      positionGains = new DefaultPID3DGains();
      orientationGains = new DefaultPID3DGains();
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
}
