package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;

public class DefaultPIDSE3Gains implements PIDSE3Gains
{
   private final DefaultPID3DGains positionGains;
   private final DefaultPID3DGains orientationGains;

   public DefaultPIDSE3Gains()
   {
      this(GainCoupling.NONE, true);
   }

   public DefaultPIDSE3Gains(GainCoupling gainCoupling, boolean useIntegrator)
   {
      this(gainCoupling, gainCoupling, useIntegrator, useIntegrator);
   }

   public DefaultPIDSE3Gains(GainCoupling gainCouplingPosition, GainCoupling gainCouplingOrientation, boolean useIntegratorPosition,
                             boolean useIntegratorOrientation)
   {
      positionGains = new DefaultPID3DGains(gainCouplingPosition, useIntegratorPosition);
      orientationGains = new DefaultPID3DGains(gainCouplingOrientation, useIntegratorOrientation);
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
