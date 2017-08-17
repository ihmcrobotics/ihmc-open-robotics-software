package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class DefaultYoPIDSE3Gains implements YoPIDSE3Gains
{
   private final YoPID3DGains positionGains;
   private final YoPID3DGains orientationGains;

   public DefaultYoPIDSE3Gains(YoPID3DGains positionGains, YoPID3DGains orientationGains)
   {
      this.positionGains = positionGains;
      this.orientationGains = orientationGains;
   }

   public DefaultYoPIDSE3Gains(String prefix, PIDSE3Gains other, YoVariableRegistry registry)
   {
      this(prefix, other.getPositionGains(), other.getOrientationGains(), registry);
   }

   public DefaultYoPIDSE3Gains(String prefix, PID3DGainsReadOnly positionGains, PID3DGainsReadOnly orientationGains, YoVariableRegistry registry)
   {
      this(prefix, positionGains.getGainCoupling(), orientationGains.getGainCoupling(), positionGains.isUseIntegrator(), orientationGains.isUseIntegrator(),
           registry);
      setPositionGains(positionGains);
      setOrientationGains(orientationGains);
   }

   public DefaultYoPIDSE3Gains(String prefix, GainCoupling gainCoupling, boolean useIntegrator, YoVariableRegistry registry)
   {
      this(prefix, gainCoupling, gainCoupling, useIntegrator, useIntegrator, registry);
   }

   public DefaultYoPIDSE3Gains(String prefix, GainCoupling gainCouplingPosition, GainCoupling gainCouplingOrientation, boolean useIntegratorPosition,
                               boolean useIntegratorOrientation, YoVariableRegistry registry)
   {
      positionGains = new DefaultYoPID3DGains(prefix + "Position", gainCouplingPosition, useIntegratorPosition, registry);
      orientationGains = new DefaultYoPID3DGains(prefix + "Orientation", gainCouplingOrientation, useIntegratorOrientation, registry);
   }

   @Override
   public YoPID3DGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public YoPID3DGains getOrientationGains()
   {
      return orientationGains;
   }
}
