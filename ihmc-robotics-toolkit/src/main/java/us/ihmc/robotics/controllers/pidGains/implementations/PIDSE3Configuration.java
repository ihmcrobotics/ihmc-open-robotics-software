package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;

public class PIDSE3Configuration
{
   private final PID3DConfiguration positionConfiguration;

   private final PID3DConfiguration orientationConfiguration;

   public PIDSE3Configuration(GainCoupling gainCoupling, boolean useIntegrator)
   {
      this(gainCoupling, useIntegrator, null, null);
   }

   public PIDSE3Configuration(GainCoupling gainCoupling, boolean useIntegrator, PIDSE3GainsReadOnly gains)
   {
      this(gainCoupling, useIntegrator, gains.getPositionGains(), gains.getOrientationGains());
   }

   public PIDSE3Configuration(GainCoupling gainCoupling, boolean useIntegrator, PID3DGainsReadOnly positionGains, PID3DGainsReadOnly orientationGains)
   {
      this.positionConfiguration = new PID3DConfiguration(gainCoupling, useIntegrator, positionGains);
      this.orientationConfiguration = new PID3DConfiguration(gainCoupling, useIntegrator, orientationGains);
   }

   public PID3DConfiguration getPositionConfiguration()
   {
      return positionConfiguration;
   }

   public PID3DConfiguration getOrientationConfiguration()
   {
      return orientationConfiguration;
   }
}
