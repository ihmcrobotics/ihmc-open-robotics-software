package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnessesReadOnly;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnessesReadOnly;
import us.ihmc.robotics.controllers.pidGains.PDSE3StiffnessesReadOnly;

public class PDSE3Configuration
{
   private final PD3DConfiguration positionConfiguration;

   private final PD3DConfiguration orientationConfiguration;

   public PDSE3Configuration(GainCoupling gainCoupling)
   {
      this(gainCoupling, null, null);
   }

   public PDSE3Configuration(GainCoupling gainCoupling, PDSE3StiffnessesReadOnly gains)
   {
      this(gainCoupling, gains.getPositionStiffnesses(), gains.getOrientationStiffnesses());
   }

   public PDSE3Configuration(GainCoupling gainCoupling, PD3DStiffnessesReadOnly positionGains, PD3DStiffnessesReadOnly orientationGains)
   {
      this.positionConfiguration = new PD3DConfiguration(gainCoupling, positionGains);
      this.orientationConfiguration = new PD3DConfiguration(gainCoupling,  orientationGains);
   }

   public PD3DConfiguration getPositionConfiguration()
   {
      return positionConfiguration;
   }

   public PD3DConfiguration getOrientationConfiguration()
   {
      return orientationConfiguration;
   }
}
