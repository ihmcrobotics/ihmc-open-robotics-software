package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SymmetricParameterizedPIDSE3Gains extends ParameterizedPID3DGains
{
   public SymmetricParameterizedPIDSE3Gains(String suffix, boolean useIntegrator, YoVariableRegistry registry)
   {
      super(suffix, GainCoupling.XYZ, useIntegrator, registry);
   }

}
