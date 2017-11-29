package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SymmetricParameterizedPIDSE3Gains extends ParameterizedPIDSE3Gains
{
   public SymmetricParameterizedPIDSE3Gains(String suffix, boolean useIntegrator, YoVariableRegistry registry)
   {
      super(suffix, GainCoupling.XYZ, GainCoupling.XYZ, useIntegrator, useIntegrator, registry);
   }

}
