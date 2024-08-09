package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ParameterizedPDController extends AbstractPDController
{
   public ParameterizedPDController(String suffix, YoRegistry registry)
   {
      super(new DoubleParameter("kp_" + suffix, registry, 0.0),
            new DoubleParameter("kd_" + suffix, registry, 0.0),
            new DoubleParameter("positionDeadband_" + suffix, registry, 0.0),
            suffix,
            registry);
   }

   public ParameterizedPDController(DoubleProvider proportionalGain, DoubleProvider derivativeGain, String suffix, YoRegistry registry)
   {
      super(proportionalGain, derivativeGain, () -> 0.0, suffix, registry);
   }
}
