package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ParameterizedPIDController extends AbstractPIDController
{
   public ParameterizedPIDController(String suffix, YoRegistry registry)
   {
      super(new DoubleParameter("kp_" + suffix, registry, 0.0),
            new DoubleParameter("ki_" + suffix, registry, 0.0),
            new DoubleParameter("kd_" + suffix, registry, 0.0),
            new DoubleParameter("positionDeadband_" + suffix, registry, 0.0),
            new DoubleParameter("maxIntegralError_" + suffix, registry, Double.POSITIVE_INFINITY),
            new DoubleParameter("maxOutput_" + suffix, registry, Double.POSITIVE_INFINITY),
            new DoubleParameter("leak_" + suffix, registry, 1.0),
            suffix,
            registry);
   }

   public ParameterizedPIDController(DoubleParameter proportionalGain, DoubleParameter integralGain, DoubleParameter derivativeGain,
                                     DoubleParameter maxIntegralError, String suffix, YoRegistry registry)
   {
      super(proportionalGain, integralGain, derivativeGain, () -> 0.0,
            maxIntegralError, () -> Double.POSITIVE_INFINITY, () -> 1.0, suffix, registry);
   }
}
