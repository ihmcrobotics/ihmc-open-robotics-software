package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterizedPDController extends AbstractPDController
{

   private final DoubleProvider proportionalGain;
   private final DoubleProvider derivativeGain;
   private final DoubleProvider positionDeadband;

   public ParameterizedPDController(String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);

      proportionalGain = new DoubleParameter("kp_" + suffix, registry, 0.0);
      derivativeGain = new DoubleParameter("kd_" + suffix, registry, 0.0);
      positionDeadband = new DoubleParameter("positionDeadband_" + suffix, registry, 0.0);
   }

   public ParameterizedPDController(DoubleProvider proportionalGain, DoubleProvider derivativeGain, DoubleProvider positionDeadband, String suffix,
                                    YoVariableRegistry registry)
   {
      super(suffix, registry);

      this.proportionalGain = proportionalGain;
      this.derivativeGain = derivativeGain;
      this.positionDeadband = positionDeadband;
   }

   public ParameterizedPDController(DoubleProvider proportionalGain, DoubleProvider derivativeGain, String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);

      this.proportionalGain = proportionalGain;
      this.derivativeGain = derivativeGain;
      this.positionDeadband = () -> 0.0;
   }

   @Override
   public double getProportionalGain()
   {
      return proportionalGain.getValue();
   }

   @Override
   public double getDerivativeGain()
   {
      return derivativeGain.getValue();
   }

   @Override
   public double getPositionDeadband()
   {
      return positionDeadband.getValue();
   }
}
