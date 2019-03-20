package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@link YoParameter} based implementation of the {@link PIDGainsReadOnly} interface.
 * <p>
 * Each parameter is backed by a {@link DoubleParameter}. This means these gains are only tunable
 * through a parameter tuning application and are not mutable from code. Default values can be
 * specified at construction time.
 * </p>
 */
public class ParameterizedPIDGains extends ParameterizedPDGains implements PIDGainsReadOnly
{
   private final DoubleParameter ki;
   private final DoubleParameter maxIntegralError;
   private final DoubleParameter integralLeakRatio;

   public ParameterizedPIDGains(String suffix, YoVariableRegistry registry)
   {
      this(suffix, null, registry);
   }

   public ParameterizedPIDGains(String suffix, PIDGainsReadOnly defaults, YoVariableRegistry registry)
   {
      super(suffix, defaults, registry);

      if (defaults == null)
      {
         ki = new DoubleParameter("ki" + suffix, registry, 0.0);
         maxIntegralError = new DoubleParameter("maxIntegralError" + suffix, registry, Double.POSITIVE_INFINITY);
         integralLeakRatio = new DoubleParameter("integralLeakRatio" + suffix, registry, 1.0);
      }
      else
      {
         ki = new DoubleParameter("ki" + suffix, registry, defaults.getKi());
         maxIntegralError = new DoubleParameter("maxIntegralError" + suffix, registry, defaults.getMaxIntegralError());
         integralLeakRatio = new DoubleParameter("integralLeakRatio" + suffix, registry, defaults.getIntegralLeakRatio());
      }
   }

   public ParameterizedPIDGains(String suffix, PDGainsReadOnly defaults, YoVariableRegistry registry)
   {
      super(suffix, defaults, registry);

      ki = new DoubleParameter("ki" + suffix, registry, 0.0);
      maxIntegralError = new DoubleParameter("maxIntegralError" + suffix, registry, Double.POSITIVE_INFINITY);
      integralLeakRatio = new DoubleParameter("integralLeakRatio" + suffix, registry, 1.0);
   }

   @Override
   public double getKi()
   {
      return ki.getValue();
   }

   @Override
   public double getMaxIntegralError()
   {
      return maxIntegralError.getValue();
   }

   @Override
   public double getIntegralLeakRatio()
   {
      return integralLeakRatio.getValue();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PIDGainsReadOnly)
         return PIDGainsReadOnly.super.equals((PIDGainsReadOnly) object);
      else
         return false;
   }
}
