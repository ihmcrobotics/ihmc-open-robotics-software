package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@link YoParameter} based implementation of the {@link PDGainsReadOnly} interface.
 * <p>
 * Each parameter is backed by a {@link DoubleParameter}. This means these gains are only
 * tunable through a parameter tuning application and are not mutable from code. Default
 * values can be specified at construction time.
 * </p>
 */
public class ParameterizedPDGains implements PDGainsReadOnly
{
   private final DoubleParameter kp;
   private final DoubleParameter zeta;
   private final DoubleParameter maximumFeedback;
   private final DoubleParameter maximumFeedbackRate;
   private final DoubleParameter positionDeadband;

   public ParameterizedPDGains(String suffix, YoVariableRegistry registry)
   {
      this(suffix, null, registry);
   }

   public ParameterizedPDGains(String suffix, PDGainsReadOnly defaults, YoVariableRegistry registry)
   {
      if (defaults == null)
      {
         kp = new DoubleParameter("kp" + suffix, registry);
         zeta = new DoubleParameter("zeta" + suffix, registry);
         maximumFeedback = new DoubleParameter("maximumFeedback" + suffix, registry, Double.POSITIVE_INFINITY);
         maximumFeedbackRate = new DoubleParameter("maximumFeedbackRate" + suffix, registry, Double.POSITIVE_INFINITY);
         positionDeadband = new DoubleParameter("positionDeadband" + suffix, registry, 0.0);
      }
      else
      {
         kp = new DoubleParameter("kp" + suffix, registry, defaults.getKp());
         zeta = new DoubleParameter("zeta" + suffix, registry, GainCalculator.computeDampingRatio(defaults.getKp(), defaults.getKd()));
         maximumFeedback = new DoubleParameter("maximumFeedback" + suffix, registry, defaults.getMaximumFeedback());
         maximumFeedbackRate = new DoubleParameter("maximumFeedbackRate" + suffix, registry, defaults.getMaximumFeedbackRate());
         positionDeadband = new DoubleParameter("positionDeadband" + suffix, registry, defaults.getPositionDeadband());
      }
   }

   @Override
   public double getKp()
   {
      return kp.getValue();
   }

   @Override
   public double getKd()
   {
      return GainCalculator.computeDerivativeGain(kp.getValue(), zeta.getValue());
   }

   @Override
   public double getMaximumFeedback()
   {
      return maximumFeedback.getValue();
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maximumFeedbackRate.getValue();
   }

   @Override
   public double getPositionDeadband()
   {
      return positionDeadband.getValue();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PDGainsReadOnly)
         return PDGainsReadOnly.super.equals((PDGainsReadOnly) object);
      else
         return false;
   }
}
