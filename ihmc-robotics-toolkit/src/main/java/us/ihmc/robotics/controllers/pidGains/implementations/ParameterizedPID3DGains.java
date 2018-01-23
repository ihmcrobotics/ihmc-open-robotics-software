package us.ihmc.robotics.controllers.pidGains.implementations;

import java.util.EnumMap;
import java.util.Map;

import us.ihmc.euclid.Axis;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.yoVariables.listener.ParameterChangedListener;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Provides a default implementation for Parameterized PID gains in three dimensions.
 * <p>
 * If this object is created a {@link GainCoupling} can be specified that will
 * determine what Parameters will be created for tuning.
 * </p>
 * <p>
 * A flag can be provided to determine whether the integral gain tuning
 * parameters will be created.
 * </p>
 */
public class ParameterizedPID3DGains implements PID3DGainsReadOnly
{
   private final GainCoupling gainCoupling;
   private final boolean useIntegrator;

   private final Map<Axis, DoubleParameter> kpMap = new EnumMap<>(Axis.class);
   private final Map<Axis, DoubleParameter> kiMap = new EnumMap<>(Axis.class);
   private final Map<Axis, DoubleParameter> zetaMap = new EnumMap<>(Axis.class);

   private final DoubleParameter maxIntegralError;
   private final DoubleParameter maxDerivativeError;
   private final DoubleParameter maxProportionalError;
   private final DoubleParameter maxFeedback;
   private final DoubleParameter maxFeedbackRate;

   private final Map<Axis, YoDouble> kdMap = new EnumMap<>(Axis.class);
   private final double[] tempProportionalGains = new double[3];
   private final double[] tempDerivativeGains = new double[3];
   private final double[] tempIntegralGains = new double[3];

   public ParameterizedPID3DGains(String suffix, GainCoupling gainCoupling, boolean useIntegrator, YoVariableRegistry registry)
   {
      this.gainCoupling = gainCoupling;
      this.useIntegrator = useIntegrator;

      populateMap(kpMap, "kp", suffix, gainCoupling, registry);
      DefaultYoPID3DGains.populateMap(kdMap, "kd", suffix, gainCoupling, registry);
      populateMap(zetaMap, "zeta", suffix, gainCoupling, registry);

      if (useIntegrator)
      {
         populateMap(kiMap, "ki", suffix, gainCoupling, registry);
         maxIntegralError = new DoubleParameter("maxIntegralError" + suffix, registry, Double.POSITIVE_INFINITY);
      }
      else
      {
         maxIntegralError = null;
      }

      createDampingUpdaters(kpMap, kdMap, zetaMap, gainCoupling);

      maxDerivativeError = new DoubleParameter("maxDerivativeError" + suffix, registry, Double.POSITIVE_INFINITY);
      maxProportionalError = new DoubleParameter("maxProportionalError" + suffix, registry, Double.POSITIVE_INFINITY);
      maxFeedback = new DoubleParameter("maximumFeedback" + suffix, registry, Double.POSITIVE_INFINITY);
      maxFeedbackRate = new DoubleParameter("maximumFeedbackRate" + suffix, registry, Double.POSITIVE_INFINITY);
   }

   public ParameterizedPID3DGains(String suffix, PID3DGainsReadOnly defaults, YoVariableRegistry registry)
   {
      this.gainCoupling = defaults.getGainCoupling();
      this.useIntegrator = defaults.isUseIntegrator();

      double[] defaultProportionalGains = defaults.getProportionalGains();
      double[] defaultDerivativeGains = defaults.getDerivativeGains();
      double[] defaultIntegralGains = defaults.getIntegralGains();
      double[] defaultZetas = new double[3];

      for (int i = 0; i < 3; i++)
      {
         defaultZetas[i] = GainCalculator.computeDampingRatio(defaultProportionalGains[i], defaultDerivativeGains[i]);
      }

      populateMap(kpMap, "kp", suffix, gainCoupling, defaultProportionalGains, registry);
      DefaultYoPID3DGains.populateMap(kdMap, "kd", suffix, gainCoupling, registry);
      populateMap(zetaMap, "zeta", suffix, gainCoupling, defaultZetas, registry);

      if (useIntegrator)
      {
         populateMap(kiMap, "ki", suffix, gainCoupling, defaultIntegralGains, registry);
         maxIntegralError = new DoubleParameter("maxIntegralError" + suffix, registry, defaults.getMaximumIntegralError());
      }
      else
      {
         maxIntegralError = null;
      }

      createDampingUpdaters(kpMap, kdMap, zetaMap, gainCoupling);

      maxDerivativeError = new DoubleParameter("maxDerivativeError" + suffix, registry, defaults.getMaximumDerivativeError());
      maxProportionalError = new DoubleParameter("maxProportionalError" + suffix, registry, defaults.getMaximumProportionalError());
      maxFeedback = new DoubleParameter("maximumFeedback" + suffix, registry, defaults.getMaximumFeedback());
      maxFeedbackRate = new DoubleParameter("maximumFeedbackRate" + suffix, registry, defaults.getMaximumFeedbackRate());
   }

   private static void populateMap(Map<Axis, DoubleParameter> mapToFill, String prefix, String suffix, GainCoupling gainCoupling, YoVariableRegistry registry)
   {
      switch (gainCoupling)
      {
      case NONE:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "X" + suffix, registry));
         mapToFill.put(Axis.Y, new DoubleParameter(prefix + "Y" + suffix, registry));
         mapToFill.put(Axis.Z, new DoubleParameter(prefix + "Z" + suffix, registry));
         break;
      case XY:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "XY" + suffix, registry));
         mapToFill.put(Axis.Y, mapToFill.get(Axis.X));
         mapToFill.put(Axis.Z, new DoubleParameter(prefix + "Z" + suffix, registry));
         break;
      case YZ:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "X" + suffix, registry));
         mapToFill.put(Axis.Y, new DoubleParameter(prefix + "YZ" + suffix, registry));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.Y));
         break;
      case XZ:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "XZ" + suffix, registry));
         mapToFill.put(Axis.Y, new DoubleParameter(prefix + "Y" + suffix, registry));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.X));
         break;
      case XYZ:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "XYZ" + suffix, registry));
         mapToFill.put(Axis.Y, mapToFill.get(Axis.X));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.X));
         break;
      }
   }

   private static void populateMap(Map<Axis, DoubleParameter> mapToFill, String prefix, String suffix, GainCoupling gainCoupling, double[] defaults,
                                   YoVariableRegistry registry)
   {
      switch (gainCoupling)
      {
      case NONE:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "X" + suffix, registry, defaults[0]));
         mapToFill.put(Axis.Y, new DoubleParameter(prefix + "Y" + suffix, registry, defaults[1]));
         mapToFill.put(Axis.Z, new DoubleParameter(prefix + "Z" + suffix, registry, defaults[2]));
         break;
      case XY:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "XY" + suffix, registry, defaults[0]));
         mapToFill.put(Axis.Y, mapToFill.get(Axis.X));
         mapToFill.put(Axis.Z, new DoubleParameter(prefix + "Z" + suffix, registry, defaults[2]));
         break;
      case YZ:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "X" + suffix, registry, defaults[0]));
         mapToFill.put(Axis.Y, new DoubleParameter(prefix + "YZ" + suffix, registry, defaults[1]));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.Y));
         break;
      case XZ:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "XZ" + suffix, registry, defaults[0]));
         mapToFill.put(Axis.Y, new DoubleParameter(prefix + "Y" + suffix, registry, defaults[1]));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.X));
         break;
      case XYZ:
         mapToFill.put(Axis.X, new DoubleParameter(prefix + "XYZ" + suffix, registry, defaults[0]));
         mapToFill.put(Axis.Y, mapToFill.get(Axis.X));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.X));
         break;
      }
   }

   private static void createDampingUpdaters(Map<Axis, DoubleParameter> kpMap, Map<Axis, YoDouble> kdMap, Map<Axis, DoubleParameter> zetaMap,
                                             GainCoupling gainCoupling)
   {
      switch (gainCoupling)
      {
      case NONE:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis.Y, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis.Z, kpMap, kdMap, zetaMap);
         break;
      case XY:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis.Z, kpMap, kdMap, zetaMap);
         break;
      case YZ:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis.Y, kpMap, kdMap, zetaMap);
         break;
      case XZ:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis.Y, kpMap, kdMap, zetaMap);
         break;
      case XYZ:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap);
         break;
      }
   }

   private static void addDampingUpdater(Axis axis, Map<Axis, DoubleParameter> kpMap, Map<Axis, YoDouble> kdMap, Map<Axis, DoubleParameter> zetaMap)
   {
      DoubleParameter kp = kpMap.get(axis);
      YoDouble kd = kdMap.get(axis);
      DoubleParameter zeta = zetaMap.get(axis);

      ParameterChangedListener updater = (parameter) -> {
         if (kp.isLoaded() && zeta.isLoaded())
         {
            kd.set(GainCalculator.computeDerivativeGain(kp.getValue(), zeta.getValue()));
         }
      };

      kp.addParameterChangedListener(updater);
      zeta.addParameterChangedListener(updater);
   }

   @Override
   public double[] getProportionalGains()
   {
      fillFromMap(kpMap, tempProportionalGains);
      return tempProportionalGains;
   }

   @Override
   public double[] getDerivativeGains()
   {
      DefaultYoPID3DGains.fillFromMap(kdMap, tempDerivativeGains);
      return tempDerivativeGains;
   }

   @Override
   public double[] getIntegralGains()
   {
      if (!useIntegrator)
      {
         for (int i = 0; i < 3; i++)
         {
            tempIntegralGains[i] = 0.0;
         }
         return tempIntegralGains;
      }

      fillFromMap(kiMap, tempIntegralGains);
      return tempIntegralGains;
   }

   private static void fillFromMap(Map<Axis, DoubleParameter> map, double[] arrayToFill)
   {
      arrayToFill[0] = map.get(Axis.X).getValue();
      arrayToFill[1] = map.get(Axis.Y).getValue();
      arrayToFill[2] = map.get(Axis.Z).getValue();
   }

   @Override
   public double getMaximumIntegralError()
   {
      if (!useIntegrator)
      {
         return 0.0;
      }

      return maxIntegralError.getValue();
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return maxDerivativeError.getValue();
   }

   @Override
   public double getMaximumProportionalError()
   {
      return maxProportionalError.getValue();
   }

   @Override
   public double getMaximumFeedback()
   {
      return maxFeedback.getValue();
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maxFeedbackRate.getValue();
   }

   public DoubleParameter getMaximumDerivativeErrorParameter()
   {
      return maxDerivativeError;
   }

   public DoubleParameter getMaximumProportionalErrorParameter()
   {
      return maxProportionalError;
   }

   @Override
   public GainCoupling getGainCoupling()
   {
      return gainCoupling;
   }

   @Override
   public boolean isUseIntegrator()
   {
      return useIntegrator;
   }
}
