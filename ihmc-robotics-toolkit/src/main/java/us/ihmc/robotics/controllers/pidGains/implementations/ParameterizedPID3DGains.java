package us.ihmc.robotics.controllers.pidGains.implementations;

import java.util.EnumMap;
import java.util.Map;

import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.yoVariables.listener.YoParameterChangedListener;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
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
   private final boolean usingIntegrator;

   private final Map<Axis3D, DoubleParameter> kpMap = new EnumMap<>(Axis3D.class);
   private final Map<Axis3D, DoubleParameter> kiMap = new EnumMap<>(Axis3D.class);
   private final Map<Axis3D, DoubleParameter> zetaMap = new EnumMap<>(Axis3D.class);

   private final DoubleParameter maxIntegralError;
   private final DoubleParameter maxDerivativeError;
   private final DoubleParameter maxProportionalError;
   private final DoubleParameter maxFeedback;
   private final DoubleParameter maxFeedbackRate;

   private final Map<Axis3D, YoDouble> kdMap = new EnumMap<>(Axis3D.class);
   private final double[] tempProportionalGains = new double[3];
   private final double[] tempDerivativeGains = new double[3];
   private final double[] tempIntegralGains = new double[3];

   public ParameterizedPID3DGains(String suffix, PID3DConfiguration configuration, YoRegistry registry)
   {
      this(suffix, configuration.getGainCoupling(), configuration.isUseIntegrator(), configuration.getGains(), registry);
   }

   public ParameterizedPID3DGains(String suffix, GainCoupling gainCoupling, boolean useIntegrator, YoRegistry registry)
   {
      this(suffix, gainCoupling, useIntegrator, null, registry);
   }

   public ParameterizedPID3DGains(String suffix, GainCoupling gainCoupling, boolean useIntegrator, PID3DGainsReadOnly defaults, YoRegistry registry)
   {
      this.usingIntegrator = useIntegrator;

      if (defaults == null)
      {
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
      else
      {
         double[] defaultProportionalGains = defaults.getProportionalGains();
         double[] defaultDerivativeGains = defaults.getDerivativeGains();
         double[] defaultIntegralGains = defaults.getIntegralGains();
         double[] defaultZetas = new double[3];

         for (int i = 0; i < 3; i++)
         {
            double proportionalGain = defaultProportionalGains[i];
            if (proportionalGain != 0.0)
            {
               defaultZetas[i] = GainCalculator.computeDampingRatio(proportionalGain, defaultDerivativeGains[i]);
            }
            else
            {
               defaultZetas[i] = 0.0;
            }
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
   }

   private static void populateMap(Map<Axis3D, DoubleParameter> mapToFill, String prefix, String suffix, GainCoupling gainCoupling, YoRegistry registry)
   {
      switch (gainCoupling)
      {
      case NONE:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "X" + suffix, registry));
         mapToFill.put(Axis3D.Y, new DoubleParameter(prefix + "Y" + suffix, registry));
         mapToFill.put(Axis3D.Z, new DoubleParameter(prefix + "Z" + suffix, registry));
         break;
      case XY:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "XY" + suffix, registry));
         mapToFill.put(Axis3D.Y, mapToFill.get(Axis3D.X));
         mapToFill.put(Axis3D.Z, new DoubleParameter(prefix + "Z" + suffix, registry));
         break;
      case YZ:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "X" + suffix, registry));
         mapToFill.put(Axis3D.Y, new DoubleParameter(prefix + "YZ" + suffix, registry));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.Y));
         break;
      case XZ:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "XZ" + suffix, registry));
         mapToFill.put(Axis3D.Y, new DoubleParameter(prefix + "Y" + suffix, registry));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.X));
         break;
      case XYZ:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "XYZ" + suffix, registry));
         mapToFill.put(Axis3D.Y, mapToFill.get(Axis3D.X));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.X));
         break;
      }
   }

   private static void populateMap(Map<Axis3D, DoubleParameter> mapToFill, String prefix, String suffix, GainCoupling gainCoupling, double[] defaults,
                                   YoRegistry registry)
   {
      switch (gainCoupling)
      {
      case NONE:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "X" + suffix, registry, defaults[0]));
         mapToFill.put(Axis3D.Y, new DoubleParameter(prefix + "Y" + suffix, registry, defaults[1]));
         mapToFill.put(Axis3D.Z, new DoubleParameter(prefix + "Z" + suffix, registry, defaults[2]));
         break;
      case XY:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "XY" + suffix, registry, defaults[0]));
         mapToFill.put(Axis3D.Y, mapToFill.get(Axis3D.X));
         mapToFill.put(Axis3D.Z, new DoubleParameter(prefix + "Z" + suffix, registry, defaults[2]));
         break;
      case YZ:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "X" + suffix, registry, defaults[0]));
         mapToFill.put(Axis3D.Y, new DoubleParameter(prefix + "YZ" + suffix, registry, defaults[1]));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.Y));
         break;
      case XZ:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "XZ" + suffix, registry, defaults[0]));
         mapToFill.put(Axis3D.Y, new DoubleParameter(prefix + "Y" + suffix, registry, defaults[1]));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.X));
         break;
      case XYZ:
         mapToFill.put(Axis3D.X, new DoubleParameter(prefix + "XYZ" + suffix, registry, defaults[0]));
         mapToFill.put(Axis3D.Y, mapToFill.get(Axis3D.X));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.X));
         break;
      }
   }

   private static void createDampingUpdaters(Map<Axis3D, DoubleParameter> kpMap, Map<Axis3D, YoDouble> kdMap, Map<Axis3D, DoubleParameter> zetaMap,
                                             GainCoupling gainCoupling)
   {
      switch (gainCoupling)
      {
      case NONE:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis3D.Y, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis3D.Z, kpMap, kdMap, zetaMap);
         break;
      case XY:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis3D.Z, kpMap, kdMap, zetaMap);
         break;
      case YZ:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis3D.Y, kpMap, kdMap, zetaMap);
         break;
      case XZ:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap);
         addDampingUpdater(Axis3D.Y, kpMap, kdMap, zetaMap);
         break;
      case XYZ:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap);
         break;
      }
   }

   private static void addDampingUpdater(Axis3D axis, Map<Axis3D, DoubleParameter> kpMap, Map<Axis3D, YoDouble> kdMap, Map<Axis3D, DoubleParameter> zetaMap)
   {
      DoubleParameter kp = kpMap.get(axis);
      YoDouble kd = kdMap.get(axis);
      DoubleParameter zeta = zetaMap.get(axis);

      YoParameterChangedListener updater = (parameter) -> {
         if (kp.isLoaded() && zeta.isLoaded())
         {
            kd.set(GainCalculator.computeDerivativeGain(kp.getValue(), zeta.getValue()));
         }
      };

      updater.changed(null);
      kp.addListener(updater);
      zeta.addListener(updater);
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
      if (!usingIntegrator)
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

   private static void fillFromMap(Map<Axis3D, DoubleParameter> map, double[] arrayToFill)
   {
      arrayToFill[0] = map.get(Axis3D.X).getValue();
      arrayToFill[1] = map.get(Axis3D.Y).getValue();
      arrayToFill[2] = map.get(Axis3D.Z).getValue();
   }

   @Override
   public double getMaximumIntegralError()
   {
      if (!usingIntegrator)
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
}
