package us.ihmc.robotics.controllers.pidGains.implementations;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;

import us.ihmc.euclid.Axis;
import us.ihmc.robotics.controllers.pidGains.DampingUpdater;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Provides a default implementation for Yo PID gains in three dimensions.
 * <p>
 * If this object is created a {@link GainCoupling} can be specified that will
 * determine what YoVariables will be created for tuning.
 * Note, that regardless of the specified gain coupling the getters and setters
 * are designed for three dimensions.
 * </p>
 * <p>
 * A flag can be provided to determine whether the integral gain tuning
 * YoVariables will be created.
 * </p>
 */
public class DefaultYoPID3DGains implements YoPID3DGains
{
   private final boolean usingIntegrator;

   private final Map<Axis, YoDouble> kpMap = new EnumMap<>(Axis.class);
   private final Map<Axis, YoDouble> kdMap = new EnumMap<>(Axis.class);
   private final Map<Axis, YoDouble> kiMap = new EnumMap<>(Axis.class);
   private final Map<Axis, YoDouble> zetaMap = new EnumMap<>(Axis.class);

   private final YoDouble maxIntegralError;
   private final YoDouble maxDerivativeError;
   private final YoDouble maxProportionalError;
   private final YoDouble maxFeedback;
   private final YoDouble maxFeedbackRate;

   private final double[] tempProportionalGains = new double[3];
   private final double[] tempDerivativeGains = new double[3];
   private final double[] tempIntegralGains = new double[3];

   private final YoBoolean updateFromDampingRatio;

   public DefaultYoPID3DGains(String suffix, PID3DConfiguration configuration, YoVariableRegistry registry)
   {
      this(suffix, configuration.getGainCoupling(), configuration.isUseIntegrator(), configuration.getGains(), registry);
   }

   public DefaultYoPID3DGains(String suffix, GainCoupling gainCoupling, boolean useIntegrator, YoVariableRegistry registry)
   {
      this(suffix, gainCoupling, useIntegrator, null, registry);
   }

   public DefaultYoPID3DGains(String suffix, GainCoupling gainCoupling, boolean useIntegrator, PID3DGainsReadOnly gains, YoVariableRegistry registry)
   {
      this.usingIntegrator = useIntegrator;

      populateMap(kpMap, "kp", suffix, gainCoupling, registry);
      populateMap(kdMap, "kd", suffix, gainCoupling, registry);
      populateMap(zetaMap, "zeta", suffix, gainCoupling, registry);

      if (useIntegrator)
      {
         populateMap(kiMap, "ki", suffix, gainCoupling, registry);
         maxIntegralError = new YoDouble("maxIntegralError" + suffix, registry);
      }
      else
      {
         maxIntegralError = null;
      }

      updateFromDampingRatio = new YoBoolean("UpdateFromDampingRatio" + suffix, registry);
      updateFromDampingRatio.set(true);
      createDampingUpdaters(kpMap, kdMap, zetaMap, updateFromDampingRatio, gainCoupling);

      maxDerivativeError = new YoDouble("maxDerivativeError" + suffix, registry);
      maxProportionalError = new YoDouble("maxProportionalError" + suffix, registry);
      maxFeedback = new YoDouble("maximumFeedback" + suffix, registry);
      maxFeedbackRate = new YoDouble("maximumFeedbackRate" + suffix, registry);

      maxFeedback.set(Double.POSITIVE_INFINITY);
      maxFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);

      if (gains != null)
      {
         set(gains);
      }
   }

   static void populateMap(Map<Axis, YoDouble> mapToFill, String prefix, String suffix, GainCoupling gainCoupling, YoVariableRegistry registry)
   {
      switch (gainCoupling)
      {
      case NONE:
         mapToFill.put(Axis.X, new YoDouble(prefix + "X" + suffix, registry));
         mapToFill.put(Axis.Y, new YoDouble(prefix + "Y" + suffix, registry));
         mapToFill.put(Axis.Z, new YoDouble(prefix + "Z" + suffix, registry));
         break;
      case XY:
         mapToFill.put(Axis.X, new YoDouble(prefix + "XY" + suffix, registry));
         mapToFill.put(Axis.Y, mapToFill.get(Axis.X));
         mapToFill.put(Axis.Z, new YoDouble(prefix + "Z" + suffix, registry));
         break;
      case YZ:
         mapToFill.put(Axis.X, new YoDouble(prefix + "X" + suffix, registry));
         mapToFill.put(Axis.Y, new YoDouble(prefix + "YZ" + suffix, registry));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.Y));
         break;
      case XZ:
         mapToFill.put(Axis.X, new YoDouble(prefix + "XZ" + suffix, registry));
         mapToFill.put(Axis.Y, new YoDouble(prefix + "Y" + suffix, registry));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.X));
         break;
      case XYZ:
         mapToFill.put(Axis.X, new YoDouble(prefix + "XYZ" + suffix, registry));
         mapToFill.put(Axis.Y, mapToFill.get(Axis.X));
         mapToFill.put(Axis.Z, mapToFill.get(Axis.X));
         break;
      }
   }

   private static void createDampingUpdaters(Map<Axis, YoDouble> kpMap, Map<Axis, YoDouble> kdMap, Map<Axis, YoDouble> zetaMap, YoBoolean update,
                                             GainCoupling gainCoupling)
   {
      switch (gainCoupling)
      {
      case NONE:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis.Y, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis.Z, kpMap, kdMap, zetaMap, update);
         break;
      case XY:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis.Z, kpMap, kdMap, zetaMap, update);
         break;
      case YZ:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis.Y, kpMap, kdMap, zetaMap, update);
         break;
      case XZ:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis.Y, kpMap, kdMap, zetaMap, update);
         break;
      case XYZ:
         addDampingUpdater(Axis.X, kpMap, kdMap, zetaMap, update);
         break;
      }
   }

   private static void addDampingUpdater(Axis axis, Map<Axis, YoDouble> kpMap, Map<Axis, YoDouble> kdMap, Map<Axis, YoDouble> zetaMap, YoBoolean update)
   {
      YoDouble kp = kpMap.get(axis);
      YoDouble kd = kdMap.get(axis);
      YoDouble zeta = zetaMap.get(axis);

      DampingUpdater kdUpdater = new DampingUpdater(kp, kd, zeta, update);
      kp.addVariableChangedListener(kdUpdater);
      zeta.addVariableChangedListener(kdUpdater);
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
      fillFromMap(kdMap, tempDerivativeGains);
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

   static void fillFromMap(Map<Axis, YoDouble> map, double[] arrayToFill)
   {
      arrayToFill[0] = map.get(Axis.X).getDoubleValue();
      arrayToFill[1] = map.get(Axis.Y).getDoubleValue();
      arrayToFill[2] = map.get(Axis.Z).getDoubleValue();
   }

   @Override
   public double getMaximumIntegralError()
   {
      if (!usingIntegrator)
      {
         return 0.0;
      }

      return maxIntegralError.getDoubleValue();
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return maxDerivativeError.getDoubleValue();
   }

   @Override
   public double getMaximumProportionalError()
   {
      return maxProportionalError.getDoubleValue();
   }

   @Override
   public double getMaximumFeedback()
   {
      return maxFeedback.getDoubleValue();
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maxFeedbackRate.getDoubleValue();
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      kpMap.get(Axis.X).set(proportionalGainX);
      kpMap.get(Axis.Y).set(proportionalGainY);
      kpMap.get(Axis.Z).set(proportionalGainZ);
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      updateFromDampingRatio.set(false);
      kdMap.get(Axis.X).set(derivativeGainX);
      kdMap.get(Axis.Y).set(derivativeGainY);
      kdMap.get(Axis.Z).set(derivativeGainZ);
      updateDampingRatios();
   }

   public void setDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      updateFromDampingRatio.set(true);
      zetaMap.get(Axis.X).set(dampingRatioX);
      zetaMap.get(Axis.Y).set(dampingRatioY);
      zetaMap.get(Axis.Z).set(dampingRatioZ);
   }

   public void setDampingRatios(double dampingRatio)
   {
      setDampingRatios(dampingRatio, dampingRatio, dampingRatio);
   }

   private void updateDampingRatios()
   {
      for (int i = 0; i < Axis.values.length; i++)
      {
         YoDouble kp = kpMap.get(Axis.values[i]);
         YoDouble kd = kdMap.get(Axis.values[i]);
         YoDouble zeta = zetaMap.get(Axis.values[i]);
         zeta.set(GainCalculator.computeDampingRatio(kp.getDoubleValue(), kd.getDoubleValue()));
      }
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      if (!usingIntegrator)
      {
         return;
      }

      kiMap.get(Axis.X).set(integralGainX);
      kiMap.get(Axis.Y).set(integralGainY);
      kiMap.get(Axis.Z).set(integralGainZ);
      this.maxIntegralError.set(maxIntegralError);
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      this.maxFeedback.set(maxFeedback);
      this.maxFeedbackRate.set(maxFeedbackRate);
   }

   @Override
   public void setMaxDerivativeError(double maxDerivativeError)
   {
      this.maxDerivativeError.set(maxDerivativeError);
   }

   @Override
   public void setMaxProportionalError(double maxProportionalError)
   {
      this.maxProportionalError.set(maxProportionalError);
   }

   @Override
   public YoDouble getYoMaximumFeedback()
   {
      return maxFeedback;
   }

   @Override
   public YoDouble getYoMaximumFeedbackRate()
   {
      return maxFeedbackRate;
   }

   @Override
   public YoDouble getYoMaximumDerivativeError()
   {
      return maxDerivativeError;
   }

   @Override
   public YoDouble getYoMaximumProportionalError()
   {
      return maxProportionalError;
   }


   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PID3DGainsReadOnly)
         return YoPID3DGains.super.equals((PID3DGainsReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": kp: " + Arrays.toString(getProportionalGains()) + ", kd: " + Arrays.toString(getDerivativeGains()) + ", ki: "
            + Arrays.toString(getIntegralGains());
   }
}
