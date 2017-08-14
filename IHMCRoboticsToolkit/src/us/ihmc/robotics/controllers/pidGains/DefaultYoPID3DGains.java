package us.ihmc.robotics.controllers.pidGains;

import java.util.EnumMap;
import java.util.Map;

import us.ihmc.robotics.Axis;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DefaultYoPID3DGains implements PID3DGains
{
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

   public DefaultYoPID3DGains(String suffix, GainCoupling gainCoupling, YoVariableRegistry registry)
   {
      populateMap(kpMap, "kp", suffix, gainCoupling, registry);
      populateMap(kdMap, "kd", suffix, gainCoupling, registry);
      populateMap(kiMap, "ki", suffix, gainCoupling, registry);
      populateMap(zetaMap, "zeta", suffix, gainCoupling, registry);

      updateFromDampingRatio = new YoBoolean("UpdateFromDampingRatio" + suffix, registry);
      updateFromDampingRatio.set(true);
      createDampingUpdaters(kpMap, kdMap, zetaMap, updateFromDampingRatio, gainCoupling);

      maxIntegralError = new YoDouble("maxIntegralError" + suffix, registry);
      maxDerivativeError = new YoDouble("maxDerivativeError" + suffix, registry);
      maxProportionalError = new YoDouble("maxProportionalError" + suffix, registry);
      maxFeedback = new YoDouble("maximumFeedback" + suffix, registry);
      maxFeedbackRate = new YoDouble("maximumFeedbackRate" + suffix, registry);

      maxFeedback.set(Double.POSITIVE_INFINITY);
      maxFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   private static void populateMap(Map<Axis, YoDouble> mapToFill, String prefix, String suffix, GainCoupling gainCoupling, YoVariableRegistry registry)
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

   private static void createDampingUpdaters(Map<Axis, YoDouble> kpMap, Map<Axis, YoDouble> kdMap, Map<Axis, YoDouble> zetaMap, YoBoolean update, GainCoupling gainCoupling)
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

      // If kp or zeta is changed update kd.
      DampingUpdater kdUpdater = new DampingUpdater(kp, kd, zeta, update);
      kp.addVariableChangedListener(kdUpdater);
      zeta.addVariableChangedListener(kdUpdater);

      // If kd is changed update zeta.
      // This will not update listeners to avoid an infinite update loop.
      ZetaUpdater zetaUpdater = new ZetaUpdater(kp, kd, zeta, update);
      kd.addVariableChangedListener(zetaUpdater);
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
      fillFromMap(kiMap, tempIntegralGains);
      return tempIntegralGains;
   }

   private static void fillFromMap(Map<Axis, YoDouble> map, double[] arrayToFill)
   {
      arrayToFill[0] = map.get(Axis.X).getDoubleValue();
      arrayToFill[1] = map.get(Axis.Y).getDoubleValue();
      arrayToFill[2] = map.get(Axis.Z).getDoubleValue();
   }

   @Override
   public double getMaximumIntegralError()
   {
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
      kdMap.get(Axis.X).set(derivativeGainX);
      kdMap.get(Axis.Y).set(derivativeGainY);
      kdMap.get(Axis.Z).set(derivativeGainZ);
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
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
}
