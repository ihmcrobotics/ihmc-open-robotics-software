package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.controllers.pidGains.DampingUpdater;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
//import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
//import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnessesReadOnly;
import us.ihmc.robotics.controllers.pidGains.YoPD3DStiffnesses;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;

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
public class DefaultYoPD3DStiffnesses implements YoPD3DStiffnesses
{

   private final Map<Axis3D, YoDouble> kpMap = new EnumMap<>(Axis3D.class);
   private final Map<Axis3D, YoDouble> kdMap = new EnumMap<>(Axis3D.class);
   private final Map<Axis3D, YoDouble> kiMap = new EnumMap<>(Axis3D.class);
   private final Map<Axis3D, YoDouble> zetaMap = new EnumMap<>(Axis3D.class);

   private final YoDouble maxDerivativeError;
   private final YoDouble maxProportionalError;
   private final YoDouble maxFeedback;
   private final YoDouble maxFeedbackRate;

   private final double[] tempProportionalGains = new double[3];
   private final double[] tempDerivativeGains = new double[3];
   private final double[] tempIntegralGains = new double[3];

   private final YoBoolean updateFromDampingRatio;

   public DefaultYoPD3DStiffnesses(String suffix, GainCoupling gainCoupling, YoRegistry registry)
   {
      this(suffix, gainCoupling, null, registry);
   }

   public DefaultYoPD3DStiffnesses(String suffix, GainCoupling gainCoupling, PD3DStiffnessesReadOnly gains, YoRegistry registry)
   {

      populateMap(kpMap, "kp", suffix, gainCoupling, registry);
      populateMap(kdMap, "kd", suffix, gainCoupling, registry);
      populateMap(zetaMap, "zeta", suffix, gainCoupling, registry);

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

   static void populateMap(Map<Axis3D, YoDouble> mapToFill, String prefix, String suffix, GainCoupling gainCoupling, YoRegistry registry)
   {
      switch (gainCoupling)
      {
      case NONE:
         mapToFill.put(Axis3D.X, new YoDouble(prefix + "X" + suffix, registry));
         mapToFill.put(Axis3D.Y, new YoDouble(prefix + "Y" + suffix, registry));
         mapToFill.put(Axis3D.Z, new YoDouble(prefix + "Z" + suffix, registry));
         break;
      case XY:
         mapToFill.put(Axis3D.X, new YoDouble(prefix + "XY" + suffix, registry));
         mapToFill.put(Axis3D.Y, mapToFill.get(Axis3D.X));
         mapToFill.put(Axis3D.Z, new YoDouble(prefix + "Z" + suffix, registry));
         break;
      case YZ:
         mapToFill.put(Axis3D.X, new YoDouble(prefix + "X" + suffix, registry));
         mapToFill.put(Axis3D.Y, new YoDouble(prefix + "YZ" + suffix, registry));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.Y));
         break;
      case XZ:
         mapToFill.put(Axis3D.X, new YoDouble(prefix + "XZ" + suffix, registry));
         mapToFill.put(Axis3D.Y, new YoDouble(prefix + "Y" + suffix, registry));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.X));
         break;
      case XYZ:
         mapToFill.put(Axis3D.X, new YoDouble(prefix + "XYZ" + suffix, registry));
         mapToFill.put(Axis3D.Y, mapToFill.get(Axis3D.X));
         mapToFill.put(Axis3D.Z, mapToFill.get(Axis3D.X));
         break;
      }
   }

   private static void createDampingUpdaters(Map<Axis3D, YoDouble> kpMap, Map<Axis3D, YoDouble> kdMap, Map<Axis3D, YoDouble> zetaMap, YoBoolean update,
                                             GainCoupling gainCoupling)
   {
      switch (gainCoupling)
      {
      case NONE:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis3D.Y, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis3D.Z, kpMap, kdMap, zetaMap, update);
         break;
      case XY:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis3D.Z, kpMap, kdMap, zetaMap, update);
         break;
      case YZ:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis3D.Y, kpMap, kdMap, zetaMap, update);
         break;
      case XZ:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap, update);
         addDampingUpdater(Axis3D.Y, kpMap, kdMap, zetaMap, update);
         break;
      case XYZ:
         addDampingUpdater(Axis3D.X, kpMap, kdMap, zetaMap, update);
         break;
      }
   }

   private static void addDampingUpdater(Axis3D axis, Map<Axis3D, YoDouble> kpMap, Map<Axis3D, YoDouble> kdMap, Map<Axis3D, YoDouble> zetaMap, YoBoolean update)
   {
      YoDouble kp = kpMap.get(axis);
      YoDouble kd = kdMap.get(axis);
      YoDouble zeta = zetaMap.get(axis);

      DampingUpdater kdUpdater = new DampingUpdater(kp, kd, zeta, update);
      kp.addListener(kdUpdater);
      zeta.addListener(kdUpdater);
   }

   @Override
   public double[] getProportionalStiffnesses()
   {
      fillFromMap(kpMap, tempProportionalGains);
      return tempProportionalGains;
   }

   @Override
   public double[] getDerivativeStiffnesses()
   {
      fillFromMap(kdMap, tempDerivativeGains);
      return tempDerivativeGains;
   }

   static void fillFromMap(Map<Axis3D, YoDouble> map, double[] arrayToFill)
   {
      arrayToFill[0] = map.get(Axis3D.X).getDoubleValue();
      arrayToFill[1] = map.get(Axis3D.Y).getDoubleValue();
      arrayToFill[2] = map.get(Axis3D.Z).getDoubleValue();
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
   public void setProportionalStiffnesses(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      kpMap.get(Axis3D.X).set(proportionalGainX);
      kpMap.get(Axis3D.Y).set(proportionalGainY);
      kpMap.get(Axis3D.Z).set(proportionalGainZ);
   }

   @Override
   public void setDerivativeStiffnesses(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      updateFromDampingRatio.set(false);
      kdMap.get(Axis3D.X).set(derivativeGainX);
      kdMap.get(Axis3D.Y).set(derivativeGainY);
      kdMap.get(Axis3D.Z).set(derivativeGainZ);
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
      if (object instanceof PD3DStiffnessesReadOnly)
         return YoPD3DStiffnesses.super.equals((PD3DStiffnessesReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": kp: " + Arrays.toString(getProportionalStiffnesses()) + ", kd: " + Arrays.toString(getDerivativeStiffnesses());
   }
}
