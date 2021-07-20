package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MPCParameters
{
   private static final boolean includeVelocityObjective = true;
   private static final boolean includeRhoMinInequality = true;
   private static final boolean includeRhoMaxInequality = false;
   private static final boolean includeForceMinimization = false;
   private static final boolean includeRhoMinimization = true;
   private static final boolean includeRhoRateMinimization = true;

   private static final double defaultMinRhoValue = 0.0;//05;

   public static final double defaultInitialComWeight = 5e3;
   public static final double defaultInitialComVelocityWeight = 5e1;
   public static final double defaultFinalComWeight = 5e2;
   public static final double defaultFinalVRPWeight = 5e1;
   public static final double defaultVrpTrackingWeight = 1e2;
   public static final double defaultRhoTrackingWeight = 1e-3;
   public static final double defaultRhoRateTrackingWeight = 1e-6;
   public static final double defaultForceTrackingWeight = 1e-4;

   private static final double defaultOrientationAngleTrackingWeight = 1e-2;
   private static final double defaultOrientationVelocityTrackingWeight = 1e-6;

   private static final double defaultInitialOrientationWeight = 1e6;
   private static final double defaultFinalOrientationWeight = 1e-6;

   private static final ConstraintType initialCoMPositionConstraintType = ConstraintType.EQUALITY;
   private static final ConstraintType initialCoMVelocityConstraintType = ConstraintType.EQUALITY;
   private static final ConstraintType finalCoMPositionConstraintType = ConstraintType.EQUALITY;
   private static final ConstraintType finalCoMVelocityConstraintType = ConstraintType.EQUALITY;
   private static final ConstraintType finalVRPPositionConstraintType = ConstraintType.EQUALITY;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble minRhoValue = new YoDouble("minRhoValue", registry);
   private final YoDouble initialComWeight = new YoDouble("initialComWeight", registry);
   private final YoDouble initialComVelocityWeight = new YoDouble("initialComVelocityWeight", registry);
   private final YoDouble finalComWeight = new YoDouble("finalComWeight", registry);
   private final YoDouble finalVRPWeight = new YoDouble("finalVRPWeight", registry);
   private final YoDouble vrpTrackingWeight = new YoDouble("vrpTrackingWeight", registry);
   private final YoDouble rhoTrackingWeight = new YoDouble("rhoTrackingWeight", registry);
   private final YoDouble rhoRateTrackingWeight = new YoDouble("rhoRateTrackingWeight", registry);
   private final YoDouble forceTrackingWeight = new YoDouble("forceTrackingWeight", registry);

   private final YoDouble orientationAngleTrackingWeight = new YoDouble("orientationAngleTrackingWeight", registry);
   private final YoDouble orientationVelocityTrackingWeight = new YoDouble("orientationVelocityTrackingWeight", registry);
   private final YoDouble initialOrientationWeight = new YoDouble("initialOrientationWeight", registry);
   private final YoDouble finalOrientationWeight = new YoDouble("finalOrientationWeight", registry);

   public MPCParameters(YoRegistry parentRegistry)
   {
      minRhoValue.set(defaultMinRhoValue);
      initialComWeight.set(defaultInitialComWeight);
      initialComVelocityWeight.set(defaultInitialComVelocityWeight);
      finalComWeight.set(defaultFinalComWeight);
      finalVRPWeight.set(defaultFinalVRPWeight);
      vrpTrackingWeight.set(defaultVrpTrackingWeight);
      rhoTrackingWeight.set(defaultRhoTrackingWeight);
      rhoRateTrackingWeight.set(defaultRhoRateTrackingWeight);
      forceTrackingWeight.set(defaultForceTrackingWeight);

      orientationAngleTrackingWeight.set(defaultOrientationAngleTrackingWeight);
      orientationVelocityTrackingWeight.set(defaultOrientationVelocityTrackingWeight);
      initialOrientationWeight.set(defaultInitialOrientationWeight);
      finalOrientationWeight.set(defaultFinalOrientationWeight);

      parentRegistry.addChild(registry);
   }

   public boolean includeVelocityObjective()
   {
      return includeVelocityObjective;
   }

   public boolean includeRhoMinInequality()
   {
      return includeRhoMinInequality;
   }

   public boolean includeRhoMaxInequality()
   {
      return includeRhoMaxInequality;
   }

   public boolean includeForceMinimization()
   {
      return includeForceMinimization;
   }

   public boolean includeRhoMinimization()
   {
      return includeRhoMinimization;
   }

   public boolean includeRhoRateMinimization()
   {
      return includeRhoRateMinimization;
   }

   public ConstraintType getInitialCoMPositionConstraintType()
   {
      return initialCoMPositionConstraintType;
   }

   public ConstraintType getInitialCoMVelocityConstraintType()
   {
      return initialCoMVelocityConstraintType;
   }

   public ConstraintType getFinalCoMPositionConstraintType()
   {
      return finalCoMPositionConstraintType;
   }

   public ConstraintType getFinalCoMVelocityConstraintType()
   {
      return finalCoMVelocityConstraintType;
   }

   public ConstraintType getFinalVRPPositionConstraintType()
   {
      return finalVRPPositionConstraintType;
   }

   public double getMinRhoValue()
   {
      return minRhoValue.getValue();
   }

   public double getInitialComWeight()
   {
      return initialComWeight.getValue();
   }

   public double getInitialComVelocityWeight()
   {
      return initialComVelocityWeight.getValue();
   }

   public double getFinalComWeight()
   {
      return finalComWeight.getDoubleValue();
   }

   public double getFinalVRPWeight()
   {
      return finalVRPWeight.getDoubleValue();
   }

   public double getVRPTrackingWeight()
   {
      return vrpTrackingWeight.getDoubleValue();
   }

   public double getRhoTrackingWeight()
   {
      return rhoTrackingWeight.getDoubleValue();
   }

   public double getRhoRateTrackingWeight()
   {
      return rhoRateTrackingWeight.getDoubleValue();
   }

   public double getForceTrackingWeight()
   {
      return forceTrackingWeight.getDoubleValue();
   }

   public double getInitialOrientationWeight()
   {
      return initialOrientationWeight.getDoubleValue();
   }

   public double getFinalOrientationWeight()
   {
      return finalOrientationWeight.getDoubleValue();
   }

   public DoubleProvider getOrientationAngleTrackingWeightProvider()
   {
      return orientationAngleTrackingWeight;
   }

   public DoubleProvider getOrientationVelocityTrackingWeightProvider()
   {
      return orientationVelocityTrackingWeight;
   }
}
