package us.ihmc.robotics.physics;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoContactParameters extends YoConstraintParameters implements ContactParametersBasics
{
   private final YoDouble minimumPenetration;
   private final YoDouble coefficientOfFriction;
   private final YoBoolean computeMomentFriction;
   private final YoDouble coulombMomentFrictionRatio;

   public YoContactParameters(String prefix, YoRegistry registry)
   {
      super(prefix, registry);

      String minPenName;
      String cofName;
      String cmfName;
      String cmfrName;

      if (prefix == null || prefix.isEmpty())
      {
         minPenName = "minimumPenetration";
         cofName = "coefficientOfFriction";
         cmfName = "computeMomentFriction";
         cmfrName = "coulombMomentFrictionRatio";
      }
      else
      {
         minPenName = prefix + "MinimumPenetration";
         cofName = prefix + "CoefficientOfFriction";
         cmfName = prefix + "ComputeMomentFriction";
         cmfrName = prefix + "CoulombMomentFrictionRatio";
      }

      minimumPenetration = new YoDouble(minPenName, registry);
      coefficientOfFriction = new YoDouble(cofName, registry);
      computeMomentFriction = new YoBoolean(cmfName, registry);
      coulombMomentFrictionRatio = new YoDouble(cmfrName, registry);
   }

   @Override
   public void setMinimumPenetration(double minimumPenetration)
   {
      this.minimumPenetration.set(minimumPenetration);
   }

   @Override
   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   @Override
   public void setComputeFrictionMoment(boolean computeFrictionMoment)
   {
      this.computeMomentFriction.set(computeFrictionMoment);
   }

   @Override
   public void setCoulombMomentFrictionRatio(double coulombFrictionMomentRatio)
   {
      this.coulombMomentFrictionRatio.set(coulombFrictionMomentRatio);
   }

   @Override
   public double getMinimumPenetration()
   {
      return minimumPenetration.getValue();
   }

   @Override
   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction.getValue();
   }

   @Override
   public boolean getComputeFrictionMoment()
   {
      return computeMomentFriction.getValue();
   }

   @Override
   public double getCoulombMomentFrictionRatio()
   {
      return coulombMomentFrictionRatio.getValue();
   }
}
