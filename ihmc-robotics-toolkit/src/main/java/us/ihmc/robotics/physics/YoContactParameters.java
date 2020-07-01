package us.ihmc.robotics.physics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoContactParameters extends YoConstraintParameters implements ContactParametersBasics
{
   private final YoDouble minimumPenetration;
   private final YoDouble coefficientOfFriction;
   private final YoBoolean computeMomentFriction;
   private final YoDouble coulombMomentFrictionRatio;
   private final YoDouble slipErrorReductionParameter;

   public YoContactParameters(String prefix, YoVariableRegistry registry)
   {
      super(prefix, registry);

      String minPenName;
      String cofName;
      String cmfName;
      String cmfrName;
      String slipERPName;

      if (prefix == null || prefix.isEmpty())
      {
         minPenName = "minimumPenetration";
         cofName = "coefficientOfFriction";
         cmfName = "computeMomentFriction";
         cmfrName = "coulombMomentFrictionRatio";
         slipERPName = "slipErrorReductionParameter";
      }
      else
      {
         minPenName = prefix + "MinimumPenetration";
         cofName = prefix + "CoefficientOfFriction";
         cmfName = prefix + "ComputeMomentFriction";
         cmfrName = prefix + "CoulombMomentFrictionRatio";
         slipERPName = "SlipErrorReductionParameter";
      }

      minimumPenetration = new YoDouble(minPenName, registry);
      coefficientOfFriction = new YoDouble(cofName, registry);
      computeMomentFriction = new YoBoolean(cmfName, registry);
      coulombMomentFrictionRatio = new YoDouble(cmfrName, registry);
      slipErrorReductionParameter = new YoDouble(slipERPName, registry);
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
      this.setCoulombMomentFrictionRatio(coulombFrictionMomentRatio);
   }

   @Override
   public void setSlipErrorReductionParameter(double slipErrorReductionParameter)
   {
      this.slipErrorReductionParameter.set(slipErrorReductionParameter);
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

   @Override
   public double getSlipErrorReductionParameter()
   {
      return slipErrorReductionParameter.getValue();
   }
}
