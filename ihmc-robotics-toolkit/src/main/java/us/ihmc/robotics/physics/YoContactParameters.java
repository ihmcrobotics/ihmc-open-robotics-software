package us.ihmc.robotics.physics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoContactParameters extends YoConstraintParameters implements ContactParametersBasics
{
   private final YoDouble minimumPenetration;
   private final YoDouble coefficientOfFriction;
   private final YoDouble slipErrorReductionParameter;

   public YoContactParameters(String prefix, YoVariableRegistry registry)
   {
      super(prefix, registry);

      String minPenName;
      String cofName;
      String slipERPName;

      if (prefix == null || prefix.isEmpty())
      {
         minPenName = "minimumPenetration";
         cofName = "coefficientOfFriction";
         slipERPName = "slipErrorReductionParameter";
      }
      else
      {
         minPenName = prefix + "MinimumPenetration";
         cofName = prefix + "CoefficientOfFriction";
         slipERPName = "SlipErrorReductionParameter";
      }

      minimumPenetration = new YoDouble(minPenName, registry);
      coefficientOfFriction = new YoDouble(cofName, registry);
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
   public double getSlipErrorReductionParameter()
   {
      return slipErrorReductionParameter.getValue();
   }
}
