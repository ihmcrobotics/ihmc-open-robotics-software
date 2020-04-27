package us.ihmc.robotics.physics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoContactParameters extends YoConstraintParameters implements ContactParametersBasics
{
   private final YoDouble coefficientOfFriction;
   private final YoDouble slipErrorReductionParameter;

   public YoContactParameters(String prefix, YoVariableRegistry registry)
   {
      super(prefix, registry);

      String cofName;
      String slipERPName;

      if (prefix == null || prefix.isEmpty())
      {
         cofName = "coefficientOfFriction";
         slipERPName = "slipErrorReductionParameter";
      }
      else
      {
         cofName = prefix + "CoefficientOfFriction";
         slipERPName = "SlipErrorReductionParameter";
      }

      coefficientOfFriction = new YoDouble(cofName, registry);
      slipErrorReductionParameter = new YoDouble(slipERPName, registry);
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
