package us.ihmc.robotics.physics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoContactParameters extends YoConstraintParameters implements ContactParametersBasics
{
   private final YoDouble coefficientOfFriction;

   public YoContactParameters(String prefix, YoVariableRegistry registry)
   {
      super(prefix, registry);

      String cofName;

      if (prefix == null || prefix.isEmpty())
         cofName = "coefficientOfFriction";
      else
         cofName = prefix + "CoefficientOfFriction";

      coefficientOfFriction = new YoDouble(cofName, registry);
   }

   @Override
   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   @Override
   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction.getValue();
   }
}
