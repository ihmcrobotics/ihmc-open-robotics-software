package us.ihmc.robotics.physics;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoConstraintParameters implements ConstraintParametersBasics
{
   private final YoDouble coefficientOfRestitution;
   private final YoDouble restitutionThreshold;
   private final YoDouble errorReductionParameter;

   public YoConstraintParameters(String prefix, YoRegistry registry)
   {
      String corName, erpName, rthName;

      if (prefix == null || prefix.isEmpty())
      {
         corName = "coefficientOfRestitution";
         rthName = "restitutionThreshold";
         erpName = "errorReductionParameter";
      }
      else
      {
         corName = prefix + "CoefficientOfRestitution";
         rthName = prefix + "RestitutionThreshold";
         erpName = prefix + "ErrorReductionParameter";
      }

      coefficientOfRestitution = new YoDouble(corName, registry);
      restitutionThreshold = new YoDouble(rthName, registry);
      errorReductionParameter = new YoDouble(erpName, registry);
   }

   @Override
   public void setCoefficientOfRestitution(double coefficientOfRestitution)
   {
      this.coefficientOfRestitution.set(coefficientOfRestitution);
   }

   @Override
   public void setRestitutionThreshold(double restitutionThreshold)
   {
      this.restitutionThreshold.set(restitutionThreshold);
   }

   @Override
   public void setErrorReductionParameter(double errorReductionParameter)
   {
      this.errorReductionParameter.set(errorReductionParameter);
   }

   @Override
   public double getCoefficientOfRestitution()
   {
      return coefficientOfRestitution.getValue();
   }

   @Override
   public double getRestitutionThreshold()
   {
      return restitutionThreshold.getValue();
   }

   @Override
   public double getErrorReductionParameter()
   {
      return errorReductionParameter.getValue();
   }
}
