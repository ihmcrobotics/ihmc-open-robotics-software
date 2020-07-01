package us.ihmc.robotics.physics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoConstraintParameters implements ConstraintParametersBasics
{
   private final YoDouble coefficientOfRestitution;
   private final YoDouble restitutionThreshold;
   private final YoDouble errorReductionParameter;
   private final YoDouble constraintForceMixing;

   public YoConstraintParameters(String prefix, YoVariableRegistry registry)
   {
      String corName, erpName, cfmName, rthName;

      if (prefix == null || prefix.isEmpty())
      {
         corName = "coefficientOfRestitution";
         rthName = "restitutionThreshold";
         erpName = "errorReductionParameter";
         cfmName = "constraintForceMixing";
      }
      else
      {
         corName = prefix + "CoefficientOfRestitution";
         rthName = prefix + "RestitutionThreshold";
         erpName = prefix + "ErrorReductionParameter";
         cfmName = prefix + "ConstraintForceMixing";
      }

      coefficientOfRestitution = new YoDouble(corName, registry);
      restitutionThreshold = new YoDouble(rthName, registry);
      errorReductionParameter = new YoDouble(erpName, registry);
      constraintForceMixing = new YoDouble(cfmName, registry);
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
   public void setConstraintForceMixing(double constraintForceMixing)
   {
      this.constraintForceMixing.set(constraintForceMixing);
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

   @Override
   public double getConstraintForceMixing()
   {
      return constraintForceMixing.getValue();
   }
}
