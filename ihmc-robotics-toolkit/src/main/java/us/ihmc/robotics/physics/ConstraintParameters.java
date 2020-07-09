package us.ihmc.robotics.physics;

public class ConstraintParameters implements ConstraintParametersBasics
{
   private double coefficientOfRestitution;
   private double restitutionThreshold;
   private double errorReductionParameter;

   public ConstraintParameters()
   {
   }

   public ConstraintParameters(double coefficientOfRestitution, double restitutionThreshold, double errorReductionParameter)
   {
      this.coefficientOfRestitution = coefficientOfRestitution;
      this.errorReductionParameter = errorReductionParameter;
   }

   @Override
   public void setCoefficientOfRestitution(double coefficientOfRestitution)
   {
      this.coefficientOfRestitution = coefficientOfRestitution;
   }

   @Override
   public void setRestitutionThreshold(double restitutionThreshold)
   {
      this.restitutionThreshold = restitutionThreshold;
   }

   @Override
   public void setErrorReductionParameter(double errorReductionParameter)
   {
      this.errorReductionParameter = errorReductionParameter;
   }

   @Override
   public double getCoefficientOfRestitution()
   {
      return coefficientOfRestitution;
   }

   @Override
   public double getRestitutionThreshold()
   {
      return restitutionThreshold;
   }

   @Override
   public double getErrorReductionParameter()
   {
      return errorReductionParameter;
   }
}
