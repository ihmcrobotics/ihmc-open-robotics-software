package us.ihmc.robotics.physics;

public class ConstraintParameters implements ConstraintParametersBasics
{
   private double coefficientOfRestitution;
   private double restitutionThreshold;
   private double errorReductionParameter;
   private double constraintForceMixing;

   public ConstraintParameters()
   {
   }

   public ConstraintParameters(double coefficientOfRestitution, double restitutionThreshold, double errorReductionParameter, double constraintForceMixing)
   {
      this.coefficientOfRestitution = coefficientOfRestitution;
      this.errorReductionParameter = errorReductionParameter;
      this.constraintForceMixing = constraintForceMixing;
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
   public void setConstraintForceMixing(double constraintForceMixing)
   {
      this.constraintForceMixing = constraintForceMixing;
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

   @Override
   public double getConstraintForceMixing()
   {
      return constraintForceMixing;
   }
}
