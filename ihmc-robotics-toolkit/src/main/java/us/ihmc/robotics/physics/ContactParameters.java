package us.ihmc.robotics.physics;

public class ContactParameters implements ContactParametersBasics
{
   private double minimumPenetration;
   private double coefficientOfFriction;
   private double coefficientOfRestitution;
   private double restitutionThreshold;
   private double errorReductionParameter;
   private double slipErrorReductionParameter;
   private double constraintForceMixing;

   public ContactParameters()
   {
   }

   public ContactParameters(double minimumPenetration, double coefficientOfFriction, double coefficientOfRestitution, double restitutionThreshold,
                            double errorReductionParameter, double slipErrorReductionParameter, double constraintForceMixing)
   {
      this.minimumPenetration = minimumPenetration;
      this.coefficientOfFriction = coefficientOfFriction;
      this.coefficientOfRestitution = coefficientOfRestitution;
      this.restitutionThreshold = restitutionThreshold;
      this.errorReductionParameter = errorReductionParameter;
      this.slipErrorReductionParameter = slipErrorReductionParameter;
      this.constraintForceMixing = constraintForceMixing;
   }

   @Override
   public void setMinimumPenetration(double minimumPenetration)
   {
      this.minimumPenetration = minimumPenetration;
   }

   @Override
   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction = coefficientOfFriction;
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
   public void setSlipErrorReductionParameter(double slipErrorReductionParameter)
   {
      this.slipErrorReductionParameter = slipErrorReductionParameter;
   }

   @Override
   public void setConstraintForceMixing(double constraintForceMixing)
   {
      this.constraintForceMixing = constraintForceMixing;
   }

   @Override
   public double getMinimumPenetration()
   {
      return minimumPenetration;
   }

   @Override
   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
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
   public double getSlipErrorReductionParameter()
   {
      return slipErrorReductionParameter;
   }

   @Override
   public double getConstraintForceMixing()
   {
      return constraintForceMixing;
   }
}
