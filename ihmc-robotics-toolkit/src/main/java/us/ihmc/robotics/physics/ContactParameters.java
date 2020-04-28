package us.ihmc.robotics.physics;

public class ContactParameters implements ContactParametersBasics
{
   private double coefficientOfFriction;
   private double coefficientOfRestitution;
   private double errorReductionParameter;
   private double constraintForceMixing;

   public ContactParameters()
   {
   }

   public ContactParameters(double coefficientOfFriction, double coefficientOfRestitution, double errorReductionParameter, double constraintForceMixing)
   {
      this.coefficientOfFriction = coefficientOfFriction;
      this.coefficientOfRestitution = coefficientOfRestitution;
      this.errorReductionParameter = errorReductionParameter;
      this.constraintForceMixing = constraintForceMixing;
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
