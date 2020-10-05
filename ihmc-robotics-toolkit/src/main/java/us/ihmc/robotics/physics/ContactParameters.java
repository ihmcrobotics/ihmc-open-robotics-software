package us.ihmc.robotics.physics;

public class ContactParameters implements ContactParametersBasics
{
   private double minimumPenetration;
   private double coefficientOfFriction;
   private double coefficientOfRestitution;
   private double restitutionThreshold;
   private double errorReductionParameter;
   private boolean computeMomentFriction;
   private double coulombMomentFrictionRatio;

   public static ContactParameters defaultIneslasticContactParameters(boolean computeMomentFriction)
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(0.0);
      contactParameters.setRestitutionThreshold(0.0);
      contactParameters.setErrorReductionParameter(0.0);
      contactParameters.setComputeFrictionMoment(computeMomentFriction);
      contactParameters.setCoulombMomentFrictionRatio(0.3);
      return contactParameters;
   }

   public ContactParameters()
   {
   }

   public ContactParameters(double minimumPenetration, double coefficientOfFriction, double coefficientOfRestitution, double restitutionThreshold,
                            double errorReductionParameter, boolean computeMomentFriction, double coulombMomentFrictionRatio)
   {
      this.minimumPenetration = minimumPenetration;
      this.coefficientOfFriction = coefficientOfFriction;
      this.coefficientOfRestitution = coefficientOfRestitution;
      this.restitutionThreshold = restitutionThreshold;
      this.errorReductionParameter = errorReductionParameter;
      this.computeMomentFriction = computeMomentFriction;
      this.coulombMomentFrictionRatio = coulombMomentFrictionRatio;
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
   public void setComputeFrictionMoment(boolean computeFrictionMoment)
   {
      computeMomentFriction = computeFrictionMoment;
   }

   @Override
   public void setCoulombMomentFrictionRatio(double coulombFrictionMomentRatio)
   {
      coulombMomentFrictionRatio = coulombFrictionMomentRatio;
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
   public boolean getComputeFrictionMoment()
   {
      return computeMomentFriction;
   }

   @Override
   public double getCoulombMomentFrictionRatio()
   {
      return coulombMomentFrictionRatio;
   }
}
