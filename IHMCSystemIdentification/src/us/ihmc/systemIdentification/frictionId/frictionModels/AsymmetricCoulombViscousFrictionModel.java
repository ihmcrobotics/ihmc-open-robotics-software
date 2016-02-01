package us.ihmc.systemIdentification.frictionId.frictionModels;

/**
 * 
 * Asymmetric friction force based on coulomb + viscous model.
 * All parameters must be >= 0.
 * 
 * @param positiveCoulomb            - coulomb friction parameter for positive velocities.
 * @param positiveViscous            - viscous friction parameter for positive velocities.
 * @param negativeCoulomb            - coulomb friction parameter for negative velocities.
 * @param negativeViscous            - viscous friction parameter for negative velocities.
 */
public class AsymmetricCoulombViscousFrictionModel extends JointFrictionModel
{
   private static final int NUMBER_OF_PARAMETERS = 4;

   private double positiveCoulomb, positiveViscous, negativeCoulomb, negativeViscous;
   private double friction;
   
   public AsymmetricCoulombViscousFrictionModel()
   {
      this(0.0, 0.0, 0.0, 0.0);
   }

   public AsymmetricCoulombViscousFrictionModel(double positiveCoulomb, double positiveViscous, double negativeCoulomb,
         double negativeViscous)
   {
      super(FrictionModel.ASYMMETRIC_COULOMB_VISCOUS, NUMBER_OF_PARAMETERS);

      this.positiveCoulomb = Math.abs(positiveCoulomb);
      this.positiveViscous = Math.abs(positiveViscous);
      this.negativeCoulomb = Math.abs(negativeCoulomb);
      this.negativeViscous = Math.abs(negativeViscous);
   }

   @Override
   public void computeFrictionForce(double qd)
   {
      if (qd >= 0)
      {
         friction = Math.signum(qd) * positiveCoulomb + qd * positiveViscous;
      }
      else
      {
         friction = Math.signum(qd) * negativeCoulomb + qd * negativeViscous;
      }
   }

   @Override
   public double getFrictionForce()
   {
      return friction;
   }

   @Override
   public void updateParameters(double[] newParameters)
   {
      if (newParameters.length == numberOfParameters)
      {
         this.positiveCoulomb = Math.abs(newParameters[0]);
         this.positiveViscous = Math.abs(newParameters[1]);
         this.negativeCoulomb = Math.abs(newParameters[2]);
         this.negativeViscous = Math.abs(newParameters[3]);
      }
      else
      {
         throw new UnsupportedOperationException("Wrong number of parameters for friction model " + model.name());
      }
   }

   @Override
   public double[] getSuitableInitialValues(double ratio)
   {
      return new double[] {ratio, ratio/20.0, ratio, ratio/20.0};
   }

}
