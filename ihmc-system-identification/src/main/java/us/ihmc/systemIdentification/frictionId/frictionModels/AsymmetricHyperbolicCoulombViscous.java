package us.ihmc.systemIdentification.frictionId.frictionModels;

/**
 * 
 * Asymmetric friction force based on coulomb + hyperbolic tangent + viscous model.
 * All parameters must be >= 0.
 * 
 * @param positiveCoulomb            - Coulomb friction parameter for positive velocities.
 * @param positiveHyperCoefficient   - hyperbolic component for positive velocities.
 * @param positiveViscous            - viscous friction parameter for positive velocities.
 * @param negativeCoulomb            - Coulomb friction parameter for negative velocities.
 * @param negativeHyperCoefficient   - hyperbolic component for negative velocities.
 * @param negativeViscous            - viscous friction parameter for negative velocities.
 */

public class AsymmetricHyperbolicCoulombViscous extends JointFrictionModel
{
   private static final int NUMBER_OF_PARAMETERS = 6;

   private double positiveCoulomb, positiveHyperCoefficient, positiveViscous, negativeCoulomb, negativeHyperCoefficient, negativeViscous;
   private double friction;
   
   public AsymmetricHyperbolicCoulombViscous()
   {
      this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }
   
   public AsymmetricHyperbolicCoulombViscous(double positiveCoulomb, double positiveHyperCoefficient, double positiveViscous, double negativeCoulomb,
                                             double negativeHyperCoefficient, double negativeViscous)
   {
      super(FrictionModel.ASYMMETRIC_HYPERBOLIC_COULOMB_VISCOUS, NUMBER_OF_PARAMETERS);

      this.positiveCoulomb = Math.abs(positiveCoulomb);
      this.positiveHyperCoefficient = Math.abs(positiveHyperCoefficient);
      this.positiveViscous = Math.abs(positiveViscous);
      this.negativeCoulomb = Math.abs(negativeCoulomb);
      this.negativeHyperCoefficient = Math.abs(negativeHyperCoefficient);
      this.negativeViscous = Math.abs(negativeViscous);
   }

   @Override
   public void computeFrictionForce(double qd)
   {
      if (qd >= 0)
      {
         friction = Math.signum(qd) * positiveCoulomb + Math.tanh(qd) * positiveHyperCoefficient + qd * positiveViscous;
      }
      else
      {
         friction = Math.signum(qd) * negativeCoulomb + Math.tanh(qd) * negativeHyperCoefficient + qd * negativeViscous;
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
         this.positiveHyperCoefficient = Math.abs(newParameters[1]);
         this.positiveViscous = Math.abs(newParameters[2]);
         this.negativeCoulomb = Math.abs(newParameters[3]);
         this.negativeHyperCoefficient = Math.abs(newParameters[4]);
         this.negativeViscous = Math.abs(newParameters[5]);
      }
      else
      {
         throw new UnsupportedOperationException("Wrong number of parameters for friction model " + model.name());
      }
   }

   @Override
   public double[] getSuitableInitialValues(double ratio)
   {
      return new double[] {ratio, ratio/20.0, ratio/20.0, ratio, ratio/20.0, ratio/20.0};
   }

}
