package us.ihmc.systemIdentification.frictionId.frictionModels;

/**
 * 
 * Asymmetric friction force based on coulomb + viscous + stribeck model.
 * All parameters must be >= 0.
 * Based on paper (Nissing - 2002)
 * 
 * @param positiveSigma              - parameter for viscous friction in case of positive velocity.
 * @param positiveFc0                - parameter for Coulomb friction in case of positive velocity.
 * @param positiveFs0                - parameter for static friction in case of positive velocity.
 * @param positiveCs                 - Stribeck velocity in case of positive velocity.
 * @param negativeSigma              - parameter for viscous friction in case of negative velocity.
 * @param negativeFc0                - parameter for Coulomb friction in case of negative velocity.
 * @param negativeFs0                - parameter for static friction in case of negative velocity.
 * @param negativeCs                 - Stribeck velocity in case of negative velocity.
 * 
 */
public class AsymmetricCoulombViscousStribeckFrictionModel extends JointFrictionModel
{
   private static final int NUMBER_OF_PARAMETERS = 8;

   private double positiveSigma, positiveFc0, positiveFs0, positiveCs, negativeSigma, negativeFc0, negativeFs0, negativeCs;
   private double friction;

   public AsymmetricCoulombViscousStribeckFrictionModel()
   {
      this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }
   
   public AsymmetricCoulombViscousStribeckFrictionModel(double positiveSigma, double positiveFc0, double positiveFs0,
         double positiveCs, double negativeSigma, double negativeFc0, double negativeFs0, double negativeCs)
   {
      super(FrictionModel.ASYMMETRIC_COULOMB_VISCOUS_STRIBECK, NUMBER_OF_PARAMETERS);

      this.positiveSigma = Math.abs(positiveSigma);
      this.positiveFc0 = Math.abs(positiveFc0);
      this.positiveFs0 = Math.abs(positiveFs0);
      this.positiveCs = Math.abs(positiveCs);
      this.negativeSigma = Math.abs(negativeSigma);
      this.negativeFc0 = Math.abs(negativeFc0);
      this.negativeFs0 = Math.abs(negativeFs0);
      this.negativeCs = Math.abs(negativeCs);
   }

   @Override
   public void computeFrictionForce(double qd)
   {
      if (qd >= 0)
      {
         friction = positiveSigma * qd + Math.signum(qd) * (positiveFc0 + positiveFs0 * Math.exp(-(Math.abs(qd) / positiveCs)));
      }
      else
      {
         friction = negativeSigma * qd + Math.signum(qd) * (negativeFc0 + negativeFs0 * Math.exp(-(Math.abs(qd) / negativeCs)));
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
         this.positiveSigma = Math.abs(newParameters[0]);
         this.positiveFc0 = Math.abs(newParameters[1]);
         this.positiveFs0 = Math.abs(newParameters[2]);
         this.positiveCs = Math.abs(newParameters[3]);
         this.negativeSigma = Math.abs(newParameters[4]);
         this.negativeFc0 = Math.abs(newParameters[5]);
         this.negativeFs0 = Math.abs(newParameters[6]);
         this.negativeCs = Math.abs(newParameters[7]);
      }
      else
      {
         throw new UnsupportedOperationException("Wrong number of parameters for friction model " + model.name());
      }
   }

   @Override
   public double[] getSuitableInitialValues(double ratio)
   {
      return new double[] {ratio/10.0, ratio, ratio/20.0, ratio/100.0, ratio/10.0, ratio, ratio/20.0, ratio/100.0};
   }
} 
