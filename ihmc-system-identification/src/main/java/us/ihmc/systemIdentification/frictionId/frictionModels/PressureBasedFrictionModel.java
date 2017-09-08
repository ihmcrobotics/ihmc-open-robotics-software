package us.ihmc.systemIdentification.frictionId.frictionModels;

/**
 * 
 * Pressure based asymmetric friction model.
 * Based on paper (A. Bonchis - 2006).
 * 
 * @param x1, x2, x3, x4, x5 - for the other parameters refer to paper.
 */
public class PressureBasedFrictionModel extends JointFrictionModel
{
   private static final int NUMBER_OF_PARAMETERS = 5;

   private double x1, x2, x3, x4, x5;
   private double friction;
   
   public PressureBasedFrictionModel()
   {
      this(0.0, 0.0, 0.0, 0.0, 0.0);
   }

   public PressureBasedFrictionModel(double x1, double x2, double x3, double x4, double x5)
   {
      super(FrictionModel.PRESSURE_BASED, NUMBER_OF_PARAMETERS);

      this.x1 = x1;
      this.x2 = x2;
      this.x3 = x3;
      this.x4 = x4;
      this.x5 = x5;
   }

   public void computeFrictionForce(double qd, double negPressure, double posPressure)
   {
      friction = x1 * Math.exp(x2 * qd) + x3 * (posPressure - negPressure) + x4 * negPressure + x5 * qd;
   }

   @Deprecated
   public void computeFrictionForce(double qd)
   {
      throw new UnsupportedOperationException("This method is not applicabile for the pressure based friction model. Use the method with presures instead.");
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
         this.x1 = newParameters[0];
         this.x2 = newParameters[1];
         this.x3 = newParameters[2];
         this.x4 = newParameters[3];
         this.x5 = newParameters[4];
      }
      else
      {
         throw new UnsupportedOperationException("Wrong number of parameters for friction model " + model.name());
      }
   }

   @Override
   public double[] getSuitableInitialValues(double ratio)
   {
      return new double[] {ratio, ratio/10.0, ratio/100.0, ratio/100.0, ratio/10.0};
   }

}
