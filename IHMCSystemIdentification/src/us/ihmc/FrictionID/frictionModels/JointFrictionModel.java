package us.ihmc.FrictionID.frictionModels;

public abstract class JointFrictionModel
{
   protected final FrictionModel model;
   protected final int numberOfParameters;

   /**
    *  Call this constructor from the class that extends the JointFrictionModel.
    *  
    *  @param model                      - specify the FrictionModel that is implemented.
    *  @param numberOfParameters         - declare the number of parameters used by the friction model.
    */
   public JointFrictionModel(FrictionModel model, int numberOfParameters)
   {
      this.numberOfParameters = numberOfParameters;
      this.model = model;
   }

   public FrictionModel getFrictionModel()
   {
      return model;
   }

   public int getNumberOfParameters()
   {
      return numberOfParameters;
   }

   /**
    * Use this method to update the friction parameters.
    * 
    * @param newParameters - ordered array containing the friction parameters as specified in the constructor.
    */
   public abstract void updateParameters(double[] newParameters);

   /**
    * Use this method to compute the friction force inside of the model.
    * 
    * @param qd - velocity for friction calculation.
    */
   public abstract void computeFrictionForce(double qd);

   /**
    * Use this method to get the friction force inside of the model.
    *
    */
   public abstract double getFrictionForce();
   
   /**
    * If you are doing a parameter identification use this method to get a set of suitable initial value for this model.
    * 
    * @param ratio - multiplier to scale the parameters, usually the Coulomb friction absolute value.
    */
   public abstract double[] getSuitableInitialValues(double ratio);
}