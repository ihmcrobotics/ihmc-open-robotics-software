package us.ihmc.systemIdentification.frictionId.frictionModels;

public class NoCompensationFrictionModel extends JointFrictionModel
{
   private static final int NUMBER_OF_PARAMETERS = 0;

   public NoCompensationFrictionModel()
   {
      super(FrictionModel.OFF, NUMBER_OF_PARAMETERS);
   }

   @Override
   public void computeFrictionForce(double qd)
   {

   }

   @Override
   public double getFrictionForce()
   {
      return 0.0;
   }

   @Override
   public void updateParameters(double[] newParameters)
   {

   }

   @Override
   public double[] getSuitableInitialValues(double ratio)
   {
      return null;
   }
}