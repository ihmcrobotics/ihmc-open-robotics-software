package us.ihmc.simulationconstructionset.util.globalParameters;


public class MultiplicativeDoubleGlobalParameter extends DoubleGlobalParameter
{
   private final DoubleGlobalParameter[] globalParametersToMultiply;

   public MultiplicativeDoubleGlobalParameter(String name, String description, DoubleGlobalParameter[] globalParametersToMultiply,
           GlobalParameterChangedListener listener)
   {
      super(name, description, globalParametersToMultiply, listener);

      this.globalParametersToMultiply = globalParametersToMultiply;
      update("computing initial value from parents.");
   }

   @Override
   public void update(String comment)
   {
      double value = 1.0;

      for (DoubleGlobalParameter doubleGlobalParameter : globalParametersToMultiply)
      {
         value = value * doubleGlobalParameter.getValue();
      }

      this.setDoubleValue(value, comment + ", multiplicative");
   }

}
