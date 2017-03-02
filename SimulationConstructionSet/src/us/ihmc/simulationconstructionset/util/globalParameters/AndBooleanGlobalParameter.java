package us.ihmc.simulationconstructionset.util.globalParameters;

public class AndBooleanGlobalParameter extends BooleanGlobalParameter
{
   private final BooleanGlobalParameter[] booleanGlobalParametersToAnd;

   public AndBooleanGlobalParameter(String name, String description, BooleanGlobalParameter[] booleanGlobalParametersToAnd,
                                    GlobalParameterChangedListener listener)
   {
      super(name, description, booleanGlobalParametersToAnd, listener);

      this.booleanGlobalParametersToAnd = booleanGlobalParametersToAnd;
      update("computing initial value from parents.");
   }

   @Override
   public void update(String comment)
   {
      boolean value = true;

      for (BooleanGlobalParameter booleanGlobalParameter : booleanGlobalParametersToAnd)
      {
         value = value && booleanGlobalParameter.getValue();
      }

      this.setBooleanValue(value, comment + ", multiplicative");
   }


}
