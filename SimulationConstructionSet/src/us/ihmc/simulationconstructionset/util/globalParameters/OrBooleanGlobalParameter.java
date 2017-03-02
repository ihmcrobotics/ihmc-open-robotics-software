package us.ihmc.simulationconstructionset.util.globalParameters;

public class OrBooleanGlobalParameter extends BooleanGlobalParameter
{
   private final BooleanGlobalParameter[] booleanGlobalParametersToOr;

   public OrBooleanGlobalParameter(String name, String description, BooleanGlobalParameter[] booleanGlobalParametersToOr,
                                   GlobalParameterChangedListener listener)
   {
      super(name, description, booleanGlobalParametersToOr, listener);

      this.booleanGlobalParametersToOr = booleanGlobalParametersToOr;
      update("computing initial value from parents.");
   }

   @Override
   public void update(String comment)
   {
      boolean value = false;

      for (BooleanGlobalParameter booleanGlobalParameter : booleanGlobalParametersToOr)
      {
         value = value || booleanGlobalParameter.getValue();
      }

      this.setBooleanValue(value, comment + ", multiplicative");
   }


}
