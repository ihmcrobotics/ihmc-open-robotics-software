package us.ihmc.simulationconstructionset.util.globalParameters;

public class SystemOutGlobalParameterChangedListener implements GlobalParameterChangedListener
{
   public void booleanValueChanged(GlobalParameter globalParameter, String comment, boolean previousValue, boolean newValue)
   {
      printToSystemOut(globalParameter, comment, ((Boolean) previousValue).toString(), ((Boolean) newValue).toString());
   }

   public void doubleValueChanged(GlobalParameter globalParameter, String comment, double previousValue, double newValue)
   {
      printToSystemOut(globalParameter, comment, ((Double) previousValue).toString(), ((Double) newValue).toString());

   }

   public void enumValueChanged(GlobalParameter globalParameter, String comment, Enum<?> previousValue, Enum<?> newValue)
   {
      printToSystemOut(globalParameter, comment, ((Enum<?>) previousValue).toString(), ((Enum<?>) newValue).toString());
   }

   public void globalParameterCreated(GlobalParameter globalParameter)
   {
      System.out.println(globalParameter.getName() + " created" + ", value=" + globalParameter.getValueInStringFormat());
   }

   public void integerValueChanged(GlobalParameter globalParameter, String comment, int previousValue, int newValue)
   {
      printToSystemOut(globalParameter, comment, ((Integer) previousValue).toString(), ((Integer) newValue).toString());

   }

   private void printToSystemOut(GlobalParameter globalParameter, String comment, String previousValue, String newValue)
   {
      System.out.println(globalParameter.getName() + ", previousValue=" + previousValue + ", newValue= " + newValue + " " + comment);
   }

}
