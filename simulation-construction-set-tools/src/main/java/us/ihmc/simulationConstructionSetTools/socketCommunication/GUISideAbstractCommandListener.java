package us.ihmc.simulationConstructionSetTools.socketCommunication;

public interface GUISideAbstractCommandListener
{

   public abstract void doHello(String name, String info);

   public abstract void doAllRegistriesAndVariables(String[] registryNames, String[][] variableNames, float[][] initialValues);

   public abstract void doRegistrySettingsProcessed(int[] registryIndices, boolean[] isSent, boolean[] isDisallowSendingSet, boolean[] isLogged, int registrySettingsIdentifier);

   public abstract void doSet(int index, float value);

   public abstract void doPeriod(int periodmsec);

   public abstract void doDisconnect();

   public abstract void doUserCommand(String command);

   public abstract void doData(float[] data);

   public abstract void doTextMessage(String message);

}