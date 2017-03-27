package us.ihmc.simulationConstructionSetTools.socketCommunication;

public interface UserCommandListener
{
   // public void doUserCommand(String string);
   public void doUserCommand(String command, UserCommandNetworkReader reader);
}
