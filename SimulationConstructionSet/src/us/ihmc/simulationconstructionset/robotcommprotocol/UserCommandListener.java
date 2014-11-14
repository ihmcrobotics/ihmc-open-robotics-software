package us.ihmc.simulationconstructionset.robotcommprotocol;

public interface UserCommandListener
{
   // public void doUserCommand(String string);
   public void doUserCommand(String command, UserCommandNetworkReader reader);
}
