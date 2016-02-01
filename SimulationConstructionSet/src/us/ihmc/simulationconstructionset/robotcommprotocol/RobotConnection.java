package us.ihmc.simulationconstructionset.robotcommprotocol;

public interface RobotConnection
{
   public boolean isConnected();
   
   public void attemptConnectionToHost();
   public void disconnect();
   
   public void pause();
   
   
   public void setRecord(boolean record);
   
}
