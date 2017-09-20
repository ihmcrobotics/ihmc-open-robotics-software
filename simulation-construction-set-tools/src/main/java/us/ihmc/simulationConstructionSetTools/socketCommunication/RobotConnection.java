package us.ihmc.simulationConstructionSetTools.socketCommunication;

public interface RobotConnection
{
   public boolean isConnected();
   
   public void attemptConnectionToHost();
   public void disconnect();
   
   public void pause();
   
   
   public void setRecord(boolean record);
   
}
