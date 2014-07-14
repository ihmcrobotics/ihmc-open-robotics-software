package us.ihmc.darpaRoboticsChallenge.handControl;

public interface HandCommandManager
{
   public void spawnHandControllerThreadManager();
   
   public void sendHandCommand(Object packet);
   
   public String getTcpPort();
}
