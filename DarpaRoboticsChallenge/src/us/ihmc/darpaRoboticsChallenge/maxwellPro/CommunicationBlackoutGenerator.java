package us.ihmc.darpaRoboticsChallenge.maxwellPro;

public interface CommunicationBlackoutGenerator
{
   public int calculateNextBlackoutLength(int currentTime);
}
