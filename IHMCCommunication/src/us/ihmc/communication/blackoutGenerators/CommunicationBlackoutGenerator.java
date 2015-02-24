package us.ihmc.communication.blackoutGenerators;

public interface CommunicationBlackoutGenerator
{
   public int calculateNextBlackoutLength(int currentTime);
}
