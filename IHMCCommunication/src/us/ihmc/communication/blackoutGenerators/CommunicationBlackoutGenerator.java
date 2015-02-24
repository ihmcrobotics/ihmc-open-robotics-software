package us.ihmc.communication.blackoutGenerators;

public interface CommunicationBlackoutGenerator
{
   public long calculateNextBlackoutLength(long currentTime);
}
