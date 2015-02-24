package us.ihmc.communication.blackoutGenerators;

public interface CommunicationBlackoutSimulator
{
   public boolean blackoutCommunication();
   public long getCurrentTime();
}
