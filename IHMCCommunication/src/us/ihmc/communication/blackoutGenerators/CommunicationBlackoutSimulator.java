package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;


public interface CommunicationBlackoutSimulator
{
   public boolean blackoutCommunication();
   public long getCurrentTime(TimeUnit timeUnit);
   public void enableBlackouts(boolean enable);
   public CommunicationBlackoutGenerator getBlackoutGenerator();
}
