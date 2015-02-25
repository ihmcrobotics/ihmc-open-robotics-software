package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;

public interface CommunicationBlackoutGenerator
{
   public long calculateNextBlackoutLength(long currentTime, TimeUnit timeUnit);
}
