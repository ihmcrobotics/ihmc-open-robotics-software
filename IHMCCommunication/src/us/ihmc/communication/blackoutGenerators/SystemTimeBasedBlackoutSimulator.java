package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;

public class SystemTimeBasedBlackoutSimulator extends StandardBlackoutSimulator
{
   public SystemTimeBasedBlackoutSimulator(CommunicationBlackoutGenerator blackoutGenerator)
   {
      super(blackoutGenerator);
   }

   @Override
   public long getCurrentTime(TimeUnit timeUnit)
   {
      return timeUnit.convert(System.currentTimeMillis(), TimeUnit.MILLISECONDS);
   }
}
