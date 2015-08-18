package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;


public class ConstantBlackoutGenerator implements CommunicationBlackoutGenerator
{
   private final long blackoutLength;
   private final TimeUnit timeUnit;
   
   public ConstantBlackoutGenerator(long blackoutLength, TimeUnit timeUnit)
   {
      this.blackoutLength = blackoutLength;
      this.timeUnit = timeUnit;
   }

   @Override
   public long calculateNextBlackoutLength(long currentTime, TimeUnit timeUnit)
   {
      return timeUnit.convert(blackoutLength, this.timeUnit);
   }

}
