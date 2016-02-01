package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

public class UserSettableBlackoutGenerator implements CommunicationBlackoutGenerator
{
   private final TimeUnit timeUnit;
   private final AtomicLong blackoutLength = new AtomicLong();
   
   public UserSettableBlackoutGenerator(TimeUnit timeUnit)
   {
      this.timeUnit = timeUnit;
   }

   @Override
   public long calculateNextBlackoutLength(long currentTime, TimeUnit timeUnit)
   {
      return timeUnit.convert(blackoutLength.get(), this.timeUnit);
   }

   public void setBlackoutDuration(long blackoutDuration, TimeUnit timeUnit)
   {
      blackoutLength.set(this.timeUnit.convert(blackoutDuration, timeUnit));
   }

}
