package us.ihmc.communication.blackoutGenerators;

import java.util.Random;
import java.util.concurrent.TimeUnit;

public class TimeDependentStepFunctionBlackoutGenerator implements CommunicationBlackoutGenerator
{
   private final TimeUnit timeUnit;
   private long maxBlackoutLength;
   private long runLength;
   
   private Random random = new Random(System.currentTimeMillis());
   
   public TimeDependentStepFunctionBlackoutGenerator(TimeUnit timeUnit, long maxBlackoutLength, long runLength)
   {
      this.timeUnit = timeUnit;
      this.maxBlackoutLength = maxBlackoutLength;
      this.runLength = runLength;
   }
   
   @Override
   public long calculateNextBlackoutLength(long currentTime, TimeUnit timeUnit)
   {
      currentTime = this.timeUnit.convert(currentTime, timeUnit);
      long blackoutLength = 0;
      double runCompletionRatio = (double)currentTime/(double)runLength;
      if(runCompletionRatio <= (1.0/3.0))
      {
         blackoutLength = Math.abs(random.nextLong()) & maxBlackoutLength;
      }
      else if(runCompletionRatio <= (2.0/3.0))
      {
         double upperBound = maxBlackoutLength * (2.0/3.0);
         if(upperBound == 0.0)
            blackoutLength = 0;
         else
            blackoutLength = random.nextInt((int)Math.round(upperBound));
      }
      else
      {
         double upperBound = maxBlackoutLength * (1.0/3.0);
         if(upperBound == 0.0)
            blackoutLength = 0;
         else
            blackoutLength = random.nextInt((int)Math.round(upperBound));
      }
      
      return timeUnit.convert(blackoutLength + 1, this.timeUnit);
   }

}
