package us.ihmc.communication.blackoutGenerators;

import java.util.Random;

public class TimeDependentStepFunctionBlackoutGenerator implements CommunicationBlackoutGenerator
{
   private long maxBlackoutLength;
   private long runLength;
   
   private Random random = new Random(System.currentTimeMillis());
   
   public TimeDependentStepFunctionBlackoutGenerator(long maxBlackoutLength, long runLength)
   {
      this.maxBlackoutLength = maxBlackoutLength;
      this.runLength = runLength;
   }
   
   @Override
   public long calculateNextBlackoutLength(long currentTime)
   {
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
      
      return blackoutLength + 1;
   }

}
