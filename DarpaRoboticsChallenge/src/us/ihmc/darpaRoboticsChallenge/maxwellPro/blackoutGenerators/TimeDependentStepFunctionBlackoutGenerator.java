package us.ihmc.darpaRoboticsChallenge.maxwellPro.blackoutGenerators;

import java.util.Random;

import us.ihmc.darpaRoboticsChallenge.maxwellPro.MaxwellProBlackoutGenerator;

public class TimeDependentStepFunctionBlackoutGenerator implements MaxwellProBlackoutGenerator
{
   private int maxBlackoutLength;
   private int runLength;
   
   private Random random = new Random(System.currentTimeMillis());
   
   public TimeDependentStepFunctionBlackoutGenerator(int maxBlackoutLength, int runLength)
   {
      this.maxBlackoutLength = maxBlackoutLength;
      this.runLength = runLength;
   }
   
   @Override
   public int calculateNextBlackoutLength(int currentTime)
   {
      int blackoutLength = 0;
      double runCompletionRatio = (double)currentTime/(double)runLength;
      if(runCompletionRatio <= (1.0/3.0))
      {
         blackoutLength = random.nextInt(maxBlackoutLength);
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
