package us.ihmc.darpaRoboticsChallenge.maxwellPro.blackoutGenerators;

import java.util.Random;

import us.ihmc.darpaRoboticsChallenge.maxwellPro.MaxwellProBlackoutGenerator;

public class TimeDependentGaussianBlackoutGenerator implements MaxwellProBlackoutGenerator
{
   private int maxBlackoutLength;
   private int runLength;
   private int blackoutSlopeOffset;
   
   private Random random = new Random(System.currentTimeMillis());
   
   public TimeDependentGaussianBlackoutGenerator(int maxBlackoutLength, int runLength, int blackoutSlopeOffset)
   {
      this.maxBlackoutLength = maxBlackoutLength;
      this.runLength = runLength;
      this.blackoutSlopeOffset = blackoutSlopeOffset;
   }

   @Override
   public int calculateNextBlackoutLength(int currentTime)
   {
      double m = (double)((2 * blackoutSlopeOffset) - maxBlackoutLength)/(double)runLength;
      int y0 = maxBlackoutLength - blackoutSlopeOffset;
      double gaussianOffset = 0.1;
      int blackoutLength = (int)Math.round(m * currentTime + y0);
      int gaussian = (int) (random.nextGaussian() * (maxBlackoutLength/8) - gaussianOffset);
      
      blackoutLength += gaussian;
      
      if(blackoutLength > maxBlackoutLength)
         blackoutLength = maxBlackoutLength;
      
      if(blackoutLength < 0)
         blackoutLength = 0;
      
      return blackoutLength;
   }
}
