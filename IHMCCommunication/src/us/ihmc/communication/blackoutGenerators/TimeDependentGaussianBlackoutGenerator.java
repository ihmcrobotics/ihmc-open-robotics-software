package us.ihmc.communication.blackoutGenerators;

import java.util.Random;

public class TimeDependentGaussianBlackoutGenerator implements CommunicationBlackoutGenerator
{
   private long maxBlackoutLength;
   private long runLength;
   private long blackoutSlopeOffset;
   
   private Random random = new Random(System.currentTimeMillis());
   
   public TimeDependentGaussianBlackoutGenerator(long maxBlackoutLength, long runLength, long blackoutSlopeOffset)
   {
      this.maxBlackoutLength = maxBlackoutLength;
      this.runLength = runLength;
      this.blackoutSlopeOffset = blackoutSlopeOffset;
   }

   @Override
   public long calculateNextBlackoutLength(long currentTime)
   {
      double m = (double)((2 * blackoutSlopeOffset) - maxBlackoutLength)/(double)runLength;
      long y0 = maxBlackoutLength - blackoutSlopeOffset;
      double gaussianOffset = 0.1;
      long blackoutLength = Math.round(m * currentTime + y0);
      long gaussian = (long) (random.nextGaussian() * (maxBlackoutLength/8) - gaussianOffset);
      
      blackoutLength += gaussian;
      
      if(blackoutLength > maxBlackoutLength)
         blackoutLength = maxBlackoutLength;
      
      if(blackoutLength < 0)
         blackoutLength = 0;
      
      return blackoutLength;
   }
}
