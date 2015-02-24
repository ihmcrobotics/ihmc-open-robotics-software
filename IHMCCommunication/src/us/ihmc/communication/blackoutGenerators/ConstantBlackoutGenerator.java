package us.ihmc.communication.blackoutGenerators;


public class ConstantBlackoutGenerator implements CommunicationBlackoutGenerator
{
   private final long blackoutLength;
   
   public ConstantBlackoutGenerator(long blackoutLength)
   {
      this.blackoutLength = blackoutLength;
   }

   @Override
   public long calculateNextBlackoutLength(long currentTime)
   {
      return blackoutLength;
   }

}
