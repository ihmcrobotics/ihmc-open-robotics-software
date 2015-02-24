package us.ihmc.communication.blackoutGenerators;


public class ConstantBlackoutGenerator implements CommunicationBlackoutGenerator
{
   private final int blackoutLength;
   
   public ConstantBlackoutGenerator(int blackoutLength)
   {
      this.blackoutLength = blackoutLength;
   }

   @Override
   public int calculateNextBlackoutLength(int currentTime)
   {
      return blackoutLength;
   }

}
