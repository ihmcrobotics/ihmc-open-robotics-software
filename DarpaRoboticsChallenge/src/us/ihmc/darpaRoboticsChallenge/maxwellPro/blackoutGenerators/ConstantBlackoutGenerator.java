package us.ihmc.darpaRoboticsChallenge.maxwellPro.blackoutGenerators;

import us.ihmc.darpaRoboticsChallenge.maxwellPro.CommunicationBlackoutGenerator;

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
