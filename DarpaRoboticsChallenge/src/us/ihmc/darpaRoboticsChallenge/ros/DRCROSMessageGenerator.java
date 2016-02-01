package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.utilities.ros.ROSMessageGenerator;

public class DRCROSMessageGenerator
{
   public static void generate() throws Exception
   {
      ROSMessageGenerator messageGenerator = new ROSMessageGenerator(true);
      for (Class clazz : IHMCRosApiMessageMap.PACKET_LIST)
      {
         messageGenerator.createNewRosMessage(clazz, true);
      }
   }

   public static void main(String... args) throws Exception
   {
      generate();
   }
}
