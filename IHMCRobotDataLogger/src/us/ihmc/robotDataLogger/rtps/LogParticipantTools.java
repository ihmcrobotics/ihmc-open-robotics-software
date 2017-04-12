package us.ihmc.robotDataLogger.rtps;

import us.ihmc.pubsub.common.Guid;

public class LogParticipantTools
{
   public static String createGuidString(Guid guid)
   {
      StringBuilder guidBuilder = new StringBuilder();
      guidBuilder.append("{");
      for (byte val : guid.getGuidPrefix().getValue())
      {
         guidBuilder.append(String.format("%02x", val));
      }
      //      guidBuilder.append("-");
      //      for(byte val : guid.getEntity().getValue())
      //      {
      //         guidBuilder.append(String.format("%02x", val));
      //      }
      guidBuilder.append("}");
      return guidBuilder.toString();

   }
}
