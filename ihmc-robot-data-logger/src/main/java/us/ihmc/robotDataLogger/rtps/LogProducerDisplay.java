package us.ihmc.robotDataLogger.rtps;

import us.ihmc.robotDataLogger.Announcement;

@Deprecated
public class LogProducerDisplay
{

   @Deprecated
   /**
    * Not neccessary anymore
    * 
    * @author Jesper Smith
    *
    */
   public interface RobotIPToNameRemapHandler
   {
      String getRemap(String ipAddress);
   }

   @Deprecated
   /**
    * Not used anymore. 
    * 
    * Disable autoDiscovery and hardcode hosts.
    * 
    * @author Jesper Smith
    *
    */
   public interface LogSessionFilter
   {
      boolean shouldAddToDisplay(Announcement description);
   }

}
