package us.ihmc.robotDataLogger.rtps;

import us.ihmc.robotDataLogger.Announcement;

/**
 * Interface to show notify logging sessions coming and going
 * 
 * @author jesper
 *
 */
public interface LogAnnouncementListener
{
   /**
    * A new session came online
    * 
    * @param announcement
    */
   public void logSessionCameOnline(Announcement announcement);
   
   /**
    * A session went offline. This only gets called when the participant is destroyed neatly
    * @param announcement
    */
   public void logSessionWentOffline(Announcement announcement);

}
