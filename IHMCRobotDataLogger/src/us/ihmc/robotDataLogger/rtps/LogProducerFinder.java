package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.concurrent.LinkedBlockingQueue;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.listeners.LogAnnouncementListener;

public class LogProducerFinder
{
   private final LinkedBlockingQueue<Announcement> sessions = new LinkedBlockingQueue<>();

   
   public LogProducerFinder(DataConsumerParticipant participant) throws IOException
   {
      participant.listenForAnnouncements(new LogSessionCallback());
   }
   
   public Announcement getAnnounceRequestByIP(String IPAdress)
   {
      PrintTools.info("Looking for: " + IPAdress + " to come online");
      Announcement announcement;
      try
      {
         while ((announcement = sessions.take()) != null)
         {
            if(LogProducerDisplay.ipToString(announcement.getDataIP()).equals(IPAdress))
            {
               return announcement;
            }
           
         }
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }

      return null;
   }

   
   private class LogSessionCallback implements LogAnnouncementListener
   {
      public LogSessionCallback()
      {
      }

      @Override
      public void logSessionCameOnline(final Announcement description)
      {
         try
         {
            sessions.put(description);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }

      }

      @Override
      public void logSessionWentOffline(Announcement description)
      {
         
      }
   }
}
