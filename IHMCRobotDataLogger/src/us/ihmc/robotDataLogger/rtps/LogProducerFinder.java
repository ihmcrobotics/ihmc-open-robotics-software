package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.HashMap;
import java.util.concurrent.LinkedBlockingQueue;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import us.ihmc.commons.PrintTools;
import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.rtps.LogProducerDisplay.LogSessionFilter;

public class LogProducerFinder
{
   private final LinkedBlockingQueue<Announcement> sessions = new LinkedBlockingQueue<>();

   
   public LogProducerFinder(DataConsumerParticipant participant) throws IOException
   {
      participant.listenForAnnouncements(new LogSessionCallback());
   }
   
   public Announcement getAnnounceRequestByIP(String IPAdress)
   {

      return getAnnounceRequestByIP(IPAdress, 100000000);
   }

   public Announcement getAnnounceRequestByIP(String IPAdress, int timeOut)
   {
      int timeOutCounter = 0;
      
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
