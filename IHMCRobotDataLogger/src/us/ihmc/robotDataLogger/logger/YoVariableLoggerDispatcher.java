package us.ihmc.robotDataLogger.logger;

import java.io.IOException;
import java.util.concurrent.LinkedBlockingQueue;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.listeners.LogAnnouncementListener;
import us.ihmc.robotDataLogger.rtps.DataConsumerParticipant;

public class YoVariableLoggerDispatcher implements LogAnnouncementListener
{
   private final DataConsumerParticipant participant;

   private final LinkedBlockingQueue<Announcement> announcenments = new LinkedBlockingQueue<>();

   public YoVariableLoggerDispatcher(YoVariableLoggerOptions options) throws IOException
   {

      System.out.println("Starting YoVariableLoggerDispatcher");
      
      participant = new DataConsumerParticipant("YoVariableLoggerDispatcher");
      participant.listenForAnnouncements(this);
      System.out.println("Client started, waiting for announcements");
      
      
      Announcement announcement;
      try
      {
         while((announcement = announcenments.take()) != null)
         {
            System.out.println("New control session came online " + announcement);
            if (announcement.getLog())
            {
               System.out.println("Logging sesion " + announcement.getNameAsString() + " on " + announcement.getHostNameAsString());
               try
               {
                  new YoVariableLogger(announcement, options);
                  System.out.println("Logging session started");
               }
               catch (Exception e)
               {
                  e.printStackTrace();
               }
            }
         }
      }
      catch (InterruptedException e)
      {
         System.err.println("Variable server go interrupted, shutting down");         
      }
      
      participant.remove();
      
      
   }

   public static void main(String[] args) throws JSAPException, IOException
   {
      YoVariableLoggerOptions options = YoVariableLoggerOptions.parse(args);
      new YoVariableLoggerDispatcher(options);
   }

   @Override
   public void logSessionCameOnline(Announcement request)
   {
      try
      {
         announcenments.put(request);
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
