package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.robotDataLogger.rtps.DataConsumerParticipant;
import us.ihmc.robotDataLogger.rtps.LogProducerDisplay;

public class YoVariableClient
{
   private final YoVariableClientImplementation yoVariableClientImplementation;
   private final LogProducerDisplay.LogSessionFilter[] sessionFilters;
   private final DataConsumerParticipant dataConsumerParticipant;

   /**
    * Start a new client while allowing the user to select a desired logging session
    * 
    * @param listener
    * @param filters
    */
   public YoVariableClient(YoVariablesUpdatedListener listener, LogProducerDisplay.LogSessionFilter... filters)
   {
      try
      {
         this.dataConsumerParticipant = new DataConsumerParticipant("YoVariableClient");
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      this.yoVariableClientImplementation = new YoVariableClientImplementation(dataConsumerParticipant, listener);
      this.sessionFilters = filters;
   }

   /**
    * Connect to an already selected log session
    * @param request
    * @param yoVariablesUpdatedListener
    */
   public YoVariableClient(DataConsumerParticipant participant, final YoVariablesUpdatedListener yoVariablesUpdatedListener)
   {
      this.dataConsumerParticipant = participant;
      this.yoVariableClientImplementation = new YoVariableClientImplementation(participant, yoVariablesUpdatedListener);
      this.sessionFilters = null;
   }

   public void start()
   {
      try
      {
         start(15000);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void start(int timeout) throws IOException
   {
      Announcement announcement = LogProducerDisplay.getAnnounceRequest(dataConsumerParticipant, sessionFilters);
      start(timeout, announcement);
   }

   public void start(int timeout, Announcement announcement) throws IOException
   {
      yoVariableClientImplementation.start(timeout, announcement);
   }
   
   
   public void reconnect() throws IOException
   {
      yoVariableClientImplementation.reconnect();
   }
   
}
