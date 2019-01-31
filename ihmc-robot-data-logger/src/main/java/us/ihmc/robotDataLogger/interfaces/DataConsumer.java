package us.ihmc.robotDataLogger.interfaces;

import java.io.IOException;

import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.Handshake;

public interface DataConsumer
{

   /**
    * Requests the model file 
    *  
    * @param announcement
    * 
    * @return byte[] array of the model file
    * 
    * @throws IOException if no reply has been received within  the timeout
    * @throws RuntimeException if no model file is announced in the announcement
    */
   byte[] getModelFile(Announcement announcement, int timeout) throws IOException;

   /**
    * Requests the resource zip 
    *  
    * @param announcement
    * 
    * @return byte[] array of the resource bundle
    * 
    * @throws IOException if no reply has been received within the timeout
    * @throws RuntimeException if no resource bundle is announced in the announcement
    */
   byte[] getResourceZip(Announcement announcement, int timeout) throws IOException;

   /**
    * Request the handshake 
    * 
    * 
    * @param announcement 
    * 
    * @return Handshake
    * 
    * @throws IOException if no reply has been received within the timeout
    */
   Handshake getHandshake(Announcement announcement, int timeout) throws IOException;

   /**
    * Remove the participant from the domain.
    * 
    * After calling this function 
    */
   void remove();

   /**
    * Broadcast a clear log request for the current session
    * 
    * If no session is available, this request gets silently ignored.
    * 
    * @throws IOException
    */
   void sendClearLogRequest() throws IOException;

}