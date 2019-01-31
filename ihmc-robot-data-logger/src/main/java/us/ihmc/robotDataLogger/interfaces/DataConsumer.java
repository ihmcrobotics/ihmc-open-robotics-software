package us.ihmc.robotDataLogger.interfaces;

import java.io.IOException;

import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.listeners.ClearLogListener;
import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.rtps.RTPSDebugRegistry;
import us.ihmc.robotDataLogger.rtps.VariableChangedProducer;

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
   byte[] getModelFile(int timeout) throws IOException;

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
   byte[] getResourceZip(int timeout) throws IOException;

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
   Handshake getHandshake(int timeout) throws IOException;

   /**
    * Broadcast a clear log request for the current session
    * 
    * If no session is available, this request gets silently ignored.
    * 
    * @throws IOException
    */
   void sendClearLogRequest() throws IOException;

   /**
    * Start a new session
    * 
    * @param parser
    * @param yoVariableClient
    * @param variableChangedProducer
    * @param timeStampListener
    * @param clearLogListener
    * @param rtpsDebugRegistry
    */
   void startSession(IDLYoVariableHandshakeParser parser, YoVariableClientImplementation yoVariableClient, VariableChangedProducer variableChangedProducer,
                      TimestampListener timeStampListener, ClearLogListener clearLogListener, RTPSDebugRegistry rtpsDebugRegistry);

   /**
    * 
    * @return true if the current session is connected
    */
   boolean isSessionActive();
   
   /**
    * Disconnect the session, but allow to reconnect
    */
   void disconnectSession();

   /**
    * Close the connection completely. This makes reconnecting impossible.
    */
   void close();

   /**
    * @return true if the consumer is closed.
    */
   boolean isClosed();

   /**
    * Reconnect to the same host to continue the log.
    * 
    * @return true if the reconnect is successful
    */
   boolean reconnect();


}