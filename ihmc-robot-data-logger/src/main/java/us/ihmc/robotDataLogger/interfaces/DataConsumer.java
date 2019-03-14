package us.ihmc.robotDataLogger.interfaces;

import java.io.IOException;

import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.YoVariableClientImplementation;
import us.ihmc.robotDataLogger.handshake.IDLYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

public interface DataConsumer
{

   /**
    * Requests the model file 
    *  
    * 
    * @return byte[] array of the model file
    * 
    * @throws IOException if no reply has been received within  the timeout
    * @throws RuntimeException if no model file is announced in the announcement
    */
   byte[] getModelFile() throws IOException;

   /**
    * Requests the resource zip 
    *  
    * 
    * @return byte[] array of the resource bundle
    * 
    * @throws IOException if no reply has been received within the timeout
    * @throws RuntimeException if no resource bundle is announced in the announcement
    */
   byte[] getResourceZip() throws IOException;

   /**
    * Request the handshake 
    * 
    * 
    * @return Handshake
    * 
    * @throws IOException if no reply has been received within the timeout
    */
   Handshake getHandshake() throws IOException;

   /**
    * Send a command to the server. 
    * 
    * If the command is a broadcast command, this gets broadcast to all clients (including this one) by the server.
    * 
    * If no session is available, this request gets silently ignored.
    * 
    * @throws IOException
    */
   void sendCommand(DataServerCommand command, int argument) throws IOException;

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
                      TimestampListener timeStampListener, CommandListener clearLogListener, DebugRegistry rtpsDebugRegistry) throws IOException;

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
   boolean reconnect() throws IOException;

   /**
    * Send a request to the session to change a variable
    * 
    * @param identifier
    * @param valueAsDouble
    */
   void writeVariableChangeRequest(int identifier, double valueAsDouble);


}