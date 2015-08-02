package us.ihmc.robotDataCommunication;

import java.net.SocketTimeoutException;
import java.util.List;

import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionDisplay;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class YoVariableClient
{
   private enum ClientState
   {
      WAITING, RUNNING, STOPPED
   }

   private final YoVariablesUpdatedListener listener;

   private final LogControlClient logControlClient;
   private final YoVariableConsumer yoVariableConsumer;
   private ClientState state = ClientState.WAITING;

   private final YoVariableHandshakeParser handshakeParser;

   public YoVariableClient(YoVariablesUpdatedListener listener, String registryPrefix)
   {
      this(LogSessionDisplay.getAnnounceRequest(), listener, registryPrefix);
   }

   public YoVariableClient(AnnounceRequest request, YoVariablesUpdatedListener listener, String registryPrefix)
   {
      this.logControlClient = new LogControlClient(request.getControlIP(), request.getControlPort(), listener);
      this.handshakeParser = new YoVariableHandshakeParser(registryPrefix, listener.populateRegistry());
      this.yoVariableConsumer = new YoVariableConsumer(request.getDataIP(), request.getDataPort(), handshakeParser.getYoVariablesList(),
            handshakeParser.getJointStates(), listener);
      this.listener = listener;

      listener.setYoVariableClient(this);
   }

   public void start()
   {
      try
      {
         start(-1);
      }
      catch (SocketTimeoutException e)
      {
         throw new RuntimeException(e);
      }
   }

   public synchronized void start(long timeout) throws SocketTimeoutException
   {
      if (state != ClientState.WAITING)
      {
         throw new RuntimeException("Client already started");
      }

      System.out.println("Requesting handshake");
      LogHandshake handshake = yoVariableConsumer.getHandshake();

      logControlClient.connect();
      handshakeParser.parseFrom(handshake.protoShake);

      listener.start(handshake, handshakeParser);
      if (listener.changesVariables())
      {
         List<YoVariable<?>> variablesList = handshakeParser.getYoVariablesList();
         logControlClient.startVariableChangedProducers(variablesList, listener.executeVariableChangedListeners());
      }

      int bufferSize = handshakeParser.getBufferSize();
      yoVariableConsumer.start(bufferSize);

      state = ClientState.RUNNING;
   }

   public void requestStop()
   {
      yoVariableConsumer.stop();
   }
   
   public synchronized void disconnected()
   {
      if (state == ClientState.RUNNING)
      {
         logControlClient.close();
         listener.disconnected();

         state = ClientState.STOPPED;
      }
   }

   public synchronized boolean isRunning()
   {
      return state == ClientState.RUNNING;
   }

   public void sendClearLogRequest()
   {
      logControlClient.sendClearLogRequest();
   }
}
