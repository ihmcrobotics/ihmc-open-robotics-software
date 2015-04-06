package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.net.SocketTimeoutException;

import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionDisplay;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFModelLoader;

public class YoVariableClient
{
   private enum ClientState
   {
      WAITING, RUNNING, STOPPED
   }

   private final YoVariablesUpdatedListener listener;

   private final LogControlClient logControlClient;
   private final YoVariableConsumer yoVariableConsumer;
   private final boolean showOverheadView;
   private ClientState state = ClientState.WAITING;

   private final YoVariableHandshakeParser handshakeParser;

   private static AnnounceRequest getRequest()
   {
      try
      {
         AnnounceRequest request = LogSessionDisplay.selectLogSession();
         return request;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public YoVariableClient(YoVariablesUpdatedListener listener, String registryPrefix, boolean showOverheadView)
   {
      this(getRequest(), listener, registryPrefix, showOverheadView);
   }

   public YoVariableClient(AnnounceRequest request, YoVariablesUpdatedListener listener, String registryPrefix, boolean showOverheadView)
   {

      this.logControlClient = new LogControlClient(request.getControlIP(), request.getControlPort(), listener);
      this.handshakeParser = new YoVariableHandshakeParser(registryPrefix, listener.populateRegistry());
      this.yoVariableConsumer = new YoVariableConsumer(request.getDataIP(), request.getDataPort(), handshakeParser.getYoVariablesList(),
            handshakeParser.getJointStates(), listener);
      this.listener = listener;
      this.showOverheadView = showOverheadView;

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

      LogHandshake handshake = yoVariableConsumer.getHandshake();

      handshakeParser.parseFrom(handshake.protoShake);
      listener.receivedHandshake(handshake);
      LogModelLoader modelLoader = null;
      System.out.println(handshake.modelLoaderClass);
      if (handshake.modelLoaderClass != null)
      {
         modelLoader = new SDFModelLoader();
         modelLoader.load(handshake.modelName, handshake.model, handshake.resourceDirectories, handshake.resourceZip);
         System.out.println(modelLoader);
      }

      logControlClient.connect();

      int numberOfVariables = handshakeParser.getNumberOfVariables();
      int numberOfJointStateVariables = handshakeParser.getNumberOfJointStateVariables();
      int bufferSize = (1 + numberOfVariables + numberOfJointStateVariables) * 8;

      listener.start(modelLoader, handshakeParser.getRootRegistry(), handshakeParser.getJointStates(),
            handshakeParser.getDynamicGraphicObjectsListRegistry(), bufferSize, showOverheadView);

      if (listener.changesVariables())
      {
         logControlClient.startVariableChangedProducers(handshakeParser.getYoVariablesList());
      }
      yoVariableConsumer.start(bufferSize);

      state = ClientState.RUNNING;
   }

   public void requestStop()
   {
      yoVariableConsumer.requestStopS();      
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
