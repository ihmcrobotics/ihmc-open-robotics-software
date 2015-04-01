package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.net.SocketTimeoutException;

import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionDisplay;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient;

public class YoVariableClient
{
   private enum ClientState
   {
      WAITING,
      RUNNING,
      STOPPED
   }
   
   private final YoVariablesUpdatedListener listener;

   private final LogControlClient logControlClient;
   private final YoVariableConsumer yoVariableConsumer;
   private final boolean showOverheadView;
   private ClientState state = ClientState.WAITING;

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
     
      this.logControlClient = new LogControlClient(request.getControlIP(), request.getControlPort(), listener, registryPrefix, listener.populateRegistry());
      this.yoVariableConsumer = new YoVariableConsumer(request.getDataIP(), request.getDataPort(), logControlClient.getYoVariablesList(),
            logControlClient.getJointStates(), listener);
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
      
      logControlClient.connect();
      logControlClient.waitForHandshake();

      int numberOfVariables = logControlClient.getNumberOfVariables();
      int numberOfJointStateVariables = logControlClient.getNumberOfJointStateVariables();
      int bufferSize = (1 + numberOfVariables + numberOfJointStateVariables) * 8;
      
      listener.start(logControlClient.getModelLoader(), logControlClient.getRootRegistry(), logControlClient.getJointStates(), logControlClient.getDynamicGraphicObjectsListRegistry(), bufferSize, showOverheadView);
      
      if (listener.changesVariables())
      {
         logControlClient.startVariableChangedProducers();
      }
      yoVariableConsumer.start(bufferSize);

      state = ClientState.RUNNING;
   }


   public synchronized void close()
   {
      if(state == ClientState.RUNNING)
      {
         yoVariableConsumer.close();
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
