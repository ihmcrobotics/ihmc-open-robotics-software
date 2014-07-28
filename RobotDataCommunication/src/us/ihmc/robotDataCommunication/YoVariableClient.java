package us.ihmc.robotDataCommunication;

import java.net.SocketTimeoutException;



public class YoVariableClient
{
   private final YoVariablesUpdatedListener listener;

   private final YoVariableHandshakeClient yoVariableHandshakeClient;
   private final YoVariableChangedProducer yoVariableChangedProducer;
   private final YoVariableConsumer yoVariableConsumer;
   private final boolean showOverheadView;
   private boolean started = false;


   public YoVariableClient(String host, int port, YoVariablesUpdatedListener listener, String registryPrefix, boolean showOverheadView)
   {
      int handshakePort = port;
      int consumerPort = port + 1;
      int producerPort = port + 2;

      this.yoVariableHandshakeClient = new YoVariableHandshakeClient(host, handshakePort, listener, registryPrefix, listener.populateRegistry());
      this.yoVariableConsumer = new YoVariableConsumer(host, consumerPort, yoVariableHandshakeClient.getYoVariablesList(),
            yoVariableHandshakeClient.getJointStates(), listener);
      if(listener.changesVariables())
      {
         this.yoVariableChangedProducer = new YoVariableChangedProducer(host, producerPort, yoVariableHandshakeClient.getYoVariablesList());
      }
      else
      {
         this.yoVariableChangedProducer = null;
      }
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
      if (started)
      {
         throw new RuntimeException("Client already started");
      }

      yoVariableHandshakeClient.connect(timeout);
      listener.setRegistry(yoVariableHandshakeClient.getRootRegistry());
      listener.setJointStates(yoVariableHandshakeClient.getJointStates());
      listener.registerDynamicGraphicObjectListsRegistry(yoVariableHandshakeClient.getDynamicGraphicObjectsListRegistry(), showOverheadView);

      if(yoVariableChangedProducer != null)
      {
         yoVariableChangedProducer.start();
      }
      yoVariableConsumer.start(yoVariableHandshakeClient.getNumberOfVariables(), yoVariableHandshakeClient.getNumberOfJointStateVariables());
      listener.start();

      started = true;
   }
   
   public void waitFor()
   {
      try
      {
         yoVariableConsumer.join();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   public void close()
   {
      if(yoVariableChangedProducer != null)
      {
         yoVariableChangedProducer.close();
      }
      yoVariableConsumer.close();
      listener.disconnected();
   }
}
