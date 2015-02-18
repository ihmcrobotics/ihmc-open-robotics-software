package us.ihmc.multicastLogDataProtocol.control;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.List;

import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFModelLoader;
import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.robotDataCommunication.YoVariablesUpdatedListener;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class LogControlClient implements NetStateListener 
{
   private final KryoObjectClient client;
   private final YoVariableHandshakeParser handshakeParser;
   
   private boolean receivedHandshake;
   private final Object lock = new Object();
   
   private LogModelLoader modelLoader;
   private final YoVariablesUpdatedListener listener;
   
   public LogControlClient(byte[] host, int port, YoVariablesUpdatedListener listener, String registryPrefix, boolean registerYoVariables)
   {
      this.listener = listener;
      try
      {
         client = new KryoObjectClient(InetAddress.getByAddress(host), port, new LogControlClassList(), LogControlServer.RESOURCE_FOLDER_SIZE, LogControlServer.RESOURCE_FOLDER_SIZE);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
      handshakeParser = new YoVariableHandshakeParser(registryPrefix, registerYoVariables);      
      client.attachStateListener(this);
      client.attachListener(LogHandshake.class, new LogHandShakeConsumer());
      client.attachListener(ClearLogRequest.class, new ClearLogRequestListener());
   }
   
   @Override
   public void connected()
   {
      client.consumeObject(new HandshakeRequest());
   }

   @Override
   public void disconnected()
   {
      // TODO Auto-generated method stub
      
   }
   
   public void close()
   {
      if(client.isConnected())
      {
         client.close();  
      }
   }
   
   public void connect()
   {
      try
      {
         client.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public void waitForHandshake()
   {
      synchronized(lock)
      {
         while(!receivedHandshake)
         {
            try
            {
               lock.wait();
            }
            catch (InterruptedException e)
            {
            }
         }
      }
   }
   
   private class LogHandShakeConsumer implements ObjectConsumer<LogHandshake>
   {
      
      @Override
      public void consumeObject(LogHandshake object)
      {
         synchronized(lock)
         {
            if(!receivedHandshake)
            {
               handshakeParser.parseFrom(object.protoShake);
               listener.receivedHandshake(object);
               if(object.modelLoaderClass != null)
               {
                  modelLoader = new SDFModelLoader();
                  modelLoader.load(object.modelName, object.model, object.resourceDirectories, object.resourceZip);
               }
               
               receivedHandshake = true;
               lock.notifyAll();
            }            
         }
      }
      
   }

   public List<YoVariable<?>> getYoVariablesList()
   {
      return handshakeParser.getYoVariablesList();
   }

   public List<JointState<? extends Joint>> getJointStates()
   {
      return handshakeParser.getJointStates();
   }
   
   
   public void startVariableChangedProducers()
   {
      List<YoVariable<?>> variables = getYoVariablesList();
      for (int i = 0; i < variables.size(); i++)
      {
         LogControlVariableChangeListener variableChangedListener = new LogControlVariableChangeListener(i);
         variables.get(i).addVariableChangedListener(variableChangedListener);
      }
   }
   
   public class LogControlVariableChangeListener implements VariableChangedListener
   {
      private final int id;
      
      public LogControlVariableChangeListener(int id)
      {
         this.id = id;
      }

      public void variableChanged(YoVariable<?> v)
      {         
         VariableChangeRequest request = new VariableChangeRequest();
         request.variableID = id;
         request.requestedValue = v.getValueAsDouble();
         client.consumeObject(request);
      }
   }
   
   public class ClearLogRequestListener implements ObjectConsumer<ClearLogRequest>
   {

      @Override
      public void consumeObject(ClearLogRequest object)
      {
         listener.clearLog();
      }
      
   }
   
   public void sendClearLogRequest()
   {
      client.consumeObject(new ClearLogRequest());
   }

   public int getNumberOfVariables()
   {
      return handshakeParser.getNumberOfVariables();
   }

   public int getNumberOfJointStateVariables()
   {
      return handshakeParser.getNumberOfJointStateVariables();
   }

   public YoVariableRegistry getRootRegistry()
   {
      return handshakeParser.getRootRegistry();
   }

   public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return handshakeParser.getDynamicGraphicObjectsListRegistry();
   }

   public LogModelLoader getModelLoader()
   {
      return modelLoader;
   }
}
