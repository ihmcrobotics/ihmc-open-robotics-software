package us.ihmc.multicastLogDataProtocol.control;

import java.io.IOException;
import java.net.BindException;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataCommunication.VariableChangedMessage;
import us.ihmc.robotDataCommunication.YoVariableHandShakeBuilder;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class LogControlServer
{
   protected static int RESOURCE_FOLDER_SIZE = 3 * 72000000;

   private int port = LogDataProtocolSettings.LOG_DATA_PORT_RANGE_START;
   private KryoObjectServer server;

   private final LogModelProvider logModelProvider;

   private final YoVariableHandShakeBuilder handshakeBuilder;
   private final LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData;

   private final LogHandshake handshake = new LogHandshake();

   public LogControlServer(LogModelProvider modelProvider, List<RigidBody> rootBodies,
         LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData, double dt)
   {
      this.logModelProvider = modelProvider;
      handshakeBuilder = new YoVariableHandShakeBuilder(rootBodies, dt);
      this.variableChangeData = variableChangeData;
   }

   public YoVariableHandShakeBuilder getHandshakeBuilder()
   {
      return handshakeBuilder;
   }

   public void start()
   {
      try
      {
         boolean connected = false;
         do
         {
            server = new KryoObjectServer(port, new LogControlClassList(), RESOURCE_FOLDER_SIZE, RESOURCE_FOLDER_SIZE);
            try
            {
               server.connect();
               connected = true;
            }
            catch (BindException e)
            {
               server.close();
               port++;
            }
         }
         while (!connected);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      handshake.protoShake = handshakeBuilder.toByteArray();

      if (logModelProvider != null)
      {
         handshake.modelLoaderClass = logModelProvider.getLoader().getCanonicalName();
         handshake.modelName = logModelProvider.getModelName();
         handshake.model = logModelProvider.getModel();
         handshake.resourceDirectories = logModelProvider.getResourceDirectories();
         handshake.resourceZip = logModelProvider.getResourceZip();
         if(handshake.resourceZip.length > RESOURCE_FOLDER_SIZE)
         {
            throw new RuntimeException("Resource folder takes more than " + RESOURCE_FOLDER_SIZE + ". Delete duplicate/unused textures and models from the resource folder. Increasing the maximum size will slow down visualizer startup.");
         }
      }
      server.attachListener(HandshakeRequest.class, new HandshakeRequestListener());
      server.attachListener(VariableChangeRequest.class, new VariableChangeRequestListener());
      server.attachListener(ClearLogRequest.class, new ClearLogRequestListener());
   }

   public class HandshakeRequestListener implements ObjectConsumer<HandshakeRequest>
   {
      @Override
      public void consumeObject(HandshakeRequest object)
      {
         server.consumeObject(handshake);
      }
   }

   public class VariableChangeRequestListener implements ObjectConsumer<VariableChangeRequest>
   {

      @Override
      public void consumeObject(VariableChangeRequest object)
      {
         VariableChangedMessage message;
         Pair<YoVariable<?>, YoVariableRegistry> variableAndRootRegistry = handshakeBuilder.getVariablesAndRootRegistries().get(object.variableID);

         ConcurrentRingBuffer<VariableChangedMessage> buffer = variableChangeData.get(variableAndRootRegistry.second());
         while ((message = buffer.next()) == null)
         {
            ThreadTools.sleep(1);
         }

         if (message != null)
         {
            message.setVariable(variableAndRootRegistry.first());
            message.setVal(object.requestedValue);
            buffer.commit();
         }
      }

   }

   public class ClearLogRequestListener implements ObjectConsumer<ClearLogRequest>
   {

      @Override
      public void consumeObject(ClearLogRequest object)
      {
         System.out.println("Broadcasting clear log");
         server.consumeObject(object);
      }
      
   }

   
   public int getPort()
   {
      return port;
   }

   public void close()
   {
      server.close();
   }

}
