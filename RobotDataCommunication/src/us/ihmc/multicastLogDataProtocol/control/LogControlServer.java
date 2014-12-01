package us.ihmc.multicastLogDataProtocol.control;

import java.io.IOException;
import java.net.BindException;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataCommunication.VariableChangedMessage;
import us.ihmc.robotDataCommunication.YoVariableHandShakeBuilder;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class LogControlServer
{
   private int port = LogDataProtocolSettings.LOG_DATA_PORT;
   private KryoObjectServer server;

   private final YoVariableHandShakeBuilder handshakeBuilder;
   private final LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData;

   private final LogHandshake handshake = new LogHandshake();

   public LogControlServer(LogModelProvider modelProvider, List<RigidBody> rootBodies,
         LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData, double dt)
   {
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
            server = new KryoObjectServer(port, new LogControlClassList());
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

      server.attachListener(HandshakeRequest.class, new HandshakeRequestListener());
      server.attachListener(VariableChangeRequest.class, new VariableChangeRequestListener());
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
         while((message = buffer.next()) == null)
         {
            ThreadTools.sleep(1);
         }
         
         if(message != null)
         {
            message.setVariable(variableAndRootRegistry.first());
            message.setVal(object.requestedValue);
            buffer.commit();                  
         }
      }

   }

}
