package us.ihmc.robotDataCommunication;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotDataCommunication.jointState.JointHolder;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;


public class YoVariableServer implements RobotVisualizer
{
   private static final int VARIABLE_BUFFER_CAPACITY = 128;
   private static final int CHANGED_BUFFER_CAPACITY = 128;
   
   private final double dt;
   
   // Data to send
   private List<RigidBody> mainBodies = new ArrayList<>();
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry mainDynamicGraphicObjectsListRegistry;
   private final ArrayList<Pair<YoVariableRegistry, YoGraphicsListRegistry>> variableData = new ArrayList<>();
   
   // Variable data
   private ConcurrentRingBuffer<FullStateBuffer> mainBuffer;
   private final LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<RegistryBuffer>> buffers = new LinkedHashMap<>(); 
   
   // Change data
   private final LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData = new LinkedHashMap<>();
      
   // State
   private boolean started = false;

   // ZMQ
   private final int handshakePort;
   private final int producerPort;
   private final int consumerPort;
   
   // Servers
   private YoVariableHandshakeServer handshakeServer;
   private YoVariableProducer producer;
   private YoVariableChangedConsumer consumer;
   
   public YoVariableServer(int port, double dt)
   {
      this.handshakePort = port;
      this.producerPort = port + 1;
      this.consumerPort = port + 2;
      
      this.dt = dt;
   }

   public synchronized void start()
   {
      if(started)
      {
         throw new RuntimeException("Server already started");
      }
      
      handshakeServer = new YoVariableHandshakeServer(handshakePort, mainBodies, dt);
      List<JointHolder> jointHolders = handshakeServer.getJointHolders();
      
      ArrayList<YoVariable<?>> variables = new ArrayList<>();
      int mainOffset = handshakeServer.addRegistry(mainRegistry, variables);
      if(mainDynamicGraphicObjectsListRegistry != null)
      {
         handshakeServer.addDynamicGraphicObjects(mainDynamicGraphicObjectsListRegistry);
      }
      FullStateBuffer.Builder builder = new FullStateBuffer.Builder(mainOffset, variables, jointHolders);
      mainBuffer = new ConcurrentRingBuffer<FullStateBuffer>(builder, VARIABLE_BUFFER_CAPACITY);
      variableChangeData.put(mainRegistry, new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(), CHANGED_BUFFER_CAPACITY));
      
      for(Pair<YoVariableRegistry, YoGraphicsListRegistry> data : variableData)
      {
         addVariableBuffer(data);
      }
            
      producer = new YoVariableProducer(producerPort, handshakeServer.getNumberOfVariables(), FullStateBuffer.getNumberOfJointStates(jointHolders), mainBuffer,
            buffers.values());
      consumer = new YoVariableChangedConsumer(consumerPort, handshakeServer.getVariablesAndRootRegistries(), variableChangeData);
      
      
      handshakeServer.start();
      producer.start();
      consumer.start();
      
      started = true;
   }

   public synchronized void close()
   {
      if(started)
      {
         handshakeServer.close();
         producer.close();
         consumer.close();         
      }
   }

   /**
    * Update main buffer data.
    * 
    * @param timestamp timestamp to send to logger
    */
   public void update(long timestamp)
   {
      update(timestamp, mainRegistry);
   }
   
   public void update(long timestamp, YoVariableRegistry registry)
   {
      if(!started)
      {
         return;
      }
      ConcurrentRingBuffer<? extends RegistryBuffer> ringBuffer;
      if(registry.equals(mainRegistry))
      {
         ringBuffer = mainBuffer;
      }
      else
      {
         ringBuffer = buffers.get(registry);
         if(ringBuffer == null)
         {
            throw new RuntimeException("Cannot find root registry " + registry.getName());
         }
      }
      updateVariableBuffer(timestamp, ringBuffer);
      updateChangedVariables(registry);
      
      
   }
   
   private void addVariableBuffer(Pair<YoVariableRegistry, YoGraphicsListRegistry> data)
   {
      ArrayList<YoVariable<?>> variables = new ArrayList<>();
      YoVariableRegistry registry = data.first();
      YoGraphicsListRegistry yoGraphicsListRegistry = data.second();
      int variableOffset = handshakeServer.addRegistry(registry, variables);
      if(yoGraphicsListRegistry != null)
      {
         handshakeServer.addDynamicGraphicObjects(yoGraphicsListRegistry);
      }
      RegistryBuffer.Builder builder = new RegistryBuffer.Builder(variableOffset, variables);
      ConcurrentRingBuffer<RegistryBuffer> buffer = new ConcurrentRingBuffer<>(builder, VARIABLE_BUFFER_CAPACITY);
      
      buffers.put(registry, buffer);
      variableChangeData.put(registry, new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(), CHANGED_BUFFER_CAPACITY));
   }

   private void updateChangedVariables(YoVariableRegistry rootRegistry)
   {
      ConcurrentRingBuffer<VariableChangedMessage> buffer = variableChangeData.get(rootRegistry);
      buffer.poll();
      VariableChangedMessage msg;
      while((msg = buffer.read()) != null)
      {
         msg.getVariable().setValueFromDouble(msg.getVal());            
      }
      buffer.flush();
   }

   private void updateVariableBuffer(long timestamp, ConcurrentRingBuffer<? extends RegistryBuffer> ringBuffer)
   {
      RegistryBuffer buffer = ringBuffer.next();
      if(buffer != null)
      {
         buffer.update(timestamp);
         ringBuffer.commit();
      }
   }
   
   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Pair<YoVariableRegistry, YoGraphicsListRegistry> data = new Pair<YoVariableRegistry, YoGraphicsListRegistry>(registry, yoGraphicsListRegistry);
      variableData.add(data);
   }

   @Override
   public void setMainRegistry(YoVariableRegistry registry, FullRobotModel fullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if(fullRobotModel != null)
      {
         mainBodies.add(fullRobotModel.getElevator());
      }
      mainRegistry = registry;
      mainDynamicGraphicObjectsListRegistry = yoGraphicsListRegistry;
   }
}
