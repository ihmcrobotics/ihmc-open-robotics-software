package us.ihmc.robotDataCommunication;

import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionBroadcaster;
import us.ihmc.multicastLogDataProtocol.control.LogControlServer;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataCommunication.jointState.JointHolder;
import us.ihmc.robotDataCommunication.logger.LogSettings;
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
   private final Class<?> mainClazz;
   private final InetAddress bindAddress;   
   private final LogModelProvider logModelProvider;
   private final LogSettings logSettings;
   
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

   
   
   // Servers
   private LogSessionBroadcaster sessionBroadcaster;
   private LogControlServer controlServer;
   private YoVariableProducer producer;
   private YoVariableHandShakeBuilder handshakeBuilder;

   
   
   public YoVariableServer(Class<?> mainClazz, LogModelProvider logModelProvider, LogSettings logSettings, double dt)
   {
      this.dt = dt;
      this.mainClazz = mainClazz;
      this.bindAddress = LogUtils.getMyIP(NetworkParameters.getHost(NetworkParameterKeys.logger));
      this.logModelProvider = logModelProvider;
      this.logSettings = logSettings;
   }

   public synchronized void start()
   {
      if(started)
      {
         throw new RuntimeException("Server already started");
      }
      
      handshakeBuilder = new YoVariableHandShakeBuilder(mainBodies, dt);

      startControlServer();
      
      InetSocketAddress controlAddress = new InetSocketAddress(bindAddress, controlServer.getPort());
      sessionBroadcaster = new LogSessionBroadcaster(controlAddress, bindAddress, mainClazz, logSettings);
      producer = new YoVariableProducer(sessionBroadcaster, handshakeBuilder, logModelProvider, mainBuffer,
            buffers.values());
            
      sessionBroadcaster.requestPort();
      producer.start();
      sessionBroadcaster.start();
      
      started = true;
   }

   private List<JointHolder> startControlServer()
   {
      
      controlServer = new LogControlServer(handshakeBuilder, variableChangeData, dt);
      List<JointHolder> jointHolders = controlServer.getHandshakeBuilder().getJointHolders();
      
      ArrayList<YoVariable<?>> variables = new ArrayList<>();
      int mainOffset = controlServer.getHandshakeBuilder().addRegistry(mainRegistry, variables);
      if(mainDynamicGraphicObjectsListRegistry != null)
      {
         controlServer.getHandshakeBuilder().addDynamicGraphicObjects(mainDynamicGraphicObjectsListRegistry);
      }
      FullStateBuffer.Builder builder = new FullStateBuffer.Builder(mainOffset, variables, jointHolders);
      mainBuffer = new ConcurrentRingBuffer<FullStateBuffer>(builder, VARIABLE_BUFFER_CAPACITY);
      variableChangeData.put(mainRegistry, new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(), CHANGED_BUFFER_CAPACITY));
      
      for(Pair<YoVariableRegistry, YoGraphicsListRegistry> data : variableData)
      {
         addVariableBuffer(data);
      }

      controlServer.start();
      return jointHolders;
   }

   public synchronized void close()
   {
      if(started)
      {
         sessionBroadcaster.close();
         producer.close();
         controlServer.close();         
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
      int variableOffset = controlServer.getHandshakeBuilder().addRegistry(registry, variables);
      if(yoGraphicsListRegistry != null)
      {
         controlServer.getHandshakeBuilder().addDynamicGraphicObjects(yoGraphicsListRegistry);
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
