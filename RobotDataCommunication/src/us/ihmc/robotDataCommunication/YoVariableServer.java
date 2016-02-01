package us.ihmc.robotDataCommunication;

import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.visualizer.RobotVisualizer;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionBroadcaster;
import us.ihmc.multicastLogDataProtocol.control.LogControlServer;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataCommunication.jointState.JointHolder;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.util.PeriodicThreadScheduler;


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
   private final ArrayList<ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry>> variableData = new ArrayList<>();
   
   // Variable data
   private ConcurrentRingBuffer<FullStateBuffer> mainBuffer;
   private final LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<RegistryBuffer>> buffers = new LinkedHashMap<>(); 
   
   // Change data
   private final LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData = new LinkedHashMap<>();
      
   // State
   private boolean started = false;

   private final PeriodicThreadScheduler scheduler;
   
   // Servers
   private LogSessionBroadcaster sessionBroadcaster;
   private LogControlServer controlServer;
   private YoVariableProducer producer;
   private YoVariableHandShakeBuilder handshakeBuilder;

   
   private long uid = 0; 
   
   
   public YoVariableServer(Class<?> mainClazz, PeriodicThreadScheduler scheduler, LogModelProvider logModelProvider, LogSettings logSettings, double dt)
   {
      this.dt = dt;
      this.mainClazz = mainClazz;
      this.scheduler = scheduler;
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
      producer = new YoVariableProducer(scheduler, sessionBroadcaster, handshakeBuilder, logModelProvider, mainBuffer,
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
      
      for(ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry> data : variableData)
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
         updateMainVariableBuffer(timestamp);
      }
      else
      {
         ringBuffer = buffers.get(registry);
         if(ringBuffer == null)
         {
            throw new RuntimeException("Cannot find root registry " + registry.getName());
         }
         updateVariableBuffer(timestamp, ringBuffer);
      }
      updateChangedVariables(registry);
      
      
   }
   
   private void addVariableBuffer(ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry> data)
   {
      ArrayList<YoVariable<?>> variables = new ArrayList<>();
      YoVariableRegistry registry = data.getLeft();
      YoGraphicsListRegistry yoGraphicsListRegistry = data.getRight();
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
   private void updateMainVariableBuffer(long timestamp)
   {
      FullStateBuffer buffer = mainBuffer.next();
      if(buffer != null)
      {
         buffer.update(timestamp, uid);
         mainBuffer.commit();
      }
      uid++;
   }
   
   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry> data = new ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry>(registry, yoGraphicsListRegistry);
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
