package us.ihmc.robotDataLogger;

import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commons.Conversions;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionBroadcaster;
import us.ihmc.multicastLogDataProtocol.control.LogControlServer;
import us.ihmc.multicastLogDataProtocol.control.SummaryProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.visualizer.RobotVisualizer;
import us.ihmc.robotics.TickAndUpdatable;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.util.PeriodicThreadScheduler;


public class YoVariableServer implements RobotVisualizer, TickAndUpdatable
{
   private static final int VARIABLE_BUFFER_CAPACITY = 128;
   private static final int CHANGED_BUFFER_CAPACITY = 128;
   
   private final double dt;
   private final String mainClazz;
   private final InetAddress bindAddress;   
   private final LogModelProvider logModelProvider;
   private final LogSettings logSettings;
   
   // Data to send
   private List<RigidBody> mainBodies = new ArrayList<>();
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry mainYoGraphicsListRegistry;
   private final ArrayList<ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry>> variableData = new ArrayList<>();
   
   // Variable data
   private ConcurrentRingBuffer<FullStateBuffer> mainBuffer;
   private final LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<RegistryBuffer>> buffers = new LinkedHashMap<>(); 
   
   // Change data
   private final LinkedHashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData = new LinkedHashMap<>();
      
   private IntegerYoVariable skippedMainRegistryTicksDueFullBuffer;
   private HashMap<YoVariableRegistry, IntegerYoVariable> skippedRegistryTicksDueFullBuffer = new HashMap<>();
   
   // State
   private boolean started = false;

   private final PeriodicThreadScheduler scheduler;
   
   // Servers
   private LogSessionBroadcaster sessionBroadcaster;
   private LogControlServer controlServer;
   private YoVariableProducer producer;
   private YoVariableHandShakeBuilder handshakeBuilder;

   
   private long uid = 0; 
   
   private boolean sendKeepAlive = false;
   
   private final SummaryProvider summaryProvider = new SummaryProvider();
   
   public YoVariableServer(Class<?> mainClazz, PeriodicThreadScheduler scheduler, LogModelProvider logModelProvider, LogSettings logSettings, double dt)
   {
      this(mainClazz.getSimpleName(), scheduler, logModelProvider, logSettings, dt);
   }
   
   public YoVariableServer(String mainClazz, PeriodicThreadScheduler scheduler, LogModelProvider logModelProvider, LogSettings logSettings, double dt)
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
      
      skippedMainRegistryTicksDueFullBuffer = new IntegerYoVariable("skippedMainRegistryTicksDueFullBuffer", mainRegistry);
      for(ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry> registry : variableData)
      {
         skippedRegistryTicksDueFullBuffer.put(registry.getLeft(), new IntegerYoVariable("skipped" + registry.getLeft().getName() +"RegistryTicksDueFullBuffer", mainRegistry));
      }
      
      handshakeBuilder = new YoVariableHandShakeBuilder(mainBodies, dt);

      startControlServer();
      
      InetSocketAddress controlAddress = new InetSocketAddress(bindAddress, controlServer.getPort());
      sessionBroadcaster = new LogSessionBroadcaster(controlAddress, bindAddress, mainClazz, logSettings);
      producer = new YoVariableProducer(scheduler, sessionBroadcaster, handshakeBuilder, logModelProvider, mainBuffer,
            buffers.values(), summaryProvider, sendKeepAlive);
            
      sessionBroadcaster.requestPort();
      producer.start();
      sessionBroadcaster.start();
      
      started = true;
   }

   public void setSendKeepAlive(boolean sendKeepAlive)
   {
      this.sendKeepAlive = sendKeepAlive;
   }
   
   private List<JointHolder> startControlServer()
   {
      
      controlServer = new LogControlServer(handshakeBuilder, variableChangeData, dt);
      List<JointHolder> jointHolders = controlServer.getHandshakeBuilder().getJointHolders();
      
      ArrayList<YoVariable<?>> variables = new ArrayList<>();
      int mainOffset = controlServer.getHandshakeBuilder().addRegistry(mainRegistry, variables);
      if(mainYoGraphicsListRegistry != null)
      {
         controlServer.getHandshakeBuilder().addDynamicGraphicObjects(mainYoGraphicsListRegistry);
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

   @Override
   public void tickAndUpdate()
   {
      this.tickAndUpdate(0.0);
   }

   @Override
   public void tickAndUpdate(double timeToSetInSeconds)
   {
      this.update(Conversions.secondsToNanoseconds(timeToSetInSeconds));   
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
         updateVariableBuffer(timestamp, ringBuffer, registry);
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

   private void updateVariableBuffer(long timestamp, ConcurrentRingBuffer<? extends RegistryBuffer> ringBuffer, YoVariableRegistry registry)
   {
      RegistryBuffer buffer = ringBuffer.next();
      if(buffer != null)
      {
         buffer.update(timestamp);
         ringBuffer.commit();
      }
      else
      {
         skippedRegistryTicksDueFullBuffer.get(registry).increment();
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
      else
      {
         skippedMainRegistryTicksDueFullBuffer.increment();
      }
      producer.publishTimestampRealtime(timestamp);
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
      mainYoGraphicsListRegistry = yoGraphicsListRegistry;
      
   }
   
   private YoVariable<?> findVariableInRegistries(String variableName)
   {
      YoVariable<?> ret = mainRegistry.getVariable(variableName);
      if(ret == null)
      {
         for(ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry> v : variableData)
         {
            ret = v.getLeft().getVariable(variableName);
            if(ret != null)
            {
               return ret;
            }
         }
      }
      return ret;
   }
   

   public void createSummary(YoVariable<?> isWalkingVariable)
   {
      createSummary(isWalkingVariable.getFullNameWithNameSpace());
   }
   
   public void createSummary(String summaryTriggerVariable)
   {
      if(findVariableInRegistries(summaryTriggerVariable) == null)
      {
         throw new RuntimeException("Variable " + summaryTriggerVariable + " is not registered with the logger");
      }
      this.summaryProvider.setSummarize(true);
      this.summaryProvider.setSummaryTriggerVariable(summaryTriggerVariable);
   }
   
   public void addSummarizedVariable(String variable)
   {
      if(findVariableInRegistries(variable) == null)
      {
         throw new RuntimeException("Variable " + variable + " is not registered with the logger");
      }
      this.summaryProvider.addSummarizedVariable(variable);
   }
   
   public void addSummarizedVariable(YoVariable<?> variable)
   {      
      this.summaryProvider.addSummarizedVariable(variable);
   }


}
