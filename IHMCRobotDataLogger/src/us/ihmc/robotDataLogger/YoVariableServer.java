package us.ihmc.robotDataLogger;

import java.io.IOException;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commons.Conversions;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.handshake.SummaryProvider;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotDataLogger.rtps.DataProducerParticipant;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.visualizer.RobotVisualizer;
import us.ihmc.robotics.TickAndUpdatable;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicThreadScheduler;


public class YoVariableServer implements RobotVisualizer, TickAndUpdatable, VariableChangedListener
{
   private static final int VARIABLE_BUFFER_CAPACITY = 128;
   private static final int CHANGED_BUFFER_CAPACITY = 128;
   
   private final double dt;
   private final InetAddress bindAddress;
   
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
   private final DataProducerParticipant dataProducerParticipant;
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
      this.scheduler = scheduler;
      this.bindAddress = LogUtils.getMyIP(NetworkParameters.getHost(NetworkParameterKeys.logger));
      
      try
      {
         this.dataProducerParticipant = new DataProducerParticipant(mainClazz);
         dataProducerParticipant.setDataAddress(bindAddress);
         dataProducerParticipant.setPort(getRandomPort());
         dataProducerParticipant.setDataProducerListener(this);
         dataProducerParticipant.setModelFileProvider(logModelProvider);
         dataProducerParticipant.setLog(logSettings.isLog());
         addCameras(logSettings);
         
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
   }
   
   
   public void addCameras(LogSettings logSettings)
   {
      String cameraString;
      if(NetworkParameters.hasKey(NetworkParameterKeys.loggedCameras))
      {
         cameraString = NetworkParameters.getHost(NetworkParameterKeys.loggedCameras);
      }
      else
      {
         cameraString = null;
      }
         
      if(cameraString != null && !cameraString.trim().isEmpty())
      {
         String[] split = cameraString.split(",");
         for(int i = 0; i < split.length; i++)
         {
            try
            {
               byte camera = Byte.parseByte(split[i].trim());
               if(camera >= 0 && camera <= 127)
               {
                  dataProducerParticipant.addCamera(CameraType.CAPTURE_CARD, "Camera-" + camera, String.valueOf(camera));
               }
               else
               {
                  throw new NumberFormatException();
               }
            }
            catch(NumberFormatException e)
            {
               throw new RuntimeException("Invalid camera id: " + split[i] +". Valid camera ids are 0-127");
            }
         }
      }
      
      if(logSettings.getVideoStream() != null)
      {
         dataProducerParticipant.addCamera(CameraType.NETWORK_STREAM, logSettings.getVideoStream(), logSettings.getVideoStream());
      }
   }
   
   public static int getRandomPort()
   {
      return LogDataProtocolSettings.LOG_DATA_PORT_RANGE_START + (new Random().nextInt(1023) + 1);
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

      registerVariablesAndCreateBuffers();
      
      
      
      try
      {
         dataProducerParticipant.setHandshake(handshakeBuilder.getHandShake());
         dataProducerParticipant.activate();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      producer = new YoVariableProducer(scheduler, handshakeBuilder, dataProducerParticipant, mainBuffer,
            buffers.values(), sendKeepAlive);
      producer.start();     
      started = true;
   }

   public void setSendKeepAlive(boolean sendKeepAlive)
   {
      this.sendKeepAlive = sendKeepAlive;
   }
   
   private List<JointHolder> registerVariablesAndCreateBuffers()
   {
      List<JointHolder> jointHolders = handshakeBuilder.getJointHolders();
      
      ArrayList<YoVariable<?>> variables = new ArrayList<>();
      int mainOffset = handshakeBuilder.addRegistry(mainRegistry, variables);
      if(mainYoGraphicsListRegistry != null)
      {
         handshakeBuilder.addDynamicGraphicObjects(mainYoGraphicsListRegistry);
      }
      FullStateBuffer.Builder builder = new FullStateBuffer.Builder(mainOffset, variables, jointHolders);
      mainBuffer = new ConcurrentRingBuffer<FullStateBuffer>(builder, VARIABLE_BUFFER_CAPACITY);
      variableChangeData.put(mainRegistry, new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(), CHANGED_BUFFER_CAPACITY));
      
      for(ImmutablePair<YoVariableRegistry, YoGraphicsListRegistry> data : variableData)
      {
         addVariableBuffer(data);
      }

      return jointHolders;
   }

   public synchronized void close()
   {
      if(started)
      {
         producer.close();
         dataProducerParticipant.remove();         
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
      int variableOffset = handshakeBuilder.addRegistry(registry, variables);
      if(yoGraphicsListRegistry != null)
      {
         handshakeBuilder.addDynamicGraphicObjects(yoGraphicsListRegistry);
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

   @Override
   public void changeVariable(int id, double newValue)
   {
      VariableChangedMessage message;
      ImmutablePair<YoVariable<?>, YoVariableRegistry> variableAndRootRegistry = handshakeBuilder.getVariablesAndRootRegistries().get(id);

      ConcurrentRingBuffer<VariableChangedMessage> buffer = variableChangeData.get(variableAndRootRegistry.getRight());
      while ((message = buffer.next()) == null)
      {
         ThreadTools.sleep(1);
      }

      if (message != null)
      {
         message.setVariable(variableAndRootRegistry.getLeft());
         message.setVal(newValue);
         buffer.commit();
      }

   }


}
