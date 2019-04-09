package us.ihmc.robotDataLogger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import gnu.trove.list.array.TByteArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.dataBuffers.CustomLogDataPublisherType;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.SummaryProvider;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.interfaces.DataProducer;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.server.WebsocketDataProducer;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableServer implements RobotVisualizer, VariableChangedListener
{
   private static final int CHANGED_BUFFER_CAPACITY = 128;

   private final double dt;

   private String rootRegistryName = "main";
   
   private YoVariableRegistry mainRegistry = null;
   private final ArrayList<RegistrySendBufferBuilder> registeredBuffers = new ArrayList<>();
   private final HashMap<YoVariableRegistry, RegistryPublisher> publishers = new HashMap<>();

   // Change data
   private final HashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData = new HashMap<>();

   // State
   private boolean started = false;
   private boolean stopped = false;

   // Servers
   private final DataProducer dataProducer;
   private YoVariableHandShakeBuilder handshakeBuilder;

   private volatile long latestTimestamp;
   
   private final SummaryProvider summaryProvider = new SummaryProvider();

   private final LogWatcher logWatcher = new LogWatcher();

   @Deprecated
   /**
    * A thread scheduler is not necessary anymore. This function is left in for backwards compatibility.
    * 
    * @param mainClazz
    * @param schedulerFactory
    * @param logModelProvider
    * @param dataServerSettings
    * @param dt
    */
   public YoVariableServer(Class<?> mainClazz, PeriodicThreadSchedulerFactory schedulerFactory, LogModelProvider logModelProvider, DataServerSettings dataServerSettings, double dt)
   {
      this(mainClazz, logModelProvider, dataServerSettings, dt);
   }
   
   
   @Deprecated
   /**
    * A thread scheduler is not necessary anymore. This function is left in for backwards compatibility.
    * 
    * @param mainClazz
    * @param schedulerFactory
    * @param logModelProvider
    * @param dataServerSettings
    * @param dt
    */
   public YoVariableServer(String mainClazz, PeriodicThreadSchedulerFactory schedulerFactory, LogModelProvider logModelProvider, DataServerSettings dataServerSettings, double dt)
   {
      this(mainClazz, logModelProvider, dataServerSettings, dt);
   }
   
   /**
    * Create a YoVariable server with mainClazz.getSimpleName(). For example, see other constructor.
    *
    * @param mainClazz
    * @param schedulerFactory
    * @param logModelProvider
    * @param dataServerSettings
    * @param dt
    */
   public YoVariableServer(Class<?> mainClazz, LogModelProvider logModelProvider, DataServerSettings dataServerSettings, double dt)
   {
      this(mainClazz.getSimpleName(), logModelProvider, dataServerSettings, dt);
   }

   /**
    * To create a YoVariableServer:
    *
    * <ol>
    * <li>Create a YoVariableRegistry</li>
    * <li>Add YoVariables</li>
    * <li>Create YoVariableServer</li>
    * <li>Set the YoVariableServer's main registry to the one you made</li>
    * <li>Call YoVariableServer.start()</li>
    * <li>Schedule a thread that calls YoVariableServer.update() periodically</li>
    * </ol>
    *
    * Pseuo-code for starting a YoVariableServer:
    *
    * <pre>
    * {@code
    * YoVariableRegistry registry = new YoVariableRegistry("hello"); // cannot be "root", reserved word
    * YoDouble doubleYo = new YoDouble("x", registry);
    *
    * PeriodicNonRealtimeThreadSchedulerFactory schedulerFactory = new PeriodicNonRealtimeThreadSchedulerFactory();
    * YoVariableServer yoVariableServer = new YoVariableServer("HelloYoServer", schedulerFactory,
    *                                                          null, new LogSettings(false), 0.01);
    * yoVariableServer.setMainRegistry(registry, null, null);
    * yoVariableServer.start();
    *
    * PeriodicThreadScheduler updateScheduler = schedulerFactory.createPeriodicThreadScheduler("update");
    *
    * AtomicLong timestamp = new AtomicLong();   // must schedule updates yourself or the server will timeout
    * updateScheduler.schedule(() -> {
    *    yoVariableServer.update(timestamp.getAndAdd(10000));
    * }, 10, TimeUnit.MILLISECONDS);
    * }
    * </pre>
    *
    * @param mainClazz
    * @param schedulerFactory
    * @param logModelProvider
    * @param dataServerSettings
    * @param dt
    */
   public YoVariableServer(String mainClazz, LogModelProvider logModelProvider, DataServerSettings dataServerSettings, double dt)
   {
      LoggerConfigurationLoader config;
      try
      {
         config = new LoggerConfigurationLoader();
      }
      catch (IOException e1)
      {
         throw new RuntimeException("Cannot load configuration to start logger, aborting", e1);
      }

      this.dt = dt;

      this.dataProducer = new WebsocketDataProducer(mainClazz, logModelProvider, this, logWatcher, dataServerSettings);
      addCameras(config, dataServerSettings);
      
   }
   
   public void setRootRegistryName(String name)
   {
      this.rootRegistryName = name;
   }

   private void addCameras(LoggerConfigurationLoader config, DataServerSettings logSettings)
   {
      TByteArrayList cameras = config.getCameras();
      for (int i = 0; i < cameras.size(); i++)
      {
         dataProducer.addCamera(CameraType.CAPTURE_CARD, "Camera-" + cameras.get(i), String.valueOf(cameras.get(i)));
      }

      if (logSettings.getVideoStream() != null)
      {
         dataProducer.addCamera(CameraType.NETWORK_STREAM, logSettings.getVideoStream(), logSettings.getVideoStream());
      }
   }

   public synchronized void start()
   {
      if (started)
      {
         throw new RuntimeException("Server already started");
      }

      handshakeBuilder = new YoVariableHandShakeBuilder(rootRegistryName, dt);
      handshakeBuilder.setFrames(ReferenceFrame.getWorldFrame());
      int maxVariables = 0;
      int maxStates = 0;
      for (int i = 0; i < registeredBuffers.size(); i++)
      {
         RegistrySendBufferBuilder builder = registeredBuffers.get(i);
         YoVariableRegistry registry = builder.getYoVariableRegistry();
         handshakeBuilder.addRegistryBuffer(builder);

         variableChangeData.put(registry, new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(), CHANGED_BUFFER_CAPACITY));
            
         if(builder.getNumberOfVariables() > maxVariables)
         {
            maxVariables = builder.getNumberOfVariables();
         }
         if(builder.getNumberOfJointStates() > maxStates)
         {
            maxStates = builder.getNumberOfJointStates();
         }
         
      }
      
      
      CustomLogDataPublisherType type = new CustomLogDataPublisherType(maxVariables, maxStates);

      for (int i = 0; i < registeredBuffers.size(); i++)
      {
         RegistrySendBufferBuilder builder = registeredBuffers.get(i);
         YoVariableRegistry registry = builder.getYoVariableRegistry();
         
         try
         {
            publishers.put(registry, dataProducer.createRegistryPublisher(type, builder));
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }

      handshakeBuilder.setSummaryProvider(summaryProvider);

      try
      {
         for (int i = 0; i < registeredBuffers.size(); i++)
         {
            RegistrySendBufferBuilder builder = registeredBuffers.get(i);
            YoVariableRegistry registry = builder.getYoVariableRegistry();
            publishers.get(registry).start();
         }

         dataProducer.setHandshake(handshakeBuilder.getHandShake());
         dataProducer.announce();
         
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      started = true;
   }

   @Override
   public synchronized void close()
   {
      if (started && !stopped)
      {
         stopped = false;
         for (int i = 0; i < registeredBuffers.size(); i++)
         {
            RegistrySendBufferBuilder builder = registeredBuffers.get(i);
            YoVariableRegistry registry = builder.getYoVariableRegistry();
            publishers.get(registry).stop();
         }
         dataProducer.remove();
         
      }
   }


   /**
    * Update main buffer data.
    * 
    * Note: If the timestamp is not increasing between updates(), no data might be send to clients.
    * 
    * @param timestamp timestamp to send to logger
    */
   @Override
   public void update(long timestamp)
   {
      update(timestamp, mainRegistry);
   }

   /**
    * Update registry data
    * 
    * Note: If the timestamp is not increasing between updates(), no data might be send to clients.
    * 
    * @param timestamp timestamp to send to the logger
    * @param registry Top level registry to update
    */
   @Override
   public void update(long timestamp, YoVariableRegistry registry)
   {
      if (!started && !stopped)
      {
         return;
      }
      if (registry == mainRegistry)
      {
         dataProducer.publishTimestamp(timestamp);
         latestTimestamp = timestamp;
      }

      RegistryPublisher publisher = publishers.get(registry);
      publisher.update(timestamp);
      updateChangedVariables(registry);

      logWatcher.update(timestamp);
   }

   private void updateChangedVariables(YoVariableRegistry rootRegistry)
   {
      ConcurrentRingBuffer<VariableChangedMessage> buffer = variableChangeData.get(rootRegistry);
      buffer.poll();
      VariableChangedMessage msg;
      while ((msg = buffer.read()) != null)
      {
         msg.getVariable().setValueFromDouble(msg.getVal());
      }
      buffer.flush();
   }

   @Override
   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registeredBuffers.add(new RegistrySendBufferBuilder(registry, null, yoGraphicsListRegistry));
   }

   @Override
   public void setMainRegistry(YoVariableRegistry registry, RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (this.mainRegistry != null)
      {
         throw new RuntimeException("Main registry is already set");
      }
      registeredBuffers.add(new RegistrySendBufferBuilder(registry, rootBody, yoGraphicsListRegistry));
      this.mainRegistry = registry;
   }

   private YoVariable<?> findVariableInRegistries(String variableName)
   {

      for (RegistrySendBufferBuilder buffer : registeredBuffers)
      {
         YoVariableRegistry registry = buffer.getYoVariableRegistry();
         YoVariable<?> ret = registry.getVariable(variableName);
         if (ret != null)
         {
            return ret;
         }
      }
      return null;
   }

   public void createSummary(YoVariable<?> isWalkingVariable)
   {
      createSummary(isWalkingVariable.getFullNameWithNameSpace());
   }

   public void createSummary(String summaryTriggerVariable)
   {
      if (findVariableInRegistries(summaryTriggerVariable) == null)
      {
         throw new RuntimeException("Variable " + summaryTriggerVariable + " is not registered with the logger");
      }
      this.summaryProvider.setSummarize(true);
      this.summaryProvider.setSummaryTriggerVariable(summaryTriggerVariable);
   }

   public void addSummarizedVariable(String variable)
   {
      if (findVariableInRegistries(variable) == null)
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

   @Override
   public long getLatestTimestamp()
   {
      return latestTimestamp;
   }

   public boolean isLogging()
   {
      return logWatcher.isLogging();
   }
}
