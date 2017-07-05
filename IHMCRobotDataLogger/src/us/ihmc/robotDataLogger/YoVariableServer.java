package us.ihmc.robotDataLogger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import gnu.trove.list.array.TByteArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.SummaryProvider;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotDataLogger.rtps.DataProducerParticipant;
import us.ihmc.robotDataLogger.rtps.RegistryPublisher;
import us.ihmc.robotics.TickAndUpdatable;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableServer implements RobotVisualizer, TickAndUpdatable, VariableChangedListener
{
   private static final int CHANGED_BUFFER_CAPACITY = 128;

   private final double dt;

   private YoVariableRegistry mainRegistry = null;
   private final ArrayList<RegistrySendBufferBuilder> registeredBuffers = new ArrayList<>();
   private final HashMap<YoVariableRegistry, RegistryPublisher> publishers = new HashMap<>();

   // Change data
   private final HashMap<YoVariableRegistry, ConcurrentRingBuffer<VariableChangedMessage>> variableChangeData = new HashMap<>();

   // State
   private boolean started = false;

   private final PeriodicThreadScheduler scheduler;

   // Servers
   private final DataProducerParticipant dataProducerParticipant;
   private YoVariableHandShakeBuilder handshakeBuilder;

   private boolean sendKeepAlive = false;

   private volatile long latestTimestamp;
   
   private final SummaryProvider summaryProvider = new SummaryProvider();

   public YoVariableServer(Class<?> mainClazz, PeriodicThreadScheduler scheduler, LogModelProvider logModelProvider, LogSettings logSettings, double dt)
   {
      this(mainClazz.getSimpleName(), scheduler, logModelProvider, logSettings, dt);
   }

   public YoVariableServer(String mainClazz, PeriodicThreadScheduler scheduler, LogModelProvider logModelProvider, LogSettings logSettings, double dt)
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
      this.scheduler = scheduler;

      try
      {
         this.dataProducerParticipant = new DataProducerParticipant(mainClazz, logModelProvider, this);
         dataProducerParticipant.setLog(logSettings.isLog());
         addCameras(config, logSettings);

      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

   }

   private void addCameras(LoggerConfigurationLoader config, LogSettings logSettings)
   {
      TByteArrayList cameras = config.getCameras();
      for (int i = 0; i < cameras.size(); i++)
      {
         dataProducerParticipant.addCamera(CameraType.CAPTURE_CARD, "Camera-" + cameras.get(i), String.valueOf(cameras.get(i)));
      }

      if (logSettings.getVideoStream() != null)
      {
         dataProducerParticipant.addCamera(CameraType.NETWORK_STREAM, logSettings.getVideoStream(), logSettings.getVideoStream());
      }
   }

   public synchronized void start()
   {
      if (started)
      {
         throw new RuntimeException("Server already started");
      }

      handshakeBuilder = new YoVariableHandShakeBuilder(dt);
      for (int i = 0; i < registeredBuffers.size(); i++)
      {
         RegistrySendBufferBuilder builder = registeredBuffers.get(i);
         YoVariableRegistry registry = builder.getYoVariableRegistry();
         handshakeBuilder.addRegistryBuffer(builder);

         variableChangeData.put(registry, new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(), CHANGED_BUFFER_CAPACITY));

         try
         {
            publishers.put(registry, dataProducerParticipant.createRegistryPublisher(scheduler, builder));
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

         dataProducerParticipant.setHandshake(handshakeBuilder.getHandShake());
         dataProducerParticipant.announce();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      started = true;
   }

   public void setSendKeepAlive(boolean sendKeepAlive)
   {
      this.sendKeepAlive = sendKeepAlive;
   }

   public synchronized void close()
   {
      if (started)
      {
         for (int i = 0; i < registeredBuffers.size(); i++)
         {
            RegistrySendBufferBuilder builder = registeredBuffers.get(i);
            YoVariableRegistry registry = builder.getYoVariableRegistry();
            publishers.get(registry).stop();
         }
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

   /**
    * Update registry data
    * 
    * @param timestamp timestamp to send to the logger
    * @param registry Top level registry to update
    */
   public void update(long timestamp, YoVariableRegistry registry)
   {
      if (!started)
      {
         return;
      }
      if (registry == mainRegistry)
      {
         try
         {
            dataProducerParticipant.publishTimestamp(timestamp);
            latestTimestamp = timestamp;
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      RegistryPublisher publisher = publishers.get(registry);
      publisher.update(timestamp);
      updateChangedVariables(registry);

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

   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registeredBuffers.add(new RegistrySendBufferBuilder(registry, null, yoGraphicsListRegistry));
   }

   @Override
   public void setMainRegistry(YoVariableRegistry registry, RigidBody rootBody, YoGraphicsListRegistry yoGraphicsListRegistry)
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

}
