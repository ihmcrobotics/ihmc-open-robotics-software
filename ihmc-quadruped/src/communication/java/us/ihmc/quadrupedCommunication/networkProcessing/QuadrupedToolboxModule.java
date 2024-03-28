package us.ihmc.quadrupedCommunication.networkProcessing;

import com.google.common.base.CaseFormat;
import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.QuadrupedAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.CommandInputManager.HasReceivedInputListener;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * This is a base class for any toolbox in the network manager. See the KinematicsToolboxModule as
 * an example.
 */
public abstract class QuadrupedToolboxModule
{
   protected static final boolean DEBUG = false;
   protected static final double YO_VARIABLE_SERVER_DT = 0.01;
   protected static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 1;

   protected final String name = getClass().getSimpleName();
   protected final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   protected final YoRegistry registry = new YoRegistry(name);
   protected final YoDouble yoTime = new YoDouble("localTime", registry);
   protected final String robotName;
   protected final FullQuadrupedRobotModel fullRobotModel;

   protected final RealtimeROS2Node realtimeROS2Node;
   protected final CommandInputManager inputManager;
   protected final OutputManager outputManager;
   protected final NetworkSubscriber networkSubscriber;

   protected final ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);
   protected final ScheduledExecutorService executorService;
   protected ScheduledFuture<?> toolboxTaskScheduled = null;
   protected ScheduledFuture<?> yoVariableServerScheduled = null;
   protected Runnable toolboxRunnable = null;
   protected final int updatePeriodMilliseconds;

   protected final QuadrupedRobotDataReceiver robotDataReceiver;

   protected final YoDouble timeWithoutInputsBeforeGoingToSleep = new YoDouble("timeWithoutInputsBeforeGoingToSleep", registry);
   protected final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);
   protected final AtomicBoolean receivedInput = new AtomicBoolean();
   private final LogModelProvider modelProvider;
   private final boolean startYoVariableServer;
   private final DataServerSettings dataServerSettings;
   private YoVariableServer yoVariableServer;

   public QuadrupedToolboxModule(String robotName, FullQuadrupedRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
                                 DataServerSettings dataServerSettings)
   {
      this(robotName, fullRobotModelToLog, modelProvider, startYoVariableServer, dataServerSettings, DEFAULT_UPDATE_PERIOD_MILLISECONDS);
   }

   public QuadrupedToolboxModule(String robotName, FullQuadrupedRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
                                 DataServerSettings dataServerSettings, int updatePeriodMilliseconds)
   {
      this(robotName, fullRobotModelToLog, modelProvider, startYoVariableServer, dataServerSettings, updatePeriodMilliseconds, PubSubImplementation.FAST_RTPS);
   }

   public QuadrupedToolboxModule(String robotName, FullQuadrupedRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
                                 DataServerSettings dataServerSettings, PubSubImplementation pubSubImplementation)
   {
      this(robotName, fullRobotModelToLog, modelProvider, startYoVariableServer, dataServerSettings, DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);
   }

   public QuadrupedToolboxModule(String robotName, FullQuadrupedRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
                                 DataServerSettings dataServerSettings, int updatePeriodMilliseconds, PubSubImplementation pubSubImplementation)
   {
      this.robotName = robotName;
      this.updatePeriodMilliseconds = updatePeriodMilliseconds;

      this.modelProvider = modelProvider;
      this.startYoVariableServer = startYoVariableServer;
      this.dataServerSettings = dataServerSettings;
      this.fullRobotModel = fullRobotModelToLog;
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, "ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, name));
      inputManager = new CommandInputManager(name, createListOfSupportedCommands());
      outputManager = new OutputManager(createMapOfSupportedOutputMessages());
      networkSubscriber = new NetworkSubscriber(getInputTopic(), inputManager, getOutputTopic(), outputManager,
                                                realtimeROS2Node);

      executorService = Executors.newScheduledThreadPool(1, threadFactory);

      timeWithoutInputsBeforeGoingToSleep.set(Double.MAX_VALUE);
      inputManager.registerHasReceivedInputListener(new HasReceivedInputListener()
      {
         private final Set<Class<? extends Command<?, ?>>> silentCommands = silentCommands();

         @Override
         public void hasReceivedInput(Class<? extends Command<?, ?>> commandClass)
         {
            if (!silentCommands.contains(commandClass))
               receivedInput.set(true);
         }
      });

      ROS2Topic controllerOutputTopic = QuadrupedAPI.getQuadrupedControllerOutputTopic(robotName);
      if (fullRobotModel != null)
      {
         robotDataReceiver = new QuadrupedRobotDataReceiver(fullRobotModel, null);
         ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, RobotConfigurationData.class, controllerOutputTopic,
                                              s -> robotDataReceiver.receivedPacket(s.takeNextData()));
      }
      else
      {
         robotDataReceiver = null;
      }

      networkSubscriber.addMessageFilter(createMessageFilter());

      ROS2Tools
            .createCallbackSubscriptionTypeNamed(realtimeROS2Node, ToolboxStateMessage.class, getInputTopic(), s -> receivedPacket(s.takeNextData()));


      registerExtraSubscribers(realtimeROS2Node);
      realtimeROS2Node.spin();
   }

   public void setRootRegistry(YoRegistry rootRegistry, YoGraphicsListRegistry rootGraphicsListRegistry)
   {
      rootRegistry.addChild(registry);
      if (rootGraphicsListRegistry != null)
      {
         ArrayList<YoGraphicsList> graphicsLists = new ArrayList<>();
         ArrayList<ArtifactList> artifactsLists = new ArrayList<>();

         yoGraphicsListRegistry.getRegisteredYoGraphicsLists(graphicsLists);
         yoGraphicsListRegistry.getRegisteredArtifactLists(artifactsLists);

         for (YoGraphicsList graphicsList : graphicsLists)
            rootGraphicsListRegistry.registerYoGraphicsList(graphicsList);
         for (ArtifactList artifactList : artifactsLists)
            rootGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   protected void setTimeWithoutInputsBeforeGoingToSleep(double time)
   {
      timeWithoutInputsBeforeGoingToSleep.set(time);
   }

   protected void startYoVariableServer(Class<?> clazz)
   {
      if (!startYoVariableServer || dataServerSettings == null)
         return;

      yoVariableServer = new YoVariableServer(clazz, modelProvider, dataServerSettings, YO_VARIABLE_SERVER_DT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
      startYoVariableServerOnAThread(yoVariableServer);

      yoVariableServerScheduled = executorService
            .scheduleAtFixedRate(createYoVariableServerRunnable(yoVariableServer), 0, updatePeriodMilliseconds, TimeUnit.MILLISECONDS);
   }

   private void startYoVariableServerOnAThread(final YoVariableServer yoVariableServer)
   {
      new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            yoVariableServer.start();
         }
      }).start();
   }

   private Runnable createYoVariableServerRunnable(final YoVariableServer yoVariableServer)
   {
      return new Runnable()
      {
         double serverTime = 0.0;

         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            serverTime += Conversions.millisecondsToSeconds(updatePeriodMilliseconds);
            yoVariableServer.update(Conversions.secondsToNanoseconds(serverTime));
         }
      };
   }

   public NetworkSubscriber.MessageFilter createMessageFilter()
   {
      return new NetworkSubscriber.MessageFilter()
      {
         private final Set<Class<? extends Settable<?>>> exceptions = filterExceptions();

         @Override
         public boolean isMessageValid(Object message)
         {
            if (exceptions.contains(message.getClass()))
            {
               if (toolboxTaskScheduled == null)
               {
                  if (DEBUG)
                     LogTools.info(name + " is sleeping: " + message.getClass().getSimpleName() + " is ignored.", QuadrupedToolboxModule.this);
                  return false;
               }
               else
               {
                  return true;
               }
            }

            return true;
         }
      };
   }

   public void receivedPacket(ToolboxStateMessage message)
   {
      if (DEBUG)
         PrintTools.info("Received a state message.");

      switch (ToolboxState.fromByte(message.getRequestedToolboxState()))
      {
      case WAKE_UP:
         wakeUp();
         break;
      case REINITIALIZE:
         reinitialize();
         break;
      case SLEEP:
         sleep();
         break;
      }
   }

   public void wakeUp()
   {
      if (toolboxTaskScheduled != null)
      {
         if (DEBUG)
            PrintTools.error(this, "This toolbox is already running.");
         return;
      }

      if (DEBUG)
         PrintTools.debug(this, "Waking up");

      createToolboxRunnable();
      toolboxTaskScheduled = executorService.scheduleAtFixedRate(toolboxRunnable, 0, updatePeriodMilliseconds, TimeUnit.MILLISECONDS);
      getToolboxController().setFutureToListenTo(toolboxTaskScheduled);
      reinitialize();
      receivedInput.set(true);
   }

   private void reinitialize()
   {
      getToolboxController().requestInitialize();
   }

   public void sleep()
   {
      if (DEBUG)
         PrintTools.debug(this, "Going to sleep");

      destroyToolboxRunnable();

      if (toolboxTaskScheduled == null)
      {
         if (DEBUG)
            PrintTools.error(this, "There is no task running.");
         return;
      }

      getToolboxController().setFutureToListenTo(null);
      toolboxTaskScheduled.cancel(true);
      toolboxTaskScheduled = null;
   }

   public void destroy()
   {
      sleep();

      if (yoVariableServerScheduled != null)
      {
         yoVariableServerScheduled.cancel(true);
         yoVariableServerScheduled = null;
      }
      executorService.shutdownNow();

      if (yoVariableServer != null)
      {
         yoVariableServer.close();
         yoVariableServer = null;
      }
      realtimeROS2Node.destroy();

      if (DEBUG)
         PrintTools.debug(this, "Destroyed");
   }

   private void createToolboxRunnable()
   {
      if (toolboxRunnable != null)
      {
         if (DEBUG)
            PrintTools.error(this, "toolboxRunnable is not null.");
         return;
      }

      toolboxRunnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            if (robotDataReceiver != null)
               robotDataReceiver.updateRobotModel();

            try
            {
               getToolboxController().update();
               yoTime.add(Conversions.millisecondsToSeconds(updatePeriodMilliseconds));

               if (receivedInput.getAndSet(false))
                  timeOfLastInput.set(yoTime.getDoubleValue());
               if (yoTime.getDoubleValue() - timeOfLastInput.getDoubleValue() >= timeWithoutInputsBeforeGoingToSleep.getDoubleValue())
                  sleep();
               else if (getToolboxController().isDone())
                  sleep();
            }
            catch (InterruptedException e1)
            {
               e1.printStackTrace();
               sleep();
            }
            catch (Exception e)
            {
               e.printStackTrace();
               sleep();
               throw e;
            }

         }
      };
   }

   private void destroyToolboxRunnable()
   {
      toolboxRunnable = null;
   }

   abstract public void registerExtraSubscribers(RealtimeROS2Node realtimeROS2Node);

   abstract public QuadrupedToolboxController getToolboxController();

   /**
    * @return used to create the {@link CommandInputManager} and to defines the input API.
    */
   abstract public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands();

   /**
    * @return used to create the {@link StatusMessageOutputManager} and to defines the output API.
    */
   abstract public Map<Class<? extends Settable<?>>, ROS2Topic> createMapOfSupportedOutputMessages();

   /**
    * @return the collection of commands that cannot wake up this module.
    */
   public Set<Class<? extends Command<?, ?>>> silentCommands()
   {
      return Collections.emptySet();
   }

   /**
    * @return the collection of messages that are allowed to go through the message filter.
    */
   public Set<Class<? extends Settable<?>>> filterExceptions()
   {
      return Collections.emptySet();
   }

   public abstract ROS2Topic getOutputTopic();

   public abstract ROS2Topic getInputTopic();
}
