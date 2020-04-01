package us.ihmc.avatar.networkProcessor.modules;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import com.google.common.base.CaseFormat;

import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageFilter;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
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
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is a base class for any toolbox in the network manager. See the KinematicsToolboxModule as
 * an example.
 */
public abstract class ToolboxModule implements CloseableAndDisposable
{
   protected static final boolean DEBUG = false;
   protected static final double YO_VARIABLE_SERVER_DT = 0.01;
   protected static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 1;

   protected final String name = getClass().getSimpleName();
   protected final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);
   protected final YoDouble yoTime = new YoDouble("localTime", registry);
   protected final String robotName;
   protected final FullHumanoidRobotModel fullRobotModel;

   protected final RealtimeRos2Node realtimeRos2Node;
   protected final CommandInputManager commandInputManager;
   protected final StatusMessageOutputManager statusOutputManager;
   protected final ControllerNetworkSubscriber controllerNetworkSubscriber;

   protected final ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);
   protected final ScheduledExecutorService executorService;
   protected ScheduledFuture<?> toolboxTaskScheduled = null;
   protected ScheduledFuture<?> yoVariableServerScheduled = null;
   protected Runnable toolboxRunnable = null;
   protected final int updatePeriodMilliseconds;
   protected final YoBoolean isLogging = new YoBoolean("isLogging", registry);

   protected final YoDouble timeWithoutInputsBeforeGoingToSleep = new YoDouble("timeWithoutInputsBeforeGoingToSleep", registry);
   protected final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);
   protected final AtomicBoolean receivedInput = new AtomicBoolean();
   private final LogModelProvider modelProvider;
   private final boolean startYoVariableServer;
   protected YoVariableServer yoVariableServer;

   public ToolboxModule(String robotName, FullHumanoidRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer)
   {
      this(robotName, fullRobotModelToLog, modelProvider, startYoVariableServer, DEFAULT_UPDATE_PERIOD_MILLISECONDS);
   }

   public ToolboxModule(String robotName, FullHumanoidRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
                        int updatePeriodMilliseconds)
   {
      this(robotName, fullRobotModelToLog, modelProvider, startYoVariableServer, updatePeriodMilliseconds, PubSubImplementation.FAST_RTPS);
   }

   public ToolboxModule(String robotName, FullHumanoidRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
                        PubSubImplementation pubSubImplementation)
   {
      this(robotName, fullRobotModelToLog, modelProvider, startYoVariableServer, DEFAULT_UPDATE_PERIOD_MILLISECONDS, pubSubImplementation);
   }

   public ToolboxModule(String robotName, FullHumanoidRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
                        int updatePeriodMilliseconds, PubSubImplementation pubSubImplementation)
   {
      this.robotName = robotName;

      this.modelProvider = modelProvider;
      this.startYoVariableServer = startYoVariableServer;
      this.fullRobotModel = fullRobotModelToLog;
      this.updatePeriodMilliseconds = updatePeriodMilliseconds;
      realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, name));
      commandInputManager = new CommandInputManager(name, createListOfSupportedCommands());
      statusOutputManager = new StatusMessageOutputManager(createListOfSupportedStatus());
      controllerNetworkSubscriber = new ControllerNetworkSubscriber(getSubscriberTopicNameGenerator(), commandInputManager, getPublisherTopicNameGenerator(),
                                                                    statusOutputManager, realtimeRos2Node);

      executorService = Executors.newScheduledThreadPool(1, threadFactory);

      timeWithoutInputsBeforeGoingToSleep.set(0.5);
      commandInputManager.registerHasReceivedInputListener(new HasReceivedInputListener()
      {
         private final Set<Class<? extends Command<?, ?>>> silentCommands = silentCommands();

         @Override
         public void hasReceivedInput(Class<? extends Command<?, ?>> commandClass)
         {
            if (!silentCommands.contains(commandClass))
               receivedInput.set(true);
         }
      });

      controllerNetworkSubscriber.addMessageFilter(createMessageFilter());

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, ToolboxStateMessage.class, getSubscriberTopicNameGenerator(), new NewMessageListener<ToolboxStateMessage>()
      {
         private final ToolboxStateMessage message = new ToolboxStateMessage();
         @Override
         public void onNewDataMessage(Subscriber<ToolboxStateMessage> s)
         {
            s.takeNextData(message, null);
            receivedPacket(message);
         }
      });
      registerExtraPuSubs(realtimeRos2Node);
      realtimeRos2Node.spin();
   }

   public void setRootRegistry(YoVariableRegistry rootRegistry, YoGraphicsListRegistry rootGraphicsListRegistry)
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

   protected void startYoVariableServer()
   {
      if (!startYoVariableServer)
         return;

      yoVariableServer = new YoVariableServer(getClass(), modelProvider, getYoVariableServerSettings(), YO_VARIABLE_SERVER_DT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
      startYoVariableServerOnAThread(yoVariableServer);

      yoVariableServerScheduled = executorService.scheduleAtFixedRate(createYoVariableServerRunnable(yoVariableServer), 0, updatePeriodMilliseconds,
                                                                      TimeUnit.MILLISECONDS);
   }

   public DataServerSettings getYoVariableServerSettings()
   {
      return new DataServerSettings(false);
   }

   private void startYoVariableServerOnAThread(final YoVariableServer yoVariableServer)
   {
      new Thread(yoVariableServer::start).start();
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

   public MessageFilter createMessageFilter()
   {
      return new MessageFilter()
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
                     LogTools.info(name + " is sleeping: " + message.getClass().getSimpleName() + " is ignored.");
                  return false;
               }
               else
               {
                  return true;
               }
            }

            // FIXME
            //            if (message.getDestination() != thisDesitination)
            //            {
            //               if (DEBUG)
            //                  PrintTools.error(ToolboxModule.this, name + ": isMessageValid " + message.getDestination() + "!=" + thisDesitination);
            //               return false;
            //            }
            //
            //            if (toolboxTaskScheduled == null)
            //            {
            //               wakeUp(message.getSource());
            //            }
            //            else if (activeMessageSource.getOrdinal() != message.getSource())
            //            {
            //               if (DEBUG)
            //                  PrintTools.error(ToolboxModule.this, "Expecting messages from " + activeMessageSource.getEnumValue() + " received message from: "
            //                        + PacketDestination.values[message.getSource()]);
            //               return false;
            //            }

            return true;
         }
      };
   }

   public void receivedPacket(ToolboxStateMessage message)
   {
      if (DEBUG)
         LogTools.info("Received a state message.");

      ToolboxState requestedState = ToolboxState.fromByte(message.getRequestedToolboxState());
      if(requestedState == null)
      {
         return;
      }

      boolean isAwake = toolboxRunnable != null;
      if (requestedState == ToolboxState.WAKE_UP && !isAwake)
      {
         wakeUp();
      }
      else if(requestedState == ToolboxState.REINITIALIZE && isAwake)
      {
         reinitialize();
      }
      else if(requestedState == ToolboxState.SLEEP && isAwake)
      {
         sleep();
      }

      handleLoggingRequest(message);
   }

   public void wakeUp()
   {
      if (toolboxTaskScheduled != null)
      {
         if (DEBUG)
            LogTools.error("This toolbox is already running.");
         return;
      }

      if (DEBUG)
         LogTools.debug("Waking up");

      reinitialize();
      receivedInput.set(true);
      getToolboxController().setFutureToListenTo(toolboxTaskScheduled);
      createToolboxRunnable();
      toolboxTaskScheduled = executorService.scheduleAtFixedRate(toolboxRunnable, 0, updatePeriodMilliseconds, TimeUnit.MILLISECONDS);
   }

   private void reinitialize()
   {
      getToolboxController().requestInitialize();
   }

   public void sleep()
   {

      if (DEBUG)
         LogTools.debug("Going to sleep");

      getToolboxController().notifyToolboxStateChange(ToolboxState.SLEEP);

      destroyToolboxRunnable();

      if (toolboxTaskScheduled == null)
      {
         if (DEBUG)
            LogTools.error("There is no task running.");
         return;
      }

      getToolboxController().setFutureToListenTo(null);
      toolboxTaskScheduled.cancel(true);
      toolboxTaskScheduled = null;

      if(isLogging.getValue())
      {
         stopLogging();
      }
   }

   private void handleLoggingRequest(ToolboxStateMessage message)
   {
      boolean isAwake = toolboxRunnable != null;
      if (!isAwake)
         return;

      if (message.getRequestLogging() && !isLogging.getValue())
      {
         startLogging();
         isLogging.set(true);
      }
      else if (!message.getRequestLogging() && isLogging.getValue())
      {
         stopLogging();
         isLogging.set(false);
      }
   }

   @Override
   public void closeAndDispose()
   {
      destroy();
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
      realtimeRos2Node.destroy();

      if (DEBUG)
         LogTools.debug("Destroyed");
   }

   private void createToolboxRunnable()
   {
      if (toolboxRunnable != null)
      {
         if (DEBUG)
            LogTools.error("toolboxRunnable is not null.");
         return;
      }

      toolboxRunnable = () ->
      {
         if (Thread.interrupted())
            return;

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
         catch (Exception e)
         {
            e.printStackTrace();
            sleep();
            throw e;
         }
      };
   }

   private void destroyToolboxRunnable()
   {
      toolboxRunnable = null;
   }

   protected void startLogging()
   {
   }

   protected void stopLogging()
   {
   }

   abstract public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node);

   abstract public ToolboxController getToolboxController();

   /**
    * @return used to create the {@link CommandInputManager} and to defines the input API.
    */
   abstract public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands();

   /**
    * @return used to create the {@link StatusMessageOutputManager} and to defines the output API.
    */
   abstract public List<Class<? extends Settable<?>>> createListOfSupportedStatus();

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

   public abstract ROS2Tools.MessageTopicNameGenerator getPublisherTopicNameGenerator();

   public abstract ROS2Tools.MessageTopicNameGenerator getSubscriberTopicNameGenerator();
}
