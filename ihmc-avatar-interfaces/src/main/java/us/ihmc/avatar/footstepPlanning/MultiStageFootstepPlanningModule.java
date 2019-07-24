package us.ihmc.avatar.footstepPlanning;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.PlanningStatisticsRequestMessage;
import controller_msgs.msg.dds.RequestFootstepPlannerParametersMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class MultiStageFootstepPlanningModule
{
   private static final boolean DEBUG = false;
   private static final double YO_VARIABLE_SERVER_DT = 0.01;
   private static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 2;

   private final String name = getClass().getSimpleName();
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final String robotName;
   private final FullHumanoidRobotModel fullRobotModel;

   private final RealtimeRos2Node realtimeRos2Node;

   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> yoVariableServerScheduled = null;
   private final int updatePeriodMilliseconds = 1;

   private final AtomicBoolean receivedInput = new AtomicBoolean();
   private final LogModelProvider modelProvider;
   private final boolean startYoVariableServer;
   private final YoVariableServer yoVariableServer;

   private final MultiStageFootstepPlanningController footstepPlanningController;
   
   public MultiStageFootstepPlanningModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer)
   {
      this(drcRobotModel, modelProvider, startYoVariableServer, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public MultiStageFootstepPlanningModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer,
                                           DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this.robotName = drcRobotModel.getSimpleRobotName();

      this.modelProvider = modelProvider;
      this.startYoVariableServer = startYoVariableServer;
      this.fullRobotModel = drcRobotModel.createFullRobotModel();
      realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, ROS2Tools.FOOTSTEP_PLANNER.getNodeName());
      CommandInputManager commandInputManager = new CommandInputManager(name, FootstepPlannerCommunicationProperties.getSupportedCommands());
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(FootstepPlannerCommunicationProperties.getSupportedStatusMessages());
      new ControllerNetworkSubscriber(FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName), commandInputManager,
                                      FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName), statusOutputManager, realtimeRos2Node);

      ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);
      executorService = Executors.newScheduledThreadPool(1, threadFactory);

      commandInputManager.registerHasReceivedInputListener(command -> receivedInput.set(true));

      footstepPlanningController = new MultiStageFootstepPlanningController(drcRobotModel, commandInputManager, statusOutputManager, executorService, registry,
                                                                            DEFAULT_UPDATE_PERIOD_MILLISECONDS);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, ToolboxStateMessage.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> receivedPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlanningRequestPacket.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> footstepPlanningController.processRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlannerParametersPacket.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> footstepPlanningController.processFootstepPlannerParameters(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, VisibilityGraphsParametersPacket.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> footstepPlanningController.processVisibilityGraphsParameters(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PlanningStatisticsRequestMessage.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> footstepPlanningController.processPlanningStatisticsRequest());
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RequestFootstepPlannerParametersMessage.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> footstepPlanningController.broadcastPlannerParameters());

      IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher = ROS2Tools
            .createPublisher(realtimeRos2Node, TextToSpeechPacket.class, ROS2Tools::generateDefaultTopicName);
      IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> parametersPublisher = ROS2Tools
            .createPublisher(realtimeRos2Node, FootstepPlannerParametersPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));

      footstepPlanningController.setTextToSpeechPublisher(textToSpeechPublisher);
      footstepPlanningController.setParametersPublisher(parametersPublisher);

      realtimeRos2Node.spin();

      yoVariableServer = startYoVariableServer();
   }

   private YoVariableServer startYoVariableServer()
   {
      if (!startYoVariableServer)
         return null;

      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), modelProvider, LogSettings.TOOLBOX, YO_VARIABLE_SERVER_DT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
      new Thread(() -> yoVariableServer.start()).start();

      yoVariableServerScheduled = executorService
            .scheduleAtFixedRate(createYoVariableServerRunnable(yoVariableServer), 0, updatePeriodMilliseconds, TimeUnit.MILLISECONDS);

      return yoVariableServer;
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

   public void receivedPacket(ToolboxStateMessage message)
   {
      switch (ToolboxState.fromByte(message.getRequestedToolboxState()))
      {
      case WAKE_UP:
         footstepPlanningController.wakeUp();
         break;
      case REINITIALIZE:
         footstepPlanningController.reinitialize();
         break;
      case SLEEP:
         footstepPlanningController.sleep();
         break;
      }
   }

   public void destroy()
   {
      footstepPlanningController.destroy();

      if (yoVariableServerScheduled != null)
      {
         yoVariableServerScheduled.cancel(true);
      }
      executorService.shutdownNow();

      if (yoVariableServer != null)
      {
         yoVariableServer.close();
      }
      realtimeRos2Node.destroy();

      if (DEBUG)
         PrintTools.debug(this, "Destroyed");
   }
}
