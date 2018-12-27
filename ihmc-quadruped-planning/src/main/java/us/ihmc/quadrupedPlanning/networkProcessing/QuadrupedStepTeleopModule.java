package us.ihmc.quadrupedPlanning.networkProcessing;

import com.google.common.base.CaseFormat;
import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.input.QuadrupedRobotModelProviderNode;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class QuadrupedStepTeleopModule
{
   private static final boolean DEBUG = false;
   private static final double YO_VARIABLE_SERVER_DT = 0.01;
   private static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 2;

   private final String name = getClass().getSimpleName();
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final FullQuadrupedRobotModel fullRobotModel;

   private final RealtimeRos2Node realtimeRos2Node;

   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> taskScheduled = null;
   private ScheduledFuture<?> yoVariableServerScheduled = null;
   private final int updatePeriodMilliseconds = 1;

   private final AtomicBoolean receivedInput = new AtomicBoolean();
   private final LogModelProvider modelProvider;
   private final boolean startYoVariableServer;
   private final YoVariableServer yoVariableServer;

   private final QuadrupedStepTeleopController stepTeleopController;

   public QuadrupedStepTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettings defaultXGaitSettings,
                                    double nominalHeight, LogModelProvider modelProvider, boolean startYoVariableServer,
                                    DomainFactory.PubSubImplementation pubSubImplementation)
   {

      this.modelProvider = modelProvider;
      this.startYoVariableServer = startYoVariableServer;
      this.fullRobotModel = modelFactory.createFullRobotModel();

      String robotName = modelFactory.getRobotDescription().getName();

      realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, name));
      CommandInputManager commandInputManager = new CommandInputManager(name, FootstepPlannerCommunicationProperties.getSupportedCommands());
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(FootstepPlannerCommunicationProperties.getSupportedStatusMessages());
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(
            QuadrupedStepTeleopCommunicationProperties.subscriberTopicNameGenerator(robotName), commandInputManager,
            QuadrupedStepTeleopCommunicationProperties.publisherTopicNameGenerator(robotName), statusOutputManager, realtimeRos2Node);

      ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);
      executorService = Executors.newScheduledThreadPool(1, threadFactory);

      commandInputManager.registerHasReceivedInputListener(command -> receivedInput.set(true));

      QuadrupedRobotModelProviderNode robotModelProvider = new QuadrupedRobotModelProviderNode(robotName, realtimeRos2Node, modelFactory);


      stepTeleopController = new QuadrupedStepTeleopController(defaultXGaitSettings, nominalHeight, commandInputManager, statusOutputManager,
                                                               robotModelProvider, executorService, registry, yoGraphicsListRegistry,
                                                               DEFAULT_UPDATE_PERIOD_MILLISECONDS);

      //      String controllerPubGenerator
      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, ToolboxStateMessage.class,
                                           QuadrupedStepTeleopCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> receivedPacket(s.takeNextData()));

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> stepTeleopController.setPaused(true));

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedBodyPathPlanMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processBodyPathPlanMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processFootstepStatusMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processSteppingStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedGroundPlaneMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processGroundPlaneMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator,
                                           s -> stepTeleopController.processTimestamp(s.takeNextData().getTimestamp()));


      realtimeRos2Node.spin();

      yoVariableServer = startYoVariableServer();
   }

   private YoVariableServer startYoVariableServer()
   {
      if (!startYoVariableServer)
         return null;

      PeriodicThreadSchedulerFactory scheduler = new PeriodicNonRealtimeThreadSchedulerFactory();
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), scheduler, modelProvider, LogSettings.TOOLBOX, YO_VARIABLE_SERVER_DT);
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
      if (taskScheduled != null)
      {
         return;
      }

      switch (ToolboxState.fromByte(message.getRequestedToolboxState()))
      {
      case WAKE_UP:
         stepTeleopController.wakeUp();
         break;
      case REINITIALIZE:
         stepTeleopController.reinitialize();
         break;
      case SLEEP:
         stepTeleopController.sleep();
         break;
      }
   }

   public void destroy()
   {
      stepTeleopController.destroy();

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
