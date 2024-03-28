package us.ihmc.atlas.jfxvisualizer;

import java.util.List;

import org.apache.commons.lang3.tuple.Triple;

import javafx.application.Platform;
import javafx.stage.Stage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasUIAuxiliaryData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerTerminationCondition;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * This class provides a visualizer for the footstep planner module.
 * It allows user to create plans, log and load plans from disk, tune parameters,
 * and debug plans.
 */
public class AtlasFootstepPlannerUI extends ApplicationNoModule
{
   private static final double GOAL_DISTANCE_PROXIMITY = 0.1;

   private SharedMemoryJavaFXMessager messager;
   private RemoteUIMessageConverter messageConverter;

   private FootstepPlannerUI ui;
   private FootstepPlanningModule plannerModule;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      List<String> parameters = getParameters().getRaw();
      boolean launchPlannerToolbox = parameters == null || !parameters.contains(getSuppressToolboxFlag());

      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
      DRCRobotModel previewModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
      messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);

      RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ihmc_footstep_planner_ui");
      AtlasLowLevelMessenger robotLowLevelMessenger = new AtlasLowLevelMessenger(ros2Node, drcRobotModel.getSimpleRobotName());
      ROS2PublisherBasics<REAStateRequestMessage> reaStateRequestPublisher
            = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(REAStateRequestMessage.class).withTopic(REACommunicationProperties.inputTopic));
      messageConverter = new RemoteUIMessageConverter(ros2Node, messager, drcRobotModel.getSimpleRobotName());

      messager.startMessager();
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalDistanceProximity, GOAL_DISTANCE_PROXIMITY);

      ui = FootstepPlannerUI.createUI(primaryStage,
                                      messager,
                                      drcRobotModel.getAStarBodyPathPlannerParameters(),
                                      drcRobotModel.getFootstepPlannerParameters("ForLookAndStep"),
                                      drcRobotModel.getSwingPlannerParameters(),
                                      drcRobotModel,
                                      previewModel,
                                      drcRobotModel.getJointMap(),
                                      drcRobotModel.getContactPointParameters(),
                                      drcRobotModel.getWalkingControllerParameters(),
                                      new AtlasUIAuxiliaryData(),
                                      drcRobotModel.getCollisionBoxProvider());
      ui.show();

      if (launchPlannerToolbox)
      {
         plannerModule = FootstepPlanningModuleLauncher.createModule(drcRobotModel, DomainFactory.PubSubImplementation.FAST_RTPS);

         // Create logger and connect to messager
         FootstepPlannerLogger logger = new FootstepPlannerLogger(plannerModule);
         Runnable loggerRunnable = () -> logger.logSessionAndReportToMessager(messager);
         messager.addTopicListener(FootstepPlannerMessagerAPI.RequestGenerateLog, b -> new Thread(loggerRunnable).start());
         messager.addTopicListener(FootstepPlannerMessagerAPI.PlanSingleStep, planSingleStep ->
         {
            if (planSingleStep)
            {
               FootstepPlannerTerminationCondition terminationCondition = (plannerTime, iterations, bestPathFinalStep, bestSecondToFinalStep, bestPathSize) -> bestPathSize >= 1;
               plannerModule.addCustomTerminationCondition(terminationCondition);
            }
            else
            {
               plannerModule.clearCustomTerminationConditions();
            }
         });

         // Automatically send graph data over messager
         plannerModule.addStatusCallback(status -> handleMessagerCallbacks(plannerModule, status));
      }
   }

   private void handleMessagerCallbacks(FootstepPlanningModule planningModule, FootstepPlannerOutput status)
   {
      if (status.getFootstepPlanningResult() != null && status.getFootstepPlanningResult().terminalResult())
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.GraphData,
                                Triple.of(planningModule.getEdgeDataMap(), planningModule.getIterationData(), planningModule.getFootstepPlanVariableDescriptors()));
      }
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      messager.closeMessager();
      messageConverter.destroy();
      ui.stop();

      if (plannerModule != null)
         plannerModule.closeAndDispose();

      Platform.exit();
   }

   /**
    * Pass this as a program argument to suppress the toolbox from being launched
    */
   public static String getSuppressToolboxFlag()
   {
      return "suppressToolbox";
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}