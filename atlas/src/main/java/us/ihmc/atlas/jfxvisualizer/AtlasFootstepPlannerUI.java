package us.ihmc.atlas.jfxvisualizer;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HeightMapMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasUIAuxiliaryData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

/**
 * This class provides a visualizer for the footstep planner module.
 * It allows user to create plans, log and load plans from disk, tune parameters,
 * and debug plans.
 */
public class AtlasFootstepPlannerUI extends Application
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
      IHMCRealtimeROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher
            = ROS2Tools.createPublisherTypeNamed(ros2Node, REAStateRequestMessage.class, REACommunicationProperties.inputTopic);
      messageConverter = new RemoteUIMessageConverter(ros2Node, messager, drcRobotModel.getSimpleRobotName());

      messager.startMessager();
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalDistanceProximity, GOAL_DISTANCE_PROXIMITY);

      ui = FootstepPlannerUI.createMessagerUI(primaryStage,
                                              messager,
                                              drcRobotModel.getVisibilityGraphsParameters(),
                                              drcRobotModel.getFootstepPlannerParameters(),
                                              drcRobotModel.getSwingPlannerParameters(),
                                              drcRobotModel,
                                              previewModel,
                                              drcRobotModel.getJointMap(),
                                              drcRobotModel.getContactPointParameters(),
                                              drcRobotModel.getWalkingControllerParameters(),
                                              new AtlasUIAuxiliaryData());
      ui.setRobotLowLevelMessenger(robotLowLevelMessenger);
      ui.setREAStateRequestPublisher(reaStateRequestPublisher);
      ui.show();

      if (launchPlannerToolbox)
      {
         plannerModule = FootstepPlanningModuleLauncher.createModule(drcRobotModel, DomainFactory.PubSubImplementation.FAST_RTPS);

         // Create logger and connect to messager
         FootstepPlannerLogger logger = new FootstepPlannerLogger(plannerModule);
         Runnable loggerRunnable = () -> logger.logSessionAndReportToMessager(messager);
         messager.registerTopicListener(FootstepPlannerMessagerAPI.RequestGenerateLog, b -> new Thread(loggerRunnable).start());
         messager.registerTopicListener(FootstepPlannerMessagerAPI.PlanSingleStep, planSingleStep ->
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

         // Height map planner
         AtomicReference<HeightMapMessage> heightMapMessage = messager.createInput(FootstepPlannerMessagerAPI.HeightMapMessage);
         AtomicReference<FootstepPlannerParametersReadOnly> plannerParameters = messager.createInput(PlannerParameters);
         ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
         AtomicReference<Pose3DReadOnly> leftFootStartPose = messager.createInput(LeftFootPose);
         AtomicReference<Pose3DReadOnly> rightFootStartPose = messager.createInput(RightFootPose);
         AtomicReference<Pose3DReadOnly> leftFootGoalPose = messager.createInput(LeftFootGoalPose);
         AtomicReference<Pose3DReadOnly> rightFootGoalPose = messager.createInput(RightFootGoalPose);

         RobotContactPointParameters<RobotSide> contactPointParameters = drcRobotModel.getContactPointParameters();
         SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
         {
            ConvexPolygon2D footPolygon = new ConvexPolygon2D();
            contactPointParameters.getControllerFootGroundContactPoints().get(side).forEach(footPolygon::addVertex);
            footPolygon.update();
            footPolygons.put(side, footPolygon);
         }

         new ROS2Callback<>(ros2Node, HeightMapMessage.class, ROS2Tools.HEIGHT_MAP_OUTPUT, message -> messager.submitMessage(HeightMapMessage, message));
         messager.registerTopicListener(FootstepPlannerMessagerAPI.PlanWithHeightMap, p -> executorService.execute(() ->
                                 {
                                    Pose3D start = new Pose3D();
                                    Pose3D goal = new Pose3D();
                                    start.interpolate(leftFootStartPose.get(), rightFootStartPose.get(), 0.5);
                                    goal.interpolate(leftFootGoalPose.get(), rightFootGoalPose.get(), 0.5);

                                    HeightMapMessage heightMap = heightMapMessage.get();
                                    if (heightMap == null)
                                    {
                                       LogTools.error("No height map available");
                                       return;
                                    }

                                    HeightMapData heightMapData = new HeightMapData(heightMap.getXyResolution(), heightMap.getGridSizeXy());

                                    int xSearch = 62;
                                    int ySearch = 59;
                                    for (int i = 0; i < heightMap.getXCells().size(); i++)
                                    {
                                       if (heightMap.getXCells().get(i) == xSearch && heightMap.getYCells().get(i) == ySearch)
                                          System.out.println("Found (" + xSearch + ", " + ySearch + ") at index " + i);
                                    }

                                    heightMapData.getXCells().addAll(heightMap.getXCells());
                                    heightMapData.getYCells().addAll(heightMap.getYCells());
                                    for (int i = 0; i < heightMap.getHeights().size(); i++)
                                    {
                                       heightMapData.getHeights().add(heightMap.getHeights().get(i));
                                    }

                                    heightMapData.sort();
                                    FootstepPlan footstepPlan = HeightMapFootstepPlanner.plan(start, goal, plannerParameters.get(), footPolygons, heightMapData);
                                    if (!footstepPlan.isEmpty())
                                    {
                                       FootstepDataListMessage outputMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(
                                             footstepPlan,
                                             -1.0,
                                             -1.0);
                                       messager.submitMessage(FootstepPlanResponse, outputMessage);
                                    }
                                 }));
      }
   }

   private void handleMessagerCallbacks(FootstepPlanningModule planningModule, FootstepPlannerOutput status)
   {
      if (status.getFootstepPlanningResult() != null && status.getFootstepPlanningResult().terminalResult())
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.GraphData,
                                Triple.of(planningModule.getEdgeDataMap(), planningModule.getIterationData(), planningModule.getVariableDescriptors()));
         messager.submitMessage(FootstepPlannerMessagerAPI.StartVisibilityMap, planningModule.getBodyPathPlanner().getSolution().getStartMap());
         messager.submitMessage(FootstepPlannerMessagerAPI.GoalVisibilityMap, planningModule.getBodyPathPlanner().getSolution().getGoalMap());
         messager.submitMessage(FootstepPlannerMessagerAPI.InterRegionVisibilityMap, planningModule.getBodyPathPlanner().getSolution().getInterRegionVisibilityMap());
         messager.submitMessage(FootstepPlannerMessagerAPI.VisibilityMapWithNavigableRegionData, planningModule.getBodyPathPlanner().getSolution().getVisibilityMapsWithNavigableRegions());
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