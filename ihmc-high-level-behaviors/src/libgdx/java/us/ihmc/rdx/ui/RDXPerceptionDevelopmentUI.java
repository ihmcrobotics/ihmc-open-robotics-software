package us.ihmc.rdx.ui;

import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HumanoidPerceptionModule;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.logging.RDXPerceptionDataLoaderPanel;
import us.ihmc.rdx.logging.RDXPerceptionDataLoggerPanel;
import us.ihmc.rdx.perception.RDXHumanoidPerceptionUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXBuildingConstructor;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.graphics.ros2.*;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.net.URISyntaxException;

public class RDXPerceptionDevelopmentUI
{
   private final PerceptionDataLoader perceptionDataLoader = new PerceptionDataLoader();

   private RDXBaseUI baseUI;
   private RDXGlobalVisualizersPanel globalVisualizersUI;

   private RDXPerceptionDataLoaderPanel perceptionLogLoaderPanel;
   private RDXPerceptionDataLoggerPanel perceptionLoggingPanel;
   private PerceptionDataLogger logger;

   private RDXEnvironmentBuilder environmentBuilder;
   private RDXBuildingConstructor buildingConstructor;
   private RDXHumanoidPerceptionUI humanoidPerceptionUI;
   private RDXPlanarRegionsGraphic planarRegionsGraphic;

   private FootstepPlanningModule footstepPlanningModule;
   private RDXFootstepPlanGraphic footstepPlanGraphic;


   private static final File logFile = new File(IHMCCommonPaths.LOGS_DIRECTORY.resolve("20230903_222303_PlanarRegionsListLogger.prllog").toString());
   private PlanarRegionsList logLoadedPlanarRegions;

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final HumanoidPerceptionModule perceptionModule = new HumanoidPerceptionModule(openCLManager);

   public RDXPerceptionDevelopmentUI()
   {
      logger = new PerceptionDataLogger();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "perception_ui_node");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      footstepPlanningModule = new FootstepPlanningModule("perceptionUI");

      globalVisualizersUI = new RDXGlobalVisualizersPanel();
      baseUI = new RDXBaseUI("Perception UI");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            footstepPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
            baseUI.getPrimaryScene().addRenderableProvider(footstepPlanGraphic);

            globalVisualizersUI.addVisualizer(new RDXROS2FramePlanarRegionsVisualizer("Rapid Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS));

            humanoidPerceptionUI = new RDXHumanoidPerceptionUI(perceptionModule, ros2Helper);
            baseUI.getImGuiPanelManager().addPanel(humanoidPerceptionUI.getRemotePerceptionUI().getPanel());

            humanoidPerceptionUI.initializePerspectiveRegionsVisualizer(ros2Node, globalVisualizersUI, true);
            humanoidPerceptionUI.initializeImageMessageVisualizer("Ouster Depth", PerceptionAPI.OUSTER_DEPTH_IMAGE, globalVisualizersUI);
            humanoidPerceptionUI.initializeImageMessageVisualizer("L515 Color", PerceptionAPI.L515_COLOR_IMAGE, globalVisualizersUI);
            humanoidPerceptionUI.initializeImageMessageVisualizer("L515 Depth", PerceptionAPI.L515_DEPTH_IMAGE, globalVisualizersUI);
            humanoidPerceptionUI.initializeImageMessageVisualizer("D455 Color", PerceptionAPI.D455_COLOR_IMAGE, globalVisualizersUI);
            humanoidPerceptionUI.initializeImageMessageVisualizer("D455 Depth", PerceptionAPI.D455_DEPTH_IMAGE, globalVisualizersUI);
            humanoidPerceptionUI.initializeImageMessageVisualizer("D435 Color", PerceptionAPI.D435_COLOR_IMAGE, globalVisualizersUI);
            humanoidPerceptionUI.initializeImageMessageVisualizer("D435 Depth", PerceptionAPI.D435_DEPTH_IMAGE, globalVisualizersUI);

            logLoadedPlanarRegions = PlanarRegionFileTools.loadRegionsFromLog(logFile);
            PerceptionFilterTools.applyConcaveHullReduction(logLoadedPlanarRegions, new PolygonizerParameters("ForGPURegions"));
            PlanarRegion initialSupportSquareRegion = PlanarRegionTools.createSquarePlanarRegion(10.0f,
                                                                                                 new Point3D(),
                                                                                                 new Quaternion());
            initialSupportSquareRegion.setRegionId(0);
            logLoadedPlanarRegions.addPlanarRegion(initialSupportSquareRegion);

            globalVisualizersUI.addVisualizer(new RDXROS2PlanarRegionsVisualizer("Rapid Regions",
                                                                                 ros2Node,
                                                                                 PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS));

            RDXROS2ColoredPointCloudVisualizer d455ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("D455 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.D455_COLOR_IMAGE);
            d455ColoredDepthVisualizer.setSubscribed(true);
            d455ColoredDepthVisualizer.setActive(true);
            globalVisualizersUI.addVisualizer(d455ColoredDepthVisualizer);



            RDXROS2ColoredPointCloudVisualizer ZEDColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED2 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.ZED2_DEPTH,
                                                                                                                   PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            globalVisualizersUI.addVisualizer(ZEDColoredDepthVisualizer);
            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED2 Color Stereo",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT)));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED2 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.ZED2_DEPTH));

            RDXROS2BigVideoVisualizer blackflyRightVideoVisualizer = new RDXROS2BigVideoVisualizer("IHMC Blackfly Right",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.BLACKFLY_VIDEO.get(RobotSide.RIGHT));
            blackflyRightVideoVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(blackflyRightVideoVisualizer);

            RDXROS2ColoredPointCloudVisualizer l515ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("L515 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.L515_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.L515_COLOR_IMAGE);
            l515ColoredDepthVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(l515ColoredDepthVisualizer);

            RDXROS2ColoredPointCloudVisualizer d435ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("D435 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.D435_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.D435_COLOR_IMAGE);
            d435ColoredDepthVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(d435ColoredDepthVisualizer);

            RDXROS2PointCloudVisualizer l515ColoredPointCloudVisualizer = new RDXROS2PointCloudVisualizer("L515 Colored Point Cloud",
                                                                                                          ros2Node,
                                                                                                          PerceptionAPI.FUSED_SENSOR_HEAD_POINT_CLOUD);
            l515ColoredPointCloudVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(l515ColoredPointCloudVisualizer);

            RDXROS2PointCloudVisualizer d435ColoredPointCloudVisualizer = new RDXROS2PointCloudVisualizer("D435 Colored Point Cloud",
                                                                                                          ros2Node,
                                                                                                          PerceptionAPI.D435_COLORED_POINT_CLOUD);
            d435ColoredPointCloudVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(d435ColoredPointCloudVisualizer);

            RDXROS2OusterPointCloudVisualizer ousterPointCloudVisualizer = new RDXROS2OusterPointCloudVisualizer("Ouster Point Cloud",
                                                                                                                 PubSubImplementation.FAST_RTPS,
                                                                                                                 PerceptionAPI.OUSTER_DEPTH_IMAGE);

            ousterPointCloudVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(ousterPointCloudVisualizer);

            RDXROS2RigidBodyPoseVisualizer mocapPoseVisualizer = new RDXROS2RigidBodyPoseVisualizer("Mocap Pose",
                                                                                                    PubSubImplementation.FAST_RTPS,
                                                                                                    PerceptionAPI.MOCAP_RIGID_BODY);
            mocapPoseVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(mocapPoseVisualizer);

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            buildingConstructor = new RDXBuildingConstructor(baseUI.getPrimary3DPanel());

            perceptionLoggingPanel = new RDXPerceptionDataLoggerPanel("Perception Logger", logger);
            baseUI.getImGuiPanelManager().addPanel(perceptionLoggingPanel);

            perceptionLogLoaderPanel = new RDXPerceptionDataLoaderPanel(perceptionDataLoader);
            baseUI.getImGuiPanelManager().addPanel(perceptionLogLoaderPanel);

            baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(buildingConstructor.getPanelName(), buildingConstructor::renderImGuiWidgets);

            RDXPanel evaluationPanel = new RDXPanel("Evaluation Panel");
            baseUI.getImGuiPanelManager().addPanel(evaluationPanel);
            evaluationPanel.setRenderMethod(this::renderImGuiWidgets);

            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI);

            environmentBuilder.create();
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            buildingConstructor.create();
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getRealRenderables, RDXSceneLevel.MODEL);

            planarRegionsGraphic = new RDXPlanarRegionsGraphic();
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionsGraphic);
            planarRegionsGraphic.generateMeshes(logLoadedPlanarRegions);
            planarRegionsGraphic.update();

            globalVisualizersUI.create();
         }

         public void planFootsteps()
         {
            Pose3D leftStartPose = new Pose3D();
            Pose3D rightStartPose = new Pose3D();
            Pose3D leftGoalPose = new Pose3D();
            Pose3D rightGoalPose = new Pose3D();
            leftStartPose.getPosition().set(-0.5, -0.5, 0.0);
            leftGoalPose.getPosition().set(2.5, 4.5, 0.0);

            float yaw = (float) Math.atan2(leftGoalPose.getY() - leftStartPose.getY(), leftGoalPose.getX() - leftStartPose.getX());

            leftStartPose.appendYawRotation(yaw);
            leftGoalPose.appendYawRotation(yaw);

            rightStartPose.set(leftStartPose);
            rightStartPose.appendTranslation(0.0, -0.2, 0.0);

            rightGoalPose.set(leftGoalPose);
            rightGoalPose.appendTranslation(0.0, -0.2, 0.0);

            FootstepPlannerRequest request = new FootstepPlannerRequest();
            request.setStartFootPoses(leftStartPose, rightStartPose);
            request.setGoalFootPoses(leftGoalPose, rightGoalPose);
            request.setPlanarRegionsList(logLoadedPlanarRegions);

            request.setPlanBodyPath(false);
            request.setPerformAStarSearch(true);

            FootstepPlannerOutput plannerOutput = footstepPlanningModule.handleRequest(request);

            if (plannerOutput != null)
            {
               FootstepPlanningResult footstepPlanningResult = plannerOutput.getFootstepPlanningResult();

               LogTools.info("Result: {}" + String.format("\tPlan Iterations: %d, Plan Time: %.4f, Total Time: %.4f, Steps: %d",
                              plannerOutput.getPlannerTimings().getStepPlanningIterations(),
                              plannerOutput.getPlannerTimings().getTimePlanningStepsSeconds(),
                              plannerOutput.getPlannerTimings().getTotalElapsedSeconds(),
                              plannerOutput.getFootstepPlan().getNumberOfSteps()),
                              footstepPlanningResult);

               FootstepDataListMessage message = FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 0.6, 0.3);

               footstepPlanGraphic.generateMeshesAsync(message, "Evaluation Footstep Plan");
               footstepPlanGraphic.update();
            }
         }

         public void renderImGuiWidgets()
         {
            if (ImGui.button("Plan Footsteps"))
            {
               planFootsteps();
            }
         }

         @Override
         public void render()
         {
            globalVisualizersUI.update();
            footstepPlanGraphic.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            environmentBuilder.destroy();
            perceptionLoggingPanel.destroy();
            globalVisualizersUI.destroy();
            baseUI.dispose();
            ros2Node.destroy();
         }
      });
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new RDXPerceptionDevelopmentUI();
   }
}

