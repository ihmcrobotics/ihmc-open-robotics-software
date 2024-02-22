package us.ihmc.rdx.ui;

import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
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
import us.ihmc.rdx.ui.graphics.ros2.*;
import us.ihmc.rdx.ui.graphics.RDXGlobalVisualizersPanel;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
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

   private static final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("IROS_2023/20230228_201947_PerceptionLog.hdf5").toString();
   private static final File logFile = new File(IHMCCommonPaths.LOGS_DIRECTORY.resolve("20230911_171527_PlanarRegionsListLogger.prllog").toString());

   private PlanarRegionsList logLoadedPlanarRegions = new PlanarRegionsList();

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final HumanoidPerceptionModule perceptionModule;

   private int index = 0;

   public RDXPerceptionDevelopmentUI()
   {
      logger = new PerceptionDataLogger();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "perception_ui_node");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      footstepPlanningModule = new FootstepPlanningModule("perceptionUI");

      perceptionModule = new HumanoidPerceptionModule(openCLManager);
      perceptionModule.initializeRealsenseDepthImage(720, 1280);

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

            humanoidPerceptionUI.initializePerspectiveRegionsVisualizer(ros2Node);

            globalVisualizersUI.addVisualizer(new RDXROS2PlanarRegionsVisualizer("Rapid Regions",
                                                                                 ros2Node,
                                                                                 PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS));

            RDXROS2ColoredPointCloudVisualizer d455ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("D455 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.D455_COLOR_IMAGE);
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
            globalVisualizersUI.addVisualizer(blackflyRightVideoVisualizer);

            RDXROS2ColoredPointCloudVisualizer l515ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("L515 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.L515_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.L515_COLOR_IMAGE);
            globalVisualizersUI.addVisualizer(l515ColoredDepthVisualizer);

            RDXROS2ColoredPointCloudVisualizer d435ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("D435 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.D435_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.D435_COLOR_IMAGE);
            globalVisualizersUI.addVisualizer(d435ColoredDepthVisualizer);

            RDXROS2PointCloudVisualizer l515ColoredPointCloudVisualizer = new RDXROS2PointCloudVisualizer("L515 Colored Point Cloud",
                                                                                                          ros2Node,
                                                                                                          PerceptionAPI.FUSED_SENSOR_HEAD_POINT_CLOUD);
            globalVisualizersUI.addVisualizer(l515ColoredPointCloudVisualizer);

            RDXROS2PointCloudVisualizer d435ColoredPointCloudVisualizer = new RDXROS2PointCloudVisualizer("D435 Colored Point Cloud",
                                                                                                          ros2Node,
                                                                                                          PerceptionAPI.D435_COLORED_POINT_CLOUD);
            globalVisualizersUI.addVisualizer(d435ColoredPointCloudVisualizer);

            RDXROS2OusterPointCloudVisualizer ousterPointCloudVisualizer = new RDXROS2OusterPointCloudVisualizer("Ouster Point Cloud",
                                                                                                                 PubSubImplementation.FAST_RTPS,
                                                                                                                 PerceptionAPI.OUSTER_DEPTH_IMAGE);

            globalVisualizersUI.addVisualizer(ousterPointCloudVisualizer);

            RDXROS2RigidBodyPoseVisualizer mocapPoseVisualizer = new RDXROS2RigidBodyPoseVisualizer("Mocap Pose",
                                                                                                    PubSubImplementation.FAST_RTPS,
                                                                                                    PerceptionAPI.MOCAP_RIGID_BODY);
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

         public void loadPlanarRegionLog()
         {
            PlanarRegion initialSupportSquareRegion = PlanarRegionTools.createSquarePlanarRegion(10.0f,
                                                                                                 new Point3D(),
                                                                                                 new Quaternion());
            initialSupportSquareRegion.setRegionId(100);
            logLoadedPlanarRegions.addPlanarRegion(initialSupportSquareRegion);

            logLoadedPlanarRegions.addPlanarRegionsList(PlanarRegionFileTools.loadRegionsFromLog(logFile));
            logLoadedPlanarRegions.applyTransform(new RigidBodyTransform(new Quaternion(), new Point3D(0.25, 2.0, -0.25)));
            PerceptionFilterTools.applyConcaveHullReduction(logLoadedPlanarRegions, new PolygonizerParameters("ForGPURegions"));
         }

         public void renderImGuiWidgets()
         {
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

