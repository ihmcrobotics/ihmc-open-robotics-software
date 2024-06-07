package us.ihmc.rdx.ui;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.logging.RDXPerceptionDataLoaderPanel;
import us.ihmc.rdx.logging.RDXPerceptionDataLoggerPanel;
import us.ihmc.rdx.perception.RDXRemotePerceptionUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXBuildingConstructor;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2BigVideoVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2OusterPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2PlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2PointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RigidBodyPoseVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import java.net.URISyntaxException;

public class RDXPerceptionUI
{
   private final PerceptionDataLoader perceptionDataLoader = new PerceptionDataLoader();

   private RDXBaseUI baseUI;
   private RDXPerceptionVisualizerPanel perceptionVisualizerPanel;

   private RDXPerceptionDataLoaderPanel perceptionLogLoaderPanel;
   private RDXPerceptionDataLoggerPanel perceptionLoggingPanel;
   private PerceptionDataLogger logger;

   private RDXEnvironmentBuilder environmentBuilder;
   private RDXBuildingConstructor buildingConstructor;
   private RDXRemotePerceptionUI remotePerceptionUI;

   public RDXPerceptionUI()
   {
      logger = new PerceptionDataLogger();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "perception_ui_node");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();
      baseUI = new RDXBaseUI("Perception UI");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            perceptionVisualizerPanel.addVisualizer(new RDXROS2FramePlanarRegionsVisualizer("Rapid Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2PlanarRegionsVisualizer("Rapid Regions",
                                                                                 ros2Node,
                                                                                 PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("Ouster Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.OUSTER_DEPTH_IMAGE));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("L515 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.L515_COLOR_IMAGE));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("L515 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.L515_DEPTH_IMAGE));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("D455 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D455_COLOR_IMAGE));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("D455 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D455_DEPTH_IMAGE));

            RDXROS2ColoredPointCloudVisualizer d455ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("D455 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.D455_COLOR_IMAGE);
            d455ColoredDepthVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(d455ColoredDepthVisualizer);

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("D455 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D455_COLOR_IMAGE));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("D455 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D455_DEPTH_IMAGE));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("D435 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D435_COLOR_IMAGE));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("D435 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D435_DEPTH_IMAGE));

            RDXROS2ColoredPointCloudVisualizer ZEDColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED2 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.ZED2_DEPTH,
                                                                                                                   PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(ZEDColoredDepthVisualizer);
            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED2 Color Stereo",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT)));

            perceptionVisualizerPanel.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED2 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.ZED2_DEPTH));

            RDXROS2BigVideoVisualizer blackflyRightVideoVisualizer = new RDXROS2BigVideoVisualizer("IHMC Blackfly Right",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.BLACKFLY_VIDEO.get(RobotSide.RIGHT));
            perceptionVisualizerPanel.addVisualizer(blackflyRightVideoVisualizer);

            RDXROS2ColoredPointCloudVisualizer l515ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("L515 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.L515_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.L515_COLOR_IMAGE);
            perceptionVisualizerPanel.addVisualizer(l515ColoredDepthVisualizer);

            RDXROS2ColoredPointCloudVisualizer d435ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("D435 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.D435_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.D435_COLOR_IMAGE);
            perceptionVisualizerPanel.addVisualizer(d435ColoredDepthVisualizer);

            RDXROS2PointCloudVisualizer l515ColoredPointCloudVisualizer = new RDXROS2PointCloudVisualizer("L515 Colored Point Cloud",
                                                                                                          ros2Node,
                                                                                                          PerceptionAPI.FUSED_SENSOR_HEAD_POINT_CLOUD);
            perceptionVisualizerPanel.addVisualizer(l515ColoredPointCloudVisualizer);

            RDXROS2PointCloudVisualizer d435ColoredPointCloudVisualizer = new RDXROS2PointCloudVisualizer("D435 Colored Point Cloud",
                                                                                                          ros2Node,
                                                                                                          PerceptionAPI.D435_COLORED_POINT_CLOUD);
            perceptionVisualizerPanel.addVisualizer(d435ColoredPointCloudVisualizer);

            RDXROS2OusterPointCloudVisualizer ousterPointCloudVisualizer = new RDXROS2OusterPointCloudVisualizer("Ouster Point Cloud",
                                                                                                                 PubSubImplementation.FAST_RTPS,
                                                                                                                 PerceptionAPI.OUSTER_DEPTH_IMAGE);

            perceptionVisualizerPanel.addVisualizer(ousterPointCloudVisualizer);

            RDXROS2RigidBodyPoseVisualizer mocapPoseVisualizer = new RDXROS2RigidBodyPoseVisualizer("Mocap Pose",
                                                                                                    PubSubImplementation.FAST_RTPS,
                                                                                                    PerceptionAPI.MOCAP_RIGID_BODY);
            perceptionVisualizerPanel.addVisualizer(mocapPoseVisualizer);


            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            buildingConstructor = new RDXBuildingConstructor(baseUI.getPrimary3DPanel());

            perceptionLoggingPanel = new RDXPerceptionDataLoggerPanel("Perception Logger", logger);
            baseUI.getImGuiPanelManager().addPanel(perceptionLoggingPanel);

            perceptionLogLoaderPanel = new RDXPerceptionDataLoaderPanel(perceptionDataLoader);
            baseUI.getImGuiPanelManager().addPanel(perceptionLogLoaderPanel);

            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(buildingConstructor.getPanelName(), buildingConstructor::renderImGuiWidgets);

            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);

            remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper);
            remotePerceptionUI.setBlackflyLensProperties(SensorHeadParameters.BENCHTOP_BLACKFLY_LENS_COMBO);

            environmentBuilder.create();
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            buildingConstructor.create();
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getRealRenderables, RDXSceneLevel.MODEL);

            perceptionVisualizerPanel.create();
         }

         @Override
         public void render()
         {
            perceptionVisualizerPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            environmentBuilder.destroy();
            perceptionLoggingPanel.destroy();
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
            ros2Node.destroy();
         }
      });
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new RDXPerceptionUI();
   }
}

