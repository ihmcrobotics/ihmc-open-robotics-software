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
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.ui.graphics.ros2.*;
import us.ihmc.rdx.ui.graphics.RDXGeneralToolsPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import java.net.URISyntaxException;

public class RDXPerceptionUI
{
   private final PerceptionDataLoader perceptionDataLoader = new PerceptionDataLoader();

   private RDXBaseUI baseUI;
   private RDXGeneralToolsPanel globalVisualizersUI;

   private RDXPerceptionDataLoaderPanel perceptionLogLoaderPanel;
   private RDXPerceptionDataLoggerPanel perceptionLoggingPanel;
   private PerceptionDataLogger logger;

   private RDXEnvironmentBuilder environmentBuilder;
   private RDXRemotePerceptionUI remotePerceptionUI;

   public RDXPerceptionUI()
   {
      logger = new PerceptionDataLogger();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "perception_ui_node");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      globalVisualizersUI = new RDXGeneralToolsPanel(baseUI);
      baseUI = new RDXBaseUI("Perception UI");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            globalVisualizersUI.addVisualizer(new RDXROS2FramePlanarRegionsVisualizer("Rapid Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS));

            globalVisualizersUI.addVisualizer(new RDXROS2PlanarRegionsVisualizer("Rapid Regions",
                                                                                 ros2Node,
                                                                                 PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("Ouster Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.OUSTER_DEPTH_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("L515 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.L515_COLOR_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("L515 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.L515_DEPTH_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("D455 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D455_COLOR_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("D455 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D455_DEPTH_IMAGE));

            RDXROS2ColoredPointCloudVisualizer d455ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("D455 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.D455_COLOR_IMAGE);
            d455ColoredDepthVisualizer.setSubscribed(true);
            d455ColoredDepthVisualizer.setActive(true);
            globalVisualizersUI.addVisualizer(d455ColoredDepthVisualizer);

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("D455 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D455_COLOR_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("D455 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D455_DEPTH_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("D435 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D435_COLOR_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("D435 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                PerceptionAPI.D435_DEPTH_IMAGE));

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

            perceptionLoggingPanel = new RDXPerceptionDataLoggerPanel("Perception Logger", logger);
            baseUI.getImGuiPanelManager().addPanel(perceptionLoggingPanel);

            perceptionLogLoaderPanel = new RDXPerceptionDataLoaderPanel(perceptionDataLoader);
            baseUI.getImGuiPanelManager().addPanel(perceptionLogLoaderPanel);

            baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);

            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI);

            remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper);
            remotePerceptionUI.setBlackflyLensProperties(SensorHeadParameters.BENCHTOP_BLACKFLY_LENS_COMBO);

            environmentBuilder.create();
            environmentBuilder.loadEnvironment("FlatGround.json");

            globalVisualizersUI.create();
         }

         @Override
         public void render()
         {
            globalVisualizersUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            perceptionLoggingPanel.destroy();
            globalVisualizersUI.destroy();
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

