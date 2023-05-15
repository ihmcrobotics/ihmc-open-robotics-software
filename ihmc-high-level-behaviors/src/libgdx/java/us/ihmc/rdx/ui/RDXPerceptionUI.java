package us.ihmc.rdx.ui;

import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.logging.RDXPerceptionDataLoaderPanel;
import us.ihmc.rdx.logging.RDXPerceptionDataLoggerPanel;
import us.ihmc.rdx.perception.RDXRemotePerceptionUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXBuildingConstructor;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.ui.graphics.ros2.*;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.net.URISyntaxException;

public class RDXPerceptionUI
{
   private final PerceptionDataLoader perceptionDataLoader = new PerceptionDataLoader();

   private RDXBaseUI baseUI;
   private RDXGlobalVisualizersPanel globalVisualizersUI;

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

      globalVisualizersUI = new RDXGlobalVisualizersPanel();
      baseUI = new RDXBaseUI("Perception UI");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            addPlanarRegionsVisualizer("Rapid Regions", PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
            addPlanarRegionsVisualizer("Rapid Regions", PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);

            addImageMessageVisualizer("Ouster Depth", PerceptionAPI.OUSTER_DEPTH_IMAGE);
            addImageMessageVisualizer("D455 Color", PerceptionAPI.D455_COLOR_IMAGE);
            addImageMessageVisualizer("D455 Depth", PerceptionAPI.D455_DEPTH_IMAGE);
            addImageMessageVisualizer("D435 Color", PerceptionAPI.D435_COLOR_IMAGE);
            addImageMessageVisualizer("D435 Depth", PerceptionAPI.D435_DEPTH_IMAGE);
            addImageMessageVisualizer("ZED2 Color Stereo", PerceptionAPI.ZED2_STEREO_COLOR);
            addImageMessageVisualizer("Blackfly Fujinon Right", PerceptionAPI.BLACKFLY_FISHEYE_COLOR_IMAGE.get(RobotSide.RIGHT));

            addColoredPointCloudVisualizer("L515 Colored Depth", PerceptionAPI.L515_DEPTH_IMAGE, PerceptionAPI.L515_COLOR_IMAGE);
            addColoredPointCloudVisualizer("D435 Colored Depth", PerceptionAPI.D435_DEPTH_IMAGE, PerceptionAPI.D435_COLOR_IMAGE);
            addColoredPointCloudVisualizer("D455 Colored Depth", PerceptionAPI.D455_DEPTH_IMAGE, PerceptionAPI.D455_COLOR_IMAGE);


            RDXROS2ColoredPointCloudVisualizer ousterFisheyeColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer(
                  "Ouster Fisheye Colored Point Cloud",
                  PubSubImplementation.FAST_RTPS,
                  PerceptionAPI.OUSTER_DEPTH_IMAGE,
                  PerceptionAPI.BLACKFLY_FISHEYE_COLOR_IMAGE.get(RobotSide.RIGHT));
            ousterFisheyeColoredPointCloudVisualizer.setSubscribed(true);
            ousterFisheyeColoredPointCloudVisualizer.setActive(true);
            ousterFisheyeColoredPointCloudVisualizer.setPointSize(0.004f);
            ousterFisheyeColoredPointCloudVisualizer.setLevelOfColorDetail(2);
            globalVisualizersUI.addVisualizer(ousterFisheyeColoredPointCloudVisualizer);

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

            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI);

            remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper);
            baseUI.getImGuiPanelManager().addPanel(remotePerceptionUI.getPanel());

            environmentBuilder.create();
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            buildingConstructor.create();
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getRealRenderables, RDXSceneLevel.MODEL);

            globalVisualizersUI.create();
         }

         private void addImageMessageVisualizer(String name, ROS2Topic<ImageMessage> topic)
         {
            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer(name, PubSubImplementation.FAST_RTPS, topic));
         }

         private void addPlanarRegionsVisualizer(String name, ROS2Topic<PlanarRegionsListMessage> topic)
         {
            globalVisualizersUI.addVisualizer(new RDXROS2PlanarRegionsVisualizer(name, ros2Node, topic));
         }

         private void addColoredPointCloudVisualizer(String topic, ROS2Topic<ImageMessage> colorTopic, ROS2Topic<ImageMessage> depthTopic)
         {
            RDXROS2ColoredPointCloudVisualizer l515ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("L515 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   depthTopic,
                                                                                                                   colorTopic);
            l515ColoredDepthVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(l515ColoredDepthVisualizer);
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
      new RDXPerceptionUI();
   }
}

