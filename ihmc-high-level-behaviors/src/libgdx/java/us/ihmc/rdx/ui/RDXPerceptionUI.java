package us.ihmc.rdx.ui;

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
   private RDXRemotePerceptionUI rapidRegionsExtractionUI;

   public RDXPerceptionUI()
   {
      logger = new PerceptionDataLogger();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "perception_ui_node");

      globalVisualizersUI = new RDXGlobalVisualizersPanel();
      baseUI = new RDXBaseUI("Perception UI");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            globalVisualizersUI.addVisualizer(new RDXROS2PlanarRegionsVisualizer("Rapid Regions",
                                                                                 ros2Node,
                                                                                 ROS2Tools.PERSPECTIVE_RAPID_REGIONS));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("Ouster Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                ROS2Tools.OUSTER_DEPTH_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("L515 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                ROS2Tools.L515_COLOR_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("L515 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                ROS2Tools.L515_DEPTH_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("D435 Color",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                ROS2Tools.D435_COLOR_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("D435 Depth",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                ROS2Tools.D435_DEPTH_IMAGE));

            globalVisualizersUI.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED2 Color Stereo",
                                                                                PubSubImplementation.FAST_RTPS,
                                                                                ROS2Tools.ZED2_STEREO_COLOR));

            RDXROS2BigVideoVisualizer blackflyRightVideoVisualizer = new RDXROS2BigVideoVisualizer("IHMC Blackfly Right",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT));
            blackflyRightVideoVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(blackflyRightVideoVisualizer);

            RDXROS2ColoredPointCloudVisualizer l515ColoredDepthVisualizer = new RDXROS2ColoredPointCloudVisualizer("L515 Colored Depth",
                                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                                   ROS2Tools.L515_DEPTH_IMAGE,
                                                                                                                   ROS2Tools.L515_COLOR_IMAGE);
            l515ColoredDepthVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(l515ColoredDepthVisualizer);

            globalVisualizersUI.addVisualizer(new RDXROS2ColoredPointCloudVisualizer("D435 Colored Depth",
                                                                                     PubSubImplementation.FAST_RTPS,
                                                                                     ROS2Tools.D435_DEPTH_IMAGE,
                                                                                     ROS2Tools.D435_COLOR_IMAGE));

            RDXROS2PointCloudVisualizer l515ColoredPointCloudVisualizer = new RDXROS2PointCloudVisualizer("L515 Colored Point Cloud",
                                                                                                          ros2Node,
                                                                                                          ROS2Tools.FUSED_SENSOR_HEAD_POINT_CLOUD);
            l515ColoredPointCloudVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(l515ColoredPointCloudVisualizer);

            RDXROS2PointCloudVisualizer d435ColoredPointCloudVisualizer = new RDXROS2PointCloudVisualizer("D435 Colored Point Cloud",
                                                                                                          ros2Node,
                                                                                                          ROS2Tools.D435_COLORED_POINT_CLOUD);
            d435ColoredPointCloudVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(d435ColoredPointCloudVisualizer);

            RDXROS2OusterPointCloudVisualizer ousterPointCloudVisualizer = new RDXROS2OusterPointCloudVisualizer("Ouster Point Cloud",
                                                                                                                 PubSubImplementation.FAST_RTPS,
                                                                                                                 ROS2Tools.OUSTER_DEPTH_IMAGE);
            ousterPointCloudVisualizer.setSubscribed(true);
            globalVisualizersUI.addVisualizer(ousterPointCloudVisualizer);


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
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI, RDXSceneLevel.VIRTUAL);

            rapidRegionsExtractionUI = new RDXRemotePerceptionUI(new ROS2Helper(ros2Node));
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsExtractionUI.getPanel());

            environmentBuilder.create();
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            buildingConstructor.create();
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getRealRenderables, RDXSceneLevel.MODEL);

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

