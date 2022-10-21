package us.ihmc.rdx.ui;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.live.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Activator;

import java.net.URISyntaxException;

public class RDXPerceptionVisualizerUI
{
   private PerceptionLoggerPanel loggerPanel;

   private GDXImGuiBasedUI baseUI;
   private ImGuiGDXGlobalVisualizersPanel globalVisualizersUI;

   private GDXEnvironmentBuilder environmentBuilder;
   private GDXBuildingConstructor buildingConstructor;

   private GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();

   private RDXROS2BigVideoVisualizer blackflyRightVisualizer;
   private RDXROS2VideoVisualizer videoVisualizer;

   private Activator nativesLoadedActivator;

   public RDXPerceptionVisualizerUI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);

      globalVisualizersUI = new ImGuiGDXGlobalVisualizersPanel();
      baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", "Perception Visualizer");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            pointCloudRenderer.create(400000);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);

            loggerPanel = new PerceptionLoggerPanel("Perception Logger");
            loggerPanel.setPointCloudRenderer(pointCloudRenderer);
            baseUI.getImGuiPanelManager().addPanel(loggerPanel);

            globalVisualizersUI.addVisualizer(new RDXROS2PlanarRegionsVisualizer("Mapsense Regions", ros2Node, ROS2Tools.MAPSENSE_REGIONS));

            globalVisualizersUI.addVisualizer(new RDXROS2BigDepthVideoVisualizer("L515 Depth",
                                                                                 DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                 ROS2Tools.L515_DEPTH));

            globalVisualizersUI.addVisualizer(new RDXROS2VideoVisualizer("D435 Video", ros2Node, ROS2Tools.VIDEO, ROS2VideoFormat.JPEGYUVI420));

            globalVisualizersUI.addVisualizer(new RDXROS2BigVideoVisualizer("IHMC Blackfly Right",
                                                                            DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                            ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT)));

            globalVisualizersUI.addVisualizer(new RDXROS2PointCloudVisualizer("L515 Point Cloud",
                                                                              ros2Node,
                                                                              ROS2Tools.IHMC_ROOT.withTypeName(StereoVisionPointCloudMessage.class),
                                                                              1024 * 768,
                                                                              1));
            int pointsPerSegment = 786432;
            int numberOfSegments = 1;
            globalVisualizersUI.addVisualizer(new RDXROS2PointCloudVisualizer("L515 Colored Point Cloud",
                                                                              ros2Node,
                                                                              ROS2Tools.FUSED_SENSOR_HEAD_POINT_CLOUD,
                                                                              pointsPerSegment,
                                                                              numberOfSegments));
            pointsPerSegment = 407040;
            numberOfSegments = 1;
            globalVisualizersUI.addVisualizer(new RDXROS2PointCloudVisualizer("D435 Colored Point Cloud",
                                                                              ros2Node,
                                                                              ROS2Tools.D435_COLORED_POINT_CLOUD,
                                                                              pointsPerSegment,
                                                                              numberOfSegments));

            int os0128Multiplier = 1;
            pointsPerSegment = 131072 * os0128Multiplier;
            numberOfSegments = 1;
            globalVisualizersUI.addVisualizer(new RDXROS2PointCloudVisualizer("Ouster Point Cloud",
                                                                              ros2Node,
                                                                              ROS2Tools.OUSTER_POINT_CLOUD,
                                                                              pointsPerSegment,
                                                                              numberOfSegments));

            videoVisualizer = new RDXROS2VideoVisualizer("Primary Video", ros2Node, ROS2Tools.VIDEO, ROS2VideoFormat.JPEGYUVI420);
            globalVisualizersUI.addVisualizer(videoVisualizer);

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            buildingConstructor = new GDXBuildingConstructor(baseUI.getPrimary3DPanel());

            baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(buildingConstructor.getPanelName(), buildingConstructor::renderImGuiWidgets);

            baseUI.create();
<<<<<<<HEAD:
         ihmc - high - level - behaviors / src / libgdx / java / us / ihmc / gdx / ui / GDXPerceptionVisualizerUI.java
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI, GDXSceneLevel.VIRTUAL);

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
            if (nativesLoadedActivator.poll())
            {
               globalVisualizersUI.update();

               //               pointCloudRenderer.updateMeshFastest();

               baseUI.renderBeforeOnScreenUI();
               baseUI.renderEnd();
            }
         }

         @Override
         public void dispose()
         {
            environmentBuilder.destroy();
            globalVisualizersUI.destroy();
            baseUI.dispose();
            ros2Node.destroy();
         }
      });
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new RDXPerceptionVisualizerUI();
   }
}

