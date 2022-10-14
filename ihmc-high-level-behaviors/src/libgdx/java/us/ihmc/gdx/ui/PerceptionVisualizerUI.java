package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.Color;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.logging.PerceptionLoggerPanel;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXBuildingConstructor;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.ui.graphics.live.*;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Activator;

import java.net.URISyntaxException;

public class PerceptionVisualizerUI
{
   private PerceptionLoggerPanel loggerPanel;

   private GDXImGuiBasedUI baseUI;
   private ImGuiGDXGlobalVisualizersPanel globalVisualizersUI;

   private GDXEnvironmentBuilder environmentBuilder;
   private GDXBuildingConstructor buildingConstructor;

   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(200000, Point3D32::new);
   private GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();

   private GDXROS2BigVideoVisualizer blackflyRightVisualizer;
   private GDXROS2VideoVisualizer videoVisualizer;

   private Activator nativesLoadedActivator;

   public PerceptionVisualizerUI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);

      globalVisualizersUI = new ImGuiGDXGlobalVisualizersPanel();
      baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", "Perception Visualizer");

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {

            loggerPanel = new PerceptionLoggerPanel("Perception Logger");
            baseUI.getImGuiPanelManager().addPanel(loggerPanel);

            globalVisualizersUI.addVisualizer(new GDXROS2PlanarRegionsVisualizer("Mapsense Regions", ros2Node, ROS2Tools.MAPSENSE_REGIONS));

            globalVisualizersUI.addVisualizer(new GDXROS2BigDepthVideoVisualizer("L515 Depth",
                                                                                 DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                 ROS2Tools.L515_DEPTH));

            globalVisualizersUI.addVisualizer(new GDXROS2VideoVisualizer("D435 Video", ros2Node, ROS2Tools.VIDEO, ROS2VideoFormat.JPEGYUVI420));

            globalVisualizersUI.addVisualizer(new GDXROS2BigVideoVisualizer("IHMC Blackfly Right",
                                                                            DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                            ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT)));

            globalVisualizersUI.addVisualizer(new GDXROS2PointCloudVisualizer("L515 Point Cloud",
                                                                              ros2Node,
                                                                              ROS2Tools.IHMC_ROOT.withTypeName(StereoVisionPointCloudMessage.class),
                                                                              1024 * 768,
                                                                              1));
            int pointsPerSegment = 786432;
            int numberOfSegments = 1;
            globalVisualizersUI.addVisualizer(new GDXROS2PointCloudVisualizer("L515 Colored Point Cloud",
                                                                              ros2Node,
                                                                              ROS2Tools.FUSED_SENSOR_HEAD_POINT_CLOUD,
                                                                              pointsPerSegment,
                                                                              numberOfSegments));
            pointsPerSegment = 407040;
            numberOfSegments = 1;
            globalVisualizersUI.addVisualizer(new GDXROS2PointCloudVisualizer("D435 Colored Point Cloud",
                                                                              ros2Node,
                                                                              ROS2Tools.D435_COLORED_POINT_CLOUD,
                                                                              pointsPerSegment,
                                                                              numberOfSegments));
            int os0128Multiplier = 2;
            pointsPerSegment = 131072 * os0128Multiplier;
            numberOfSegments = 1;
            globalVisualizersUI.addVisualizer(new GDXROS2PointCloudVisualizer("Ouster Point Cloud",
                                                                              ros2Node,
                                                                              ROS2Tools.OUSTER_POINT_CLOUD,
                                                                              pointsPerSegment,
                                                                              numberOfSegments));
            videoVisualizer = new GDXROS2VideoVisualizer("Primary Video", ros2Node, ROS2Tools.VIDEO, ROS2VideoFormat.JPEGYUVI420);
            globalVisualizersUI.addVisualizer(videoVisualizer);

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            buildingConstructor = new GDXBuildingConstructor(baseUI.getPrimary3DPanel());

            baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(buildingConstructor.getPanelName(), buildingConstructor::renderImGuiWidgets);

            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI, GDXSceneLevel.VIRTUAL);

            environmentBuilder.create();
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            buildingConstructor.create();
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getRealRenderables, GDXSceneLevel.MODEL);

            globalVisualizersUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
            pointCloudRenderer.create(200000);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               globalVisualizersUI.update();

               pointCloudRenderer.setPointsToRender(pointsToRender, Color.BLUE);
               if (!pointsToRender.isEmpty())
               {
                  pointCloudRenderer.updateMesh();
               }

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
      new PerceptionVisualizerUI();
   }
}

