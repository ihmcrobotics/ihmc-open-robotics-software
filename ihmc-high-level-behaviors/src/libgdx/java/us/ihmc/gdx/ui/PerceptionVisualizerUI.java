package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Matrix4;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.bytedeco.hdf5.H5File;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.behaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.behaviors.tools.perception.PeriodicPlanarRegionPublisher;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.logging.PerceptionLoggerPanel;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXBuildingConstructor;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.objects.GDXOusterSensorObject;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.simulation.sensors.GDXSimulatedSensorFactory;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.graphics.live.*;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HDF5Tools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.Activator;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;

import java.net.URISyntaxException;

import static org.bytedeco.hdf5.global.hdf5.H5F_ACC_RDONLY;
import static org.bytedeco.opencv.global.opencv_core.convertScaleAbs;
import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKey;

public class PerceptionVisualizerUI
{
   private PerceptionLoggerPanel loggerPanel;

   private PlanarRegionSLAMMapper realsensePlanarRegionSLAM = new PlanarRegionSLAMMapper();
   private PlanarRegionsList regionsUpdate = new PlanarRegionsList();

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

            globalVisualizersUI.addVisualizer(new GDXROS2PlanarRegionsVisualizer("Lidar REA planar regions", ros2Node, ROS2Tools.LIDAR_REA_REGIONS));

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
            if(nativesLoadedActivator.poll())
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

   public void regionsCallback(PlanarRegionsListMessage regions)
   {
      this.regionsUpdate = PlanarRegionMessageConverter.convertToPlanarRegionsList(regions);
   }

   public void setupPlanarRegionSLAM(ROS2Node ros2Node)
   {
      new IHMCROS2Callback<>(ros2Node, ROS2Tools.MAPSENSE_REGIONS, this::regionsCallback);
      new PeriodicPlanarRegionPublisher(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, 0.001f, () -> realsensePlanarRegionSLAM.update(regionsUpdate)).start();
   }

   public GDXHighLevelDepthSensorSimulator createOusterLidar(RosNodeInterface ros1Node, ROS2NodeInterface ros2Node)
   {
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = GDXSimulatedSensorFactory.createOusterLidar(null, () -> 0L);
      highLevelDepthSensorSimulator.setupForROS1Depth(ros1Node, RosTools.MAPSENSE_DEPTH_IMAGE, RosTools.MAPSENSE_DEPTH_CAMERA_INFO);
      highLevelDepthSensorSimulator.setupForROS1Color(ros1Node, RosTools.L515_VIDEO, RosTools.L515_COLOR_CAMERA_INFO);
      highLevelDepthSensorSimulator.setupForROS2PointCloud(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
      return highLevelDepthSensorSimulator;
   }

   private GDXHighLevelDepthSensorSimulator createSimulatedL515(RosNodeInterface ros1Node)
   {
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = GDXSimulatedSensorFactory.createRealsenseL515(null, () -> 0L);
      highLevelDepthSensorSimulator.setupForROS1Depth(ros1Node, RosTools.MAPSENSE_DEPTH_IMAGE, RosTools.MAPSENSE_DEPTH_CAMERA_INFO);
      highLevelDepthSensorSimulator.setupForROS1Color(ros1Node, RosTools.L515_VIDEO, RosTools.L515_COLOR_CAMERA_INFO);
      return highLevelDepthSensorSimulator;
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new PerceptionVisualizerUI();
   }
}

