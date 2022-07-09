package us.ihmc.gdx.ui;

import com.badlogic.gdx.math.Matrix4;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.behaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.behaviors.tools.perception.PeriodicPlanarRegionPublisher;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXBuildingConstructor;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.objects.GDXL515SensorObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXOusterSensorObject;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.simulation.sensors.GDXSimulatedSensorFactory;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.graphics.live.*;
import us.ihmc.gdx.ui.tools.GDXTransformTuner;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;

import java.net.URISyntaxException;

public class GDXPerceptionVisualizerUI
{
   private final PlanarRegionSLAMMapper realsensePlanarRegionSLAM = new PlanarRegionSLAMMapper();
   private PlanarRegionsList regionsUpdate = new PlanarRegionsList();
   //    private final GDXHighLevelDepthSensorSimulator simulatedDepthSensor;

   private final GDXImGuiBasedUI baseUI;
   private final ImGuiMapSenseConfigurationPanel mapsenseConfigurationUI;
   private final ImGuiGDXGlobalVisualizersPanel globalVisualizersUI;
   private final GDXEnvironmentBuilder environmentBuilder;

   private final GDXBuildingConstructor buildingConstructor;

   private final RigidBodyTransform depthSensorTransform = new RigidBodyTransform();
   private final Matrix4 gdxSensorTransform = new Matrix4();
   private GDXTransformTuner l515TransformTuner;

   private final boolean LOW_RESOLUTION_SENSORS = false;
   private GDXHighLevelDepthSensorSimulator steppingL515Simulator;
   private GDXL515SensorObject l515Model;
   private GDXOusterSensorObject ousterModel;

   private GDXHighLevelDepthSensorSimulator ousterLidar;

   public GDXPerceptionVisualizerUI()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
      RosMainNode ros1Node = new RosMainNode(NetworkParameters.getROSURI(), "PerceptionVisualizer");

      baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", "Perception Visualizer");
      mapsenseConfigurationUI = new ImGuiMapSenseConfigurationPanel(ros1Node, ros2Node);

      globalVisualizersUI = new ImGuiGDXGlobalVisualizersPanel();

      globalVisualizersUI.addVisualizer(new GDXROS2PlanarRegionsVisualizer("Lidar REA planar regions", ros2Node, ROS2Tools.LIDAR_REA_REGIONS));
      GDXROS1PlanarRegionsVisualizer mapsenseRegionsVisualizer = new GDXROS1PlanarRegionsVisualizer("MapSense Planar Regions",
                                                                                                    ros2Node,
                                                                                                    RosTools.MAPSENSE_REGIONS);
      globalVisualizersUI.addVisualizer(mapsenseRegionsVisualizer);

      globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Color Video", RosTools.L515_VIDEO));
      globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Depth Video", RosTools.L515_DEPTH));
      globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Compressed Video", RosTools.L515_COMPRESSED_VIDEO));

      globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("D435 Compressed Color", RosTools.D435_VIDEO_COMPRESSED));
      globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("D435 Color", RosTools.D435_VIDEO));

      globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("ZED Left Color", RosTools.ZED2_LEFT_EYE_VIDEO));
      globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("ZED Right Color", RosTools.ZED2_RIGHT_EYE_VIDEO));

      globalVisualizersUI.addVisualizer(new GDXROS1PointCloudVisualizer("Head Ouster", RosTools.OUSTER_POINT_CLOUD));
      globalVisualizersUI.addVisualizer(new GDXROS2PointCloudVisualizer("MultiSense lidar scan", ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN));

      globalVisualizersUI.addVisualizer(new GDXROS1OdometryVisualizer("SLAM Poses", RosTools.SLAM_POSE));
      globalVisualizersUI.addVisualizer(new GDXROS1OdometryVisualizer("Internal Sensor Pose", "/atlas/sensors/chest_l515/pose"));

      globalVisualizersUI.addVisualizer(new GDXROS1PointCloudVisualizer("Semantic Target Cloud", RosTools.SEMANTIC_TARGET_CLOUD));
      globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("Semantic Mask", RosTools.SEMANTIC_MASK));
      globalVisualizersUI.addVisualizer(new GDXROS1OdometryVisualizer("Semantic Target Pose", RosTools.SEMANTIC_TARGET_POSE));

      globalVisualizersUI.addVisualizer(new GDXROS1PointCloudVisualizer("Ouster Point Cloud", RosTools.OUSTER_POINT_CLOUD));

      //        simulatedDepthSensor = createSimulatedDepthSensor(ros1Node);
      //        simulatedDepthSensor.setSensorFrameToWorldTransform(depthSensorTransform);
      //        simulatedDepthSensor.setRenderPointCloudDirectly(true);
      //        simulatedDepthSensor.setDebugCoordinateFrame(true);

      environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
      buildingConstructor = new GDXBuildingConstructor(baseUI.getPrimary3DPanel());

      baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
      baseUI.getImGuiPanelManager().addPanel(mapsenseRegionsVisualizer.getLoggingPanel());
      //        baseUI.getImGuiDockingSetup().addWindow(simulatedDepthSensor.getWindowName(), simulatedDepthSensor::renderImGuiWindow);
      baseUI.getImGuiPanelManager().addPanel(mapsenseConfigurationUI.getWindowName(), mapsenseConfigurationUI::render);
      baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
      baseUI.getImGuiPanelManager().addPanel(buildingConstructor.getPanelName(), buildingConstructor::renderImGuiWidgets);

      //        l515TransformTuner = new GDXTransformTuner(robotModel.getSensorInformation().getSteppingCameraTransform());
      //        baseUI.getImGuiPanelManager().addPanel("L515 Transform Tuner", l515TransformTuner::renderImGuiWidgets);

      ousterLidar = createOusterLidar(ros1Node, ros2Node);
      ousterLidar.setupForROS1PointCloud(ros1Node, RosTools.OUSTER_POINT_CLOUD);
      ousterLidar.setSensorEnabled(true);
      ousterLidar.setRenderPointCloudDirectly(true);
      ousterLidar.setSensorFrameToWorldTransform(depthSensorTransform);

      ros1Node.execute();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI, GDXSceneLevel.VIRTUAL);

            //                simulatedDepthSensor.create();
            //                baseUI.getSceneManager().addRenderableProvider(simulatedDepthSensor, GDXSceneLevel.VIRTUAL);

            baseUI.getPrimaryScene().addRenderableProvider(ousterLidar, GDXSceneLevel.VIRTUAL);
            baseUI.getImGuiPanelManager().addPanel(ousterLidar);

            environmentBuilder.create();
            baseUI.getPrimaryScene().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.getPrimaryScene().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            buildingConstructor.create();
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);

            globalVisualizersUI.create();
            //                l515Model = new GDXL515SensorObject();
            //                environmentBuilderUI.getModelInput().addInstance(l515Model);

            ousterModel = new GDXOusterSensorObject();
            baseUI.getPrimaryScene().addRenderableProvider(ousterModel::getRealRenderables);
            environmentBuilder.addObject(ousterModel);

            //                environmentBuilderUI.getModelInput().addInstance(ousterModel);

         }

         @Override
         public void render()
         {
            //                gdxSensorTransform.set(l515Model.getRealisticModelInstance().transform);
            //                GDXTools.toEuclid(gdxSensorTransform, depthSensorTransform);
            //                simulatedDepthSensor.render(baseUI.getSceneManager());

            gdxSensorTransform.set(ousterModel.getRealisticModelInstance().transform);
            GDXTools.toEuclid(gdxSensorTransform, depthSensorTransform);

            ousterLidar.render(baseUI.getPrimaryScene());

            globalVisualizersUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            //                simulatedDepthSensor.dispose();
            environmentBuilder.destroy();
            globalVisualizersUI.destroy();
            baseUI.dispose();
            ros2Node.destroy();
            ros1Node.shutdown();
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
      new GDXPerceptionVisualizerUI();
   }


}

