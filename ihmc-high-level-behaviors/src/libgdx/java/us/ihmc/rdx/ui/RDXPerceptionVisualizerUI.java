package us.ihmc.rdx.ui;

import com.badlogic.gdx.math.Matrix4;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.behaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.behaviors.tools.perception.PeriodicPlanarRegionPublisher;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXBuildingConstructor;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.objects.RDXL515SensorObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXOusterSensorObject;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.graphics.live.*;
import us.ihmc.rdx.ui.tools.RDXTransformTuner;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;

import java.net.URISyntaxException;

public class RDXPerceptionVisualizerUI
{
   private final PlanarRegionSLAMMapper realsensePlanarRegionSLAM = new PlanarRegionSLAMMapper();
   private PlanarRegionsList regionsUpdate = new PlanarRegionsList();
   //    private final RDXHighLevelDepthSensorSimulator simulatedDepthSensor;

   private final RDXBaseUI baseUI;
   private final ImGuiMapSenseConfigurationPanel mapsenseConfigurationUI;
   private final RDXGlobalVisualizersPanel globalVisualizersUI;
   private final RDXEnvironmentBuilder environmentBuilder;

   private final RDXBuildingConstructor buildingConstructor;

   private final RigidBodyTransform depthSensorTransform = new RigidBodyTransform();
   private final Matrix4 gdxSensorTransform = new Matrix4();
   private RDXTransformTuner l515TransformTuner;

   private final boolean LOW_RESOLUTION_SENSORS = false;
   private RDXHighLevelDepthSensorSimulator steppingL515Simulator;
   private RDXL515SensorObject l515Model;
   private RDXOusterSensorObject ousterModel;

   private RDXHighLevelDepthSensorSimulator ousterLidar;

   public RDXPerceptionVisualizerUI()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
      RosMainNode ros1Node = new RosMainNode(NetworkParameters.getROSURI(), "PerceptionVisualizer");

      baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", "Perception Visualizer");
      mapsenseConfigurationUI = new ImGuiMapSenseConfigurationPanel(ros1Node, ros2Node);

      globalVisualizersUI = new RDXGlobalVisualizersPanel();

      globalVisualizersUI.addVisualizer(new RDXROS2PlanarRegionsVisualizer("Lidar REA planar regions", ros2Node, ROS2Tools.LIDAR_REA_REGIONS));
      RDXROS1PlanarRegionsVisualizer mapsenseRegionsVisualizer = new RDXROS1PlanarRegionsVisualizer("MapSense Planar Regions",
                                                                                                    ros2Node,
                                                                                                    RosTools.MAPSENSE_REGIONS);
      globalVisualizersUI.addVisualizer(mapsenseRegionsVisualizer);

      globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("L515 Color Video", RosTools.L515_VIDEO));
      globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("L515 Depth Video", RosTools.L515_DEPTH));
      globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("L515 Compressed Video", RosTools.L515_COMPRESSED_VIDEO));

      globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("D435 Compressed Color", RosTools.D435_VIDEO_COMPRESSED));
      globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("D435 Color", RosTools.D435_VIDEO));

      globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("ZED Left Color", RosTools.ZED2_LEFT_EYE_VIDEO));
      globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("ZED Right Color", RosTools.ZED2_RIGHT_EYE_VIDEO));

      globalVisualizersUI.addVisualizer(new RDXROS1PointCloudVisualizer("Head Ouster", RosTools.OUSTER_POINT_CLOUD));
      globalVisualizersUI.addVisualizer(new RDXROS2PointCloudVisualizer("MultiSense lidar scan", ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN));

      globalVisualizersUI.addVisualizer(new RDXROS1OdometryVisualizer("SLAM Poses", RosTools.SLAM_POSE));
      globalVisualizersUI.addVisualizer(new RDXROS1OdometryVisualizer("Internal Sensor Pose", "/atlas/sensors/chest_l515/pose"));

      globalVisualizersUI.addVisualizer(new RDXROS1PointCloudVisualizer("Semantic Target Cloud", RosTools.SEMANTIC_TARGET_CLOUD));
      globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("Semantic Mask", RosTools.SEMANTIC_MASK));
      globalVisualizersUI.addVisualizer(new RDXROS1OdometryVisualizer("Semantic Target Pose", RosTools.SEMANTIC_TARGET_POSE));

      globalVisualizersUI.addVisualizer(new RDXROS1PointCloudVisualizer("Ouster Point Cloud", RosTools.OUSTER_POINT_CLOUD));

      //        simulatedDepthSensor = createSimulatedDepthSensor(ros1Node);
      //        simulatedDepthSensor.setSensorFrameToWorldTransform(depthSensorTransform);
      //        simulatedDepthSensor.setRenderPointCloudDirectly(true);
      //        simulatedDepthSensor.setDebugCoordinateFrame(true);

      environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
      buildingConstructor = new RDXBuildingConstructor(baseUI.getPrimary3DPanel());

      baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
      baseUI.getImGuiPanelManager().addPanel(mapsenseRegionsVisualizer.getLoggingPanel());
      //        baseUI.getImGuiDockingSetup().addWindow(simulatedDepthSensor.getWindowName(), simulatedDepthSensor::renderImGuiWindow);
      baseUI.getImGuiPanelManager().addPanel(mapsenseConfigurationUI.getWindowName(), mapsenseConfigurationUI::render);
      baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
      baseUI.getImGuiPanelManager().addPanel(buildingConstructor.getPanelName(), buildingConstructor::renderImGuiWidgets);

      //        l515TransformTuner = new RDXTransformTuner(robotModel.getSensorInformation().getSteppingCameraTransform());
      //        baseUI.getImGuiPanelManager().addPanel("L515 Transform Tuner", l515TransformTuner::renderImGuiWidgets);

      ousterLidar = createOusterLidar(ros1Node, ros2Node);
      ousterLidar.setupForROS1PointCloud(ros1Node, RosTools.OUSTER_POINT_CLOUD);
      ousterLidar.setSensorEnabled(true);
      ousterLidar.setRenderPointCloudDirectly(true);
      ousterLidar.setSensorFrameToWorldTransform(depthSensorTransform);

      ros1Node.execute();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI, RDXSceneLevel.VIRTUAL);

            //                simulatedDepthSensor.create();
            //                baseUI.getSceneManager().addRenderableProvider(simulatedDepthSensor, RDXSceneLevel.VIRTUAL);

            baseUI.getPrimaryScene().addRenderableProvider(ousterLidar::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(ousterLidar);

            environmentBuilder.create();
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            buildingConstructor.create();
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getRealRenderables, RDXSceneLevel.MODEL);

            globalVisualizersUI.create();
            //                l515Model = new RDXL515SensorObject();
            //                environmentBuilderUI.getModelInput().addInstance(l515Model);

            ousterModel = new RDXOusterSensorObject();
            baseUI.getPrimaryScene().addRenderableProvider(ousterModel::getRealRenderables);
            environmentBuilder.addObject(ousterModel);

            //                environmentBuilderUI.getModelInput().addInstance(ousterModel);

         }

         @Override
         public void render()
         {
            //                gdxSensorTransform.set(l515Model.getRealisticModelInstance().transform);
            //                LibGDXTools.toEuclid(gdxSensorTransform, depthSensorTransform);
            //                simulatedDepthSensor.render(baseUI.getSceneManager());

            gdxSensorTransform.set(ousterModel.getRealisticModelInstance().transform);
            LibGDXTools.toEuclid(gdxSensorTransform, depthSensorTransform);

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

   public RDXHighLevelDepthSensorSimulator createOusterLidar(RosNodeInterface ros1Node, ROS2NodeInterface ros2Node)
   {
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = RDXSimulatedSensorFactory.createOusterLidar(null, () -> 0L);
      highLevelDepthSensorSimulator.setupForROS1Depth(ros1Node, RosTools.MAPSENSE_DEPTH_IMAGE, RosTools.MAPSENSE_DEPTH_CAMERA_INFO);
      highLevelDepthSensorSimulator.setupForROS1Color(ros1Node, RosTools.L515_VIDEO, RosTools.L515_COLOR_CAMERA_INFO);
      highLevelDepthSensorSimulator.setupForROS2PointCloud(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
      return highLevelDepthSensorSimulator;
   }

   private RDXHighLevelDepthSensorSimulator createSimulatedL515(RosNodeInterface ros1Node)
   {
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = RDXSimulatedSensorFactory.createRealsenseL515(null, () -> 0L);
      highLevelDepthSensorSimulator.setupForROS1Depth(ros1Node, RosTools.MAPSENSE_DEPTH_IMAGE, RosTools.MAPSENSE_DEPTH_CAMERA_INFO);
      highLevelDepthSensorSimulator.setupForROS1Color(ros1Node, RosTools.L515_VIDEO, RosTools.L515_COLOR_CAMERA_INFO);
      return highLevelDepthSensorSimulator;
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new RDXPerceptionVisualizerUI();
   }


}

