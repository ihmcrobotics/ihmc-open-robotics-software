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
   private static final String HDF5_FILENAME = "/home/quantum/Workspace/Data/Atlas_Logs/ROSBags/atlas_perception_run_1.h5";

   private PlanarRegionSLAMMapper realsensePlanarRegionSLAM = new PlanarRegionSLAMMapper();
   private PlanarRegionsList regionsUpdate = new PlanarRegionsList();

   private GDXImGuiBasedUI baseUI;
   private ImGuiGDXGlobalVisualizersPanel globalVisualizersUI;
   private GDXEnvironmentBuilder environmentBuilder;

   private GDXBuildingConstructor buildingConstructor;

   private RigidBodyTransform depthSensorTransform = new RigidBodyTransform();
   private Matrix4 gdxSensorTransform = new Matrix4();

   private GDXOusterSensorObject ousterModel;

   private GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private GDXROS2VideoVisualizer videoVisualizer;
   private GDXROS2BigVideoVisualizer blackflyRightVisualizer;

   private GDXHighLevelDepthSensorSimulator ousterLidar;

   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(200000, Point3D32::new);
   private Mat depthMap;

   private Activator nativesLoadedActivator;

   private int frameIndex = 0;

   public PerceptionVisualizerUI()
   {

      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
      //      RosMainNode ros1Node = new RosMainNode(NetworkParameters.getROSURI(), "PerceptionVisualizer");

      globalVisualizersUI = new ImGuiGDXGlobalVisualizersPanel();
      baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", "Perception Visualizer");

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {

            //            mapsenseConfigurationUI = new ImGuiMapSenseConfigurationPanel(ros1Node, ros2Node);

            globalVisualizersUI.addVisualizer(new GDXROS2PlanarRegionsVisualizer("Lidar REA planar regions", ros2Node, ROS2Tools.LIDAR_REA_REGIONS));

            globalVisualizersUI.addVisualizer(new GDXROS2BigDepthVideoVisualizer("L515 Depth",
                                                                                 DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                 ROS2Tools.L515_DEPTH));
            blackflyRightVisualizer = new GDXROS2BigVideoVisualizer("IHMC Blackfly Right",
                                                                    DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                    ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT));
            globalVisualizersUI.addVisualizer(blackflyRightVisualizer);
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

            //            GDXROS1PlanarRegionsVisualizer mapsenseRegionsVisualizer = new GDXROS1PlanarRegionsVisualizer("MapSense Planar Regions",
            //                    ros2Node,
            //                    RosTools.MAPSENSE_REGIONS);
            //            globalVisualizersUI.addVisualizer(mapsenseRegionsVisualizer);

            //            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Color Video", RosTools.L515_VIDEO));
            //            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Depth Video", RosTools.L515_DEPTH));
            //            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Compressed Video", RosTools.L515_COMPRESSED_VIDEO));
            //
            //            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("D435 Compressed Color", RosTools.D435_VIDEO_COMPRESSED));
            //            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("D435 Color", RosTools.D435_VIDEO));
            //
            //            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("ZED Left Color", RosTools.ZED2_LEFT_EYE_VIDEO));
            //            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("ZED Right Color", RosTools.ZED2_RIGHT_EYE_VIDEO));
            //
            //            globalVisualizersUI.addVisualizer(new GDXROS1PointCloudVisualizer("Head Ouster", RosTools.OUSTER_POINT_CLOUD));
            //            globalVisualizersUI.addVisualizer(new GDXROS2PointCloudVisualizer("MultiSense lidar scan", ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN));
            //
            //            globalVisualizersUI.addVisualizer(new GDXROS1OdometryVisualizer("SLAM Poses", RosTools.SLAM_POSE));
            //            globalVisualizersUI.addVisualizer(new GDXROS1OdometryVisualizer("Internal Sensor Pose", "/atlas/sensors/chest_l515/pose"));
            //
            //            globalVisualizersUI.addVisualizer(new GDXROS1PointCloudVisualizer("Semantic Target Cloud", RosTools.SEMANTIC_TARGET_CLOUD));
            //            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("Semantic Mask", RosTools.SEMANTIC_MASK));
            //            globalVisualizersUI.addVisualizer(new GDXROS1OdometryVisualizer("Semantic Target Pose", RosTools.SEMANTIC_TARGET_POSE));
            //
            //            globalVisualizersUI.addVisualizer(new GDXROS1PointCloudVisualizer("Ouster Point Cloud", RosTools.OUSTER_POINT_CLOUD));

            //        simulatedDepthSensor = createSimulatedDepthSensor(ros1Node);
            //        simulatedDepthSensor.setSensorFrameToWorldTransform(depthSensorTransform);
            //        simulatedDepthSensor.setRenderPointCloudDirectly(true);
            //        simulatedDepthSensor.setDebugCoordinateFrame(true);

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            buildingConstructor = new GDXBuildingConstructor(baseUI.getPrimary3DPanel());

            baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);
            //            baseUI.getImGuiPanelManager().addPanel(mapsenseRegionsVisualizer.getLoggingPanel());
            //        baseUI.getImGuiDockingSetup().addWindow(simulatedDepthSensor.getWindowName(), simulatedDepthSensor::renderImGuiWindow);
            //            baseUI.getImGuiPanelManager().addPanel(mapsenseConfigurationUI.getWindowName(), mapsenseConfigurationUI::render);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(buildingConstructor.getPanelName(), buildingConstructor::renderImGuiWidgets);

            //        l515TransformTuner = new GDXTransformTuner(robotModel.getSensorInformation().getSteppingCameraTransform());
            //        baseUI.getImGuiPanelManager().addPanel("L515 Transform Tuner", l515TransformTuner::renderImGuiWidgets);

            //            ousterLidar = createOusterLidar(ros1Node, ros2Node);
            //            ousterLidar.setupForROS1PointCloud(ros1Node, RosTools.OUSTER_POINT_CLOUD);
            //            ousterLidar.setSensorEnabled(true);
            //            ousterLidar.setRenderPointCloudDirectly(true);
            //            ousterLidar.setSensorFrameToWorldTransform(depthSensorTransform);

            //            ros1Node.execute();

            H5File file = new H5File(HDF5_FILENAME, H5F_ACC_RDONLY);

            HDF5Tools.loadPointCloud(file, pointsToRender, frameIndex);

            depthMap = new Mat(768, 1024, opencv_core.CV_16UC1);
            HDF5Tools.loadDepthMap(file, frameIndex, depthMap);

            //            BytedecoOpenCVTools.printMat(depthMap, "Depth");

            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersUI, GDXSceneLevel.VIRTUAL);

            //                simulatedDepthSensor.create();
            //                baseUI.getSceneManager().addRenderableProvider(simulatedDepthSensor, GDXSceneLevel.VIRTUAL);

            //            baseUI.getPrimaryScene().addRenderableProvider(ousterLidar);
            //            baseUI.getImGuiPanelManager().addPanel(ousterLidar);

            environmentBuilder.create();
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            buildingConstructor.create();
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(buildingConstructor::getRealRenderables, GDXSceneLevel.MODEL);

            globalVisualizersUI.create();
            //                l515Model = new GDXL515SensorObject();
            //                environmentBuilderUI.getModelInput().addInstance(l515Model);

            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
            pointCloudRenderer.create(200000);

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

            //            ousterLidar.render(baseUI.getPrimaryScene());

            globalVisualizersUI.update();

            frameIndex++;

            if ((frameIndex % 30) == 0)
            {
               H5File file = new H5File(HDF5_FILENAME, H5F_ACC_RDONLY);
               HDF5Tools.loadPointCloud(file, pointsToRender, frameIndex / 30);
               LogTools.info("Loading Cloud: {}", frameIndex / 30);

               depthMap = new Mat(768, 1024, opencv_core.CV_16UC1);
               HDF5Tools.loadDepthMap(file, frameIndex / 30, depthMap);
               LogTools.info("Image Loaded: {} {}", depthMap.arrayWidth(), depthMap.arrayHeight());

               //               BytedecoOpenCVTools.printMat(depthMap, "Depth");

               Mat image = new Mat(768, 1024, opencv_core.CV_8UC1);

               depthMap.convertTo(image, opencv_core.CV_8UC1, 256.0f, 0.0f);
               imshow("Loaded Image", image);
               waitKey(1);
            }

            pointCloudRenderer.setPointsToRender(pointsToRender, Color.BLUE);
            if (!pointsToRender.isEmpty())
            {
               pointCloudRenderer.updateMesh();
            }

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
            //            ros1Node.shutdown();
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

