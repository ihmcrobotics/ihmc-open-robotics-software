package us.ihmc.rdx.perception.sceneGraph;

import org.bytedeco.opencl.global.OpenCL;
import org.jetbrains.annotations.Nullable;
import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.rapidRegions.RapidRegionsExtractorParameters;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXZEDSVORecorderPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXDetectionManagerSettings;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXYOLOv8Settings;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO.RecordMode;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.RestartableThrottledThread;

/**
 * A self contained demo and development environment for our scene graph functionality.
 */
public class RDXSceneGraphDemo
{
   private enum SensorMode
   {
      SIMULATED, ZED_SVO_RECORDING
   }

   private static final SensorMode SENSOR_MODE = SensorMode.ZED_SVO_RECORDING;
   // Drive folder with recordings https://drive.google.com/drive/u/0/folders/17TIgXgNPslUyzBFWy6Waev11fx__3w9D
   private static final String SVO_FILE_NAME = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240626_161110_ZEDRecording_RobotWalkONRCoursePart1.svo2").toAbsolutePath().toString();
   private static final PubSubImplementation PUB_SUB_IMPLEMENTATION = PubSubImplementation.FAST_RTPS;

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private RDXPerceptionVisualizersPanel perceptionVisualizerPanel;
   private RDXYOLOv8Settings yoloSettingsVisualizer;
   private RDXROS2ImageMessageVisualizer yoloAnnotatedImageVisualizer;
   private RDXROS2FramePlanarRegionsVisualizer planarRegionsVisualizer;
   private RDXDetectionManagerSettings detectionManagerSettings;
   private DetectionManager detectionManager;
   private YOLOv8DetectionExecutor yolov8DetectionExecutor;
   private ROS2SceneGraph onRobotSceneGraph;
   private RDXSceneGraphUI sceneGraphUI;

   private RestartableThrottledThread perceptionUpdateThread;

   // Planar regions stuff
   private RapidPlanarRegionsExtractor planarRegionsExtractor;
   private ROS2StoredPropertySet<RapidRegionsExtractorParameters> planarRegionsExtractorParameterSync;
   private final TypedNotification<PlanarRegionsList> newPlanarRegions = new TypedNotification<>();
   private final OpenCLManager planarRegionsOpenCLManager = new OpenCLManager();

   // Simulated sensor related things
   @Nullable
   private RDXPose3DGizmo sensorPoseGizmo;
   @Nullable
   private RDXHighLevelDepthSensorSimulator simulatedCamera;

   // ZED SVO sensor related things
   @Nullable
   private ZEDColorDepthImageRetrieverSVO zedColorDepthImageRetrieverSVO;
   @Nullable
   private ZEDColorDepthImagePublisher zedColorDepthImagePublisher;
   @Nullable
   private RawImage zedDepthImage;
   private final SideDependentList<RawImage> zedColorImages = new SideDependentList<>();
   private final MutableReferenceFrame sensorFrame = new MutableReferenceFrame();
   private RDXZEDSVORecorderPanel zedSVORecorderPanel;

   public RDXSceneGraphDemo()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy));

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.VIRTUAL, RDXSceneLevel.MODEL, RDXSceneLevel.GROUND_TRUTH);

            ros2Node = ROS2Tools.createROS2Node(PUB_SUB_IMPLEMENTATION, "perception_scene_graph_demo");
            ros2Helper = new ROS2Helper(ros2Node);

            detectionManager = new DetectionManager(ros2Helper);

            // Add perception visualizers
            perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            createVisualizers();
            perceptionVisualizerPanel.create();

            // Setup sensors
            if (SENSOR_MODE == SensorMode.SIMULATED)
            {
               setupSimulatedSensor();
            }
            else if (SENSOR_MODE == SensorMode.ZED_SVO_RECORDING)
            {
               setupZEDSVOSensor();
            }

            // Setup scene graph
            onRobotSceneGraph = new ROS2SceneGraph(ros2Helper);
            sceneGraphUI = new RDXSceneGraphUI(ros2Helper, baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(sceneGraphUI::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(sceneGraphUI.getPanel());

            // Add rapid region parameters panel
            ImGuiRemoteROS2StoredPropertySet rapidRegionsParameterPanel = new ImGuiRemoteROS2StoredPropertySet(ros2Helper,
                                                                                                               new RapidRegionsExtractorParameters(),
                                                                                                               PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsParameterPanel.createPanel());

            perceptionUpdateThread = new RestartableThrottledThread("PerceptionUpdateThread", 30.0, new RunnableThatThrows()
            {
               // Main perception thread loop
               @Override
               public void run() throws Throwable
               {
                  if (SENSOR_MODE == SensorMode.ZED_SVO_RECORDING)
                  {
                     zedDepthImage = zedColorDepthImageRetrieverSVO.getLatestRawDepthImage();
                     for (RobotSide side : RobotSide.values)
                        zedColorImages.put(side, zedColorDepthImageRetrieverSVO.getLatestRawColorImage(side));

                     zedColorDepthImagePublisher.setNextGpuDepthImage(zedDepthImage.get());
                     for (RobotSide side : RobotSide.values)
                        zedColorDepthImagePublisher.setNextColorImage(zedColorImages.get(side).get(), side);

                     sensorFrame.update(transform -> transform.set(zedColorDepthImageRetrieverSVO.getLatestSensorPose()));
                  }

                  updatePlanarRegions();
                  runYOLO();
                  updateSceneGraph();
               }

               private void updatePlanarRegions()
               {
                  if (planarRegionsExtractor == null)
                  {
                     int imageHeight = zedDepthImage.getImageHeight();
                     int imageWidth = zedDepthImage.getImageWidth();
                     double fx = zedDepthImage.getFocalLengthX();
                     double fy = zedDepthImage.getFocalLengthY();
                     double cx = zedDepthImage.getPrincipalPointX();
                     double cy = zedDepthImage.getPrincipalPointY();
                     planarRegionsExtractor = new RapidPlanarRegionsExtractor(planarRegionsOpenCLManager, imageHeight, imageWidth, fx, fy, cx, cy);
                     planarRegionsExtractor.getDebugger().setEnabled(false);

                     planarRegionsExtractorParameterSync = new ROS2StoredPropertySet<>(ros2Helper,
                                                                                       PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS,
                                                                                       planarRegionsExtractor.getParameters());
                  }

                  planarRegionsExtractorParameterSync.updateAndPublishThrottledStatus();

                  FramePlanarRegionsList framePlanarRegionsList = new FramePlanarRegionsList();

                  // TODO: Get rid of BytedecoImage, RapidPlanarRegionsExtractor requires it
                  BytedecoImage bytedecoImage = new BytedecoImage(zedDepthImage.getCpuImageMat().clone());
                  bytedecoImage.createOpenCLImage(planarRegionsOpenCLManager, OpenCL.CL_MEM_READ_WRITE);
                  planarRegionsExtractor.update(bytedecoImage, sensorFrame.getReferenceFrame(), framePlanarRegionsList);
                  planarRegionsExtractor.setProcessing(false);
                  bytedecoImage.destroy(planarRegionsOpenCLManager);

                  PlanarRegionsList planarRegionsInWorldFrame = framePlanarRegionsList.getPlanarRegionsList().copy();
                  planarRegionsInWorldFrame.applyTransform(sensorFrame.getReferenceFrame().getTransformToWorldFrame());

                  newPlanarRegions.set(planarRegionsInWorldFrame);

                  PerceptionMessageTools.publishFramePlanarRegionsList(framePlanarRegionsList, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ros2Helper);

                  zedDepthImage.release();
               }

               private void runYOLO()
               {
                  if (yolov8DetectionExecutor == null)
                  {
                     yolov8DetectionExecutor = new YOLOv8DetectionExecutor(ros2Helper, yoloAnnotatedImageVisualizer::isActive);
                     yolov8DetectionExecutor.addDetectionConsumerCallback(detectionManager::addDetections);
                  }

                  if (SENSOR_MODE == SensorMode.SIMULATED)
                  {
                     synchronized (simulatedCamera)
                     {
                        // TODO:
                        // yoloColorImageBGR = simulatedCamera.createRawColorImageBGR();
                        // yoloDepthImageDiscretized = simulatedCamera.createRawDepthImageDiscretized();
                     }
                  }
                  else if (SENSOR_MODE == SensorMode.ZED_SVO_RECORDING)
                  {
                     yolov8DetectionExecutor.runYOLODetectionOnAllModels(zedColorImages.get(RobotSide.LEFT), zedDepthImage);
                  }
               }

               private void updateSceneGraph()
               {
                  // TODO: finish
                  onRobotSceneGraph.updateSubscription();
                  onRobotSceneGraph.updateDetections(detectionManager);

                  if (newPlanarRegions.poll())
                     for (SceneNode sceneNode : onRobotSceneGraph.getSceneNodesByID())
                        if (sceneNode instanceof DoorNode doorNode)
                           doorNode.getDoorPanel().filterAndSetPlanarRegionFromPlanarRegionsList(newPlanarRegions.read());

                  onRobotSceneGraph.updateOnRobotOnly(sensorFrame.getReferenceFrame());
                  onRobotSceneGraph.updatePublication();
               }
            });
            perceptionUpdateThread.start();
         }

         @Override
         public void render()
         {
            if (SENSOR_MODE == SensorMode.SIMULATED)
            {
               synchronized (simulatedCamera)
               {
                  simulatedCamera.render(baseUI.getPrimaryScene());
               }
            }
            else if (SENSOR_MODE == SensorMode.ZED_SVO_RECORDING)
            {
               zedSVORecorderPanel.update();
            }

            sceneGraphUI.update();
            perceptionVisualizerPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            if (simulatedCamera != null)
            {
               simulatedCamera.dispose();
            }

            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   private void destroy()
   {
      perceptionUpdateThread.stop();

      yolov8DetectionExecutor.destroy();

      if (zedColorDepthImageRetrieverSVO != null)
         zedColorDepthImageRetrieverSVO.destroy();
      if (zedColorDepthImagePublisher != null)
         zedColorDepthImagePublisher.destroy();

      planarRegionsOpenCLManager.destroy();
   }

   private void setupSimulatedSensor()
   {
      sensorPoseGizmo = new RDXPose3DGizmo("SimulatedSensorGizmo");
      sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
      sensorPoseGizmo.setResizeAutomatically(true);
      sensorPoseGizmo.getTransformToParent().getTranslation().setZ(0.7);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
      baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

      simulatedCamera = RDXSimulatedSensorFactory.createBlackflyFisheye(sensorPoseGizmo.getGizmoFrame(), System::nanoTime);
      simulatedCamera.setSensorEnabled(true);
      simulatedCamera.setRenderColorVideoDirectly(true);
      baseUI.getPrimaryScene().addRenderableProvider(simulatedCamera::getRenderables);
   }

   private void setupZEDSVOSensor()
   {
      sensorPoseGizmo = new RDXPose3DGizmo("SVOZEDSensorGizmo");
      sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
      sensorPoseGizmo.setResizeAutomatically(true);
      sensorPoseGizmo.setGizmoFrame(sensorFrame.getReferenceFrame());
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
      baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

      zedColorDepthImageRetrieverSVO = new ZEDColorDepthImageRetrieverSVO(0, () -> true, () -> true, ros2Helper, RecordMode.PLAYBACK, SVO_FILE_NAME);
      zedColorDepthImageRetrieverSVO.start();
      zedColorDepthImagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES,
                                                                    PerceptionAPI.ZED2_DEPTH,
                                                                    PerceptionAPI.ZED2_CUT_OUT_DEPTH);

      zedSVORecorderPanel = new RDXZEDSVORecorderPanel(ros2Helper);
   }

   private void createVisualizers()
   {
      // ZED2 colored point cloud visualizer
      {
         RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                                                     PubSubImplementation.FAST_RTPS,
                                                                                                                     PerceptionAPI.ZED2_DEPTH,
                                                                                                                     PerceptionAPI.ZED2_COLOR_IMAGES.get(
                                                                                                                           RobotSide.LEFT));
         zed2ColoredPointCloudVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);
         zed2ColoredPointCloudVisualizer.setActive(true);
         perceptionVisualizerPanel.addVisualizer(zed2ColoredPointCloudVisualizer);
      }

      // ZED left color visualizer
      {
         RDXROS2ImageMessageVisualizer zedLeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                                                                       PubSubImplementation.FAST_RTPS,
                                                                                                       PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
         zedLeftColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_COLOR);
         perceptionVisualizerPanel.addVisualizer(zedLeftColorImageVisualizer);
      }

      // ZED 2 color right image visualizer
      {
         RDXROS2ImageMessageVisualizer zedRightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Right",
                                                                                                        PubSubImplementation.FAST_RTPS,
                                                                                                        PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
         zedRightColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_COLOR);
         perceptionVisualizerPanel.addVisualizer(zedRightColorImageVisualizer);
      }

      // ZED 2 depth image visualizer
      {
         RDXROS2ImageMessageVisualizer zed2DepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth Image",
                                                                                                    PubSubImplementation.FAST_RTPS,
                                                                                                    PerceptionAPI.ZED2_DEPTH);
         zed2DepthImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_DEPTH);
         perceptionVisualizerPanel.addVisualizer(zed2DepthImageVisualizer);
      }

      // Create detection manager settings
      {
         detectionManagerSettings = new RDXDetectionManagerSettings("Detection Manager Settings", ros2Helper);
         perceptionVisualizerPanel.addVisualizer(detectionManagerSettings);
      }

      // Create YOLO settings viz
      {
         yoloSettingsVisualizer = new RDXYOLOv8Settings("YOLOv8", ros2Helper);
         yoloSettingsVisualizer.setActive(true);
         perceptionVisualizerPanel.addVisualizer(yoloSettingsVisualizer);
      }

      // Create YOLO annotated image viz
      {
         yoloAnnotatedImageVisualizer = new RDXROS2ImageMessageVisualizer("YOLOv8 Annotated Image", PUB_SUB_IMPLEMENTATION, PerceptionAPI.YOLO_ANNOTATED_IMAGE);
         yoloAnnotatedImageVisualizer.setActive(true);
         perceptionVisualizerPanel.addVisualizer(yoloAnnotatedImageVisualizer);
      }

      // Create planar regions viz
      {
         planarRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Planar Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
         planarRegionsVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_PLANAR_REGIONS);
         planarRegionsVisualizer.setActive(false);
         perceptionVisualizerPanel.addVisualizer(planarRegionsVisualizer);
      }

      // Create ArUcoMarker pose viz
      {
         //         RDXROS2ArUcoMarkerPosesVisualizer arUcoMarkerPosesVisualizer = new RDXROS2ArUcoMarkerPosesVisualizer("ArUco Marker Poses",
         //                                                                                                              ros2Helper,
         //                                                                                                              PerceptionAPI.ARUCO_MARKER_POSES);
         //         arUcoMarkerPosesVisualizer.setActive(true);
         //         perceptionVisualizerPanel.addVisualizer(arUcoMarkerPosesVisualizer);
      }
   }

   public static void main(String[] args)
   {
      new RDXSceneGraphDemo();
   }
}
