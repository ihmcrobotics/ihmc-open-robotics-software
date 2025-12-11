package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.ImageSensorPublishThread;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.rapidRegions.RapidRegionsExtractorParameters;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXZEDSVORecorderPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXDetectionManagerSettings;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;

import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_NEURAL;
import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_PERFORMANCE;

/**
 * A self contained demo and development environment for our scene graph functionality.
 * Requires everything installed from ihmc-perception/README.md
 */
public class RDXSceneGraphDemo
{
   // Drive folder with recordings https://drive.google.com/drive/u/0/folders/17TIgXgNPslUyzBFWy6Waev11fx__3w9D
   private static final String SVO_FILE_NAME = "/opt/ihmc/LogData/UserFolders/TomaszFolder/20251020_ZEDXMini_DoorChargeBarrierBottle.svo2";

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private ROS2PeerClockOffsetEstimator robotClockOffsetEstimator;
   private ROS2PeerClockOffsetEstimator uiClockOffsetEstimator;
   private ModelInstance sensorPoseGraphic;
   private RDXPerceptionVisualizersPanel perceptionVisualizerPanel;
   private DetectionManager detectionManager;
   private YOLOv8DetectionExecutor yolov8DetectionExecutor;
   private ROS2SceneGraph onRobotSceneGraph;
   private RDXSceneGraphUI sceneGraphUI;

   private RepeatingTaskThread perceptionUpdateThread;

   // Planar regions stuff
   private RapidPlanarRegionsExtractor planarRegionsExtractor;
   private ROS2StoredPropertySet<RapidRegionsExtractorParameters> planarRegionsExtractorParameterSync;
   private final TypedNotification<PlanarRegionsList> newPlanarRegions = new TypedNotification<>();
   private final OpenCLManager planarRegionsOpenCLManager = new OpenCLManager();

   // ZED SVO sensor related things
   private ROS2ZEDSVOPlaybackSensor zedSVOPlayer;
   private ImageSensorPublishThread zedPublishThread;
   private RawImage zedDepthImage;
   private final SideDependentList<RawImage> zedColorImages = new SideDependentList<>();
   private final MutableReferenceFrame sensorFrame = new MutableReferenceFrame();
   private RDXZEDSVORecorderPanel zedSVORecorderPanel;

   public RDXSceneGraphDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.VIRTUAL, RDXSceneLevel.MODEL, RDXSceneLevel.GROUND_TRUTH);

            ros2Node = new ROS2NodeBuilder().build("perception_scene_graph_demo");
            ros2Helper = new ROS2Helper(ros2Node);
            robotClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
            uiClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

            detectionManager = new DetectionManager(ros2Node);

            // Add perception visualizers
            perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();

            for (RobotSide side : RobotSide.values)
            {
               RDXROS2ImageMessageVisualizer zedColorImageVisualizer
                     = new RDXROS2ImageMessageVisualizer("ZED 2 Color %s".formatted(side.getPascalCaseName()),
                                                         ros2Node,
                                                         PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(side));
               zedColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_EXPERIMENTAL_ZED_PUBLICATION);
               perceptionVisualizerPanel.addVisualizer(zedColorImageVisualizer);
            }

            RDXROS2ImageMessageVisualizer zed2DepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth Image",
                                                                                                       ros2Node,
                                                                                                       PerceptionAPI.EXPERIMENTAL_ZED_DEPTH);
            zed2DepthImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_EXPERIMENTAL_ZED_PUBLICATION);
            perceptionVisualizerPanel.addVisualizer(zed2DepthImageVisualizer);

            RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer
                  = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                           ros2Node,
                                                           PerceptionAPI.EXPERIMENTAL_ZED_DEPTH,
                                                           PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.LEFT));
            zed2ColoredPointCloudVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_EXPERIMENTAL_ZED_PUBLICATION);
            zed2ColoredPointCloudVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2ColoredPointCloudVisualizer);

            perceptionVisualizerPanel.addVisualizer(new RDXDetectionManagerSettings("Detection Manager Settings", ros2Node));

            RDXROS2YOLOv8Visualizer yoloSettingsVisualizer = new RDXROS2YOLOv8Visualizer("YOLOv8",
                                                                                         ros2Node,
                                                                                         uiClockOffsetEstimator,
                                                                                         PerceptionAPI.YOLO_ANNOTATED_IMAGE);
            yoloSettingsVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(yoloSettingsVisualizer);

            RDXROS2FramePlanarRegionsVisualizer planarRegionsVisualizer
                  = new RDXROS2FramePlanarRegionsVisualizer("Planar Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
            planarRegionsVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_PLANAR_REGIONS);
            planarRegionsVisualizer.setActive(true);
            planarRegionsVisualizer.setOpacity(0.25);
            perceptionVisualizerPanel.addVisualizer(planarRegionsVisualizer);

            perceptionVisualizerPanel.create(baseUI);

            sensorPoseGraphic = RDXModelBuilder.createCoordinateFrameInstance(0.1);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGraphic, RDXSceneLevel.VIRTUAL);

            boolean enableNeuralMode = CUDATools.hasCUDADeviceOfAtLeast(CUDATools.getDeviceName(0), "RTX 3080");
            zedSVOPlayer = new ROS2ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, enableNeuralMode ? SL_DEPTH_MODE_NEURAL : SL_DEPTH_MODE_PERFORMANCE, SVO_FILE_NAME);
            zedSVOPlayer.useTrackedPose(true);
            zedSVOPlayer.run(true);

            zedPublishThread = new ImageSensorPublishThread(ros2Node, zedSVOPlayer);
            zedPublishThread.addTopic(PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.LEFT), ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
            zedPublishThread.addTopic(PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.RIGHT), ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
            zedPublishThread.addTopic(PerceptionAPI.EXPERIMENTAL_ZED_DEPTH, ZEDImageSensor.DEPTH_IMAGE_KEY);
            zedPublishThread.startRepeating();

            zedSVORecorderPanel = new RDXZEDSVORecorderPanel(ros2Helper);

            // Setup scene graph
            onRobotSceneGraph = new ROS2SceneGraph(ros2Helper);
            sceneGraphUI = new RDXSceneGraphUI(ros2Helper, baseUI);

            // Add rapid region parameters panel
            ImGuiRemoteROS2StoredPropertySet rapidRegionsParameterPanel
                  = new ImGuiRemoteROS2StoredPropertySet(ros2Node,
                                                         new RapidRegionsExtractorParameters(),
                                                         PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsParameterPanel.createPanel());

            perceptionUpdateThread = new RepeatingTaskThread("PerceptionUpdateThread", () ->
            {
               zedSVOPlayer.waitForGrab();

               zedDepthImage = zedSVOPlayer.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
               zedColorImages.put(RobotSide.LEFT, zedSVOPlayer.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY));
               zedColorImages.put(RobotSide.RIGHT, zedSVOPlayer.getImage(ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY));

               sensorFrame.update(transform -> transform.set(zedSVOPlayer.getTrackedSensorFrame().getTransformToWorldFrame()));
               LibGDXTools.toLibGDX(sensorFrame.getTransformToParent(), sensorPoseGraphic.transform);

               if (planarRegionsExtractor == null)
               {
                  int imageHeight = zedDepthImage.getHeight();
                  int imageWidth = zedDepthImage.getWidth();
                  double fx = zedDepthImage.getFocalLengthX();
                  double fy = zedDepthImage.getFocalLengthY();
                  double cx = zedDepthImage.getPrincipalPointX();
                  double cy = zedDepthImage.getPrincipalPointY();
                  planarRegionsExtractor = new RapidPlanarRegionsExtractor(planarRegionsOpenCLManager, imageHeight, imageWidth, fx, fy, cx, cy);
                  planarRegionsExtractor.getDebugger().setEnabled(false);

                  planarRegionsExtractorParameterSync = new ROS2StoredPropertySet<>(ros2Node,
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

               if (yolov8DetectionExecutor == null)
               {
                  yolov8DetectionExecutor = new YOLOv8DetectionExecutor(robotClockOffsetEstimator, yoloSettingsVisualizer::isActive);
                  yolov8DetectionExecutor.addDetectionConsumerCallback(detectionManager::addDetections);
               }

               yolov8DetectionExecutor.runNextEnabledModel(zedColorImages.get(RobotSide.LEFT), zedDepthImage);

               // TODO: finish
               onRobotSceneGraph.updateSubscription();
//               onRobotSceneGraph.updateDetections(detectionManager);

               if (newPlanarRegions.poll())
                  for (SceneNode sceneNode : onRobotSceneGraph.getSceneNodesByID())
                     if (sceneNode instanceof DoorNode doorNode)
                        doorNode.getDoorPanel().filterAndSetPlanarRegionFromPlanarRegionsList(newPlanarRegions.read());

               onRobotSceneGraph.updateOnRobotOnly(sensorFrame.getReferenceFrame());
               onRobotSceneGraph.updatePublication();
            }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
            perceptionUpdateThread.startRepeating();
         }

         @Override
         public void render()
         {
            zedSVORecorderPanel.update();

            sceneGraphUI.update();
            perceptionVisualizerPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            perceptionUpdateThread.blockingKill();

            yolov8DetectionExecutor.destroy();

            if (zedSVOPlayer != null)
               zedSVOPlayer.close();
            if (zedPublishThread != null)
               zedPublishThread.kill();

            planarRegionsOpenCLManager.destroy();

            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSceneGraphDemo();
   }
}
