package us.ihmc.rdx.perception;

import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.detections.doors.DoorDetectionManager;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionThread;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractionThread;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.doors.RDXROS2DoorDetectionPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Settings;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_NEURAL;
import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_PERFORMANCE;

public class RDXDoorDetectionManagerDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                   .toAbsolutePath()
                                                                                   .toString();

   private final ROS2Node ros2Node;
   private final ROS2PeerClockOffsetEstimator robotClockOffsetEstimator;
   private final ROS2PeerClockOffsetEstimator uiClockOffsetEstimator;
   private final OpenCLManager openCLManager;

   private final DoorDetectionManager doorDetectionManager;

   private final ROS2ZEDSVOPlaybackSensor zed;
   private final YOLOv8DetectionThread yoloThread;
   private final RapidPlanarRegionsExtractionThread planarRegionThread;

   private final RDXBaseUI baseUI;
   private final RDXZEDSVORecorderPanel zedSVOPanel;
   private final RDXRawImagePointCloudRenderer pointCloudRenderer;
   private final RDXPlanarRegionsGraphic planarRegionsGraphic;
   private final RDXROS2YOLOv8Settings yoloSettings;
   private final RDXROS2DoorDetectionPanel doorDetectionPanel;
   private long lastSequenceNumber = -1L;

   private final AtomicBoolean destroyed = new AtomicBoolean(false);

   private RDXDoorDetectionManagerDemo()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy));

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName());
      robotClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      uiClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      openCLManager = new OpenCLManager();

      doorDetectionManager = new DoorDetectionManager(ros2Node);

      boolean enableNeuralMode = CUDATools.hasCUDADeviceOfAtLeast(CUDATools.getDeviceName(0), "RTX 3080");
      zed = new ROS2ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, enableNeuralMode ? SL_DEPTH_MODE_NEURAL : SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
      zed.useTrackedPose(true);

      yoloThread = new YOLOv8DetectionThread(robotClockOffsetEstimator, () -> true);
      yoloThread.setImageSensor(zed, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY, ZEDImageSensor.DEPTH_IMAGE_KEY);
      yoloThread.addDetectionConsumerCallback(doorDetectionManager::registerNewDetections);

      BlockingQueue<RawImage> rawImageCollection = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      planarRegionThread = new RapidPlanarRegionsExtractionThread(ros2Node, rawImageCollection);
      planarRegionThread.addPlanarRegionsConsumer(doorDetectionManager::updatePlanarRegions);

      baseUI = new RDXBaseUI();
      zedSVOPanel = new RDXZEDSVORecorderPanel(ros2Helper);
      pointCloudRenderer = new RDXRawImagePointCloudRenderer();
      planarRegionsGraphic = new RDXPlanarRegionsGraphic();
      planarRegionsGraphic.setBlendOpacity(0.1f);
      planarRegionThread.addPlanarRegionsConsumer(framePlanarRegions ->
      {
         PlanarRegionsList planarRegionsList = framePlanarRegions.getPlanarRegionsList();
         planarRegionsList.applyTransform(framePlanarRegions.getSensorToWorldFrameTransform());
         planarRegionsGraphic.generateMeshes(planarRegionsList);
      });
      yoloSettings = new RDXROS2YOLOv8Settings(ros2Node, uiClockOffsetEstimator);
      doorDetectionPanel = new RDXROS2DoorDetectionPanel(ros2Node);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            pointCloudRenderer.create(1280 * 720);
            pointCloudRenderer.setColoringMethod(ColoringMethod.COLOR_IMAGE);

            baseUI.getImGuiPanelManager().addPanel("ZED SVO", zedSVOPanel::render);
            baseUI.getImGuiPanelManager().addPanel("YOLO Settings", yoloSettings::renderSettings);
            baseUI.getImGuiPanelManager().addPanel(doorDetectionPanel);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionsGraphic);
            baseUI.getPrimaryScene().addRenderableProvider(doorDetectionPanel);
            baseUI.create();

            zed.run(true);
            yoloThread.startRepeating();
            planarRegionThread.startRepeating();
         }

         @Override
         public void render()
         {
            updatePointCloud();
            zedSVOPanel.update();
            planarRegionsGraphic.update();
            yoloSettings.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            pointCloudRenderer.dispose();
            yoloSettings.destroy();
            baseUI.dispose();

            destroy();
         }
      });
   }

   private void updatePointCloud()
   {
      RawImage depthImage = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
      if (depthImage == null)
         return;

      RawImage colorImage = zed.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      if (colorImage == null)
      {
         depthImage.release();
         return;
      }

      if (lastSequenceNumber != depthImage.getSequenceNumber())
      {
         lastSequenceNumber = depthImage.getSequenceNumber();
         pointCloudRenderer.updateMesh(depthImage, colorImage);
         doorDetectionManager.update();
      }

      depthImage.release();
      colorImage.release();
   }

   private void destroy()
   {
      if (destroyed.getAndSet(true))
         return;

      yoloThread.blockingKill();
      planarRegionThread.blockingKill();
      zed.close();

      robotClockOffsetEstimator.destroy();
      uiClockOffsetEstimator.destroy();
      ros2Node.destroy();
      openCLManager.destroy();
   }

   public static void main(String[] args)
   {
      new RDXDoorDetectionManagerDemo();
   }
}
