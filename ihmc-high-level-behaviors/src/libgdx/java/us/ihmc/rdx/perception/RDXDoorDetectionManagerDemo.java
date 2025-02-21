package us.ihmc.rdx.perception;

import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.doors.DetectedDoor;
import us.ihmc.perception.detections.doors.DoorDetectionManager;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionThread;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractionThread;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;

import java.util.List;

public class RDXDoorDetectionManagerDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                   .toAbsolutePath()
                                                                                   .toString();

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final OpenCLManager openCLManager;

   private final DoorDetectionManager doorDetectionManager;

   private final ZEDSVOPlaybackSensor zed;
   private final YOLOv8DetectionThread yoloThread;
   private final RapidPlanarRegionsExtractionThread planarRegionThread;

   private final RDXBaseUI baseUI;
   private final RDXRawImagePointCloudRenderer pointCloudRenderer;
   private long lastSequenceNumber = -1L;

   private RDXDoorDetectionManagerDemo()
   {
      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName());
      ros2Helper = new ROS2Helper(ros2Node);
      openCLManager = new OpenCLManager();

      doorDetectionManager = new DoorDetectionManager();

      zed = new ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, SVO_FILE);
      zed.useTrackedPose(true);

      yoloThread = new YOLOv8DetectionThread(() -> true);
      yoloThread.setImageSensor(zed, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY, ZEDImageSensor.DEPTH_IMAGE_KEY);
      yoloThread.addDetectionConsumerCallback(doorDetectionManager::updateDetections);

      planarRegionThread = new RapidPlanarRegionsExtractionThread(ros2Node, openCLManager, zed, ZEDImageSensor.DEPTH_IMAGE_KEY);
      planarRegionThread.addPlanarRegionsConsumer(doorDetectionManager::updatePlanarRegions);

      baseUI = new RDXBaseUI();
      pointCloudRenderer = new RDXRawImagePointCloudRenderer();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            pointCloudRenderer.create(1280 * 720);
            pointCloudRenderer.setColoringMethod(ColoringMethod.COLOR_IMAGE);

            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
            baseUI.create();

            zed.run(true);
            yoloThread.startRepeating();
            planarRegionThread.startRepeating();
         }

         @Override
         public void render()
         {
            updatePointCloud();

            baseUI.renderBeforeOnScreenUI();

            // Render other stuff

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            pointCloudRenderer.dispose();
            baseUI.dispose();

            destroy();
         }
      });
   }

   private void renderDetections()
   {
      List<DetectedDoor> detectedDoors = doorDetectionManager.getDetectedDoors();

      for (DetectedDoor detectedDoor : detectedDoors)
      {

      }
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
      }

      depthImage.release();
      colorImage.release();
   }

   private void destroy()
   {
      ros2Node.destroy();
      openCLManager.destroy();

      zed.close();
      yoloThread.kill();
      planarRegionThread.kill();
   }

   public static void main(String[] args)
   {
      new RDXDoorDetectionManagerDemo();
   }
}
