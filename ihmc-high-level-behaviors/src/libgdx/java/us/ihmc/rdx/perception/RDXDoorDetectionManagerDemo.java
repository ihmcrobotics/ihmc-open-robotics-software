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
import us.ihmc.rdx.perception.doors.RDXDetectedDoor;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Settings;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RDXDoorDetectionManagerDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                   .toAbsolutePath()
                                                                                   .toString();

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final OpenCLManager openCLManager;

   private final DoorDetectionManager doorDetectionManager;
   private final Map<DetectedDoor, RDXDetectedDoor> rdxDetectedDoors = new HashMap<>();

   private final ZEDSVOPlaybackSensor zed;
   private final YOLOv8DetectionThread yoloThread;
   private final RapidPlanarRegionsExtractionThread planarRegionThread;

   private final RDXBaseUI baseUI;
   private final RDXZEDSVORecorderPanel zedSVOPanel;
   private final RDXRawImagePointCloudRenderer pointCloudRenderer;
   private final RDXROS2YOLOv8Settings yoloSetings;
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
      zedSVOPanel = new RDXZEDSVORecorderPanel(ros2Helper);
      pointCloudRenderer = new RDXRawImagePointCloudRenderer();
      yoloSetings = new RDXROS2YOLOv8Settings(ros2Node);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            pointCloudRenderer.create(1280 * 720);
            pointCloudRenderer.setColoringMethod(ColoringMethod.COLOR_IMAGE);

            baseUI.getImGuiPanelManager().addPanel("ZED SVO", zedSVOPanel::render);
            baseUI.getImGuiPanelManager().addPanel("YOLO Settings", yoloSetings::renderSettings);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool, sceneLevels) ->
            {
               for (RDXDetectedDoor door : rdxDetectedDoors.values())
                  door.getRenderables(renderables, pool, sceneLevels);
            });
            baseUI.create();

            zed.run(true);
            yoloThread.startRepeating();
            planarRegionThread.startRepeating();
         }

         @Override
         public void render()
         {
            updatePointCloud();
            updateDetections();
            zedSVOPanel.update();
            yoloSetings.publishSettingsMessageIfChanged();

            baseUI.renderBeforeOnScreenUI();

            // Render other stuff

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            pointCloudRenderer.dispose();
            yoloSetings.destroy();
            baseUI.dispose();

            destroy();
         }
      });
   }

   private void updateDetections()
   {
      List<DetectedDoor> detectedDoors = doorDetectionManager.getDetectedDoors();

      rdxDetectedDoors.keySet().removeIf(key -> !detectedDoors.contains(key));

      for (DetectedDoor detectedDoor : detectedDoors)
      {
         RDXDetectedDoor rdxDetectedDoor = rdxDetectedDoors.get(detectedDoor);
         if (rdxDetectedDoor == null)
         {
            rdxDetectedDoor = new RDXDetectedDoor(detectedDoor);
            rdxDetectedDoors.put(detectedDoor, rdxDetectedDoor);
         }

         rdxDetectedDoor.update();
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
