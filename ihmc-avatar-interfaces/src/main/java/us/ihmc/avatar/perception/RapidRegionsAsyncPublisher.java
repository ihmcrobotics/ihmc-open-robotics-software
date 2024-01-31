package us.ihmc.avatar.perception;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Supplier;

public class RapidRegionsAsyncPublisher
{
   private final ROS2Helper ros2Helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RapidPlanarRegionsExtractor extractor;
   private final Mat depthImageMeters32FC1;
   private final Supplier<Boolean> enabled;
   private final ScheduledExecutorService executorService
         = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final BytedecoImage bytedecoDepthImage;

   public RapidRegionsAsyncPublisher(ROS2Helper ros2Helper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     RapidPlanarRegionsExtractor extractor,
                                     int imageHeight,
                                     int imageWidth,
                                     Mat depthImageMeters32FC1,
                                     OpenCLManager openCLManager,
                                     Supplier<Boolean> enabled)
   {
      this.ros2Helper = ros2Helper;
      this.syncedRobot = syncedRobot;
      this.extractor = extractor;
      this.depthImageMeters32FC1 = depthImageMeters32FC1;
      this.enabled = enabled;

      bytedecoDepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void update()
   {
      boolean enabled = this.enabled.get();
      extractor.setEnabled(enabled);
      executorService.submit(() ->
      {
         OpenCVTools.convertFloatToShort(depthImageMeters32FC1, bytedecoDepthImage.getBytedecoOpenCVMat(), 1000.0, 0.0);

         FramePlanarRegionsList frameRegions = new FramePlanarRegionsList();
         extractor.updateRobotConfigurationData(syncedRobot.getLatestRobotConfigurationData());
         extractor.update(bytedecoDepthImage, syncedRobot.getReferenceFrames().getSteppingCameraFrame(), frameRegions);
         PerceptionMessageTools.publishFramePlanarRegionsList(frameRegions, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ros2Helper);
         extractor.setProcessing(false);
      });
   }

   public void destroy()
   {
      executorService.shutdownNow();
   }
}
