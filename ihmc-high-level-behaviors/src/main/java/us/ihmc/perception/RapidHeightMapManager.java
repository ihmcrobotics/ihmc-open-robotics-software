package us.ihmc.perception;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;

import java.time.Instant;

/**
 * This class takes care of managing a {@link RapidHeightMapExtractor} in the {@link us.ihmc.PerceptionAndAutonomyProcess}.
 */
public class RapidHeightMapManager
{
   private final RapidHeightMapExtractor heightMapExtractor;
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final FramePose3D cameraPoseForHeightMap = new FramePose3D();
   private final RigidBodyTransform sensorToWorldForHeightMap = new RigidBodyTransform();
   private final RigidBodyTransform sensorToGroundForHeightMap = new RigidBodyTransform();
   private final RigidBodyTransform groundToWorldForHeightMap = new RigidBodyTransform();
   private final BytedecoImage heightMapBytedecoImage;
   private final Notification resetHeightMapRequested = new Notification();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();

   public RapidHeightMapManager(OpenCLManager openCLManager,
                                DRCRobotModel robotModel,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                CameraIntrinsics depthImageIntrinsics,
                                ROS2PublishSubscribeAPI ros2)
   {
      heightMapExtractor = new RapidHeightMapExtractor(openCLManager, leftFootSoleFrame, rightFootSoleFrame);
      heightMapExtractor.setDepthIntrinsics(depthImageIntrinsics);

      heightMapBytedecoImage = new BytedecoImage(depthImageIntrinsics.getWidth(), depthImageIntrinsics.getHeight(), opencv_core.CV_16UC1);
      heightMapBytedecoImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      heightMapExtractor.create(heightMapBytedecoImage, 1);

      ros2.subscribeViaVolatileCallback(PerceptionAPI.RESET_HEIGHT_MAP, message -> resetHeightMapRequested.set());
      if (robotModel != null) // Will be null on test bench
      {
         ros2.subscribeViaVolatileCallback(HumanoidControllerAPI.getTopic(HighLevelStateChangeStatusMessage.class, robotModel.getSimpleRobotName()),
         message ->
         { // Automatically reset the height map when the robot goes into the walking state
            if (message.getEndHighLevelControllerName() == HighLevelStateChangeStatusMessage.WALKING)
              resetHeightMapRequested.set();
         });
      }
   }

   public void update(Mat latestDepthImage,
                      Instant imageAquisitionTime,
                      ReferenceFrame d455SensorFrame,
                      ReferenceFrame d455ZUpSensorFrame,
                      ROS2PublishSubscribeAPI ros2)
   {
      if (latestDepthImage.type() == opencv_core.CV_32FC1) // Support our simulated sensors
         OpenCVTools.convertFloatToShort(latestDepthImage, heightMapBytedecoImage.getBytedecoOpenCVMat(), 1000.0, 0.0);
      else
         latestDepthImage.copyTo(heightMapBytedecoImage.getBytedecoOpenCVMat());

      if (resetHeightMapRequested.poll())
      {
         heightMapExtractor.reset();
      }

      d455SensorFrame.getTransformToDesiredFrame(sensorToWorldForHeightMap, ReferenceFrame.getWorldFrame());
      d455SensorFrame.getTransformToDesiredFrame(sensorToGroundForHeightMap, d455ZUpSensorFrame);
      d455ZUpSensorFrame.getTransformToDesiredFrame(groundToWorldForHeightMap, ReferenceFrame.getWorldFrame());

      cameraPoseForHeightMap.setToZero(d455SensorFrame);
      cameraPoseForHeightMap.changeFrame(ReferenceFrame.getWorldFrame());

      heightMapExtractor.update(sensorToWorldForHeightMap, sensorToGroundForHeightMap, groundToWorldForHeightMap);

      Mat croppedHeightMapImage = heightMapExtractor.getTerrainMapData().getHeightMap();

      OpenCVTools.compressImagePNG(croppedHeightMapImage, compressedCroppedHeightMapPointer);
      PerceptionMessageTools.publishCompressedDepthImage(compressedCroppedHeightMapPointer,
                                                         PerceptionAPI.HEIGHT_MAP_CROPPED,
                                                         croppedHeightMapImageMessage,
                                                         ros2,
                                                         cameraPoseForHeightMap,
                                                         imageAquisitionTime,
                                                         heightMapExtractor.getSequenceNumber(),
                                                         croppedHeightMapImage.rows(),
                                                         croppedHeightMapImage.cols(),
                                                         (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor());
   }

   public void destroy()
   {
      heightMapExtractor.destroy();
   }

   public RapidHeightMapExtractor getHeightMapExtractor()
   {
      return heightMapExtractor;
   }
}
