package us.ihmc.perception;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractorCUDA;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractorInterface;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.time.Instant;

/**
 * This class takes care of managing a {@link RapidHeightMapExtractor}. This class can be used on remote processes or locally as well.
 */
public class RapidHeightMapManager
{
   private final RapidHeightMapExtractorInterface rapidHeightMapExtractor;
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final FramePose3D cameraPose = new FramePose3D();
   private final boolean runWithCUDA;
   private GpuMat deviceDepthImage;
   private final Mat hostDepthImage = new Mat();
   private BytedecoImage heightMapBytedecoImage;

   private final Notification resetHeightMapRequested = new Notification();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();

   private final String simpleRobotName;
   private ROS2PublishSubscribeAPI heightMapPublisher;

   public RapidHeightMapManager(String simpleRobotName,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                CameraIntrinsics depthImageIntrinsics,
                                boolean runWithCUDA)
   {
      this.runWithCUDA = runWithCUDA;
      this.simpleRobotName = simpleRobotName;

      if (runWithCUDA)
      {
         deviceDepthImage = new GpuMat(depthImageIntrinsics.getHeight(), depthImageIntrinsics.getWidth(), opencv_core.CV_16UC1);
         rapidHeightMapExtractor = new RapidHeightMapExtractorCUDA(leftFootSoleFrame, rightFootSoleFrame, deviceDepthImage, 1);
      }
      else
      {
         OpenCLManager openCLManager = new OpenCLManager();
         heightMapBytedecoImage = new BytedecoImage(depthImageIntrinsics.getWidth(), depthImageIntrinsics.getHeight(), opencv_core.CV_16UC1);
         heightMapBytedecoImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         rapidHeightMapExtractor = new RapidHeightMapExtractor(openCLManager, leftFootSoleFrame, rightFootSoleFrame, heightMapBytedecoImage, 1);
      }

      rapidHeightMapExtractor.setDepthIntrinsics(depthImageIntrinsics);
   }

   /**
    * Attaches a callback function that resets the height map on the next update if a message is received on the {@link PerceptionAPI#RESET_HEIGHT_MAP} topic.
    * @param ros2 pub/sub api to which to attach the subscriber.
    */
   public void attachResetRequestSubscriber(ROS2PublishSubscribeAPI ros2)
   {
      // We use a notification in order to only call resetting the height map in one place
      ros2.subscribeViaVolatileCallback(PerceptionAPI.RESET_HEIGHT_MAP, message -> resetHeightMapRequested.set());
   }

   /**
    * Attaches a callback function that resets the height map on the next update if a message is state change is detected that causes the robot to leave the
    * {@link us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName#WALKING} state..
    * @param ros2API pub/sub api to which to attach the subscriber.
    */
   public void attachResetOnStateChangeSubscriber(ROS2PublishSubscribeAPI ros2API)
   {
      if (simpleRobotName != null) // Will be null on test bench
      {
         ros2API.subscribeViaVolatileCallback(HumanoidControllerAPI.getTopic(HighLevelStateChangeStatusMessage.class, simpleRobotName), message ->
         { // Automatically reset the height map when the robot goes into the walking state
            if (message.getEndHighLevelControllerName() == HighLevelStateChangeStatusMessage.WALKING)
               resetHeightMapRequested.set();
         });
      }
   }

   /**
    * Sets the publisher and topic on which to publish the cropped height map. Uses the {@link PerceptionAPI#HEIGHT_MAP_CROPPED} topic, by default.
    * @param ros2 pub/sub api to which to attach the subscriber.
    */
   public void createHeightMapPublisher(ROS2PublishSubscribeAPI ros2)
   {
      createHeightMapPublisher(ros2, PerceptionAPI.HEIGHT_MAP_CROPPED);
   }

   /**
    * Sets the publisher and topic on which to publish the cropped height map. Uses the {@link PerceptionAPI#HEIGHT_MAP_CROPPED} topic, by default.
    * @param ros2 pub/sub api to which to attach the subscriber.
    */
   public void createHeightMapPublisher(ROS2PublishSubscribeAPI ros2, ROS2Topic<ImageMessage> publishTopic)
   {
      ros2.createPublisher(publishTopic);
      this.heightMapPublisher = ros2;
   }

   /**
    * Updates the internal height map from the latest sensor data.
    * @param latestDepthImage Depth image coming from the camera, represented in camera frame. The intrinsics are passed in in the constructor.
    * @param imageAcquisitionTime Time at which the image was aquired.
    * @param cameraFrame frame that defines this camera.
    * @param cameraZUpFrame Z up frame for the camera, where the orientation is aligned with z up in the world.
    */
   public void update(Mat latestDepthImage, Instant imageAcquisitionTime, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame)
   {
      if (runWithCUDA)
      {
         if (latestDepthImage.type() == opencv_core.CV_32FC1) // Support our simulated sensors
         {
            OpenCVTools.convertFloatToShort(latestDepthImage, hostDepthImage, 1000.0, 0.0);
         }
         else
         {
            latestDepthImage.convertTo(hostDepthImage, opencv_core.CV_16UC1);
         }

         deviceDepthImage.upload(hostDepthImage);
      }
      else
      {
         if (latestDepthImage.type() == opencv_core.CV_32FC1) // Support our simulated sensors
         {
            OpenCVTools.convertFloatToShort(latestDepthImage, heightMapBytedecoImage.getBytedecoOpenCVMat(), 1000.0, 0.0);
         }
         else
         {
            latestDepthImage.convertTo(heightMapBytedecoImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1);
         }
      }

      if (resetHeightMapRequested.poll())
      {
         rapidHeightMapExtractor.reset();
      }

      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();

      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      rapidHeightMapExtractor.update(sensorToWorld, sensorToGround, groundToWorld);


      if (heightMapPublisher != null)
      {
         Mat croppedHeightMapImage = rapidHeightMapExtractor.getTerrainMapData().getHeightMap();
         OpenCVTools.compressImagePNG(croppedHeightMapImage, compressedCroppedHeightMapPointer);
         PerceptionMessageTools.publishCompressedDepthImage(compressedCroppedHeightMapPointer,
                                                            PerceptionAPI.HEIGHT_MAP_CROPPED,
                                                            croppedHeightMapImageMessage,
                                                            heightMapPublisher,
                                                            cameraPose,
                                                            imageAcquisitionTime,
                                                            rapidHeightMapExtractor.getSequenceNumber(),
                                                            croppedHeightMapImage.rows(),
                                                            croppedHeightMapImage.cols(),
                                                            (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor());
      }
   }

   public HeightMapData getLatestHeightMapData()
   {
      return RapidHeightMapExtractorCUDA.packHeightMapData(rapidHeightMapExtractor);
   }

   public TerrainMapData getTerrainMapData()
   {
      return rapidHeightMapExtractor.getTerrainMapData();
   }

   public void destroy()
   {
      rapidHeightMapExtractor.destroy();
   }
}