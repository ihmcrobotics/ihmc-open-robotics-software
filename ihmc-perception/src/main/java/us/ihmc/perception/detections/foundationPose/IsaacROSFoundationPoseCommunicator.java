package us.ihmc.perception.detections.foundationPose;

import ihmc_common_msgs.msg.dds.Box3DMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import vision_msgs.msg.dds.Detection3D;
import vision_msgs.msg.dds.Detection3DArray;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

public class IsaacROSFoundationPoseCommunicator implements AutoCloseable
{
   private static final RotationMatrix FOUNDATION_POSE_TO_IHMC_ROTATION = new RotationMatrix(new double[] {0, 0, 1,
                                                                                                          -1, 0, 0,
                                                                                                           0,-1, 0});

   private final ROS2Node ros2Node;
   private final RawImagePublisher imagePublisher;
   private final ROS2Publisher<Empty> resetRequestPublisher;
   private final ROS2Publisher<Box3DMessage> resultRelayPublisher;
   private final ROS2Subscription<Detection3DArray> poseEstimationResultSubscription;
   private final ROS2Subscription<Detection3DArray> trackingResultSubscription;

   public SyncedFoundationPoseParameters parameters;

   private final ROS2MutableFrame sensorFrame;

   private final IsaacROSFoundationPoseObject objectToTrack;
   private final Point3D targetPoint;

   private volatile IsaacROSFoundationPoseInstantDetection latestResult;
   private final List<Consumer<IsaacROSFoundationPoseInstantDetection>> resultCallbacks;

   public IsaacROSFoundationPoseCommunicator(IsaacROSFoundationPoseObject objectToTrack, CRDTInfo crdtInfo)
   {
      this.objectToTrack = objectToTrack;

      targetPoint = new Point3D();

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName() + "Node");
      imagePublisher = new RawImagePublisher(ros2Node, 0.5);
      resetRequestPublisher = ros2Node.createPublisher(objectToTrack.topics.reset());
      resultRelayPublisher = ros2Node.createPublisher(objectToTrack.topics.ihmcResult());
      poseEstimationResultSubscription = ros2Node.createSubscription2(objectToTrack.topics.poseEstimationOutput(), this::updateLatestResult);
      trackingResultSubscription = ros2Node.createSubscription2(objectToTrack.topics.trackingOutput(), this::updateLatestResult);

      parameters = new SyncedFoundationPoseParameters(ros2Node, crdtInfo, objectToTrack);

      sensorFrame = new ROS2MutableFrame(ros2Node, objectToTrack.meshName + "_ImageFrame", ReferenceFrame.getWorldFrame());

      resultCallbacks = new ArrayList<>();
   }

   public void update()
   {
      parameters.update();
   }

   private void updateLatestResult(Detection3DArray results)
   {
      // Ensure the result message has a result
      Detection3D result = results.getDetections().getFirst();
      if (result == null)
         return;

      // Get the pose in world
      FramePose3D poseInWorld = new FramePose3D(sensorFrame, result.getBbox().getCenter());
      poseInWorld.prependRotation(FOUNDATION_POSE_TO_IHMC_ROTATION);
      synchronized (sensorFrame) // synchronize over the sensor frame when changing frame to avoid data race
      {
         poseInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      }

      // Update the latest result
      latestResult = new IsaacROSFoundationPoseInstantDetection(objectToTrack, new Box3D(poseInWorld, result.getBbox().getSize()), Instant.now());

      // Relay the result using IHMC coordinates
      Box3DMessage resultRelayMessage = new Box3DMessage();
      resultRelayMessage.getPose().set(poseInWorld);
      resultRelayMessage.getSize().set(result.getBbox().getSize());
      resultRelayPublisher.publish(resultRelayMessage);

      // Run the callbacks
      for (Consumer<IsaacROSFoundationPoseInstantDetection> resultCallback : resultCallbacks)
         resultCallback.accept(latestResult);
   }

   public void updatePoseEstimation(List<InstantDetection> detections)
   {
      // Do nothing if pose estimation is not enabled
      if (!parameters.getEnabled().getValue())
         return;

      // Get a copy of the target point so we don't synchronize too long
      final Point3D targetPointCopy;
      synchronized (targetPoint)
      {
         targetPointCopy = new Point3D(targetPoint);
      }

      // Find the YOLO detection that's closest to the target point
      Optional<InstantDetection> closestYOLODetection
            = detections.stream()
                        .filter(detection -> detection instanceof YOLOv8InstantDetection)
                        .min(Comparator.comparingDouble(detection -> detection.getPose().getPosition().distanceSquared(targetPointCopy)));

      // Update pose estimation is a YOLO detection was found
      closestYOLODetection.ifPresent(detection -> updatePoseEstimation((YOLOv8InstantDetection) detection));
   }

   public void updatePoseEstimation(YOLOv8InstantDetection yoloDetection)
   {
      updatePoseEstimation(yoloDetection.getColorImage(), yoloDetection.getDepthImage(), yoloDetection.getObjectMask());
   }

   public void updatePoseEstimation(RawImage colorImage, RawImage depthImage, RawImage segmentation)
   {
      colorImage.get();
      depthImage.get();
      segmentation.get();

      // Convert images into correct types
      RawImage rgbImage = RawImageTools.convertColor(colorImage, PixelFormat.RGB8);

      GpuMat depth32Mat = new GpuMat();
      depthImage.getGpuImageMat().convertTo(depth32Mat, opencv_core.CV_32FC1, 0.001);
      RawImage depth32FImage = depthImage.replaceImage(depth32Mat, PixelFormat.GRAY_F32);

      RawImage resizedSegmentation = RawImageTools.resize(segmentation, depth32FImage.getWidth(), depth32FImage.getHeight());

      // Update the sensor frame (publishes TFMessage so FoundationPose gets the frame)
      synchronized (sensorFrame)
      {
         sensorFrame.setNewTransformToParent(colorImage.getTransformToWorld());
         sensorFrame.update();
      }

      // Get the topics to publish on
      ROS2Topic<Image> rgbTopic = objectToTrack.topics.rgbImage();
      ROS2Topic<Image> depthTopic = objectToTrack.topics.depthImage();
      ROS2Topic<Image> segmentationTopic = objectToTrack.topics.segmentation();
      ROS2Topic<CameraInfo> cameraInfoTopic = objectToTrack.topics.cameraInfo();

      // Publish the images to FoundationPose
      imagePublisher.publishImage(rgbTopic, rgbImage, sensorFrame);
      imagePublisher.publishImage(depthTopic, depth32FImage, sensorFrame);
      imagePublisher.publishImage(segmentationTopic, resizedSegmentation, sensorFrame);
      imagePublisher.publishImage(cameraInfoTopic, rgbImage, sensorFrame);

      // Release the images
      resizedSegmentation.release();
      depth32FImage.release();
      rgbImage.release();

      segmentation.release();
      depthImage.release();
      colorImage.release();
   }

   /**
    * Reset the tracking of the object.
    */
   public void resetTracking()
   {
      resetRequestPublisher.publish(new Empty());
   }

   /**
    * Get the latest bounding box received from FoundationPose.
    *
    * @return Bounding box of the object being tracked.
    */
   public IsaacROSFoundationPoseInstantDetection getLatestResult()
   {
      return latestResult;
   }

   /**
    * Add a callback triggered when a new result is received from FoundationPose.
    *
    * @param boundingBoxConsumer Callback to run when a result is received.
    */
   public void addResultCallback(Consumer<IsaacROSFoundationPoseInstantDetection> boundingBoxConsumer)
   {
      resultCallbacks.add(boundingBoxConsumer);
   }

   public void enable(boolean enable)
   {
      parameters.getEnabled().setValue(enable);
   }

   public void enableAutoReset(boolean enable)
   {
      parameters.getAutoResetEnabled().setValue(enable);
   }

   public void setResetDistance(double meters)
   {
      parameters.getResetDistance().setValue(meters);
   }

   public void setTargetPoint(Point3D targetPoint)
   {
      synchronized (this.targetPoint)
      {
         this.targetPoint.set(targetPoint);
      }
   }

   @Override
   public void close()
   {
      parameters.close();
      poseEstimationResultSubscription.remove();
      trackingResultSubscription.remove();
      resetRequestPublisher.remove();
      resultRelayPublisher.remove();
      imagePublisher.close();
      ros2Node.destroy();
   }
}
