package us.ihmc.perception.detections.foundationPose;

import ihmc_common_msgs.msg.dds.Box3DMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
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

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

public class IsaacROSFoundationPoseCommunicator implements AutoCloseable
{
   private static final RotationMatrix FOUNDATION_POSE_TO_IHMC_ROTATION = new RotationMatrix(new double[] {0, 0, 1,
                                                                                                          -1, 0, 0,
                                                                                                           0,-1, 0});

   private final ROS2Node ros2Node;
   private final ROS2PeerClockOffsetEstimator clockOffsetEstimator;
   private final RawImagePublisher imagePublisher;
   private final ROS2Publisher<Empty> resetRequestPublisher;
   private final ROS2Publisher<Box3DMessage> resultPublisher;
   private final ROS2Subscription<Detection3DArray> poseEstimationResultSubscription;
   private final ROS2Subscription<Detection3DArray> trackingResultSubscription;

   public SyncedFoundationPoseParameters parameters;

   private final ROS2MutableFrame sensorFrame;

   private final IsaacROSFoundationPoseObject objectToTrack;
   private final FramePose3D pose;
   private final Box3D boundingBox;
   private final Box3DMessage resultMessage;
   private final ReadWriteLock resultLock;
   private final List<Consumer<Box3D>> resultCallbacks;

   public IsaacROSFoundationPoseCommunicator(IsaacROSFoundationPoseObject objectToTrack)
   {
      this.objectToTrack = objectToTrack;

      pose = new FramePose3D();
      pose.setToNaN();
      boundingBox = new Box3D();
      boundingBox.setToNaN();
      resultMessage = new Box3DMessage();
      resultLock = new ReentrantReadWriteLock();
      resultCallbacks = new ArrayList<>();

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName() + "Node");
      clockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      imagePublisher = new RawImagePublisher(ros2Node, 0.5);
      resetRequestPublisher = ros2Node.createPublisher(objectToTrack.topics.reset());
      resultPublisher = ros2Node.createPublisher(objectToTrack.topics.ihmcResult());
      poseEstimationResultSubscription = ros2Node.createSubscription2(objectToTrack.topics.poseEstimationOutput(), this::updateLatestResult);
      trackingResultSubscription = ros2Node.createSubscription2(objectToTrack.topics.trackingOutput(), this::updateLatestResult);

      parameters = new SyncedFoundationPoseParameters(ros2Node, new CRDTInfo(ROS2ActorDesignation.ROBOT, clockOffsetEstimator), objectToTrack);

      sensorFrame = new ROS2MutableFrame(ros2Node, objectToTrack.meshName + "_ImageFrame", ReferenceFrame.getWorldFrame());
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

      // We'll be writing to the result pose and bounding box, so we must lock the write lock
      resultLock.writeLock().lock();
      try
      {
         // Also synchronizing over the sensor frame
         synchronized (sensorFrame)
         {
            pose.setReferenceFrame(sensorFrame);
            pose.set(result.getBbox().getCenter());
            pose.prependRotation(FOUNDATION_POSE_TO_IHMC_ROTATION);
            pose.changeFrame(ReferenceFrame.getWorldFrame());
         }

         boundingBox.set(pose, result.getBbox().getSize());
      }
      finally
      {
         resultLock.writeLock().unlock();
      }

      // Relay the result using IHMC coordinates
      Box3D box = getLatestBoundingBox();
      resultMessage.getPose().set(box.getPose());
      resultMessage.getSize().set(box.getSize());
      resultPublisher.publish(resultMessage);

      // Run the callbacks
      for (Consumer<Box3D> resultCallback : resultCallbacks)
         resultCallback.accept(getLatestBoundingBox());
   }

   public void updateDetections(List<InstantDetection> latestDetections)
   {
      // Do nothing if not enabled
      if (!parameters.getEnabled().getValue())
         return;

      for (InstantDetection detection : latestDetections)
      {
         if (detection instanceof YOLOv8InstantDetection yoloDetection && yoloDetection.getDetectedObjectClass().equals(objectToTrack.yoloClass))
         {
            boolean autoResetEnabled = parameters.getAutoResetEnabled().getValue();
            double resetDistance = parameters.getResetDistance().getValue();

            if (autoResetEnabled && !pose.containsNaN()
                && yoloDetection.getPose().getPosition().distanceSquared(pose.getPosition()) > resetDistance * resetDistance)
               resetTracking();

            // Get the images
            RawImage colorImage = yoloDetection.getColorImage().get();
            RawImage depthImage = yoloDetection.getDepthImage().get();
            RawImage segmentation = yoloDetection.getObjectMask().get();

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

            return;
         }
      }
   }

   /**
    * Reset the tracking of the object.
    */
   public void resetTracking()
   {
      resetRequestPublisher.publish(new Empty());
   }

   /**
    * Get the latest pose received from FoundationPose.
    *
    * @return Pose of the object being tracked.
    */
   public FramePose3D getLatestPose()
   {
      FramePose3D poseCopy;

      resultLock.readLock().lock();
      try
      {
         poseCopy = new FramePose3D(pose);
      }
      finally
      {
         resultLock.readLock().unlock();
      }

      return poseCopy;
   }

   /**
    * Get the latest bounding box received from FoundationPose.
    *
    * @return Bounding box of the object being tracked.
    */
   public Box3D getLatestBoundingBox()
   {
      Box3D boundingBoxCopy;

      resultLock.readLock().lock();
      try
      {
         boundingBoxCopy = new Box3D(boundingBox);
      }
      finally
      {
         resultLock.readLock().unlock();
      }

      return boundingBoxCopy;
   }

   /**
    * Add a callback triggered when a new result is received from FoundationPose.
    *
    * @param boundingBoxConsumer Callback to run when a result is received.
    */
   public void addResultCallback(Consumer<Box3D> boundingBoxConsumer)
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

   @Override
   public void close()
   {
      parameters.close();
      poseEstimationResultSubscription.remove();
      trackingResultSubscription.remove();
      resetRequestPublisher.remove();
      resultPublisher.remove();
      imagePublisher.close();
      clockOffsetEstimator.destroy();
      ros2Node.destroy();
   }
}
