package us.ihmc.perception.detections.foundationPose;

import ihmc_common_msgs.Box3DMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Byte;
import std_msgs.Empty;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import vision_msgs.Detection3D;
import vision_msgs.Detection3DArray;

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
   private final ROS2Publisher<Byte> statePublisher;
   private final ROS2Subscription<Detection3DArray> poseEstimationResultSubscription;
   private final ROS2Subscription<Detection3DArray> trackingResultSubscription;
   private final ROS2Subscription<Empty> resetRequestSubscription;

   private final SyncedIsaacROSFoundationPoseParameters parameters;
   private State state;
   private State previousState;

   private final ROS2MutableFrame sensorFrame;

   private final IsaacROSFoundationPoseObject objectToTrack;
   private final Point3D targetPoint;
   private final TypedNotification<Point3DReadOnly> newTargetPoint;

   private volatile IsaacROSFoundationPoseInstantDetection latestResult;
   private final List<Consumer<IsaacROSFoundationPoseInstantDetection>> resultCallbacks;

   public IsaacROSFoundationPoseCommunicator(IsaacROSFoundationPoseObject objectToTrack, CRDTInfo crdtInfo)
   {
      this.objectToTrack = objectToTrack;

      targetPoint = new Point3D();
      newTargetPoint = new TypedNotification<>();

      ros2Node = new ROS2Node(getClass().getSimpleName() + "Node");
      imagePublisher = new RawImagePublisher(ros2Node, 0.5);
      resetRequestPublisher = ros2Node.createPublisher(objectToTrack.topics.reset());
      resultRelayPublisher = ros2Node.createPublisher(objectToTrack.topics.ihmcResult());
      statePublisher = ros2Node.createPublisher(objectToTrack.topics.ihmcState());
      poseEstimationResultSubscription = ros2Node.createSubscription(objectToTrack.topics.poseEstimationOutput(), reader -> this.updateLatestResult(reader.read()));
      trackingResultSubscription = ros2Node.createSubscription(objectToTrack.topics.trackingOutput(), reader -> this.updateLatestResult(reader.read()));
      resetRequestSubscription = ros2Node.createSubscription(objectToTrack.topics.reset(), reader -> changeState(State.ESTIMATING_POSE));

      parameters = new SyncedIsaacROSFoundationPoseParameters(ros2Node, crdtInfo, objectToTrack);
      parameters.getEnabled().setValue(false);
      state = State.DISABLED;
      previousState = state;

      sensorFrame = new ROS2MutableFrame(objectToTrack.meshDirectory + "_ImageFrame", ReferenceFrame.getWorldFrame());

      resultCallbacks = new ArrayList<>();
   }

   /**
    * A general update method.
    */
   public void update()
   {
      parameters.update();

      boolean enabled = parameters.getEnabled().getValue();

      if (!enabled && previousState != State.DISABLED)
         changeState(State.DISABLED);
      else if (enabled && previousState == State.DISABLED)
         resetTracking();
   }

   private void updateLatestResult(Detection3DArray results)
   {
      // Ensure the result message has a result
      if (results.getDetections().size() == 0)
         return;
      Detection3D result = results.getDetections().get(0);
      if (result == null)
         return;

      // Get the pose in world
      FramePose3D poseInWorld = new FramePose3D(sensorFrame, result.getBbox().getCenter().getPose());
      poseInWorld.prependRotation(FOUNDATION_POSE_TO_IHMC_ROTATION);
      synchronized (sensorFrame) // synchronize over the sensor frame when changing frame to avoid data race
      {
         poseInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      }

      // Update the latest result
      latestResult = new IsaacROSFoundationPoseInstantDetection(objectToTrack, new Box3D(poseInWorld, result.getBbox().getSize().getVector()), Instant.now());

      if (state != State.TRACKING)
         changeState(State.TRACKING);

      // Relay the result using IHMC coordinates
      Box3DMessage resultRelayMessage = new Box3DMessage();
      resultRelayMessage.getPose().set(poseInWorld);
      resultRelayMessage.getSize().set(result.getBbox().getSize());
      resultRelayPublisher.publish(resultRelayMessage);

      // Run the callbacks
      for (Consumer<IsaacROSFoundationPoseInstantDetection> resultCallback : resultCallbacks)
         resultCallback.accept(latestResult);
   }

   private synchronized void changeState(State newState)
   {
      previousState = state;
      state = newState;

      Byte stateMessage = new Byte();
      stateMessage.setData(state.toByte());
      statePublisher.publish(stateMessage);
   }

   /**
    * Publishes the required input to the Isaac ROS FoundationPose process to run pose estimation or tracking.
    * <p>
    * This method respects the settable parameters of this class.
    * If enabled is {@code false}, no input will be sent to FoundationPose.
    * If auto reset is enabled, tracking will be reset when the last result's position is outside the reset distance from the closest detection's position.
    * <p>
    * This method is designed to work in conjunction with
    * {@link us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor#addDetectionConsumerCallback(Consumer)}.
    *
    * @param detections List of detections which may contain the object of interest.
    *                   If the list contains multiple detections matching the type of the object of interest,
    *                   the detection closest to the target point will be used.
    */
   public void updatePoseEstimation(List<InstantDetection> detections)
   {
      // Do nothing if pose estimation is not enabled
      if (state != State.DISABLED)
      {
         // Update the target point
         if (newTargetPoint.poll())
            targetPoint.set(newTargetPoint.read());
         else if (latestResult != null)
            targetPoint.set(latestResult.getPose().getPosition());
         else
            targetPoint.set(sensorFrame.getTransformToRoot().getTranslation());

         // Find the YOLO detection that's closest to the target point
         Optional<InstantDetection> closestYOLODetection
               = detections.stream()
                           .filter(detection -> detection instanceof YOLOv8InstantDetection
                                                && detection.getDetectedObjectClass().equals(objectToTrack.yoloClass))
                           .min(Comparator.comparingDouble(detection -> detection.getPose().getPosition().distanceSquared(targetPoint)));

         if (closestYOLODetection.isPresent())
         {
            YOLOv8InstantDetection detection = (YOLOv8InstantDetection) closestYOLODetection.get();

            double resetDistance = parameters.getResetDistance().getValue();
            if (parameters.getAutoResetEnabled().getValue() && latestResult != null && state == State.TRACKING
                && detection.getPose().getPosition().distanceSquared(latestResult.getPose().getPosition()) > resetDistance * resetDistance)
               resetTracking();

            updatePoseEstimation(detection);
         }
      }
   }

   /**
    * Publishes the required input to the Isaac ROS FoundationPose process to run pose estimation or tracking.
    * <p>
    * This method does not query any settable parameters of this class.
    *
    * @param yoloDetection A YOLO detection of the obejct of interest.
    */
   public void updatePoseEstimation(YOLOv8InstantDetection yoloDetection)
   {
      updatePoseEstimation(yoloDetection.getColorImage(), yoloDetection.getDepthImage(), yoloDetection.getObjectMask());
   }

   /**
    * Publishes the required input to the Isaac ROS FoundationPose process to run pose estimation or tracking.
    * <p>
    * This method does not query any settable parameters of this class.
    *
    * @param colorImage   A color image aligned to the depth image. Must be same size as the depth image. Will be converted to RGB internally.
    * @param depthImage   A depth image. Will be converted to 32FC1 internally.
    * @param segmentation A mask of the object of interest. Will be resized to match the depth image size internally.
    */
   public void updatePoseEstimation(RawImage colorImage, RawImage depthImage, RawImage segmentation)
   {
      colorImage.get();
      depthImage.get();
      segmentation.get();

      // Convert images into correct types
      RawImage rgbImage = RawImageTools.convertColor(colorImage, PixelFormat.RGB8);

      GpuMat depth32Mat = new GpuMat();
      depthImage.getGpuImageMat().convertTo(depth32Mat, opencv_core.CV_32FC1, depthImage.getDepthDiscretization());
      RawImage depth32FImage = depthImage.replaceImage(depth32Mat, PixelFormat.GRAY_F32);

      GpuMat resizedSegmentationMat = new GpuMat();
      YOLOv8Tools.resizeWithCrop(segmentation.getGpuImageMat(), resizedSegmentationMat, depth32Mat.size());
      RawImage resizedSegmentation = depthImage.replaceImage(resizedSegmentationMat, PixelFormat.GRAY8);

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

   public void setTargetPoint(Point3DReadOnly targetPoint)
   {
      newTargetPoint.set(targetPoint);
   }

   @Override
   public void close()
   {
      parameters.close();
      ros2Node.destroySubscription(poseEstimationResultSubscription);
      ros2Node.destroySubscription(trackingResultSubscription);
      ros2Node.destroySubscription(resetRequestSubscription);
      ros2Node.destroyPublisher(resetRequestPublisher);
      ros2Node.destroyPublisher(resultRelayPublisher);
      ros2Node.destroyPublisher(statePublisher);
      imagePublisher.close();
      ros2Node.close();
   }

   public enum State
   {
      DISABLED,
      ESTIMATING_POSE,
      TRACKING;

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static State fromByte(byte ordinal)
      {
         return values()[ordinal];
      }
   }
}
