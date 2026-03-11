package us.ihmc.perception.detections.foundationPose;

import ihmc_common_msgs.msg.dds.Box3DMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
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

public class CategoryLevelFoundationPoseCommunicator implements AutoCloseable
{
   private static final RotationMatrix FOUNDATION_POSE_TO_IHMC_ROTATION = new RotationMatrix(new double[] {0, 0, 1,
                                                                                                           -1, 0, 0,
                                                                                                           0, -1, 0});

   private final ROS2Node ros2Node;
   private final RawImagePublisher imagePublisher;
   private final ROS2Publisher<Empty> resetRequestPublisher;
   private final ROS2Publisher<Box3DMessage> resultRelayPublisher;
   private final ROS2Subscription<Detection3DArray> poseEstimationResultSubscription;
   private final ROS2Subscription<Detection3DArray> trackingResultSubscription;

   private final SyncedIsaacROSFoundationPoseParameters parameters;
   private final ROS2MutableFrame sensorFrame;

   private final CategoryLevelFoundationPoseTarget target;
   private final CategoryLevelFoundationPoseAPI.CategoryLevelFoundationPoseTopics topics;

   private final Point3D targetPoint = new Point3D();
   private final TypedNotification<Point3DReadOnly> newTargetPoint = new TypedNotification<>();

   private volatile CategoryLevelFoundationPoseInstantDetection latestResult;
   private final List<Consumer<CategoryLevelFoundationPoseInstantDetection>> resultCallbacks = new ArrayList<>();

   private boolean wasEnabled = false;

   public CategoryLevelFoundationPoseCommunicator(CategoryLevelFoundationPoseTarget target, CRDTInfo crdtInfo)
   {
      this.target = target;
      this.topics = CategoryLevelFoundationPoseAPI.topics(target.category(), target.instance());

      LogTools.info("Category-level FP target: {}/{}", target.category(), target.instance());
      LogTools.info("RGB topic: {}", topics.rgbImage());

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName() + "_" + sanitize(target.key()));
      imagePublisher = new RawImagePublisher(ros2Node, 0.5);

      resetRequestPublisher = ros2Node.createPublisher(topics.reset());
      resultRelayPublisher = ros2Node.createPublisher(topics.ihmcResult());

      poseEstimationResultSubscription = ros2Node.createSubscription2(topics.poseEstimationOutput(), this::updateLatestResult);
      trackingResultSubscription = ros2Node.createSubscription2(topics.trackingOutput(), this::updateLatestResult);

      parameters = new SyncedIsaacROSFoundationPoseParameters(ros2Node,
                                                              crdtInfo,
                                                              "FoundationPose " + target.category() + "/" + target.instance() + " Parameters",
                                                              topics.ihmcParameters());
      parameters.getEnabled().setValue(false);

      sensorFrame = new ROS2MutableFrame(target.instance() + "_ImageFrame", ReferenceFrame.getWorldFrame());
   }

   private static String sanitize(String s)
   {
      return s.replace('/', '_');
   }

   public CategoryLevelFoundationPoseTarget getTarget()
   {
      return target;
   }

   public CategoryLevelFoundationPoseAPI.CategoryLevelFoundationPoseTopics getTopics()
   {
      return topics;
   }

   public void update()
   {
      parameters.update();
   }

   private void updateLatestResult(Detection3DArray results)
   {
      if (results.getDetections().isEmpty())
         return;

      Detection3D result = results.getDetections().getFirst();
      if (result == null)
         return;

      FramePose3D poseInWorld = new FramePose3D(sensorFrame, result.getBbox().getCenter());
      poseInWorld.prependRotation(FOUNDATION_POSE_TO_IHMC_ROTATION);

      synchronized (sensorFrame)
      {
         poseInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      }

      latestResult = new CategoryLevelFoundationPoseInstantDetection(target,
                                                                     new Box3D(poseInWorld, result.getBbox().getSize()),
                                                                     Instant.now());

      Box3DMessage resultRelayMessage = new Box3DMessage();
      resultRelayMessage.getPose().set(poseInWorld);
      resultRelayMessage.getSize().set(result.getBbox().getSize());
      resultRelayPublisher.publish(resultRelayMessage);

      for (Consumer<CategoryLevelFoundationPoseInstantDetection> resultCallback : resultCallbacks)
         resultCallback.accept(latestResult);
   }

   public void updatePoseEstimation(List<InstantDetection> detections)
   {
      boolean enabled = parameters.getEnabled().getValue();

      if (enabled)
      {
         if (!wasEnabled)
            resetTracking();

         if (newTargetPoint.poll())
            targetPoint.set(newTargetPoint.read());
         else if (latestResult != null)
            targetPoint.set(latestResult.getPose().getPosition());
         else
            targetPoint.set(sensorFrame.getTransformToRoot().getTranslation());

         Optional<InstantDetection> closestYOLODetection =
               detections.stream()
                         .filter(detection -> detection instanceof YOLOv8InstantDetection
                                              && detection.getDetectedObjectClass().equals(target.yoloClass()))
                         .min(Comparator.comparingDouble(detection -> detection.getPose().getPosition().distanceSquared(targetPoint)));

         closestYOLODetection.ifPresent(detection -> updatePoseEstimation((YOLOv8InstantDetection) detection));
      }

      wasEnabled = enabled;
   }

   public void updatePoseEstimation(YOLOv8InstantDetection yoloDetection)
   {
      boolean enabled = parameters.getEnabled().getValue();
      if (!enabled)
      {
         wasEnabled = false;
         return;
      }

      if (!wasEnabled)
         resetTracking();

      updatePoseEstimation(yoloDetection.getColorImage(), yoloDetection.getDepthImage(), yoloDetection.getObjectMask());
      wasEnabled = true;
   }

   public void updatePoseEstimation(RawImage colorImage, RawImage depthImage, RawImage segmentation)
   {
      colorImage.get();
      depthImage.get();
      segmentation.get();

      RawImage rgbImage = RawImageTools.convertColor(colorImage, PixelFormat.RGB8);

      GpuMat depth32Mat = new GpuMat();
      depthImage.getGpuImageMat().convertTo(depth32Mat, opencv_core.CV_32FC1, depthImage.getDepthDiscretization());
      RawImage depth32FImage = depthImage.replaceImage(depth32Mat, PixelFormat.GRAY_F32);

      RawImage resizedSegmentation = RawImageTools.resize(segmentation, depth32FImage.getWidth(), depth32FImage.getHeight());

      synchronized (sensorFrame)
      {
         sensorFrame.setNewTransformToParent(colorImage.getTransformToWorld());
         sensorFrame.update();
      }

      ROS2Topic<Image> rgbTopic = topics.rgbImage();
      ROS2Topic<Image> depthTopic = topics.depthImage();
      ROS2Topic<Image> segmentationTopic = topics.segmentation();
      ROS2Topic<CameraInfo> cameraInfoTopic = topics.cameraInfo();

      imagePublisher.publishImage(rgbTopic, rgbImage, sensorFrame);
      imagePublisher.publishImage(depthTopic, depth32FImage, sensorFrame);
      imagePublisher.publishImage(segmentationTopic, resizedSegmentation, sensorFrame);
      imagePublisher.publishImage(cameraInfoTopic, rgbImage, sensorFrame);

      resizedSegmentation.release();
      depth32FImage.release();
      rgbImage.release();

      segmentation.release();
      depthImage.release();
      colorImage.release();
   }

   public void resetTracking()
   {
      resetRequestPublisher.publish(new Empty());
   }

   public CategoryLevelFoundationPoseInstantDetection getLatestResult()
   {
      return latestResult;
   }

   public void addResultCallback(Consumer<CategoryLevelFoundationPoseInstantDetection> resultCallback)
   {
      resultCallbacks.add(resultCallback);
   }

   public void enable(boolean enable)
   {
      parameters.getEnabled().setValue(enable);
   }

   public boolean isEnabled()
   {
      return parameters.getEnabled().getValue();
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

   public SyncedIsaacROSFoundationPoseParameters getParameters()
   {
      return parameters;
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