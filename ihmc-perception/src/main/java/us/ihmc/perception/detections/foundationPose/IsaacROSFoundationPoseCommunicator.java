package us.ihmc.perception.detections.foundationPose;

import us.ihmc.commons.thread.RepeatingTaskThread;
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
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.sensors.ImageSensor;
import vision_msgs.msg.dds.Detection3D;
import vision_msgs.msg.dds.Detection3DArray;

import java.util.List;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class IsaacROSFoundationPoseCommunicator implements AutoCloseable
{
   private static final RotationMatrix FOUNDATIONPOSE_TO_IHMC_ROTATION = new RotationMatrix(new double[] {0, 0, 1,
                                                                                                         -1, 0, 0,
                                                                                                          0,-1, 0});

   private final ROS2Node ros2Node;
   private final RawImagePublisher imagePublisher;
   private final ROS2Subscription<Detection3DArray> poseEstimationResultSubscription;
   private final ROS2Subscription<Detection3DArray> trackingResultSubscription;

   private final ImageSensor imageSensor;
   private final int colorImageKey;
   private final int depthImageKey;
   private final RepeatingTaskThread imageSensorThread;

   private final IsaacROSFoundationPoseObject objectToTrack;
   private final FramePose3D pose;
   private final Box3D boundingBox;
   private final ReadWriteLock resultLock;

   public IsaacROSFoundationPoseCommunicator(IsaacROSFoundationPoseObject objectToTrack, ImageSensor imageSensor, int colorImageKey, int depthImageKey)
   {
      this.objectToTrack = objectToTrack;
      this.imageSensor = imageSensor;
      this.colorImageKey = colorImageKey;
      this.depthImageKey = depthImageKey;

      pose = new FramePose3D();
      pose.setToNaN();
      boundingBox = new Box3D();
      boundingBox.setToNaN();
      resultLock = new ReentrantReadWriteLock();

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName() + "Node");
      imagePublisher = new RawImagePublisher(ros2Node, 0.5);
      poseEstimationResultSubscription = ros2Node.createSubscription2(objectToTrack.topics.poseEstimationOutput(), this::updateLatestResult);
      trackingResultSubscription = ros2Node.createSubscription2(objectToTrack.topics.trackingOutput(), this::updateLatestResult);

      imageSensorThread = new RepeatingTaskThread(getClass().getSimpleName() + "_ImageSensorThread", this::publishImages);
   }

   private void publishImages()
   {
      try
      {
         imageSensor.waitForGrab();

         RawImage colorImage = imageSensor.getImage(colorImageKey);
         RawImage depthImage = imageSensor.getImage(depthImageKey);

         RawImage rgbImage = RawImageTools.convertColor(colorImage, PixelFormat.RGB8);

         imagePublisher.publishImage(objectToTrack.topics.rgbImage(), rgbImage);
         imagePublisher.publishImage(objectToTrack.topics.depthImage(), depthImage);
         imagePublisher.publishImage(objectToTrack.topics.cameraInfo(), rgbImage);

         rgbImage.release();
         depthImage.release();
         colorImage.release();
      }
      catch (InterruptedException ignored) {}
   }

   private void updateLatestResult(Detection3DArray results)
   {
      Detection3D result = results.getDetections().getFirst();
      if (result == null)
         return;

      resultLock.writeLock().lock();
      try
      {
         pose.setReferenceFrame(imageSensor.getImageFrame(colorImageKey));
         pose.set(result.getBbox().getCenter());
         pose.prependRotation(FOUNDATIONPOSE_TO_IHMC_ROTATION);
         pose.changeFrame(ReferenceFrame.getWorldFrame());
         boundingBox.set(pose, result.getBbox().getSize());
      }
      finally
      {
         resultLock.writeLock().unlock();
      }
   }

   // Publishes segmentation
   public void updateDetections(List<InstantDetection> latestDetections)
   {
      for (InstantDetection detection : latestDetections)
      {
         if (detection instanceof YOLOv8InstantDetection yoloDetection && yoloDetection.getDetectedObjectClass().equals(objectToTrack.yoloClass))
         {
            imagePublisher.publishImage(objectToTrack.topics.segmentation(), yoloDetection.getObjectMask());
         }
      }
   }

   public void resetTracking()
   {
      // TODO: Implement this
   }

   // Get the latest pose returned by Isaac ROS FoundationPose
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

   @Override
   public void close()
   {
      imageSensorThread.blockingKill();
      poseEstimationResultSubscription.remove();
      trackingResultSubscription.remove();
      imagePublisher.close();
      ros2Node.destroy();
   }
}
