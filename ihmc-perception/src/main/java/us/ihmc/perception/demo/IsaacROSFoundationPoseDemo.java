package us.ihmc.perception.demo;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.opencv_core.GpuMat;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.communication.ros2.tf2.ROS2FollowingFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.ImageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;
import vision_msgs.msg.dds.Detection3DArray;

import java.time.Duration;
import java.time.Instant;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class IsaacROSFoundationPoseDemo
{
   // Topics we publish to FoundationPose
   private final ROS2Topic<?> reliableQoS = new ROS2Topic<>().withQoS(ROS2QosProfile.RELIABLE());
   private final ROS2Topic<Image> rgbTopic = reliableQoS.withModule("image").withType(Image.class);
   private final ROS2Topic<Image> depthTopic = reliableQoS.withModule("depth_image").withType(Image.class);
   private final ROS2Topic<Image> segmentationTopic = reliableQoS.withModule("segmentation").withType(Image.class);
   private final ROS2Topic<CameraInfo> cameraInfoTopic = reliableQoS.withModule("camera_info").withType(CameraInfo.class);
   private static final ROS2Topic<Detection3DArray> trackingResultTopic = new ROS2Topic<>().withModule("tracking/output").withType(Detection3DArray.class);
   private static final ROS2Topic<Detection3DArray> registrationResultTopic = new ROS2Topic<>().withModule("pose_estimation/output")
                                                                                               .withType(Detection3DArray.class);

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());
   private final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

   private final RawImagePublisher imagePublisher = new RawImagePublisher(ros2Node, 0.5);

   private final ZEDImageSensor zedImageSensor;
   private final ROS2FollowingFrame ros2ZEDFrame;

   private final DetectionManager detectionManager;
   private final YOLOv8DetectionExecutor yoloExecutor;

   private final RepeatingTaskThread taskThread;

   // For measuring registration time (start FoundationPose first, then this process)
   private final AtomicReference<Instant> firstSegmentationPublished = new AtomicReference<>(null);
   private final AtomicReference<Instant> firstTrackingReceived = new AtomicReference<>(null);
   private final AtomicReference<Instant> firstRegistrationReceived = new AtomicReference<>(null);
   private ROS2Subscription<Detection3DArray> trackingSubscription
         = ros2Node.createSubscription2(trackingResultTopic, result -> firstTrackingReceived.compareAndSet(null, Instant.now()));
   private ROS2Subscription<Detection3DArray> registrationSubscription
         = ros2Node.createSubscription2(registrationResultTopic, result -> firstRegistrationReceived.compareAndSet(null, Instant.now()));

   private IsaacROSFoundationPoseDemo()
   {
      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_2I, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_NEURAL);
      zedImageSensor.enablePositionalTracking(true);
      zedImageSensor.setSensorFrame(zedImageSensor.getTrackedSensorFrame());
      zedImageSensor.run(true);

      ros2ZEDFrame = new ROS2FollowingFrame(ros2Node, "zed_frame", ReferenceFrame.getWorldFrame(), zedImageSensor.getTrackedSensorFrame());

      detectionManager = new DetectionManager(ros2Node);
      yoloExecutor = new YOLOv8DetectionExecutor(new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockOffsetEstimator), () -> true);
      yoloExecutor.addDetectionConsumerCallback(detectionManager::addDetections);
      yoloExecutor.disableAllModels();

      taskThread = new RepeatingTaskThread(getClass().getSimpleName() + "Thread", this::task);
      taskThread.startRepeating();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Shutdown"));
   }

   private void task()
   {
      try
      {
         // Wait for the ZED to produce an image
         zedImageSensor.waitForGrab();

         // Update the ZED frame
         ros2ZEDFrame.update();

         // Get the color, and convert to RGB for FoundationPose
         RawImage color = zedImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage rgb = ImageTools.convertColor(color, PixelFormat.RGB8);

         // Get the depth and convert it to 32F for FoundationPose
         RawImage depth = zedImageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
         GpuMat depthImage = depth.getGpuImageMat();
         GpuMat depthImage32F = new GpuMat();
         depthImage.convertTo(depthImage32F, opencv_core.CV_32FC1, 0.001);
         RawImage depth32F = depth.replaceImage(depthImage32F, PixelFormat.GRAY_F32);

         // Publish the color and depth
         imagePublisher.publishImage(rgbTopic, rgb, ros2ZEDFrame);
         imagePublisher.publishImage(depthTopic, depth32F, ros2ZEDFrame);
         imagePublisher.publishImage(cameraInfoTopic, rgb, ros2ZEDFrame);

         // Also publish for the UI
         imagePublisher.publishImage(PerceptionAPI.ZED_DEPTH, depth);
         imagePublisher.publishImage(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT), color);

         // Run YOLO using the color image
         yoloExecutor.runNextEnabledModel(color, depth);

         // Update the detection manager
         detectionManager.updateDetections();

         // Publish any segmentations
         publishSegmentations(rgb);

         color.release();
         rgb.release();
         depth.release();
         depth32F.release();

         // Print stuff
         if (trackingSubscription != null && firstSegmentationPublished.get() != null && firstTrackingReceived.get() != null)
         {
            trackingSubscription.remove();
            trackingSubscription = null;

            Instant segmentation = firstSegmentationPublished.get();
            Instant result = firstTrackingReceived.get();
            LogTools.info("Segmentation to tracking result time: {}", Duration.between(segmentation, result));
         }

         if (registrationSubscription != null && firstSegmentationPublished.get() != null && firstRegistrationReceived.get() != null)
         {
            registrationSubscription.remove();
            registrationSubscription = null;

            Instant segmentation = firstSegmentationPublished.get();
            Instant result = firstRegistrationReceived.get();
            LogTools.info("Segmentation to registration result time: {}", Duration.between(segmentation, result));
         }
      }
      catch (InterruptedException ignored) {}
   }

   private void publishSegmentations(RawImage rgb)
   {
      Set<PersistentDetection> detections = detectionManager.getDetections();
      for (PersistentDetection detection : detections)
      {
         if (detection.getInstantDetectionClass() != YOLOv8InstantDetection.class)
            continue;

         YOLOv8InstantDetection latestDetection = (YOLOv8InstantDetection) detection.getMostRecentDetection();

         // We only need the mask for FoundationPose
         RawImage segmentation = latestDetection.getObjectMask().get();

         // Make sure it's the same size as the depth and rgb images
         RawImage resizedSegmentation;
         if (segmentation.getWidth() != rgb.getWidth() || segmentation.getHeight() != rgb.getHeight())
         {
            double widthScale = (double) rgb.getWidth() / segmentation.getWidth();
            double heightScale = (double) rgb.getHeight() / segmentation.getHeight();

            GpuMat originalMask = segmentation.getGpuImageMat();
            GpuMat resizedMask = new GpuMat();
            opencv_cudawarping.resize(originalMask, resizedMask, rgb.getGpuImageMat().size());

            CameraIntrinsics resizedIntrinsics = segmentation.getIntrinsicsCopy();
            resizedIntrinsics.setWidth(rgb.getWidth());
            resizedIntrinsics.setHeight(rgb.getHeight());
            resizedIntrinsics.setFx(widthScale * resizedIntrinsics.getFx());
            resizedIntrinsics.setFy(heightScale * resizedIntrinsics.getFy());
            resizedIntrinsics.setCx(widthScale * resizedIntrinsics.getCx());
            resizedIntrinsics.setCy(heightScale * resizedIntrinsics.getCy());

            resizedSegmentation = new RawImage(null,
                                               resizedMask,
                                               segmentation.getPixelFormat(),
                                               resizedIntrinsics,
                                               segmentation.getCameraModel(),
                                               segmentation.getTransformToWorld(),
                                               segmentation.getAcquisitionTime(),
                                               segmentation.getSequenceNumber(),
                                               segmentation.getDepthDiscretization());
         }
         else
         {
            resizedSegmentation = segmentation.get();
         }

         imagePublisher.publishImage(segmentationTopic, resizedSegmentation, ros2ZEDFrame);
         firstSegmentationPublished.compareAndSet(null, Instant.now());

         resizedSegmentation.release();
         segmentation.release();
      }
   }

   private void destroy()
   {
      taskThread.blockingKill();
      imagePublisher.close();
      zedImageSensor.close();
      yoloExecutor.destroy();
      peerClockOffsetEstimator.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      try
      {
         new IsaacROSFoundationPoseDemo();
      }
      catch (Throwable t)
      {
         Throwable throwable = t;
         LogTools.error("OOPS");
         LogTools.error(throwable.getMessage());
         LogTools.error(throwable.getCause());
         while (throwable != null) {
            for (StackTraceElement frame : throwable.getStackTrace()) {
               System.err.println(frame.toString());
            }
            throwable = throwable.getCause();
            if (throwable != null) {
               System.err.println("Caused by: " + throwable.getMessage());
            }
         }
      }
   }
}
