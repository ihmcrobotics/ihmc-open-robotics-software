package us.ihmc.perception.detections.supervisePose;

import ihmc_common_msgs.msg.dds.Box3DMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Byte;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.CameraIntrinsics;
import vision_msgs.msg.dds.Detection3D;
import vision_msgs.msg.dds.Detection3DArray;

import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.Consumer;

public class SupervisePoseCommunicator implements AutoCloseable
{
   private static final RotationMatrix SUPERVISE_POSE_TO_IHMC_ROTATION =
         new RotationMatrix(new double[] {0, 0, 1,
                                          -1, 0, 0,
                                          0, -1, 0});

   private final ROS2Node ros2Node;
   private final RawImagePublisher imagePublisher;

   private final ROS2Publisher<Empty> resetRequestPublisher;
   private final ROS2Publisher<Box3DMessage> resultRelayPublisher;
   private final ROS2Publisher<Byte> statePublisher;

   private final ROS2Subscription<Detection3DArray> poseEstimationResultSubscription;
   private final ROS2Subscription<Detection3DArray> trackingResultSubscription;
   private final ROS2Subscription<Empty> resetRequestSubscription;

   private final SyncedSupervisePoseParameters parameters;
   private final ROS2MutableFrame sensorFrame;

   private final SupervisePoseTarget target;
   private final SupervisePoseAPI.SupervisePoseTopics topics;

   private final Point3D targetPoint = new Point3D();
   private final TypedNotification<Point3DReadOnly> newTargetPoint = new TypedNotification<>();

   private volatile SupervisePoseInstantDetection latestResult;
   private final List<Consumer<SupervisePoseInstantDetection>> resultCallbacks = new ArrayList<>();

   private volatile Pose3D rawSupervisePose = null;

   private State state;
   private State previousState;

   private static final boolean EXPORT_SUPERVISEPOSE_INPUTS = true;
   private static final String EXPORT_ROOT = System.getProperty("user.home") + "/supervisepose_topic_dump";

   public SupervisePoseCommunicator(SupervisePoseTarget target, CRDTInfo crdtInfo)
   {
      this.target = target;
      this.topics = SupervisePoseAPI.topics(target.category(), target.instance());

      LogTools.info("SupervisePose target: {}/{}", target.category(), target.instance());
      LogTools.info("RGB topic: {}", topics.rgbImage());

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName() + "_" + sanitize(target.key()));
      imagePublisher = new RawImagePublisher(ros2Node, 1.0);

      resetRequestPublisher = ros2Node.createPublisher(topics.reset());
      resultRelayPublisher = ros2Node.createPublisher(topics.ihmcResult());
      statePublisher = ros2Node.createPublisher(topics.ihmcState());

      poseEstimationResultSubscription =
            ros2Node.createSubscription2(topics.poseEstimationOutput(), this::updateLatestResult);

      trackingResultSubscription =
            ros2Node.createSubscription2(topics.trackingOutput(), this::updateLatestResult);

      resetRequestSubscription =
            ros2Node.createSubscription2(topics.reset(), message -> changeState(State.ESTIMATING_POSE));

      SupervisePoseObject object =
            SupervisePoseObject.fromCategoryAndInstance(target.category(), target.instance());

      parameters = new SyncedSupervisePoseParameters(ros2Node, crdtInfo, object);
      parameters.getEnabled().setValue(false);

      state = State.DISABLED;
      previousState = state;

      sensorFrame =
            new ROS2MutableFrame(target.instance() + "_ImageFrame", ReferenceFrame.getWorldFrame());
   }

   private static String sanitize(String value)
   {
      return value.replace('/', '_');
   }

   public SupervisePoseTarget getTarget()
   {
      return target;
   }

   public SupervisePoseAPI.SupervisePoseTopics getTopics()
   {
      return topics;
   }

   public void update()
   {
      parameters.update();

      boolean enabled = parameters.getEnabled().getValue();

      if (!enabled && state != State.DISABLED)
      {
         changeState(State.DISABLED);
      }
      else if (enabled && state == State.DISABLED)
      {
         resetTracking();
      }
   }

   private void updateLatestResult(Detection3DArray results)
   {
      if (results.getDetections().isEmpty())
         return;

      Detection3D result = results.getDetections().getFirst();
      if (result == null)
         return;

      FramePose3D poseInWorld =
            new FramePose3D(sensorFrame, result.getBbox().getCenter());

      rawSupervisePose = new Pose3D(result.getBbox().getCenter());

      poseInWorld.prependRotation(SUPERVISE_POSE_TO_IHMC_ROTATION);

      synchronized (sensorFrame)
      {
         poseInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      }

      latestResult =
            new SupervisePoseInstantDetection(target,
                                              new Box3D(poseInWorld, result.getBbox().getSize()),
                                              Instant.now());

      if (state != State.TRACKING)
         changeState(State.TRACKING);

      LogTools.info("Received SupervisePose result for {}/{}",
                    target.category(),
                    target.instance());

      Box3DMessage resultRelayMessage = new Box3DMessage();
      resultRelayMessage.getPose().set(poseInWorld);
      resultRelayMessage.getSize().set(result.getBbox().getSize());
      resultRelayPublisher.publish(resultRelayMessage);

      for (Consumer<SupervisePoseInstantDetection> resultCallback : resultCallbacks)
         resultCallback.accept(latestResult);
   }

   private synchronized void changeState(State newState)
   {
      if (state == newState)
         return;

      previousState = state;
      state = newState;

      Byte stateMessage = new Byte();
      stateMessage.setData(state.toByte());
      statePublisher.publish(stateMessage);

      LogTools.info(String.format(
            "SupervisePose state changed for %s/%s: %s -> %s",
            target.category(),
            target.instance(),
            previousState,
            state));
   }

   public void updatePoseEstimation(List<InstantDetection> detections)
   {
      if (state == State.DISABLED)
         return;

      if (newTargetPoint.poll())
      {
         targetPoint.set(newTargetPoint.read());
      }
      else if (latestResult != null)
      {
         targetPoint.set(latestResult.getPose().getPosition());
      }
      else
      {
         targetPoint.set(sensorFrame.getTransformToRoot().getTranslation());
      }

      Optional<InstantDetection> closestYOLODetection =
            detections.stream()
                      .filter(detection -> detection instanceof YOLOv8InstantDetection
                                           && detection.getDetectedObjectClass().equals(target.yoloClass()))
                      .min(Comparator.comparingDouble(detection ->
                                                            detection.getPose().getPosition().distanceSquared(targetPoint)));

      closestYOLODetection.ifPresent(detection ->
                                           updatePoseEstimation((YOLOv8InstantDetection) detection));
   }

   public void updatePoseEstimation(YOLOv8InstantDetection yoloDetection)
   {
      if (state == State.DISABLED)
         return;

      if (newTargetPoint.poll())
      {
         targetPoint.set(newTargetPoint.read());
      }
      else if (latestResult != null)
      {
         targetPoint.set(latestResult.getPose().getPosition());
      }
      else
      {
         targetPoint.set(sensorFrame.getTransformToRoot().getTranslation());
      }

      double resetDistance = parameters.getResetDistance().getValue();

      if (parameters.getAutoResetEnabled().getValue()
          && latestResult != null
          && state == State.TRACKING
          && yoloDetection.getPose()
                          .getPosition()
                          .distanceSquared(latestResult.getPose().getPosition())
             > resetDistance * resetDistance)
      {
         LogTools.info("Auto reset triggered for {}/{}: YOLO detection too far from latest SupervisePose result. resetDistance={}",
                       target.category(),
                       target.instance(),
                       resetDistance);

         resetTracking();
      }

      updatePoseEstimation(yoloDetection.getColorImage(),
                           yoloDetection.getDepthImage(),
                           yoloDetection.getObjectMask());
   }

   public void updatePoseEstimation(RawImage colorImage,
                                    RawImage depthImage,
                                    RawImage segmentation)
   {
      colorImage.get();
      depthImage.get();
      segmentation.get();

      RawImage rgbImage =
            RawImageTools.convertColor(colorImage, PixelFormat.RGB8);

      GpuMat depth32Mat = new GpuMat();
      depthImage.getGpuImageMat()
                .convertTo(depth32Mat,
                           opencv_core.CV_32FC1,
                           depthImage.getDepthDiscretization());

      RawImage depth32FImage =
            depthImage.replaceImage(depth32Mat, PixelFormat.GRAY_F32);

      GpuMat resizedSegmentationMat = new GpuMat();
      YOLOv8Tools.resizeWithCrop(segmentation.getGpuImageMat(),
                                 resizedSegmentationMat,
                                 depth32Mat.size());

      RawImage resizedSegmentation =
            depthImage.replaceImage(resizedSegmentationMat, PixelFormat.GRAY8);

      synchronized (sensorFrame)
      {
         sensorFrame.setNewTransformToParent(colorImage.getTransformToWorld());
         sensorFrame.update();
      }

      ROS2Topic<Image> rgbTopic = topics.rgbImage();
      ROS2Topic<Image> depthTopic = topics.depthImage();
      ROS2Topic<Image> segmentationTopic = topics.segmentation();
      ROS2Topic<CameraInfo> cameraInfoTopic = topics.cameraInfo();

      //  exportSupervisePoseInputs(rgbImage, depth32FImage, resizedSegmentation);

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
      LogTools.info("Publishing reset for {}/{} on {}",
                    target.category(),
                    target.instance(),
                    topics.reset());

      changeState(State.ESTIMATING_POSE);
      resetRequestPublisher.publish(new Empty());
   }

   public SupervisePoseInstantDetection getLatestResult()
   {
      return latestResult;
   }

   public void addResultCallback(Consumer<SupervisePoseInstantDetection> resultCallback)
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

   public State getState()
   {
      return state;
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

   public SyncedSupervisePoseParameters getParameters()
   {
      return parameters;
   }

   private void exportSupervisePoseInputs(RawImage rgbImage,
                                          RawImage depthImage,
                                          RawImage maskImage)
   {
      if (!EXPORT_SUPERVISEPOSE_INPUTS)
         return;

      RawImage rgbExport = null;
      RawImage depthExport = null;
      RawImage maskExport = null;

      Mat bgrMat = null;
      Mat depthU16 = null;

      try
      {
         rgbExport = rgbImage.get();
         depthExport = depthImage.get();
         maskExport = maskImage.get();

         if (rgbExport == null || depthExport == null || maskExport == null)
            return;

         String safeTargetName = sanitize(target.key());

         Path targetDirectory = Path.of(EXPORT_ROOT, safeTargetName);
         Path rgbDirectory = targetDirectory.resolve("rgb");
         Path depthDirectory = targetDirectory.resolve("depth");
         Path maskDirectory = targetDirectory.resolve("mask");
         Path poseDirectory = targetDirectory.resolve("pose");

         Files.createDirectories(rgbDirectory);
         Files.createDirectories(depthDirectory);
         Files.createDirectories(maskDirectory);
         Files.createDirectories(poseDirectory);

         long timestampNs =
               rgbExport.getAcquisitionTime().getEpochSecond() * 1_000_000_000L
               + rgbExport.getAcquisitionTime().getNano();

         String frameName = String.valueOf(timestampNs);

         Mat rgbCpu = rgbExport.getCpuImageMat();
         Mat depthCpu = depthExport.getCpuImageMat();
         Mat maskCpu = maskExport.getCpuImageMat();

         bgrMat = new Mat();
         opencv_imgproc.cvtColor(rgbCpu, bgrMat, opencv_imgproc.COLOR_RGB2BGR);
         opencv_imgcodecs.imwrite(rgbDirectory.resolve(frameName + ".png").toString(), bgrMat);

         depthU16 = new Mat();
         depthCpu.convertTo(depthU16, opencv_core.CV_16UC1, 1000.0, 0.0);
         opencv_imgcodecs.imwrite(depthDirectory.resolve(frameName + ".png").toString(), depthU16);

         opencv_imgcodecs.imwrite(maskDirectory.resolve(frameName + ".png").toString(), maskCpu);

         CameraIntrinsics intrinsics = rgbExport.getIntrinsicsCopy();

         try (PrintWriter writer =
                    new PrintWriter(Files.newBufferedWriter(targetDirectory.resolve("cam_K.txt"))))
         {
            writer.printf(Locale.US,
                          "%.18e %.18e %.18e%n",
                          intrinsics.getFx(),
                          0.0,
                          intrinsics.getCx());

            writer.printf(Locale.US,
                          "%.18e %.18e %.18e%n",
                          0.0,
                          intrinsics.getFy(),
                          intrinsics.getCy());

            writer.printf(Locale.US,
                          "%.18e %.18e %.18e%n",
                          0.0,
                          0.0,
                          1.0);
         }

         SupervisePoseInstantDetection resultToExport = latestResult;

         if (resultToExport != null && rawSupervisePose != null)
         {
            writePoseJson(poseDirectory.resolve(frameName + ".json"),
                          rawSupervisePose,
                          new Vector3D(resultToExport.getBoundingBox().getSize()),
                          intrinsics,
                          rgbExport.getWidth(),
                          rgbExport.getHeight());
         }
      }
      catch (Exception exception)
      {
         LogTools.error("Failed to export SupervisePose inputs for {}",
                        target.key(),
                        exception);
      }
      finally
      {
         if (depthU16 != null)
            depthU16.release();

         if (bgrMat != null)
            bgrMat.release();

         if (maskExport != null)
            maskExport.release();

         if (depthExport != null)
            depthExport.release();

         if (rgbExport != null)
            rgbExport.release();
      }
   }

   private void writePoseJson(Path jsonPath,
                              Pose3DReadOnly pose,
                              Vector3D size,
                              CameraIntrinsics intrinsics,
                              int width,
                              int height) throws Exception
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.set(pose);

      Quaternion quaternion = new Quaternion(pose.getOrientation());

      double x = pose.getPosition().getX();
      double y = pose.getPosition().getY();
      double z = pose.getPosition().getZ();

      try (PrintWriter writer =
                 new PrintWriter(Files.newBufferedWriter(jsonPath)))
      {
         writer.printf(Locale.US, "{%n");
         writer.printf(Locale.US, "  \"camera_data\": {%n");
         writer.printf(Locale.US, "    \"height\": %d,%n", height);
         writer.printf(Locale.US, "    \"width\": %d,%n", width);

         writer.printf(Locale.US,
                       "    \"intrinsics\": {\"fx\": %.9f, \"fy\": %.9f, \"cx\": %.9f, \"cy\": %.9f}%n",
                       intrinsics.getFx(),
                       intrinsics.getFy(),
                       intrinsics.getCx(),
                       intrinsics.getCy());

         writer.printf(Locale.US, "  },%n");
         writer.printf(Locale.US, "  \"objects\": [%n");
         writer.printf(Locale.US, "    {%n");
         writer.printf(Locale.US, "      \"class\": \"%s\",%n", target.category());
         writer.printf(Locale.US, "      \"name\": \"%s\",%n", target.instance());
         writer.printf(Locale.US, "      \"provenance\": \"supervisepose\",%n");

         writer.printf(Locale.US, "      \"transform_matrix\": [%n");

         writer.printf(Locale.US,
                       "        [%.9f, %.9f, %.9f, %.9f],%n",
                       transform.getRotation().getM00(),
                       transform.getRotation().getM01(),
                       transform.getRotation().getM02(),
                       x);

         writer.printf(Locale.US,
                       "        [%.9f, %.9f, %.9f, %.9f],%n",
                       transform.getRotation().getM10(),
                       transform.getRotation().getM11(),
                       transform.getRotation().getM12(),
                       y);

         writer.printf(Locale.US,
                       "        [%.9f, %.9f, %.9f, %.9f],%n",
                       transform.getRotation().getM20(),
                       transform.getRotation().getM21(),
                       transform.getRotation().getM22(),
                       z);

         writer.printf(Locale.US, "        [0.0, 0.0, 0.0, 1.0]%n");
         writer.printf(Locale.US, "      ],%n");

         writer.printf(Locale.US,
                       "      \"location\": [%.9f, %.9f, %.9f],%n",
                       x,
                       y,
                       z);

         writer.printf(Locale.US,
                       "      \"quaternion_xyzw\": [%.9f, %.9f, %.9f, %.9f],%n",
                       quaternion.getX(),
                       quaternion.getY(),
                       quaternion.getZ(),
                       quaternion.getS());

         writer.printf(Locale.US,
                       "      \"scale\": [%.9f, %.9f, %.9f]%n",
                       size.getX(),
                       size.getY(),
                       size.getZ());

         writer.printf(Locale.US, "    }%n");
         writer.printf(Locale.US, "  ]%n");
         writer.printf(Locale.US, "}%n");
      }
   }

   @Override
   public void close()
   {
      parameters.close();

      poseEstimationResultSubscription.remove();
      trackingResultSubscription.remove();
      resetRequestSubscription.remove();

      resetRequestPublisher.remove();
      resultRelayPublisher.remove();
      statePublisher.remove();

      imagePublisher.close();
      ros2Node.destroy();
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
