package us.ihmc.perception.detections.supervisePose;

import ihmc_common_msgs.Box3DMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.javacpp.BytePointer;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Byte_;
import std_msgs.Empty;
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
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.sensors.CameraIntrinsics;
import vision_msgs.Detection3D;
import vision_msgs.Detection3DArray;
import perception_msgs.ImageMessage;

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
   private final ROS2Publisher<Byte_> statePublisher;
   private final ROS2Publisher<ImageMessage> overlayImagePublisher;

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
   private volatile boolean internallyPublishingReset = false;
   private final List<Consumer<SupervisePoseInstantDetection>> resultCallbacks = new ArrayList<>();

   private volatile Pose3D rawSupervisePose = null;

   private final SupervisePoseMeshOverlayRenderer meshOverlayRenderer;

   private final Object latestRGBImageLock = new Object();
   private RawImage latestRGBImage = null;

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

      ros2Node = new ROS2Node(getClass().getSimpleName() + "_" + sanitize(target.key()));
      imagePublisher = new RawImagePublisher(ros2Node, 1.0);

      resetRequestPublisher = ros2Node.createPublisher(topics.reset());
      resultRelayPublisher = ros2Node.createPublisher(topics.ihmcResult());
      statePublisher = ros2Node.createPublisher(topics.ihmcState());
      overlayImagePublisher = ros2Node.createPublisher(topics.overlayedImage());

      poseEstimationResultSubscription = ros2Node.createSubscription(topics.poseEstimationOutput(), reader ->
      {
         Detection3DArray message = reader.read();
         if (message != null)
            updateLatestResult(message);
      });

      trackingResultSubscription = ros2Node.createSubscription(topics.trackingOutput(), reader ->
      {
         Detection3DArray message = reader.read();
         if (message != null)
            updateLatestResult(message);
      });

      resetRequestSubscription =
            ros2Node.createSubscription(topics.reset(), message ->
            {
               if (internallyPublishingReset)
               {
                  internallyPublishingReset = false;
                  return;
               }

               LogTools.info(String.format(
                     "Resetting SupervisePose for %s/%s. Source=UI_OR_EXTERNAL",
                     target.category(),
                     target.instance()));

               changeState(State.ESTIMATING_POSE);
            });

      SupervisePoseObject object = SupervisePoseObject.fromCategoryAndInstance(target.category(), target.instance());

      Path meshPath = object.getMeshPath();

      meshOverlayRenderer = new SupervisePoseMeshOverlayRenderer(meshPath);

      parameters = new SyncedSupervisePoseParameters(ros2Node, crdtInfo, object);
      parameters.getEnabled().setValue(false);

      state = State.DISABLED;
      previousState = state;

      sensorFrame = new ROS2MutableFrame(target.instance() + "_ImageFrame", ReferenceFrame.getWorldFrame());
   }

   private static String sanitize(String value)
   {
      return value.replace('/', '_');
   }

   public SupervisePoseTarget getTarget()
   {
      return target;
   }

   public enum ResetReason
   {
      UI,
      YOLO_POSITION,
      ENABLED,
      UNKNOWN
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
         resetTracking(ResetReason.ENABLED);
      }
   }

   private void updateLatestResult(Detection3DArray results)
   {
      if (results.getDetections().isEmpty())
         return;

      Detection3D result = results.getDetections().get(0);
      if (result == null)
         return;

      FramePose3D poseInWorld = new FramePose3D(sensorFrame, result.getBbox().getCenter().getPose());

      Pose3D poseSnapshot = new Pose3D(result.getBbox().getCenter().getPose());

      rawSupervisePose = poseSnapshot;

      poseInWorld.prependRotation(SUPERVISE_POSE_TO_IHMC_ROTATION);

      synchronized (sensorFrame)
      {
         poseInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      }

      latestResult = new SupervisePoseInstantDetection(target, new Box3D(poseInWorld, result.getBbox().getSize().getVector()), Instant.now());

      if (state != State.TRACKING)
         changeState(State.TRACKING);

      // LogTools.info("Received SupervisePose result for {}/{}", target.category(), target.instance());

      Box3DMessage resultRelayMessage = new Box3DMessage();
      resultRelayMessage.getPose().set(poseInWorld);
      resultRelayMessage.getSize().set(result.getBbox().getSize().getVector());
      resultRelayPublisher.publish(resultRelayMessage);

      for (Consumer<SupervisePoseInstantDetection> resultCallback : resultCallbacks)
         resultCallback.accept(latestResult);

      publishPoseDrivenOverlay(poseSnapshot);
   }

   private void publishPoseDrivenOverlay(Pose3D poseSnapshot)
   {
      RawImage imageSnapshot;

      synchronized (latestRGBImageLock)
      {
         if (latestRGBImage == null)
            return;

         /*
          * Retain a local reference so the cache can safely be replaced by the
          * camera thread while this method renders and publishes.
          */
         imageSnapshot = latestRGBImage.get();
      }

      try
      {
         Mat sourceImage = imageSnapshot.getCpuImageMat();

         if (sourceImage == null || sourceImage.isNull())
            return;

         try (Mat overlayImage = sourceImage.clone();
              BytePointer encodedImage = new BytePointer())
         {
            CameraIntrinsics cameraIntrinsics =
                  imageSnapshot.getIntrinsicsCopy();

            meshOverlayRenderer.renderWireframe(overlayImage,
                                                poseSnapshot,
                                                cameraIntrinsics);

            boolean encoded =
                  opencv_imgcodecs.imencode(".jpg",
                                            overlayImage,
                                            encodedImage);

            if (!encoded)
            {
               LogTools.error("Failed to encode mesh overlay for {}/{}",
                              target.category(),
                              target.instance());
               return;
            }

            ImageMessage imageMessage = new ImageMessage();

            PerceptionMessageTools.packImageMessage(imageSnapshot,
                                                    encodedImage,
                                                    CompressionType.JPEG,
                                                    imageMessage);

            overlayImagePublisher.publish(imageMessage);
         }
      }
      catch (Exception exception)
      {
         LogTools.error("Failed to publish mesh overlay for {}/{}: {}",
                        target.category(),
                        target.instance(),
                        exception.getMessage());
      }
      finally
      {
         imageSnapshot.release();
      }
   }

   private synchronized void changeState(State newState)
   {
      if (state == newState)
         return;

      previousState = state;
      state = newState;

      Byte_ stateMessage = new Byte_();
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

      updatePoseEstimation(yoloDetection.getColorImage(),
                           yoloDetection.getDepthImage(),
                           yoloDetection.getObjectMask());
   }

   private void cacheLatestRGBImage(RawImage rgbImage)
   {
      synchronized (latestRGBImageLock)
      {
         if (latestRGBImage != null)
            latestRGBImage.release();

         /*
          * Retain a separate reference because the local rgbImage reference is
          * released at the end of updatePoseEstimation(...).
          */
         latestRGBImage = rgbImage.get();
      }
   }

   public void updatePoseEstimation(RawImage colorImage, RawImage depthImage, RawImage segmentation)
   {
      colorImage.get();
      depthImage.get();
      segmentation.get();

      cacheLatestRGBImage(colorImage);

      RawImage rgbImage = RawImageTools.convertColor(colorImage, PixelFormat.RGB8);

      GpuMat depth32Mat = new GpuMat();

      depthImage.getGpuImageMat().convertTo(depth32Mat, opencv_core.CV_32FC1, depthImage.getDepthDiscretization());

      RawImage depth32FImage = depthImage.replaceImage(depth32Mat, PixelFormat.GRAY_F32);

      GpuMat resizedSegmentationMat = new GpuMat();
      YOLOv8Tools.resizeWithCrop(segmentation.getGpuImageMat(), resizedSegmentationMat, depth32Mat.size());

      RawImage resizedSegmentation = depthImage.replaceImage(resizedSegmentationMat, PixelFormat.GRAY8);

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

   public void updateTrackingInputs(RawImage colorImage, RawImage depthImage)
   {
      colorImage.get();
      depthImage.get();

      RawImage rgbImage = null;
      RawImage depth32FImage = null;
      GpuMat depth32Mat = null;

      try
      {
         cacheLatestRGBImage(colorImage);

         rgbImage = RawImageTools.convertColor(colorImage, PixelFormat.RGB8);

         depth32Mat = new GpuMat();
         depthImage.getGpuImageMat().convertTo(depth32Mat, opencv_core.CV_32FC1, depthImage.getDepthDiscretization());

         depth32FImage = depthImage.replaceImage(depth32Mat, PixelFormat.GRAY_F32);

         synchronized (sensorFrame)
         {
            sensorFrame.setNewTransformToParent(colorImage.getTransformToWorld());
            sensorFrame.update();
         }

         imagePublisher.publishImage(topics.rgbImage(), rgbImage, sensorFrame);
         imagePublisher.publishImage(topics.depthImage(), depth32FImage, sensorFrame);
         imagePublisher.publishImage(topics.cameraInfo(), rgbImage, sensorFrame);
      }
      finally
      {
         if (depth32FImage != null)
            depth32FImage.release();

         if (rgbImage != null)
            rgbImage.release();

         depthImage.release();
         colorImage.release();
      }
   }

   public boolean checkTrackingReset(YOLOv8InstantDetection yoloDetection)
   {
      if (state != State.TRACKING)
         return false;

      if (!parameters.getAutoResetEnabled().getValue())
         return false;

      if (latestResult == null)
         return false;

      double resetDistance = parameters.getResetDistance().getValue();

      double distanceSquared =
            yoloDetection.getPose()
                         .getPosition()
                         .distanceSquared(
                               latestResult.getPose().getPosition());

      if (distanceSquared <= resetDistance * resetDistance)
         return false;

      LogTools.info(String.format(
            "Auto reset triggered for %s/%s: "
            + "YOLO detection differs from the tracked pose. "
            + "distance=%.4f, resetDistance=%.4f",
            target.category(),
            target.instance(),
            Math.sqrt(distanceSquared),
            resetDistance));

      resetTracking(ResetReason.YOLO_POSITION);
      return true;
   }

   public void resetTracking(ResetReason reason)
   {
      LogTools.info(String.format("Resetting SupervisePose for %s/%s. Source=%s", target.category(), target.instance(), reason));

      changeState(State.ESTIMATING_POSE);
      internallyPublishingReset = true;
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

   private void exportSupervisePoseInputs(RawImage rgbImage, RawImage depthImage, RawImage maskImage)
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

         long timestampNs = rgbExport.getAcquisitionTime().getEpochSecond() * 1_000_000_000L + rgbExport.getAcquisitionTime().getNano();

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

   private void writePoseJson(Path jsonPath, Pose3DReadOnly pose, Vector3D size, CameraIntrinsics intrinsics, int width, int height) throws Exception
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

   public void renderMeshOverlay(Mat image,
                                 CameraIntrinsics cameraIntrinsics)
   {
      if (image == null || image.isNull())
         return;

      if (cameraIntrinsics == null)
         return;

      if (!isEnabled() || state != State.TRACKING)
         return;

      Pose3D poseSnapshot = rawSupervisePose;

      if (poseSnapshot == null)
         return;

      meshOverlayRenderer.renderWireframe(
            image,
            new Pose3D(poseSnapshot),
            cameraIntrinsics);
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
      ros2Node.destroyPublisher(overlayImagePublisher);

      synchronized (latestRGBImageLock)
      {
         if (latestRGBImage != null)
         {
            latestRGBImage.release();
            latestRGBImage = null;
         }
      }

      meshOverlayRenderer.destroy();

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
