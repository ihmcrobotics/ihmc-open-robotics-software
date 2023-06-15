package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.*;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.ROS2DetectableSceneNodesPublisher;
import us.ihmc.perception.sceneGraph.ROS2DetectableSceneNodesSubscription;
import us.ihmc.perception.sceneGraph.arUco.ArUcoSceneTools;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;

public class DualBlackflyCamera
{
   private static final double PUBLISH_RATE = 15.0;

   private final RobotSide side;
   private final ROS2SyncedRobotModel syncedRobot;
   private SpinnakerBlackfly spinnakerBlackfly;
   private final ROS2StoredPropertySet<IntrinsicCameraMatrixProperties> ousterFisheyeColoringIntrinsicsROS2;

   private final ROS2Helper ros2Helper;

   private final IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics;

   private final PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;
   private final ROS2TunedRigidBodyTransform remoteTunableCameraTransform;
   private final ROS2DetectableSceneNodesPublisher detectableSceneObjectsPublisher = new ROS2DetectableSceneNodesPublisher();
   private final ROS2DetectableSceneNodesSubscription detectableSceneNodesSubscription;

   private final AtomicReference<GpuMat> latestImageOnGpu = new AtomicReference<>();
   private final AtomicReference<Instant> spinImageAcquisitionTime = new AtomicReference<>();
   private final ImageMessage imageMessage = new ImageMessage();
   private final IHMCROS2Publisher<ImageMessage> ros2ImagePublisher;
   private long imageMessageSequenceNumber = 0;

   // These update when a camera image resolution change is detected
   private int imageWidth = 0;
   private int imageHeight = 0;
   private long numberOfBytesInFrame = 0;
   private GpuMat distortedRGBImage;
   private GpuMat undistortedImage;
   private BytedecoImage undistortedBytedecoImage;
   private GpuMat undistortionMap1;
   private GpuMat undistortionMap2;
   private GpuMat processThreadSourceImage;
   private GpuMat undistortThreadSourceImage;
   private final LinkedList<GpuMat> gpuMatDeque = new LinkedList<>();

   private final Object processThreadSyncObject = new Object();
   private final Object undistortThreadSyncObject = new Object();

   private final CUDAImageEncoder cudaImageEncoder;

   private volatile boolean destroyed = false;

   private long numCreated = 0L;
   private long numReleased = 0L;

   public DualBlackflyCamera(RobotSide side,
                             ROS2SyncedRobotModel syncedRobot,
                             RigidBodyTransform cameraTransformToParent,
                             ROS2Node ros2Node,
                             SpinnakerBlackfly spinnakerBlackfly,
                             IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics,
                             PredefinedSceneNodeLibrary predefinedSceneNodeLibrary)
   {
      this.side = side;
      this.syncedRobot = syncedRobot;
      this.spinnakerBlackfly = spinnakerBlackfly;
      this.ousterFisheyeColoringIntrinsics = ousterFisheyeColoringIntrinsics;
      this.predefinedSceneNodeLibrary = predefinedSceneNodeLibrary;

      cudaImageEncoder = new CUDAImageEncoder();

      ROS2Topic<ImageMessage> imageTopic = PerceptionAPI.BLACKFLY_FISHEYE_COLOR_IMAGE.get(side);
      ros2ImagePublisher = ROS2Tools.createPublisher(ros2Node, imageTopic, ROS2QosProfile.BEST_EFFORT());
      ros2Helper = new ROS2Helper(ros2Node);

      ousterFisheyeColoringIntrinsicsROS2 = new ROS2StoredPropertySet<>(ros2Helper,
                                                                        DualBlackflyComms.OUSTER_FISHEYE_COLORING_INTRINSICS,
                                                                        ousterFisheyeColoringIntrinsics);

      remoteTunableCameraTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                           ROS2Tools.OBJECT_DETECTION_CAMERA_TO_PARENT_TUNING,
                                                                           cameraTransformToParent);

      detectableSceneNodesSubscription = new ROS2DetectableSceneNodesSubscription(predefinedSceneNodeLibrary.getDetectableSceneNodes(),
                                                                                  ros2Helper,
                                                                                  ROS2IOTopicQualifier.COMMAND);

      // Camera read thread
      ThreadTools.startAsDaemon(() ->
      {
         Throttler cameraReadThrottler = new Throttler();
         cameraReadThrottler.setFrequency(PUBLISH_RATE);

         while (!destroyed)
         {
            cameraReadThrottler.waitAndRun();
            cameraRead();

            synchronized (DualBlackflyCamera.this)
            {
               DualBlackflyCamera.this.notifyAll();
            }
         }
      }, "SpinnakerCameraRead");

      ThreadTools.startAThread(() ->
      {
         while (!destroyed)
         {
            try
            {
               synchronized (DualBlackflyCamera.this)
               {
                  DualBlackflyCamera.this.wait();
               }
            }
            catch (InterruptedException e)
            {
               LogTools.error(e);
            }

            // TODO: peekLast isn't working for some reason
            processThreadSourceImage = gpuMatDeque.peekLast();
            imageProcessAndPublish();
         }
      }, "SpinnakerImageProcessAndPublish");

      ThreadTools.startAsDaemon(() ->
      {
         while (!destroyed)
         {
            try
            {
               synchronized (DualBlackflyCamera.this)
               {
                  DualBlackflyCamera.this.wait();
               }
            }
            catch (InterruptedException e)
            {
               LogTools.error(e);
            }

            undistortThreadSourceImage = gpuMatDeque.peekLast();
            undistortImageAndUpdateArUco();
         }
      }, "SpinnakerImageUndistortAndUpdateArUco");

      ThreadTools.startAThread(() ->
      {
         while (!destroyed)
         {
            try
            {
               synchronized (processThreadSyncObject)
               {
                  processThreadSyncObject.wait();
               }
               synchronized (undistortThreadSyncObject)
               {
                  undistortThreadSyncObject.wait();
               }
            }
            catch (InterruptedException e)
            {
               LogTools.error(e);
            }

            GpuMat oldestGpuMat = gpuMatDeque.peekFirst();

            while (!oldestGpuMat.equals(processThreadSourceImage) && !oldestGpuMat.equals(undistortThreadSourceImage))
            {
               GpuMat gpuMat = gpuMatDeque.pop();
               gpuMat.release();
               gpuMat.close();
               ++numReleased;

               oldestGpuMat = gpuMatDeque.peekFirst();
               if (oldestGpuMat == null)
                  break;
            }
         }
      }, "SpinnakerImageDeallocation");
   }

   private void initialize(int imageWidth, int imageHeight)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      numberOfBytesInFrame = (long) imageWidth * imageHeight; // BGR8
      LogTools.info("Blackfly {} resolution detected: {} x {}", spinnakerBlackfly.getSerialNumber(), imageWidth, imageHeight);

      distortedRGBImage = new GpuMat(imageWidth, imageHeight, opencv_core.CV_8UC3);
      undistortedImage = new GpuMat(imageWidth, imageHeight, opencv_core.CV_8UC3);
      undistortedBytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);

      initializeImageUndistortion();
   }

   private void initializeImageUndistortion()
   {
      Mat cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(cameraMatrix);
      cameraMatrix.ptr(0, 0).putDouble(SensorHeadParameters.FOCAL_LENGTH_X_FOR_UNDISORTION);
      cameraMatrix.ptr(1, 1).putDouble(SensorHeadParameters.FOCAL_LENGTH_Y_FOR_UNDISORTION);
      cameraMatrix.ptr(0, 2).putDouble(SensorHeadParameters.PRINCIPAL_POINT_X_FOR_UNDISORTION);
      cameraMatrix.ptr(1, 2).putDouble(SensorHeadParameters.PRINCIPAL_POINT_Y_FOR_UNDISORTION);
      Mat newCameraMatrixEstimate = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(newCameraMatrixEstimate);
      Mat distortionCoefficients = new Mat(SensorHeadParameters.K1_FOR_UNDISORTION,
                                           SensorHeadParameters.K2_FOR_UNDISORTION,
                                           SensorHeadParameters.K3_FOR_UNDISORTION,
                                           SensorHeadParameters.K4_FOR_UNDISORTION);
      Size sourceImageSize = new Size(imageWidth, imageHeight);
      Size undistortedImageSize = new Size((int) (SensorHeadParameters.UNDISTORTED_IMAGE_SCALE * imageWidth),
                                           (int) (SensorHeadParameters.UNDISTORTED_IMAGE_SCALE * imageHeight));
      Mat rectificationTransformation = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(rectificationTransformation);

      double balanceNewFocalLength = 0.0;
      double fovScaleFocalLengthDivisor = 1.0;

      opencv_calib3d.fisheyeEstimateNewCameraMatrixForUndistortRectify(cameraMatrix,
                                                                       distortionCoefficients,
                                                                       sourceImageSize,
                                                                       rectificationTransformation,
                                                                       newCameraMatrixEstimate,
                                                                       balanceNewFocalLength,
                                                                       undistortedImageSize,
                                                                       fovScaleFocalLengthDivisor);
      // Fisheye undistortion
      // https://docs.opencv.org/4.7.0/db/d58/group__calib3d__fisheye.html#ga167df4b00a6fd55287ba829fbf9913b9
      Mat tempUndistortionMat1 = new Mat();
      Mat tempUndistortionMat2 = new Mat();

      opencv_calib3d.fisheyeInitUndistortRectifyMap(cameraMatrix,
                                                    distortionCoefficients,
                                                    rectificationTransformation,
                                                    newCameraMatrixEstimate,
                                                    undistortedImageSize,
                                                    opencv_core.CV_32F,
                                                    tempUndistortionMat1,
                                                    tempUndistortionMat2);

      undistortionMap1 = new GpuMat();
      undistortionMap1.upload(tempUndistortionMat1);
      undistortionMap2 = new GpuMat();
      undistortionMap2.upload(tempUndistortionMat2);

      // Create arUco marker detection
      ReferenceFrame blackflyCameraFrame = syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame();
      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(blackflyCameraFrame);
      arUcoMarkerDetection.setSourceImageForDetection(undistortedBytedecoImage);
      newCameraMatrixEstimate.copyTo(arUcoMarkerDetection.getCameraMatrix());
      arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection, ros2Helper, predefinedSceneNodeLibrary.getArUcoMarkerIDsToSizes());

      // Close pointers
      tempUndistortionMat2.close();
      tempUndistortionMat1.close();
      rectificationTransformation.close();
      undistortedImageSize.close();
      sourceImageSize.close();
      distortionCoefficients.close();
      newCameraMatrixEstimate.close();
      cameraMatrix.close();
   }

   /**
    * Read the latest image from the camera, deallocate
    * the memory for the last image (if it existed), and
    * set the atomic reference to the new image data
    */
   private void cameraRead()
   {
      // Get image from camera
      spinImage spinImage = new spinImage(); // Release at the end
      spinnakerBlackfly.getNextImage(spinImage);

      // Initialize stuff on the first run
      if (imageWidth == 0 || imageHeight == 0)
      {
         initialize(spinnakerBlackfly.getWidth(spinImage), spinnakerBlackfly.getHeight(spinImage));
      }

      // Get pointer to image data
      BytePointer spinImageData = new BytePointer(numberOfBytesInFrame); // close at the end
      spinnakerBlackfly.setPointerToSpinImageData(spinImage, spinImageData);

      // Upload image data to GPU
      BytedecoImage sourceBytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      sourceBytedecoImage.changeAddress(spinImageData.address());
      GpuMat latestImageSetter = new GpuMat(imageWidth, imageHeight, opencv_core.CV_8UC3);
      latestImageSetter.upload(sourceBytedecoImage.getBytedecoOpenCVMat());

      gpuMatDeque.push(latestImageSetter);

      if (numCreated - numReleased > 100)
      {
         throw new RuntimeException("GpuMats are not being released in time");
      }

      numCreated++;

      spinImageAcquisitionTime.set(Instant.now());

      spinImageData.close();
      spinnakerBlackfly.releaseImage(spinImage);
   }

   /**
    * Get the last image read from the camera read thread, undistort fisheye,
    * update arUco marker detection (if right fisheye camera), change pixel formats,
    * apply jpeg compression, and publish over the network
    */
   private void imageProcessAndPublish()
   {
      Instant spinImageAcquisitionTime = this.spinImageAcquisitionTime.get();
      if (processThreadSourceImage == null || spinImageAcquisitionTime == null)
         return;

      ReferenceFrame ousterLidarFrame = syncedRobot.getReferenceFrames().getOusterLidarFrame();
      RigidBodyTransform ousterToBlackflyTransfrom = new RigidBodyTransform();

      if (side == RobotSide.RIGHT)
      {
         ReferenceFrame blackflyCameraFrame = syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame();
         ousterLidarFrame.getTransformToDesiredFrame(ousterToBlackflyTransfrom, blackflyCameraFrame);
      }
      // TODO: left behavior?

      remoteTunableCameraTransform.update();
      syncedRobot.update();

      BytePointer jpegImageBytePointer = new BytePointer(numberOfBytesInFrame); // close at the end

      // Encode the image
      cudaImageEncoder.encodeBGR(processThreadSourceImage, jpegImageBytePointer, imageWidth, imageHeight);

      synchronized (processThreadSyncObject)
      {
         processThreadSyncObject.notify();
      }

      ousterFisheyeColoringIntrinsicsROS2.updateAndPublishThrottledStatus();

      // Create and publish image message
      ImageMessageDataPacker compressedImageDataPacker = new ImageMessageDataPacker(jpegImageBytePointer.limit());
      compressedImageDataPacker.pack(imageMessage, jpegImageBytePointer);
      MessageTools.toMessage(spinImageAcquisitionTime, imageMessage.getAcquisitionTime());
      imageMessage.setImageWidth(imageWidth);
      imageMessage.setImageHeight(imageHeight);
      imageMessage.setFocalLengthXPixels((float) ousterFisheyeColoringIntrinsics.getFocalLengthX());
      imageMessage.setFocalLengthYPixels((float) ousterFisheyeColoringIntrinsics.getFocalLengthY());
      imageMessage.setPrincipalPointXPixels((float) ousterFisheyeColoringIntrinsics.getPrinciplePointX());
      imageMessage.setPrincipalPointYPixels((float) ousterFisheyeColoringIntrinsics.getPrinciplePointY());
      CameraModel.EQUIDISTANT_FISHEYE.packMessageFormat(imageMessage);
      imageMessage.setSequenceNumber(imageMessageSequenceNumber++);
      ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(imageMessage);
      imageMessage.getPosition().set(ousterToBlackflyTransfrom.getTranslation());
      imageMessage.getOrientation().set(ousterToBlackflyTransfrom.getRotation());
      ros2ImagePublisher.publish(imageMessage);

      // Close pointers
      //jpegImageBytePointer.close();
   }

   private void undistortImageAndUpdateArUco()
   {
      if (undistortThreadSourceImage == null)
         return;

      opencv_cudaimgproc.cvtColor(undistortThreadSourceImage, distortedRGBImage, opencv_imgproc.COLOR_BGR2RGB);

      synchronized (undistortThreadSyncObject)
      {
         undistortThreadSyncObject.notify();
      }

      opencv_cudawarping.remap(distortedRGBImage,
                               undistortedImage,
                               undistortionMap1,
                               undistortionMap2,
                               opencv_imgproc.INTER_LINEAR);

      ReferenceFrame ousterLidarFrame = syncedRobot.getReferenceFrames().getOusterLidarFrame();
      RigidBodyTransform ousterToBlackflyTransfrom = new RigidBodyTransform();

      if (side == RobotSide.RIGHT)
      {
         ReferenceFrame blackflyCameraFrame = syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame();
         ousterLidarFrame.getTransformToDesiredFrame(ousterToBlackflyTransfrom, blackflyCameraFrame);

         arUcoMarkerDetection.update();
         // TODO: Maybe publish a separate image for ArUco marker debugging sometime.
         // arUcoMarkerDetection.drawDetectedMarkers(blackflySourceImage.getBytedecoOpenCVMat());
         // arUcoMarkerDetection.drawRejectedPoints(blackflySourceImage.getBytedecoOpenCVMat());
         arUcoMarkerPublisher.update();

         detectableSceneNodesSubscription.update(); // Receive overridden poses from operator
         ArUcoSceneTools.updateLibraryPosesFromDetectionResults(arUcoMarkerDetection, predefinedSceneNodeLibrary);
         detectableSceneObjectsPublisher.publish(predefinedSceneNodeLibrary.getDetectableSceneNodes(), ros2Helper, ROS2IOTopicQualifier.STATUS);
      }
      // TODO: left behavior?

      undistortedImage.download(undistortedBytedecoImage.getBytedecoOpenCVMat());
   }

   public void destroy()
   {
      destroyed = true;
      cudaImageEncoder.destroy();
      if (spinnakerBlackfly != null)
         spinnakerBlackfly.stopAcquiringImages();
      if (arUcoMarkerDetection != null)
         arUcoMarkerDetection.destroy();
      spinnakerBlackfly = null;
      arUcoMarkerDetection = null;
   }
}
