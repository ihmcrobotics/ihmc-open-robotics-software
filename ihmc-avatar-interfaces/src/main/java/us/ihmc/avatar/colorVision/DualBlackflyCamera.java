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
   private static final double PUBLISH_RATE = 20.0;

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

   private final AtomicReference<Instant> spinImageAcquisitionTime = new AtomicReference<>();
   private final ImageMessage imageMessage = new ImageMessage();
   private final IHMCROS2Publisher<ImageMessage> ros2ImagePublisher;
   private long imageMessageSequenceNumber = 0;

   private final CUDAImageEncoder cudaImageEncoder;

   // These update when a camera image resolution change is detected
   private int imageWidth = 0;
   private int imageHeight = 0;
   private long imageFrameSize = 0;
   private GpuMat distortedRGBImage;
   private GpuMat undistortedImage;
   private BytedecoImage undistortedBytedecoImage;
   private GpuMat undistortionMap1;
   private GpuMat undistortionMap2;

   // These are used to ensure safe deallocation of images
   private GpuMat sourceImageBeingUsedForProcessing;
   private GpuMat sourceImageBeingUsedForUndistortion;
   private final LinkedList<GpuMat> receivedImagesDeque = new LinkedList<>(); /* deque of images received from the blackfly camera.
                                                                               * last element = newest image received
                                                                               * first element = oldest image received */
   private final Object processThreadSyncObject = new Object();   // sync objects are used to synchronize deallocation of images (GpuMats)
   private final Object undistortThreadSyncObject = new Object(); // notify the deallocation thread when a thread is finished using a source image
   private long numberOfGpuMatsAllocated = 0L;   // keep count of images allocated and deallocated
   private long numberOfGpuMatsDeallocated = 0L; // an exception is thrown if more than 100 images are allocated at a time

   private volatile boolean destroyed = false;

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

      ROS2Topic<ImageMessage> imageTopic = PerceptionAPI.BLACKFLY_FISHEYE_COLOR_IMAGE.get(side);
      ros2ImagePublisher = ROS2Tools.createPublisher(ros2Node, imageTopic, ROS2QosProfile.BEST_EFFORT());
      ros2Helper = new ROS2Helper(ros2Node);
      cudaImageEncoder = new CUDAImageEncoder();

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
      ThreadTools.startAThread(() ->
      {
         Throttler cameraReadThrottler = new Throttler();
         cameraReadThrottler.setFrequency(PUBLISH_RATE);

         while (!destroyed)
         {
            // Read an image at PUBLISH_RATE hz
            cameraReadThrottler.waitAndRun();
            cameraRead();

            // Notify threads when a new image is available
            synchronized (DualBlackflyCamera.this)
            {
               DualBlackflyCamera.this.notifyAll();
            }
         }
      }, "SpinnakerCameraRead");

      // Image process and publish thread
      ThreadTools.startAsDaemon(() ->
      {
         while (!destroyed)
         {
            // Wait until new image is available
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

            // Process and publish the new image
            imageProcessAndPublish();
         }
      }, "SpinnakerImageProcessAndPublish");

      // Undistort image and update aruco thread
      ThreadTools.startAsDaemon(() ->
      {
         while (!destroyed)
         {
            // Wait until new image is available
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

            // Undistort  the new image
            undistortImageAndUpdateArUco();
         }
      }, "SpinnakerImageUndistortAndUpdateArUco");

      // Deallocation thread
      ThreadTools.startAsDaemon(() ->
      {
         while (!destroyed)
         {
            // Wait until process and undistort threads are done using the newest image
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

            // Get the oldest image in the deque
            GpuMat oldestGpuMat = receivedImagesDeque.peekFirst();
            if (oldestGpuMat == null)
               continue;

            // While the threads aren't using the oldest image
            while (!oldestGpuMat.equals(sourceImageBeingUsedForProcessing) && !oldestGpuMat.equals(sourceImageBeingUsedForUndistortion))
            {
               // Deallocate the oldest image
               GpuMat gpuMat = receivedImagesDeque.pollFirst();
               gpuMat.release();
               gpuMat.close();
               ++numberOfGpuMatsDeallocated;

               // Get next oldest image
               oldestGpuMat = receivedImagesDeque.peekFirst();
               if (oldestGpuMat == null)
                  break;
            }
         }
      }, "SpinnakerImageDeallocation");
   }

   /**
    * Initializes variables that depend on image width and height.
    * @param imageWidth the width, in pixels, of the image received
    * @param imageHeight the height, in pixels, of the image received
    */
   private void initialize(int imageWidth, int imageHeight)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      imageFrameSize = (long) imageWidth * imageHeight;
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
      BytePointer spinImageData = new BytePointer(imageFrameSize); // close at the end
      spinnakerBlackfly.setPointerToSpinImageData(spinImage, spinImageData);

      // Upload image data to GPU
      BytedecoImage sourceBytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      sourceBytedecoImage.changeAddress(spinImageData.address());
      GpuMat latestImageSetter = new GpuMat(imageWidth, imageHeight, opencv_core.CV_8UC3);
      latestImageSetter.upload(sourceBytedecoImage.getBytedecoOpenCVMat());

      // Add image to deque
      receivedImagesDeque.offerLast(latestImageSetter);

      // Ensure images are being deallocated
      if (numberOfGpuMatsAllocated - numberOfGpuMatsDeallocated > 100)
      {
         throw new RuntimeException("GpuMats are not being released in time");
      }
      numberOfGpuMatsAllocated++;

      spinImageAcquisitionTime.set(Instant.now());

      // Close pointers
      spinImageData.close();
      spinnakerBlackfly.releaseImage(spinImage);
   }

   /**
    * Get the last image read from the camera read thread, apply jpeg compression, and publish over the network
    */
   private void imageProcessAndPublish()
   {
      // Get newest image and ensure it's not null
      sourceImageBeingUsedForProcessing = receivedImagesDeque.peekLast();
      Instant spinImageAcquisitionTime = this.spinImageAcquisitionTime.get();
      if (sourceImageBeingUsedForProcessing == null || spinImageAcquisitionTime == null)
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

      BytePointer jpegImageBytePointer = new BytePointer(imageFrameSize); // close at the end

      // Encode the image
      cudaImageEncoder.encodeBGR(sourceImageBeingUsedForProcessing.data(), jpegImageBytePointer, imageWidth, imageHeight, sourceImageBeingUsedForProcessing.step());

      // Notify deallocation thread that the images has been processed, and is ready for deallocation
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
      jpegImageBytePointer.close();
   }

   /**
    * undistort fisheye, update arUco marker detection (if right fisheye camera)
    */
   private void undistortImageAndUpdateArUco()
   {
      // Get newest image and ensure it's not null
      sourceImageBeingUsedForUndistortion = receivedImagesDeque.peekLast();
      if (sourceImageBeingUsedForUndistortion == null)
         return;

      // Convert color from BGR to RGB
      opencv_cudaimgproc.cvtColor(sourceImageBeingUsedForUndistortion, distortedRGBImage, opencv_imgproc.COLOR_BGR2RGB);

      // Notify deallocation thread that the image has been used, and is ready for deallocation
      synchronized (undistortThreadSyncObject)
      {
         undistortThreadSyncObject.notify();
      }

      opencv_cudawarping.remap(distortedRGBImage,
                               undistortedImage,
                               undistortionMap1,
                               undistortionMap2,
                               opencv_imgproc.INTER_LINEAR);

      if (side == RobotSide.RIGHT)
      {
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
