package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.*;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.sceneGraph.arUco.ArUcoSceneTools;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
import us.ihmc.tools.time.FrequencyCalculator;

import java.time.Instant;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class DualBlackflyCamera
{
   private static final boolean ENABLE_ARUCO_MARKER_DETECTION = true;
   private static final boolean DEBUG_SHUTDOWN = false;

   private final RobotSide side;
   private final Supplier<HumanoidReferenceFrames> humanoidReferenceFramesSupplier;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ROS2Helper ros2Helper;

   private SpinnakerBlackfly spinnakerBlackfly;
   private final BlackflyLensProperties blackflyLensProperties;
   private final ROS2SceneGraph sceneGraph;

   private final FrequencyCalculator readFrequencyCalculator = new FrequencyCalculator();
   private final Timer printCameraReadRateTimer = new Timer();

   private final ROS2StoredPropertySet<IntrinsicCameraMatrixProperties> ousterFisheyeColoringIntrinsicsROS2;

   private final IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics;

   private final MutableReferenceFrame blackflyFrameForSceneNodeUpdate = new MutableReferenceFrame();
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;
   private final ROS2TunedRigidBodyTransform remoteTunableCameraTransform;

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

   /**
    * Deque of images received from the blackfly camera.
    * last element = newest image received
    * first element = oldest image received
    */
   private final LinkedList<GpuMat> receivedImagesDeque = new LinkedList<>();
   private final Object newImageReadNotifier = new Object();
   /**
    * Sync objects are used to synchronize deallocation of images (GpuMats)
    */
   private final Object encodingCompletionNotifier = new Object();
   /**
    * Notify the deallocation thread when a thread is finished using a source image
    */
   private final Object undistortionCompletionNotifier = new Object();
   /**
    * Keep count of images allocated and deallocated
    */
   private long numberOfGpuMatsAllocated = 0L;
   private long numberOfGpuMatsDeallocated = 0L;

   // Threads
   private final Thread cameraReadThread;
   private final Thread imageEncodeAndPublishThread;
   private final Thread imageUndistortAndUpdateArUcoThread;
   private final Thread imageDeallocationThread;
   private volatile boolean destroyed = false;

   public DualBlackflyCamera(DRCRobotModel robotModel,
                             RobotSide side,
                             Supplier<HumanoidReferenceFrames> referenceFramesSupplier,
                             ROS2Node ros2Node,
                             SpinnakerBlackfly spinnakerBlackfly,
                             BlackflyLensProperties blackflyLensProperties,
                             ROS2SceneGraph sceneGraph)
   {
      this.side = side;
      this.humanoidReferenceFramesSupplier = referenceFramesSupplier;

      this.spinnakerBlackfly = spinnakerBlackfly;
      this.blackflyLensProperties = blackflyLensProperties;
      this.ousterFisheyeColoringIntrinsics = SensorHeadParameters.loadOusterFisheyeColoringIntrinsicsOnRobot(blackflyLensProperties);
      this.sceneGraph = sceneGraph;

      ROS2Topic<ImageMessage> imageTopic = PerceptionAPI.BLACKFLY_FISHEYE_COLOR_IMAGE.get(side);
      ros2ImagePublisher = ROS2Tools.createPublisher(ros2Node, imageTopic, ROS2QosProfile.BEST_EFFORT());
      ros2Helper = new ROS2Helper(ros2Node);
      cudaImageEncoder = new CUDAImageEncoder();

      ousterFisheyeColoringIntrinsicsROS2 = new ROS2StoredPropertySet<>(ros2Helper,
                                                                        DualBlackflyComms.OUSTER_FISHEYE_COLORING_INTRINSICS,
                                                                        ousterFisheyeColoringIntrinsics);

      RigidBodyTransform cameraTransformToParent = robotModel.getSensorInformation().getSituationalAwarenessCameraTransform(side);

      remoteTunableCameraTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                           PerceptionAPI.SITUATIONAL_AWARENESS_CAMERA_TO_PARENT_TUNING.get(side),
                                                                           cameraTransformToParent);

      // Camera read thread
      cameraReadThread = new Thread(() ->
      {
         printCameraReadRateTimer.reset();

         while (!destroyed)
         {
            // Do not sleep in this thread, read as fast as the camera can supply frames
            cameraRead();

            // Calculate read frequency (maybe it's lower than MAX_IMAGE_READ_FREQUENCY)
            readFrequencyCalculator.ping();

            // Every so often show the read frequency in console
            if (printCameraReadRateTimer.getElapsedTime() > 10.0)
            {
               LogTools.info(side  + " Blackfly reading at " + String.format("%.2f", getReadFrequency()) + "hz");
               printCameraReadRateTimer.reset();
            }

            // Notify threads when a new image is available
            synchronized (newImageReadNotifier)
            {
               newImageReadNotifier.notifyAll();
            }
         }
         if (DEBUG_SHUTDOWN)
            System.out.println("Camera read thread has finished");
      }, "SpinnakerCameraRead");

      // Image process and publish thread
      imageEncodeAndPublishThread = new Thread(() ->
      {
         while (!destroyed)
         {
            // Wait until new image is available
            try
            {
               synchronized (newImageReadNotifier)
               {
                  newImageReadNotifier.wait();
               }
            }
            catch (InterruptedException interruptedException)
            {
               LogTools.error(interruptedException);
            }

            // Process and publish the new image
            imageProcessAndPublish();
         }
         if (DEBUG_SHUTDOWN)
            System.out.println("Image encode and publish thread has finished");
      }, "SpinnakerImageEncodeAndPublish");

      // Undistort image and update aruco thread
      imageUndistortAndUpdateArUcoThread = new Thread(() ->
      {
         while (!destroyed)
         {
            // Wait until new image is available
            try
            {
               synchronized (newImageReadNotifier)
               {
                  newImageReadNotifier.wait();
               }
            }
            catch (InterruptedException interruptedException)
            {
               LogTools.error(interruptedException);
            }

            // Undistort the new image
            undistortImageAndUpdateArUco();
         }
         if (DEBUG_SHUTDOWN)
            System.out.println("Image undistort and update ArUco thread has finished");
      }, "SpinnakerImageUndistortAndUpdateArUco");

      // Deallocation thread
      imageDeallocationThread = new Thread(() ->
      {
         while (!destroyed)
         {
            // Wait until process and undistort threads are done using the newest image
            try
            {
               synchronized (encodingCompletionNotifier)
               {
                  encodingCompletionNotifier.wait(500);
               }
               synchronized (undistortionCompletionNotifier)
               {
                  undistortionCompletionNotifier.wait(500);
               }
            }
            catch (InterruptedException interruptedException)
            {
               LogTools.error(interruptedException);
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
         if (DEBUG_SHUTDOWN)
            System.out.println("Deallocation thread has finished");
      }, "SpinnakerImageDeallocation");

      cameraReadThread.start();
      imageEncodeAndPublishThread.start();
      if (ENABLE_ARUCO_MARKER_DETECTION)
         imageUndistortAndUpdateArUcoThread.start();
      imageDeallocationThread.start();
   }

   /**
    * Initializes variables that depend on image width and height.
    *
    * @param imageWidth  the width, in pixels, of the image received
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
      cameraMatrix.ptr(0, 0).putDouble(blackflyLensProperties.getFocalLengthXForUndistortion());
      cameraMatrix.ptr(1, 1).putDouble(blackflyLensProperties.getFocalLengthYForUndistortion());
      cameraMatrix.ptr(0, 2).putDouble(blackflyLensProperties.getPrincipalPointXForUndistortion());
      cameraMatrix.ptr(1, 2).putDouble(blackflyLensProperties.getPrincipalPointYForUndistortion());
      Mat newCameraMatrixEstimate = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(newCameraMatrixEstimate);
      Mat distortionCoefficients = new Mat(blackflyLensProperties.getK1ForUndistortion(),
                                           blackflyLensProperties.getK2ForUndistortion(),
                                           blackflyLensProperties.getK3ForUndistortion(),
                                           blackflyLensProperties.getK4ForUndistortion());
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
      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(blackflyFrameForSceneNodeUpdate.getReferenceFrame());
      arUcoMarkerDetection.setSourceImageForDetection(undistortedBytedecoImage);
      newCameraMatrixEstimate.copyTo(arUcoMarkerDetection.getCameraMatrix());
      arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection, ros2Helper, sceneGraph.getArUcoMarkerIDToNodeMap());

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

      // Get camera frame
      synchronized (blackflyFrameForSceneNodeUpdate)
      {
         blackflyFrameForSceneNodeUpdate.update(transformToParent ->
            transformToParent.set(humanoidReferenceFramesSupplier.get().getSituationalAwarenessCameraFrame(side).getTransformToWorldFrame()));
      }

      // Get camera pose
      synchronized (cameraPose)
      {
         cameraPose.setToZero(humanoidReferenceFramesSupplier.get().getSituationalAwarenessCameraFrame(side));
         cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
      }

      // Initialize stuff on the first run
      if (imageWidth == 0 || imageHeight == 0)
      {
         initialize(spinnakerBlackfly.getWidth(spinImage), spinnakerBlackfly.getHeight(spinImage));
      }

      // Get image data
      BytePointer spinImageData = new BytePointer(imageFrameSize); // close at the end
      spinnakerBlackfly.setPointerToSpinImageData(spinImage, spinImageData);
      BytedecoImage sourceBytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);
      sourceBytedecoImage.changeAddress(spinImageData.address());

      // Upload image to GPU
      GpuMat deviceSourceImage = new GpuMat(imageWidth, imageHeight, opencv_core.CV_8UC1);
      deviceSourceImage.upload(sourceBytedecoImage.getBytedecoOpenCVMat());

      // Convert from BayerRG8 to BGR for further conversion
      GpuMat latestImageSetter = new GpuMat(imageWidth, imageHeight, opencv_core.CV_8UC3);
      opencv_cudaimgproc.cvtColor(deviceSourceImage, latestImageSetter, opencv_imgproc.COLOR_BayerBG2BGR); // for some reason you need to convert BayerBG to BGR

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
      deviceSourceImage.release();
      deviceSourceImage.close();
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

      remoteTunableCameraTransform.update();

      BytePointer jpegImageBytePointer = new BytePointer(imageFrameSize); // close at the end

      // Encode the image
      cudaImageEncoder.encodeBGR(sourceImageBeingUsedForProcessing.data(),
                                 jpegImageBytePointer,
                                 imageWidth,
                                 imageHeight,
                                 sourceImageBeingUsedForProcessing.step());

      // Notify deallocation thread that the images has been processed, and is ready for deallocation
      synchronized (encodingCompletionNotifier)
      {
         encodingCompletionNotifier.notify();
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
      synchronized (cameraPose)
      {
         imageMessage.getPosition().set(cameraPose.getTranslation());
         imageMessage.getOrientation().set(cameraPose.getRotation());
      }
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
      synchronized (undistortionCompletionNotifier)
      {
         undistortionCompletionNotifier.notify();
      }

      opencv_cudawarping.remap(distortedRGBImage, undistortedImage, undistortionMap1, undistortionMap2, opencv_imgproc.INTER_LINEAR);

      undistortedImage.download(undistortedBytedecoImage.getBytedecoOpenCVMat());

      if (side == RobotSide.RIGHT)
      {
         arUcoMarkerDetection.update();
         // TODO: Maybe publish a separate image for ArUco marker debugging sometime.
//          arUcoMarkerDetection.drawDetectedMarkers(undistortedBytedecoImage.getBytedecoOpenCVMat());
//          arUcoMarkerDetection.drawRejectedPoints(undistortedBytedecoImage.getBytedecoOpenCVMat());
         arUcoMarkerPublisher.update();

         sceneGraph.updateSubscription(); // Receive overridden poses from operator
         ArUcoSceneTools.updateSceneGraph(arUcoMarkerDetection, sceneGraph);
         synchronized (blackflyFrameForSceneNodeUpdate)
         {
            sceneGraph.updateOnRobotOnly(blackflyFrameForSceneNodeUpdate.getReferenceFrame());
         }
         sceneGraph.updatePublication();
      }
      // TODO: left behavior?
   }

   public void destroy()
   {
      if (DEBUG_SHUTDOWN)
         System.out.println("Destroying dual blackfly camera");
      destroyed = true;

      // Notify deallocation thread to ensure it doesn't hang
      synchronized (encodingCompletionNotifier)
      {
         encodingCompletionNotifier.notify();
      }
      synchronized (undistortionCompletionNotifier)
      {
         undistortionCompletionNotifier.notify();
      }

      // Wait until threads finish their loops
      try
      {
         cameraReadThread.join();
         imageEncodeAndPublishThread.join();
         if (ENABLE_ARUCO_MARKER_DETECTION)
            imageUndistortAndUpdateArUcoThread.join();
         imageDeallocationThread.join();
      }
      catch (InterruptedException interruptedException)
      {
         LogTools.error(interruptedException);
      }

      // Destroy objects
      cudaImageEncoder.destroy();
      if (spinnakerBlackfly != null)
         spinnakerBlackfly.stopAcquiringImages();
      if (arUcoMarkerDetection != null)
         arUcoMarkerDetection.destroy();
      spinnakerBlackfly = null;
      arUcoMarkerDetection = null;
   }

   /**
    * Get the rate at which images are being read from the camera. Changes over time.
    * @return the frequency (hz)
    */
   public double getReadFrequency()
   {
      return readFrequencyCalculator.getFrequency();
   }
}
