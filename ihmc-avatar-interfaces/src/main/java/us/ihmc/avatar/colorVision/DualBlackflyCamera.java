package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
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
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.comms.ImageMessageFormat;
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
import java.util.concurrent.atomic.AtomicReference;

public class DualBlackflyCamera
{
   private static final IntPointer COMPRESSION_PARAMETERS = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);
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

   private final AtomicReference<BytePointer> spinImageData = new AtomicReference<>();
   private final AtomicReference<Instant> spinImageAcquisitionTime = new AtomicReference<>();
   private final ImageMessage imageMessage = new ImageMessage();
   private final IHMCROS2Publisher<ImageMessage> ros2ImagePublisher;
   private long imageMessageSequenceNumber = 0;
   private final BytePointer jpegImageBytePointer = new BytePointer(imageMessage.getData().capacity());
   private final ImageMessageDataPacker compressedImageDataPacker = new ImageMessageDataPacker(jpegImageBytePointer.capacity());

   // These update when a camera image resolution change is detected
   private int imageWidth = 0;
   private int imageHeight = 0;
   private long numberOfBytesInFrame = 0;
   private BytedecoImage blackflySourceImage;
   private Mat yuv420Image;
   private Mat rgbaMat;
   private BytedecoImage undistortedImage;
   private Mat distortedRGBImage;
   private Mat undistortionMap1;
   private Mat undistortionMap2;
   private Scalar undistortionRemapBorderValue;

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
      ThreadTools.startAsDaemon(new Runnable()
      {
         @Override
         public void run()
         {
            while (!destroyed)
            {
               cameraRead();
            }
         }
      }, "SpinnakerCameraRead");

      // Image processing and publishing thread
      ThreadTools.startAThread(new Runnable()
      {
         private final Throttler throttler = new Throttler();

         @Override
         public void run()
         {
            throttler.setFrequency(PUBLISH_RATE);

            while (!destroyed)
            {
               imageProcessAndPublish();
               throttler.waitAndRun();
            }
         }
      }, "SpinnakerImageProcessAndPublish");
   }

   private void changeResolution(int imageWidth, int imageHeight)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      numberOfBytesInFrame = (long) imageWidth * imageHeight; // BayerRG8

      // Setup basic color images
      {
         blackflySourceImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);
         yuv420Image = new Mat(imageHeight * 1.5, imageWidth, opencv_core.CV_8UC1);
         rgbaMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4); // Mat for color conversion
      }

      // Setup distortion images
      {
         undistortedImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
         distortedRGBImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3);

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

         undistortionMap1 = new Mat();
         undistortionMap2 = new Mat();
         undistortionRemapBorderValue = new Scalar();

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
         opencv_calib3d.fisheyeInitUndistortRectifyMap(cameraMatrix,
                                                       distortionCoefficients,
                                                       rectificationTransformation,
                                                       newCameraMatrixEstimate,
                                                       undistortedImageSize,
                                                       opencv_core.CV_16SC2,
                                                       undistortionMap1,
                                                       undistortionMap2);

         // Create arUco marker detection
         ReferenceFrame blackflyCameraFrame = syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame();
         arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
         arUcoMarkerDetection.create(blackflyCameraFrame);
         arUcoMarkerDetection.setSourceImageForDetection(undistortedImage);
         newCameraMatrixEstimate.copyTo(arUcoMarkerDetection.getCameraMatrix());
         arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection, ros2Helper, predefinedSceneNodeLibrary.getArUcoMarkerIDsToSizes());
      }
   }

   /**
    * Read the latest image from the camera, deallocate
    * the memory for the last image (if it existed), and
    * set the atomic reference to the new image data
    */
   private void cameraRead()
   {
      spinImage spinImage = new spinImage(); // Release at the end

      spinnakerBlackfly.getNextImage(spinImage);

      if (this.imageWidth == 0 || this.imageHeight == 0)
      {
         int imageWidth = spinnakerBlackfly.getWidth(spinImage);
         int imageHeight = spinnakerBlackfly.getHeight(spinImage);
         LogTools.info("Blackfly {} resolution detected: {} x {}", spinnakerBlackfly.getSerialNumber(), imageWidth, imageHeight);
         changeResolution(imageWidth, imageHeight);
      }

      BytePointer spinImageData = new BytePointer(numberOfBytesInFrame);
      spinnakerBlackfly.setPointerToSpinImageData(spinImage, spinImageData);

      BytePointer lastSpinImageData = this.spinImageData.getAndSet(spinImageData);
      if (lastSpinImageData != null)
         lastSpinImageData.deallocate();

      this.spinImageAcquisitionTime.set(Instant.now());

      spinnakerBlackfly.releaseImage(spinImage);
   }

   /**
    * Get the last image read from the camera read thread, undistort fisheye,
    * update arUco marker detection (if right fisheye camera), change pixel formats,
    * apply jpeg compression, and publish over the network
    */
   private void imageProcessAndPublish()
   {
      BytePointer spinImageData = this.spinImageData.get();
      Instant spinImageAcquisitionTime = this.spinImageAcquisitionTime.get();

      if (spinImageData == null || spinImageAcquisitionTime == null)
         return;

      blackflySourceImage.rewind();
      blackflySourceImage.changeAddress(spinImageData.address());

      opencv_imgproc.cvtColor(blackflySourceImage.getBytedecoOpenCVMat(), distortedRGBImage, opencv_imgproc.COLOR_BayerBG2RGB);
      opencv_imgproc.remap(distortedRGBImage,
                           undistortedImage.getBytedecoOpenCVMat(),
                           undistortionMap1,
                           undistortionMap2,
                           opencv_imgproc.INTER_LINEAR,
                           opencv_core.BORDER_CONSTANT,
                           undistortionRemapBorderValue);

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

      remoteTunableCameraTransform.update();
      syncedRobot.update();

      // Converting BayerRG8 -> RGBA -> YUV
      // Here we use COLOR_BayerBG2RGBA opencv conversion. The Blackfly cameras are set to use BayerRG pixel format.
      // But, for some reason, it's actually BayerBG. Changing to COLOR_BayerRG2RGBA will result in the wrong colors.
      opencv_imgproc.cvtColor(blackflySourceImage.getBytedecoOpenCVMat(), rgbaMat, opencv_imgproc.COLOR_BayerBG2RGBA);
      opencv_imgproc.cvtColor(rgbaMat, yuv420Image, opencv_imgproc.COLOR_RGBA2YUV_I420);

      // TODO: speed this up, it uses a lot of CPU
      opencv_imgcodecs.imencode(".jpg", yuv420Image, jpegImageBytePointer, COMPRESSION_PARAMETERS);

      ousterFisheyeColoringIntrinsicsROS2.updateAndPublishThrottledStatus();

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
      ImageMessageFormat.COLOR_JPEG_YUVI420.packMessageFormat(imageMessage);
      imageMessage.getPosition().set(ousterToBlackflyTransfrom.getTranslation());
      imageMessage.getOrientation().set(ousterToBlackflyTransfrom.getRotation());
      ros2ImagePublisher.publish(imageMessage);
   }

   public void destroy()
   {
      destroyed = true;
      if (spinnakerBlackfly != null)
         spinnakerBlackfly.stopAcquiringImages();
      if (arUcoMarkerDetection != null)
         arUcoMarkerDetection.destroy();
      spinnakerBlackfly = null;
      arUcoMarkerDetection = null;
   }
}
