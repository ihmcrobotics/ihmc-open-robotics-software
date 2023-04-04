package us.ihmc.avatar.colorVision;

import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import org.bytedeco.spinnaker.global.Spinnaker_C;
import perception_msgs.msg.dds.ImageMessage;
import std_msgs.msg.dds.Float64;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.time.FrequencyCalculator;

import java.time.Instant;
import java.util.List;

public class DualBlackflyCamera
{
   private final String serialNumber;
   private final ROS2SyncedRobotModel syncedRobot;
   private RigidBodyTransform cameraTransformToParent;
   private SpinnakerBlackfly blackfly;
   private final spinImage spinImage = new spinImage();
   private BytePointer spinImageDataPointer;
   private RobotSide side;
   private ROS2Helper ros2Helper;
   private RealtimeROS2Node realtimeROS2Node;
   private IHMCRealtimeROS2Publisher<ImageMessage> ros2ImagePublisher;
   private long numberOfBytesInFrame;
   private int imageWidth;
   private int imageHeight;
   private final FrequencyCalculator imagePublishRateCalculator = new FrequencyCalculator();
   private BytedecoImage blackflySourceImage;
   private Mat cameraMatrix;
   private Mat newCameraMatrixEstimate;
   private Mat distortionCoefficients;
   private Size sourceImageSize;
   private Size undistortedImageSize;
   private Mat rectificationTransformation;
   private Mat undistortionMap1;
   private Mat undistortionMap2;
   private Scalar undistortionRemapBorderValue;
   private Mat distortedRGBImage;
   private BytedecoImage undistortedImage;
   private Mat yuv420Image;
   private Mat rgbaMat;
   private final RigidBodyTransform ousterToBlackflyTransfrom = new RigidBodyTransform();
   private final ImageMessage imageMessage = new ImageMessage();
   private final BytePointer jpegImageBytePointer = new BytePointer(imageMessage.getData().capacity());
   private final ImageMessageDataPacker compressedImageDataPacker = new ImageMessageDataPacker(jpegImageBytePointer.capacity());
   private IntPointer compressionParameters;
   private final Stopwatch getNextImageDuration = new Stopwatch();
   private final Stopwatch convertColorDuration = new Stopwatch();
   private final Stopwatch encodingDuration = new Stopwatch();
   private final Stopwatch copyDuration = new Stopwatch();
   private long sequenceNumber = 0;
   private List<OpenCVArUcoMarker> arUcoMarkersToTrack;
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;
   private IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics;
   private ROS2StoredPropertySet<IntrinsicCameraMatrixProperties> ousterFisheyeColoringIntrinsicsROS2;
   private ROS2TunedRigidBodyTransform remoteTunableCameraTransform;

   public DualBlackflyCamera(String serialNumber, ROS2SyncedRobotModel syncedRobot, RigidBodyTransform cameraTransformToParent)
   {
      this.serialNumber = serialNumber;
      this.syncedRobot = syncedRobot;
      this.cameraTransformToParent = cameraTransformToParent;
   }

   public void create(SpinnakerBlackfly blackfly,
                      RobotSide side,
                      ROS2Helper ros2Helper,
                      RealtimeROS2Node realtimeROS2Node,
                      List<OpenCVArUcoMarker> arUcoMarkersToTrack,
                      IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics)
   {
      this.blackfly = blackfly;
      this.side = side;
      this.ros2Helper = ros2Helper;
      this.realtimeROS2Node = realtimeROS2Node;
      this.arUcoMarkersToTrack = arUcoMarkersToTrack;
      this.ousterFisheyeColoringIntrinsics = ousterFisheyeColoringIntrinsics;

      blackfly.setAcquisitionMode(Spinnaker_C.spinAcquisitionModeEnums.AcquisitionMode_Continuous);
      blackfly.setPixelFormat(Spinnaker_C.spinPixelFormatEnums.PixelFormat_BayerRG8);
      blackfly.startAcquiringImages();
   }

   public void update()
   {
      Instant now = Instant.now();
      getNextImageDuration.start();
      if (blackfly.getNextImage(spinImage))
      {
         getNextImageDuration.suspend();

         if (ros2ImagePublisher == null)
         {
            imageWidth = blackfly.getWidth(spinImage);
            imageHeight = blackfly.getHeight(spinImage);
            LogTools.info("Blackfly {} resolution detected: {} x {}", serialNumber, imageWidth, imageHeight);
            numberOfBytesInFrame = (long) imageWidth * imageHeight; // BayerRG8
            spinImageDataPointer = new BytePointer(numberOfBytesInFrame);

            blackflySourceImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);

            yuv420Image = new Mat(imageHeight * 1.5, imageWidth, opencv_core.CV_8UC1);
            rgbaMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4); // Mat for color conversion

            compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

            ousterFisheyeColoringIntrinsicsROS2 = new ROS2StoredPropertySet<>(ros2Helper,
                                                                              DualBlackflyComms.OUSTER_FISHEYE_COLORING_INTRINSICS,
                                                                              ousterFisheyeColoringIntrinsics);

            ROS2Topic<ImageMessage> imageTopic = ROS2Tools.BLACKFLY_FISHEYE_COLOR_IMAGE.get(side);
            LogTools.info("Publishing ROS 2 color images: {}", imageTopic);
            ros2ImagePublisher = ROS2Tools.createPublisher(realtimeROS2Node, imageTopic, ROS2QosProfile.BEST_EFFORT());
         }
         else // We don't want to publish until the node is spinning which will be next time
         {
            blackfly.setBytedecoPointerToSpinImageData(spinImage, spinImageDataPointer);
            blackflySourceImage.rewind();
            blackflySourceImage.changeAddress(spinImageDataPointer.address());

            // TODO: Still need a flip anywhere?
            // opencv_core.flip(blackflySourceImage.getBytedecoOpenCVMat(), blackflySourceImage.getBytedecoOpenCVMat(), BytedecoOpenCVTools.FLIP_BOTH);

            if (side == RobotSide.RIGHT)
            {
               ReferenceFrame blackflyCameraFrame = syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame();
               ReferenceFrame ousterLidarFrame = syncedRobot.getReferenceFrames().getOusterLidarFrame();
               if (arUcoMarkerDetection == null)
               {
                  undistortedImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
                  distortedRGBImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3);

                  cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
                  opencv_core.setIdentity(cameraMatrix);
                  cameraMatrix.ptr(0, 0).putDouble(SensorHeadParameters.FOCAL_LENGTH_X_FOR_UNDISORTION);
                  cameraMatrix.ptr(1, 1).putDouble(SensorHeadParameters.FOCAL_LENGTH_Y_FOR_UNDISORTION);
                  cameraMatrix.ptr(0, 2).putDouble(SensorHeadParameters.PRINCIPAL_POINT_X_FOR_UNDISORTION);
                  cameraMatrix.ptr(1, 2).putDouble(SensorHeadParameters.PRINCIPAL_POINT_Y_FOR_UNDISORTION);
                  newCameraMatrixEstimate = new Mat(3, 3, opencv_core.CV_64F);
                  opencv_core.setIdentity(newCameraMatrixEstimate);
                  distortionCoefficients = new Mat(SensorHeadParameters.K1_FOR_UNDISORTION,
                                                   SensorHeadParameters.K2_FOR_UNDISORTION,
                                                   SensorHeadParameters.K3_FOR_UNDISORTION,
                                                   SensorHeadParameters.K4_FOR_UNDISORTION);
                  sourceImageSize = new Size(imageWidth, imageHeight);
                  undistortedImageSize = new Size((int) (SensorHeadParameters.UNDISTORTED_IMAGE_SCALE * imageWidth),
                                                  (int) (SensorHeadParameters.UNDISTORTED_IMAGE_SCALE * imageHeight));
                  rectificationTransformation = new Mat(3, 3, opencv_core.CV_64F);
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
                  // https://docs.opencv.org/4.6.0/db/d58/group__calib3d__fisheye.html#ga167df4b00a6fd55287ba829fbf9913b9
                  opencv_calib3d.fisheyeInitUndistortRectifyMap(cameraMatrix,
                                                                distortionCoefficients,
                                                                rectificationTransformation,
                                                                newCameraMatrixEstimate,
                                                                undistortedImageSize,
                                                                opencv_core.CV_16SC2,
                                                                undistortionMap1,
                                                                undistortionMap2);
                  arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
                  arUcoMarkerDetection.create(blackflyCameraFrame);
                  arUcoMarkerDetection.setSourceImageForDetection(undistortedImage);
                  newCameraMatrixEstimate.copyTo(arUcoMarkerDetection.getCameraMatrix());
                  arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection,
                                                                            arUcoMarkersToTrack,
                                                                            syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame(),
                                                                            ros2Helper);

                  remoteTunableCameraTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                                       ROS2Tools.OBJECT_DETECTION_CAMERA_TO_PARENT_TUNING,
                                                                                       cameraTransformToParent);
               }

               remoteTunableCameraTransform.update();
               syncedRobot.update();
               ousterLidarFrame.getTransformToDesiredFrame(ousterToBlackflyTransfrom, blackflyCameraFrame);

               opencv_imgproc.cvtColor(blackflySourceImage.getBytedecoOpenCVMat(), distortedRGBImage, opencv_imgproc.COLOR_BayerBG2RGB);
               opencv_imgproc.remap(distortedRGBImage,
                                    undistortedImage.getBytedecoOpenCVMat(),
                                    undistortionMap1,
                                    undistortionMap2,
                                    opencv_imgproc.INTER_LINEAR,
                                    opencv_core.BORDER_CONSTANT,
                                    undistortionRemapBorderValue);

               arUcoMarkerDetection.update();
               // TODO: Maybe publish a separate image for ArUco marker debugging sometime.
               // arUcoMarkerDetection.drawDetectedMarkers(blackflySourceImage.getBytedecoOpenCVMat());
               // arUcoMarkerDetection.drawRejectedPoints(blackflySourceImage.getBytedecoOpenCVMat());
               arUcoMarkerPublisher.update();
            }

            convertColorDuration.start();
            // Converting BayerRG8 -> RGBA -> YUV
            // Here we use COLOR_BayerBG2RGBA opencv conversion. The Blackfly cameras are set to use BayerRG pixel format.
            // But, for some reason, it's actually BayerBG. Changing to COLOR_BayerRG2RGBA will result in the wrong colors.
            opencv_imgproc.cvtColor(blackflySourceImage.getBytedecoOpenCVMat(), rgbaMat, opencv_imgproc.COLOR_BayerBG2RGBA);
            opencv_imgproc.cvtColor(rgbaMat, yuv420Image, opencv_imgproc.COLOR_RGBA2YUV_I420);
            convertColorDuration.suspend();

            encodingDuration.start();
            opencv_imgcodecs.imencode(".jpg", yuv420Image, jpegImageBytePointer, compressionParameters);
            encodingDuration.suspend();

            ousterFisheyeColoringIntrinsicsROS2.updateAndPublishThrottledStatus();

            copyDuration.start();
            compressedImageDataPacker.pack(imageMessage, jpegImageBytePointer);
            copyDuration.suspend();
            MessageTools.toMessage(now, imageMessage.getAcquisitionTime());
            imageMessage.setImageWidth(imageWidth);
            imageMessage.setImageHeight(imageHeight);
            imageMessage.setFocalLengthXPixels((float) ousterFisheyeColoringIntrinsics.getFocalLengthX());
            imageMessage.setFocalLengthYPixels((float) ousterFisheyeColoringIntrinsics.getFocalLengthY());
            imageMessage.setPrincipalPointXPixels((float) ousterFisheyeColoringIntrinsics.getPrinciplePointX());
            imageMessage.setPrincipalPointYPixels((float) ousterFisheyeColoringIntrinsics.getPrinciplePointY());
            CameraModel.EQUIDISTANT_FISHEYE.packMessageFormat(imageMessage);
            imageMessage.setSequenceNumber(sequenceNumber++);
            ImageMessageFormat.COLOR_JPEG_YUVI420.packMessageFormat(imageMessage);
            imageMessage.getPosition().set(ousterToBlackflyTransfrom.getTranslation());
            imageMessage.getOrientation().set(ousterToBlackflyTransfrom.getRotation());
            ros2ImagePublisher.publish(imageMessage);

            imagePublishRateCalculator.ping();
         }
      }
      Spinnaker_C.spinImageRelease(spinImage);

      sendStatisticMessage(getNextImageDuration, DualBlackflyComms.GET_IMAGE_DURATION);
      sendStatisticMessage(convertColorDuration, DualBlackflyComms.CONVERT_COLOR_DURATION);
      sendStatisticMessage(encodingDuration, DualBlackflyComms.ENCODING_DURATION);
      sendStatisticMessage(copyDuration, DualBlackflyComms.COPY_DURATION);
      sendStatisticMessage(imagePublishRateCalculator.getFrequency(), DualBlackflyComms.PUBLISH_RATE.get(side));
   }

   private void sendStatisticMessage(Stopwatch stopwatch, ROS2Topic<Float64> topic)
   {
      double elapsed = stopwatch.totalElapsed();
      if (!Double.isNaN(elapsed))
      {
         sendStatisticMessage(elapsed, topic);
      }
   }

   private void sendStatisticMessage(double value, ROS2Topic<Float64> topic)
   {
      Float64 float64Message = new Float64();
      float64Message.setData(value);
      ros2Helper.publish(topic, float64Message);
   }

   public void destroy()
   {
      if (blackfly != null)
         blackfly.stopAcquiringImages();
   }

   public String getSerialNumber()
   {
      return serialNumber;
   }

   public IHMCRealtimeROS2Publisher<ImageMessage> getRos2ImagePublisher()
   {
      return ros2ImagePublisher;
   }
}
