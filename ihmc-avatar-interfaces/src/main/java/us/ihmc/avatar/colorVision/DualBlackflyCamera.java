package us.ihmc.avatar.colorVision;

import boofcv.struct.calib.CameraPinholeBrown;
import perception_msgs.msg.dds.ArUcoMarkerPoses;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_calib3d;
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
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.tools.time.FrequencyCalculator;

import java.time.Instant;
import java.util.HashMap;
import java.util.List;

public class DualBlackflyCamera
{
   private final String serialNumber;
   private final ROS2SyncedRobotModel syncedRobot;
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
   private Mat distortionCoefficients;
   private CameraPinholeBrown cameraPinholeBrown;
   private Mat undistortedImageMat;
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
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private final FramePose3D framePoseOfMarker = new FramePose3D();
   private final ArUcoMarkerPoses arUcoMarkerPoses = new ArUcoMarkerPoses();
   private final HashMap<Integer, OpenCVArUcoMarker> arUcoMarkersToTrack = new HashMap<>();
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;

   public DualBlackflyCamera(String serialNumber, ROS2SyncedRobotModel syncedRobot)
   {
      this.serialNumber = serialNumber;
      this.syncedRobot = syncedRobot;
   }

   public void create(SpinnakerBlackfly blackfly,
                      RobotSide side,
                      ROS2Helper ros2Helper,
                      RealtimeROS2Node realtimeROS2Node,
                      List<OpenCVArUcoMarker> arUcoMarkersToTrack)
   {
      this.blackfly = blackfly;
      this.side = side;
      this.ros2Helper = ros2Helper;
      this.realtimeROS2Node = realtimeROS2Node;

      arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection,
                                                                arUcoMarkersToTrack,
                                                                syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame(),
                                                                ros2Helper);

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
            numberOfBytesInFrame = imageWidth * imageHeight * 4;
            spinImageDataPointer = new BytePointer(numberOfBytesInFrame);

            blackflySourceImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8U);

            // From OpenCV calibrateCamera with Blackfly serial number 17372478 with FE185C086HA-1 fisheye lens
            // Procedure conducted by Bhavyansh Mishra on 12/14/2021
            cameraPinholeBrown = new CameraPinholeBrown();
            cameraPinholeBrown.setFx(499.3716197917922);
            cameraPinholeBrown.setFy(506.42956667285574);
            cameraPinholeBrown.setCx(1043.8826790137316);
            cameraPinholeBrown.setCy(572.2558510618412);

            cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
            cameraMatrix.ptr(0, 0).putDouble(cameraPinholeBrown.getFx());
            cameraMatrix.ptr(0, 1).putDouble(0.0);
            cameraMatrix.ptr(0, 2).putDouble(cameraPinholeBrown.getCx());
            cameraMatrix.ptr(1, 0).putDouble(0.0);
            cameraMatrix.ptr(1, 1).putDouble(cameraPinholeBrown.getFy());
            cameraMatrix.ptr(1, 2).putDouble(cameraPinholeBrown.getCy());
            cameraMatrix.ptr(2, 0).putDouble(0.0);
            cameraMatrix.ptr(2, 1).putDouble(0.0);
            cameraMatrix.ptr(2, 2).putDouble(1.0);
            distortionCoefficients = new Mat(-0.1304880574839372, 0.0343337720836711, 0, 0, 0.002347490605947351,
                                             0.163868408051474, -0.02493286434834704, 0.01394671162254435);
            distortionCoefficients.reshape(1, 8);
            undistortedImageMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8U);

            yuv420Image = new Mat(imageHeight, imageWidth, opencv_core.CV_8U);
            rgbaMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8U); // Mat for color conversion

            compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

            ROS2Topic<ImageMessage> imageTopic = ROS2Tools.BLACKFLY_FISHEYE_COLOR_IMAGE.get(side);
            LogTools.info("Publishing ROS 2 color images: {}", imageTopic);
            ros2ImagePublisher = ROS2Tools.createPublisher(realtimeROS2Node, imageTopic, ROS2QosProfile.BEST_EFFORT());
         }
         else // We don't want to publish until the node is spinning which will be next time
         {
            blackfly.setBytedecoPointerToSpinImageData(spinImage, spinImageDataPointer);
            blackflySourceImage.rewind();
            blackflySourceImage.changeAddress(spinImageDataPointer.address());

//            opencv_core.flip(blackflySourceImage.getBytedecoOpenCVMat(), blackflySourceImage.getBytedecoOpenCVMat(), BytedecoOpenCVTools.FLIP_BOTH);

            opencv_calib3d.undistort(blackflySourceImage.getBytedecoOpenCVMat(), undistortedImageMat, cameraMatrix, distortionCoefficients);
            Mat postDistortionMat = undistortedImageMat;
//            Mat postDistortionMat = blackflySourceImage.getBytedecoOpenCVMat();

            if (side == RobotSide.RIGHT)
            {
               ReferenceFrame blackflyCameraFrame = syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame();
               ReferenceFrame ousterLidarFrame = syncedRobot.getReferenceFrames().getOusterLidarFrame();
               if (arUcoMarkerDetection == null)
               {
                  undistortedImage = new BytedecoImage(postDistortionMat);

                  arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
                  arUcoMarkerDetection.create(blackflyCameraFrame);
                  arUcoMarkerDetection.setSourceImageForDetection(undistortedImage);
                  arUcoMarkerDetection.setCameraInstrinsics(cameraPinholeBrown);
               }

               syncedRobot.update();
               ousterLidarFrame.getTransformToDesiredFrame(ousterToBlackflyTransfrom, blackflyCameraFrame);
               
               arUcoMarkerDetection.update();
               arUcoMarkerDetection.drawDetectedMarkers(postDistortionMat);
               arUcoMarkerDetection.drawRejectedPoints(postDistortionMat);
               arUcoMarkerPublisher.update();

               SwapReference<Mat> ids = arUcoMarkerDetection.getIds();
               arUcoMarkerPoses.getMarkerId().clear();
               arUcoMarkerPoses.getOrientation().clear();
               arUcoMarkerPoses.getPosition().clear();
               for (int i = 0; i < ids.getForThreadTwo().rows(); i++)
               {
                  int markerID = ids.getForThreadTwo().ptr(i, 0).getInt();
                  OpenCVArUcoMarker markerToTrack = arUcoMarkersToTrack.get(markerID);

                  if (markerToTrack != null)
                  {
                     framePoseOfMarker.setIncludingFrame(blackflyCameraFrame, arUcoMarkerDetection.getPose(markerToTrack));
                     framePoseOfMarker.changeFrame(ReferenceFrame.getWorldFrame());

                     arUcoMarkerPoses.getMarkerId().add(markerID);
                     arUcoMarkerPoses.getOrientation().add().set(framePoseOfMarker.getOrientation());
                     arUcoMarkerPoses.getPosition().add().set(framePoseOfMarker.getX(), framePoseOfMarker.getY(), framePoseOfMarker.getZ());
                  }
               }

               ros2Helper.publish(DualBlackflyComms.FRAME_POSE, arUcoMarkerPoses);
            }

            convertColorDuration.start();
            // Converting BayerRG8 -> RGBA -> YUV
            // Here we use COLOR_BayerBG2RGBA opencv conversion. The Blackfly cameras are set to use BayerRG pixel format.
            // But, for some reason, it's actually BayerBG. Changing to COLOR_BayerRG2RGBA will result in the wrong colors.
            opencv_imgproc.cvtColor(postDistortionMat, rgbaMat, opencv_imgproc.COLOR_BayerBG2RGBA);
            opencv_imgproc.cvtColor(rgbaMat, yuv420Image, opencv_imgproc.COLOR_RGBA2YUV_I420);
            convertColorDuration.suspend();

            encodingDuration.start();
            opencv_imgcodecs.imencode(".jpg", yuv420Image, jpegImageBytePointer, compressionParameters);
            encodingDuration.suspend();

            copyDuration.start();
            compressedImageDataPacker.pack(imageMessage, jpegImageBytePointer);
            copyDuration.suspend();
            MessageTools.toMessage(now, imageMessage.getAcquisitionTime());
            imageMessage.setImageWidth(imageWidth);
            imageMessage.setImageHeight(imageHeight);
            imageMessage.setFocalLengthXPixels((float) SensorHeadParameters.FOCAL_LENGTH_X_FOR_COLORING);
            imageMessage.setFocalLengthYPixels((float) SensorHeadParameters.FOCAL_LENGTH_Y_FOR_COLORING);
            imageMessage.setPrincipalPointXPixels((float) SensorHeadParameters.PRINCIPAL_POINT_X_FOR_COLORING);
            imageMessage.setPrincipalPointYPixels((float) SensorHeadParameters.PRINCIPAL_POINT_Y_FOR_COLORING);
            imageMessage.setIsEquidistantFisheyeCameraModel(true);
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
