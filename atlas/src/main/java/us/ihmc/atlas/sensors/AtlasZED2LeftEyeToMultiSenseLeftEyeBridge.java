package us.ihmc.atlas.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import perception_msgs.msg.dds.VideoPacket;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.awt.*;
import java.awt.color.ColorSpace;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;
import java.nio.ByteBuffer;

public class AtlasZED2LeftEyeToMultiSenseLeftEyeBridge
{
   private static final double OUTPUT_FREQUENCY = UnitConversions.hertzToSeconds(10.0);

   public AtlasZED2LeftEyeToMultiSenseLeftEyeBridge()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "zed2_to_left_eye");
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_to_left_eye");
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
      CommunicationHelper ros2Helper = new CommunicationHelper(robotModel, ros2Node);
      ROS2SyncedRobotModel syncedRobot = ros2Helper.newSyncedRobot();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      Throttler throttler = new Throttler();
      YUVPictureConverter converter = new YUVPictureConverter();
      JPEGEncoder encoder = new JPEGEncoder();

      int imageWidth = 640;
      int imageHeight = 360;
      double fx = 500.0;
      double fy = 500.0;
      CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, imageWidth / 2.0, imageHeight / 2.0, imageWidth, imageHeight);

      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("ZED2LeftEyeToLeftEyeBridge", true);


      ColorSpace colorSpace = ColorSpace.getInstance(ColorSpace.CS_sRGB);
      ComponentColorModel colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);

      String bgr8Name = "bgr8";
//      int bgr8 = ImageEncodingTools.getCvType(bgr8Name);
      Mat inputImageMat = new Mat(720, 1280, opencv_core.CV_8UC3);
      Mat decodedImageMat = new Mat(720, 1280, opencv_core.CV_8UC3);
      Mat compressedImageMat = new Mat(720, 1280, opencv_core.CV_8UC3);
//      ImEncoding.getColorConversionCode(bgr8Name, ImageEncodingTools.YUV422);
//      YUVPicture yuvPicture = new YUVPicture(YUVPicture.YUVSubsamplingType.YUV420, 1280, 720);
//      Vector<Integer> compressionParameters = new Vector<>(Arrays.asList(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75));
      IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);
      BytePointer outputBuffer = new BytePointer();

      AbstractRosTopicSubscriber<sensor_msgs.Image> subscriber
            = new AbstractRosTopicSubscriber<sensor_msgs.Image>(sensor_msgs.Image._TYPE)
      {
         @Override
         public void onNewMessage(sensor_msgs.Image ros1Image)
         {
            if (throttler.run(OUTPUT_FREQUENCY))
            {
               executor.submit(() ->
               {
                  syncedRobot.update();
//                  if (syncedRobot.getDataReceptionTimerSnapshot().isRunning(3.0))
                  {
                     try
                     {

                        ChannelBuffer nettyImageData = ros1Image.getData();
                        ByteBuffer dataByteBuffer = nettyImageData.toByteBuffer();
                        int arrayOffset = nettyImageData.arrayOffset();
                        dataByteBuffer.position(arrayOffset);
                        ByteBuffer offsetByteBuffer = dataByteBuffer.slice();
//                        byte[] nettyBackingArray = nettyImageData.array();
//                        int dataLength = nettyBackingArray.length - arrayOffset;
//                        ByteBuffer offsetByteBuffer = ByteBuffer.wrap(nettyBackingArray, arrayOffset, dataLength);
//                        ByteBuffer offsetByteBuffer = new ByteBuffer(0, 0, dataLength, dataLength, nettyBackingArray, arrayOffset);
//                        System.out.println(dataByteBuffer.isDirect());

                        BytePointer imageDataPointer = new BytePointer(offsetByteBuffer);
                        inputImageMat.data(imageDataPointer);

                        opencv_imgproc.cvtColor(inputImageMat, decodedImageMat, opencv_imgproc.COLOR_BGR2YUV_I420);

                        opencv_imgcodecs.imencode(".jpg", decodedImageMat, outputBuffer, compressionParameters);

                        byte[] data = new byte[outputBuffer.asBuffer().remaining()];
                        outputBuffer.asBuffer().get(data);

                        //                        opencv_imgcodecs.imdecode(inputImageMat, opencv_imgcodecs.IMREAD_ANYCOLOR, decodedImageMat);


//                        ROSOpenCVImage cvImage = ROSOpenCVImageTools.toCvCopy(ros1Image, ImageEncodingTools.RGBA8);
//                        Buffer cvBuffer = cvImage.image.createBuffer();
//
//
//                        BufferedImage bufferedImage = RosTools.bufferedImageFromRosMessageRaw(colorModel, ros1Image);
//
//                        YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVPicture.YUVSubsamplingType.YUV420);
//
//
//                        ByteBuffer buffer = encoder.encode(picture, 75);
//
//                        byte[] data = new byte[buffer.remaining()];
//                        buffer.get(data);

                        FramePose3DReadOnly ousterPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getOusterLidarFrame);
                        ousterPose.get(transformToWorld);

                        VideoPacket message = new VideoPacket();
                        message.setTimestamp(System.nanoTime());
                        message.getData().add(data);
                        message.getPosition().set(ousterPose.getPosition());
                        message.getOrientation().set(ousterPose.getOrientation());
                        message.setVideoSource(VideoPacket.VIDEO_SOURCE_MULTISENSE_LEFT_EYE);
                        message.getIntrinsicParameters().set(HumanoidMessageTools.toIntrinsicParametersMessage(depthCameraIntrinsics));

                        ros2Helper.publish(PerceptionAPI.VIDEO, message);
                     }
                     catch (Exception e)
                     {
                        LogTools.error(e.getMessage());
                        e.printStackTrace();
                     }
                  }
               });
            }
         }
      };
      ros1Node.attachSubscriber(RosTools.ZED2_LEFT_EYE_VIDEO, subscriber);
      ros1Node.execute();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         executor.destroy();
         ros1Node.shutdown();
      }, "IHMC-ZED2LeftEyeROS1ToLeftEyeBridgeShutdown"));
   }

   public static void main(String[] args)
   {
      new AtlasZED2LeftEyeToMultiSenseLeftEyeBridge();
   }
}
