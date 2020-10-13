package us.ihmc.ihmcPerception.lineSegmentDetector;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ihmcPerception.camera.RosCameraInfoSubscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosCompressedImageSubscriber;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import static us.ihmc.ihmcPerception.lineSegmentDetector.RealSenseL515DataPublisher.displayBufferedImage;

public class RealSenseL515CompressedImageSubscriber extends RosCompressedImageSubscriber
{
   private String l515CompressedImageTopic = "/camera/color/image_raw/compressed";
   private String l515CameraInfoTopic = "/camera/color/camera_info";
   private BufferedImage latestBufferedImage;

   private static final YUVPictureConverter converter = new YUVPictureConverter();
   private static final JPEGEncoder encoder = new JPEGEncoder();

   private RosCameraInfoSubscriber colorImageInfoSubscriber;

   private final IHMCROS2Publisher<VideoPacket> colorPublisher;

   public RealSenseL515CompressedImageSubscriber(RosMainNode rosNode, ROS2Node ros2Node)
   {
      super();
      colorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT);
      colorImageInfoSubscriber = new RosCameraInfoSubscriber(l515CameraInfoTopic);
      rosNode.attachSubscriber(l515CompressedImageTopic, this);
      rosNode.attachSubscriber(l515CameraInfoTopic, colorImageInfoSubscriber);
   }

   @Override
   protected void imageReceived(long timeStamp, BufferedImage image)
   {
      System.out.println(image);
      if (latestBufferedImage == null)
      {
         latestBufferedImage = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
      }
      latestBufferedImage.getGraphics().drawImage(image, 0, 0, null);

      displayBufferedImage(latestBufferedImage, 32, false);

      CameraPinholeBrown intrinsicParameters = colorImageInfoSubscriber.getIntrinisicParameters();
      YUVPicture picture = converter.fromBufferedImage(latestBufferedImage, YUVPicture.YUVSubsamplingType.YUV420);
      ByteBuffer buffer = null;
      try
      {
         buffer = encoder.encode(picture, 35);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      byte[] data = new byte[buffer.remaining()];
      buffer.get(data);

      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      VideoPacket message = HumanoidMessageTools.createVideoPacket(VideoSource.MULTISENSE_LEFT_EYE,
                                                                   timeStamp,
                                                                   data,
                                                                   position,
                                                                   orientation,
                                                                   intrinsicParameters);
      colorPublisher.publish(message);
      picture.delete();
   }
}
