package us.ihmc.avatar.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.Image32;
import controller_msgs.msg.dds.VideoPacket;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.Image;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ihmcPerception.camera.RosCameraCompressedImageReceiver;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

public class RealsenseImageROS1Bridge extends AbstractRosTopicSubscriber<Image>
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

   private final IHMCROS2Publisher<VideoPacket> imagePublisher;
   private final RosCameraCompressedImageReceiver cameraImageReceiver;

   public RealsenseImageROS1Bridge()
   {
      super(Image._TYPE);

      URI masterURI = NetworkParameters.getROSURI();
      RosMainNode rosMainNode = new RosMainNode(masterURI, "ImagePublisher", true);
      rosMainNode.attachSubscriber("/depthcam/color/image_raw", this);
      rosMainNode.execute();

      LogTools.info(ROS2Tools.D435_VIDEO);
      imagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, VideoPacket.class, ROS2Tools.D435_VIDEO);
   }

   class CompressedVideoHandler implements us.ihmc.communication.producers.CompressedVideoHandler
   {
      @Override
      public void addNetStateListener(ConnectionStateListener compressedVideoDataServer)
      {

      }

      @Override
      public boolean isConnected()
      {
         return true;
      }

      @Override
      public void onFrame(VideoSource videoSource, byte[] compressedImageData, long timestamp, Point3DReadOnly cameraPosition,
                          QuaternionReadOnly cameraOrientation, CameraPinholeBrown intrinsicParamaters)
      {

      }
   }

   @Override
   public void onNewMessage(Image image)
   {
      int width = image.getWidth();
      int height = image.getHeight();

      Image32 message = new Image32();
      BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

      message.setHeight(height);
      message.setWidth(width);

      ChannelBuffer data = image.getData();
      byte[] array = data.array();
      int dataIndex = data.arrayOffset();
      for (int i = 0; i < height; i++)
      {
         for (int j = 0; j < width; j++)
         {
            int b = array[dataIndex];
            dataIndex++;
            int g = array[dataIndex];
            dataIndex++;
            int r = array[dataIndex];
            dataIndex++;

            int rgbColor = convertBGR2RGB(b, g, r);
            message.getRgbdata().add(rgbColor);
            bufferedImage.setRGB(j, i, rgbColor);
         }
      }

      VideoPacket message = HumanoidMessageTools.createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters);

      imagePublisher.publish(message);
   }

   private static int convertBGR2RGB(int b, int g, int r)
   {
      int rgb = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
      return rgb;
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new RealsenseImageROS1Bridge();
   }
}
