package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.CameraPinholeBrown;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoDataServer;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ros2.ROS2Node;

public abstract class ImageProcessingBehavior extends VideoPacketListenerBehavior
{
   private final CompressedVideoDataServer videoDataServer;

   public ImageProcessingBehavior(String robotName, String namePrefix, ROS2Node ros2Node)
   {
      super(robotName, namePrefix, ros2Node);

      ROS2PublisherBasics<VideoPacket> publisher = createBehaviorOutputPublisher(VideoPacket.class);
      videoDataServer = CompressedVideoDataFactory.createCompressedVideoDataServer(new UIVideoHandler(publisher));
   }

   public abstract void processImageToSend(BufferedImage bufferedImageToPack, Point3DReadOnly cameraPositionToPack, QuaternionReadOnly cameraOrientationToPack,
                                           CameraPinholeBrown intrinsicParametersToPack);

   @Override
   public void onFrame(VideoSource videoSource, BufferedImage bufferedImage, long timestamp, Point3DReadOnly cameraPosition,
                       QuaternionReadOnly cameraOrientation, CameraPinholeBrown intrinsicParameters)
   {
      processImageToSend(bufferedImage, cameraPosition, cameraOrientation, intrinsicParameters);

      videoDataServer.onFrame(VideoSource.IMAGE_PROCESSING_BEHAVIOR, bufferedImage, 0, cameraPosition, cameraOrientation, intrinsicParameters);
   }

   class UIVideoHandler implements CompressedVideoHandler
   {
      private final ROS2PublisherBasics<VideoPacket> publisher;

      public UIVideoHandler(ROS2PublisherBasics<VideoPacket> publisher)
      {
         this.publisher = publisher;
      }

      @Override
      public void onFrame(VideoSource videoSource, byte[] data, long timeStamp, Point3DReadOnly position, QuaternionReadOnly orientation,
                          CameraPinholeBrown intrinsicParameters)
      {
         publisher.publish(HumanoidMessageTools.createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters));
      }

      @Override
      public void addNetStateListener(ConnectionStateListener compressedVideoDataServer)
      {
      }

      @Override
      public boolean isConnected()
      {
         return true;
      }
   }
}
