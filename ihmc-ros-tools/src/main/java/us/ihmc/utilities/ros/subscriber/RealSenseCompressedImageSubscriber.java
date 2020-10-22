package us.ihmc.utilities.ros.subscriber;

import controller_msgs.msg.dds.VideoPacket;
import sensor_msgs.CompressedImage;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.utilities.ros.RosMainNode;

public abstract class RealSenseCompressedImageSubscriber extends AbstractRosTopicSubscriber<CompressedImage>
{

   private CompressedImage latestCompressedImage;
   private Point3D32[] points;

   public RealSenseCompressedImageSubscriber(RosMainNode rosMainNode, String compressedImageTopic)
   {
      super(CompressedImage._TYPE);
      rosMainNode.attachSubscriber(compressedImageTopic, this);
   }

   @Override
   public void onNewMessage(CompressedImage compressedImage)
   {
      latestCompressedImage = compressedImage;
      compressedImageReceived(compressedImage);
   }

   protected abstract void compressedImageReceived(CompressedImage image);
}
