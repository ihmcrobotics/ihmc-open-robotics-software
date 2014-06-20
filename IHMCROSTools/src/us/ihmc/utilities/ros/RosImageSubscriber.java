package us.ihmc.utilities.ros;

import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;
import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.graphics3DAdapter.camera.JPanelCameraStreamer;

public abstract class RosImageSubscriber extends AbstractRosTopicSubscriber<sensor_msgs.CompressedImage>
{
   private final ColorModel colorModel;
   

   public RosImageSubscriber()
   {
      super(sensor_msgs.CompressedImage._TYPE);
      ColorSpace colorSpace = ColorSpace.getInstance(ColorSpace.CS_sRGB);
      this.colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
   }

   public void onNewMessage(sensor_msgs.CompressedImage message)
   {
      long timeStamp = message.getHeader().getStamp().totalNsecs();
      imageReceived(timeStamp, RosTools.bufferedImageFromRosMessageJpeg(colorModel, message));
   }

   protected abstract void imageReceived(long timeStamp, BufferedImage image);

   public static void main(String[] args) throws URISyntaxException
   {

      final JPanelCameraStreamer leftCameraStreamer = new JPanelCameraStreamer();
      leftCameraStreamer.createAndDisplayInNewWindow("left");

      final JPanelCameraStreamer rightCameraStreamer = new JPanelCameraStreamer();
      rightCameraStreamer.createAndDisplayInNewWindow("right");

      RosImageSubscriber leftImageReceiver = new RosImageSubscriber()
      {

         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            leftCameraStreamer.updateImage(image);
         }

      };
      RosImageSubscriber rightImageReceiver = new RosImageSubscriber()
      {

         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            rightCameraStreamer.updateImage(image);
         }

      };

      RosMainNode rosMainNode = new RosMainNode(new URI("http://localhost:11311"), "darpaRoboticsChallange/rosImageSubscriber");
      rosMainNode.attachSubscriber("/multisense/camera/left/image_rect_color/compressed", leftImageReceiver);
      rosMainNode.attachSubscriber("/multisense/camera/right/image_rect_color/compressed", rightImageReceiver);
      rosMainNode.execute();

   }
}
