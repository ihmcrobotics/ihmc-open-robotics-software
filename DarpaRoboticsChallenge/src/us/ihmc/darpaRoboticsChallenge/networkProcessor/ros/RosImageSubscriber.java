package us.ihmc.darpaRoboticsChallenge.networkProcessor.ros;

import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;
import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.graphics3DAdapter.camera.JPanelCameraStreamer;

public abstract class RosImageSubscriber extends RosTopicSubscriber<sensor_msgs.Image>
{
   private final ColorModel colorModel;
   

   public RosImageSubscriber()
   {
      super(sensor_msgs.Image._TYPE);
      ColorSpace colorSpace = ColorSpace.getInstance(ColorSpace.CS_sRGB);
      this.colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
   }

   public void onNewMessage(sensor_msgs.Image message)
   {
      imageReceived(RosTools.bufferedImageFromRosMessage(colorModel, message));
   }

   protected abstract void imageReceived(BufferedImage image);

   public static void main(String[] args) throws URISyntaxException
   {

      final JPanelCameraStreamer leftCameraStreamer = new JPanelCameraStreamer();
      leftCameraStreamer.createAndDisplayInNewWindow("left");

      final JPanelCameraStreamer rightCameraStreamer = new JPanelCameraStreamer();
      rightCameraStreamer.createAndDisplayInNewWindow("right");

      RosImageSubscriber leftImageReceiver = new RosImageSubscriber()
      {

         @Override
         protected void imageReceived(BufferedImage image)
         {
            leftCameraStreamer.updateImage(image);
         }

      };
      RosImageSubscriber rightImageReceiver = new RosImageSubscriber()
      {

         @Override
         protected void imageReceived(BufferedImage image)
         {
            rightCameraStreamer.updateImage(image);
         }

      };

      RosMainNode rosMainNode = new RosMainNode(new URI("http://localhost:11311"));
      rosMainNode.attachSubscriber("/multisense_sl/camera/left/image_rect_color", leftImageReceiver);
      rosMainNode.attachSubscriber("/multisense_sl/camera/right/image_rect_color", rightImageReceiver);
      rosMainNode.execute();

   }
}
