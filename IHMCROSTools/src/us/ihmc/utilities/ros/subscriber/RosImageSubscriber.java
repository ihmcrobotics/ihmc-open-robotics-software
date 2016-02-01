package us.ihmc.utilities.ros.subscriber;

import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;

import us.ihmc.utilities.ros.RosTools;

public abstract class RosImageSubscriber extends AbstractRosTopicSubscriber<sensor_msgs.Image>
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
      long timeStamp = message.getHeader().getStamp().totalNsecs();
      imageReceived(timeStamp, RosTools.bufferedImageFromRosMessageRaw(colorModel, message));
   }

   protected abstract void imageReceived(long timeStamp, BufferedImage image);
}
