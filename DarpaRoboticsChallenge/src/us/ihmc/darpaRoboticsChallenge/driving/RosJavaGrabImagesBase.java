package us.ihmc.darpaRoboticsChallenge.driving;

import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import sensor_msgs.CompressedImage;
import us.ihmc.tools.inputDevices.keyboard.linux.RepeatingReleasedEventsFixer;
import us.ihmc.utilities.ros.RosTools;

/**
 * @author Peter Abeles
 */
public abstract class RosJavaGrabImagesBase extends AbstractNodeMain
{
   private static final boolean COLOR_IMAGE = true;
   private static final String STEREO_NAMESPACE = "/multisense/camera/";
   private static final String IMAGE = "image_raw" + (COLOR_IMAGE ? "/compressed" : "/compressed");
   protected Subscriber<CompressedImage> leftEyeImageSubscriber, rightEyeImageSubscriber;

   private ColorSpace colorSpace;
   private ColorModel colorModel;

   ConnectedNode connectedNode;

   BufferedImage imageLeft;
   BufferedImage imageRight;

   long timestampLeft=1;
   long timestampRight=-1;


   public RosJavaGrabImagesBase()
   {
      new RepeatingReleasedEventsFixer().install();

      colorSpace = (COLOR_IMAGE ? ColorSpace.getInstance(ColorSpace.CS_sRGB) : ColorSpace.getInstance(ColorSpace.CS_GRAY));
      colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
   }

   public void onStart(ConnectedNode connectedNode)
   {
      this.connectedNode = connectedNode;

      leftEyeImageSubscriber = connectedNode.newSubscriber(STEREO_NAMESPACE + "left/" + IMAGE, CompressedImage._TYPE);
      rightEyeImageSubscriber = connectedNode.newSubscriber(STEREO_NAMESPACE + "right/" + IMAGE, CompressedImage._TYPE);

      leftEyeImageSubscriber.addMessageListener(new MessageListener<CompressedImage>()
      {
         public void onNewMessage(CompressedImage message)
         {
            handleImageLeft(message);
         }
      });
      rightEyeImageSubscriber.addMessageListener(new MessageListener<CompressedImage>()
      {
         public void onNewMessage(CompressedImage message)
         {
            handleImageRight(message);
         }
      });

   }

   protected synchronized void handleImageLeft(CompressedImage message)
   {
      timestampLeft = message.getHeader().getStamp().totalNsecs();
      imageLeft = bufferedImageFromRosMessage(message);

      checkProcessImage();
   }

   protected synchronized void handleImageRight(CompressedImage message)
   {
      timestampRight = message.getHeader().getStamp().totalNsecs();
      imageRight = bufferedImageFromRosMessage(message);

      checkProcessImage();
   }

   private void checkProcessImage() {
      if( timestampLeft == timestampRight ) {
         processImages(imageLeft, imageRight);
      }
   }

   protected abstract void processImages( BufferedImage left , BufferedImage right );

   public GraphName getDefaultNodeName()
   {
      return GraphName.of("darpaRoboticsChallenge/DRCGazeboDrivingInterface");
   }

   protected BufferedImage bufferedImageFromRosMessage(CompressedImage imageMessage)
   {
      return RosTools.bufferedImageFromRosMessageJpeg(colorModel, imageMessage);
   }
}
