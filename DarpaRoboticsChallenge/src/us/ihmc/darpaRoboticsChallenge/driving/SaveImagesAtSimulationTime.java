package us.ihmc.darpaRoboticsChallenge.driving;

import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

import boofcv.io.image.UtilImageIO;
import rosgraph_msgs.Clock;
import sensor_msgs.CompressedImage;
import us.ihmc.utilities.ros.RosTools;

/**
 * Saves images at the specified rate in simulation time
 *
 * @author Peter Abeles
 */
public class SaveImagesAtSimulationTime
{
   String directory;

   private ColorSpace colorSpace;
   private ColorModel colorModel;

   volatile double simulationTime;

   volatile double previousTime;
   volatile double period;

   volatile BufferedImage imageLeft;
   volatile BufferedImage imageRight;

   volatile long timestampLeft;
   volatile long timestampRight=-1;

   volatile int numFrames = 0;


   public SaveImagesAtSimulationTime( String directory , double period , Subscriber<Clock> subscriberTime ,
                                      Subscriber<CompressedImage> subscriberLeftEye ,
                                      Subscriber<CompressedImage> subscriberRightEye )
   {
      this.directory = directory;
      this.period = period;

      subscriberTime.addMessageListener(new MessageListener<Clock>()
      {
         public void onNewMessage(Clock message)
         {
            simulationTime = message.getClock().toSeconds();
         }
      });

      subscriberLeftEye.addMessageListener(new MessageListener<CompressedImage>()
      {
         public void onNewMessage(CompressedImage message)
         {
            handleImageLeft(message);
         }
      });

      subscriberRightEye.addMessageListener(new MessageListener<CompressedImage>()
      {
         public void onNewMessage(CompressedImage message)
         {
            handleImageRight(message);
         }
      });

      colorSpace = ColorSpace.getInstance(ColorSpace.CS_sRGB);
      colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
   }

   protected synchronized void handleImageLeft(CompressedImage message)
   {
      timestampLeft = message.getHeader().getStamp().totalNsecs();
      imageLeft = RosTools.bufferedImageFromRosMessageJpeg(colorModel, message);

      checkProcessImage();
   }

   protected synchronized void handleImageRight(CompressedImage message)
   {
      timestampRight = message.getHeader().getStamp().totalNsecs();
      imageRight = RosTools.bufferedImageFromRosMessageJpeg(colorModel, message);

      checkProcessImage();
   }

   private void checkProcessImage() {
      if( timestampLeft == timestampRight ) {
         process( imageLeft , imageRight );
      } else {
      }
   }

   private synchronized void process( BufferedImage left , BufferedImage right ) {
      if( simulationTime > previousTime + period ) {
//         System.out.println("####### saving image time "+simulationTime);
         if( right == null )
            System.out.println("WTF");
         previousTime = simulationTime;
         UtilImageIO.saveImage(left, String.format("%s/left%05d.ppm",directory,numFrames));
         UtilImageIO.saveImage(right, String.format("%s/right%05d.ppm",directory,numFrames));
         numFrames++;
      }
   }
}
