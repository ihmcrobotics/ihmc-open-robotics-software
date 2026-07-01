package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.BigVideoPacket;
import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.tools.string.StringTools;

public class RDXROS2BigDepthVideoVisualizer extends RDXROS2ImageVisualizer<BigVideoPacket>
{
   private final String titleBeforeAdditions;
   private final ROS2Topic<BigVideoPacket> topic;
   private AsyncROS2Node asyncROS2Node = null;
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final Object syncObject = new Object();
   private BytePointer messageBytePointer;
   private Mat inputDepthMat;
   private Mat normalizedScaledImage;
//   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();

   public RDXROS2BigDepthVideoVisualizer(String title, ROS2Topic<BigVideoPacket> topic)
   {
      super(title, topic.getName(), false);
      titleBeforeAdditions = title;
      this.topic = topic;

      addActivenessChangeCallback(isActive ->
      {
         if (isActive && asyncROS2Node == null)
         {
            subscribe();
         }
         else if (!isActive && asyncROS2Node != null)
         {
            unsubscribe();
         }
      });
   }

   private void subscribe()
   {
      this.asyncROS2Node = new AsyncROS2Node(StringTools.titleToSnakeCase(titleBeforeAdditions));
      // synchronize with the update method
      asyncROS2Node.createSubscription(topic, subscriber ->
      {
         synchronized (syncObject)
         {
            if (!subscriber.read(videoPacket))
               return;
//            delayPlot.addValue(TimeTools.calculateDelay(videoPacket.getAcquisitionTimeSecondsSinceEpoch(), videoPacket.getAcquisitionTimeAdditionalNanos()));
         }
         submitImageUpdate(imageVisualizer ->
         {
            synchronized (syncObject)
            {
               IDLByteSequence imageTByteArrayList = videoPacket.getData();
               int numberOfBytes = imageTByteArrayList.size();

               if (messageBytePointer == null || messageBytePointer.capacity() < imageTByteArrayList.capacity())
               {
                  messageBytePointer = new BytePointer(imageTByteArrayList.size());
               }

               messageBytePointer.position(0);
               messageBytePointer.put(imageTByteArrayList.getBuffer().array(), imageTByteArrayList.getBuffer().arrayOffset(), imageTByteArrayList.size());
               messageBytePointer.limit(imageTByteArrayList.size());

               if (inputDepthMat == null)
               {
                  inputDepthMat = new Mat(videoPacket.getImageHeight(), videoPacket.getImageWidth(), opencv_core.CV_32FC1);
               }

               inputDepthMat.data(messageBytePointer);

               messageSizeReadout.update(numberOfBytes);

               getFrequency().ping();
            }

            if (normalizedScaledImage == null)
            {
               normalizedScaledImage = new Mat(videoPacket.getImageHeight(), videoPacket.getImageWidth(), opencv_core.CV_32FC1);
            }

            OpenCVTools.clampTo8BitUnsignedChar(inputDepthMat, normalizedScaledImage, 0.0, 255.0);

            synchronized (this) // synchronize with the update method
            {
               imageVisualizer.setImage(normalizedScaledImage, PixelFormat.GRAY8);
            }
         });
      });
   }

   @Override
   public void renderImGuiWidgets()
   {
      messageSizeReadout.renderImGuiWidgets();
//      if (getHasReceivedOne())
//      {
//         delayPlot.renderImGuiWidgets();
//      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      unsubscribe();
   }

   private void unsubscribe()
   {
      if (asyncROS2Node != null)
      {
         asyncROS2Node.close();
         asyncROS2Node = null;
      }
   }

   @Override
   public ROS2Topic<BigVideoPacket> getTopic()
   {
      return topic;
   }
}
