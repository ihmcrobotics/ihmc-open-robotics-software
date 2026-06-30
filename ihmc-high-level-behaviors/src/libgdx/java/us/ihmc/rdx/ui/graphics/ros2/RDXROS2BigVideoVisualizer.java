package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.BigVideoPacket;
import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.tools.string.StringTools;

public class RDXROS2BigVideoVisualizer extends RDXROS2ImageVisualizer<BigVideoPacket>
{
   private final String titleBeforeAdditions;
   private final ROS2Topic<BigVideoPacket> topic;
   private AsyncROS2Node realtimeROS2Node = null;
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final Object syncObject = new Object();
   private final BytePointer messageEncodedBytePointer = new BytePointer(25000000);
   private final Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
   //   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();

   public RDXROS2BigVideoVisualizer(String title, ROS2Topic<BigVideoPacket> topic)
   {
      super(title, topic.getName(), false);
      titleBeforeAdditions = title;
      this.topic = topic;

      addActivenessChangeCallback(isAlive ->
      {
         if (isAlive && realtimeROS2Node == null)
         {
            subscribe();
         }
         else if (!isAlive && realtimeROS2Node != null)
         {
            unsubscribe();
         }
      });
   }

   private void subscribe()
   {
      realtimeROS2Node = new AsyncROS2Node(StringTools.titleToSnakeCase(titleBeforeAdditions));
      // imdecode takes the longest by far out of all this stuff
      // synchronize with the update method
      // YUV I420 has 1.5 times the height of the image
      realtimeROS2Node.createSubscription(topic, subscriber ->
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
               int numberOfBytes = videoPacket.getData().size();
               IDLByteSequence data = videoPacket.getData();
               messageEncodedBytePointer.put(data.getBuffer().array(), data.getBuffer().arrayOffset(), data.size());
               messageEncodedBytePointer.limit(numberOfBytes);

               inputJPEGMat.cols(numberOfBytes);
               inputJPEGMat.data(messageEncodedBytePointer);

               messageSizeReadout.update(numberOfBytes);
            }

            // imdecode takes the longest by far out of all this stuff
            opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, inputYUVI420Mat);

            synchronized (this) // synchronize with the update method
            {
               imageVisualizer.setImage(inputYUVI420Mat, PixelFormat.YUV_I420);
            }

            getFrequency().ping();
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
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.close();
         realtimeROS2Node = null;
      }
   }

   @Override
   public ROS2Topic<BigVideoPacket> getTopic()
   {
      return topic;
   }
}
