package us.ihmc.rdx.ui.graphics.ros2;

import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.BigVideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.rdx.imgui.ImPlotDoublePlot;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

public class RDXROS2BigDepthVideoVisualizer extends RDXOpenCVVideoVisualizer
{
   private final String titleBeforeAdditions;
   private final PubSubImplementation pubSubImplementation;
   private final ROS2Topic<BigVideoPacket> topic;
   private RealtimeROS2Node realtimeROS2Node = null;
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Object syncObject = new Object();
   private byte[] messageDataHeapArray;
   private BytePointer messageBytePointer;
   private Mat inputDepthMat;
   private Mat normalizedScaledImage;
   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();

   public RDXROS2BigDepthVideoVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<BigVideoPacket> topic)
   {
      super(title + " (ROS 2)", topic.getName(), false);
      titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      this.topic = topic;

      setActivenessChangeCallback(isActive ->
      {
         if (isActive && realtimeROS2Node == null)
         {
            subscribe();
         }
         else if (!isActive && realtimeROS2Node != null)
         {
            unsubscribe();
         }
      });
   }

   private void subscribe()
   {
      this.realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      // synchronize with the update method
      realtimeROS2Node.createSubscription(topic, subscriber ->
      {
         synchronized (syncObject)
         {
            videoPacket.getData().resetQuick();
            subscriber.takeNextData(videoPacket, sampleInfo);
            delayPlot.addValue(TimeTools.calculateDelay(videoPacket.getAcquisitionTimeSecondsSinceEpoch(), videoPacket.getAcquisitionTimeAdditionalNanos()));
         }
         doReceiveMessageOnThread(() ->
         {
            synchronized (syncObject)
            {
               IDLSequence.Byte imageTByteArrayList = videoPacket.getData();
               int numberOfBytes = imageTByteArrayList.size();

               if (messageDataHeapArray == null || messageDataHeapArray.length != imageTByteArrayList.size())
               {
                  messageDataHeapArray = new byte[imageTByteArrayList.size()];
                  messageBytePointer = new BytePointer(imageTByteArrayList.size());
               }

               imageTByteArrayList.toArray(messageDataHeapArray);
               messageBytePointer.position(0);
               messageBytePointer.put(messageDataHeapArray, 0, imageTByteArrayList.size());
               messageBytePointer.limit(imageTByteArrayList.size());

               if (inputDepthMat == null)
               {
                  inputDepthMat = new Mat(videoPacket.getImageHeight(), videoPacket.getImageWidth(), opencv_core.CV_32FC1);
               }

               inputDepthMat.data(messageBytePointer);

               messageSizeReadout.update(numberOfBytes);
            }

            if (normalizedScaledImage == null)
            {
               normalizedScaledImage = new Mat(videoPacket.getImageHeight(), videoPacket.getImageWidth(), opencv_core.CV_32FC1);
            }

            OpenCVTools.clampTo8BitUnsignedChar(inputDepthMat, normalizedScaledImage, 0.0, 255.0);

            synchronized (this) // synchronize with the update method
            {
               updateImageDimensions(videoPacket.getImageWidth(), videoPacket.getImageHeight());
               OpenCVTools.convert8BitGrayTo8BitRGBA(normalizedScaledImage, getRGBA8Mat());
            }
         });
      });
      realtimeROS2Node.spin();
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      messageSizeReadout.renderImGuiWidgets();
      if (getHasReceivedOne())
      {
         getFrequencyPlot().renderImGuiWidgets();
         delayPlot.renderImGuiWidgets();
      }
   }

   @Override
   public void destroy()
   {
      unsubscribe();
      super.destroy();
   }

   private void unsubscribe()
   {
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }
   }
}
