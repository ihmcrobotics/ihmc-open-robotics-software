package us.ihmc.gdx.ui.graphics.live;

import controller_msgs.msg.dds.BigVideoPacket;
import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

public class GDXROS2BigDepthVideoVisualizer extends GDXOpenCVVideoVisualizer
{
   private final ROS2Topic<BigVideoPacket> topic;
   private final RealtimeROS2Node realtimeROS2Node;
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Object syncObject = new Object();
   private byte[] messageDataHeapArray;
   private BytePointer messageBytePointer;
   private Mat inputDepthMat;
   private Mat normalizedScaledImage;
   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);

   public GDXROS2BigDepthVideoVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<BigVideoPacket> topic)
   {
      super(title + " (ROS 2)", topic.getName(), false);
      this.topic = topic;
      this.realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(title));
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT(), subscriber ->
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
            }

            if (normalizedScaledImage == null)
            {
               normalizedScaledImage = new Mat(videoPacket.getImageHeight(), videoPacket.getImageWidth(), opencv_core.CV_32FC1);
            }

            BytedecoOpenCVTools.clampTo8BitUnsignedChar(inputDepthMat, normalizedScaledImage, 0.0, 255.0);

            synchronized (this) // synchronize with the update method
            {
               updateImageDimensions(videoPacket.getImageWidth(), videoPacket.getImageHeight());
               BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(normalizedScaledImage, getRGBA8Mat());
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
      if (getHasReceivedOne())
      {
         getFrequencyPlot().renderImGuiWidgets();
         delayPlot.renderImGuiWidgets();
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      realtimeROS2Node.destroy();
   }
}
