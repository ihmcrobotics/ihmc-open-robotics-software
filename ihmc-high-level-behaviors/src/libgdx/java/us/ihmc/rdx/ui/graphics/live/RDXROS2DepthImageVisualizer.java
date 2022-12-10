package us.ihmc.rdx.ui.graphics.live;

import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.rdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class RDXROS2DepthImageVisualizer extends RDXOpenCVVideoVisualizer
{
   private final ROS2Topic<ImageMessage> topic;
   private final RealtimeROS2Node realtimeROS2Node;
   private final ImageMessage imageMessage = new ImageMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Object syncObject = new Object();
   private int depthWidth;
   private int depthHeight;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;
   private Mat inputCompressedDepthMat;
   private Mat decompressedDepthMat;
   private Mat normalizedScaledImage;
   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);

   public RDXROS2DepthImageVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<ImageMessage> topic)
   {
      super(title + " (ROS 2)", topic.getName(), false);
      this.topic = topic;
      this.realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(title));
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT(), subscriber ->
      {
         synchronized (syncObject)
         {
            imageMessage.getData().resetQuick();
            subscriber.takeNextData(imageMessage, sampleInfo);
            delayPlot.addValue(TimeTools.calculateDelay(imageMessage.getAcquisitionTimeSecondsSinceEpoch(), imageMessage.getAcquisitionTimeAdditionalNanos()));
         }
         doReceiveMessageOnThread(() ->
         {
            int numberOfBytes;
            int imageFormat;
            synchronized (syncObject) // For interacting with the ImageMessage
            {
               depthWidth = imageMessage.getImageWidth();
               depthHeight = imageMessage.getImageHeight();
               imageFormat = imageMessage.getFormat();

               if (incomingCompressedImageBuffer == null)
               {
                  // TODO: The 2 is because 16 bit, two bytes, we should put this in ImageMessage
                  // TODO: Possibly even ImageMessage should be split into ColorImageMessage and DepthImageMessage
                  incomingCompressedImageBuffer = ByteBuffer.allocateDirect(depthWidth * depthHeight * 2);
                  incomingCompressedImageBuffer.order(ByteOrder.nativeOrder());
                  incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);

                  inputCompressedDepthMat = new Mat(1, 1, opencv_core.CV_8UC1);
                  decompressedDepthMat = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1);
                  normalizedScaledImage = new Mat(depthHeight, depthWidth, opencv_core.CV_32FC1);
               }

               numberOfBytes = imageMessage.getData().size();
               incomingCompressedImageBuffer.rewind();
               incomingCompressedImageBuffer.limit(depthWidth * depthHeight * 2);
               for (int i = 0; i < numberOfBytes; i++)
               {
                  incomingCompressedImageBuffer.put(imageMessage.getData().get(i));
               }
               incomingCompressedImageBuffer.flip();
            }

            inputCompressedDepthMat.cols(numberOfBytes);
            inputCompressedDepthMat.data(incomingCompressedImageBytePointer);

            System.out.println(inputCompressedDepthMat.ptr(7, 39).getShort());
            System.out.println(inputCompressedDepthMat.ptr(7, 56).getShort());
            System.out.println(inputCompressedDepthMat.ptr(7, 324).getShort());
            System.out.println(inputCompressedDepthMat.ptr(7, 885).getShort());
            System.out.println(inputCompressedDepthMat.ptr(7, 1788).getShort());

            opencv_imgcodecs.imdecode(inputCompressedDepthMat, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedDepthMat);

            BytedecoOpenCVTools.clampTo8BitUnsignedChar(decompressedDepthMat, normalizedScaledImage, 0.0, 255.0);

            synchronized (this) // synchronize with the update method
            {
               updateImageDimensions(imageMessage.getImageWidth(), imageMessage.getImageHeight());
               BytedecoOpenCVTools.convertGrayToRGBA(normalizedScaledImage, getRGBA8Mat());
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
