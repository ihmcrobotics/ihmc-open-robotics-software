package us.ihmc.rdx.ui.graphics.live.ros2;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.memory.NativeMemoryTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.live.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.live.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.live.RDXSequenceDiscontinuityPlot;
import us.ihmc.rdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

import java.nio.ByteBuffer;

public class RDXROS2DepthImageVisualizer extends RDXOpenCVVideoVisualizer
{
   private final String titleBeforeAdditions;
   private final PubSubImplementation pubSubImplementation;
   private final ROS2Topic<ImageMessage> topic;
   private RealtimeROS2Node realtimeROS2Node;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean subscribed = new ImBoolean(true);
   private final ImageMessage imageMessage = new ImageMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Object syncObject = new Object();
   private int depthWidth;
   private int depthHeight;
   private int numberOfPixels;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;
   private Mat inputCompressedDepthMat;
   private Mat decompressedDepthMat;
   private Mat normalizedScaledImage;
   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot sequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();

   public RDXROS2DepthImageVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<ImageMessage> topic)
   {
      super(title + " (ROS 2)", topic.getName(), false);
      titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      this.topic = topic;

      setSubscribed(subscribed.get());
   }

   private void subscribe()
   {
      subscribed.set(true);
      this.realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT(), this::queueRenderImage);
      realtimeROS2Node.spin();
   }

   private void queueRenderImage(Subscriber<ImageMessage> subscriber)
   {
      synchronized (syncObject)
      {
         imageMessage.getData().resetQuick();
         subscriber.takeNextData(imageMessage, sampleInfo);
         delayPlot.addValue(TimeTools.calculateDelay(imageMessage.getAcquisitionTime().getSecondsSinceEpoch(),
                                                     imageMessage.getAcquisitionTime().getAdditionalNanos()));
      }
      doReceiveMessageOnThread(() ->
      {
         int numberOfBytes;
         int imageFormat;
         synchronized (syncObject) // For interacting with the ImageMessage
         {
            depthWidth = imageMessage.getImageWidth();
            depthHeight = imageMessage.getImageHeight();
            numberOfPixels = depthWidth * depthHeight;
            imageFormat = imageMessage.getFormat(); // TODO: Use this when we introduce more formats

            if (incomingCompressedImageBuffer == null)
            {
               // TODO: Store bytes per pixel in ImageMessage
               // TODO: Split ImageMessage into ColorImageMessage and DepthImageMessage
               int bytesIfUncompressed = numberOfPixels * Short.BYTES;
               incomingCompressedImageBuffer = NativeMemoryTools.allocate(bytesIfUncompressed);
               incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);

               inputCompressedDepthMat = new Mat(1, 1, opencv_core.CV_8UC1);
               decompressedDepthMat = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1);
               normalizedScaledImage = new Mat(depthHeight, depthWidth, opencv_core.CV_32FC1);
            }

            numberOfBytes = imageMessage.getData().size();
            incomingCompressedImageBuffer.rewind();
            incomingCompressedImageBuffer.limit(depthWidth * depthHeight * Short.BYTES);
            for (int i = 0; i < numberOfBytes; i++)
            {
               incomingCompressedImageBuffer.put(imageMessage.getData().get(i));
            }
            incomingCompressedImageBuffer.flip();

            messageSizeReadout.update(numberOfBytes);
            sequenceDiscontinuityPlot.update(imageMessage.getSequenceNumber());
         }

         inputCompressedDepthMat.cols(numberOfBytes);
         inputCompressedDepthMat.data(incomingCompressedImageBytePointer);

         opencv_imgcodecs.imdecode(inputCompressedDepthMat, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedDepthMat);

         BytedecoOpenCVTools.clampTo8BitUnsignedChar(decompressedDepthMat, normalizedScaledImage, 0.0, 255.0);

         synchronized (this) // synchronize with the update method
         {
            updateImageDimensions(imageMessage.getImageWidth(), imageMessage.getImageHeight());
            BytedecoOpenCVTools.convertGrayToRGBA(normalizedScaledImage, getRGBA8Mat());
         }
      });
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.getHidden(getTitle() + "Subscribed"), subscribed))
      {
         setSubscribed(subscribed.get());
      }
      ImGuiTools.previousWidgetTooltip("Subscribed");
      ImGui.sameLine();
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      if (getHasReceivedOne())
      {
         messageSizeReadout.renderImGuiWidgets();
         getFrequencyPlot().renderImGuiWidgets();
         delayPlot.renderImGuiWidgets();
         sequenceDiscontinuityPlot.renderImGuiWidgets();
      }
   }

   public void setSubscribed(boolean subscribed)
   {
      if (subscribed && realtimeROS2Node == null)
      {
         subscribe();
      }
      else if (!subscribed && realtimeROS2Node != null)
      {
         unsubscribe();
      }
   }

   private void unsubscribe()
   {
      subscribed.set(false);
      realtimeROS2Node.destroy();
      realtimeROS2Node = null;
   }

   public boolean isSubscribed()
   {
      return subscribed.get();
   }

   @Override
   public void destroy()
   {
      super.destroy();
      realtimeROS2Node.destroy();
   }
}
