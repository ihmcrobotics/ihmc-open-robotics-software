package us.ihmc.gdx.ui.graphics.live;

import imgui.internal.ImGui;
import org.bytedeco.opencv.global.opencv_core;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1VisualizerInterface;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class GDXROS1VideoVisualizer extends GDXOpenCVVideoVisualizer implements ImGuiGDXROS1VisualizerInterface
{
   private final boolean isCompressed;
   private AbstractRosTopicSubscriber<Image> subscriber;
   private AbstractRosTopicSubscriber<CompressedImage> compressedSubscriber;
   private final String topic;
   private GDXROS1VideoProcessor videoProcessor;
   private boolean currentlySubscribed = false;

   public GDXROS1VideoVisualizer(String title, String topic)
   {
      this(title, topic, false);
   }

   public GDXROS1VideoVisualizer(String title, String topic, boolean flipY)
   {
      super(title + " (ROS 1)", topic, flipY);
      this.topic = topic;
      isCompressed = topic.endsWith("compressed");
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      if (isCompressed)
      {
         compressedSubscriber = new AbstractRosTopicSubscriber<CompressedImage>(sensor_msgs.CompressedImage._TYPE)
         {
            @Override
            public void onNewMessage(CompressedImage image)
            {
               doReceiveMessageOnThread(() -> processIncomingMessageOnThread(image));
            }
         };
         ros1Node.attachSubscriber(topic, compressedSubscriber);
      }
      else
      {
         subscriber = new AbstractRosTopicSubscriber<Image>(sensor_msgs.Image._TYPE)
         {
            @Override
            public void onNewMessage(Image image)
            {
               doReceiveMessageOnThread(() -> processIncomingMessageOnThread(image));
            }
         };
         ros1Node.attachSubscriber(topic, subscriber);
      }
   }

   private void processIncomingMessageOnThread(CompressedImage compressedImage)
   {

   }

   private void processIncomingMessageOnThread(Image image)
   {
      ChannelBuffer nettyChannelBuffer = image.getData();

      int imageWidth = image.getWidth();
      int imageHeight = image.getHeight();
      String encoding = image.getEncoding();
      int cvType = ImageEncodingTools.getCvType(encoding);

      if (videoProcessor == null)
      {
         if (cvType == opencv_core.CV_16UC1)
         {
            videoProcessor = new GDXROS1GrayscaleVideoProcessing();
         }
         else if (cvType == opencv_core.CV_8UC4)
         {
            videoProcessor = new GDXROS1RGBA8VideoProcessing();
         }
         else
         {
            LogTools.error("Implement for cvType: {}", cvType);
         }
      }

      if (videoProcessor != null)
      {
         videoProcessor.prepare(imageWidth, imageHeight, nettyChannelBuffer);

         synchronized (this)
         {
            updateImageDimensions(imageWidth, imageHeight);
            videoProcessor.synchronizedPackPanelImage(getRGBA8Mat(), getRgba8888BytePointer());
         }
      }
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      if (isCompressed)
      {
         ros1Node.removeSubscriber(compressedSubscriber);
      }
      else
      {
         ros1Node.removeSubscriber(subscriber);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic);
      if (getHasReceivedOne())
         getFrequencyPlot().renderImGuiWidgets();
   }

   public void updateSubscribers(RosNodeInterface ros1Node)
   {
      boolean active = isActive();
      if (active != currentlySubscribed)
      {
         if (active)
         {
            subscribe(ros1Node);
         }
         else
         {
            unsubscribe(ros1Node);
         }
      }
      currentlySubscribed = active;
   }
}
