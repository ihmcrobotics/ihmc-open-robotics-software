package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.opencv.opencv_core.Mat;
import org.jetbrains.annotations.Nullable;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.perception.imageMessage.ImageMessageDecoder;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.SwapReference;

public class RDXROS2ImageMessageVisualizer extends RDXROS2OpenCVVideoVisualizer<ImageMessage>
{
   private final String titleBeforeAdditions;
   private final PubSubImplementation pubSubImplementation;
   private final ROS2Topic<ImageMessage> topic;
   private RealtimeROS2Node realtimeROS2Node;
   private final ImageMessageDecoder decoder = new ImageMessageDecoder();
   private final SwapReference<ImageMessage> imageMessageSwapReference = new SwapReference<>(new ImageMessage(), new ImageMessage());
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Mat decompressedImage = new Mat();
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot sequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();

   public RDXROS2ImageMessageVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<ImageMessage> topic)
   {
      super(title, topic.getName(), false);
      titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      this.topic = topic;

      setActivenessChangeCallback(isActive ->
      {
         if (isActive && realtimeROS2Node == null)
            subscribe();
         else if (!isActive && realtimeROS2Node != null)
            unsubscribe();
      });
   }

   private void subscribe()
   {
      this.realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      realtimeROS2Node.createSubscription(topic, this::queueRenderImage);
      realtimeROS2Node.spin();
   }

   @Override
   public void update()
   {
      super.update();
      getOpenCVVideoVisualizer().setActive(isActive());
      getOpenCVVideoVisualizer().update();
   }

   @Nullable
   @Override
   public RDXPanel getPanel()
   {
      return getOpenCVVideoVisualizer().getPanel();
   }

   private void queueRenderImage(Subscriber<ImageMessage> subscriber)
   {
      ImageMessage imageMessageA = imageMessageSwapReference.getForThreadOne();
      imageMessageA.getData().resetQuick();
      subscriber.takeNextData(imageMessageA, sampleInfo);
      getFrequency().ping();

      messageSizeReadout.update(imageMessageA.getData().size());
      sequenceDiscontinuityPlot.update(imageMessageA.getSequenceNumber());
      imageMessageSwapReference.swap();

      getOpenCVVideoVisualizer().doReceiveMessageOnThread(() ->
      {
         ImageMessage imageMessageB;
         synchronized (imageMessageSwapReference)
         {
            imageMessageB = imageMessageSwapReference.getForThreadTwo();
            decoder.decodeMessageToRGBA(imageMessageB, decompressedImage);
         }

         synchronized (this) // synchronize with the update method
         {
            getOpenCVVideoVisualizer().updateImageDimensions(imageMessageB.getImageWidth(), imageMessageB.getImageHeight());
            decompressedImage.copyTo(getOpenCVVideoVisualizer().getRGBA8Mat());
         }
      });
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (getOpenCVVideoVisualizer().getHasRenderedOne())
      {
         renderStatistics();
      }

      getOpenCVVideoVisualizer().renderImGuiWidgets();
   }

   private void unsubscribe()
   {
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }
   }

   @Override
   public void destroy()
   {
      unsubscribe();
      super.destroy();
      getOpenCVVideoVisualizer().destroy();
      decoder.destroy();
   }

   public void renderStatistics()
   {
      messageSizeReadout.renderImGuiWidgets();
      sequenceDiscontinuityPlot.renderImGuiWidgets();
   }

   @Override
   public ROS2Topic<ImageMessage> getTopic()
   {
      return topic;
   }
}
