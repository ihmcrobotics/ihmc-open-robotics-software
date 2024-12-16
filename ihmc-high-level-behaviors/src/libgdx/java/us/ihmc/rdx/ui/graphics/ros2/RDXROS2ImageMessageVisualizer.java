package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.opencv.opencv_core.Mat;
import org.jetbrains.annotations.Nullable;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.perception.imageMessage.ImageMessageDecoder;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

public class RDXROS2ImageMessageVisualizer extends RDXROS2OpenCVVideoVisualizer<ImageMessage>
{
   private final ROS2Topic<ImageMessage> topic;
   private final ROS2Node ros2Node;
   @Nullable
   private ROS2Subscription<ImageMessage> subscription;
   private final ImageMessageDecoder decoder = new ImageMessageDecoder();
   private final SwapReference<ImageMessage> imageMessageSwapReference = new SwapReference<>(new ImageMessage(), new ImageMessage());
   private final Mat decompressedImage = new Mat();
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot sequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();

   public RDXROS2ImageMessageVisualizer(String title, ROS2Node ros2Node, ROS2Topic<ImageMessage> topic)
   {
      super(title, topic.getName(), false);
      this.topic = topic;

      this.ros2Node = ros2Node;

      setActivenessChangeCallback(isActive ->
      {
         if (isActive && subscription == null)
            subscribe();
         else if (!isActive && subscription != null)
            unsubscribe();
      });
   }

   private void subscribe()
   {
      if (subscription != null)
         subscription.remove();
      subscription = ros2Node.createSubscription(topic, subscriber -> queueRenderImage(subscriber.takeNextData()));
   }

   private void unsubscribe()
   {
      if (subscription != null)
         subscription.remove();
      subscription = null;
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

   private void queueRenderImage(ImageMessage imageMessage)
   {
      // A new message arrived, update the receive-frequency text
      getFrequency().ping();

      // Pack the message data into thread 1's image message object
      ImageMessage imageMessageA = imageMessageSwapReference.getForThreadOne();
      imageMessageA.getData().resetQuick();
      imageMessageA.set(imageMessage);

      // Update some message statistics
      messageSizeReadout.update(imageMessageA.getData().size());
      sequenceDiscontinuityPlot.update(imageMessageA.getSequenceNumber());

      // Hand the image message over to thread 2 for image decoding & visualization
      imageMessageSwapReference.swap();

      // This is thread 2!
      getOpenCVVideoVisualizer().doReceiveMessageOnThread(() ->
      {
         ImageMessage imageMessageB;
         synchronized (imageMessageSwapReference)
         {
            imageMessageB = imageMessageSwapReference.getForThreadTwo();
            PixelFormat imagePixelFormat = PixelFormat.fromImageMessage(imageMessageB);

            /*
             * Depth images can't be directly converted to RGBA, so we must first convert it into an 8 bit gray image by clamping the values,
             * and then convert from gray to RGBA. This also helps "brighten" the depth image when the range of depth values is small.
             * This is purely for visualization, and the resulting RGBA image cannot be used to measure depth.
             */
            if (imagePixelFormat == PixelFormat.GRAY16)
            {
               decoder.decodeMessage(imageMessageB, decompressedImage);
               OpenCVTools.clampTo8BitUnsignedChar(decompressedImage, decompressedImage, 0.0, 255.0);
               OpenCVTools.convertGrayToRGBA(decompressedImage, decompressedImage);
            }
            else
               decoder.decodeMessageToRGBA(imageMessageB, decompressedImage);
         }

         synchronized (this) // synchronize with the update method
         {  // Update the visualization dimensions to match the image
            getOpenCVVideoVisualizer().updateImageDimensions(imageMessageB.getImageWidth(), imageMessageB.getImageHeight());
            // Copy the decompressed image to the image being visualized
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
