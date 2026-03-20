package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.opencv.opencv_core.Mat;
import org.jetbrains.annotations.Nullable;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.perception.imageMessage.ImageMessageDecoder;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

public class RDXROS2ImageMessageVisualizer extends RDXROS2ImageVisualizer<ImageMessage>
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
   private volatile boolean hasRenderedOne = false;

   public RDXROS2ImageMessageVisualizer(String title, ROS2Node ros2Node, ROS2Topic<ImageMessage> topic)
   {
      super(title, topic.getName(), false);
      this.topic = topic;

      this.ros2Node = ros2Node;

      addActivenessChangeCallback(isActive ->
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
      subscription = ros2Node.createSubscription2(topic, this::queueRenderImage);
   }

   private void unsubscribe()
   {
      if (subscription != null)
         subscription.remove();
      subscription = null;
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
      submitImageUpdate(imageVisualizer ->
      {
         synchronized (imageMessageSwapReference)
         {
            ImageMessage imageMessageB = imageMessageSwapReference.getForThreadTwo();

            // Decode the message and get the decoded pixel format (it may be different from the pixel format in the message)
            decoder.decodeMessage(imageMessageB, decompressedImage);
            PixelFormat pixelFormat = decoder.getDecodedImagePixelFormat();

            // Update the visualized image
            setImage(imageVisualizer, decompressedImage, pixelFormat);
            hasRenderedOne = true;
         }
      });
   }

   // Protected so child classes can override this method to modify the displayed image
   protected void setImage(RDXImageVisualizer imageVisualizer, Mat image, PixelFormat pixelFormat)
   {
      imageVisualizer.setImage(image, pixelFormat);
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (hasRenderedOne)
      {
         renderStatistics();
      }

      super.renderImGuiWidgets();
   }

   @Override
   public void destroy()
   {
      super.destroy();
      unsubscribe();
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
