package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jetbrains.annotations.Nullable;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

import java.nio.ByteBuffer;

public class RDXROS2ImageMessageVisualizer extends RDXROS2OpenCVVideoVisualizer<ImageMessage>
{
   private final String titleBeforeAdditions;
   private final PubSubImplementation pubSubImplementation;
   private final ROS2Topic<ImageMessage> topic;
   private RealtimeROS2Node realtimeROS2Node;
   private final ImageMessage imageMessage = new ImageMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Object syncObject = new Object();
   private int imageWidth;
   private int imageHeight;
   private int numberOfPixels;
   private int bytesIfUncompressed = 0;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;
   private Mat compressedBytesMat;
   private Mat decompressedImage;
   private Mat normalizedScaledImage;
//   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);
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
      synchronized (syncObject)
      {
         imageMessage.getData().resetQuick();
         subscriber.takeNextData(imageMessage, sampleInfo);
//         delayPlot.addValue(TimeTools.calculateDelay(imageMessage.getAcquisitionTime().getSecondsSinceEpoch(),
//                                                     imageMessage.getAcquisitionTime().getAdditionalNanos()));
         getFrequency().ping();
      }
      getOpenCVVideoVisualizer().doReceiveMessageOnThread(() ->
      {
         int numberOfBytes;
         synchronized (syncObject) // For interacting with the ImageMessage
         {
            imageWidth = imageMessage.getImageWidth();
            imageHeight = imageMessage.getImageHeight();
            numberOfPixels = imageWidth * imageHeight;

            if (incomingCompressedImageBuffer == null)
            {
               switch (ImageMessageFormat.getFormat(imageMessage))
               {
                  case GRAY_PNG_8UC1 ->
                  {
                     LogTools.info("Creating Image Message Visualizer for {} with type PNG_8UC1", topic.getName());
                     bytesIfUncompressed = numberOfPixels;
                     incomingCompressedImageBuffer = NativeMemoryTools.allocate(bytesIfUncompressed);
                     incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);

                     compressedBytesMat = new Mat(1, 1, opencv_core.CV_8UC1);
                     decompressedImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC1);
                  }
                  case DEPTH_PNG_16UC1 ->
                  {
                     LogTools.info("Creating Image Message Visualizer for {} with type DEPTH_PNG_16UC1", topic.getName());
                     bytesIfUncompressed = numberOfPixels * 2;
                     incomingCompressedImageBuffer = NativeMemoryTools.allocate(bytesIfUncompressed);
                     incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);

                     compressedBytesMat = new Mat(1, 1, opencv_core.CV_8UC1);
                     decompressedImage = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
                     normalizedScaledImage = new Mat(imageHeight, imageWidth, opencv_core.CV_32FC1);
                  }
                  case COLOR_JPEG_YUVI420 ->
                  {
                     LogTools.info("Creating Image Message Visualizer for {} with type COLOR_JPEG_YUVI420", topic.getName());
                     bytesIfUncompressed = numberOfPixels * 3;
                     incomingCompressedImageBuffer = NativeMemoryTools.allocate(bytesIfUncompressed);
                     incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);

                     compressedBytesMat = new Mat(1, 1, opencv_core.CV_8UC1);
                     decompressedImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3);
                  }
                  case COLOR_JPEG_BGR8 ->
                  {
                     LogTools.info("Creating Image Message Visualizer for {} with the type COLOR_JPEG_BGR8", topic.getName());
                     bytesIfUncompressed = numberOfPixels * 3;
                     incomingCompressedImageBuffer = NativeMemoryTools.allocate(bytesIfUncompressed);
                     incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);

                     compressedBytesMat = new Mat(1, 1, opencv_core.CV_8UC1);
                     decompressedImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3);
                  }
               }
            }

            numberOfBytes = imageMessage.getData().size();
            incomingCompressedImageBuffer.rewind();
            incomingCompressedImageBuffer.limit(bytesIfUncompressed);
            for (int i = 0; i < numberOfBytes; i++)
            {
               incomingCompressedImageBuffer.put(imageMessage.getData().get(i));
            }
            incomingCompressedImageBuffer.flip();

            messageSizeReadout.update(numberOfBytes);
            sequenceDiscontinuityPlot.update(imageMessage.getSequenceNumber());
         }

         compressedBytesMat.cols(numberOfBytes);
         compressedBytesMat.data(incomingCompressedImageBytePointer);
         opencv_imgcodecs.imdecode(compressedBytesMat, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedImage);

         synchronized (this) // synchronize with the update method
         {
            getOpenCVVideoVisualizer().updateImageDimensions(imageMessage.getImageWidth(), imageMessage.getImageHeight());

            switch (ImageMessageFormat.getFormat(imageMessage))
            {
               case GRAY_PNG_8UC1 ->
               {
                  OpenCVTools.convertGrayToRGBA(decompressedImage, getOpenCVVideoVisualizer().getRGBA8Mat());
               }
               case DEPTH_PNG_16UC1 ->
               {
                  OpenCVTools.clampTo8BitUnsignedChar(decompressedImage, normalizedScaledImage, 0.0, 255.0);
                  OpenCVTools.convertGrayToRGBA(normalizedScaledImage, getOpenCVVideoVisualizer().getRGBA8Mat());
               }
               case COLOR_JPEG_YUVI420 ->
               {
                  opencv_imgproc.cvtColor(decompressedImage, getOpenCVVideoVisualizer().getRGBA8Mat(), opencv_imgproc.COLOR_YUV2RGBA_I420);
               }
               case COLOR_JPEG_BGR8 ->
               {
                  opencv_imgproc.cvtColor(decompressedImage, getOpenCVVideoVisualizer().getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA);
               }
            }
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
   }

   public void renderStatistics()
   {
      messageSizeReadout.renderImGuiWidgets();
//      delayPlot.renderImGuiWidgets();
      sequenceDiscontinuityPlot.renderImGuiWidgets();
   }

   @Override
   public ROS2Topic<ImageMessage> getTopic()
   {
      return topic;
   }
}
