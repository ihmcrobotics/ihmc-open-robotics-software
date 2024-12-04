package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2SRTStreamTopicPair;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;

import javax.annotation.Nullable;
import java.time.Instant;

public class ROS2SRTVideoStreamImageMessageRelayWorker
{
   /**
    * Set to true to enable compression to JPEG (using nvJPEG) before publishing the ImageMessage.
    * Introduces GPU overhead and more latency, but useful for recording rosbags.
    */
   private static final boolean COMPRESS_USING_NVJPEG = true;

   private final ROS2PublisherBasics<ImageMessage> publisher;
   private final ROS2SRTVideoSubscriber subscriber;

   @Nullable
   private CUDAJPEGProcessor cudajpegProcessor;

   private final ImageMessage imageMessage;
   private long frameSequenceNumber = 0L;

   public ROS2SRTVideoStreamImageMessageRelayWorker(ROS2Node publisherNode, ROS2Node subscriberNode, ROS2SRTStreamTopicPair streamTopicPair)
   {
      PixelFormat outputPixelFormat = streamTopicPair.isDepth() ? PixelFormat.GRAY16 : PixelFormat.BGR8;

      imageMessage = new ImageMessage();
      imageMessage.setPixelFormat(outputPixelFormat.toByte());
      imageMessage.setCompressionType(COMPRESS_USING_NVJPEG ? CompressionType.NVJPEG.toByte() : CompressionType.UNCOMPRESSED.toByte());
      imageMessage.setCameraModel(CameraModel.PINHOLE.toByte());

      // Create publisher and subscriber using two separate nodes as publisher should ideally only publish on loopback.
      publisher = publisherNode.createPublisher(streamTopicPair.imageMessageTopic());
      subscriber = new ROS2SRTVideoSubscriber(new ROS2Helper(subscriberNode), streamTopicPair.streamStatusTopic(), outputPixelFormat);
      subscriber.addNewFrameConsumer(this::republishFrameAsImageMessage);
      subscriber.subscribe();

      if (COMPRESS_USING_NVJPEG)
         cudajpegProcessor = new CUDAJPEGProcessor(90);
   }

   public void destroy()
   {
      subscriber.destroy();
      publisher.remove();
      if (COMPRESS_USING_NVJPEG && cudajpegProcessor != null)
         cudajpegProcessor.destroy();
   }

   private void republishFrameAsImageMessage(RawImage frame)
   {
      // Set acquisition time as now... this isn't super accurate though
      MessageTools.toMessage(Instant.now(), imageMessage.getAcquisitionTime());

      Mat frameMat = frame.getCpuImageMat();

      if (COMPRESS_USING_NVJPEG && cudajpegProcessor != null)
      {
         BytePointer encodedData = new BytePointer(OpenCVTools.dataSize(frameMat));
         cudajpegProcessor.encodeBGR(frameMat, encodedData);
         PerceptionMessageTools.packImageMessageData(imageMessage, encodedData);
         encodedData.close();
      }
      else
      {
         PerceptionMessageTools.packImageMessageData(imageMessage, frameMat.data().limit(OpenCVTools.dataSize(frameMat)));
      }

      // Pack the image message meta data
      frame.packImageMessageMetaData(imageMessage);

      if (frame.getPixelFormat() == PixelFormat.GRAY16)
         System.out.println("Publishing depth????");

      // Send the message
      publisher.publish(imageMessage);
   }
}
