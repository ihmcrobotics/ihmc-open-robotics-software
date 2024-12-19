package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
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
import us.ihmc.ros2.ROS2Publisher;

import javax.annotation.Nullable;
import java.time.Instant;

public class ROS2SRTVideoStreamImageMessageRelayWorker
{
   private final ROS2Publisher<ImageMessage> publisher;
   private final ROS2SRTVideoSubscriber subscriber;

   /**
    * Which compression type to use before publishing the ImageMessage.
    * Anything other than UNCOMPRESSED will incur some performance and latency
    * penalty but is useful for things such as recording rosbags.
    */
   private final CompressionType compressionType;

   @Nullable
   private CUDAJPEGProcessor cudajpegProcessor;

   private final ImageMessage imageMessage;
   private long frameSequenceNumber = 0L;

   public ROS2SRTVideoStreamImageMessageRelayWorker(ROS2Node loopbackPublisherNode, ROS2Node subscriberNode, ROS2SRTStreamTopicPair streamTopicPair)
   {
      this(loopbackPublisherNode, subscriberNode, streamTopicPair, CompressionType.UNCOMPRESSED);
   }

   public ROS2SRTVideoStreamImageMessageRelayWorker(ROS2Node loopbackPublisherNode,
                                                    ROS2Node subscriberNode,
                                                    ROS2SRTStreamTopicPair streamTopicPair,
                                                    CompressionType compressionType)
   {
      PixelFormat outputPixelFormat = streamTopicPair.isDepth() ? PixelFormat.GRAY16 : PixelFormat.BGR8;

      imageMessage = new ImageMessage();
      imageMessage.setPixelFormat(outputPixelFormat.toByte());
      imageMessage.setCompressionType(compressionType.toByte());
      imageMessage.setCameraModel(CameraModel.PINHOLE.toByte());

      // Create publisher and subscriber using two separate nodes as publisher should ideally only publish on loopback.
      publisher = loopbackPublisherNode.createPublisher(streamTopicPair.imageMessageTopic());
      subscriber = new ROS2SRTVideoSubscriber(new ROS2Helper(subscriberNode), streamTopicPair.streamStatusTopic(), outputPixelFormat);
      subscriber.addNewFrameConsumer(this::republishFrameAsImageMessage);
      subscriber.subscribe();

      this.compressionType = compressionType;

      switch (compressionType)
      {
         case UNCOMPRESSED, JPEG, PNG:
            break;
         case NVJPEG:
            cudajpegProcessor = new CUDAJPEGProcessor(90);
            break;
         default:
            throw new RuntimeException(compressionType.name() + " compression type not supported in " + getClass().getSimpleName());
      }
   }

   public void destroy()
   {
      subscriber.destroy();
      publisher.remove();
      if (cudajpegProcessor != null)
         cudajpegProcessor.destroy();
   }

   private void republishFrameAsImageMessage(RawImage frame)
   {
      if (frame.getPixelFormat() == PixelFormat.GRAY16)
         throw new RuntimeException("Unsupported PixelFormat (trying to republish a depth image?)");

      // Set acquisition time as now... this isn't super accurate though
      MessageTools.toMessage(Instant.now(), imageMessage.getAcquisitionTime());

      Mat frameMat = frame.getCpuImageMat();

      switch (compressionType)
      {
         case UNCOMPRESSED ->
         {
            PerceptionMessageTools.packImageMessageData(imageMessage, frameMat.data().limit(OpenCVTools.dataSize(frameMat)));
         }
         case JPEG ->
         {
            BytePointer encodedData = new BytePointer(OpenCVTools.dataSize(frameMat));
            opencv_imgcodecs.imencode(".jpg", frameMat, encodedData);
            PerceptionMessageTools.packImageMessageData(imageMessage, encodedData);
            encodedData.close();
         }
         case NVJPEG ->
         {
            BytePointer encodedData = new BytePointer(OpenCVTools.dataSize(frameMat));
            if (cudajpegProcessor != null)
               cudajpegProcessor.encodeBGR(frameMat, encodedData);
            PerceptionMessageTools.packImageMessageData(imageMessage, encodedData);
            encodedData.close();
         }
         case PNG ->
         {
            BytePointer encodedData = new BytePointer(OpenCVTools.dataSize(frameMat));
            opencv_imgcodecs.imencode(".png", frameMat, encodedData);
            PerceptionMessageTools.packImageMessageData(imageMessage, encodedData);
            encodedData.close();
         }
      }

      // Pack the image message meta data
      frame.packImageMessageMetaData(imageMessage);

      // Send the message
      publisher.publish(imageMessage);
   }
}
