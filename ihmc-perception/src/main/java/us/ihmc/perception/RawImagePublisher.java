package us.ihmc.perception;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.streaming.ROS2SRTSensorStreamer;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import static us.ihmc.perception.imageMessage.CompressionType.NVJPEG;
import static us.ihmc.perception.imageMessage.CompressionType.ZSTD_NVJPEG_HYBRID;

public class RawImagePublisher implements AutoCloseable
{
   private final CUDACompressionTools compressionTools;
   private final CUDAJPEGProcessor jpegProcessor;
   private final ROS2SRTSensorStreamer sensorStreamer;

   private final ROS2Helper ros2Helper;
   private final ImageMessage imageMessage;

   public RawImagePublisher(ROS2Node ros2Node)
   {
      compressionTools = new CUDACompressionTools();
      jpegProcessor = new CUDAJPEGProcessor();
      sensorStreamer = new ROS2SRTSensorStreamer(ros2Node);

      ros2Helper = new ROS2Helper(ros2Node);
      imageMessage = new ImageMessage();
   }

   @SuppressWarnings("unchecked") // Trust me bro, I know what I'm doing
   public void publishImage(ROS2Topic<? extends Packet<?>> imageTopic, RawImage imageToPublish, CameraModel cameraModel) // TODO: Remove CameraModel after PR merge
   {
      if (imageTopic.getType().equals(ImageMessage.class))
      {  // Topic is an ImageMessage topic -> publish as image message
         publishAsImageMessage((ROS2Topic<ImageMessage>) imageTopic, imageToPublish, cameraModel);
      }
      else if (imageTopic.getType().equals(SRTStreamStatus.class))
      {  // Topic is an SRT stream topic -> stream video over SRT
         sensorStreamer.sendFrame((ROS2Topic<SRTStreamStatus>) imageTopic, imageToPublish);
      }
   }

   private void publishAsImageMessage(ROS2Topic<ImageMessage> imageTopic, RawImage imageToPublish, CameraModel cameraModel)
   {
      GpuMat imageToCompress = imageToPublish.getGpuImageMat();
      BytePointer compressedImage;
      CompressionType compressionType;

      switch (imageToPublish.getPixelFormat())
      {
         case GRAY16: // Depth image -> compress using ZSTD nvJPEG hybrid compression
            compressedImage = compressionTools.compressDepth(imageToCompress);
            compressionType = ZSTD_NVJPEG_HYBRID;
            break;
         case BGRA8: // BGRA image -> convert to BGR, then compress using nvJPEG
            GpuMat bgr8Image = new GpuMat();
            opencv_cudaimgproc.cvtColor(imageToCompress, bgr8Image, opencv_imgproc.COLOR_BGRA2BGR);
            imageToCompress = bgr8Image;
         case BGR8: // BGR image -> compress using nvJPEG
            compressedImage = new BytePointer(imageToCompress.limit());
            jpegProcessor.encodeBGR(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         case RGBA8: // RGBA image -> convert to RGB, then compress using nvJPEG
            GpuMat rgb8Image = new GpuMat();
            opencv_cudaimgproc.cvtColor(imageToCompress, rgb8Image, opencv_imgproc.COLOR_RGBA2RGB);
            imageToCompress = rgb8Image;
         case RGB8: // RGB image -> compress using nvJPEG
            compressedImage = new BytePointer(imageToCompress.limit());
            jpegProcessor.encodeRGB(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         case GRAY8: // Black and white image -> compress using nvJPEG
            compressedImage = new BytePointer(imageToCompress.limit());
            jpegProcessor.encodeGray(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         default:
            throw new NotImplementedException("Tomasz has not implemented the compression method for this pixel format yet.");
      }

      // Pack the message and send it off
      PerceptionMessageTools.packImageMessage(imageToPublish, compressedImage, compressionType, cameraModel, imageMessage);
      ros2Helper.publish(imageTopic, imageMessage);
   }

   @Override
   public void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      compressionTools.destroy();
      jpegProcessor.destroy();
      sensorStreamer.destroy();
      System.out.println("Closed " + getClass().getSimpleName());
   }
}
