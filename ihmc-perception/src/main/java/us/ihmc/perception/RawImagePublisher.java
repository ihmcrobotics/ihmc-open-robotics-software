package us.ihmc.perception;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.SRTStreamStatus;
import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.streaming.ROS2SRTSensorStreamer;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.time.Instant;
import java.util.Arrays;

import static us.ihmc.perception.imageMessage.CompressionType.NVJPEG;
import static us.ihmc.perception.imageMessage.CompressionType.ZSTD_NVJPEG_HYBRID;

public class RawImagePublisher implements AutoCloseable
{
   private final CUDACompressionTools compressionTools;
   private final CUDAJPEGProcessor jpegProcessor;
   private final ROS2SRTSensorStreamer sensorStreamer;

   private final ROS2Helper ros2Helper;
   private final ImageMessage imageMessage;
   private final Image ros2Image;

   private boolean destroyed = false;

   public RawImagePublisher(ROS2Node ros2Node)
   {
      compressionTools = new CUDACompressionTools();
      jpegProcessor = new CUDAJPEGProcessor();
      sensorStreamer = new ROS2SRTSensorStreamer(ros2Node);

      ros2Helper = new ROS2Helper(ros2Node);
      imageMessage = new ImageMessage();
      ros2Image = new Image();
   }

   @SuppressWarnings("unchecked") // Trust me bro, I know what I'm doing
   public synchronized void publishImage(ROS2Topic<? extends Packet<?>> imageTopic, RawImage imageToPublish)
   {
      if (destroyed)
         return;

      if (imageTopic.getType().equals(ImageMessage.class))
      {  // Topic is an ImageMessage topic -> publish as image message
         publishAsImageMessage((ROS2Topic<ImageMessage>) imageTopic, imageToPublish);
      }
      else if (imageTopic.getType().equals(Image.class))
      {  // Topic is a standard ROS2 Image topic -> publish as standard ROS2 Image message
         publishAsROS2Image((ROS2Topic<Image>) imageTopic, imageToPublish);
      }
      else if (imageTopic.getType().equals(SRTStreamStatus.class))
      {  // Topic is an SRT stream topic -> stream video over SRT
         sensorStreamer.sendFrame((ROS2Topic<SRTStreamStatus>) imageTopic, imageToPublish);
      }
   }

   private void publishAsImageMessage(ROS2Topic<ImageMessage> imageTopic, RawImage imageToPublish)
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
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress));
            jpegProcessor.encodeBGR(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         case RGBA8: // RGBA image -> convert to RGB, then compress using nvJPEG
            GpuMat rgb8Image = new GpuMat();
            opencv_cudaimgproc.cvtColor(imageToCompress, rgb8Image, opencv_imgproc.COLOR_RGBA2RGB);
            imageToCompress = rgb8Image;
         case RGB8: // RGB image -> compress using nvJPEG
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress));
            jpegProcessor.encodeRGB(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         case GRAY8: // Black and white image -> compress using nvJPEG
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress));
            jpegProcessor.encodeGray(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         default:
            throw new NotImplementedException("Tomasz has not implemented the compression method for this pixel format yet.");
      }

      // Pack the message and send it off
      PerceptionMessageTools.packImageMessage(imageToPublish, compressedImage, compressionType, imageMessage);
      ros2Helper.publish(imageTopic, imageMessage);
   }

   private void publishAsROS2Image(ROS2Topic<Image> imageTopic, RawImage imageToPublish)
   {
      // Set the header
      Instant imageAcquisitionTime = imageToPublish.getAcquisitionTime();
      ros2Image.getHeader().getStamp().setSec((int) imageAcquisitionTime.getEpochSecond());
      ros2Image.getHeader().getStamp().setNanosec(imageAcquisitionTime.getNano());
      ros2Image.getHeader().setFrameId(""); // TODO: Figure out how to do frame ids with RawImage

      // Set dimensions
      ros2Image.setWidth(imageToPublish.getWidth());
      ros2Image.setHeight(imageToPublish.getHeight());

      // Set encoding
      PixelFormat pixelFormat = imageToPublish.getPixelFormat();
      String encoding = switch (pixelFormat)
      {
         case BGR8, BGRA8, RGB8, RGBA8 -> pixelFormat.name().toLowerCase();
         case GRAY8, GRAY16 -> pixelFormat.name().toLowerCase().replace("gray", "mono");
         default -> getOpenCVTypeString(imageToPublish.getOpenCVType());
      };
      ros2Image.setEncoding(encoding);

      // Set byte order
      ros2Image.setIsBigendian((byte) 0);

      // Set step
      Mat cpuImage = imageToPublish.getCpuImageMat();
      ros2Image.setStep(cpuImage.step());

      // Set data
      Arrays.fill(ros2Image.getData(), (byte) 0);
      cpuImage.data().get(ros2Image.getData(), 0, (int) OpenCVTools.memorySize(cpuImage));

      // Publish the image
      ros2Helper.publish(imageTopic, ros2Image);
   }

   private String getOpenCVTypeString(int openCVType)
   {
      int depth = openCVType & opencv_core.CV_MAT_DEPTH_MASK;
      int channels = ((openCVType - depth) >> opencv_core.CV_CN_SHIFT) + 1;

      StringBuilder typeString = new StringBuilder(5);
      switch (depth)
      {
         case opencv_core.CV_8U -> typeString.append("8UC");
         case opencv_core.CV_8S -> typeString.append("8SC");
         case opencv_core.CV_16U -> typeString.append("16UC");
         case opencv_core.CV_16S -> typeString.append("16SC");
         case opencv_core.CV_32S -> typeString.append("32SC");
         case opencv_core.CV_32F -> typeString.append("32FC");
         case opencv_core.CV_64F -> typeString.append("64FC");
      }

      typeString.append(channels);

      return typeString.toString();
   }

   @Override
   public synchronized void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      compressionTools.destroy();
      jpegProcessor.destroy();
      sensorStreamer.destroy();
      destroyed = true;
      System.out.println("Closed " + getClass().getSimpleName());
   }
}
