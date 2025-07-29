package us.ihmc.perception;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.SRTStreamStatus;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.streaming.ROS2SRTSensorStreamer;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import static us.ihmc.perception.imageMessage.CompressionType.*;

public class RawImagePublisher implements AutoCloseable
{
   private CUDACompressionTools compressionTools;
   private final CUDAJPEGProcessor jpegProcessor;
   private final ROS2SRTSensorStreamer sensorStreamer;

   private final ROS2Helper ros2Helper;
   private final ImageMessage imageMessage;
   private final Image ros2Image;

   private boolean destroyed = false;

   public RawImagePublisher(ROS2Node ros2Node)
   {
      try
      {
         compressionTools = new CUDACompressionTools();
      }
      catch (Exception e)
      {
         compressionTools = null;
      }

      jpegProcessor = new CUDAJPEGProcessor();
      sensorStreamer = new ROS2SRTSensorStreamer(ros2Node);

      ros2Helper = new ROS2Helper(ros2Node);
      imageMessage = new ImageMessage();
      ros2Image = new Image();
   }

   public void publishImage(ROS2Topic<? extends Packet<?>> imageTopic, RawImage imageToPublish)
   {
      publishImage(imageTopic, imageToPublish, null);
   }

   @SuppressWarnings("unchecked") // Trust me bro, I know what I'm doing
   public synchronized void publishImage(ROS2Topic<? extends Packet<?>> imageTopic, RawImage imageToPublish, ReferenceFrame sensorFrame)
   {
      if (destroyed)
         return;

      if (imageTopic.getType().equals(ImageMessage.class))
      {  // Topic is an ImageMessage topic -> publish as image message
         publishAsImageMessage((ROS2Topic<ImageMessage>) imageTopic, imageToPublish);
      }
      else if (imageTopic.getType().equals(Image.class))
      {  // Topic is a standard ROS2 Image topic -> publish as standard ROS2 Image message
         publishAsROS2Image((ROS2Topic<Image>) imageTopic, imageToPublish, sensorFrame);
      }
      else if (imageTopic.getType().equals(CameraInfo.class))
      {  // Topic is a camera info topic -> publish the image's camera info
         publishCameraInfo((ROS2Topic<CameraInfo>) imageTopic, imageToPublish, sensorFrame);
      }
      else if (imageTopic.getType().equals(SRTStreamStatus.class))
      {  // Topic is an SRT stream topic -> stream video over SRT
         sensorStreamer.sendFrame((ROS2Topic<SRTStreamStatus>) imageTopic, imageToPublish);
      }
      else
         throw new IllegalArgumentException(
               getClass().getSimpleName() + " doesn't know how to publish this message type (" + imageTopic.getType().getSimpleName() + ")");
   }

   private void publishAsImageMessage(ROS2Topic<ImageMessage> imageTopic, RawImage imageToPublish)
   {
      GpuMat imageToCompress = imageToPublish.getGpuImageMat();
      BytePointer compressedImage;
      CompressionType compressionType;

      switch (imageToPublish.getPixelFormat())
      {
         case GRAY16: // Depth image -> compress using ZSTD nvJPEG hybrid compression (or default to PNG if nvCOMP isn't available)
            if (compressionTools != null)
            {
               compressedImage = compressionTools.compressDepth(imageToCompress);
               compressionType = ZSTD_NVJPEG_HYBRID;
            }
            else
            {
               compressedImage = new BytePointer();
               OpenCVTools.compressImagePNG(imageToPublish.getCpuImageMat(), compressedImage);
               compressionType = PNG;
            }
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

   private void publishAsROS2Image(ROS2Topic<Image> imageTopic, RawImage imageToPublish, ReferenceFrame sensorFrame)
   {
      if (sensorFrame == null)
         throw new IllegalArgumentException("A sensor frame must be provided to publish ROS 2 Image messages");

      // Pack the Image message
      PerceptionMessageTools.packImageMessage(imageToPublish, sensorFrame.getName(), ros2Image);

      // Publish the image
      ros2Helper.publish(imageTopic, ros2Image);
   }

   private void publishCameraInfo(ROS2Topic<CameraInfo> cameraInfoTopic, RawImage image, ReferenceFrame sensorFrame)
   {
      if (sensorFrame == null)
         throw new IllegalArgumentException("A sensor frame must be provided to publish ROS 2 CameraInfo messages");

      // Create and pack a CameraInfo message
      CameraInfo cameraInfo = new CameraInfo();
      PerceptionMessageTools.packCameraInfo(image, sensorFrame.getName(), cameraInfo);

      // Publish the message
      ros2Helper.publish(cameraInfoTopic, cameraInfo);
   }

   @Override
   public synchronized void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      if (compressionTools != null)
         compressionTools.destroy();
      jpegProcessor.destroy();
      sensorStreamer.destroy();
      destroyed = true;
      System.out.println("Closed " + getClass().getSimpleName());
   }
}
