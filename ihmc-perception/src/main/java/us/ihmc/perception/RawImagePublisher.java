package us.ihmc.perception;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import perception_msgs.msg.dds.ImageMessage;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import static us.ihmc.perception.imageMessage.CompressionType.*;

public class RawImagePublisher implements AutoCloseable
{
   private final ROS2Helper ros2Helper;
   private final ImageMessage imageMessage;
   private final Image ros2Image;

   private CUDAJPEGProcessor nvJPEG;

   private double publishScale = 1.0;
   private boolean destroyed = false;

   public RawImagePublisher(ROS2Node ros2Node)
   {
      ros2Helper = new ROS2Helper(ros2Node);
      imageMessage = new ImageMessage();
      ros2Image = new Image();

      if (CUDATools.hasNVJPEG())
         nvJPEG = new CUDAJPEGProcessor();
      else
         LogTools.warn("nvJPEG not found. Using CPU JPEG compression. Publishing images will be slow and CPU intensive.");
   }

   public RawImagePublisher(ROS2Node ros2Node, double publishScale)
   {
      this(ros2Node);

      this.publishScale = publishScale;
   }

   public void setPublishScale(double publishScale)
   {
      this.publishScale = publishScale;
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
      else
         throw new IllegalArgumentException(
               getClass().getSimpleName() + " doesn't know how to publish this message type (" + imageTopic.getType().getSimpleName() + ")");
   }

   private void publishAsImageMessage(ROS2Topic<ImageMessage> imageTopic, RawImage imageToPublish)
   {
      RawImage imageToCompress = imageToPublish;
      RawImage scaledImage = null;
      RawImage colorConvertedImage = null;

      if (publishScale != 1.0)
      {
         scaledImage = RawImageTools.scale(imageToCompress, publishScale, opencv_imgproc.INTER_NEAREST);
         imageToCompress = scaledImage;
      }

      BytePointer compressedImage;
      CompressionType compressionType;

      switch (imageToCompress.getPixelFormat())
      {
         case GRAY16:
            compressedImage = new BytePointer();
            OpenCVTools.compressImagePNG(imageToCompress.getCpuImageMat(), compressedImage);
            compressionType = PNG;
            break;

         case BGRA8: // Convert to BGR8 first
            colorConvertedImage = RawImageTools.convertColor(imageToCompress, PixelFormat.BGR8);
            imageToCompress = colorConvertedImage;
         case BGR8:
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress.getGpuImageMat()));
            if (nvJPEG != null)
            {
               nvJPEG.encodeBGR(imageToCompress.getGpuImageMat(), compressedImage);
               compressionType = NVJPEG;
            }
            else
            {
               opencv_imgcodecs.imencode(".jpg", imageToCompress.getCpuImageMat(), compressedImage, OpenCVTools.compressionParametersJPG);
               compressionType = JPEG;
            }
            break;

         case RGBA8: // Convert to RGB8 first
            colorConvertedImage = RawImageTools.convertColor(imageToCompress, PixelFormat.RGB8);
            imageToCompress = colorConvertedImage;
         case RGB8:
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress.getGpuImageMat()));
            if (nvJPEG != null)
            {
               nvJPEG.encodeRGB(imageToCompress.getGpuImageMat(), compressedImage);
               compressionType = NVJPEG;
            }
            else
            {
               opencv_imgcodecs.imencode(".jpg", imageToCompress.getCpuImageMat(), compressedImage, OpenCVTools.compressionParametersJPG);
               compressionType = JPEG;
            }
            break;

         case GRAY8:
            compressedImage = new BytePointer(OpenCVTools.dataSize(imageToCompress.getGpuImageMat()));
            if (nvJPEG != null)
            {
               nvJPEG.encodeGray(imageToCompress.getGpuImageMat(), compressedImage);
               compressionType = NVJPEG;
            }
            else
            {
               opencv_imgcodecs.imencode(".jpg", imageToCompress.getCpuImageMat(), compressedImage, OpenCVTools.compressionParametersJPG);
               compressionType = JPEG;
            }
            break;
         default:
            throw new NotImplementedException("Tomasz has not implemented the compression method for this pixel format yet.");
      }

      // Pack the message and send it off
      PerceptionMessageTools.packImageMessage(imageToCompress, compressedImage, compressionType, imageMessage);
      ros2Helper.publish(imageTopic, imageMessage);

      // Close stuff
      compressedImage.close();
      if (scaledImage != null)
         scaledImage.release();
      if (colorConvertedImage != null)
         colorConvertedImage.release();
   }

   private void publishAsROS2Image(ROS2Topic<Image> imageTopic, RawImage imageToPublish, ReferenceFrame sensorFrame)
   {
      if (sensorFrame == null)
         throw new IllegalArgumentException("A sensor frame must be provided to publish ROS 2 Image messages");

      RawImage scaledImage = null;

      // Scale the image if needed
      if (publishScale != 1.0)
      {
         scaledImage = RawImageTools.scale(imageToPublish, publishScale);
         imageToPublish = scaledImage;
      }

      // Pack the Image message
      PerceptionMessageTools.packImageMessage(imageToPublish, sensorFrame.getName(), ros2Image);

      // Publish the image
      ros2Helper.publish(imageTopic, ros2Image);

      // Close stuff
      if (scaledImage != null)
         scaledImage.release();
   }

   private void publishCameraInfo(ROS2Topic<CameraInfo> cameraInfoTopic, RawImage image, ReferenceFrame sensorFrame)
   {
      if (sensorFrame == null)
         throw new IllegalArgumentException("A sensor frame must be provided to publish ROS 2 CameraInfo messages");

      // Get the correct intrinsics
      CameraIntrinsics intrinsics = image.getIntrinsicsCopy();
      if (publishScale != 1.0)
         intrinsics = RawImageTools.scale(intrinsics, publishScale);

      // Create and pack a CameraInfo message
      CameraInfo cameraInfo = new CameraInfo();
      PerceptionMessageTools.packCameraInfo(image.getAcquisitionTime(), intrinsics, image.getTransformToWorld(), sensorFrame.getName(), cameraInfo);

      // Publish the message
      ros2Helper.publish(cameraInfoTopic, cameraInfo);
   }

   @Override
   public synchronized void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      if (nvJPEG != null)
         nvJPEG.destroy();
      destroyed = true;
      System.out.println("Closed " + getClass().getSimpleName());
   }
}
