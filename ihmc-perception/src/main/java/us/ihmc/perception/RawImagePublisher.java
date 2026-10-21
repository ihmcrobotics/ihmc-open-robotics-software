package us.ihmc.perception;

import static us.ihmc.perception.imageMessage.CompressionType.*;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import perception_msgs.ImageMessage;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.sensors.CameraIntrinsics;

public class RawImagePublisher implements AutoCloseable
{
   private final ROS2PublisherMap publisherMap;
   private ImageMessage imageMessage;
   private final Image ros2Image;

   private CUDAJPEGProcessor nvJPEG;

   private double publishScale = 1.0;
   private boolean destroyed = false;
   private boolean imageTimingEnabled = false;
   private final java.util.Map<String, ImageTiming> imageTimings = new java.util.HashMap<>();

   /** Log per-topic timing summaries every five seconds for standard ROS images only. */
   public synchronized void setImageTimingEnabled(boolean enabled)
   {
      imageTimingEnabled = enabled;
      imageTimings.clear();
   }

   private static class ImageTiming
   {
      private long windowStart = System.nanoTime();
      private long count;
      private double ageSum, downloadSum, packingSum, publishingSum, publishingMax;

      private void record(String topic, long bytes, double age, long download, long packing, long publishing)
      {
         count++;
         ageSum += age;
         downloadSum += download / 1.0e6;
         packingSum += packing / 1.0e6;
         publishingSum += publishing / 1.0e6;
         publishingMax = Math.max(publishingMax, publishing / 1.0e6);
         long now = System.nanoTime();
         if (now - windowStart < 5_000_000_000L)
            return;
         LogTools.info(String.format(java.util.Locale.ROOT,
                                     "Hydra image timing %s: n=%d rate=%.2f Hz bytes=%d entryAgeAvg=%.1f ms CPUAccessAvg=%.1f ms packAvg=%.1f ms publishAvg=%.1f ms publishMax=%.1f ms",
                                     topic, count, count * 1.0e9 / (now - windowStart), bytes, ageSum / count,
                                     downloadSum / count, packingSum / count, publishingSum / count, publishingMax));
         count = 0;
         ageSum = downloadSum = packingSum = publishingSum = publishingMax = 0.0;
         windowStart = now;
      }
   }

   public RawImagePublisher(ROS2Node ros2Node)
   {
      publisherMap = new ROS2PublisherMap(ros2Node);
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

   public void publishImage(ROS2Topic<? extends ROS2Message<?>> imageTopic, RawImage imageToPublish)
   {
      publishImage(imageTopic, imageToPublish, null);
   }

   @SuppressWarnings("unchecked") // Trust me bro, I know what I'm doing
   public synchronized void publishImage(ROS2Topic<? extends ROS2Message<?>> imageTopic, RawImage imageToPublish, ReferenceFrame sensorFrame)
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
      ImageMessage messageToPublish = getOrCreateImageMessage();
      PerceptionMessageTools.packImageMessage(imageToCompress, compressedImage, compressionType, messageToPublish);
      publisherMap.publish(imageTopic, messageToPublish);

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

      long downloadStart = 0;
      long packingStart = 0;
      double entryAgeMilliseconds = 0.0;
      if (imageTimingEnabled)
      {
         entryAgeMilliseconds = java.time.Duration.between(imageToPublish.getAcquisitionTime(), java.time.Instant.now()).toNanos() / 1.0e6;
         downloadStart = System.nanoTime();
         // Includes any CPU-image lock wait and lazy GPU download. Packing then reuses this image.
         imageToPublish.getCpuImageMat();
         packingStart = System.nanoTime();
      }

      // Pack the Image message
      PerceptionMessageTools.packImageMessage(imageToPublish, sensorFrame.getName(), ros2Image);

      // Publish includes Java serialization and the DDS write call, not receiver delivery.
      long publishingStart = imageTimingEnabled ? System.nanoTime() : 0;
      publisherMap.publish(imageTopic, ros2Image);
      if (imageTimingEnabled)
      {
         long publishingEnd = System.nanoTime();
         imageTimings.computeIfAbsent(imageTopic.getName(), key -> new ImageTiming())
                     .record(imageTopic.getName(), ros2Image.getData().size(), entryAgeMilliseconds,
                             packingStart - downloadStart, publishingStart - packingStart, publishingEnd - publishingStart);
      }

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
      publisherMap.publish(cameraInfoTopic, cameraInfo);
   }

   private ImageMessage getOrCreateImageMessage()
   {
      if (imageMessage == null)
         imageMessage = new ImageMessage();
      return imageMessage;
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
