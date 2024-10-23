package us.ihmc.perception;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.streaming.ROS2SRTSensorStreamer;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;

import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.perception.imageMessage.CompressionType.NVJPEG;
import static us.ihmc.perception.imageMessage.CompressionType.ZSTD_NVJPEG_HYBRID;

public class ImageSensorPublisher implements AutoCloseable
{
   private final Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap;

   private final ImageSensor imageSensor;

   private final CUDACompressionTools compressionTools;
   private final CUDAJPEGProcessor jpegProcessor;
   private final ROS2SRTSensorStreamer sensorStreamer;

   private final ROS2Helper ros2Helper;
   private final ImageMessage imageMessage;

   private final Thread publishThread;
   private volatile boolean running = true;
   private final AtomicBoolean isSending = new AtomicBoolean(false);
   
   public ImageSensorPublisher(ROS2Node ros2Node, ImageSensor imageSensor, Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap)
   {
      this.imageSensor = imageSensor;
      this.imageKeyToTopicMap = imageKeyToTopicMap;

      compressionTools = new CUDACompressionTools();
      jpegProcessor = new CUDAJPEGProcessor();
      sensorStreamer = new ROS2SRTSensorStreamer(ros2Node);

      ros2Helper = new ROS2Helper(ros2Node);
      imageMessage = new ImageMessage();

      publishThread = ThreadTools.startAsDaemon(this::run, imageSensor.getSensorName() + "Publisher");
   }

   @SuppressWarnings("unchecked") // Trust me bro, I know what I'm doing
   public void run()
   {
      while (running)
      {
         try
         {
            imageSensor.waitForGrab();

            if (!isSending.compareAndSet(false, true))
               continue;

            for (Entry<Integer, ROS2Topic<? extends Packet<?>>> imageEntry : imageKeyToTopicMap.entrySet())
            {
               int imageKey = imageEntry.getKey();
               ROS2Topic<? extends Packet<?>> imageTopic = imageEntry.getValue();

               RawImage imageToPublish = imageSensor.getImage(imageKey);
               if (imageTopic.getType().equals(ImageMessage.class))
                  publishAsImageMessage(imageToPublish, (ROS2Topic<ImageMessage>) imageTopic);
               else if (imageTopic.getType().equals(SRTStreamStatus.class))
                  sensorStreamer.sendFrame((ROS2Topic<SRTStreamStatus>) imageTopic, imageToPublish);
               imageToPublish.release();
            }
            isSending.set(false);
         }
         catch (InterruptedException ignored) {}
      }
   }

   private void publishAsImageMessage(RawImage imageToPublish, ROS2Topic<ImageMessage> imageTopic)
   {
      GpuMat imageToCompress = imageToPublish.getGpuImageMat();
      BytePointer compressedImage;
      CompressionType compressionType;

      switch (imageToPublish.getPixelFormat())
      {
         case GRAY16:
            compressedImage = compressionTools.compressDepth(imageToCompress);
            compressionType = ZSTD_NVJPEG_HYBRID;
            break;
         case BGRA8:
            GpuMat bgr8Image = new GpuMat();
            opencv_cudaimgproc.cvtColor(imageToCompress, bgr8Image, opencv_imgproc.COLOR_BGRA2BGR);
            imageToCompress = bgr8Image;
         case BGR8:
            compressedImage = new BytePointer(imageToCompress.limit());
            jpegProcessor.encodeBGR(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         case RGBA8:
            GpuMat rgb8Image = new GpuMat();
            opencv_cudaimgproc.cvtColor(imageToCompress, rgb8Image, opencv_imgproc.COLOR_RGBA2RGB);
            imageToCompress = rgb8Image;
         case RGB8:
            compressedImage = new BytePointer(imageToCompress.limit());
            jpegProcessor.encodeRGB(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         case GRAY8:
            compressedImage = new BytePointer(imageToCompress.limit());
            jpegProcessor.encodeGray(imageToCompress, compressedImage);
            compressionType = NVJPEG;
            break;
         default:
            throw new NotImplementedException("Tomasz has not implemented the compression method for this pixel format yet.");
      }

      PerceptionMessageTools.packImageMessage(imageToPublish, compressedImage, compressionType, imageSensor.getCameraModel(), imageMessage);

      ros2Helper.publish(imageTopic, imageMessage);
   }

   @Override
   public void close() throws InterruptedException
   {
      System.out.println("Closing " + getClass().getSimpleName());
      running = false;
      publishThread.interrupt();
      publishThread.join();

      compressionTools.destroy();
      jpegProcessor.destroy();
      sensorStreamer.destroy();
      System.out.println("Closed " + getClass().getSimpleName());
   }
}
