package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.ImageMessageDataPacker;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class RealsenseColorDepthImagePublisher
{
   private final ROS2Node ros2Node;
   private final ROS2PublisherBasics<ImageMessage> ros2DepthImagePublisher;
   private final ROS2PublisherBasics<ImageMessage> ros2ColorImagePublisher;

   private CUDAJPEGProcessor imageEncoder;
   private CUDACompressionTools compressionTools;

   private long lastDepthSequenceNumber = -1L;
   private long lastColorSequenceNumber = -1L;
   private RawImage nextCpuDepthImage;
   private RawImage nextCpuColorImage;

   private final RestartableThread publishDepthThread;
   private final Lock depthPublishLock = new ReentrantLock();
   private final Condition newDepthImageAvailable = depthPublishLock.newCondition();

   private final RestartableThread publishColorThread;
   private final Lock colorPublishLock = new ReentrantLock();
   private final Condition newColorImageAvailable = colorPublishLock.newCondition();

   public RealsenseColorDepthImagePublisher(ROS2Topic<ImageMessage> depthTopic,
                                            ROS2Topic<ImageMessage> colorTopic)
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_color_depth_publisher");
      ros2DepthImagePublisher = ros2Node.createPublisher(depthTopic);
      ros2ColorImagePublisher = ros2Node.createPublisher(colorTopic);

      publishDepthThread = new RestartableThread("RealsenseDepthImagePublisher", this::publishDepthThreadFunction);
      publishColorThread = new RestartableThread("RealsenseColorImagePublisher", this::publishColorThreadFunction);

      publishDepthThread.start();
      publishColorThread.start();
   }

   private void publishDepthThreadFunction()
   {
      depthPublishLock.lock();
      try
      {
         while ((nextCpuDepthImage == null || nextCpuDepthImage.isEmpty() || nextCpuDepthImage.getSequenceNumber() == lastDepthSequenceNumber)
                && publishDepthThread.isRunning())
         {
            newDepthImageAvailable.await();
         }

         publishDepthImage(nextCpuDepthImage);
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
      finally
      {
         depthPublishLock.unlock();
      }
   }

   private void publishDepthImage(RawImage depthImageToPublish)
   {
      // Redundant safety checks
      if (depthImageToPublish != null && !depthImageToPublish.isEmpty() && depthImageToPublish.getSequenceNumber() != lastDepthSequenceNumber)
      {
         if (compressionTools == null)
            compressionTools = new CUDACompressionTools();

         // Encode depth image to png
         BytePointer depthPNGPointer = compressionTools.compressDepth(depthImageToPublish.getGpuImageMat());

         // Publish image
         ImageMessage depthImageMessage = new ImageMessage();
         PerceptionMessageTools.packImageMessage(depthImageToPublish,
                                                 depthPNGPointer,
                                                 CompressionType.ZSTD_NVJPEG_HYBRID,
                                                 CameraModel.PINHOLE,
                                                 depthImageMessage);

         ros2DepthImagePublisher.publish(depthImageMessage);
         lastDepthSequenceNumber = depthImageToPublish.getSequenceNumber();

         // Close stuff
         depthPNGPointer.close();
         depthImageToPublish.release();
      }
   }

   private void publishColorThreadFunction()
   {
      colorPublishLock.lock();
      try
      {
         while ((nextCpuColorImage == null || nextCpuColorImage.isEmpty() || nextCpuColorImage.getSequenceNumber() == lastColorSequenceNumber)
                && publishColorThread.isRunning())
         {
            newColorImageAvailable.await();
         }

         publishColorImage(nextCpuColorImage);
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
      finally
      {
         colorPublishLock.unlock();
      }
   }

   private void publishColorImage(RawImage colorImageToPublish)
   {
      // Redundant safety checks
      if (colorImageToPublish != null && !colorImageToPublish.isEmpty() && colorImageToPublish.getSequenceNumber() != lastColorSequenceNumber)
      {
         if (imageEncoder == null)
            imageEncoder = new CUDAJPEGProcessor();

         // Compress image
         BytePointer colorJPEGPointer = new BytePointer((long) colorImageToPublish.getHeight() * colorImageToPublish.getWidth());
         imageEncoder.encodeBGR(colorImageToPublish.getGpuImageMat(), colorJPEGPointer);

         // Publish compressed image
         ImageMessage colorImageMessage = new ImageMessage();
         PerceptionMessageTools.packImageMessage(colorImageToPublish, colorJPEGPointer, CompressionType.NVJPEG, CameraModel.PINHOLE, colorImageMessage);

         ros2ColorImagePublisher.publish(colorImageMessage);
         lastColorSequenceNumber = colorImageToPublish.getSequenceNumber();

         // Close stuff
         colorJPEGPointer.close();
         colorImageToPublish.release();
      }
   }

   public void startDepth()
   {
      publishDepthThread.start();
   }

   public void startColor()
   {
      publishColorThread.start();
   }

   public void startAll()
   {
      startDepth();
      startColor();
   }

   public void stopDepth()
   {
      publishDepthThread.stop();
      depthPublishLock.lock();
      try
      {
         newDepthImageAvailable.signal();
      }
      finally
      {
         depthPublishLock.unlock();
      }
   }

   public void stopColor()
   {
      publishColorThread.stop();
      colorPublishLock.lock();
      try
      {
         newColorImageAvailable.signal();
      }
      finally
      {
         colorPublishLock.unlock();
      }
   }

   public void stopAll()
   {
      stopDepth();
      stopColor();
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      stopAll();
      depthPublishLock.lock();
      try
      {
         newDepthImageAvailable.signal();
      }
      finally
      {
         depthPublishLock.unlock();
      }
      colorPublishLock.lock();
      try
      {
         newColorImageAvailable.signal();
      }
      finally
      {
         colorPublishLock.unlock();
      }
      publishColorThread.blockingStop();

      if (nextCpuDepthImage != null)
         nextCpuDepthImage.release();
      if (nextCpuColorImage != null)
         nextCpuColorImage.release();

      if (imageEncoder != null)
         imageEncoder.destroy();
      if (compressionTools != null)
         compressionTools.destroy();

      ros2DepthImagePublisher.remove();
      ros2ColorImagePublisher.remove();
      ros2Node.destroy();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   public void setNextDepthImage(RawImage depthImage)
   {
      depthPublishLock.lock();
      try
      {
         nextCpuDepthImage = depthImage;
         newDepthImageAvailable.signal();
      }
      finally
      {
         depthPublishLock.unlock();
      }
   }

   public void setNextColorImage(RawImage colorImage)
   {
      colorPublishLock.lock();
      try
      {
         nextCpuColorImage = colorImage;
         newColorImageAvailable.signal();
      }
      finally
      {
         colorPublishLock.unlock();
      }
   }
}
