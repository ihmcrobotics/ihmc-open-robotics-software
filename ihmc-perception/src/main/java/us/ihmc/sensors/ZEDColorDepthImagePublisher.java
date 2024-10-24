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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class ZEDColorDepthImagePublisher
{
   private final ROS2Node ros2Node;
   private final SideDependentList<ROS2PublisherBasics<ImageMessage>> ros2ColorImagePublishers;
   private final ROS2PublisherBasics<ImageMessage> ros2DepthImagePublisher;
   private final ROS2PublisherBasics<ImageMessage> ros2CutOutDepthImagePublisher;

   private final SideDependentList<CUDAJPEGProcessor> imageEncoders = new SideDependentList<>();
   private final CUDACompressionTools dataCompressor = new CUDACompressionTools();

   private long lastDepthSequenceNumber = -1L;
   private long lastCutOutDepthSequenceNumber = -1L;
   private final SideDependentList<Long> lastColorSequenceNumbers = new SideDependentList<>(-1L, -1L);

   private RawImage nextGpuDepthImage;
   private RawImage nextCutOutDepthImage;
   private final SideDependentList<RawImage> nextGpuColorImages = new SideDependentList<>();

   private final RestartableThread publishDepthThread;
   private final Lock depthPublishLock = new ReentrantLock();
   private final Condition newDepthImageAvailable = depthPublishLock.newCondition();

   private final RestartableThread publishCutOutDepthThread;
   private final Lock cutOutDepthLock = new ReentrantLock();
   private final Condition newCutOutDepthImageAvailable = cutOutDepthLock.newCondition();

   private final SideDependentList<RestartableThread> publishColorThreads = new SideDependentList<>();
   private final SideDependentList<Lock> colorPublishLocks = new SideDependentList<>(new ReentrantLock(), new ReentrantLock());
   private final SideDependentList<Condition> newColorImagesAvailable = new SideDependentList<>(colorPublishLocks.get(RobotSide.LEFT).newCondition(),
                                                                                                colorPublishLocks.get(RobotSide.RIGHT).newCondition());

   public ZEDColorDepthImagePublisher(SideDependentList<ROS2Topic<ImageMessage>> colorTopics,
                                      ROS2Topic<ImageMessage> depthTopic,
                                      ROS2Topic<ImageMessage> cutoutDepthTopic)
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed_color_depth_publisher");
      ros2ColorImagePublishers = new SideDependentList<>(ros2Node.createPublisher(colorTopics.get(RobotSide.LEFT)),
                                                         ros2Node.createPublisher(colorTopics.get(RobotSide.RIGHT)));
      ros2DepthImagePublisher = ros2Node.createPublisher(depthTopic);
      ros2CutOutDepthImagePublisher = ros2Node.createPublisher(cutoutDepthTopic);

      publishDepthThread = new RestartableThread("ZEDDepthImagePublisher", this::publishDepthThreadFunction);
      publishDepthThread.start();

      publishCutOutDepthThread = new RestartableThread("ZEDCutOutDepthImagePublisher", this::publishCutOutDepth);
      publishCutOutDepthThread.start();

      publishColorThreads.put(RobotSide.LEFT, new RestartableThread("ZEDLeftColorImagePublisher", this::publishLeftColorThreadFunction));
      publishColorThreads.put(RobotSide.RIGHT, new RestartableThread("ZEDRightColorImagePublisher", this::publishRightColorThreadFunction));
      publishColorThreads.forEach(RestartableThread::start);
   }

   private void publishDepthThreadFunction()
   {
      depthPublishLock.lock();
      try
      {
         while ((nextGpuDepthImage == null || nextGpuDepthImage.isEmpty() || nextGpuDepthImage.getSequenceNumber() == lastDepthSequenceNumber)
                && publishDepthThread.isRunning())
         {
            newDepthImageAvailable.await();
         }

         if (nextGpuDepthImage != null)
         {
            ros2DepthImagePublisher.publish(createDepthImageMessage(nextGpuDepthImage));
            lastDepthSequenceNumber = nextGpuDepthImage.getSequenceNumber();
         }
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

   private void publishCutOutDepth()
   {
      cutOutDepthLock.lock();
      try
      {
         while ((nextCutOutDepthImage == null || nextCutOutDepthImage.isEmpty() || nextCutOutDepthImage.getSequenceNumber() == lastCutOutDepthSequenceNumber)
                && publishCutOutDepthThread.isRunning())
         {
            newCutOutDepthImageAvailable.await();
         }

         if (nextCutOutDepthImage != null)
         {
            ros2CutOutDepthImagePublisher.publish(createDepthImageMessage(nextCutOutDepthImage));
            lastCutOutDepthSequenceNumber = nextCutOutDepthImage.getSequenceNumber();
         }
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
      finally
      {
         cutOutDepthLock.unlock();
      }
   }

   private ImageMessage createDepthImageMessage(RawImage depthImageToPublish)
   {
      ImageMessage depthImageMessage = new ImageMessage();

      if (depthImageToPublish != null && depthImageToPublish.isAvailable())
      {
         depthImageToPublish.get();

         // Encode depth image to png
         BytePointer compressedDepthPointer = dataCompressor.compressDepth(depthImageToPublish.getGpuImageMat());

         // Pack the image message
         PerceptionMessageTools.packImageMessage(depthImageToPublish, compressedDepthPointer, CompressionType.ZSTD_NVJPEG_HYBRID, depthImageMessage);

         // Close GpuMat
         compressedDepthPointer.close();

         depthImageToPublish.release();
      }

      return depthImageMessage;
   }

   private void publishLeftColorThreadFunction()
   {
      RobotSide side = RobotSide.LEFT;
      colorPublishLocks.get(side).lock();
      try
      {
         while ((nextGpuColorImages.get(side) == null || nextGpuColorImages.get(side).isEmpty()
                 || nextGpuColorImages.get(side).getSequenceNumber() == lastColorSequenceNumbers.get(side)) && publishColorThreads.get(side).isRunning())
         {
            newColorImagesAvailable.get(side).await();
         }

         publishColorImage(nextGpuColorImages.get(side), side);
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
      finally
      {
         colorPublishLocks.get(side).unlock();
      }
   }

   private void publishRightColorThreadFunction()
   {
      RobotSide side = RobotSide.RIGHT;
      colorPublishLocks.get(side).lock();
      try
      {
         while ((nextGpuColorImages.get(side) == null || nextGpuColorImages.get(side).isEmpty()
                 || nextGpuColorImages.get(side).getSequenceNumber() == lastColorSequenceNumbers.get(side)) && publishColorThreads.get(side).isRunning())
         {
            newColorImagesAvailable.get(side).await();
         }

         publishColorImage(nextGpuColorImages.get(side), side);
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
      finally
      {
         colorPublishLocks.get(side).unlock();
      }
   }

   private void publishColorImage(RawImage colorImageToPublish, RobotSide side)
   {
      // Perform safety checks
      if (colorImageToPublish != null && !colorImageToPublish.isEmpty() && colorImageToPublish.getSequenceNumber() != lastColorSequenceNumbers.get(side))
      {
         if (imageEncoders.get(side) == null)
            imageEncoders.put(side, new CUDAJPEGProcessor());

         // Compress image
         BytePointer colorJPEGPointer = new BytePointer((long) colorImageToPublish.getHeight() * colorImageToPublish.getWidth());
         imageEncoders.get(side).encodeBGR(colorImageToPublish.getGpuImageMat(), colorJPEGPointer);

         // Publish compressed image
         ImageMessage colorImageMessage = new ImageMessage();
         PerceptionMessageTools.packImageMessage(colorImageToPublish, colorJPEGPointer, CompressionType.NVJPEG, colorImageMessage);
         ros2ColorImagePublishers.get(side).publish(colorImageMessage);

         lastColorSequenceNumbers.put(side, colorImageToPublish.getSequenceNumber());

         // Close stuff
         colorJPEGPointer.close();
         colorImageToPublish.release();
      }
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      publishDepthThread.stop();
      publishCutOutDepthThread.stop();
      publishColorThreads.forEach(RestartableThread::stop);
      depthPublishLock.lock();
      try
      {
         newDepthImageAvailable.signal();
      }
      finally
      {
         depthPublishLock.unlock();
      }

      if (nextGpuDepthImage != null)
         nextGpuDepthImage.release();

      cutOutDepthLock.lock();
      try
      {
         newCutOutDepthImageAvailable.signal();
      }
      finally
      {
         cutOutDepthLock.unlock();
      }

      if (nextCutOutDepthImage != null)
         nextCutOutDepthImage.release();

      for (RobotSide side : RobotSide.values)
      {
         colorPublishLocks.get(side).lock();
         try
         {
            newColorImagesAvailable.get(side).signal();
         }
         finally
         {
            colorPublishLocks.get(side).unlock();
         }
         publishColorThreads.get(side).blockingStop();
         if (imageEncoders.get(side) != null)
            imageEncoders.get(side).destroy();
         if (nextGpuColorImages.get(side) != null)
            nextGpuColorImages.get(side).release();
         ros2ColorImagePublishers.get(side).remove();
      }

      ros2DepthImagePublisher.remove();
      ros2Node.destroy();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   public void setNextGpuDepthImage(RawImage depthImage)
   {
      depthPublishLock.lock();
      try
      {
         if (nextGpuDepthImage != null)
            nextGpuDepthImage.release();
         nextGpuDepthImage = depthImage;
         newDepthImageAvailable.signal();
      }
      finally
      {
         depthPublishLock.unlock();
      }
   }

   public void setNextCutOutDepthImage(RawImage cutOutDepthImage)
   {
      cutOutDepthLock.lock();
      try
      {
         if (nextCutOutDepthImage != null)
            nextCutOutDepthImage.release();
         nextCutOutDepthImage = cutOutDepthImage;
         newCutOutDepthImageAvailable.signal();
      }
      finally
      {
         cutOutDepthLock.unlock();
      }
   }

   public void setNextColorImage(RawImage colorImage, RobotSide side)
   {
      colorPublishLocks.get(side).lock();
      try
      {
         nextGpuColorImages.put(side, colorImage);
         newColorImagesAvailable.get(side).signal();
      }
      finally
      {
         colorPublishLocks.get(side).unlock();
      }
   }
}
