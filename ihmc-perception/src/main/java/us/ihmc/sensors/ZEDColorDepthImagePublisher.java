package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
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

   private final SideDependentList<CUDAImageEncoder> imageEncoders = new SideDependentList<>();

   private long lastDepthSequenceNumber = -1L;
   private final SideDependentList<Long> lastColorSequenceNumbers = new SideDependentList<>(-1L, -1L);

   private RawImage nextGpuDepthImage;
   private final SideDependentList<RawImage> nextGpuColorImages = new SideDependentList<>();

   private final RestartableThread publishDepthThread;
   private final Lock depthPublishLock = new ReentrantLock();
   private final Condition newDepthImageAvailable = depthPublishLock.newCondition();

   private final SideDependentList<RestartableThread> publishColorThreads = new SideDependentList<>();
   private final SideDependentList<Lock> colorPublishLocks = new SideDependentList<>(new ReentrantLock(), new ReentrantLock());
   private final SideDependentList<Condition> newColorImagesAvailable = new SideDependentList<>(colorPublishLocks.get(RobotSide.LEFT).newCondition(),
                                                                                                colorPublishLocks.get(RobotSide.RIGHT).newCondition());

   public ZEDColorDepthImagePublisher(SideDependentList<ROS2Topic<ImageMessage>> colorTopics,
                                      ROS2Topic<ImageMessage> depthTopic)
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed_color_depth_publisher");
      ros2ColorImagePublishers = new SideDependentList<>(ROS2Tools.createPublisher(ros2Node, colorTopics.get(RobotSide.LEFT)),
                                                         ROS2Tools.createPublisher(ros2Node, colorTopics.get(RobotSide.RIGHT)));
      ros2DepthImagePublisher = ROS2Tools.createPublisher(ros2Node, depthTopic);

      publishDepthThread = new RestartableThread("ZEDDepthImagePublisher", this::publishDepthThreadFunction);
      publishDepthThread.start();

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

         publishDepthImage(nextGpuDepthImage);
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
         // Encode depth image to png
         BytePointer depthPNGPointer = new BytePointer();
         OpenCVTools.compressImagePNG(depthImageToPublish.getCpuImageMat(), depthPNGPointer);

         // Publish image
         ImageMessage depthImageMessage = new ImageMessage();
         ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(depthPNGPointer.limit());
         imageMessageDataPacker.pack(depthImageMessage, depthPNGPointer);
         MessageTools.toMessage(depthImageToPublish.getAcquisitionTime(), depthImageMessage.getAcquisitionTime());
         depthImageMessage.setFocalLengthXPixels(depthImageToPublish.getFocalLengthX());
         depthImageMessage.setFocalLengthYPixels(depthImageToPublish.getFocalLengthY());
         depthImageMessage.setPrincipalPointXPixels(depthImageToPublish.getPrincipalPointX());
         depthImageMessage.setPrincipalPointYPixels(depthImageToPublish.getPrincipalPointY());
         depthImageMessage.setImageWidth(depthImageToPublish.getImageWidth());
         depthImageMessage.setImageHeight(depthImageToPublish.getImageHeight());
         depthImageMessage.getPosition().set(depthImageToPublish.getPosition());
         depthImageMessage.getOrientation().set(depthImageToPublish.getOrientation());
         depthImageMessage.setSequenceNumber(depthImageToPublish.getSequenceNumber());
         depthImageMessage.setDepthDiscretization(depthImageToPublish.getDepthDiscretization());
         CameraModel.PINHOLE.packMessageFormat(depthImageMessage);
         ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(depthImageMessage);

         ros2DepthImagePublisher.publish(depthImageMessage);

         lastDepthSequenceNumber = depthImageToPublish.getSequenceNumber();

         // Close GpuMat
         depthPNGPointer.close();
         depthImageToPublish.release();
      }
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
            imageEncoders.put(side, new CUDAImageEncoder());

         // Compress image
         BytePointer colorJPEGPointer = new BytePointer((long) colorImageToPublish.getImageHeight() * colorImageToPublish.getImageWidth());
         imageEncoders.get(side)
                      .encodeBGR(colorImageToPublish.getGpuImageMat().data(),
                                 colorJPEGPointer,
                                 colorImageToPublish.getImageWidth(),
                                 colorImageToPublish.getImageHeight(),
                                 colorImageToPublish.getGpuImageMat().step());

         // Publish compressed image
         ImageMessage colorImageMessage = new ImageMessage();
         ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(colorJPEGPointer.limit());
         imageMessageDataPacker.pack(colorImageMessage, colorJPEGPointer);
         MessageTools.toMessage(colorImageToPublish.getAcquisitionTime(), colorImageMessage.getAcquisitionTime());
         colorImageMessage.setFocalLengthXPixels(colorImageToPublish.getFocalLengthX());
         colorImageMessage.setFocalLengthYPixels(colorImageToPublish.getFocalLengthY());
         colorImageMessage.setPrincipalPointXPixels(colorImageToPublish.getPrincipalPointX());
         colorImageMessage.setPrincipalPointYPixels(colorImageToPublish.getPrincipalPointY());
         colorImageMessage.setImageWidth(colorImageToPublish.getImageWidth());
         colorImageMessage.setImageHeight(colorImageToPublish.getImageHeight());
         colorImageMessage.getPosition().set(colorImageToPublish.getPosition());
         colorImageMessage.getOrientation().set(colorImageToPublish.getOrientation());
         colorImageMessage.setSequenceNumber(colorImageToPublish.getSequenceNumber());
         colorImageMessage.setDepthDiscretization(colorImageToPublish.getDepthDiscretization());
         CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
         ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(colorImageMessage);
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
         nextGpuDepthImage = depthImage;
         newDepthImageAvailable.signal();
      }
      finally
      {
         depthPublishLock.unlock();
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
