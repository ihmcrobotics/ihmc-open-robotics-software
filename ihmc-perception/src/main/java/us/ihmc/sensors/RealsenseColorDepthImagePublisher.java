package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class RealsenseColorDepthImagePublisher
{
   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<ImageMessage> ros2DepthImagePublisher;
   private final IHMCROS2Publisher<ImageMessage> ros2ColorImagePublisher;

   private final CUDAImageEncoder imageEncoder = new CUDAImageEncoder();

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
      ros2DepthImagePublisher = ROS2Tools.createPublisher(ros2Node, depthTopic);
      ros2ColorImagePublisher = ROS2Tools.createPublisher(ros2Node, colorTopic);

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
         // Compress image
         BytePointer colorJPEGPointer = new BytePointer((long) colorImageToPublish.getImageHeight() * colorImageToPublish.getImageWidth());
         imageEncoder.encodeBGR(colorImageToPublish.getGpuImageMat().data(),
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

      if (nextCpuDepthImage != null)
         nextCpuDepthImage.release();
      if (nextCpuColorImage != null)
         nextCpuColorImage.release();

      imageEncoder.destroy();

      ros2DepthImagePublisher.destroy();
      ros2ColorImagePublisher.destroy();
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
