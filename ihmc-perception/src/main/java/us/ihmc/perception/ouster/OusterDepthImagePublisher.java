package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OusterDepthImagePublisher
{
   private static final IntPointer COMPRESSION_PARAMETERS = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);

   private final ROS2Node ros2Node;
   private final ROS2PublisherBasics<ImageMessage> ros2DepthImagePublisher;

   private long lastSequenceNumber = -1L;
   private RawImage nextCpuDepthImage;
   private final OusterNetServer ouster;

   private final RestartableThread publishDepthThread;
   private final Lock depthPublishLock = new ReentrantLock();
   private final Condition newDepthImageAvailable = depthPublishLock.newCondition();

   public OusterDepthImagePublisher(OusterNetServer ouster, ROS2Topic<ImageMessage> depthTopic)
   {
      this.ouster = ouster;

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_depth_publisher");
      ros2DepthImagePublisher = ros2Node.createPublisher(depthTopic);

      publishDepthThread = new RestartableThread("OusterDepthImagePublisher", this::publishDepthThreadFunction);
      publishDepthThread.start();
   }

   private void publishDepthThreadFunction()
   {
      depthPublishLock.lock();
      try
      {
         while ((nextCpuDepthImage == null || nextCpuDepthImage.isEmpty() || nextCpuDepthImage.getSequenceNumber() == lastSequenceNumber)
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
      if (depthImageToPublish != null && !depthImageToPublish.isEmpty() && depthImageToPublish.getSequenceNumber() != lastSequenceNumber)
      {
         // Encode depth image to png
         BytePointer depthPNGPointer = new BytePointer();
         opencv_imgcodecs.imencode(".png", depthImageToPublish.getCpuImageMat(), depthPNGPointer, COMPRESSION_PARAMETERS);

         // Publish image
         ImageMessage depthImageMessage = new ImageMessage();
         PerceptionMessageTools.packImageMessage(depthImageToPublish,
                                                 depthPNGPointer,
                                                 CompressionType.PNG,
                                                 CameraModel.OUSTER,
                                                 ouster.getBeamAltitudeAnglesBuffer(),
                                                 ouster.getBeamAzimuthAnglesBuffer(),
                                                 depthImageMessage);

         ros2DepthImagePublisher.publish(depthImageMessage);
         lastSequenceNumber = depthImageToPublish.getSequenceNumber();

         // Close stuff
         depthPNGPointer.close();
         depthImageToPublish.release();
      }
   }

   public void startDepth()
   {
      publishDepthThread.start();
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

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      stopDepth();

      if (nextCpuDepthImage != null)
         nextCpuDepthImage.release();

      ros2DepthImagePublisher.remove();
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
}
