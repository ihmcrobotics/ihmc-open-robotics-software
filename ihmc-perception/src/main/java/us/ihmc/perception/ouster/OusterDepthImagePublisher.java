package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OusterDepthImagePublisher
{
   private static final IntPointer COMPRESSION_PARAMETERS = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);

   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<ImageMessage> ros2DepthImagePublisher;

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
      ros2DepthImagePublisher = ROS2Tools.createPublisher(ros2Node, depthTopic);

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
         opencv_imgcodecs.imencode(".png", depthImageToPublish.getCpuImageMatrix(), depthPNGPointer, COMPRESSION_PARAMETERS);

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
         CameraModel.OUSTER.packMessageFormat(depthImageMessage);
         ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(depthImageMessage);
         MessageTools.packIDLSequence(ouster.getBeamAltitudeAnglesBuffer(), depthImageMessage.getOusterBeamAltitudeAngles());
         MessageTools.packIDLSequence(ouster.getBeamAzimuthAnglesBuffer(), depthImageMessage.getOusterBeamAzimuthAngles());

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

      ros2DepthImagePublisher.destroy();
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
