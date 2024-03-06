package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class BlackflyImagePublisher
{
   private final ROS2Node ros2Node;
   private final ROS2StoredPropertySet<IntrinsicCameraMatrixProperties> ousterFisheyeColoringIntrinsicsROS2;
   private final IHMCROS2Publisher<ImageMessage> ros2DistoredImagePublisher;

   private final CUDAImageEncoder imageEncoder = new CUDAImageEncoder();

   private long lastImageSequenceNumber = -1L;
   private RawImage nextGpuDistortedImage;

   private final RestartableThread publishDistoredColorThread;
   private final Lock imagePublishLock = new ReentrantLock();
   private final Condition newImageAvailable = imagePublishLock.newCondition();

   private float publishedImageScaleFactor = 1.0f;

   public BlackflyImagePublisher(BlackflyLensProperties lensProperties, ROS2Topic<ImageMessage> distortedImageTopic, float publishedImageScaleFactor)
   {
      IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics = SensorHeadParameters.loadOusterFisheyeColoringIntrinsicsOnRobot(lensProperties);

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "blackfly_publisher");
      ros2DistoredImagePublisher = ROS2Tools.createPublisher(ros2Node, distortedImageTopic, ROS2QosProfile.BEST_EFFORT());
      ousterFisheyeColoringIntrinsicsROS2 = new ROS2StoredPropertySet<>(new ROS2Helper(ros2Node),
                                                                        BlackflyComms.OUSTER_FISHEYE_COLORING_INTRINSICS,
                                                                        ousterFisheyeColoringIntrinsics);

      this.publishedImageScaleFactor = publishedImageScaleFactor;

      publishDistoredColorThread = new RestartableThread("BlackflyDistortedImagePublisher", this::distortedImagePublisherThreadFunction);
      publishDistoredColorThread.start();
   }

   private void distortedImagePublisherThreadFunction()
   {
      imagePublishLock.lock();
      try
      {
         while ((nextGpuDistortedImage == null || nextGpuDistortedImage.isEmpty() || nextGpuDistortedImage.getSequenceNumber() == lastImageSequenceNumber)
                && publishDistoredColorThread.isRunning())
         {
            newImageAvailable.await();
         }

         publishDistortedColor(nextGpuDistortedImage);
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
         imagePublishLock.unlock();
      }
   }

   private void publishDistortedColor(RawImage distortedImageToPublish)
   {
      RawImage imageToPublish = null;

      if (distortedImageToPublish != null)
         imageToPublish = new RawImage(distortedImageToPublish);
      imagePublishLock.unlock();

      if (imageToPublish != null && !imageToPublish.isEmpty() && imageToPublish.getSequenceNumber() != lastImageSequenceNumber)
      {
         // Scale image by publishedImageScaleFactor to reduce size over the network
         GpuMat scaledImageMat = new GpuMat();

         int scaledWidth = Math.round(imageToPublish.getImageWidth() * publishedImageScaleFactor);
         int scaledHeight = Math.round(imageToPublish.getImageHeight() * publishedImageScaleFactor);

         opencv_cudawarping.resize(imageToPublish.getGpuImageMat(), scaledImageMat, new Size(scaledWidth, scaledHeight));

         // Compress image
         BytePointer distortedImageJPEGPointer = new BytePointer((long) scaledImageMat.rows() * scaledImageMat.cols());
         imageEncoder.encodeBGR(scaledImageMat.data(),
                                distortedImageJPEGPointer,
                                scaledWidth,
                                scaledHeight,
                                scaledImageMat.step());
         
         // Publish intrinsics
         ousterFisheyeColoringIntrinsicsROS2.updateAndPublishThrottledStatus();
         
         // Publish compressed image
         ImageMessage distortedImageMessage = new ImageMessage();
         ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(distortedImageJPEGPointer.limit());
         imageMessageDataPacker.pack(distortedImageMessage, distortedImageJPEGPointer);
         MessageTools.toMessage(imageToPublish.getAcquisitionTime(), distortedImageMessage.getAcquisitionTime());
         distortedImageMessage.setFocalLengthXPixels(imageToPublish.getFocalLengthX() * publishedImageScaleFactor);
         distortedImageMessage.setFocalLengthYPixels(imageToPublish.getFocalLengthY() * publishedImageScaleFactor);
         distortedImageMessage.setPrincipalPointXPixels(imageToPublish.getPrincipalPointX() * publishedImageScaleFactor);
         distortedImageMessage.setPrincipalPointYPixels(imageToPublish.getPrincipalPointY() * publishedImageScaleFactor);
         distortedImageMessage.setImageWidth(scaledWidth);
         distortedImageMessage.setImageHeight(scaledHeight);
         distortedImageMessage.getPosition().set(imageToPublish.getPosition());
         distortedImageMessage.getOrientation().set(imageToPublish.getOrientation());
         distortedImageMessage.setSequenceNumber(imageToPublish.getSequenceNumber());
         CameraModel.EQUIDISTANT_FISHEYE.packMessageFormat(distortedImageMessage);
         ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(distortedImageMessage);
         ros2DistoredImagePublisher.publish(distortedImageMessage);

         lastImageSequenceNumber = imageToPublish.getSequenceNumber();

         // Close stuff
         distortedImageJPEGPointer.close();
         scaledImageMat.close();
         imageToPublish.release();
      }
   }

   public void startImagePublishing()
   {
      publishDistoredColorThread.start();
   }

   public void startAll()
   {
      startImagePublishing();
   }

   public void stopImagePublishing()
   {
      publishDistoredColorThread.stop();
      imagePublishLock.lock();
      try
      {
         newImageAvailable.signal();
      }
      finally
      {
         imagePublishLock.unlock();
      }
   }

   public void stopAll()
   {
      stopImagePublishing();
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      publishDistoredColorThread.stop();
      imagePublishLock.lock();
      try
      {
         newImageAvailable.signal();
      }
      finally
      {
         imagePublishLock.unlock();
      }

      if (nextGpuDistortedImage != null)
         nextGpuDistortedImage.release();

      imageEncoder.destroy();
      ros2DistoredImagePublisher.destroy();
      ros2Node.destroy();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   public void setNextDistortedImage(RawImage distortedImage)
   {
      imagePublishLock.lock();
      try
      {
         if (nextGpuDistortedImage != null)
            nextGpuDistortedImage.release();
         nextGpuDistortedImage = distortedImage;
         newImageAvailable.signal();
      }
      finally
      {
         imagePublishLock.unlock();
      }
   }
}
