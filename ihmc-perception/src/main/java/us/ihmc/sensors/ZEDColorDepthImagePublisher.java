package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

public class ZEDColorDepthImagePublisher
{
   private final ZEDModelData zedModelData;

   private final Supplier<ReferenceFrame> sensorFrameSupplier;
   private final FramePose3D leftCameraFramePose = new FramePose3D();
   private final SideDependentList<FramePose3D> cameraPosesInDepthFrame = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final SideDependentList<IHMCROS2Publisher<ImageMessage>> ros2ColorImagePublishers;
   private final IHMCROS2Publisher<ImageMessage> ros2DepthImagePublisher;

   private final SideDependentList<CUDAImageEncoder> imageEncoders = new SideDependentList<>(new CUDAImageEncoder(), new CUDAImageEncoder());

   private long lastDepthSequenceNumber = -1L;
   private final SideDependentList<Long> lastColorSequenceNumbers = new SideDependentList<>(-1L, -1L);

   private RawImage nextDepthImage;
   private SideDependentList<RawImage> nextColorImages = new SideDependentList<>();

   private final Thread publishDepthThread = new DepthPublisherThread();
   private final Lock depthPublishLock = new ReentrantLock();
   private final Condition newDepthImageAvailable = depthPublishLock.newCondition();

   private final SideDependentList<Thread> publishColorThreads = new SideDependentList<>(new ColorPublisherThread(RobotSide.LEFT),
                                                                                         new ColorPublisherThread(RobotSide.RIGHT));
   private final SideDependentList<Lock> colorPublishLocks = new SideDependentList<>(new ReentrantLock(), new ReentrantLock());
   private final SideDependentList<Condition> newColorImagesAvailable = new SideDependentList<>(colorPublishLocks.get(RobotSide.LEFT).newCondition(),
                                                                                                colorPublishLocks.get(RobotSide.RIGHT).newCondition());

   private boolean running = true;

   public ZEDColorDepthImagePublisher(ZEDModelData zedModelData,
                                      SideDependentList<ROS2Topic<ImageMessage>> colorTopics,
                                      ROS2Topic<ImageMessage> depthTopic,
                                      Supplier<ReferenceFrame> sensorFrameSupplier)
   {
      this.zedModelData = zedModelData;
      this.sensorFrameSupplier = sensorFrameSupplier;
      cameraPosesInDepthFrame.get(RobotSide.RIGHT).getPosition().subY(2.0 * zedModelData.getCenterToCameraDistance());

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_node");
      ros2ColorImagePublishers = new SideDependentList<>(ROS2Tools.createPublisher(ros2Node, colorTopics.get(RobotSide.LEFT), ROS2QosProfile.BEST_EFFORT()),
                                                         ROS2Tools.createPublisher(ros2Node, colorTopics.get(RobotSide.RIGHT), ROS2QosProfile.BEST_EFFORT()));
      ros2DepthImagePublisher = ROS2Tools.createPublisher(ros2Node, depthTopic, ROS2QosProfile.BEST_EFFORT());
   }

   private void publishDepthImage(RawImage gpuDepthImage16UC1)
   {
      // Perform safety checks
      if (gpuDepthImage16UC1 != null && !gpuDepthImage16UC1.isEmpty() && gpuDepthImage16UC1.getSequenceNumber() != lastDepthSequenceNumber)
      {
         // Encode depth image to png
         BytePointer depthPNGPointer = new BytePointer();
         OpenCVTools.compressImagePNG(gpuDepthImage16UC1.getCpuImageMatrix(), depthPNGPointer);

         // Get present position of sensor
         leftCameraFramePose.setToZero(sensorFrameSupplier.get());
         leftCameraFramePose.getPosition().addY(zedModelData.getCenterToCameraDistance());
         leftCameraFramePose.changeFrame(ReferenceFrame.getWorldFrame());

         // Publish image
         ImageMessage depthImageMessage = new ImageMessage();
         ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(depthPNGPointer.limit());
         imageMessageDataPacker.pack(depthImageMessage, depthPNGPointer);
         MessageTools.toMessage(gpuDepthImage16UC1.getAcquisitionTime(), depthImageMessage.getAcquisitionTime());
         depthImageMessage.setFocalLengthXPixels(gpuDepthImage16UC1.getFocalLengthX());
         depthImageMessage.setFocalLengthYPixels(gpuDepthImage16UC1.getFocalLengthY());
         depthImageMessage.setPrincipalPointXPixels(gpuDepthImage16UC1.getPrincipalPointX());
         depthImageMessage.setPrincipalPointYPixels(gpuDepthImage16UC1.getPrincipalPointY());
         depthImageMessage.setImageWidth(gpuDepthImage16UC1.getImageWidth());
         depthImageMessage.setImageHeight(gpuDepthImage16UC1.getImageHeight());
         depthImageMessage.getPosition().set(leftCameraFramePose.getPosition());
         depthImageMessage.getOrientation().set(leftCameraFramePose.getOrientation());
         depthImageMessage.setSequenceNumber(gpuDepthImage16UC1.getSequenceNumber());
         depthImageMessage.setDepthDiscretization(gpuDepthImage16UC1.getDepthDiscretization());
         CameraModel.PINHOLE.packMessageFormat(depthImageMessage);
         ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(depthImageMessage);

         ros2DepthImagePublisher.publish(depthImageMessage);

         lastDepthSequenceNumber = gpuDepthImage16UC1.getSequenceNumber();

         // Close GpuMat
         depthPNGPointer.close();
         gpuDepthImage16UC1.destroy();
      }
   }

   private void publishColorImage(RawImage gpuColorImageBGR, RobotSide side)
   {
      // Perform safety checks
      if (gpuColorImageBGR != null && !gpuColorImageBGR.isEmpty() && gpuColorImageBGR.getSequenceNumber() != lastColorSequenceNumbers.get(side))
      {
         // Compress image
         BytePointer colorJPEGPointer = new BytePointer((long) gpuColorImageBGR.getImageHeight() * gpuColorImageBGR.getImageWidth());
         imageEncoders.get(side).encodeBGR(gpuColorImageBGR.getGpuImageMatrix().data(),
                                colorJPEGPointer,
                                gpuColorImageBGR.getImageWidth(),
                                gpuColorImageBGR.getImageHeight(),
                                gpuColorImageBGR.getGpuImageMatrix().step());

         // Publish compressed image
         ImageMessage colorImageMessage = new ImageMessage();
         ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(colorJPEGPointer.limit());
         imageMessageDataPacker.pack(colorImageMessage, colorJPEGPointer);
         MessageTools.toMessage(gpuColorImageBGR.getAcquisitionTime(), colorImageMessage.getAcquisitionTime());
         colorImageMessage.setFocalLengthXPixels(gpuColorImageBGR.getFocalLengthX());
         colorImageMessage.setFocalLengthYPixels(gpuColorImageBGR.getFocalLengthY());
         colorImageMessage.setPrincipalPointXPixels(gpuColorImageBGR.getPrincipalPointX());
         colorImageMessage.setPrincipalPointYPixels(gpuColorImageBGR.getPrincipalPointY());
         colorImageMessage.setImageWidth(gpuColorImageBGR.getImageWidth());
         colorImageMessage.setImageHeight(gpuColorImageBGR.getImageHeight());
         colorImageMessage.getPosition().set(cameraPosesInDepthFrame.get(side).getPosition());
         colorImageMessage.getOrientation().set(cameraPosesInDepthFrame.get(side).getOrientation());
         colorImageMessage.setSequenceNumber(gpuColorImageBGR.getSequenceNumber());
         colorImageMessage.setDepthDiscretization(gpuColorImageBGR.getDepthDiscretization());
         CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
         ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(colorImageMessage);
         ros2ColorImagePublishers.get(side).publish(colorImageMessage);

         lastColorSequenceNumbers.put(side, gpuColorImageBGR.getSequenceNumber());

         // Close stuff
         colorJPEGPointer.close();
         gpuColorImageBGR.destroy();
      }
   }

   public void start()
   {
      publishDepthThread.start();
      for (RobotSide side : RobotSide.values)
      {
         publishColorThreads.get(side).start();
      }
   }

   public void stop()
   {
      running = false;

      try
      {
         publishDepthThread.join();
         for (RobotSide side : RobotSide.values)
         {
            publishColorThreads.get(side).join();
         }
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }

      nextDepthImage.destroy();
      for (RobotSide side : RobotSide.values)
      {
         imageEncoders.get(side).destroy();
         nextColorImages.get(side).destroy();
      }
   }

   public void setNextDepthImage(RawImage depthImage)
   {
      depthPublishLock.lock();
      try
      {
         nextDepthImage = depthImage;
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
         nextColorImages.put(side, colorImage);
         newColorImagesAvailable.get(side).signal();
      }
      finally
      {
         colorPublishLocks.get(side).unlock();
      }
   }

   // TODO: Use some better thread class
   private class DepthPublisherThread extends Thread
   {
      public DepthPublisherThread()
      {
         super("ZEDDepthImagePublisher");
      }

      @Override
      public void run()
      {
         while (running)
         {
            depthPublishLock.lock();
            try
            {
               while (nextDepthImage == null ||
                      nextDepthImage.isEmpty() ||
                      nextDepthImage.getSequenceNumber() == lastDepthSequenceNumber)
               {
                  newDepthImageAvailable.await();
               }

               RawImage depthImageToPublish = new RawImage(nextDepthImage);
               publishDepthImage(depthImageToPublish);
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
      }
   }

   private class ColorPublisherThread extends Thread
   {
      private final RobotSide side;

      public ColorPublisherThread(RobotSide side)
      {
         super("ZED" + side.getPascalCaseName() + "ImagePublisher");
         this.side = side;
      }

      @Override
      public void run()
      {
         while (running)
         {
            colorPublishLocks.get(side).lock();
            try
            {
               while (nextColorImages.get(side) == null ||
                      nextColorImages.get(side).isEmpty() ||
                      nextColorImages.get(side).getSequenceNumber() == lastColorSequenceNumbers.get(side))
               {
                  newColorImagesAvailable.get(side).await();
               }

               RawImage colorImageToPublish = new RawImage(nextColorImages.get(side));
               publishColorImage(colorImageToPublish, side);
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
      }
   }
}
