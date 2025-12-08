package us.ihmc.perception.streaming;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;

import java.time.Instant;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.ScheduledThreadPoolExecutor;

/**
 * Subscribes to a ZED SDK stream and republishes the data as ROS 2 messages.
 */
public class ROS2ZEDSDKVideoStreamImageMessageRelay extends ZEDImageSensor
{
   private static int lastCameraId = 0;

   private final ROS2Helper ros2Helper;
   private final RepeatingTaskThread publishThread;
   private final ExecutorService publisherExecutor;

   private final ImageMessage lastDepthImageMessage;
   private final ImageMessage lastLeftColorImageMessage;
   private final ImageMessage lastRightColorImageMessage;

   private final ROS2Topic<ImageMessage> depthTopic;
   private final SideDependentList<ROS2Topic<ImageMessage>> colorTopics;

   public ROS2ZEDSDKVideoStreamImageMessageRelay(ROS2Node ros2Node,
                                                 ZEDModelData zedModel,
                                                 int slDepthMode,
                                                 String remoteStreamingAddress,
                                                 int remoteStreamingPort,
                                                 ROS2Topic<ImageMessage> depthTopic,
                                                 SideDependentList<ROS2Topic<ImageMessage>> colorTopics)
   {
      super(lastCameraId++, zedModel, slDepthMode, remoteStreamingAddress, remoteStreamingPort);

      ros2Helper = new ROS2Helper(ros2Node);
      publishThread = new RepeatingTaskThread(getClass().getSimpleName() + "-PublishThread", this::publish);
      publisherExecutor = new ScheduledThreadPoolExecutor(3 * 10);

      lastDepthImageMessage = new ImageMessage();
      lastLeftColorImageMessage = new ImageMessage();
      lastRightColorImageMessage = new ImageMessage();

      this.depthTopic = depthTopic;
      this.colorTopics = colorTopics;

      publishThread.startRepeating();
   }

   private void publish() throws InterruptedException
   {
      if (isSensorRunning())
      {
         double timeout = 0.5;
         waitForGrab(timeout);

         RawImage depthImage = getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
         RawImage leftColorImage = getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage rightColorImage = getImage(ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);

         // Pack all RawImages into ImageMessages
         packImageMessage(depthImage, lastDepthImageMessage);
         packImageMessage(leftColorImage, lastLeftColorImageMessage);
         packImageMessage(rightColorImage, lastRightColorImageMessage);

         // Publish async to not block receiving new images
         publisherExecutor.submit(() -> ros2Helper.publish(depthTopic, lastDepthImageMessage));
         publisherExecutor.submit(() -> ros2Helper.publish(colorTopics.get(RobotSide.LEFT), lastLeftColorImageMessage));
         publisherExecutor.submit(() -> ros2Helper.publish(colorTopics.get(RobotSide.RIGHT), lastRightColorImageMessage));

         rightColorImage.release();
         leftColorImage.release();
         depthImage.release();
      }
      else
      {
         ThreadTools.park(0.5);
      }
   }

   private void packImageMessage(RawImage frame, ImageMessage imageMessage)
   {
      // Set acquisition time as now... this isn't super accurate though
      MessageTools.toMessage(Instant.now(), imageMessage.getAcquisitionTime());

      Mat frameMat = frame.getCpuImageMat();
      PerceptionMessageTools.packImageMessageData(imageMessage, frameMat.data().limit(OpenCVTools.dataSize(frameMat)));
      PerceptionMessageTools.packImageMessageMetadata(imageMessage, frame);

      imageMessage.setCompressionType(CompressionType.UNCOMPRESSED.toByte());
   }

   @Override
   public void close()
   {
      publisherExecutor.shutdownNow();
      publishThread.blockingKill();

      super.close();
   }
}
