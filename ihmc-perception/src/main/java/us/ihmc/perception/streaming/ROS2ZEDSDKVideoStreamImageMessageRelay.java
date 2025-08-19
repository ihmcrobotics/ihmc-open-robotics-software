package us.ihmc.perception.streaming;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotDataLogger.logger.ZEDSVOLoggerManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.time.Instant;

/**
 * Subscribes to a ZED SDK stream and republishes the data as ROS 2 messages.
 */
public class ROS2ZEDSDKVideoStreamImageMessageRelay extends RepeatingTaskThread
{
   private static final boolean ZED_SDK_LOADED = ZEDJavaAPINativeLibrary.load();

   private final ROS2Helper ros2Helper;

   /*
    * These fields are null until a remote connection has been established
    */
   private ZEDImageSensor remoteZEDImageSensor;
   private ImageMessage lastDepthImageMessage;
   private ImageMessage lastLeftColorImageMessage;
   private ImageMessage lastRightColorImageMessage;

   public ROS2ZEDSDKVideoStreamImageMessageRelay(ROS2Node ros2Node, ZEDModelData zedModel, int slDepthMode)
   {
      super("ROS2ZEDSDKVideoStreamImageMessageRelay");

      ros2Helper = new ROS2Helper(ros2Node);

      // #runTask will repeat indefinitely and wait for each new frame from the ZED, if connected.
      startRepeating();

      // Listen for the robot broadcasting that it's available to stream from
      ros2Node.createSubscription2(ZEDSVOLoggerManager.ZED_SDK_ANNOUNCE_TOPIC, msg ->
      {
         if (remoteZEDImageSensor == null && ZED_SDK_LOADED)
         {
            synchronized (ROS2ZEDSDKVideoStreamImageMessageRelay.this)
            {
               /*
                * Create the image sensor
                */
               int cameraID = 10;
               String remoteStreamingAddress = msg.getAddressAsString();
               int remoteStreamingPort = msg.getPort();
               remoteZEDImageSensor = new ZEDImageSensor(cameraID, zedModel, slDepthMode, remoteStreamingAddress, remoteStreamingPort);
               remoteZEDImageSensor.run(true);

               lastDepthImageMessage = new ImageMessage();
               lastLeftColorImageMessage = new ImageMessage();
               lastRightColorImageMessage = new ImageMessage();

               double waited = 0.0;
               double timeout = 5.0;
               while (!zed.sl_is_opened(remoteZEDImageSensor.getCameraID()))
               {
                  if (waited >= timeout)
                  {
                     System.out.println("hit the timeout");
                     remoteZEDImageSensor.close();
                     remoteZEDImageSensor = null;
                     break;
                  }

                  ThreadTools.park(1.0);
                  waited += 1.0;
               }
            }
         }
      });
   }

   @Override
   protected synchronized void runTask() throws Throwable
   {
      if (remoteZEDImageSensor != null && remoteZEDImageSensor.isSensorRunning())
      {
         System.out.println("connected");

         double timeout = 0.5;
         remoteZEDImageSensor.waitForGrab(timeout);

         RawImage depthImage = remoteZEDImageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
         RawImage leftColorImage = remoteZEDImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage rightColorImage = remoteZEDImageSensor.getImage(ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);

         // Pack all RawImages into ImageMessages
         packImageMessage(depthImage, lastDepthImageMessage);
         packImageMessage(leftColorImage, lastLeftColorImageMessage);
         packImageMessage(rightColorImage, lastRightColorImageMessage);

         ros2Helper.publish(PerceptionAPI.ZED_DEPTH, lastDepthImageMessage);
         ros2Helper.publish(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT), lastLeftColorImageMessage);
         ros2Helper.publish(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.RIGHT), lastRightColorImageMessage);

         rightColorImage.release();
         leftColorImage.release();
         depthImage.release();
      }
      else
      {
         System.out.println("sensor not connected");
         // Sensor not connected, sleep for some time
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
}
