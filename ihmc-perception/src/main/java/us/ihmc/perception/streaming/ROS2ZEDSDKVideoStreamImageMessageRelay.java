package us.ihmc.perception.streaming;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.robotDataLogger.logger.ZEDSVOLoggerManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;

import java.net.InetAddress;
import java.net.UnknownHostException;

/**
 * Subscribes to a ZED SDK stream and republishes the data as ROS 2 messages.
 */
public class ROS2ZEDSDKVideoStreamImageMessageRelay extends RepeatingTaskThread
{
   private final ROS2Node localNode;

   /*
    * These fields are null until a remote connection has been established
    */
   private ZEDImageSensor remoteZEDImageSensor;
   private RawImagePublisher depthImagePublisher;
   private RawImagePublisher leftColorImagePublisher;
   private RawImagePublisher rightColorImagePublisher;

   public ROS2ZEDSDKVideoStreamImageMessageRelay(ZEDModelData zedModel, int slDepthMode)
   {
      super("ROS2ZEDSDKVideoStreamImageMessageRelay");

      try
      {
         localNode = new ROS2NodeBuilder().addressRestriction(InetAddress.getLocalHost()).build("zed_sdk_relay_node");
      }
      catch (UnknownHostException e)
      {
         LogTools.error("Unable to bind ROS2Node to localhost");
         throw new RuntimeException(e);
      }

      // #runTask will repeat indefinitely and wait for each new frame from the ZED, if connected.
      startRepeating();

      // Listen for the robot broadcasting that it's available to stream from
      localNode.createSubscription2(ZEDSVOLoggerManager.ZED_SDK_ANNOUNCE_TOPIC, msg ->
      {
         if (remoteZEDImageSensor == null)
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

               /*
                * Create the image publishers
                */
               depthImagePublisher = new RawImagePublisher(localNode);
               leftColorImagePublisher = new RawImagePublisher(localNode);
               rightColorImagePublisher = new RawImagePublisher(localNode);
            }
         }
      });
   }

   @Override
   protected synchronized void runTask() throws Throwable
   {
      if (remoteZEDImageSensor != null && remoteZEDImageSensor.isSensorRunning())
      {
         double timeout = 0.5;
         remoteZEDImageSensor.waitForGrab(timeout);

         RawImage depthImage = remoteZEDImageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
         RawImage leftColorImage = remoteZEDImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage rightColorImage = remoteZEDImageSensor.getImage(ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);

         depthImagePublisher.publishImage(PerceptionAPI.ZED_DEPTH, depthImage);
         leftColorImagePublisher.publishImage(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT), depthImage);
         rightColorImagePublisher.publishImage(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.RIGHT), depthImage);

         rightColorImage.release();
         leftColorImage.release();
         depthImage.release();
      }
      else
      {
         // Sensor not connected, sleep for some time
         ThreadTools.park(0.5);
      }
   }
}
