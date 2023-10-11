package us.ihmc.perception.sensorPublishing;

import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2HeartbeatMonitor;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.Throttler;

public class SensorPublisher
{
   private ROS2HeartbeatMonitor zedColorHeartbeat;
   private ROS2HeartbeatMonitor zedDepthHeartbeat;
   private final ZEDColorDepthImageRetriever zedImageGrabber;
   private final ZEDColorDepthImagePublisher zedImagePublisher;
   private final Thread zedPublishThread;
   private final Throttler zedThreadThrottler = new Throttler();

   private boolean running = true;

   SensorPublisher(ROS2PublishSubscribeAPI ros2,
                   ZEDColorDepthImageRetriever zedImageGrabber,
                   ZEDColorDepthImagePublisher zedImagePublisher)
   {
      zedColorHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_COLOR);
      zedDepthHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_DEPTH);
      this.zedImageGrabber = zedImageGrabber;
      this.zedImagePublisher = zedImagePublisher;
      zedThreadThrottler.setFrequency(30.0); // publish at 30hz

      zedPublishThread = new Thread(() ->
      {
         while(running)
         {
            zedThreadThrottler.waitAndRun();
            RawImage gpuDepthImage16UC11 = zedImageGrabber.getLatestRawDepthImage();
            RawImage gpuLeftColorImage = zedImageGrabber.getLatestRawColorImage(RobotSide.LEFT);
            RawImage gpuRightColorImage = zedImageGrabber.getLatestRawColorImage(RobotSide.RIGHT);
            // Do processing on image
            this.zedImagePublisher.publishDepthImage(gpuDepthImage16UC11);
            this.zedImagePublisher.publishColorImage(gpuLeftColorImage, RobotSide.LEFT);
            this.zedImagePublisher.publishColorImage(gpuRightColorImage, RobotSide.RIGHT);
         }
      }, "ZEDImagePublisherThread");
   }

   public void start()
   {
      zedImageGrabber.start();
      zedPublishThread.start();
   }

   public void stop()
   {
      running = false;

      try
      {
         zedPublishThread.join();
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "sensor_publisher");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ZEDColorDepthImageRetriever zedImageGrabber = new ZEDColorDepthImageRetriever(0);
      ZEDColorDepthImagePublisher zedImagePublisher = new ZEDColorDepthImagePublisher(zedImageGrabber.getZedModelData(),
                                                                                      PerceptionAPI.ZED2_COLOR_IMAGES,
                                                                                      PerceptionAPI.ZED2_DEPTH,
                                                                                      ReferenceFrame::getWorldFrame);

      SensorPublisher publisher = new SensorPublisher(ros2Helper, zedImageGrabber, zedImagePublisher);
      publisher.start();
   }
}
