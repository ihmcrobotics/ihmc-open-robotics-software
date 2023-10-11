package us.ihmc;

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
import us.ihmc.tools.thread.PausableThrottledThread;

public class PerceptionAndAutonomyProcess
{
   private ROS2HeartbeatMonitor zedColorHeartbeat;
   private ROS2HeartbeatMonitor zedDepthHeartbeat;
   private final ZEDColorDepthImageRetriever zedImageGrabber;
   private final ZEDColorDepthImagePublisher zedImagePublisher;
   private final PausableThrottledThread zedProcessAndPublishThread;

   private boolean running = true;

   PerceptionAndAutonomyProcess(ROS2PublishSubscribeAPI ros2,
                                ZEDColorDepthImageRetriever zedImageGrabber,
                                ZEDColorDepthImagePublisher zedImagePublisher)
   {
      zedColorHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_COLOR);
      zedDepthHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_DEPTH);
      this.zedImageGrabber = zedImageGrabber;
      this.zedImagePublisher = zedImagePublisher;

      zedProcessAndPublishThread = new PausableThrottledThread("ZEDImageProcessAndPublish", 30.0, () ->
      {
         RawImage gpuDepthImage16UC11 = zedImageGrabber.getLatestRawDepthImage();
         RawImage gpuLeftColorImage = zedImageGrabber.getLatestRawColorImage(RobotSide.LEFT);
         RawImage gpuRightColorImage = zedImageGrabber.getLatestRawColorImage(RobotSide.RIGHT);

         // Do processing on image

         this.zedImagePublisher.setNextDepthImage(gpuDepthImage16UC11);
         this.zedImagePublisher.setNextColorImage(gpuLeftColorImage, RobotSide.LEFT);
         this.zedImagePublisher.setNextColorImage(gpuRightColorImage, RobotSide.RIGHT);
      });
   }

   public void start()
   {
      zedProcessAndPublishThread.start();
      zedImagePublisher.start();
      zedImageGrabber.start();
   }

   public void stop()
   {
      running = false;
      zedProcessAndPublishThread.stop();
      zedImagePublisher.stop();
      zedImageGrabber.stop();
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "sensor_publisher");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ZEDColorDepthImageRetriever zedImageGrabber = new ZEDColorDepthImageRetriever(0);
      // TODO: Make variables private static final fields
      ZEDColorDepthImagePublisher zedImagePublisher = new ZEDColorDepthImagePublisher(zedImageGrabber.getZedModelData(),
                                                                                      PerceptionAPI.ZED2_COLOR_IMAGES,
                                                                                      PerceptionAPI.ZED2_DEPTH,
                                                                                      ReferenceFrame::getWorldFrame);

      PerceptionAndAutonomyProcess publisher = new PerceptionAndAutonomyProcess(ros2Helper, zedImageGrabber, zedImagePublisher);
      publisher.start();
   }
}
