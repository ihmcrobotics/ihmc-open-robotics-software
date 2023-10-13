package us.ihmc;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2HeartbeatMonitor;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.function.Supplier;

public class PerceptionAndAutonomyProcess
{
   private static final int ZED_CAMERA_ID = 0;
   private static final SideDependentList<ROS2Topic<ImageMessage>> ZED_COLOR_TOPICS = PerceptionAPI.ZED2_COLOR_IMAGES;
   private static final ROS2Topic<ImageMessage> ZED_DEPTH_TOPIC = PerceptionAPI.ZED2_DEPTH;

   private RawImage zedDepthImage;
   private final SideDependentList<RawImage> zedColorImages = new SideDependentList<>();
   private ROS2HeartbeatMonitor zedColorHeartbeat;
   private ROS2HeartbeatMonitor zedDepthHeartbeat;
   private final ZEDColorDepthImageRetriever zedImageGrabber;
   private final ZEDColorDepthImagePublisher zedImagePublisher;
   private final RestartableThrottledThread zedProcessAndPublishThread;

   PerceptionAndAutonomyProcess(ROS2PublishSubscribeAPI ros2,
                                Supplier<ReferenceFrame> zedFrameSupplier)
   {
      zedColorHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_COLOR);
      zedDepthHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_DEPTH);
      this.zedImageGrabber = new ZEDColorDepthImageRetriever(ZED_CAMERA_ID);
      this.zedImagePublisher = new ZEDColorDepthImagePublisher(zedImageGrabber.getZedModelData(), ZED_COLOR_TOPICS, ZED_DEPTH_TOPIC, zedFrameSupplier);
      zedDepthHeartbeat.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            zedImageGrabber.start();
            zedImagePublisher.startDepth();
         }
         else
         {
            zedImagePublisher.stopDepth();
            if (!zedDepthHeartbeat.isAlive())
               zedImageGrabber.stop();
         }
      });
      zedColorHeartbeat.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            zedImageGrabber.start();
            zedImagePublisher.startColor();
         }
         else
         {
            zedImagePublisher.stopColor();
            if (!zedColorHeartbeat.isAlive())
               zedImageGrabber.stop();
         }
      });

      zedProcessAndPublishThread = new RestartableThrottledThread("ZEDImageProcessAndPublish", 30.0, () ->
      {
         zedDepthImage = zedImageGrabber.getLatestRawDepthImage();
         for (RobotSide side : RobotSide.values)
         {
            zedColorImages.put(side, zedImageGrabber.getLatestRawColorImage(side));
         }

         // Do processing on image

         this.zedImagePublisher.setNextDepthImage(zedDepthImage);
         for (RobotSide side : RobotSide.values)
         {
            zedImagePublisher.setNextColorImage(zedColorImages.get(side), side);
         }
      });
   }

   public void start()
   {
      zedProcessAndPublishThread.start();
      zedImageGrabber.start();
      zedImagePublisher.startAll();
   }

   public void destroy()
   {
      zedProcessAndPublishThread.stop();
      zedImagePublisher.destroy();
      zedImageGrabber.destroy();
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "perception_autonomy_process");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      PerceptionAndAutonomyProcess publisher = new PerceptionAndAutonomyProcess(ros2Helper, ReferenceFrame::getWorldFrame);
      publisher.start();
   }
}
