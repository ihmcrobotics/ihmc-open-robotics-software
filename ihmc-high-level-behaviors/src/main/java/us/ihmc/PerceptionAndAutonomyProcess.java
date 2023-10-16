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
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.function.Supplier;

public class PerceptionAndAutonomyProcess
{
   private static final int ZED_CAMERA_ID = 0;
   private static final double ZED_FPS = 30.0;
   private static final SideDependentList<ROS2Topic<ImageMessage>> ZED_COLOR_TOPICS = PerceptionAPI.ZED2_COLOR_IMAGES;
   private static final ROS2Topic<ImageMessage> ZED_DEPTH_TOPIC = PerceptionAPI.ZED2_DEPTH;

   private static final String REALSENSE_SERIAL_NUMBER = System.getProperty("d455.serial.number", "215122253249");
   private static final double REALSENSE_FPS = 20.0;

   private RawImage zedDepthImage;
   private final SideDependentList<RawImage> zedColorImages = new SideDependentList<>();
   private final ROS2HeartbeatMonitor zedColorHeartbeat;
   private final ROS2HeartbeatMonitor zedDepthHeartbeat;
   private final ZEDColorDepthImageRetriever zedImageRetriever;
   private final ZEDColorDepthImagePublisher zedImagePublisher;
   private final RestartableThrottledThread zedProcessAndPublishThread;

   private RawImage realsenseDepthImage;
   private RawImage realsenseColorImage;
   private final ROS2HeartbeatMonitor realsenseDepthHeartbeat;
   private final ROS2HeartbeatMonitor realsenseColorHeartbeat;
   private final RealsenseColorDepthImageRetriever realsenseImageRetriever;
   private final RestartableThrottledThread realsenseProcessAndPublishThread;

   PerceptionAndAutonomyProcess(ROS2PublishSubscribeAPI ros2,
                                Supplier<ReferenceFrame> zedFrameSupplier)
   {
      zedColorHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_COLOR);
      zedDepthHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_DEPTH);
      zedImageRetriever = new ZEDColorDepthImageRetriever(ZED_CAMERA_ID);
      zedImagePublisher = new ZEDColorDepthImagePublisher(zedImageRetriever.getZedModelData(), ZED_COLOR_TOPICS, ZED_DEPTH_TOPIC, zedFrameSupplier);
      zedProcessAndPublishThread = new RestartableThrottledThread("ZEDImageProcessAndPublish", ZED_FPS, this::processAndPublishZED);
      initializeZED();

      realsenseColorHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_REALSENSE_COLOR);
      realsenseDepthHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_REALSENSE_DEPTH);
      realsenseImageRetriever = new RealsenseColorDepthImageRetriever(REALSENSE_SERIAL_NUMBER, RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ);
      // TODO: add realsense publisher
      realsenseProcessAndPublishThread = new RestartableThrottledThread("RealsenseProcessAndPublish", REALSENSE_FPS, this::processAndPublishRealsense);
      initializeRealsense();
   }

   public void start()
   {
      zedProcessAndPublishThread.start();
      realsenseProcessAndPublishThread.start();
      zedImageRetriever.start();
      zedImagePublisher.startAll();
   }

   public void destroy()
   {
      zedProcessAndPublishThread.stop();
      realsenseProcessAndPublishThread.stop();
      zedImagePublisher.destroy();
      zedImageRetriever.destroy();
   }

   private void processAndPublishZED()
   {
      zedDepthImage = zedImageRetriever.getLatestRawDepthImage();
      for (RobotSide side : RobotSide.values)
      {
         zedColorImages.put(side, zedImageRetriever.getLatestRawColorImage(side));
      }

      // Do processing on image

      zedImagePublisher.setNextDepthImage(zedDepthImage);
      for (RobotSide side : RobotSide.values)
      {
         zedImagePublisher.setNextColorImage(zedColorImages.get(side), side);
      }
   }

   private void processAndPublishRealsense()
   {
      realsenseDepthImage = realsenseImageRetriever.getLatestRawDepthImage();
      realsenseColorImage = realsenseImageRetriever.getLatestRawColorImage();

      // Do processing on image

      // realsenseImagePublisher.setNextDepthImage(realsenseDepthImage);
      // realsenseImagePublisher.setNextColorImage(realsenseColorImage);
   }

   private void initializeZED()
   {
      zedDepthHeartbeat.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            zedImageRetriever.start();
            zedImagePublisher.startDepth();
         }
         else
         {
            zedImagePublisher.stopDepth();
            if (!zedDepthHeartbeat.isAlive())
               zedImageRetriever.stop();
         }
      });
      zedColorHeartbeat.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            zedImageRetriever.start();
            zedImagePublisher.startColor();
         }
         else
         {
            zedImagePublisher.stopColor();
            if (!zedColorHeartbeat.isAlive())
               zedImageRetriever.stop();
         }
      });
   }

   private void initializeRealsense()
   {

   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "perception_autonomy_process");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      PerceptionAndAutonomyProcess publisher = new PerceptionAndAutonomyProcess(ros2Helper, ReferenceFrame::getWorldFrame);
      publisher.start();
   }
}
