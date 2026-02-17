package us.ihmc.rdx.ui.graphics.ros2.yolo;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2YOLOv8Visualizer extends RDXROS2ImageMessageVisualizer
{
   private final ROS2Heartbeat demandYOLO;

   private final RDXROS2YOLOv8Settings settings;

   public RDXROS2YOLOv8Visualizer(String title,
                                  ROS2Node ros2Node,
                                  ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                  ROS2Topic<ImageMessage> yoloAnnotatedImageTopic)
   {
      super(title, ros2Node, yoloAnnotatedImageTopic);

      demandYOLO = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO);

      settings = new RDXROS2YOLOv8Settings(ros2Node, ros2ClockOffsetEstimator);
   }

   @Override
   public void updateHeartbeat()
   {
      super.updateHeartbeat();
      demandYOLO.setAlive(isActive());
      settings.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      settings.renderSettings();
      super.renderImGuiWidgets();
   }

   @Override
   public void destroy()
   {
      super.destroy();
      settings.destroy();
      demandYOLO.destroy();
   }
}
