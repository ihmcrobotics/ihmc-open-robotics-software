package us.ihmc.rdx.ui.graphics.ros2.yolo;

import imgui.ImGui;
import imgui.ImGuiStyle;
import imgui.type.ImInt;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2YOLOv8Visualizer extends RDXROS2ImageMessageVisualizer
{
   private static final String[] AVAILABLE_SENSORS = {"ZED", "D455"}; // FIXME: This is robot specific :( This class can't be used with any arbitrary robot

   private final ROS2Heartbeat demandYOLOv8ZED;
   private final ROS2Heartbeat demandYOLOv8D455;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt selectedSensor = new ImInt(0); // 0 = ZED, 1 = Realsense

   private final RDXROS2YOLOv8Settings settings;

   public RDXROS2YOLOv8Visualizer(String title,
                                  ROS2Node ros2Node,
                                  ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                  ROS2Topic<ImageMessage> yoloAnnotatedImageTopic)
   {
      super(title, ros2Node, yoloAnnotatedImageTopic);

      demandYOLOv8ZED = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO_ZED);
      demandYOLOv8D455 = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO_REALSENSE);

      settings = new RDXROS2YOLOv8Settings(ros2Node, ros2ClockOffsetEstimator);
   }

   @Override
   public void updateHeartbeat()
   {
      super.updateHeartbeat();
      demandYOLOv8ZED.setAlive(isActive() && selectedSensor.get() == 0);
      demandYOLOv8D455.setAlive(isActive() && selectedSensor.get() == 1);
      settings.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGuiStyle style = new ImGuiStyle();

      ImGui.combo(labels.get("Sensor Selection"), selectedSensor, AVAILABLE_SENSORS);

      if (ImGui.button(labels.get("Enable All")))
         settings.enableAllModels();
      ImGui.sameLine();
      if (ImGui.button(labels.get("Disable All")))
         settings.disableAllModels();

      settings.renderSettings();

      super.renderImGuiWidgets();
   }

   @Override
   public void destroy()
   {
      super.destroy();
      settings.destroy();
      demandYOLOv8D455.destroy();
      demandYOLOv8ZED.destroy();
   }
}
