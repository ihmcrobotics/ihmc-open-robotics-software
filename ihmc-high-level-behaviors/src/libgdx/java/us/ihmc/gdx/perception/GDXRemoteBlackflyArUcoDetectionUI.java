package us.ihmc.gdx.perception;

import imgui.ImGui;
import us.ihmc.avatar.colorVision.DualBlackflyComms;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.gdx.imgui.ImGuiPanel;

public class GDXRemoteBlackflyArUcoDetectionUI
{
   private final ImGuiPanel panel = new ImGuiPanel("ArUco Marker Detection", this::renderImGuiWidgets);
   private ROS2Helper ros2Helper;

   public GDXRemoteBlackflyArUcoDetectionUI(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;
   }

   public void update()
   {

   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Reconnect remote ROS 1 node"))
      {
         ros2Helper.publish(DualBlackflyComms.RECONNECT_ROS1_NODE);
      }
   }

   public void destroy()
   {

   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
