package us.ihmc.gdx.perception;

import imgui.ImGui;
import std_msgs.msg.dds.Float64;
import us.ihmc.avatar.colorVision.DualBlackflyComms;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class GDXRemoteBlackflyArUcoDetectionUI
{
   private final ImGuiPanel panel = new ImGuiPanel("ArUco Marker Detection", this::renderImGuiWidgets);
   private ROS2Helper ros2Helper;
   private final SideDependentList<IHMCROS2Input<Float64>> publishRateInputs = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlot> publishRatePlots = new SideDependentList<>();

   public GDXRemoteBlackflyArUcoDetectionUI(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;
      for (RobotSide side : RobotSide.values)
      {
         publishRateInputs.put(side, ros2Helper.subscribe(DualBlackflyComms.PUBLISH_RATE.get(side)));
         publishRatePlots.put(side, new ImPlotDoublePlot(side.getPascalCaseName() + " Publish Rate", 30));
      }
   }

   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         if (publishRateInputs.get(side).hasReceivedFirstMessage())
         {
            publishRatePlots.get(side).addValue(publishRateInputs.get(side).getLatest().getData());
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Reconnect remote ROS 1 node"))
      {
         ros2Helper.publish(DualBlackflyComms.RECONNECT_ROS1_NODE);
      }
      for (RobotSide side : RobotSide.values)
      {
         publishRatePlots.get(side).renderImGuiWidgets();
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
