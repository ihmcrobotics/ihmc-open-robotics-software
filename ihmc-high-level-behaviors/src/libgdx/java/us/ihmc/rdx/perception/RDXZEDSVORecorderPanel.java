package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import perception_msgs.msg.dds.ZEDSVOCurrentFileMessage;
import std_msgs.msg.dds.Int64;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.ros2.ROS2Node;

public class RDXZEDSVORecorderPanel
{
   private static final String PANEL_NAME = "ZED SVO Recorder";

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ROS2Helper ros2Helper;
   private ZEDSVOCurrentFileMessage latestMessage;

   private ImInt requestedPosition = new ImInt();
   private boolean paused = false;

   private long lastTimeSinceSVOInfoMessageMillis;

   public RDXZEDSVORecorderPanel(ROS2Node ros2Node)
   {
      ros2Helper = new ROS2Helper(ros2Node);

      ros2Helper.subscribeViaCallback(PerceptionAPI.ZED_SVO_CURRENT_FILE, zedsvoCurrentFileMessage ->
      {
         this.latestMessage = zedsvoCurrentFileMessage;
         lastTimeSinceSVOInfoMessageMillis = System.currentTimeMillis();
      });
   }

   public void update()
   {
      RDXBaseUI baseUI = RDXBaseUI.getInstance();

      boolean overlayPanelExists = baseUI.getPrimary3DPanel().overlayPanelExists(PANEL_NAME);
      boolean updatedRecently = System.currentTimeMillis() - lastTimeSinceSVOInfoMessageMillis < 3000;

      if (!overlayPanelExists && updatedRecently)
      {
         baseUI.getPrimary3DPanel().addOverlayPanel(PANEL_NAME, this::render);
      }
      else if (overlayPanelExists && !updatedRecently)
      {
         baseUI.getPrimary3DPanel().removeOverlayPanel(PANEL_NAME);
      }
   }

   public void render()
   {
      ImGuiTools.textBold("Current SVO:");
      ImGui.sameLine();
      ImGui.textWrapped(latestMessage.getCurrentFileName().toString());

      if (!paused)
         requestedPosition.set((int) latestMessage.getCurrentPosition());

      if (ImGuiTools.sliderInt(labels.get("Position"), requestedPosition, 0, (int) latestMessage.getLength()))
      {
         ros2Helper.publish(PerceptionAPI.ZED_SVO_PAUSE);
         paused = true;
      }
      if (ImGui.isItemDeactivatedAfterEdit())
      {
         Int64 positionMessage = new Int64();
         positionMessage.setData(requestedPosition.get());
         ros2Helper.publish(PerceptionAPI.ZED_SVO_SET_POSITION, positionMessage);
         ros2Helper.publish(PerceptionAPI.ZED_SVO_PLAY);
         paused = false;
      }

      ImGui.sameLine();

      if (ImGui.button(labels.get(paused ? "Play" : "Pause")))
      {
         ros2Helper.publish(paused ? PerceptionAPI.ZED_SVO_PLAY : PerceptionAPI.ZED_SVO_PAUSE);
         paused = !paused;
      }
   }
}
