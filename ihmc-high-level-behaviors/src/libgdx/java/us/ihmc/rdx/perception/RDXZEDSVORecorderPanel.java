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
import us.ihmc.tools.thread.Throttler;

public class RDXZEDSVORecorderPanel
{
   private static final String PANEL_NAME = "ZED SVO Recorder";

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ROS2Helper ros2Helper;
   private ZEDSVOCurrentFileMessage latestMessage;

   private final ImInt requestedPosition = new ImInt();
   private boolean holdingOnToTheSlider;
   private boolean paused;

   private final Throttler requestThrottler = new Throttler().setFrequency(5.0);

   public RDXZEDSVORecorderPanel(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;
      ros2Helper.subscribeViaCallback(PerceptionAPI.ZED_SVO_CURRENT_FILE, message -> this.latestMessage = message);
   }

   public void update()
   {
      RDXBaseUI baseUI = RDXBaseUI.getInstance();

      boolean overlayPanelExists = baseUI.getPrimary3DPanel().overlayPanelExists(PANEL_NAME);

      if (!overlayPanelExists && latestMessage != null)
      {
         baseUI.getPrimary3DPanel().addOverlayPanel(PANEL_NAME, this::render);
      }
   }

   public void render()
   {
      ImGuiTools.textBold("Current SVO:");
      ImGui.sameLine();
      ImGui.textWrapped(latestMessage.getCurrentFileName().toString());

      if (!holdingOnToTheSlider)
         requestedPosition.set((int) latestMessage.getCurrentPosition());

      if (ImGuiTools.sliderInt(labels.get("Position"), requestedPosition, 0, (int) latestMessage.getLength()))
      {
         holdingOnToTheSlider = true;

         if (requestThrottler.run())
         {
            publishPositionRequest();
         }
      }
      // Called once you let go of the slider
      if (ImGui.isItemDeactivatedAfterEdit())
      {
         holdingOnToTheSlider = false;

         publishPositionRequest();
      }

      ImGui.sameLine();

      if (ImGui.button(labels.get(paused ? "Play" : "Pause")))
      {
         ros2Helper.publish(paused ? PerceptionAPI.ZED_SVO_PLAY : PerceptionAPI.ZED_SVO_PAUSE);
         paused = !paused;
      }
   }

   private void publishPositionRequest()
   {
      Int64 positionMessage = new Int64();
      positionMessage.setData(requestedPosition.get());
      ros2Helper.publish(PerceptionAPI.ZED_SVO_SET_POSITION, positionMessage);
   }
}
