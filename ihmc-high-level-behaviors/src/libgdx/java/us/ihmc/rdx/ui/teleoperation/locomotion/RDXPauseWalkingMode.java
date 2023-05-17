package us.ihmc.rdx.ui.teleoperation.locomotion;

import controller_msgs.msg.dds.PauseWalkingMessage;
import imgui.ImGui;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

/**
 * Manages the PauseWalkingMessage that determines whether the robot is walking.
 * Buttons are displayed onto the UI that can be clicked to Pause or Continue walking.
 */
public class RDXPauseWalkingMode
{
   private final CommunicationHelper communicationHelper;
   private final PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXPauseWalkingMode(CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Walking Options:");
      ImGui.sameLine();

      if (ImGui.button(labels.get("Pause")))
      {
         setPauseWalking(true);
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Space");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Continue")))
      {
         setPauseWalking(false);
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Space");
   }

   public void setPauseWalking(boolean pauseWalking)
   {
      pauseWalkingMessage.setPause(pauseWalking);
      communicationHelper.publishToController(pauseWalkingMessage);

      if (pauseWalking)
         LogTools.info("Paused Walking");
      else
         LogTools.info("Continue Walking");
   }

   public boolean getPauseWalking()
   {
      return pauseWalkingMessage.getPause();
   }
}
