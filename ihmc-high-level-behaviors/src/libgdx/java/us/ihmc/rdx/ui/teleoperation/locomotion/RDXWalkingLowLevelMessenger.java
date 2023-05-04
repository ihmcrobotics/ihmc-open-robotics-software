package us.ihmc.rdx.ui.teleoperation.locomotion;

import imgui.ImGui;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXWalkingLowLevelMessenger
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final RobotLowLevelMessenger robotLowLevelMessenger;

   public RDXWalkingLowLevelMessenger(CommunicationHelper communicationHelper)
   {
      robotLowLevelMessenger = communicationHelper.getOrCreateRobotLowLevelMessenger();

      if (robotLowLevelMessenger == null)
      {
         String robotName = communicationHelper.getRobotModel().getSimpleRobotName();
         throw new RuntimeException("Please add implementation of RobotLowLevelMessenger for " + robotName);
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button(labels.get("Pause")))
      {
         sendPauseWalkingRequest();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Continue")))
      {
         sendContinueWalkingRequest();
      }
   }

   public void sendPauseWalkingRequest()
   {
      robotLowLevelMessenger.sendPauseWalkingRequest();
   }

   public void sendContinueWalkingRequest()
   {
      robotLowLevelMessenger.sendContinueWalkingRequest();
   }
}
