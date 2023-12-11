package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandStatusMessage;
import imgui.ImGui;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXSakeHandInformation
{
   private final RobotSide side;
   private final CommunicationHelper communicationHelper;
   private final IHMCROS2Input<SakeHandStatusMessage> statusInput;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFlashingText presentTemperatureText = new ImGuiFlashingText(ImGuiTools.RED);
   private final ImGuiFlashingText errorStatusText = new ImGuiFlashingText(ImGuiTools.RED);

   public RDXSakeHandInformation(RobotSide side, CommunicationHelper communicationHelper)
   {
      this.side = side;
      this.communicationHelper = communicationHelper;
      statusInput = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                           .withTypeName(SakeHandStatusMessage.class),
                                                  message -> message.getRobotSide() == side.toByte());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text(side.getPascalCaseName() + ":");
      ImGui.sameLine();
      if (statusInput.hasReceivedFirstMessage())
      {
         double temperature = 100 * statusInput.getLatest().getNormalizedTemperature();
         String errorMessage = statusInput.getLatest().getErrorMessageAsString();

         if (temperature < 55.0)
         {
            ImGui.textColored(ImGuiTools.greenToRedGradiatedColor(temperature, 45.0, 55.0, 60.0),
                              "Temperature: " + FormattingTools.getFormattedDecimal1D(temperature) + " C  |");
         }
         else
         {
            presentTemperatureText.renderText("Temperature: " + FormattingTools.getFormattedDecimal1D(temperature) + " C  |", true);
         }
         ImGui.sameLine();

         if (errorMessage.isEmpty())
         {
            ImGui.textColored(ImGuiTools.GREEN, "Status: all good");
         }
         else
         {
            errorStatusText.renderText(errorMessage, true);
            ImGui.sameLine();
            if (ImGui.button(labels.get("Confirm")))
            {
               sendErrorConfirmation();
            }
         }
      }
      else
      {
         ImGui.text("No status received.");
      }
   }

   private void sendErrorConfirmation()
   {
      SakeHandDesiredCommandMessage commandMessage = new SakeHandDesiredCommandMessage();
      commandMessage.setDesiredCommandOption((byte) SakeHandCommandOption.CONFIRM_ERROR.getCommandNumber());
      commandMessage.setErrorConfirmation(SakeHandCommandOption.CONFIRM_ERROR.getErrorConfirmation());
      commandMessage.setPositionRatio(SakeHandCommandOption.CONFIRM_ERROR.getGoalPosition());
      commandMessage.setTorqueRatio(SakeHandCommandOption.CONFIRM_ERROR.getGoalTorque());
      commandMessage.setRobotSide(side.toByte());
      communicationHelper.publish(ROS2Tools::getSakeHandCommandTopic, commandMessage);
   }
}
