package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.HandSakeDesiredCommandMessage;
import controller_msgs.msg.dds.HandSakeStatusMessage;
import imgui.ImGui;
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
   private final IHMCROS2Input<HandSakeStatusMessage> statusInput;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFlashingText calibrateStatusText = new ImGuiFlashingText(ImGuiTools.RED);
   private final ImGuiFlashingText needResetStatusText = new ImGuiFlashingText(ImGuiTools.RED);

   public RDXSakeHandInformation(RobotSide side, CommunicationHelper communicationHelper)
   {
      this.side = side;
      this.communicationHelper = communicationHelper;
      statusInput = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                           .withTypeName(HandSakeStatusMessage.class),
                                                  message -> message.getRobotSide() == side.toByte());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text(side.getPascalCaseName() + ":");
      ImGui.sameLine();
      if (statusInput.hasReceivedFirstMessage())
      {
         ImGui.text("Calibrated:");
         ImGui.sameLine();
         boolean calibrated = statusInput.getLatest().getCalibrated();
         calibrateStatusText.renderText(Boolean.toString(calibrated), !calibrated);
         ImGui.sameLine();
         ImGui.text("Needs reset:");
         ImGui.sameLine();
         boolean needsReset = statusInput.getLatest().getNeedsReset();
         needResetStatusText.renderText(Boolean.toString(needsReset), needsReset);
         ImGui.sameLine();
         ImGui.text("Temperature: " + FormattingTools.getFormattedDecimal1D(statusInput.getLatest().getTemperature()));
      }
      else
      {
         ImGui.text("No status received.");
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Reset", side.getCamelCaseName())))
      {
         HandSakeDesiredCommandMessage sakeCommand = new HandSakeDesiredCommandMessage();
         sakeCommand.setRobotSide(side.toByte());
         sakeCommand.setDesiredHandConfiguration(HandSakeDesiredCommandMessage.HAND_CONFIGURATION_RESET);
         communicationHelper.publish(ROS2Tools.getControllerInputTopic(communicationHelper.getRobotName()).withTypeName(HandSakeDesiredCommandMessage.class),
                                     sakeCommand);
      }
   }
}
