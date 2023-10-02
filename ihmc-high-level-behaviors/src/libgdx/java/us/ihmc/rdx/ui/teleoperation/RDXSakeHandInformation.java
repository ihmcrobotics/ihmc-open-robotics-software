package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandStatusMessage;
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
   private final IHMCROS2Input<SakeHandStatusMessage> statusInput;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFlashingText calibrateStatusText = new ImGuiFlashingText(ImGuiTools.RED);
   private final ImGuiFlashingText needResetStatusText = new ImGuiFlashingText(ImGuiTools.RED);
   private boolean calibrated = true;
   private boolean needsReset = false;

   public RDXSakeHandInformation(RobotSide side, CommunicationHelper communicationHelper)
   {
      this.side = side;
      this.communicationHelper = communicationHelper;
      statusInput = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                           .withTypeName(SakeHandStatusMessage.class),
                                                  message -> message.getRobotSide() == side.toByte());
   }

   public void update()
   {
      calibrated = statusInput.getLatest().getCalibrated();
      needsReset = statusInput.getLatest().getNeedsReset();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text(side.getPascalCaseName() + ":");
      ImGui.sameLine();
      if (statusInput.hasReceivedFirstMessage())
      {
         ImGui.text("Calibrated:");
         ImGui.sameLine();
         calibrateStatusText.renderText(Boolean.toString(calibrated), !calibrated);
         ImGui.sameLine();
         ImGui.text("Needs reset:");
         ImGui.sameLine();
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
         SakeHandDesiredCommandMessage sakeCommand = new SakeHandDesiredCommandMessage();
         sakeCommand.setRobotSide(side.toByte());
         sakeCommand.setDesiredHandConfiguration(SakeHandDesiredCommandMessage.HAND_CONFIGURATION_RESET);
         communicationHelper.publish(ROS2Tools.getControllerInputTopic(communicationHelper.getRobotName()).withTypeName(SakeHandDesiredCommandMessage.class),
                                     sakeCommand);
      }
   }

   public boolean getCalibrated()
   {
      return calibrated;
   }

   public boolean getNeedsReset()
   {
      return needsReset;
   }
}
