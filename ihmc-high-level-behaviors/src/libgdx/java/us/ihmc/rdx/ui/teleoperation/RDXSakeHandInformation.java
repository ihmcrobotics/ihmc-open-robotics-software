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
   private final ImGuiFlashingText presentTemperatureText = new ImGuiFlashingText(ImGuiTools.RED);
   private final ImGuiFlashingText errorStatusText = new ImGuiFlashingText(ImGuiTools.RED);

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
      if (statusInput.hasReceivedFirstMessage())
      {
         // TODO: Add ratio to unit conversions enum or something
         double temperature = 100 * statusInput.getLatest().getTemperature();
         boolean error = statusInput.getLatest().getIsInErrorState();

         if (temperature < 55.0)
         {
            renderGradiatedImGuiText("Temperature: " + FormattingTools.getFormattedDecimal1D(temperature) + " C  |", temperature, 45.0, 55.0);
         }
         else
         {
            presentTemperatureText.renderText("Temperature: " + FormattingTools.getFormattedDecimal1D(temperature) + " C  |", true);
         }
         ImGui.sameLine();

         if (!error)
         {
            ImGui.textColored(ImGuiTools.GREEN, "Status: all good");
         }
         else
         {
            errorStatusText.renderText("Status: ERROR", true);
         }
      }
      else
      {
         ImGui.text("No status received.");
      }
   }

   private void renderGradiatedImGuiText(String text, double value, double... colorSwitchValues)
   {
      int redValue = 0;
      int greenValue = 255;

      for (double switchValue : colorSwitchValues)
      {
         if (value < switchValue)
            break;

         redValue = 255;
         greenValue -= 255 / colorSwitchValues.length;
      }

      ImGui.textColored(redValue, greenValue, 0, 255, text);
   }
}
