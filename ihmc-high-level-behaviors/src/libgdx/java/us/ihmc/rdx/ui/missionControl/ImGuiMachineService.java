package us.ihmc.rdx.ui.missionControl;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import mission_control_msgs.msg.dds.SystemServiceActionMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Node;

import java.util.List;
import java.util.UUID;

public class ImGuiMachineService
{
   private final String serviceName;
   private final String hostname;
   private final UUID instanceId;
   private String status;
   //   private final ImGuiPanel panel;
   private final ImGuiConsoleArea consoleArea;
   private IHMCROS2Publisher<SystemServiceActionMessage> serviceActionPublisher;

   public ImGuiMachineService(String serviceName, String hostname, UUID instanceId, ROS2Node ros2Node)
   {
      this.serviceName = serviceName;
      this.hostname = hostname;
      this.instanceId = instanceId;
      //      panel = new ImGuiPanel(serviceName + "##" + instanceId, this::renderImGuiWidgets);
      consoleArea = new ImGuiConsoleArea();
      ThreadTools.startAsDaemon(() ->
      {
        serviceActionPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.getSystemServiceActionTopic(instanceId));
      }, "Service-Action-Publisher");
   }

   public String getStatus()
   {
      return status == null ? "Status not available" : status;
   }

   public void setStatus(String status)
   {
      this.status = status;
   }

   public void acceptLogLines(List<String> logLines)
   {
      logLines.forEach(consoleArea::acceptLine);
   }

   private void sendActionMessage(String systemdAction)
   {
      SystemServiceActionMessage message = new SystemServiceActionMessage();
      message.setServiceName(serviceName);
      message.setSystemdAction(systemdAction);
      serviceActionPublisher.publish(message);
   }

   public void sendStartMessage()
   {
      sendActionMessage("start");
   }

   public void sendStopMessage()
   {
      sendActionMessage("stop");
   }

   public void sendRestartMessage()
   {
      sendActionMessage("restart");
   }

   public void renderImGuiWidgets()
   {
      ImGui.text(serviceName);
      ImGui.text(status);
      if (ImGui.button("Start##" + instanceId + "-" + serviceName))
         sendStartMessage();
      ImGui.sameLine();
      // "Grey-out" button if it's mission control and do nothing when pressed
      if (serviceName.equals("mission-control-3"))
      {
         ImGui.beginDisabled(true);
         ImGui.pushStyleColor(ImGuiCol.Button, 0.5f, 0.5f, 0.5f, 1.0f);
         ImGui.button("Stop");
         ImGui.popStyleColor();
         ImGui.endDisabled();
      }
      else
      {
         if (ImGui.button("Stop##" + instanceId + "-" + serviceName))
            sendStopMessage();
      }
      ImGui.sameLine();
      if (ImGui.button("Restart##" + instanceId + "-" + serviceName))
         sendRestartMessage();
   }
}
