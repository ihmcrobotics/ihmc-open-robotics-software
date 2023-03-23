package us.ihmc.rdx.ui.missionControl;

import imgui.ImGui;
import mission_control_msgs.msg.dds.SystemServiceActionMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.ros2.ROS2Node;

import javax.annotation.Nullable;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.TimeUnit;

public class ImGuiMachineService
{
   private final String serviceName;
   private final String hostname;
   private final UUID instanceId;
   @Nullable
   private String status;
   private final ImGuiPanel logPanel;
   private final ImGuiConsoleArea consoleArea;
   private IHMCROS2Publisher<SystemServiceActionMessage> serviceActionPublisher;

   private long lastActionRequest = -1;
   private boolean waitingOnChange = false;

   public ImGuiMachineService(String serviceName, String hostname, UUID instanceId, ROS2Node ros2Node)
   {
      this.serviceName = serviceName;
      this.hostname = hostname;
      this.instanceId = instanceId;
      logPanel = new ImGuiPanel(serviceName + " Log##" + instanceId, this::renderImGuiLogPanelWidgets);
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
      if (status != null && status.startsWith("Active: "))
      {
         status = status.substring(8); // Remove the "Active: "
         if (!status.equals(this.status))
            waitingOnChange = false;
         this.status = status;
      }
      else
      {
         this.status = status;
      }
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

   private boolean hasItBeenAWhileSinceTheLastActionRequest()
   {
      return (System.currentTimeMillis() - lastActionRequest) > TimeUnit.SECONDS.toMillis(5);
   }

   public void renderImGuiWidgets()
   {
      String statusString = status.toString();

      ImGui.text(serviceName);
      ImGui.text(statusString);

      boolean allButtonsDisabled = false;

      if (waitingOnChange && !hasItBeenAWhileSinceTheLastActionRequest())
      {
         allButtonsDisabled = true;
      }

      if (allButtonsDisabled)
         ImGui.beginDisabled(true);

      // Start button
      {
         boolean disabled = statusString.startsWith("active");
         if (disabled)
            ImGui.beginDisabled(true);
         if (ImGui.button("Start##" + instanceId + "-" + serviceName))
         {
            sendStartMessage();
            waitingOnChange = true;
            lastActionRequest = System.currentTimeMillis();
         }
         if (disabled)
            ImGui.endDisabled();
      }
      ImGui.sameLine();
      // Stop button
      {
         boolean disabled = statusString.startsWith("inactive") || statusString.startsWith("failed") || serviceName.contains("mission-control-3");
         if (disabled)
            ImGui.beginDisabled(true);
         if (ImGui.button("Stop##" + instanceId + "-" + serviceName))
         {
            sendStopMessage();
            waitingOnChange = true;
            lastActionRequest = System.currentTimeMillis();
         }
         if (disabled)
            ImGui.endDisabled();
      }
      ImGui.sameLine();
      // Restart button
      {
         if (ImGui.button("Restart##" + instanceId + "-" + serviceName))
         {
            sendRestartMessage();
            waitingOnChange = true;
            lastActionRequest = System.currentTimeMillis();
         }
      }

      if (allButtonsDisabled)
         ImGui.endDisabled();
   }

   public void renderImGuiLogPanelWidgets()
   {
      consoleArea.renderImGuiWidgets();
   }
}
