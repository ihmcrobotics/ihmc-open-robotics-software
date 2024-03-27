package us.ihmc.rdx.ui.missionControl;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import mission_control_msgs.msg.dds.SystemServiceActionMessage;
import mission_control_msgs.msg.dds.SystemServiceLogRefreshMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
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
   private final RDXPanel logPanel;
   private ROS2PublisherBasics<SystemServiceLogRefreshMessage> logRefreshPublisher;
   private ROS2PublisherBasics<SystemServiceActionMessage> serviceActionPublisher;
   private final ImGuiConsoleArea consoleArea;

   /**
    * Time of the last action button press in milliseconds
    */
   private long lastActionRequestTimeMs = -1;
   /**
    * If we've clicked an action button, i.e. "Start" and we're expecting a change sometime in the near future.
    * This disables the "start" "stop", "kill", etc. buttons for a few seconds after you've pressed them,
    * so you know something happened and so you can't spam them
    */
   private boolean waitingOnStatusChange = false;

   public ImGuiMachineService(String serviceName, String hostname, UUID instanceId, RDXPanel machinePanel, ROS2Node ros2Node)
   {
      this.serviceName = serviceName;
      this.hostname = hostname;
      this.instanceId = instanceId;
      logPanel = new RDXPanel(serviceName + " Log##" + instanceId, this::renderImGuiLogPanelWidgets);
      machinePanel.queueAddChild(logPanel);

      ThreadTools.startAsDaemon(() ->
      {
         logRefreshPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.getSystemServiceLogRefreshTopic(instanceId));
      }, "Log-Refresh-Publisher-Thread");
      ThreadTools.startAsDaemon(() ->
      {
         serviceActionPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.getSystemServiceActionTopic(instanceId));
      }, "Service-Action-Publisher-Thread");

      // Request a refresh after 1 second
      ThreadTools.scheduleSingleExecution("Refresh", this::requestRefresh, 1, TimeUnit.SECONDS);

      consoleArea = new ImGuiConsoleArea();
   }

   public void requestRefresh()
   {
      // Request full log data
      SystemServiceLogRefreshMessage logRefreshMessage = new SystemServiceLogRefreshMessage();
      logRefreshMessage.setServiceName(serviceName);
      logRefreshPublisher.publish(logRefreshMessage);
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
            waitingOnStatusChange = false;
      }
      this.status = status;
   }

   public void acceptLogLines(List<String> logLines, boolean buffer)
   {
      logLines.forEach(line -> consoleArea.acceptLine(line, buffer));
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

   public void sendKillMessage()
   {
      sendActionMessage("kill");
   }

   private boolean hasItBeenAWhileSinceTheLastActionRequest()
   {
      return (System.currentTimeMillis() - lastActionRequestTimeMs) > TimeUnit.SECONDS.toMillis(5);
   }

   public void renderImGuiWidgets()
   {
      if (status == null)
         return;

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text(serviceName);
      ImGui.popFont();
      if (status.startsWith("failed"))
      {
         ImGuiTools.textColored(Color.SCARLET, status);
      }
      else if (status.startsWith("active"))
      {
         ImGuiTools.textColored(Color.LIME, status);
      }
      else
      {
         ImGui.text(status);
      }

      boolean isMissionControl3 = serviceName.contains("mission-control-3");
      boolean allButtonsDisabled = waitingOnStatusChange && !hasItBeenAWhileSinceTheLastActionRequest() || isMissionControl3;

      if (allButtonsDisabled)
         ImGui.beginDisabled(true);

      { // Start button
         boolean disabled = status.startsWith("active");
         if (disabled)
            ImGui.beginDisabled(true);
         if (ImGui.button("Start##" + instanceId + "-" + serviceName))
         {
            sendStartMessage();
            waitingOnStatusChange = true;
            lastActionRequestTimeMs = System.currentTimeMillis();
         }
         if (disabled)
            ImGui.endDisabled();
      }
      ImGui.sameLine();
      { // Stop button
         boolean disabled = status.startsWith("inactive") || status.startsWith("failed") || isMissionControl3;
         if (disabled)
            ImGui.beginDisabled(true);
         if (ImGui.button("Stop##" + instanceId + "-" + serviceName))
         {
            sendStopMessage();
            waitingOnStatusChange = true;
            lastActionRequestTimeMs = System.currentTimeMillis();
         }
         if (disabled)
            ImGui.endDisabled();
      }
      ImGui.sameLine();
      { // Restart button
         if (ImGui.button("Restart##" + instanceId + "-" + serviceName))
         {
            sendRestartMessage();
            waitingOnStatusChange = true;
            lastActionRequestTimeMs = System.currentTimeMillis();
         }
      }
      ImGui.sameLine();
      { // Kill button
         if (ImGui.button("Kill##" + instanceId + "-" + serviceName))
         {
            sendKillMessage();
            waitingOnStatusChange = true;
            lastActionRequestTimeMs = System.currentTimeMillis();
         }
      }
      ImGui.sameLine();
      { // Log button
         if (ImGui.button("Open log##" + instanceId + "-" + serviceName))
         {
            openLogPanel();
         }
      }

      if (allButtonsDisabled)
         ImGui.endDisabled();
   }

   public void openLogPanel()
   {
      logPanel.getIsShowing().set(true);
      ImGui.setWindowFocus(logPanel.getPanelName());
      imgui.internal.ImGui.dockBuilderDockWindow(logPanel.getPanelName(), 1);
   }

   public void renderImGuiLogPanelWidgets()
   {
      consoleArea.renderImGuiWidgets();
   }
}
