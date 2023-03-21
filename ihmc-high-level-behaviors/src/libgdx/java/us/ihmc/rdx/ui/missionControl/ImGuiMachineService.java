package us.ihmc.rdx.ui.missionControl;

import us.ihmc.rdx.imgui.ImGuiPanel;

import java.util.List;
import java.util.UUID;

public class ImGuiMachineService
{
   private final String serviceName;
   private String status;
   private final ImGuiPanel panel;
   private final ImGuiConsoleArea consoleArea;

   public ImGuiMachineService(String serviceName, String hostname, UUID instanceId, ImGuiPanel machinePanel)
   {
      this.serviceName = serviceName;
      panel = new ImGuiPanel(serviceName + "##" + instanceId, this::renderImGuiWidgets);
      consoleArea = new ImGuiConsoleArea();

      machinePanel.addChild(panel);
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

   public void renderImGuiWidgets()
   {
      consoleArea.renderImGuiWidgets();
   }
}
