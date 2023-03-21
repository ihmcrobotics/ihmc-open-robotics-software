package us.ihmc.rdx.ui.missionControl;

import us.ihmc.rdx.imgui.ImGuiPanel;

import java.util.List;
import java.util.UUID;

public class ImGuiMachineService
{
   private final String serviceName;
   private String status;
   private final ImGuiPanel panel;

   public ImGuiMachineService(String serviceName, UUID instanceId)
   {
      this.serviceName = serviceName;
      panel = new ImGuiPanel(serviceName + "##" + instanceId);
   }

   public String getServiceName()
   {
      return serviceName;
   }

   public String getStatus()
   {
      return status == null ? "Status not available" : status;
   }

   public void setStatus(String status)
   {
      this.status = status;
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }

   public void acceptLogLines(List<String> logLines)
   {
   }

   public void renderImGuiWidgets()
   {

   }
}
