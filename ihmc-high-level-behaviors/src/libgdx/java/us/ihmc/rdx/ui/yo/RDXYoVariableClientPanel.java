package us.ihmc.rdx.ui.yo;

import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.rdx.imgui.RDXPanel;

public class RDXYoVariableClientPanel
{
   private final RDXPanel panel;
   private final YoVariableClientHelper yoVariableClientHelper;
   private final ImGuiYoVariableClientManagerWidget managerWidget;

   public RDXYoVariableClientPanel(String registryName, String hostname, int port)
   {
      yoVariableClientHelper = new YoVariableClientHelper(registryName);
      managerWidget = new ImGuiYoVariableClientManagerWidget(yoVariableClientHelper, () -> hostname, port);
      panel = new RDXPanel(registryName + " YoVariable Client", managerWidget::renderImGuiWidgets);
   }

   public YoVariableClientHelper getYoVariableClientHelper()
   {
      return yoVariableClientHelper;
   }

   public ImGuiYoVariableClientManagerWidget getManagerWidget()
   {
      return managerWidget;
   }

   public void destroy()
   {
      managerWidget.destroy();
   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}
