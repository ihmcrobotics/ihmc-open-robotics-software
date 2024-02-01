package us.ihmc.rdx.ui.yo;

import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.rdx.imgui.RDXPanel;

public class RDXYoVariableClientPanel extends RDXPanel
{
   private final YoVariableClientHelper yoVariableClientHelper;
   private final ImGuiYoVariableClientManagerWidget managerWidget;

   public RDXYoVariableClientPanel(String registryName, String hostname, int port)
   {
      super("YoVariableClient");
      yoVariableClientHelper = new YoVariableClientHelper(registryName);
      managerWidget = new ImGuiYoVariableClientManagerWidget(yoVariableClientHelper, () -> hostname, port);

      // The setRenderMethod is done at the bottom because the other variables need to be initialized first
      setRenderMethod(managerWidget::renderImGuiWidgets);
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
}
