package us.ihmc.gdx.ui.yo;

import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.gdx.imgui.ImGuiPanel;

public class ImGuiYoVariableClientUI
{
   private final ImGuiPanel panel;
   private final YoVariableClientHelper yoVariableClientHelper;
   private final ImGuiYoVariableClientManagerWidget managerWidget;

   public ImGuiYoVariableClientUI(String registryName, String hostname, int port)
   {
      yoVariableClientHelper = new YoVariableClientHelper(registryName);
      managerWidget = new ImGuiYoVariableClientManagerWidget(yoVariableClientHelper, () -> hostname, port);
      panel = new ImGuiPanel(registryName + " YoVariable Client", managerWidget::renderImGuiWidgets);
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

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
