package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;

public class ImGuiSSHJMissionControlUI
{
   private final String REMOTE_HOSTNAME = System.getProperty("remote.hostname");
   private final String REMOTE_USERNAME = System.getProperty("remote.username");

   private final ImGuiSSHJMachine raspberryPi;
   private final ImGuiSSHJApplicationService exampleApplication1Manager;
   private final ImGuiSSHJApplicationService exampleApplication2Manager;

   public ImGuiSSHJMissionControlUI()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(),
                                                            "ihmc-open-robotics-software",
                                                            "ihmc-high-level-behaviors/src/libgdx/resources",
                                                            "Mission Control 2");
      imGuiGlfwWindow.runWithSinglePanel(this::renderImGuiWidgets);

      raspberryPi = new ImGuiSSHJMachine("Raspberry Pi", REMOTE_HOSTNAME, REMOTE_USERNAME);
      imGuiGlfwWindow.getPanelManager().addPanel(raspberryPi.getSystemdServiceManager().getLogPanel());

      exampleApplication1Manager = new ImGuiSSHJApplicationService("Example Application 1",
                                                                   "mission-control-application-1",
                                                                   REMOTE_HOSTNAME,
                                                                   REMOTE_USERNAME);
      imGuiGlfwWindow.getPanelManager().addPanel(exampleApplication1Manager.getLogPanel());
      exampleApplication1Manager.setServiceStatus(raspberryPi.subscribeToServiceStatus("mission-control-application-1"));

      exampleApplication2Manager = new ImGuiSSHJApplicationService("Example Application 2",
                                                                   "mission-control-application-2",
                                                                   REMOTE_HOSTNAME,
                                                                   REMOTE_USERNAME);
      imGuiGlfwWindow.getPanelManager().addPanel(exampleApplication2Manager.getLogPanel());
      exampleApplication2Manager.setServiceStatus(raspberryPi.subscribeToServiceStatus("mission-control-application-2"));
   }

   private void renderImGuiWidgets()
   {
      raspberryPi.renderImGuiWidgets(false);
      ImGui.separator();
      exampleApplication1Manager.renderImGuiWidgets();
      ImGui.separator();
      exampleApplication2Manager.renderImGuiWidgets();
   }

   public static void main(String[] args)
   {
      new ImGuiSSHJMissionControlUI();
   }
}
