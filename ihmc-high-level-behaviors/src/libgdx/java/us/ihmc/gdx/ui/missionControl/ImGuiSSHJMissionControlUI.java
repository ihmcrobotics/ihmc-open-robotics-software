package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;

public class ImGuiSSHJMissionControlUI
{
   private final String REMOTE_HOSTNAME = System.getProperty("remote.hostname");
   private final String REMOTE_USERNAME = System.getProperty("remote.username");

   private final ImGuiSSHJMachine raspberryPi;
//   private final ImGuiSSHJCommand missionControlServiceLogCommand;

   public ImGuiSSHJMissionControlUI()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(),
                                                            "ihmc-open-robotics-software",
                                                            "ihmc-high-level-behaviors/src/libgdx/resources",
                                                            "SSHJ Shell");
      imGuiGlfwWindow.runWithSinglePanel(this::renderImGuiWidgets);

      raspberryPi = new ImGuiSSHJMachine("Raspberry Pi", REMOTE_HOSTNAME, REMOTE_USERNAME);
      imGuiGlfwWindow.getPanelManager().addPanel(raspberryPi.getSystemdServiceManager().getLogPanel());

//      missionControlServiceLogCommand = new ImGuiSSHJCommand("Mission Control Service",
//                                                             "sudo journalctl -ef -u mission-control-2",
//                                                             REMOTE_HOSTNAME,
//                                                             REMOTE_USERNAME);
//
//      imGuiGlfwWindow.getPanelManager().addPanel(missionControlServiceLogCommand.getLogPanel());
   }

   private void renderImGuiWidgets()
   {
      raspberryPi.renderImGuiWidgets();

      ImGui.separator();

//      missionControlServiceLogCommand.renderImGuiWidgets();
   }

   public static void main(String[] args)
   {
      new ImGuiSSHJMissionControlUI();
   }
}
