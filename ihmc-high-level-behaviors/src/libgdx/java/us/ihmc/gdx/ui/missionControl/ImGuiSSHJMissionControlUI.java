package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.missionControl.MissionControlService;

import java.util.concurrent.atomic.AtomicReference;

public class ImGuiSSHJMissionControlUI
{
   private final String REMOTE_HOSTNAME = System.getProperty("remote.hostname");
   private final String REMOTE_USERNAME = System.getProperty("remote.username");

   private final ImGuiSSHJMachine raspberryPi;
   private final ImGuiSSHJApplicationService exampleApplication1Manager;
   private final AtomicReference<String> exampleApplication1Status;
   private final ImGuiSSHJApplicationService exampleApplication2Manager;
   private final AtomicReference<String> exampleApplication2Status;

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
      exampleApplication1Status = raspberryPi.subscribeToServiceStatus("mission-control-application-1");

      exampleApplication2Manager = new ImGuiSSHJApplicationService("Example Application 2",
                                                                   "mission-control-application-2",
                                                                   REMOTE_HOSTNAME,
                                                                   REMOTE_USERNAME);
      imGuiGlfwWindow.getPanelManager().addPanel(exampleApplication2Manager.getLogPanel());
      exampleApplication2Status = raspberryPi.subscribeToServiceStatus("mission-control-application-2");
   }

   private void renderImGuiWidgets()
   {
      raspberryPi.renderImGuiWidgets();

      ImGui.separator();

      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text("Example Application 1");
      ImGui.popFont();
      exampleApplication1Manager.renderImGuiWidgets();
      ImGui.text(exampleApplication1Status.get());

      ImGui.separator();

      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text("Example Application 2");
      ImGui.popFont();
      exampleApplication2Manager.renderImGuiWidgets();
      ImGui.text(exampleApplication2Status.get());
   }

   public static void main(String[] args)
   {
      new ImGuiSSHJMissionControlUI();
   }
}
