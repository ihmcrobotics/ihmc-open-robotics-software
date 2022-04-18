package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.type.ImInt;
import net.schmizz.sshj.connection.channel.direct.Session;
import net.schmizz.sshj.connection.channel.direct.Signal;
import us.ihmc.avatar.ros2.networkTest.SSHJTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;

public class ImGuiSSHJApplicationService
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiPanel logPanel;
   private final String remoteHostname;
   private final String remoteUsername;
   private final ImGuiConsoleArea consoleArea = new ImGuiConsoleArea();
   private final ImInt bufferSize = new ImInt(Conversions.megabytesToBytes(2));
   private final String serviceName;
   private final double timeout = 0.0;
   private int exitStatus = -1;
   private final SSHJInputStream standardOut = new SSHJInputStream();
   private final SSHJInputStream standardError = new SSHJInputStream();
   private Session.Command managementSSHJCommand;
   private Session.Command logMonitorSSHJCommand;
   private Thread managementRunThread;
   private Thread logMonitorRunThread;

   public ImGuiSSHJApplicationService(String applicationName, String serviceName, String remoteHostname, String remoteUsername)
   {
      this.logPanel = new ImGuiPanel(applicationName + " Log", consoleArea::renderImGuiWidgets);
      this.serviceName = serviceName;
      this.remoteHostname = remoteHostname;
      this.remoteUsername = remoteUsername;

      standardOut.resize(bufferSize.get());
      standardError.resize(bufferSize.get());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Service name: " + serviceName);

      ImGui.sameLine();
      if (ImGui.button(labels.get("Start")))
      {
         runCommand("start");
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop")))
      {
         runCommand("stop");
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Restart")))
      {
         runCommand("restart");
      }

      if (!islogMonitorThreadRunning() && ImGui.button("Start Log Monitor"))
      {
         logMonitorRunThread = ThreadTools.startAsDaemon(() ->
         {
            SSHJTools.session(remoteHostname, remoteUsername, sshj ->
            {
               exitStatus = sshj.exec("sudo journalctl -ef -o cat -u " + serviceName, timeout, sshjCommand ->
               {
                  this.logMonitorSSHJCommand = sshjCommand;
                  standardOut.setInputStream(sshjCommand.getInputStream(), sshjCommand.getRemoteCharset());
                  standardError.setInputStream(sshjCommand.getErrorStream(), sshjCommand.getRemoteCharset());
               });
            });
         }, "SSHJCommand");
      }
      if (islogMonitorThreadRunning())
      {
         if (ImGui.button("SIGINT"))
         {
            ExceptionTools.handle(() -> logMonitorSSHJCommand.signal(Signal.INT), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
      }
      if (exitStatus > -1)
      {
         ImGui.sameLine();
         ImGui.text("Exit status: " + exitStatus);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Show Log")))
      {
         logPanel.getIsShowing().set(true);
      }

      standardOut.updateConsoleText(this::acceptNewText);
      standardError.updateConsoleText(this::acceptNewText);
   }

   private void runCommand(String verb)
   {
      if (!isManagementThreadRunning())
      {
         managementRunThread = ThreadTools.startAsDaemon(() ->
         {
            SSHJTools.session(remoteHostname, remoteUsername, sshj ->
            {
               exitStatus = sshj.exec("sudo systemctl " + verb + " " + serviceName, timeout, sshjCommand ->
               {
                  this.managementSSHJCommand = sshjCommand;
               });
            });
         }, "SSHJCommand");
      }
   }

   private boolean isManagementThreadRunning()
   {
      return managementRunThread != null && managementRunThread.isAlive();
   }

   private boolean islogMonitorThreadRunning()
   {
      return logMonitorRunThread != null && logMonitorRunThread.isAlive();
   }

   private void acceptNewText(String newText)
   {
      consoleArea.acceptNewText(newText);
//      System.out.print(newText);
   }

   public ImGuiPanel getLogPanel()
   {
      return logPanel;
   }
}
