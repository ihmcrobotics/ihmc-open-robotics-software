package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.type.ImBoolean;
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

public class ImGuiSSHJSystemdServiceManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiPanel logPanel;
   private final String titleCasedServiceName;
   private final String remoteHostname;
   private final String remoteUsername;
   private final ImBoolean showLogPanel = new ImBoolean(false);
   private final ImGuiConsoleArea consoleArea = new ImGuiConsoleArea();
   private final ImInt bufferSize = new ImInt(Conversions.megabytesToBytes(2));
   private final String serviceName;
   private final double timeout = 0.0;
   private int exitStatus = -1;
   private final SSHJInputStream standardOut = new SSHJInputStream();
   private final SSHJInputStream standardError = new SSHJInputStream();
   private Session.Command sshjCommand;
   private Thread runThread;

   public ImGuiSSHJSystemdServiceManager(String machineName, String titleCasedServiceName, String serviceName, String remoteHostname, String remoteUsername)
   {
      this.logPanel = new ImGuiPanel(machineName + " " + titleCasedServiceName + " Log", consoleArea::renderImGuiWidgets);
      this.titleCasedServiceName = titleCasedServiceName;
      this.serviceName = serviceName;
      this.remoteHostname = remoteHostname;
      this.remoteUsername = remoteUsername;

      standardOut.resize(bufferSize.get());
      standardError.resize(bufferSize.get());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text(titleCasedServiceName + ": ");

      ImGui.sameLine();
      if (!isRunning())
      {
         if (ImGui.button(labels.get("Start")))
         {
            runSystemctlCommand("start");
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Stop")))
         {
            runSystemctlCommand("stop");
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Restart")))
         {
            runSystemctlCommand("restart");
         }
//         ImGui.sameLine();
//         if (ImGui.button(labels.get("Unmask")))
//         {
//            runSystemctlCommand("unmask");
//         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Status")))
         {
            runSystemctlCommand("status");
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Journal")))
         {
            runCommand("sudo journalctl -ef -o cat -u");
         }
      }
      else
      {
         if (ImGui.button("SIGINT"))
         {
            ExceptionTools.handle(() ->
            {
               sshjCommand.signal(Signal.INT);
               sshjCommand.close();
            }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Show Log")))
      {
         logPanel.getIsShowing().set(true);
      }

      standardOut.updateConsoleText(this::acceptNewText);
      standardError.updateConsoleText(this::acceptNewText);
   }

   public void runSystemctlCommand(String verb)
   {
      runCommand("sudo systemctl " + verb);
   }

   private void runCommand(String command)
   {
      if (!isRunning())
      {
         runThread = ThreadTools.startAsDaemon(() ->
         {
            SSHJTools.session(remoteHostname, remoteUsername, sshj ->
            {
               exitStatus = sshj.exec(command + " " + serviceName, timeout, sshjCommand ->
               {
                  this.sshjCommand = sshjCommand;
                  standardOut.setInputStream(sshjCommand.getInputStream(), sshjCommand.getRemoteCharset());
                  standardError.setInputStream(sshjCommand.getErrorStream(), sshjCommand.getRemoteCharset());
               });
            });
         }, "SSHJCommand");
      }
   }

   private boolean isRunning()
   {
      return runThread != null && runThread.isAlive();
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
