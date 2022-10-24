package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import imgui.type.ImString;
import net.schmizz.sshj.connection.channel.direct.Session;
import net.schmizz.sshj.connection.channel.direct.Signal;
import us.ihmc.avatar.ros2.networkTest.SSHJTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;

public class ImGuiSSHJCommand
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiPanel logPanel;
   private final String commandName;
   private final String remoteHostname;
   private final String remoteUsername;
   private final ImBoolean showLogPanel = new ImBoolean(false);
   private final ImGuiConsoleArea consoleArea = new ImGuiConsoleArea();
   private final ImInt bufferSize = new ImInt(Conversions.megabytesToBytes(2));
   private final ImString command = new ImString(10000);
   private final ImDouble timeout = new ImDouble(0.0);
   private int exitStatus = -1;
   private final SSHJInputStream standardOut = new SSHJInputStream();
   private final SSHJInputStream standardError = new SSHJInputStream();
   private Session.Command sshjCommand;
   private Thread runThread;

   public ImGuiSSHJCommand(String commandName, String command, String remoteHostname, String remoteUsername)
   {
      this.logPanel = new ImGuiPanel(commandName + " Log", consoleArea::renderImGuiWidgets);
      this.commandName = commandName;
      this.remoteHostname = remoteHostname;
      this.remoteUsername = remoteUsername;

      standardOut.resize(bufferSize.get());
      standardError.resize(bufferSize.get());
      this.command.set(command);
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Command: " + commandName);

      int inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags |= ImGuiInputTextFlags.CallbackResize;
      ImGui.inputText("Command", command, inputTextFlags);

      ImGui.inputDouble("Timeout", timeout);
      ImGui.sameLine();
      ImGui.text("(0 means no timeout)");

      if (!isRunning() && ImGui.button("Run"))
      {
         runThread = ThreadTools.startAsDaemon(() ->
         {
            SSHJTools.session(remoteHostname, remoteUsername, sshj ->
            {
               exitStatus = sshj.exec(command.get(), timeout.get(), sshjCommand ->
               {
                  this.sshjCommand = sshjCommand;
                  standardOut.setInputStream(sshjCommand.getInputStream(), sshjCommand.getRemoteCharset());
                  standardError.setInputStream(sshjCommand.getErrorStream(), sshjCommand.getRemoteCharset());
               });
            });
         }, "SSHJCommand");
      }
      if (isRunning())
      {
         if (ImGui.button("SIGINT"))
         {
            ExceptionTools.handle(() -> sshjCommand.signal(Signal.INT), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
         ImGui.sameLine();
         if (ImGui.button("Type 'q'"))
         {
            ExceptionTools.handle(() -> sshjCommand.getOutputStream().write('q'), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
      }
      if (exitStatus > -1)
      {
         ImGui.sameLine();
         ImGui.text("Exit status: " + exitStatus);
      }

      if (ImGui.button(labels.get("Show Log")))
      {
         logPanel.getIsShowing().set(true);
      }

      standardOut.updateConsoleText(this::acceptNewText);
      standardError.updateConsoleText(this::acceptNewText);
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

   public ImString getCommand()
   {
      return command;
   }
}
