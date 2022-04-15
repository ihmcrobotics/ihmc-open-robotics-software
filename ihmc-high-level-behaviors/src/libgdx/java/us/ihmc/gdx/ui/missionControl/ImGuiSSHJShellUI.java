package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
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
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.imgui.ImGuiTools;

public class ImGuiSSHJShellUI
{
   private final String REMOTE_HOSTNAME = System.getProperty("remote.hostname");
   private final String REMOTE_USERNAME = System.getProperty("remote.username");
   private final ImInt bufferSize = new ImInt(Conversions.megabytesToBytes(2));
   private final ImString consoleText = new ImString(bufferSize.get());
   private final ImString command = new ImString(bufferSize.get());
   private final ImDouble timeout = new ImDouble(0.0);
   private int exitStatus = -1;
   private final SSHJInputStream standardOut = new SSHJInputStream();
   private final SSHJInputStream standardError = new SSHJInputStream();
   private Session.Command sshjCommand;
   private boolean running = false;

   public ImGuiSSHJShellUI()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(),
                                                            "ihmc-open-robotics-software",
                                                            "ihmc-high-level-behaviors/src/libgdx/resources",
                                                            "SSHJ Shell");
      imGuiGlfwWindow.runWithSinglePanel(this::renderImGuiWidgets);

      standardOut.resize(bufferSize.get());
      standardError.resize(bufferSize.get());
      command.set("ping -c 5 archlinux.org");
   }

   private void renderImGuiWidgets()
   {
      int inputIntFlags = ImGuiInputTextFlags.None;
      inputIntFlags = ImGuiInputTextFlags.EnterReturnsTrue;
      if (ImGui.inputInt("Buffer size", bufferSize, 1, 100, inputIntFlags))
      {
         if (bufferSize.get() > consoleText.getBufferSize())
         {
            consoleText.resize(bufferSize.get());
            standardOut.resize(bufferSize.get());
            standardError.resize(bufferSize.get());
         }
         else
         {
            bufferSize.set(consoleText.getBufferSize());
         }
      }

      int inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags |= ImGuiInputTextFlags.CallbackResize;
      ImGui.inputText("Command", command, inputTextFlags);

      ImGui.inputDouble("Timeout", timeout);
      ImGui.sameLine();
      ImGui.text("(0 means no timeout)");

      if (!running && ImGui.button("Run"))
      {
         ThreadTools.startAsDaemon(() ->
         {
            SSHJTools.session(REMOTE_HOSTNAME, REMOTE_USERNAME, sshj ->
            {
               running = true;
               exitStatus = sshj.exec(command.get(), timeout.get(), sshjCommand ->
               {
                  this.sshjCommand = sshjCommand;
                  standardOut.setInputStream(sshjCommand.getInputStream(), sshjCommand.getRemoteCharset());
                  standardError.setInputStream(sshjCommand.getErrorStream(), sshjCommand.getRemoteCharset());
               });
               running = false;
            });
         }, "SSHJCommand");
      }
      if (running)
      {
         if (ImGui.button("SIGINT"))
         {
            ExceptionTools.handle(() -> sshjCommand.signal(Signal.INT), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
      }
      if (exitStatus > -1)
      {
         ImGui.sameLine();
         ImGui.text("Exit status: " + exitStatus);
      }

      standardOut.updateConsoleText(consoleText);
      standardError.updateConsoleText(consoleText);

//      ImGui.beginChild("###consoleArea");

      inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags |= ImGuiInputTextFlags.CallbackResize;
      inputTextFlags |= ImGuiInputTextFlags.ReadOnly;
      ImGui.pushFont(ImGuiTools.getConsoleFont());
      int id = ImGui.getID("console");
      ImGui.inputTextMultiline("console", consoleText, ImGui.getColumnWidth(), ImGui.getContentRegionAvailY(), inputTextFlags);
//      ImGui.beginChild(id);
//      ImGui.setScrollHereY();
//      ImGui.endChild();
      ImGui.popFont();

//      ImGui.endChild();
   }

   public static void main(String[] args)
   {
      new ImGuiSSHJShellUI();
   }
}
