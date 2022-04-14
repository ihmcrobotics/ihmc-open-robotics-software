package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.avatar.ros2.networkTest.SSHJTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.io.*;

public class ImGuiSSHJShellUI
{
   private final String REMOTE_HOSTNAME = System.getProperty("remote.hostname");
   private final String REMOTE_USERNAME = System.getProperty("remote.username");
   private final ImInt bufferSize = new ImInt(10000);
   private final ImString consoleText = new ImString(bufferSize.get());
   private final ImString command = new ImString(bufferSize.get());
   private final ImDouble timeout = new ImDouble(0.0);
   private InputStream standardOutRaw;
   private InputStream standardError;
   private int exitStatus = -1;

   public ImGuiSSHJShellUI()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(),
                                                            "ihmc-open-robotics-software",
                                                            "ihmc-high-level-behaviors/src/libgdx/resources",
                                                            "SSHJ Shell");
      imGuiGlfwWindow.runWithSinglePanel(this::renderImGuiWidgets);

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

      if (ImGui.button("Run"))
      {
         ThreadTools.startAsDaemon(() ->
         {
            SSHJTools.session(REMOTE_HOSTNAME, REMOTE_USERNAME, sshj ->
            {
               exitStatus = sshj.exec(command.get(), timeout.get(), sshjCommand ->
               {
                  standardOutRaw = sshjCommand.getInputStream();
                  standardError = sshjCommand.getErrorStream();
               });
            });
         }, "SSHJCommand");
      }
      if (exitStatus > -1)
      {
         ImGui.sameLine();
         ImGui.text("Exit status: " + exitStatus);
      }

      readFromInputStream(standardOutRaw);
      readFromInputStream(standardError);

      ImGui.beginChild("###consoleArea");

      inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags |= ImGuiInputTextFlags.CallbackResize;
      inputTextFlags |= ImGuiInputTextFlags.ReadOnly;
      ImGui.pushFont(ImGuiTools.getConsoleFont());
      ImGui.inputTextMultiline("###console", consoleText, ImGui.getColumnWidth(), ImGui.getContentRegionAvailY(), inputTextFlags);
      ImGui.popFont();

      ImGui.endChild();
   }

   private void readFromInputStream(InputStream inputStream)
   {
      if (inputStream != null)
      {
         int availableBytes = ExceptionTools.handle(inputStream::available, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         for (int i = 0; i < availableBytes; i++)
         {
            int read = ExceptionTools.handle(() -> inputStream.read(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

            if (read > 0)
            {
               consoleText.set(consoleText.get() + (char) read);
            }
         }
      }
   }

   public static void main(String[] args)
   {
      new ImGuiSSHJShellUI();
   }
}
