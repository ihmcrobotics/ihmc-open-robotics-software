package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.extension.texteditor.TextEditor;
import imgui.extension.texteditor.TextEditorLanguageDefinition;
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

import java.util.HashMap;

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
   private final TextEditor textEditor = new TextEditor();
   private TextEditorLanguageDefinition textEditorLanguageDefinition = new TextEditorLanguageDefinition();
   private Thread runThread;

   public ImGuiSSHJShellUI()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(),
                                                            "ihmc-open-robotics-software",
                                                            "ihmc-high-level-behaviors/src/libgdx/resources",
                                                            "SSHJ Shell");
      imGuiGlfwWindow.runWithSinglePanel(this::renderImGuiWidgets);

      standardOut.resize(bufferSize.get());
      standardError.resize(bufferSize.get());
      command.set("sudo journalctl -ef -u sshd");

      int[] palette = new int[]
            {
                  0xffffffff,	// Default
                  0xffd69c56,	// Keyword
                  0xff00ff00,	// Number
                  0xffffffff,	// String
                  0xffffffff, // Char literal
                  0xffffffff, // Punctuation
                  0xff408080,	// Preprocessor
                  0xffaaaaaa, // Identifier
                  0xff9bc64d, // Known identifier
                  0xffc040a0, // Preproc identifier
                  0xff206020, // Comment (single line)
                  0xffffffff, // Comment (multi line)
                  0xff101010, // Background
                  0xffe0e0e0, // Cursor
                  0x80a06020, // Selection
                  0x800020ff, // ErrorMarker
                  0x40f08000, // Breakpoint
                  0xff707000, // Line number
                  0x40000000, // Current line fill
                  0x40808080, // Current line fill (inactive)
                  0x40a0a0a0, // Current line edge
            };
      textEditor.setPalette(palette);


      HashMap<String, String> identifiers = new HashMap<>();
      textEditorLanguageDefinition = new TextEditorLanguageDefinition();
      textEditorLanguageDefinition.setIdentifiers(identifiers);
      textEditor.setLanguageDefinition(textEditorLanguageDefinition);
      textEditor.setColorizerEnable(true);

      textEditor.setReadOnly(true);
      textEditor.setShowWhitespaces(false);
      textEditor.setHandleMouseInputs(true);
//      textEditor.
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

      if (ImGui.button("Auto scroll"))
      {
         int lastLineIndex = textEditor.getTotalLines() - 1;
         int endOfLastLineColumn = textEditor.getTextLines()[lastLineIndex].length();
         textEditor.setCursorPosition(lastLineIndex, endOfLastLineColumn);
      }

      if (!isRunning() && ImGui.button("Run"))
      {
         runThread = ThreadTools.startAsDaemon(() ->
         {
            SSHJTools.session(REMOTE_HOSTNAME, REMOTE_USERNAME, sshj ->
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

      standardOut.updateConsoleText(this::acceptNewText);
      standardError.updateConsoleText(this::acceptNewText);

      ImGui.pushFont(ImGuiTools.getConsoleFont());
      textEditor.render("Console");
      ImGui.popFont();
   }

   private boolean isRunning()
   {
      return runThread != null && runThread.isAlive();
   }

   private void acceptNewText(String newText)
   {
      consoleText.set(consoleText.get() + newText);

      int previousCursorLine = textEditor.getCursorPositionLine();
      int previousCursorColumn = textEditor.getCursorPositionColumn();
      int lastLineIndex = textEditor.getTotalLines() - 1;
      int endOfLastLineColumn = textEditor.getTextLines()[lastLineIndex].length();
      boolean isAutoScroll = previousCursorLine == lastLineIndex && previousCursorColumn == endOfLastLineColumn;
      textEditor.setCursorPosition(lastLineIndex, endOfLastLineColumn);
      textEditor.setReadOnly(false);
      textEditor.insertText(newText);
      textEditor.setReadOnly(true);
      if (!isAutoScroll)
         textEditor.setCursorPosition(previousCursorLine, previousCursorColumn);

      System.out.print(newText);
   }

   public static void main(String[] args)
   {
      new ImGuiSSHJShellUI();
   }
}
