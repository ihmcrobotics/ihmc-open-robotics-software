package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImInt;
import imgui.type.ImString;
import std_msgs.Char;
import us.ihmc.avatar.ros2.networkTest.SSHJTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlot;
import us.ihmc.gdx.ui.yo.ImPlotPlotLine;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.util.ArrayList;

public class ImGuiSSHJShellUI
{
   private final String REMOTE_HOSTNAME = System.getProperty("remote.hostname");
   private final String REMOTE_USERNAME = System.getProperty("remote.username");
   private final ImInt bufferSize = new ImInt(10000);
   private final ImString consoleText = new ImString(bufferSize.get());
   private final ByteBuffer consoleByteBuffer;
   private final CharBuffer consoleCharBuffer;
   private final StringBuilder consoleStringBuilder = new StringBuilder(bufferSize.get());
   private final ResettableExceptionHandlingExecutorService readerService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private BufferedInputStream stdinStream;
   private InputStream standardError;
   private BufferedReader standardOut;
   private ArrayList<String> lines = new ArrayList<>();
   boolean consoleChanged;
   private ImPlotPlot readLinePlot = new ImPlotPlot();
   private ImPlotDoublePlotLine readLinePlotLine = new ImPlotDoublePlotLine("readLine");
   private Stopwatch stopwatch = new Stopwatch();
   private InputStream standardOutRaw;

   public ImGuiSSHJShellUI()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(),
                                                            "ihmc-open-robotics-software",
                                                            "ihmc-high-level-behaviors/src/libgdx/resources",
                                                            "SSHJ Shell");
      imGuiGlfwWindow.runWithSinglePanel(this::renderImGuiWidgets);

//      consoleByteBuffer = ByteBuffer.wrap(consoleText.getData());
      consoleByteBuffer = ByteBuffer.allocate(bufferSize.get());
      consoleCharBuffer = consoleByteBuffer.asCharBuffer();

      readLinePlot.getPlotLines().add(readLinePlotLine);
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.inputInt("Buffer size", bufferSize))
      {
         consoleText.resize(bufferSize.get());
      }

      if (ImGui.button("Run"))
      {
         ThreadTools.startAsDaemon(() ->
         {
            SSHJTools.session(REMOTE_HOSTNAME, REMOTE_USERNAME, sshj ->
            {
               double noTimeout = 0.0;
               int exitStatus = sshj.exec("ping -c 3 archlinux.org", noTimeout, sshjCommand ->
               {
//                  outputStream = new BufferedOutputStream(sshjCommand.getOutputStream());
                  standardOutRaw = sshjCommand.getInputStream();
//                  inputStream.
//                  standardOut = new BufferedReader(new InputStreamReader(inputStream, sshjCommand.getRemoteCharset()));
//                                                                               inputStream = sshjCommand.getInputStream();
//                  in.
                  standardError = sshjCommand.getErrorStream();

//                  sshjCommand.
               });
//               standardOutRaw = null;
//               standardError = null;
            });
         }, "SSHJCommand");
      }


      consoleChanged = false;
      if (standardOut != null)
      {
//         boolean anyWereRead = false;
//         int read;
//         while ((read = ExceptionTools.handle(() -> standardOut.read(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE)) > -1)
//         {
//            consoleCharBuffer.put((char) read);
//            anyWereRead = true;
//         }
//
//         if (anyWereRead)
//         {
////            consoleStringBuilder.
//            consoleText.set(consoleCharBuffer.toString());
//         }


//         standardOut.readLine()

//         consoleStringBuilder.
//         new String(consoleCharBuffer)
//         consoleText.inputData.
//
//         standardOut.
//
//         consoleText.getData()
//
//

//         readerService.clearQueueAndExecute(() ->
//         {
//            stopwatch.start();
//            ExceptionTools.handle(() ->
//            {
//               String line;
//               while ((line = standardOut.readLine()) != null)
//               {
//                  lines.add(line);
//                  consoleChanged = true;
//               }
//
//            }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
//            readLinePlotLine.addValue(stopwatch.totalElapsed());
//
//            if (consoleChanged)
//            {
//               for (String line : lines)
//               {
//                  consoleStringBuilder.append(line);
//                  consoleStringBuilder.append('\n');
//               }
//               lines.clear();
//            }
//            consoleText.set(consoleStringBuilder.toString());
//         });
////         inputStream.



      }
      if (standardError != null)
      {

      }

      if (standardOutRaw != null)
      {
         int availableBytes = ExceptionTools.handle(standardOutRaw::available, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         for (int i = 0; i < availableBytes; i++)
         {
            int read = ExceptionTools.handle(() -> standardOutRaw.read(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

            if (read > 0)
            {
               consoleText.set(consoleText.get() + (char) read);
            }
         }
      }



      readLinePlot.render();

      ImGui.beginChild("###consoleArea");

      int inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags |= ImGuiInputTextFlags.CallbackResize;
      inputTextFlags |= ImGuiInputTextFlags.ReadOnly;
      ImGui.pushFont(ImGuiTools.getConsoleFont());
//      ImGui.text(consoleText.get());
      ImGui.inputTextMultiline("###console", consoleText, ImGui.getColumnWidth(), 500, inputTextFlags);
      ImGui.popFont();

      ImGui.endChild();
   }

   public static void main(String[] args)
   {
      new ImGuiSSHJShellUI();
   }
}
