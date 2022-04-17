package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import imgui.type.ImString;
import net.schmizz.sshj.connection.channel.direct.Session;
import net.schmizz.sshj.connection.channel.direct.Signal;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.avatar.ros2.networkTest.SSHJTools;
import us.ihmc.behaviors.tools.MessagerHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.ui.tools.ImGuiMessagerManagerWidget;
import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlot;
import us.ihmc.missionControl.MissionControlService;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class ImGuiSSHJMissionControlUI
{
   private final String REMOTE_HOSTNAME = System.getProperty("remote.hostname");
   private final String REMOTE_USERNAME = System.getProperty("remote.username");
   private final ImInt bufferSize = new ImInt(Conversions.megabytesToBytes(2));
   private final ImString command = new ImString(10000);
   private final ImDouble timeout = new ImDouble(0.0);
   private final MessagerHelper messagerHelper;
   private int exitStatus = -1;
   private final SSHJInputStream standardOut = new SSHJInputStream();
   private final SSHJInputStream standardError = new SSHJInputStream();
   private Session.Command sshjCommand;
   private final ImGuiConsoleArea consoleArea = new ImGuiConsoleArea();
   private Thread runThread;
   private final ImGuiMessagerManagerWidget messagerManagerWidget;
   private final AtomicReference<MutablePair<Double, Double>> ramUsageSubscription;
   private final AtomicReference<ArrayList<Double>> cpuUsagesSubscription;

   private final ImPlotPlot ramPlot = new ImPlotPlot();
   private final ImPlotDoublePlotLine ramUsagePlotLine = new ImPlotDoublePlotLine("RAM Usage GiB");
   private final ImPlotDoublePlotLine ramTotalPlotLine = new ImPlotDoublePlotLine("RAM Total GiB");
   private final ImPlotPlot cpuPlot = new ImPlotPlot();

   public ImGuiSSHJMissionControlUI()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(),
                                                            "ihmc-open-robotics-software",
                                                            "ihmc-high-level-behaviors/src/libgdx/resources",
                                                            "SSHJ Shell");
      imGuiGlfwWindow.runWithSinglePanel(this::renderImGuiWidgets);

      standardOut.resize(bufferSize.get());
      standardError.resize(bufferSize.get());
      command.set("sudo journalctl -ef -u mission-control-2");

      ramPlot.getPlotLines().add(ramUsagePlotLine);
      ramPlot.getPlotLines().add(ramTotalPlotLine);

      messagerHelper = new MessagerHelper(MissionControlService.API.create());

      ramUsageSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.RAMUsage, MutablePair.of(0.0, 1.0));
      cpuUsagesSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.CPUUsages, new ArrayList<>());

      messagerManagerWidget = new ImGuiMessagerManagerWidget(messagerHelper, () -> REMOTE_HOSTNAME, MissionControlService.API.PORT);
   }

   private void renderImGuiWidgets()
   {
      messagerManagerWidget.renderImGuiWidgets();

      MutablePair<Double, Double> ramUsage = ramUsageSubscription.get();
      double used = ramUsage.getLeft();
      double total = ramUsage.getRight();
      ramUsagePlotLine.addValue(used);
      ramTotalPlotLine.addValue(total);
      ramPlot.render();

      ArrayList<Double> cpuUsages = cpuUsagesSubscription.get();
      for (int i = 0; i < cpuUsages.size(); i++)
      {
         if (cpuPlot.getPlotLines().size() == i)
         {
            cpuPlot.getPlotLines().add(new ImPlotDoublePlotLine("CPU Core " + i + " %"));
         }

         ((ImPlotDoublePlotLine)cpuPlot.getPlotLines().get(i)).addValue(cpuUsages.get(i));
      }
      cpuPlot.render();

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

      consoleArea.renderImGuiWidgets();
   }

   private boolean isRunning()
   {
      return runThread != null && runThread.isAlive();
   }

   private void acceptNewText(String newText)
   {
      consoleArea.acceptNewText(newText);
      System.out.print(newText);
   }

   public static void main(String[] args)
   {
      new ImGuiSSHJMissionControlUI();
   }
}
