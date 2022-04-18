package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.flag.ImPlotAxisFlags;
import imgui.extension.implot.flag.ImPlotFlags;
import imgui.flag.ImGuiCond;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.behaviors.tools.MessagerHelper;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.ui.tools.ImGuiMessagerManagerWidget;
import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlot;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.missionControl.MissionControlService;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class ImGuiSSHJMachine
{
   private final String machineName;
   private final MessagerHelper messagerHelper;

   private final ImGuiMessagerManagerWidget messagerManagerWidget;
   private final AtomicReference<String> serviceStatusSubscription;
   private final AtomicReference<MutablePair<Double, Double>> ramUsageSubscription;
   private final AtomicReference<ArrayList<Double>> cpuUsagesSubscription;

   private final ImPlotPlot ramPlot = new ImPlotPlot();
   private final ImPlotDoublePlotLine ramUsagePlotLine = new ImPlotDoublePlotLine("RAM Usage GiB", 30 * 5, 30.0, new DecimalFormat("0.0"));
   private final ImPlotDoublePlotLine ramTotalPlotLine = new ImPlotDoublePlotLine("RAM Total GiB", 30 * 5, 30.0, new DecimalFormat("0.0"));
   private double totalRAM = 1.0;
   private final ImPlotPlot cpuPlot = new ImPlotPlot();

   private final ImGuiSSHJSystemdServiceManager systemdServiceManager;

   public ImGuiSSHJMachine(String machineName, String hostname, String username)
   {
      this.machineName = machineName;
      systemdServiceManager = new ImGuiSSHJSystemdServiceManager(machineName, "Mission Control Service", "mission-control-2", hostname, username);

      int flags = ImPlotFlags.None;
      flags += ImPlotFlags.NoMenus;
      flags += ImPlotFlags.NoBoxSelect;
      flags += ImPlotFlags.NoTitle;
      flags += ImPlotFlags.NoMousePos;
      flags += ImPlotFlags.NoLegend;
      int xFlags = ImPlotAxisFlags.None;
      xFlags += ImPlotAxisFlags.NoDecorations;
      xFlags += ImPlotAxisFlags.AutoFit;
      int yFlags = ImPlotAxisFlags.None;
      yFlags += ImPlotAxisFlags.NoDecorations;
      ramPlot.setFlags(flags);
      ramPlot.setXFlags(xFlags);
      ramPlot.setYFlags(yFlags);
      ramPlot.setCustomBeforePlotLogic(() ->
      {
         ImPlot.setNextPlotLimitsY(0.0, totalRAM, ImGuiCond.Always);
      });
      ramPlot.getPlotLines().add(ramUsagePlotLine);
      ramPlot.getPlotLines().add(ramTotalPlotLine);

      cpuPlot.setFlags(flags);
      cpuPlot.setXFlags(xFlags);
      cpuPlot.setYFlags(yFlags);
      cpuPlot.setCustomBeforePlotLogic(() ->
      {
         ImPlot.setNextPlotLimitsY(0.0, 100.0, ImGuiCond.Always);
      });

      messagerHelper = new MessagerHelper(MissionControlService.API.create());

      serviceStatusSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.ServiceStatus, "Status not yet received.");
      ramUsageSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.RAMUsage, MutablePair.of(0.0, 1.0));
      cpuUsagesSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.CPUUsages, new ArrayList<>());

      messagerManagerWidget = new ImGuiMessagerManagerWidget(messagerHelper, () -> hostname, MissionControlService.API.PORT);
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text(machineName);
      ImGui.popFont();

      systemdServiceManager.renderImGuiWidgets();

      messagerManagerWidget.renderImGuiWidgets();

      ImGui.text(serviceStatusSubscription.get());

      MutablePair<Double, Double> ramUsage = ramUsageSubscription.getAndSet(null);
      if (ramUsage != null)
      {
         double used = ramUsage.getLeft();
         totalRAM = ramUsage.getRight();
         ramUsagePlotLine.addValue(used);
         ramTotalPlotLine.addValue(totalRAM);
      }
      ramPlot.render(ImGui.getColumnWidth() / 2.0f, 50.0f);

      ImGui.sameLine();

      ArrayList<Double> cpuUsages = cpuUsagesSubscription.getAndSet(null);
      if (cpuUsages != null)
      {
         for (int i = 0; i < cpuUsages.size(); i++)
         {
            if (cpuPlot.getPlotLines().size() == i)
            {
               cpuPlot.getPlotLines().add(new ImPlotDoublePlotLine("Core " + i + ":", 30 * 5, 30.0, new DecimalFormat("0.0")));
            }

            ((ImPlotDoublePlotLine) cpuPlot.getPlotLines().get(i)).addValue(cpuUsages.get(i));
         }
      }
      cpuPlot.render(ImGui.getColumnWidth(), 50.0f);
   }

   public ImGuiSSHJSystemdServiceManager getSystemdServiceManager()
   {
      return systemdServiceManager;
   }

   public AtomicReference<String> subscribeToServiceStatus(MessagerAPIFactory.Topic<String> topic)
   {
      return messagerHelper.subscribeViaReference(topic, "Status not yet received.");
   }
}
