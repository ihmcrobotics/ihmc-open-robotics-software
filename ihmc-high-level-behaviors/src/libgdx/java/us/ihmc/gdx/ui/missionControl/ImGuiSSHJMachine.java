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
import us.ihmc.missionControl.MissionControlService;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

public class ImGuiSSHJMachine
{
   private final String machineName;
   private final MessagerHelper messagerHelper;

   private final ImGuiMessagerManagerWidget messagerManagerWidget;
   private final AtomicReference<ArrayList<String>> serviceStatusSubscription;
   private final AtomicReference<MutablePair<Double, Double>> ramUsageSubscription;
   private final AtomicReference<ArrayList<Double>> cpuUsagesSubscription;
   private final AtomicReference<MutablePair<Double, Double>> networkUsageSubscription;

   private final ImPlotPlot ramPlot = new ImPlotPlot();
   private final ImPlotDoublePlotLine ramUsagePlotLine = new ImPlotDoublePlotLine("RAM Usage GiB", 30 * 5, 30.0, new DecimalFormat("0.0"));
   private final ImPlotDoublePlotLine ramTotalPlotLine = new ImPlotDoublePlotLine("RAM Total GiB", 30 * 5, 30.0, new DecimalFormat("0.0"));
   private double totalRAM = 1.0;
   private final ImPlotPlot cpuPlot = new ImPlotPlot();
   private final ImPlotPlot networkPlot = new ImPlotPlot();
   private final ImPlotDoublePlotLine networkSentPlotLine = new ImPlotDoublePlotLine("Network Sent KB/s", 30 * 5, 30.0, new DecimalFormat("0.000"));
   private final ImPlotDoublePlotLine networkReceivedPlotLine = new ImPlotDoublePlotLine("Network Received KB/s", 30 * 5, 30.0, new DecimalFormat("0.000"));

   private final ImGuiSSHJSystemdServiceManager systemdServiceManager;
   private final HashMap<String, AtomicReference<String>> serviceStatuses = new HashMap<>();

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
      yFlags += ImPlotAxisFlags.AutoFit;
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
         ImPlot.setNextPlotLimitsY(0.0, 103.0, ImGuiCond.Always);
      });

      networkPlot.setFlags(flags);
      networkPlot.setXFlags(xFlags);
      networkPlot.setYFlags(yFlags);
      networkPlot.setCustomBeforePlotLogic(() ->
      {
         ImPlot.setNextPlotLimitsY(-3000.0, 103000.0, ImGuiCond.Always);
      });
      networkPlot.getPlotLines().add(networkSentPlotLine);
      networkPlot.getPlotLines().add(networkReceivedPlotLine);

      messagerHelper = new MessagerHelper(MissionControlService.API.create());

      ArrayList<String> initialStatuses = new ArrayList<>();
      initialStatuses.add("mission-control-2:Status not yet received.");
      serviceStatusSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.ServiceStatuses, initialStatuses);
      ramUsageSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.RAMUsage, MutablePair.of(0.0, 1.0));
      cpuUsagesSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.CPUUsages, new ArrayList<>());
      networkUsageSubscription = messagerHelper.subscribeViaReference(MissionControlService.API.NetworkUsage, MutablePair.of(0.0, 0.0));

      messagerManagerWidget = new ImGuiMessagerManagerWidget(messagerHelper, () -> hostname, MissionControlService.API.PORT);
   }

   public void renderImGuiWidgets(boolean condensedView)
   {
      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text(machineName);
      ImGui.popFont();

      if (!condensedView)
      {
         systemdServiceManager.renderImGuiWidgets();
         messagerManagerWidget.renderImGuiWidgets();
      }

      ArrayList<String> statuses = serviceStatusSubscription.get();
      for (String status : statuses)
      {
         String[] split = status.split(":", 2);
         if (split[0].equals("mission-control-2"))
         {
            if (!condensedView)
            {
               ImGui.text(split[1]);
            }
         }
         else
         {
            AtomicReference<String> statusAtomicReference = serviceStatuses.get(split[0]);
            if (statusAtomicReference != null)
            {
               statusAtomicReference.set(split[1]);
            }
         }
      }

      MutablePair<Double, Double> ramUsage = ramUsageSubscription.getAndSet(null);
      if (ramUsage != null)
      {
         double used = ramUsage.getLeft();
         totalRAM = ramUsage.getRight();
         ramUsagePlotLine.addValue(used);
         ramTotalPlotLine.addValue(totalRAM);
      }
      ramPlot.render(ImGui.getColumnWidth() / 3.0f, 35.0f);

      ImGui.sameLine();

      ArrayList<Double> cpuUsages = cpuUsagesSubscription.getAndSet(null);
      if (cpuUsages != null)
      {
         for (int i = 0; i < cpuUsages.size(); i++)
         {
            if (cpuPlot.getPlotLines().size() == i)
            {
               cpuPlot.getPlotLines().add(new ImPlotDoublePlotLine("Core " + i, 30 * 5, 30.0, new DecimalFormat("0.0")));
            }

            ((ImPlotDoublePlotLine) cpuPlot.getPlotLines().get(i)).addValue(cpuUsages.get(i));
         }
      }
      cpuPlot.render(ImGui.getColumnWidth() / 2.0f, 35.0f);

      ImGui.sameLine();

      MutablePair<Double, Double> networkUsage = networkUsageSubscription.getAndSet(null);
      if (networkUsage != null)
      {
         double sent = networkUsage.getLeft();
         double received = networkUsage.getRight();
         networkSentPlotLine.addValue(sent);
         networkReceivedPlotLine.addValue(received);
      }
      networkPlot.render(ImGui.getColumnWidth(), 35.0f);
   }

   public ImGuiSSHJSystemdServiceManager getSystemdServiceManager()
   {
      return systemdServiceManager;
   }

   public AtomicReference<String> subscribeToServiceStatus(String serviceName)
   {
      AtomicReference<String> atomicReference = new AtomicReference<>("Status not yet received.");
      serviceStatuses.put(serviceName, atomicReference);
      return atomicReference;
   }

   public ImGuiMessagerManagerWidget getMessagerManagerWidget()
   {
      return messagerManagerWidget;
   }

   public MessagerHelper getMessagerHelper()
   {
      return messagerHelper;
   }
}
