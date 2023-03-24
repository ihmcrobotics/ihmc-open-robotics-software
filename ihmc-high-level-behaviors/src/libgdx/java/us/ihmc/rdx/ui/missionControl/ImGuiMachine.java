package us.ihmc.rdx.ui.missionControl;

import imgui.ImGui;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.flag.ImPlotAxisFlags;
import imgui.extension.implot.flag.ImPlotFlags;
import imgui.flag.ImGuiCond;
import mission_control_msgs.msg.dds.SystemResourceUsageMessage;
import mission_control_msgs.msg.dds.SystemServiceStatusMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.rdx.ui.yo.ImPlotPlot;
import us.ihmc.ros2.ROS2Node;

import java.text.DecimalFormat;
import java.util.*;

public class ImGuiMachine
{
   private final UUID instanceId;
   private final String hostname;

   private final ROS2Node ros2Node;
   private SystemResourceUsageMessage lastResourceUsageMessage = new SystemResourceUsageMessage();

   private final ImGuiPanel panel;
   private final Map<String, ImGuiMachineService> services = new HashMap<>();
   private final ImPlotPlot cpuPlot = new ImPlotPlot();
   private final ImPlotPlot ramPlot = new ImPlotPlot();
   private final List<ImPlotPlot> gpuPlots = new ArrayList<>(); // Empty if no NVIDIA GPUs
   private final List<ImPlotPlot> vramPlots = new ArrayList<>(); // Empty if no NVIDIA GPUs
   private final ImPlotPlot netPlot = new ImPlotPlot();

   private static int plotFlags = ImPlotFlags.None;
   private static int plotAxisXFlags = ImPlotAxisFlags.None;
   private static int plotAxisYFlags = ImPlotAxisFlags.None;

   static
   {
      plotFlags += ImPlotFlags.NoMenus;
      plotFlags += ImPlotFlags.NoBoxSelect;
      plotFlags += ImPlotFlags.NoTitle;
      plotFlags += ImPlotFlags.NoMousePos;
      plotFlags += ImPlotFlags.NoLegend;
      plotFlags += ImPlotFlags.NoChild;
      plotAxisXFlags += ImPlotAxisFlags.NoDecorations;
      plotAxisXFlags += ImPlotAxisFlags.AutoFit;
      plotAxisYFlags += ImPlotAxisFlags.NoDecorations;
      plotAxisYFlags += ImPlotAxisFlags.AutoFit;
   }

   public ImGuiMachine(UUID instanceId, String hostname, ROS2Node ros2Node)
   {
      this.instanceId = instanceId;
      this.hostname = hostname;

      panel = new ImGuiPanel(hostname + "##" + instanceId);

      cpuPlot.setFlags(plotFlags);
      cpuPlot.setXFlags(plotAxisXFlags);
      cpuPlot.setYFlags(plotAxisYFlags);
      cpuPlot.setCustomBeforePlotLogic(() -> ImPlot.setNextPlotLimitsY(0.0, 103.0, ImGuiCond.Always));

      ramPlot.setFlags(plotFlags);
      ramPlot.setXFlags(plotAxisXFlags);
      ramPlot.setYFlags(plotAxisYFlags);
      ramPlot.setCustomBeforePlotLogic(() -> ImPlot.setNextPlotLimitsY(0.0, lastResourceUsageMessage.getMemoryTotal(), ImGuiCond.Always));

      netPlot.setFlags(plotFlags);
      netPlot.setXFlags(plotAxisXFlags);
      netPlot.setYFlags(plotAxisYFlags);
      netPlot.setCustomBeforePlotLogic(() -> ImPlot.setNextPlotLimitsY(-3000.0, 103000.0, ImGuiCond.Always));

      this.ros2Node = ros2Node;
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ROS2Tools.getSystemResourceUsageTopic(instanceId),
                                           subscriber -> acceptSystemResourceUsageMessage(subscriber.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ROS2Tools.getSystemServiceStatusTopic(instanceId),
                                           subscriber -> acceptSystemServiceStatusMessage(subscriber.takeNextData()),
                                           ROS2Tools.getSystemServiceStatusQosProfile());
   }

   private void acceptSystemResourceUsageMessage(SystemResourceUsageMessage message)
   {
      // Create the plot lines if they don't exist
      {
         // CPU
         if (cpuPlot.getPlotLines().size() < message.getCpuCount())
         {
            for (int i = 0; i < message.getCpuCount(); i++)
            {
               cpuPlot.getPlotLines().add(new ImPlotDoublePlotLine("CPU (" + i + ") utilization %", 30 * 5, 30.0, new DecimalFormat("0.0")));
            }
         }

         // RAM
         if (ramPlot.getPlotLines().isEmpty())
         {
            ramPlot.getPlotLines().add(new ImPlotDoublePlotLine("RAM usage (GiB)", 30 * 5, 30.0, new DecimalFormat("0.0")));
            ramPlot.getPlotLines().add(new ImPlotDoublePlotLine("RAM total (GiB)", 30 * 5, 30.0, new DecimalFormat("0.0")));
         }

         // GPU
         int gpuIndex = 0;
         while (gpuPlots.size() < message.getNvidiaGpuCount())
         {
            ImPlotPlot plot = new ImPlotPlot();
            plot.setFlags(plotFlags);
            plot.setXFlags(plotAxisXFlags);
            plot.setYFlags(plotAxisYFlags);
            String gpuModel = message.getNvidiaGpuModels().getString(0);
            plot.setCustomBeforePlotLogic(() -> ImPlot.setNextPlotLimitsY(0.0, 103.0, ImGuiCond.Always));
            plot.getPlotLines().add(new ImPlotDoublePlotLine("GPU (" + gpuIndex + ", " + gpuModel + ") utilization %", 30 * 5, 30.0, new DecimalFormat("0.0")));
            gpuPlots.add(plot);
            gpuIndex++;
         }

         // VRAM
         gpuIndex = 0;
         while (vramPlots.size() < message.getNvidiaGpuCount())
         {
            ImPlotPlot plot = new ImPlotPlot();
            plot.setFlags(plotFlags);
            plot.setXFlags(plotAxisXFlags);
            plot.setYFlags(plotAxisYFlags);
            String gpuModel = message.getNvidiaGpuModels().getString(0);
            float totalGpuMemory = message.getNvidiaGpuMemoryTotal().get(gpuIndex);
            plot.setCustomBeforePlotLogic(() -> ImPlot.setNextPlotLimitsY(0.0, totalGpuMemory, ImGuiCond.Always));
            plot.getPlotLines().add(new ImPlotDoublePlotLine("GPU (" + gpuIndex +  ", " + gpuModel + ") memory usage (GiB)", 30 * 5, 30.0, new DecimalFormat("0.0")));
            plot.getPlotLines().add(new ImPlotDoublePlotLine("GPU (" + gpuIndex + ", " + gpuModel + ") memory total (GiB)", 30 * 5, 30.0, new DecimalFormat("0.0")));
            vramPlots.add(plot);
            gpuIndex++;
         }

         // Network
         if (netPlot.getPlotLines().size() < message.getIfaceCount())
         {
            for (int i = 0; i < message.getIfaceCount(); i++)
            {
               String iface = message.getIfaceNames().getString(i);
               netPlot.getPlotLines().add(new ImPlotDoublePlotLine(iface + " rx (Kbps)", 30 * 5, 30.0, new DecimalFormat("0.0")));
               netPlot.getPlotLines().add(new ImPlotDoublePlotLine(iface + " tx (Kbps)", 30 * 5, 30.0, new DecimalFormat("0.0")));
            }
         }
      }

      // Update the plot line values
      {
         // CPU
         for (int i = 0; i < cpuPlot.getPlotLines().size(); i++)
            ((ImPlotDoublePlotLine) cpuPlot.getPlotLines().get(i)).addValue(message.getCpuUsages().get(i));

         // RAM
         ((ImPlotDoublePlotLine) ramPlot.getPlotLines().get(0)).addValue(message.getMemoryUsed());
         ((ImPlotDoublePlotLine) ramPlot.getPlotLines().get(1)).addValue(message.getMemoryTotal());

         // GPU / VRAM
         for (int i = 0; i < message.getNvidiaGpuCount(); i++)
         {
            ImPlotPlot gpuPlot = gpuPlots.get(i);
            ImPlotPlot vramPlot = vramPlots.get(i);
            ((ImPlotDoublePlotLine) gpuPlot.getPlotLines().get(i)).addValue(message.getNvidiaGpuUtilization().get(i));
            ((ImPlotDoublePlotLine) vramPlot.getPlotLines().get(0)).addValue(message.getNvidiaGpuMemoryUsed().get(i));
            ((ImPlotDoublePlotLine) vramPlot.getPlotLines().get(1)).addValue(message.getNvidiaGpuMemoryTotal().get(i));
         }

         // Network
         for (int i = 0; i < netPlot.getPlotLines().size(); i++)
         {
            ImPlotDoublePlotLine plotLine = (ImPlotDoublePlotLine) netPlot.getPlotLines().get(i);
            String plotLineName = plotLine.getVariableName();

            // The message has rx and tx data for all interfaces
            // Find the network iface in the message for the plot line we are updating
            for (int j = 0; j < message.getIfaceCount(); j++)
            {
               String iface = message.getIfaceNames().getString(j);

               // j is our iface index
               if (plotLineName.contains(iface))
               {
                  float rx = message.getIfaceRxKbps().get(j);
                  float tx = message.getIfaceTxKbps().get(j);

                  if (plotLineName.contains("rx"))
                     plotLine.addValue(rx); // If the plot line represents rx
                  else if (plotLineName.contains("tx"))
                     plotLine.addValue(tx); // If the plot line represents tx
               }
            }
         }
      }

      lastResourceUsageMessage = message;
   }

   private void acceptSystemServiceStatusMessage(SystemServiceStatusMessage message)
   {
      String serviceName = message.getServiceNameAsString();
      final ImGuiMachineService service;

      if (!services.containsKey(serviceName))
      {
         service = new ImGuiMachineService(serviceName, hostname, instanceId, ros2Node);
         services.put(serviceName, service);
      }
      else
      {
         service = services.get(serviceName);
      }

      service.setStatus(message.getStatusAsString());

      if (message.getLogLineCount() > 0)
      {
         List<String> logLines = new ArrayList<>();

         for (int i = 0; i < message.getLogLineCount(); i++)
         {
            String logLine = message.getLogLines().getString(i);
            logLines.add(logLine);
         }

         service.acceptLogLines(logLines);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text(hostname);
      ImGui.popFont();

      // Render usage graphs
      ImGui.text("CPU");
      cpuPlot.render();
      ImGui.text("RAM");
      ramPlot.render();
      if (!gpuPlots.isEmpty())
         ImGui.text("GPUs");
      gpuPlots.forEach(ImPlotPlot::render);
      if (!vramPlots.isEmpty())
         ImGui.text("VRAM");
      vramPlots.forEach(ImPlotPlot::render);
      ImGui.text("Network");
      netPlot.render();

      // Render service statuses & buttons
      for (ImGuiMachineService service : services.values())
      {
         ImGui.separator();
         service.renderImGuiWidgets();
      }
      ImGui.separator();
   }
}
