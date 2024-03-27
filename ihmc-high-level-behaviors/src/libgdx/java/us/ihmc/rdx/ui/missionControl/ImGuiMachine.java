package us.ihmc.rdx.ui.missionControl;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.flag.ImPlotAxisFlags;
import imgui.extension.implot.flag.ImPlotFlags;
import imgui.flag.ImGuiCond;
import mission_control_msgs.msg.dds.SystemResourceUsageMessage;
import mission_control_msgs.msg.dds.SystemServiceStatusMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.ros2.ROS2Node;

import java.text.DecimalFormat;
import java.util.*;

public class ImGuiMachine
{
   private static final int GRAPH_HISTORY_LENGTH = 30;
   private static final int GRAPH_BUFFER_SIZE = GRAPH_HISTORY_LENGTH * 5;
   private static final long FLASH_RATE_MS = 1000;

   /**
    * Reasonably high value for a CPU temperature. Most CPUs throttle at 100C.
    */
   private static final int CPU_TEMP_WARN_THRESHOLD_C = 85;
   /**
    * Reasonably high value for a GPU temperature.
    */
   private static final int GPU_TEMP_WARN_THRESHOLD_C = 85;

   private final UUID instanceId;
   private final String hostname;

   private final ROS2Node ros2Node;
   private ROS2PublisherBasics<Empty> rebootPublisher;
   private SystemResourceUsageMessage lastResourceUsageMessage = new SystemResourceUsageMessage();

   private final RDXPanel panel;
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

      panel = new RDXPanel(hostname + "##" + instanceId);

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

      ThreadTools.startAsDaemon(() ->
      {
         rebootPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.getSystemRebootTopic(instanceId));
      }, "Reboot-Publisher-Thread");

      ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.getSystemResourceUsageTopic(instanceId), subscriber ->
      {
         acceptSystemResourceUsageMessage(subscriber.takeNextData());
      });
      ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.getSystemServiceStatusTopic(instanceId), subscriber ->
      {
         acceptSystemServiceStatusMessage(subscriber.takeNextData());
      });
   }

   public String getHostname()
   {
      return hostname;
   }

   public SystemResourceUsageMessage getLastResourceUsageMessage()
   {
      return lastResourceUsageMessage;
   }

   public RDXPanel getPanel()
   {
      return panel;
   }

   public void sendRebootRequest()
   {
      rebootPublisher.publish(new Empty());
   }

   private void acceptSystemResourceUsageMessage(SystemResourceUsageMessage message)
   {
      { // Create the plot lines if they don't exist
         // CPU
         if (cpuPlot.getPlotLines().size() < message.getCpuCount())
         {
            for (int i = 0; i < message.getCpuCount(); i++)
            {
               cpuPlot.getPlotLines()
                      .add(new ImPlotDoublePlotLine("CPU (" + i + ") utilization %", GRAPH_BUFFER_SIZE, GRAPH_HISTORY_LENGTH, new DecimalFormat("0.0")));
            }
         }

         // RAM
         if (ramPlot.getPlotLines().isEmpty())
         {
            ramPlot.getPlotLines().add(new ImPlotDoublePlotLine("RAM usage (GiB)", GRAPH_BUFFER_SIZE, GRAPH_HISTORY_LENGTH, new DecimalFormat("0.0")));
            ramPlot.getPlotLines().add(new ImPlotDoublePlotLine("RAM total (GiB)", GRAPH_BUFFER_SIZE, GRAPH_HISTORY_LENGTH, new DecimalFormat("0.0")));
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
            plot.getPlotLines()
                .add(new ImPlotDoublePlotLine("GPU (" + gpuIndex + ", " + gpuModel + ") utilization %",
                                              GRAPH_BUFFER_SIZE,
                                              GRAPH_HISTORY_LENGTH,
                                              new DecimalFormat("0.0")));
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
            plot.getPlotLines()
                .add(new ImPlotDoublePlotLine("GPU (" + gpuIndex + ", " + gpuModel + ") memory usage (GiB)",
                                              GRAPH_BUFFER_SIZE,
                                              GRAPH_HISTORY_LENGTH,
                                              new DecimalFormat("0.0")));
            plot.getPlotLines()
                .add(new ImPlotDoublePlotLine("GPU (" + gpuIndex + ", " + gpuModel + ") memory total (GiB)",
                                              GRAPH_BUFFER_SIZE,
                                              GRAPH_HISTORY_LENGTH,
                                              new DecimalFormat("0.0")));
            vramPlots.add(plot);
            gpuIndex++;
         }

         // Network
         if (netPlot.getPlotLines().size() < message.getIfaceCount())
         {
            for (int i = 0; i < message.getIfaceCount(); i++)
            {
               String iface = message.getIfaceNames().getString(i);
               netPlot.getPlotLines().add(new ImPlotDoublePlotLine(iface + " rx (Kbps)", GRAPH_BUFFER_SIZE, GRAPH_HISTORY_LENGTH, new DecimalFormat("0.0")));
               netPlot.getPlotLines().add(new ImPlotDoublePlotLine(iface + " tx (Kbps)", GRAPH_BUFFER_SIZE, GRAPH_HISTORY_LENGTH, new DecimalFormat("0.0")));
            }
         }
      }

      { // Update the plot line values
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
         service = new ImGuiMachineService(serviceName, hostname, instanceId, panel, ros2Node);
         services.put(serviceName, service);
         service.openLogPanel();
      }
      else
      {
         service = services.get(serviceName);
      }

      service.setStatus(message.getStatusAsString());

      if (!message.getLogData().isEmpty())
      {
         String[] logLines = MessageTools.unpackLongStringFromByteSequence(message.getLogData()).split("\n");
         service.acceptLogLines(Arrays.stream(logLines).toList(), message.getRefresh());
      }
   }

   long lastWarningFlashMs = 0L;
   boolean flashWarningFlag = false;

   /**
    * Makes the test flash red every second
    *
    * @param text the text to flash
    */
   private void flashWarningText(String text)
   {
      long now = System.currentTimeMillis();
      if (now - lastWarningFlashMs > FLASH_RATE_MS)
      {
         flashWarningFlag = !flashWarningFlag;
         lastWarningFlashMs = now;
      }

      if (flashWarningFlag)
      {
         ImGuiTools.textColored(Color.RED, text);
      }
      else
      {
         ImGui.text(text);
      }
   }

   public void renderImGuiWidgets()
   {
      String cpuWarning = "";
      // Render usage graphs
      float highestLastCPUTemp = 0f;
      // Do not assume there are the same amount of CPUs in the temps array as the CPU utilization array
      // CPU temps only map to physical cores - CPU utilization also includes logical threads
      for (int i = 0; i < lastResourceUsageMessage.getCpuTemps().size(); i++)
         highestLastCPUTemp = lastResourceUsageMessage.getCpuTemps().get(i);
      if (highestLastCPUTemp > CPU_TEMP_WARN_THRESHOLD_C)
         cpuWarning = " [high temperature (" + highestLastCPUTemp + "C)]";

      if (!cpuWarning.isEmpty())
         flashWarningText("CPU" + cpuWarning);
      else
         ImGui.text("CPU");

      cpuPlot.render();
      ImGui.text("RAM");
      ramPlot.render();
      if (!gpuPlots.isEmpty())
      {
         String gpuWarning = "";
         if (!lastResourceUsageMessage.getNvidiaGpuTemps().isEmpty())
         {
            float lastGPUTemp = lastResourceUsageMessage.getNvidiaGpuTemps().get(0);
            if (lastGPUTemp > GPU_TEMP_WARN_THRESHOLD_C)
               gpuWarning = " [high temperature (" + lastGPUTemp + "C)]";
         }
         if (!gpuWarning.isEmpty())
            flashWarningText("GPU" + gpuWarning);
         else
            ImGui.text("GPU");
      }

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
