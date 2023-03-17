package us.ihmc.rdx.ui.missionControl;

import imgui.ImGui;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.flag.ImPlotAxisFlags;
import imgui.extension.implot.flag.ImPlotFlags;
import imgui.flag.ImGuiCond;
import mission_control_msgs.msg.dds.SystemResourceUsageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.rdx.ui.yo.ImPlotPlot;
import us.ihmc.ros2.ROS2Node;

import java.text.DecimalFormat;

public class ImGuiMachine
{
   private final String instanceId;
   private final String hostname;
   private SystemResourceUsageMessage lastResourceUsageMessage = new SystemResourceUsageMessage();

   /* ImGui elements */
   private final ImPlotPlot cpuPlot = new ImPlotPlot();

   public ImGuiMachine(String instanceId, String hostname, ROS2Node ros2Node)
   {
      this.instanceId = instanceId;
      this.hostname = hostname;
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ROS2Tools.getSystemResourceUsageTopic(instanceId),
                                           subscriber -> lastResourceUsageMessage = subscriber.takeNextData());

      /* ImGui elements setup */
      {
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

         cpuPlot.setFlags(flags);
         cpuPlot.setXFlags(xFlags);
         cpuPlot.setYFlags(yFlags);
         cpuPlot.setCustomBeforePlotLogic(() -> ImPlot.setNextPlotLimitsY(0.0, 103.0, ImGuiCond.Always));
      }
   }

   public String getInstanceId()
   {
      return instanceId;
   }

   public String getHostname()
   {
      return hostname;
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text(hostname);
      ImGui.popFont();

      //      double gpuUsage = 1.0f;
      float plotWidth = false ? 4.0f : 3.0f;

      plotWidth -= 1.0f;

      ImGui.sameLine();

      for (int i = 0; i < lastResourceUsageMessage.getCpuCount(); i++)
      {
         if (cpuPlot.getPlotLines().size() == i)
            cpuPlot.getPlotLines().add(new ImPlotDoublePlotLine("Core " + i, 30 * 5, 30.0, new DecimalFormat("0.0")));

         ((ImPlotDoublePlotLine) cpuPlot.getPlotLines().get(i)).addValue(lastResourceUsageMessage.getCpuUsages().get(i));
      }
      cpuPlot.render(ImGui.getColumnWidth() / plotWidth, 35.0f);
      plotWidth -= 1.0f;

      ImGui.sameLine();
   }
}
