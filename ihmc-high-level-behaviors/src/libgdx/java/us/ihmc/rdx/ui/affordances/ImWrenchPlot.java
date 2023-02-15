package us.ihmc.rdx.ui.affordances;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.rdx.ui.yo.ImPlotIntegerPlotLine;
import us.ihmc.rdx.ui.yo.ImPlotPlot;
import us.ihmc.rdx.ui.yo.ImPlotPlotPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyCalculator;

public class ImWrenchPlot
{
   // wrench
   private SideDependentList<ImPlotPlotPanel> wrenchPlotPanels = new SideDependentList<>();

   private final SideDependentList<ImPlotPlot> linear_plots = new SideDependentList<>();
   private final SideDependentList<ImPlotPlot> angular_plots = new SideDependentList<>();

   private final SideDependentList<ImPlotDoublePlotLine> linear_x_plot_lines = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> linear_y_plot_lines = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> linear_z_plot_lines = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> angular_x_plot_lines = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> angular_y_plot_lines = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> angular_z_plot_lines = new SideDependentList<>();

   public ImWrenchPlot(RDXBaseUI baseUI)
   {
      for (RobotSide side : RobotSide.values)
      {
         wrenchPlotPanels.set(side, new ImPlotPlotPanel(side + " wrench"));
         linear_x_plot_lines.set(side, new ImPlotDoublePlotLine(side + " wrench LINEAR: x"));
         linear_y_plot_lines.set(side, new ImPlotDoublePlotLine(side + " wrench LINEAR: y"));
         linear_z_plot_lines.set(side, new ImPlotDoublePlotLine(side + " wrench LINEAR: z"));
         angular_x_plot_lines.set(side, new ImPlotDoublePlotLine(side + " wrench ANGULAR: x"));
         angular_y_plot_lines.set(side, new ImPlotDoublePlotLine(side + " wrench ANGULAR: y"));
         angular_z_plot_lines.set(side, new ImPlotDoublePlotLine(side + " wrench ANGULAR: z"));
         linear_plots.set(side, new ImPlotPlot());
         angular_plots.set(side, new ImPlotPlot());

         linear_plots.get(side).getPlotLines().add(linear_x_plot_lines.get(side));
         linear_plots.get(side).getPlotLines().add(linear_y_plot_lines.get(side));
         linear_plots.get(side).getPlotLines().add(linear_z_plot_lines.get(side));
         angular_plots.get(side).getPlotLines().add(angular_x_plot_lines.get(side));
         angular_plots.get(side).getPlotLines().add(angular_y_plot_lines.get(side));
         angular_plots.get(side).getPlotLines().add(angular_z_plot_lines.get(side));

         wrenchPlotPanels.get(side).getPlots().add(linear_plots.get(side));
         wrenchPlotPanels.get(side).getPlots().add(angular_plots.get(side));

         baseUI.getImGuiPanelManager().addPanel(wrenchPlotPanels.get(side));
      }
   }

   public void update(RobotSide side, SpatialVectorReadOnly wrench)
   {
      linear_x_plot_lines.get(side).addValue(wrench.getLinearPartX());
      linear_y_plot_lines.get(side).addValue(wrench.getLinearPartY());
      linear_z_plot_lines.get(side).addValue(wrench.getLinearPartZ());
      angular_x_plot_lines.get(side).addValue(wrench.getAngularPartX());
      angular_y_plot_lines.get(side).addValue(wrench.getAngularPartX());
      angular_z_plot_lines.get(side).addValue(wrench.getAngularPartX());
   }
}
