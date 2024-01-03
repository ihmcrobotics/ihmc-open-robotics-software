package us.ihmc.rdx.ui.affordances;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImPlotDoublePlot;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDX3DPanelHandWrenchIndicatorSide
{
   private static final int PLOT_HEIGHT = 50;

   private final RobotSide side;
   private boolean showPlots = false;
   private double force = 0.0;
   private double torque = 0.0;
   private final ImPlotDoublePlot forcePlot;
   private final ImPlotDoublePlot torquePlot;

   public RDX3DPanelHandWrenchIndicatorSide(RobotSide side)
   {
      this.side = side;
      forcePlot = new ImPlotDoublePlot(side.getPascalCaseName() + " Force");
      torquePlot = new ImPlotDoublePlot(side.getPascalCaseName() + " Torque");
      forcePlot.getPlotLine().setColor(side == RobotSide.LEFT ? ImGuiTools.DARK_RED : ImGuiTools.DARK_GREEN);
      torquePlot.getPlotLine().setColor(side == RobotSide.LEFT ? ImGuiTools.DARK_RED : ImGuiTools.DARK_GREEN);
      forcePlot.getPlot().setCustomBeforePlotLogic(() -> forcePlot.getPlotLine().setLimitYMin(80.0));
      torquePlot.getPlot().setCustomBeforePlotLogic(() -> torquePlot.getPlotLine().setLimitYMin(15.0));
   }

   public void update(double force, double torque)
   {
      this.force = force;
      this.torque = torque;
      if (showPlots)
      {
         forcePlot.addValue(force);
         torquePlot.addValue(torque);
      }
   }

   public void renderImGuiWidgets()
   {
      if (showPlots)
      {
         forcePlot.renderImGuiWidgets(ImGui.getColumnWidth(), PLOT_HEIGHT);
         torquePlot.renderImGuiWidgets(ImGui.getColumnWidth(), PLOT_HEIGHT);
      }
      else
      {
         ImGui.pushFont(ImGuiTools.getMediumFont());
         ImGui.text(side.getPascalCaseName() + " Force: " + String.format("%.2f", force) + " N");
         ImGui.text(side.getPascalCaseName() + " Torque: " + String.format("%.2f", torque) + " Nm");
         ImGui.popFont();
      }
   }

   public int getHeight()
   {
      int height;
      if (showPlots)
      {
         height = PLOT_HEIGHT * 2 + (int) Math.ceil(ImGui.getStyle().getFramePaddingY() * 2.0f);
      }
      else
      {
         ImGui.pushFont(ImGuiTools.getMediumFont());
         height = (int) Math.ceil(ImGui.getFrameHeight() * 2.0f);
         ImGui.popFont();
      }
      return height;
   }

   public double getForce()
   {
      return force;
   }

   public double getTorque()
   {
      return torque;
   }

   public void setShowPlots(boolean showPlots)
   {
      this.showPlots = showPlots;
   }

   public boolean getShowPlots()
   {
      return showPlots;
   }
}
