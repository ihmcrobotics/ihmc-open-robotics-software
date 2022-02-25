package us.ihmc.gdx.ui;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.yo.ImPlotPlot;
import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.tools.time.FrequencyCalculator;

public class ImPlotPlotDemo
{
   private final ImGuiGlfwWindow baseUI = new ImGuiGlfwWindow(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private ImPlotPlot imPlotPlot = new ImPlotPlot("ImPlot 1", 250f);
   private final ImPlotDoublePlotLine linePlot = new ImPlotDoublePlotLine("Variable");
   private ImPlotPlot fpsPlot = new ImPlotPlot("Frames per second", 250f);
   private final ImPlotDoublePlotLine fpsPlotLine = new ImPlotDoublePlotLine("Frames per second");
   private double value = 0.0;
   private Stopwatch stopwatch = new Stopwatch().start();
   private FrequencyCalculator fpsCalculator = new FrequencyCalculator();

   public ImPlotPlotDemo()
   {
      imPlotPlot.getPlotLines().add(linePlot);
      fpsPlot.getPlotLines().add(fpsPlotLine);

      ImGuiPanel mainPanel = new ImGuiPanel("ImPlots A", this::renderImGuiWidgets);
      mainPanel.getIsShowing().set(true);
      baseUI.getImGuiDockSystem().getPanelManager().addPanel(mainPanel);
      ThreadTools.startAThread(() ->
      {
         baseUI.run(this::render, () -> System.exit(0));
      }, "ImPlot Demo");
   }

   private void render()
   {
      linePlot.addValue(Math.sin(stopwatch.totalElapsed()));
      fpsCalculator.ping();
      fpsPlotLine.addValue(fpsCalculator.getFrequency());
   }

   private void renderImGuiWidgets()
   {
      imPlotPlot.render();
      fpsPlot.render();
   }

   public static void main(String[] args)
   {
      new ImPlotPlotDemo();
   }
}
