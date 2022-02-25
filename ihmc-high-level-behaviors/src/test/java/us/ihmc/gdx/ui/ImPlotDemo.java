package us.ihmc.gdx.ui;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.yo.ImPlotDouble;
import us.ihmc.gdx.ui.yo.ImPlotDoubleLine;
import us.ihmc.tools.time.FrequencyCalculator;

public class ImPlotDemo
{
   private final ImGuiGlfwWindow baseUI = new ImGuiGlfwWindow(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private ImPlotDouble imPlotDouble = new ImPlotDouble("ImPlot 1");
   private final ImPlotDoubleLine linePlot = new ImPlotDoubleLine("Variable");
   private ImPlotDouble fpsPlot = new ImPlotDouble("Frames per second");
   private final ImPlotDoubleLine fpsPlotLine = new ImPlotDoubleLine("Frames per second");
   private double value = 0.0;
   private Stopwatch stopwatch = new Stopwatch().start();
   private FrequencyCalculator fpsCalculator = new FrequencyCalculator();

   public ImPlotDemo()
   {
      imPlotDouble.getPlotLines().add(linePlot);
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
      imPlotDouble.render();
      fpsPlot.render();
   }

   public static void main(String[] args)
   {
      new ImPlotDemo();
   }
}
