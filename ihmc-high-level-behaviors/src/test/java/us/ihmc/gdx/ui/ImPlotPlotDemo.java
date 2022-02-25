package us.ihmc.gdx.ui;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.ui.yo.ImPlotPlot;
import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlotPanel;
import us.ihmc.tools.time.FrequencyCalculator;

public class ImPlotPlotDemo
{
   private final ImGuiGlfwWindow baseUI = new ImGuiGlfwWindow(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private ImPlotPlotPanel plotPanel = new ImPlotPlotPanel("Plot Panel");
   private ImPlotPlot trigPlot = new ImPlotPlot("ImPlot 1");
   private final ImPlotDoublePlotLine sineLine = new ImPlotDoublePlotLine("Sine");
   private final ImPlotDoublePlotLine cosineLine = new ImPlotDoublePlotLine("Cosine");
   private ImPlotPlot fpsPlot = new ImPlotPlot("Frames per second");
   private final ImPlotDoublePlotLine fpsPlotLine = new ImPlotDoublePlotLine("Frames per second");
   private ImPlotPlot timePlot = new ImPlotPlot("Time");
   private final ImPlotDoublePlotLine timeLine = new ImPlotDoublePlotLine("Time");
   private Stopwatch stopwatch = new Stopwatch().start();
   private FrequencyCalculator fpsCalculator = new FrequencyCalculator();

   public ImPlotPlotDemo()
   {
      trigPlot.getPlotLines().add(sineLine);
      trigPlot.getPlotLines().add(cosineLine);
      fpsPlot.getPlotLines().add(fpsPlotLine);
      timePlot.getPlotLines().add(timeLine);
      plotPanel.getPlots().add(trigPlot);
      plotPanel.getPlots().add(fpsPlot);
      plotPanel.getPlots().add(timePlot);

      baseUI.getImGuiDockSystem().getPanelManager().addPanel(plotPanel);
      ThreadTools.startAThread(() ->
      {
         baseUI.run(this::render, () -> System.exit(0));
      }, "ImPlot Demo");
   }

   private void render()
   {
      sineLine.addValue(Math.sin(stopwatch.totalElapsed()));
      cosineLine.addValue(Math.cos(stopwatch.totalElapsed()));
      fpsCalculator.ping();
      fpsPlotLine.addValue(fpsCalculator.getFrequency());
      timeLine.addValue(stopwatch.totalElapsed());
   }

   private void renderImGuiWidgets()
   {
      trigPlot.render();
      fpsPlot.render();
   }

   public static void main(String[] args)
   {
      new ImPlotPlotDemo();
   }
}
