package us.ihmc.rdx.ui;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.imgui.ImGuiGlfwWindow;
import us.ihmc.rdx.imgui.ImPlotIntegerPlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlotPanel;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.Random;

public class ImPlotPlotDemo
{
   private final ImGuiGlfwWindow baseUI = new ImGuiGlfwWindow(getClass());
   private ImPlotPlotPanel plotPanel = new ImPlotPlotPanel("Plot Panel");
   private ImPlotPlot trigPlot = new ImPlotPlot();
   private final ImPlotDoublePlotLine sineLine = new ImPlotDoublePlotLine("Sine");
   private final ImPlotDoublePlotLine cosineLine = new ImPlotDoublePlotLine("Cosine");
   private ImPlotPlot fpsPlot = new ImPlotPlot();
   private final ImPlotDoublePlotLine fpsPlotLine = new ImPlotDoublePlotLine("Frames per second");
   private ImPlotPlot timePlot = new ImPlotPlot();
   private final ImPlotDoublePlotLine timeLine = new ImPlotDoublePlotLine("Time");
   private Stopwatch stopwatch = new Stopwatch().start();
   private FrequencyCalculator fpsCalculator = new FrequencyCalculator();
   private ImPlotPlot randomIntPlot = new ImPlotPlot();
   private final ImPlotIntegerPlotLine randomIntLine = new ImPlotIntegerPlotLine("Random integer");
   private final Random random = new Random();

   public ImPlotPlotDemo()
   {
      trigPlot.getPlotLines().add(sineLine);
      trigPlot.getPlotLines().add(cosineLine);
      fpsPlot.getPlotLines().add(fpsPlotLine);
      timePlot.getPlotLines().add(timeLine);
      randomIntPlot.getPlotLines().add(randomIntLine);
      plotPanel.getPlots().add(trigPlot);
      plotPanel.getPlots().add(fpsPlot);
      plotPanel.getPlots().add(timePlot);
      plotPanel.getPlots().add(randomIntPlot);


      baseUI.getImGuiDockSystem().getPanelManager().addPanel(plotPanel);
      ThreadTools.startAThread(() ->
      {
         baseUI.run(this::create, this::render, () -> System.exit(0));
      }, "ImPlot Demo");
   }

   private void create()
   {

   }

   private void render()
   {
      sineLine.addValue(Math.sin(stopwatch.totalElapsed()));
      cosineLine.addValue(Math.cos(stopwatch.totalElapsed()));
      fpsCalculator.ping();
      fpsPlotLine.addValue(fpsCalculator.getFrequency());
      timeLine.addValue(stopwatch.totalElapsed());
      randomIntLine.addValue(random.nextInt(100));
   }

   public static void main(String[] args)
   {
      new ImPlotPlotDemo();
   }
}
