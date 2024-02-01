package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImPlotTools;
import us.ihmc.log.LogTools;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Seizure warning - this stress test generates rapidly flashing, random colors
 */
public class RDXImPlotStressTestDemo
{
   private final String WINDOW_NAME = "ImPlot Stress Test";
   private RDXBaseUI baseUI;
   private AtomicInteger numPlotsToShow = new AtomicInteger(50);
   private final Timer timer = new Timer();
   private final double[] xs = new double[500];
   private final double[] ys = new double[500];
   private final Random random = new Random();
   private boolean recalculate = true;

   public RDXImPlotStressTestDemo()
   {
      LogTools.info("Starting UI");
      baseUI = new RDXBaseUI(WINDOW_NAME);
   }

   public void launch()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            ImPlotTools.ensureImPlotInitialized();

            timer.scheduleAtFixedRate(new TimerTask()
            {
               @Override
               public void run()
               {
                  numPlotsToShow.incrementAndGet();
               }
            }, 1000, 1500);

            timer.scheduleAtFixedRate(new TimerTask()
            {
               @Override
               public void run()
               {
                  recalculate = true;
               }
            }, 0, 50);
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            ImGui.begin(WINDOW_NAME);

            if (recalculate)
            {
               for (int i = 0; i < 500; i++)
               {
                  xs[i] = random.nextInt(500) + random.nextDouble();
                  ys[i] = random.nextInt(500) + random.nextDouble();
               }
               recalculate = false;
            }

            int max = numPlotsToShow.get();
            for (int i = 0; i < max; i++)
            {
               if (i % 8 != 0)
                  ImGui.sameLine();

               ImPlot.pushColormap(random.nextInt(16));
               if (ImPlot.beginPlot("Plot " + i, "X", "Y", new ImVec2(225, 150)))
               {
                  if (random.nextBoolean())
                     ImPlot.plotLine("line" + i, xs, ys, xs.length, 0);
                  if (random.nextBoolean())
                     ImPlot.plotBars("bars" + i, xs, ys, xs.length, 0.67f, 0);
                  if (random.nextBoolean())
                     ImPlot.plotScatter("bars" + i, xs, ys, xs.length, 0);
                  if (random.nextBoolean())
                     ImPlot.plotBarsH("bars" + i, xs, ys, xs.length, 0.67f, 0);

                  ImPlot.endPlot();
               }
               ImPlot.popColormap();
            }

            ImGui.end();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args) //Seizure warning - this stress test generates rapidly flashing, random colors
   {
      RDXImPlotStressTestDemo ui = new RDXImPlotStressTestDemo();
      ui.launch();
   }
}
