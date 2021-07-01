package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import imgui.type.ImInt;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.avatar.logging.PlanarRegionsListBuffer;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

public class ImGuiGDXImPlotStressTestDemo //Seizure warning - this stress test generates rapidly flashing, random colors
{

   private final String WINDOW_NAME = "ImPlot Stress Test";

   private GDXImGuiBasedUI baseUI;

   private AtomicInteger numPlotsToShow = new AtomicInteger(50);

   private final Timer t = new Timer();

   private final Double[] xs = new Double[500];
   private final Double[] ys = new Double[500];

   private final Random rand = new Random();

   private boolean recalc = true;

   public ImGuiGDXImPlotStressTestDemo()
   {
      LogTools.info("Starting UI");
      baseUI = new GDXImGuiBasedUI(getClass(), "atlas-user-interface", "src/libgdx/resources", WINDOW_NAME);
   }

   public void launch()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            ImPlot.createContext();

            t.scheduleAtFixedRate(new TimerTask()
            {
               @Override
               public void run()
               {
                  numPlotsToShow.incrementAndGet();
               }
            }, 1000, 1500);

            t.scheduleAtFixedRate(new TimerTask()
            {
               @Override
               public void run()
               {
                  recalc = true;
               }
            }, 0, 50);
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            ImGui.begin(WINDOW_NAME);

            if (recalc) {
               for (int i = 0; i < 500; i++) {
                  xs[i] = rand.nextInt(500) + rand.nextDouble();
                  ys[i] = rand.nextInt(500) + rand.nextDouble();
               }
               recalc = false;
            }

            int max = numPlotsToShow.get();
            for (int i = 0; i < max; i++)
            {
               if (i % 8 != 0)
                  ImGui.sameLine();

               ImPlot.pushColormap(rand.nextInt(16));
               if (ImPlot.beginPlot("Plot " + i, "X", "Y", new ImVec2(225, 150)))
               {
                  if (rand.nextBoolean())
                     ImPlot.plotLine("line" + i, xs, ys);
                  if (rand.nextBoolean())
                     ImPlot.plotBars("bars" + i, xs, ys);
                  if (rand.nextBoolean())
                     ImPlot.plotScatter("bars" + i, xs, ys);
                  if (rand.nextBoolean())
                     ImPlot.plotBarsH("bars" + i, xs, ys);

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
      ImGuiGDXImPlotStressTestDemo ui = new ImGuiGDXImPlotStressTestDemo();
      ui.launch();
   }
}
