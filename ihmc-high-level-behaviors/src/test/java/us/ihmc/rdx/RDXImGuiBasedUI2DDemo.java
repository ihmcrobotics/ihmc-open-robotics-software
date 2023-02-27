package us.ihmc.rdx;

import imgui.internal.ImGui;
import org.apache.logging.log4j.Level;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
import us.ihmc.rdx.ui.RDXBaseUI2D;
import us.ihmc.rdx.ui.tools.ImGuiLogWidget;
import us.ihmc.tools.string.StringTools;

import java.time.LocalDateTime;

public class RDXImGuiBasedUI2DDemo
{
   private final RDXBaseUI2D baseUI = new RDXBaseUI2D(getClass(),
                                                      "ihmc-open-robotics-software",
                                                      "ihmc-high-level-behaviors/src/test/resources");

   private final Stopwatch stopwatch = new Stopwatch().start();
   private final ImGuiMovingPlot renderPlot = new ImGuiMovingPlot("render count", 1000, 300, 30);
   private final ImGuiLogWidget logWidget = new ImGuiLogWidget("Log");
   private long renderCount = 0;

   public RDXImGuiBasedUI2DDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getImGuiPanelManager().addPanel("Window 1", RDXImGuiBasedUI2DDemo.this::renderWindow1);
            baseUI.getImGuiPanelManager().addPanel("Window 2", RDXImGuiBasedUI2DDemo.this::renderWindow2);
            baseUI.getImGuiPanelManager().addPanel("Window 3", RDXImGuiBasedUI2DDemo.this::renderWindow3);

            logWidget.submitEntry(Level.WARN, "WARN at " + LocalDateTime.now());
            logWidget.submitEntry(Level.ERROR, "ERROR at " + LocalDateTime.now());
            logWidget.submitEntry(Level.DEBUG, "DEBUG at " + LocalDateTime.now());
            logWidget.submitEntry(Level.FATAL, "FATAL at " + LocalDateTime.now());
            logWidget.submitEntry(Level.TRACE, "TRACE at " + LocalDateTime.now());
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   private void renderWindow1()
   {
      if (ImGui.beginTabBar("main"))
      {
         if (ImGui.beginTabItem("Window"))
         {
            ImGui.text("Tab bar detected!");
            ImGui.endTabItem();
         }
         ImGui.endTabBar();
      }
      ImGui.text(StringTools.format3D("Time: {} s", stopwatch.totalElapsed()).get());
      ImGui.button("I'm a Button!");
      float[] values = new float[100];
      for (int i = 0; i < 100; i++)
      {
         values[i] = i;
      }
      ImGui.plotLines("Histogram", values, 100);
      renderPlot.calculate(renderCount++);

      logWidget.renderImGuiWidgets();
   }

   private void renderWindow2()
   {
   }

   private void renderWindow3()
   {
   }

   public static void main(String[] args)
   {
      new RDXImGuiBasedUI2DDemo();
   }
}
