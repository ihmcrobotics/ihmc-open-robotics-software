package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.internal.ImGui;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.tools.string.StringTools;

public class GDXImGuiBasedUIDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "Demo");

   private final Stopwatch stopwatch = new Stopwatch().start();
   private final ImGuiMovingPlot renderPlot = new ImGuiMovingPlot("render count", 1000, 300, 30);
   private long renderCount = 0;

   public GDXImGuiBasedUIDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.get3DSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));
            baseUI.get3DSceneManager().addModelInstance(new BoxesDemoModel().newInstance());

            baseUI.getImGuiPanelManager().addPanel("Window 1", GDXImGuiBasedUIDemo.this::renderWindow1);
            baseUI.getImGuiPanelManager().addPanel("Window 2", GDXImGuiBasedUIDemo.this::renderWindow2);
            baseUI.getImGuiPanelManager().addPanel("Window 3", GDXImGuiBasedUIDemo.this::renderWindow3);
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
   }

   private void renderWindow2()
   {
   }

   private void renderWindow3()
   {
   }

   public static void main(String[] args)
   {
      new GDXImGuiBasedUIDemo();
   }
}
