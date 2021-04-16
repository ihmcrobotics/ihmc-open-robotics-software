package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.flag.ImGuiDir;
import imgui.internal.ImGui;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.tools.string.StringTools;

public class GDXImGuiVRDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "VR Demo");

   private final Stopwatch stopwatch = new Stopwatch().start();

   public GDXImGuiVRDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));
            baseUI.getSceneManager().addModelInstance(new BoxesDemoModel().newInstance());

            baseUI.getImGuiDockingSetup().splitAdd("Window", ImGuiDir.Right, 0.20);
         }

         @Override
         public void render()
         {
            baseUI.pollVREvents();

            baseUI.renderBeforeOnScreenUI();

            ImGui.begin("Window");
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
            ImGui.end();

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      }, getClass());
   }

   public static void main(String[] args)
   {
      System.setProperty("enable.vr", "true");
      new GDXImGuiVRDemo();
   }
}
