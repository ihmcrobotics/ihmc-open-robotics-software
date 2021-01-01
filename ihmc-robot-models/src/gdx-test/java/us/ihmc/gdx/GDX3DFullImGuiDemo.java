package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.internal.ImGui;
import imgui.flag.*;
import imgui.type.ImInt;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.GDXImGuiWindowAndDockSystem;
import us.ihmc.gdx.imgui.ImGuiGDX3DWindow;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.tools.string.StringTools;

public class GDX3DFullImGuiDemo
{
   private final GDX3DApplication application = new GDX3DApplication();
   private final GDXImGuiWindowAndDockSystem imGui = new GDXImGuiWindowAndDockSystem();
   private final ImGuiGDX3DWindow gdx3DWindow = new ImGuiGDX3DWindow();

   private final Stopwatch stopwatch = new Stopwatch().start();

   public GDX3DFullImGuiDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            application.create();

            application.addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));
            application.addModelInstance(new BoxesDemoModel().newInstance());

            imGui.create();
         }

         @Override
         public void render()
         {
            application.glClearGrayscale(0.3f);
            imGui.beforeWindowManagement();
            gdx3DWindow.renderBeforeOtherWindows(application);

            {
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
               application.getCamera3D().addInputExclusionBox(ImGuiTools.windowBoundingBox());
               ImGui.end();
            }

            if (imGui.isFirstRenderCall())
            {
               ImGui.dockBuilderSetNodeSize(imGui.getCentralDockspaceId(), application.getCurrentWindowWidth(), application.getCurrentWindowHeight());
               ImInt outIdAtOppositeDir = new ImInt();
               int dockRightId = ImGui.dockBuilderSplitNode(imGui.getCentralDockspaceId(), ImGuiDir.Right, 0.20f, null, outIdAtOppositeDir);
               ImGui.dockBuilderDockWindow(gdx3DWindow.getWindowName(), imGui.getCentralDockspaceId());
               ImGui.dockBuilderDockWindow("Window", dockRightId);
               ImGui.dockBuilderFinish(imGui.getCentralDockspaceId());
            }

            imGui.afterWindowManagement();
            application.render();
            imGui.afterGDXRender();
         }

         @Override
         public void dispose()
         {
            super.dispose();
            imGui.dispose();
         }
      }, getClass());
   }

   public static void main(String[] args)
   {
      new GDX3DFullImGuiDemo();
   }
}
