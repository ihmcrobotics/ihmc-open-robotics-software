package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.internal.ImGui;
import imgui.flag.*;
import imgui.type.ImInt;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.tools.string.StringTools;

public class GDX3DFullImGuiDemo
{
   private final GDX3DApplication application = new GDX3DApplication();
   private final GDXImGuiWindowAndDockSystem imGui = new GDXImGuiWindowAndDockSystem();
   private final GDXImGui3DView gdx3DWindow = new GDXImGui3DView();

   private boolean isInitialized = false;
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

            gdx3DWindow.render(application);

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

            if (!isInitialized)
            {
               ImGui.dockBuilderSetNodeSize(imGui.getCentralDockspaceId(), application.getCurrentWindowWidth(), application.getCurrentWindowHeight());
               ImInt outIdAtOppositeDir = new ImInt();
               int dockRightId = ImGui.dockBuilderSplitNode(imGui.getCentralDockspaceId(), ImGuiDir.Right, 0.20f, null, outIdAtOppositeDir);
               ImGui.dockBuilderDockWindow("3D View", imGui.getCentralDockspaceId());
               ImGui.dockBuilderDockWindow("Window", dockRightId);
               ImGui.dockBuilderFinish(imGui.getCentralDockspaceId());
            }

            imGui.afterWindowManagement();

            application.render();

            imGui.afterGDXRender();

            isInitialized = true;
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
