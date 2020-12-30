package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.*;
import imgui.internal.ImGui;
import imgui.flag.*;
import imgui.type.ImInt;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.tools.string.StringTools;

public class GDX3DFullImGuiDemo
{
   private boolean isInitialized = false;

   public GDX3DFullImGuiDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new PrivateGDXApplication(), getClass());
   }

   class PrivateGDXApplication extends GDX3DApplication
   {
      private final GDXImGuiWindowAndDockSystem imGui = new GDXImGuiWindowAndDockSystem();

      private final Stopwatch stopwatch = new Stopwatch().start();

      @Override
      public void create()
      {
         super.create();

         addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));
         addModelInstance(new BoxesDemoModel().newInstance());

         imGui.create();
      }

      @Override
      public void render()
      {
         glClearGrayscale(0.3f);

         imGui.beforeWindowManagement();

         // TODO: Pass inputs through ImGui?
         int flags = ImGuiWindowFlags.None;
//         flags += ImGuiWindowFlags.NoNavInputs;
//         flags += ImGuiWindowFlags.NoMouseInputs;
         ImGui.begin("3D View", flags);

         float posX = ImGui.getWindowPosX();
         float posY = ImGui.getWindowPosY();
         float sizeX = ImGui.getWindowSizeX();
         float sizeY = ImGui.getWindowSizeY();
         setViewportBounds((int) posX, getCurrentWindowHeight() - (int) posY - (int) sizeY, (int) sizeX, (int) sizeY);

         ImGui.getWindowDrawList().addRectFilled(posX, posY, posX + sizeX, posY + sizeY, ImColor.floatToColor(CLEAR_COLOR, CLEAR_COLOR, CLEAR_COLOR, 1.0f));

         ImGui.end();

         getCamera3D().clearInputExclusionBoxes();

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

         getCamera3D().addInputExclusionBox(ImGuiTools.windowBoundingBox());

         ImGui.end();

         if (!isInitialized)
         {
            ImGui.dockBuilderSetNodeSize(imGui.getCentralDockspaceId(), getCurrentWindowWidth(), getCurrentWindowHeight());
            ImInt outIdAtOppositeDir = new ImInt();
            int dockRightId = ImGui.dockBuilderSplitNode(imGui.getCentralDockspaceId(), ImGuiDir.Right, 0.20f, null, outIdAtOppositeDir);
            ImGui.dockBuilderDockWindow("3D View", imGui.getCentralDockspaceId());
            ImGui.dockBuilderDockWindow("Window", dockRightId);
            ImGui.dockBuilderFinish(imGui.getCentralDockspaceId());
         }

         imGui.afterWindowManagement();

         super.render();

         imGui.afterGDXRender();

         isInitialized = true;
      }

      @Override
      public void dispose()
      {
         super.dispose();
         imGui.dispose();
      }
   }

   public static void main(String[] args)
   {
      new GDX3DFullImGuiDemo();
   }
}
