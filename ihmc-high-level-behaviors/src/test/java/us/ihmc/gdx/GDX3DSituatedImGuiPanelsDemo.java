package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.GDX3DSituatedImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDX3DBareBonesScene;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXApplicationCreator;

public class GDX3DSituatedImGuiPanelsDemo
{
   private final GDX3DBareBonesScene sceneManager = new GDX3DBareBonesScene();
   private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
   private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();
   private GDX3DSituatedImGuiPanel situatedImGuiPanel;

   public GDX3DSituatedImGuiPanelsDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            imGuiGlfw.init(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle(), true);
            imGuiGl3.init();

            ImGuiTools.setupFonts(ImGui.getIO());

            situatedImGuiPanel = new GDX3DSituatedImGuiPanel("Test Panel", () ->
            {
               ImGui.text("This is a 3D situated panel.");
               ImGui.button("Button");
            });
            situatedImGuiPanel.create(imGuiGl3, 0.3, 0.5, 10);
            sceneManager.addRenderableProvider(situatedImGuiPanel::getRenderables);

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());
         }

         @Override
         public void render()
         {
            situatedImGuiPanel.update();

            GDX3DSceneTools.glClearGray();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }

         @Override
         public void dispose()
         {
            sceneManager.dispose();
            situatedImGuiPanel.dispose();
            imGuiGl3.dispose();
            imGuiGlfw.dispose();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDX3DSituatedImGuiPanelsDemo();
   }
}
