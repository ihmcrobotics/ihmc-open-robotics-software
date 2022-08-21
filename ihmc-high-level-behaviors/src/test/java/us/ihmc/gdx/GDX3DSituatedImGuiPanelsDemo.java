package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.GDX3DSituatedImGuiPanel;
import us.ihmc.gdx.imgui.GDX3DSituatedImGuiPanelManager;
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
   private final GDX3DSituatedImGuiPanelManager situatedImGuiPanelManager = new GDX3DSituatedImGuiPanelManager();

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

            situatedImGuiPanelManager.create(imGuiGl3);
            GDX3DSituatedImGuiPanel panel = new GDX3DSituatedImGuiPanel("Test Panel", () ->
            {
               ImGui.text("This is a 3D situated panel.");
               ImGui.button("Button");
            });
            situatedImGuiPanelManager.addPanel(panel);
            sceneManager.addRenderableProvider(situatedImGuiPanelManager);

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());
         }

         @Override
         public void render()
         {
            situatedImGuiPanelManager.render();

            GDX3DSceneTools.glClearGray();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }

         @Override
         public void dispose()
         {
            sceneManager.dispose();
            situatedImGuiPanelManager.dispose();
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
