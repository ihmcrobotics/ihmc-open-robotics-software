package us.ihmc.rdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;

public class RDX3DSituatedImGuiPanelsDemo
{
   private final RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
   private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
   private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();
   private RDX3DSituatedImGuiPanel situatedImGuiPanel;

   public RDX3DSituatedImGuiPanelsDemo()
   {
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            imGuiGlfw.init(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle(), true);
            imGuiGl3.init();

            ImGuiTools.setupFonts(ImGui.getIO());

            situatedImGuiPanel = new RDX3DSituatedImGuiPanel("Test Panel", () ->
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

            RDX3DSceneTools.glClearGray();
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
      }, "RDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new RDX3DSituatedImGuiPanelsDemo();
   }
}
