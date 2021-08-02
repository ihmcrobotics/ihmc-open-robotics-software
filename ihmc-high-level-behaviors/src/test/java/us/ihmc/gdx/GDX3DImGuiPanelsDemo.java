package us.ihmc.gdx;

import imgui.ImGui;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXApplicationCreator;

public class GDX3DImGuiPanelsDemo
{
   public GDX3DImGuiPanelsDemo()
   {
      GDX3DSceneManager sceneManager = new GDX3DSceneManager();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());

            new GDXImGuiWindow("Test Window")
            {
               @Override
               public void renderImGuiWidgets()
               {
                  ImGui.text("this is a test");
                  ImGui.button("this is a button");
               }
            };

            sceneManager.addRenderableProvider(GDXImGuiVirtualWindowManager.getInstance());
         }

         @Override
         public void render()
         {
            GDX3DSceneTools.glClearGray();
            sceneManager.setViewportBoundsToWindow();

            GDXImGuiVirtualWindowManager.getInstance().update();

            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDX3DImGuiPanelsDemo();
   }
}
