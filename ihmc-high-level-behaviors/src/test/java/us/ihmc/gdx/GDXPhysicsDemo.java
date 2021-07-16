package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

public class GDXPhysicsDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   public GDXPhysicsDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.get3DSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));

            baseUI.getImGuiPanelManager().addPanel("Simulation", GDXPhysicsDemo.this::renderWindow1);
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

   }

   public static void main(String[] args)
   {
      new GDXPhysicsDemo();
   }
}
