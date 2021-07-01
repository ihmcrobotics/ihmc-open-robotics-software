package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;

public class GDX3DDemo
{
   public GDX3DDemo()
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
            sceneManager.addModelInstance(GDXModelPrimitives.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addHollowCylinder(0.1, 0.5, 0.4, new Point3D(), Color.YELLOW);
            }, "hollow"));
         }

         @Override
         public void render()
         {
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDX3DDemo();
   }
}