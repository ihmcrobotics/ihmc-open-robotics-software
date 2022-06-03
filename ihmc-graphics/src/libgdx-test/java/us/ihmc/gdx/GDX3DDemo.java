package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelBuilder;

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
            ModelInstance hollowCylinder = GDXModelBuilder.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addHollowCylinder(0.1, 0.5, 0.4, new Point3D(), Color.YELLOW);
            }, "hollow");
            hollowCylinder.transform.translate(0.2f, 0.3f, 0.5f);
            sceneManager.addModelInstance(hollowCylinder);

            ModelInstance symmetricTriangularPrism = GDXModelBuilder.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addIsoscelesTriangularPrism(0.3, 0.15, 0.05, new Point3D(), Color.ORANGE);
            }, "wedge");
            sceneManager.addModelInstance(symmetricTriangularPrism);
         }

         @Override
         public void render()
         {
            sceneManager.setViewportBoundsToWindow();
            sceneManager.renderShadowMap();
            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDX3DDemo();
   }
}