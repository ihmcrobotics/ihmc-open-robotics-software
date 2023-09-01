package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.RDXModelBuilder;

public class RDX3DDemo
{
   public RDX3DDemo()
   {
      RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());
            ModelInstance hollowCylinder = RDXModelBuilder.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addHollowCylinder(0.1, 0.5, 0.4, new Point3D(), Color.YELLOW);
            }, "hollow");
            hollowCylinder.transform.translate(0.2f, 0.3f, 0.5f);
            sceneManager.addModelInstance(hollowCylinder);

            ModelInstance symmetricTriangularPrism = RDXModelBuilder.buildModelInstance(meshBuilder ->
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
      }, "RDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new RDX3DDemo();
   }
}