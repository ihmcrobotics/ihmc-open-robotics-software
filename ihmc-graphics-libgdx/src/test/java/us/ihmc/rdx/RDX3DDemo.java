package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDX3DDemo
{
   public RDX3DDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());
            ModelInstance hollowCylinder = RDXModelBuilder.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addHollowCylinder(0.1, 0.5, 0.4, new Point3D(), Color.YELLOW);
            }, "hollow");
            hollowCylinder.transform.translate(0.2f, 0.3f, 0.5f);
            baseUI.getPrimaryScene().addModelInstance(hollowCylinder);

            ModelInstance symmetricTriangularPrism = RDXModelBuilder.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addIsoscelesTriangularPrism(0.3, 0.15, 0.05, new Point3D(), Color.ORANGE);
            }, "wedge");
            baseUI.getPrimaryScene().addModelInstance(symmetricTriangularPrism);
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

   public static void main(String[] args)
   {
      new RDX3DDemo();
   }
}